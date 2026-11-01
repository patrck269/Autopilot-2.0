-- Rotation speed controller bank.
--
-- How Create 6.0.8 destroys a rotation speed controller (RotationPropagator /
-- KineticBlockEntity / KineticNetwork, read from the installed jar):
--   * every target change detaches the controller and re-attaches it: its
--     output props drop to 0 and come back, +5 flicker each way
--   * every block on the engine network gets +5 each time the network
--     switches between overstressed and not; during a write the network is
--     re-synced with the controller's props removed and again with them back,
--     so on a network near or over capacity ONE write costs EVERY controller
--     on it up to +10
--   * flicker decays 1 per tick; a block propagating with a score over 128 is
--     destroyed (speed over the server's maxRotationSpeed also destroys)
-- All six controllers hang off one engine network, so overstress switching
-- adds up across controllers. While overstressed nothing turns, the ship
-- sinks, the autopilot asks for more, and every write makes it worse.
--
-- So the flight loop only records desired RPM; the writer coroutine calls
-- service(), which writes at most one controller per call (setTargetSpeed
-- runs on the server thread and stalls for a tick) under these rules:
--   * RPM clamped to the controller cap and rounded to an integer
--   * one controller is written at most every min_interval seconds
--   * per-controller flicker is modelled at the worst case (flicker_cost per
--     write) and kept under flicker_limit
--   * stress governor: from the stressometer (and each controller's learned
--     stress per RPM) an increase may not take predicted stress past
--     stress_high of capacity; when the engine is overstressed, rear thrust
--     and RCS are shed to 0 and the elevation is brought back under the limit
--   * near the limit (or with no stress reading) every write also draws on a
--     shared network budget, because there it can flip overstress for all
--   * changes smaller than the deadband are skipped, larger ones are slewed
--   * a sign reversal always writes 0 first
--   * a controller changed by something else (another computer, a player) is
--     left alone for foreign_hold seconds and reported
--   * kill() bypasses pacing so zero goes out immediately
local util = require("ap2.util")

local M = {}
M.__index = M

-- Write order is also priority order.
M.ORDER = { "lift", "thrust", "bow_port", "bow_stbd", "stern_port", "stern_stbd" }

local GROUP = {
  lift = "lift", thrust = "thrust",
  bow_port = "rcs", bow_stbd = "rcs", stern_port = "rcs", stern_stbd = "rcs",
}

-- devices: map channel -> wrapped controller (nil when missing)
function M.new(cfg, devices)
  local self = setmetatable({
    cfg = cfg.actuator,
    ch = {},
    urgent = false,
    locked = false,     -- another flight computer drives this ship
    stress = nil,       -- last stressometer reading { su, cap, t }
    pred = nil,         -- stress predicted after writes since that reading
    since = {},         -- writes since that reading: { id, grow }
    net_score = 0,
    net_t = 0,
  }, M)
  for _, id in ipairs(M.ORDER) do
    local c = cfg.controllers[id]
    self.ch[id] = {
      id = id,
      name = c.name,
      label = c.label,
      cap = c.cap,
      sign = c.sign or 1,
      group = GROUP[id],
      dev = devices[id],
      applied = nil,  -- last RPM known to be on the controller
      desired = nil,  -- nil = leave the controller alone
      last_write = -math.huge,
      score = 0,
      score_t = 0,
      writes = 0,
      fault = nil,
      impact = nil,   -- learned stress (SU) per RPM of this controller
      held = false,   -- an increase is being held back by the stress governor
      foreign_at = nil,
    }
  end
  return self
end

-- Read the current targets so the first writes slew from where the ship is.
function M:seed()
  for _, id in ipairs(M.ORDER) do
    local c = self.ch[id]
    if c.dev and c.dev.getTargetSpeed then
      local ok, v = pcall(c.dev.getTargetSpeed)
      if ok and util.finite(v) then c.applied = util.round(v * c.sign) end
    end
  end
end

function M:set_device(id, dev)
  local c = self.ch[id]
  c.dev = dev
  if dev == nil then c.applied = nil end
end

function M:set(id, rpm)
  local c = self.ch[id]
  if rpm == nil then
    c.desired = nil
    return
  end
  if not util.finite(rpm) then rpm = 0 end
  c.desired = util.clamp(util.round(rpm), -c.cap, c.cap)
end

-- Emergency: every controller to zero, ignoring pacing.
function M:kill()
  for _, id in ipairs(M.ORDER) do self.ch[id].desired = 0 end
  self.urgent = true
end

function M:applied(id)
  return self.ch[id].applied or 0
end

-- A stressometer reading of the engine network (stress and capacity in SU).
-- With exactly one write since the last reading, the change in stress is that
-- controller's: learn its stress per RPM from it.
function M:set_stress(su, cap, now)
  if not (util.finite(su) and util.finite(cap)) then
    self.stress, self.pred = nil, nil
    return
  end
  local prev = self.stress
  if prev and #self.since == 1 then
    local w = self.since[1]
    if math.abs(w.grow) >= 20 then
      local imp = (su - prev.su) / w.grow
      if util.finite(imp) and imp > 0 then
        local c = self.ch[w.id]
        c.impact = c.impact and (0.6 * c.impact + 0.4 * imp) or imp
      end
    end
  end
  self.stress = { su = su, cap = cap, t = now }
  self.pred = su
  self.since = {}
end

local function fresh(self, now)
  local st = self.stress
  return st ~= nil and now - st.t < 2 and st.cap > 0
end

-- Measured overstress: nothing on the network turns.
function M:overstressed(now)
  return fresh(self, now) and self.stress.su >= self.stress.cap
end

-- Stress per RPM for channel c: learned, else the network average (stress
-- over the sum of applied RPM, which also charges any other load to us).
function M:impact(c)
  if c.impact then return c.impact end
  local st = self.stress
  if not st then return nil end
  local sum = 0
  for _, id in ipairs(M.ORDER) do sum = sum + math.abs(self.ch[id].applied or 0) end
  if sum >= 50 then return st.su / sum end
  local caps = 0
  for _, id in ipairs(M.ORDER) do caps = caps + self.ch[id].cap end
  return st.cap / caps
end

local function decayed(self, c, now)
  local s = c.score - self.cfg.flicker_decay * (now - c.score_t)
  if s < 0 then s = 0 end
  return s
end

local function net_decayed(self, now)
  local s = self.net_score - self.cfg.flicker_decay * (now - self.net_t)
  if s < 0 then s = 0 end
  return s
end

-- Near the stress limit, or blind to it, a write can switch overstress for
-- the whole network.
function M:risky(now)
  if not fresh(self, now) then return true end
  return (self.pred or self.stress.su) >= self.cfg.stress_quiet * self.stress.cap
end

-- Flicker charged to the written controller: its own +10, plus up to +10 when
-- the write may flip the network's overstress state.
function M:write_cost(now)
  if self:risky(now) then return self.cfg.flicker_cost_risky end
  return self.cfg.flicker_cost
end

-- Next value to write for channel c, or nil when nothing should be written now.
function M:next_value(c, now)
  local want = c.desired
  if want == nil or c.dev == nil then return nil end
  local have = c.applied
  if self.urgent and want == 0 and have ~= 0 then return 0 end
  if self.locked then return nil end
  if c.foreign_at and now - c.foreign_at < self.cfg.foreign_hold then return nil end

  -- Stress governor. The elevation comes first: rear thrust and RCS only get
  -- the room the elevation does not need, and give theirs back when it does.
  c.held = false
  if fresh(self, now) then
    local st = self.stress
    local over = st.su >= st.cap
    if over and c.group ~= "lift" then want = 0 end
    local imp = self:impact(c)
    if imp and imp > 0 and have ~= nil then
      local room = self.cfg.stress_high * st.cap - (self.pred or st.su)
      local squeeze = over
      if c.group ~= "lift" then
        local lift = self.ch.lift
        local lift_imp = self:impact(lift)
        if lift.desired and lift.applied and lift_imp then
          local need = lift_imp * (math.abs(lift.desired) - math.abs(lift.applied))
          if need > 0 then
            room = room - need
            squeeze = true
          end
        end
      end
      if room < 0 and not squeeze then room = 0 end
      local limit = math.max(0, math.abs(have) + room / imp)
      if math.abs(want) > limit then
        want = util.sign(want) * math.floor(limit)
        c.held = true
      end
    end
  end

  if have == want then return nil end
  if now - c.last_write < self.cfg.min_interval then return nil end
  if decayed(self, c, now) + self:write_cost(now) > self.cfg.flicker_limit then return nil end
  if self:risky(now) and net_decayed(self, now) + self.cfg.network_cost > self.cfg.network_limit then
    return nil
  end
  if have == nil then
    -- Unknown controller state (getTargetSpeed failed): write the target as is.
    return want
  end
  local delta = want - have
  if want ~= 0 and math.abs(delta) < self.cfg.deadband[c.group] then return nil end
  local slew = self.cfg.slew[c.group]
  local v = have + util.clamp(delta, -slew, slew)
  if have ~= 0 and v ~= 0 and util.sign(v) ~= util.sign(have) then v = 0 end
  return v
end

-- Did something other than this bank change controller c? Adopt what is on
-- the block and leave it alone for a while.
local function foreign(self, c, now)
  if c.applied == nil or not c.dev.getTargetSpeed then return false end
  local ok, cur = pcall(c.dev.getTargetSpeed)
  if not ok or not util.finite(cur) then return false end
  local on_block = util.round(cur * c.sign)
  if on_block == c.applied then return false end
  c.applied = on_block
  c.foreign_at = now
  return true
end

-- Write at most one controller. Returns the channel id written, or nil.
-- setTargetSpeed runs on the server thread, so in game this call yields.
function M:service(now)
  -- Check one controller per pass against what is on the block, so a change
  -- made by something else is seen even when we have nothing to write.
  if not self.urgent then
    self.verify_i = (self.verify_i or 0) % #M.ORDER + 1
    local c = self.ch[M.ORDER[self.verify_i]]
    if c.dev then foreign(self, c, now) end
  end
  for _, id in ipairs(M.ORDER) do
    local c = self.ch[id]
    local v = self:next_value(c, now)
    if v ~= nil and not (self.urgent and v == 0) and foreign(self, c, now) then
      v = nil
    end
    if v ~= nil then
      local have = c.applied or 0
      local grow = math.abs(v) - math.abs(have)
      if self:risky(now) then
        self.net_score = net_decayed(self, now) + self.cfg.network_cost
        self.net_t = now
      end
      c.score = decayed(self, c, now) + self:write_cost(now)
      c.score_t = now
      c.last_write = now
      c.writes = c.writes + 1
      local ok, err = pcall(c.dev.setTargetSpeed, v * c.sign)
      if ok then
        c.applied = v
        c.fault = nil
        if self.pred then
          local imp = self:impact(c)
          if imp then self.pred = self.pred + imp * grow end
        end
        self.since[#self.since + 1] = { id = id, grow = grow }
      else
        c.applied = nil
        c.fault = tostring(err)
      end
      return id
    end
  end
  if self.urgent then
    local done = true
    for _, id in ipairs(M.ORDER) do
      local c = self.ch[id]
      if c.dev and c.applied ~= 0 then done = false end
    end
    if done then self.urgent = false end
  end
  return nil
end

function M:snapshot(now)
  local out = {}
  for _, id in ipairs(M.ORDER) do
    local c = self.ch[id]
    out[id] = {
      label = c.label,
      cap = c.cap,
      des = c.desired,
      app = c.applied,
      score = decayed(self, c, now),
      writes = c.writes,
      fault = c.fault,
      present = c.dev ~= nil,
      held = c.held,
      impact = c.impact,
      foreign = c.foreign_at ~= nil and now - c.foreign_at < 30,
    }
  end
  return out
end

-- Warnings about the controllers for the displays.
function M:alerts(now)
  local list = {}
  if self.locked then list[#list + 1] = "CONTROLLERS LOCKED: ANOTHER FLIGHT COMPUTER DRIVES THIS SHIP" end
  if self:overstressed(now) then
    list[#list + 1] = "ENGINE OVERSTRESSED: SHEDDING RCS AND THRUST"
  elseif not fresh(self, now) then
    list[#list + 1] = "NO STRESS READING: WRITES SLOWED"
  end
  for _, id in ipairs(M.ORDER) do
    local c = self.ch[id]
    if c.dev == nil then list[#list + 1] = "MISSING " .. c.label end
    if c.fault then list[#list + 1] = c.label .. " WRITE FAILED" end
    if c.held then list[#list + 1] = "STRESS LIMIT: " .. c.label .. " HELD" end
    if c.foreign_at and now - c.foreign_at < 30 then
      list[#list + 1] = c.label .. " CHANGED BY ANOTHER COMPUTER"
    end
  end
  return list
end

return M
