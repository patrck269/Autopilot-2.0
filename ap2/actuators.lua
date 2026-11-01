-- Rotation speed controller bank.
--
-- How Create 6.0.8 destroys a rotation speed controller (RotationPropagator /
-- KineticBlockEntity / KineticNetwork, read from the installed jar):
--   * a block propagating rotation with a flicker score over 128 is destroyed;
--     the score decays 1 per tick
--   * every target change detaches the controller and re-attaches it. Every
--     block that gets its rotation THROUGH that controller drops to 0 (+5) and
--     comes back (+5): its own props, and also anything further along its
--     input shaft, including the next controllers in a row that share it. So
--     a write to one controller can cost every controller after it +10, and
--     the last one in the line pays for all of them
--   * every switch of the engine network between overstressed and not costs
--     every block on it +5 (a write near capacity can switch it twice)
-- The ship's layout isn't known here, so every write is assumed to cost EVERY
-- controller: one shared flicker budget for the whole ship.
--
-- The flight loop only records desired RPM; the writer coroutine calls
-- service(), which writes at most one controller per call (setTargetSpeed
-- runs on the server thread and stalls for a tick) under these rules:
--   * RPM clamped to the controller cap and rounded to an integer
--   * a shared flicker score (flicker_cost per write, flicker_cost_risky near
--     the stress limit or without a stress reading) must stay under
--     flicker_limit: about two writes a second for the whole ship
--   * each controller is written at most every min_interval[group] seconds,
--     and changes smaller than deadband[group] are skipped
--   * when several controllers want a write, the largest weighted need wins
--   * stress governor: from the stressometer (and each controller's learned
--     stress per RPM) an increase may not take predicted stress past
--     stress_high of capacity, the elevation first; when overstressed, rear
--     thrust and RCS are shed to 0 and the elevation is brought back under
--   * large changes are slewed; a sign reversal always writes 0 first
--   * a controller changed by something else (another computer, a player) is
--     left alone for foreign_hold seconds and reported
--   * kill() bypasses pacing so zero goes out immediately
local util = require("ap2.util")

local M = {}
M.__index = M

M.ORDER = { "lift", "thrust", "bow_port", "bow_stbd", "stern_port", "stern_stbd" }

local GROUP = {
  lift = "lift", thrust = "thrust",
  bow_port = "rcs", bow_stbd = "rcs", stern_port = "rcs", stern_stbd = "rcs",
}

-- RCS controllers work in diagonal pairs; a pair is written back to back so
-- the hull never pulls with half a pair.
local PARTNER = {
  bow_stbd = "stern_port", stern_port = "bow_stbd",
  bow_port = "stern_stbd", stern_stbd = "bow_port",
}

-- Should RCS corner c be written right after its diagonal partner? Yes while
-- the partner was written last (within a second) and the two still differ.
local function follows(self, c, now)
  local p = PARTNER[c.id] and self.ch[PARTNER[c.id]]
  return p ~= nil and now - p.last_write < 1 and c.last_write < p.last_write and p.applied ~= c.applied
end

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
    score = 0,          -- shared flicker score (every write charges every controller)
    score_t = 0,
    peak = 0,
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

-- Shared flicker score now.
function M:flicker(now)
  local s = self.score - self.cfg.flicker_decay * (now - self.score_t)
  if s < 0 then s = 0 end
  return s
end

-- Near the stress limit, or blind to it, a write can also switch overstress
-- for the whole network.
function M:risky(now)
  if not fresh(self, now) then return true end
  return (self.pred or self.stress.su) >= self.cfg.stress_quiet * self.stress.cap
end

function M:write_cost(now)
  if self:risky(now) then return self.cfg.flicker_cost_risky end
  return self.cfg.flicker_cost
end

-- Target for channel c after the stress governor. The elevation comes first:
-- rear thrust and RCS only get the room it does not need.
function M:governed(c, now)
  local want = c.desired
  local have = c.applied
  c.held = false
  if not fresh(self, now) then return want end
  local st = self.stress
  local over = st.su >= st.cap
  if over and c.group ~= "lift" then want = 0 end
  local imp = self:impact(c)
  if not (imp and imp > 0 and have ~= nil) then return want end
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
  return want
end

-- Next value to write for channel c (ignoring the shared budget), or nil.
function M:next_value(c, now)
  if c.desired == nil or c.dev == nil then return nil end
  local have = c.applied
  if self.urgent and c.desired == 0 and have ~= 0 then return 0 end
  if self.locked then return nil end
  if c.foreign_at and now - c.foreign_at < self.cfg.foreign_hold then return nil end
  local want = self:governed(c, now)
  if have == want then return nil end
  local follow = follows(self, c, now)
  if not follow and now - c.last_write < self.cfg.min_interval[c.group] then return nil end
  if have == nil then
    -- Unknown controller state (getTargetSpeed failed): write the target as is.
    return want
  end
  local delta = want - have
  -- Deadband only between two running speeds: starting or stopping is always
  -- written. DEBUG (self.exact) writes exactly what it asks for.
  if not follow and not self.exact and want ~= 0 and have ~= 0
    and math.abs(delta) < self.cfg.deadband[c.group] then
    return nil
  end
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

  -- Pick the controller that needs a write most: an emergency zero first,
  -- then the second corner of an RCS pair whose first corner was just
  -- written (one corner alone strafes the hull instead of turning it: the
  -- helm's stabilisation resists the turn), then the elevation (it keeps the
  -- ship up), then the largest weighted need. A pair is only started when
  -- the budget covers both writes. Rear thrust and RCS may not dip into the
  -- last lift_reserve points of the budget, except to finish a pair.
  local best, best_v, best_need = nil, nil, -1
  local cost = self:write_cost(now)
  local after = self:flicker(now) + cost
  for _, id in ipairs(M.ORDER) do
    local c = self.ch[id]
    local v = self:next_value(c, now)
    if v ~= nil then
      local urgent = self.urgent and v == 0
      local partner = PARTNER[id] and self.ch[PARTNER[id]]
      local follow = follows(self, c, now)
      local starts_pair = partner ~= nil and not follow and partner.dev ~= nil
        and partner.desired ~= nil and self:governed(partner, now) ~= partner.applied
      -- Turning a controller down sheds stress and frees room: it may use the
      -- elevation's reserve. Turning one up may not (finishing a pair may).
      local sheds = math.abs(v) < math.abs(c.applied or 0)
      local room = self.cfg.flicker_limit
      if c.group ~= "lift" and not follow and not sheds then room = room - self.cfg.lift_reserve end
      local needed = after + (starts_pair and cost or 0)
      if urgent or needed <= room then
        local need = math.abs((c.desired or 0) - (c.applied or 0)) / c.cap * self.cfg.weight[c.group]
        if follow then need = need + 2000 elseif c.group == "lift" then need = need + 1000 end
        if urgent then need = math.huge end
        if need > best_need then best, best_v, best_need = c, v, need end
      end
    end
  end

  if best and not (self.urgent and best_v == 0) and foreign(self, best, now) then
    return nil
  end
  if best then
    local c, v = best, best_v
    local grow = math.abs(v) - math.abs(c.applied or 0)
    self.score = self:flicker(now) + self:write_cost(now)
    self.score_t = now
    if self.score > self.peak then self.peak = self.score end
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
      self.since[#self.since + 1] = { id = c.id, grow = grow }
    else
      c.applied = nil
      c.fault = tostring(err)
    end
    return c.id
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
  local score = self:flicker(now)
  for _, id in ipairs(M.ORDER) do
    local c = self.ch[id]
    out[id] = {
      label = c.label,
      cap = c.cap,
      des = c.desired,
      app = c.applied,
      score = score,
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
