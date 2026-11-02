-- Rotation speed controller bank.
--
-- How Create 6.0.8 destroys a rotation speed controller (RotationPropagator /
-- KineticBlockEntity / KineticNetwork, read from the installed jar):
--   * a block propagating rotation with a flicker score over 128 is destroyed;
--     the score decays 1 per tick
--   * every target change detaches the controller and re-attaches it. Every
--     block that gets its rotation THROUGH that controller drops to 0 (+5) and
--     comes back (+5): its own props and every controller downstream of it.
--     On this ship the four RCS controllers are downstream of the elevation
--     controller (but not of each other), so an elevation write costs the
--     elevation and all four RCS +10 each; an RCS write costs only itself
--   * every switch of the engine network between overstressed and not costs
--     every block on it +5 (a write near capacity can switch it twice)
-- The layout is in config (actuator.downstream).
--
-- The flight loop only records desired RPM; the writer coroutine calls
-- service(), which writes at most one controller per call (setTargetSpeed
-- runs on the server thread and stalls for a tick) under these rules:
--   * RPM clamped to the controller cap and rounded to an integer
--   * every controller's flicker score is modelled: a write charges
--     flicker_cost to the written controller and everything downstream of it,
--     plus flicker_risk to every controller when the engine is near its stress
--     limit (or there is no stress reading). No write may take any score past
--     flicker_limit. RCS and rear-thrust increases leave lift_reserve of each
--     RCS score free, so the elevation can always be written
--   * each controller is written at most every min_interval[group] seconds,
--     and changes smaller than deadband[group] are skipped
--   * the RCS is written as one batch: when its corners change, the ones
--     turning down go first, then the ones turning up, each group on
--     consecutive ticks and only started when the whole group fits the
--     budget, so the hull never runs on a stray corner (with the Eureka helm
--     resisting turns, one corner alone strafes instead of turning)
--   * the elevation goes first, then the largest weighted need
--   * stress governor: from the stressometer (and each controller's learned
--     stress per RPM), elevation gets first priority, RCS steering second, and
--     rear thrust last. Rear thrust yields to reserve room for requested RCS.
--     Normal thrust stays below stress_high; steering may use stress_rcs_high.
--     When overstressed, rear thrust and RCS are shed to 0 and elevation is
--     brought back under the limit
--   * a controller changed by something else (another computer, a player) is
--     left alone for foreign_hold seconds and reported
--   * kill() bypasses pacing so zero goes out immediately
local util = require("ap2.util")

local M = {}
M.__index = M

M.ORDER = { "lift", "thrust", "bow_port", "bow_stbd", "stern_port", "stern_stbd" }
local RCS = { "bow_port", "bow_stbd", "stern_port", "stern_stbd" }

local GROUP = {
  lift = "lift", thrust = "thrust",
  bow_port = "rcs", bow_stbd = "rcs", stern_port = "rcs", stern_stbd = "rcs",
}

local CORNER_NAME = {
  bow_port = "BOW PORT", bow_stbd = "BOW STBD", stern_port = "STRN PORT", stern_stbd = "STRN STBD",
}
M.CORNER_NAME = CORNER_NAME

-- Use the RCS layout found by an RCS check: map[corner] = peripheral name of
-- the controller whose props really sit at that corner. Changes cfg in place
-- (call before wrapping devices). Returns true if anything moved.
function M.apply_layout(cfg, map)
  if type(map) ~= "table" then return false end
  local known = {}
  for _, id in ipairs(RCS) do known[cfg.controllers[id].name] = true end
  for _, id in ipairs(RCS) do
    if type(map[id]) ~= "string" or not known[map[id]] then return false end
  end
  local moved = false
  for _, id in ipairs(RCS) do
    local c = cfg.controllers[id]
    if c.name ~= map[id] then moved = true end
    c.name = map[id]
    c.label = "RSC" .. (map[id]:match("(%d+)$") or "?") .. " " .. CORNER_NAME[id]
  end
  return moved
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
    batch = {},         -- RCS corners still to write in the running batch
    hit = {},           -- id -> list of controllers a write to id charges
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
      score = 0,      -- modelled flicker score
      score_t = 0,
      peak = 0,
      fault = nil,
      impact = nil,   -- learned stress (SU) per RPM of this controller
      held = false,   -- an increase is being held back by the stress governor
      foreign_at = nil,
      prop = nil,     -- modelled spin of the props behind it (RPM)
    }
    self.hit[id] = { id }
    for _, d in ipairs(self.cfg.downstream[id] or {}) do
      self.hit[id][#self.hit[id] + 1] = d
    end
  end
  return self
end

-- Read the current targets so the first writes slew from where the ship is.
function M:seed()
  for _, id in ipairs(M.ORDER) do
    local c = self.ch[id]
    if c.dev and c.dev.getTargetSpeed then
      local ok, v = pcall(c.dev.getTargetSpeed)
      if ok and util.finite(v) then
        c.applied = util.round(v * c.sign)
        c.prop = c.applied -- assume a running ship's props are already at speed
      end
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
  self.batch = {}
end

function M:applied(id)
  return self.ch[id].applied or 0
end

-- Advance the modelled prop spin to time now. Clockwork's propeller bearing
-- (PropellerBearingBlockEntity.tick) moves its spin half way to the target
-- each tick, at most 32 RPM per tick: 2000 RPM takes about 3.1 s.
function M:spin(now)
  local t = self.spin_t or now
  local steps = math.min(200, math.floor((now - t) / 0.05 + 1e-6))
  if steps <= 0 and self.spin_t then return end
  self.spin_t = (self.spin_t or now) + steps * 0.05
  local slew = self.cfg.prop_slew
  for _, id in ipairs(M.ORDER) do
    local c = self.ch[id]
    local target = c.applied or 0
    local p = c.prop or target
    for _ = 1, steps do
      p = p + util.clamp((target - p) / 2, -slew, slew)
    end
    c.prop = p
  end
end

-- Modelled RPM the props behind controller id are really turning at.
function M:prop(id, now)
  if now then self:spin(now) end
  return self.ch[id].prop or self.ch[id].applied or 0
end

-- A stressometer reading of the engine network. Learn stress per RPM when all
-- writes since the previous sample belong to one controller group. An RCS pair
-- is written on consecutive ticks, so requiring exactly one write leaves its
-- cost permanently at the conservative whole-network average.
function M:set_stress(su, cap, now)
  if not (util.finite(su) and util.finite(cap)) then
    self.stress, self.pred = nil, nil
    return
  end
  local prev = self.stress
  if prev and #self.since > 0 then
    local group, grow, same = nil, 0, true
    for _, w in ipairs(self.since) do
      local g = self.ch[w.id].group
      if group and g ~= group then same = false end
      group = group or g
      grow = grow + w.grow
    end
    if same and math.abs(grow) >= 20 then
      local imp = (su - prev.su) / grow
      if util.finite(imp) and imp > 0 then
        for _, w in ipairs(self.since) do
          local c = self.ch[w.id]
          c.impact = c.impact and (0.6 * c.impact + 0.4 * imp) or imp
        end
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

-- Modelled flicker score of channel c now.
function M:score(c, now)
  local s = c.score - self.cfg.flicker_decay * (now - c.score_t)
  if s < 0 then s = 0 end
  return s
end

-- Highest modelled score on any controller.
function M:flicker(now)
  local m = 0
  for _, id in ipairs(M.ORDER) do m = math.max(m, self:score(self.ch[id], now)) end
  return m
end

-- Near the stress limit, or blind to it, a write can also switch overstress
-- for the whole network.
function M:risky(now)
  if not fresh(self, now) then return true end
  return (self.pred or self.stress.su) >= self.cfg.stress_quiet * self.stress.cap
end

-- Would writing ids (in order) keep every score within the limit? reserve:
-- keep lift_reserve of each RCS score free for the elevation.
function M:fits(ids, now, reserve)
  local sc = {}
  for _, id in ipairs(M.ORDER) do sc[id] = self:score(self.ch[id], now) end
  local risk = self:risky(now) and self.cfg.flicker_risk or 0
  for _, id in ipairs(ids) do
    for _, d in ipairs(self.hit[id]) do sc[d] = sc[d] + self.cfg.flicker_cost end
    if risk > 0 then
      for d in pairs(sc) do sc[d] = sc[d] + risk end
    end
  end
  for d, v in pairs(sc) do
    local limit = self.cfg.flicker_limit
    if reserve and GROUP[d] == "rcs" then limit = limit - self.cfg.lift_reserve end
    if v > limit then return false end
  end
  return true
end

local function charge(self, id, now)
  local risk = self:risky(now) and self.cfg.flicker_risk or 0
  local function add(c, n)
    c.score = self:score(c, now) + n
    c.score_t = now
    if c.score > c.peak then c.peak = c.score end
  end
  for _, d in ipairs(self.hit[id]) do add(self.ch[d], self.cfg.flicker_cost) end
  if risk > 0 then
    for _, d in ipairs(M.ORDER) do add(self.ch[d], risk) end
  end
end

-- Stress-govern all four RCS corners as one force vector. Independent limits
-- can leave one corner of a diagonal pair stronger than the other, turning a
-- requested rotation into a strafe. Returns id -> target, or nil when there
-- is not enough telemetry to coordinate the group.
function M:rcs_governed(now)
  if not fresh(self, now) then return nil end
  local st = self.stress
  local target = {}
  if st.su >= st.cap then
    for _, id in ipairs(RCS) do target[id] = 0 end
    return target
  end
  local base = self.pred or st.su
  local demand = 0
  for _, id in ipairs(RCS) do
    local c = self.ch[id]
    local imp = self:impact(c)
    if not (imp and imp > 0 and c.applied ~= nil) then return nil end
    base = base - imp * math.abs(c.applied)
    demand = demand + imp * math.abs(c.desired or 0)
  end
  local lift = self.ch.lift
  local lift_imp = self:impact(lift)
  local lift_growth = 0
  if lift.desired and lift.applied and lift_imp then
    lift_growth = lift_imp * math.max(0, math.abs(lift.desired) - math.abs(lift.applied))
    base = base + lift_growth
  end
  -- If lift needs more power, RCS first sheds enough for lift to reach the
  -- normal 85% ceiling. Once lift is steady, steering may use its 95% ceiling.
  local share = lift_growth > 0 and self.cfg.stress_high
    or (self.cfg.stress_rcs_high or self.cfg.stress_high)
  local room = math.max(0, share * st.cap - base)
  local scale = demand > 0 and math.min(1, room / demand) or 1
  for _, id in ipairs(RCS) do
    local c = self.ch[id]
    target[id] = util.round((c.desired or 0) * scale)
    c.held = math.abs(target[id]) < math.abs(c.desired or 0)
  end
  return target
end

-- Target for channel c after the stress governor. Priority is elevation, RCS,
-- then rear thrust. Steering gets extra headroom because losing the bearing at
-- cruise speed is less safe than slowing the acceleration.
function M:governed(c, now)
  local want = c.desired
  local have = c.applied
  c.held = false
  if not fresh(self, now) then return want end
  if c.group == "rcs" then
    local rcs = self:rcs_governed(now)
    if rcs then return rcs[c.id] end
  end
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
  if c.group == "thrust" then
    -- Reserve the feasible, coordinated RCS target before rear thrust uses the
    -- rest. This makes speed yield to steering without starving the lift.
    local rcs = self:rcs_governed(now)
    for _, id in ipairs(RCS) do
      local r = self.ch[id]
      local r_imp = self:impact(r)
      local target = rcs and rcs[id] or r.desired
      if target and r.applied and r_imp then
        local need = r_imp * (math.abs(target) - math.abs(r.applied))
        if need > 0 then
          room = room - need
          squeeze = true
        end
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

-- Next value to write for channel c (ignoring the budget), or nil.
-- in_batch: part of a running RCS batch, so no interval or deadband.
function M:next_value(c, now, in_batch)
  if c.desired == nil or c.dev == nil then return nil end
  local have = c.applied
  if self.urgent and c.desired == 0 and have ~= 0 then return 0 end
  if self.locked then return nil end
  if c.foreign_at and now - c.foreign_at < self.cfg.foreign_hold then return nil end
  local want = self:governed(c, now)
  if have == want then return nil end
  if not in_batch and now - c.last_write < self.cfg.min_interval[c.group] then return nil end
  if have == nil then
    -- Unknown controller state (getTargetSpeed failed): write the target as is.
    return want
  end
  local delta = want - have
  -- Deadband only between two running speeds: starting or stopping is always
  -- written. DEBUG (self.exact) writes exactly what it asks for.
  if not in_batch and not self.exact and want ~= 0 and have ~= 0
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

local function write(self, c, v, now)
  if not (self.urgent and v == 0) and foreign(self, c, now) then return nil end
  local grow = math.abs(v) - math.abs(c.applied or 0)
  charge(self, c.id, now)
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

-- The RCS group to write now, or nil: the corners turning down (or, if none,
-- the corners turning up), started only when at least one corner is due
-- (interval and deadband) and the whole group fits the budget.
function M:rcs_group(now)
  local due = false
  local down, up = {}, {}
  for _, id in ipairs(RCS) do
    local c = self.ch[id]
    if self:next_value(c, now) ~= nil then due = true end
    local v = self:next_value(c, now, true)
    if v ~= nil then
      local list = math.abs(v) < math.abs(c.applied or 0) and down or up
      list[#list + 1] = id
    end
  end
  if not due then return nil end
  local group = #down > 0 and down or up
  if #group == 0 then return nil end
  if not self:fits(group, now, group == up) then return nil end
  return group
end

-- Write at most one controller. Returns the channel id written, or nil.
-- setTargetSpeed runs on the server thread, so in game this call yields.
function M:service(now)
  self:spin(now)
  -- Emergency zeros first.
  if self.urgent then
    for _, id in ipairs(M.ORDER) do
      local c = self.ch[id]
      if c.dev and c.applied ~= 0 and c.desired == 0 then return write(self, c, 0, now) end
    end
    self.urgent = false
  end

  -- Check one controller per pass against what is on the block, so a change
  -- made by something else is seen even when we have nothing to write.
  self.verify_i = (self.verify_i or 0) % #M.ORDER + 1
  local vc = self.ch[M.ORDER[self.verify_i]]
  if vc.dev then foreign(self, vc, now) end

  -- A running RCS batch goes on, one corner per tick.
  while #self.batch > 0 do
    local c = self.ch[table.remove(self.batch, 1)]
    local v = self:next_value(c, now, true)
    if v ~= nil then return write(self, c, v, now) end
  end

  -- Otherwise the elevation, then the largest weighted need.
  local best, best_v, best_need, best_group = nil, nil, -1, nil
  for _, id in ipairs({ "lift", "thrust" }) do
    local c = self.ch[id]
    local v = self:next_value(c, now)
    if v ~= nil then
      local sheds = math.abs(v) < math.abs(c.applied or 0)
      if self:fits({ id }, now, id ~= "lift" and not sheds) then
        local need = math.abs((c.desired or 0) - (c.applied or 0)) / c.cap * self.cfg.weight[c.group]
        if id == "lift" then need = need + 1000 end
        if need > best_need then best, best_v, best_need = c, v, need end
      end
    end
  end
  local group = self:rcs_group(now)
  if group then
    local need = 0
    for _, id in ipairs(group) do
      local c = self.ch[id]
      need = math.max(need, math.abs((c.desired or 0) - (c.applied or 0)) / c.cap * self.cfg.weight.rcs)
    end
    if need > best_need then
      best_group = group
      best = nil
    end
  end

  if best_group then
    self.batch = {}
    for i = 2, #best_group do self.batch[#self.batch + 1] = best_group[i] end
    local c = self.ch[best_group[1]]
    return write(self, c, self:next_value(c, now, true), now)
  end
  if best then return write(self, best, best_v, now) end
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
      score = self:score(c, now),
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
