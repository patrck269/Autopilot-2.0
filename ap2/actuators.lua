-- Rotation speed controller bank.
--
-- Every setTargetSpeed rebuilds the Create kinetic network behind the
-- controller and charges it flicker points; Create destroys the block when the
-- score passes 128 (it decays one point per tick). setTargetSpeed also runs on
-- the server thread and stalls the calling coroutine for a tick, so the flight
-- loop only records desired RPM and a separate writer coroutine calls
-- service(), which writes at most one controller per call under these rules:
--   * RPM is clamped to the controller cap and rounded to an integer
--   * one controller is written at most every min_interval seconds
--   * a modelled flicker score must stay under flicker_limit
--   * changes smaller than the deadband are skipped, larger ones are slewed
--   * a sign reversal always writes 0 first
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
  local self = setmetatable({ cfg = cfg.actuator, ch = {}, urgent = false }, M)
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

local function decayed(self, c, now)
  local s = c.score - self.cfg.flicker_decay * (now - c.score_t)
  if s < 0 then s = 0 end
  return s
end

-- Next value to write for channel c, or nil when nothing should be written now.
function M:next_value(c, now)
  local want = c.desired
  if want == nil or c.dev == nil then return nil end
  local have = c.applied
  if have == want then return nil end
  if self.urgent and want == 0 then return 0 end
  if now - c.last_write < self.cfg.min_interval then return nil end
  if decayed(self, c, now) + self.cfg.flicker_cost > self.cfg.flicker_limit then return nil end
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

-- Write at most one controller. Returns the channel id written, or nil.
-- setTargetSpeed runs on the server thread, so in game this call yields.
function M:service(now)
  for _, id in ipairs(M.ORDER) do
    local c = self.ch[id]
    local v = self:next_value(c, now)
    if v ~= nil then
      c.score = decayed(self, c, now) + self.cfg.flicker_cost
      c.score_t = now
      c.last_write = now
      c.writes = c.writes + 1
      local ok, err = pcall(c.dev.setTargetSpeed, v * c.sign)
      if ok then
        c.applied = v
        c.fault = nil
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
    }
  end
  return out
end

return M
