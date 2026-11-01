-- Heading hold on the four RCS pullers.
--
-- The RCS only ever pulls (positive RPM), in diagonal pairs:
--   right turn (u > 0): bow starboard + stern port
--   left turn  (u < 0): bow port + stern starboard
-- A turn changes two controllers and none ever reverses. Every controller
-- write costs flicker on every controller sharing its input shaft (see
-- ap2/actuators.lua), so the RCS is kept to as few writes as possible.
--
-- The turning authority is learned in flight, sign included. Every 0.5 s the
-- measured yaw acceleration is fitted to
--     yaw_accel = b * u - d * yaw_rate
-- (u = turn force actually applied, -1..1). Commands are computed as an
-- acceleration and divided by b, so if the pairs turn the hull the other way
-- (b < 0) the other pair is used, and a weak or strong RCS gets the right
-- gain. Until the direction has been confirmed by a few samples, RCS output is
-- held to probe_limit and half the turn rate.
--
-- Bearing error sets a yaw-rate command (proportional, capped by max_rate and
-- by the rate the learned authority can still stop); a PI loop on yaw rate
-- gives the yaw acceleration. The integral holds off steady torques such as a
-- rear thrust line that misses the centre of mass; above high_speed it works
-- harder because those torques grow with thrust and airspeed.
local util = require("ap2.util")
local rls = require("ap2.rls")

local M = {}
M.__index = M

-- saved: { yaw_b = } learned on an earlier flight, or nil.
function M.new(cfg, saved)
  local c = cfg.heading
  local guess = c.yaw_authority
  local yaw = {
    -- Wide starting variance: the real authority may be many times the guess.
    est = rls.new({ guess, 0.1 }, { math.max(100, 100 * guess * guess), 0.25 }, 0.95),
    ok = false,   -- direction confirmed
    count = 0,    -- informative samples since the last direction change
  }
  if saved and util.finite(saved.yaw_b) and math.abs(saved.yaw_b) >= c.min_authority then
    yaw.est.t[1], yaw.ok = saved.yaw_b, true
  end
  return setmetatable({ c = c, yaw = yaw, integ = 0, err = 0, r_cmd = 0, win = nil, active = true }, M)
end

function M:reset()
  self.integ = 0
end

-- Learn the turning authority from 0.5 s windows. u is the turn force the
-- controllers are actually applying (from their current RPM).
function M:observe(s, u, now)
  local w = self.win
  if w == nil then
    self.win = { t0 = now, r0 = s.yaw_rate, u = 0, r = 0, n = 0 }
    return
  end
  w.u, w.r, w.n = w.u + u, w.r + s.yaw_rate, w.n + 1
  local span = now - w.t0
  if span < 0.5 then return end
  self.win = { t0 = now, r0 = s.yaw_rate, u = 0, r = 0, n = 0 }

  local mu, mr = w.u / w.n, w.r / w.n
  if math.abs(mu) < 0.03 and math.abs(mr) < 0.3 then return end
  local l = self.yaw
  local before = l.est.t[1]
  l.est:update(mu, -mr, (s.yaw_rate - w.r0) / span)
  if l.est.t[2] < 0 then l.est.t[2] = 0 end
  local b = l.est.t[1]
  if util.sign(b) ~= util.sign(before) then
    l.ok, l.count = false, 0
  end
  if math.abs(mu) >= 0.05 then l.count = l.count + 1 end
  if not l.ok and l.count >= 4 and math.abs(b) >= self.c.min_authority then l.ok = true end
end

-- Returns the turn force u, normalised to [-1, 1].
-- deadzone: bearing error (deg) tolerated before an active correction starts.
-- Once settled (within settle_deg and nearly still) the loop stops correcting
-- and holds a steady trim: the turning force its integral has learned is
-- needed to cancel steady torques (an off-centre thrust line). With no such
-- torque the trim is about zero and the RCS rests. That avoids pulsing the
-- thrusters on and off for errors too small to matter, and every write costs
-- flicker on every controller.
function M:command(s, bearing, dt, deadzone)
  local c = self.c
  local err = util.wrap180(bearing - s.hdg)
  local dist = math.abs(err)
  local b = self.yaw.est.t[1]
  local limit = self.yaw.ok and 1 or c.probe_limit
  local b_eff = (b >= 0 and 1 or -1) * math.max(math.abs(b), c.min_authority)

  if self.active then
    if dist < c.settle_deg and math.abs(s.yaw_rate) < c.settle_rate then self.active = false end
  elseif dist > deadzone or math.abs(s.yaw_rate) > c.engage_rate then
    self.active = true
  end
  if not self.active then
    self.err, self.r_cmd = err, 0
    return util.clamp(self.integ / b_eff, -limit, limit)
  end

  local decel = math.min(c.yaw_accel, 0.5 * math.max(math.abs(b), c.min_authority))
  local max_rate = self.yaw.ok and c.max_rate or 0.5 * c.max_rate
  local r_cmd = util.sign(err) * math.min(max_rate, c.bearing_gain * dist, math.sqrt(2 * decel * dist))
  local r_err = r_cmd - s.yaw_rate
  local int_gain = c.int_gain
  if s.gs > c.high_speed then int_gain = int_gain * c.high_speed_gain end
  local grown = util.clamp(self.integ + int_gain * r_err * dt, -c.int_limit, c.int_limit)
  -- No wind-up while the output is pinned at its limit.
  local u_try = (grown + c.rate_gain * r_err) / b_eff
  if math.abs(u_try) < limit or math.abs(grown) < math.abs(self.integ) then self.integ = grown end

  self.err, self.r_cmd = err, r_cmd
  return util.clamp((self.integ + c.rate_gain * r_err) / b_eff, -limit, limit)
end

-- Turn force currently applied, recovered from the four RPMs.
function M.applied(rpm, caps)
  local function f(id) return util.ssq(rpm[id] or 0) / (caps[id] * caps[id]) end
  return (f("bow_stbd") + f("stern_port") - f("bow_port") - f("stern_stbd")) / 2
end

-- Map the turn force to the four pullers' RPM: one diagonal pair pulls, the
-- other rests. Commands under min_rpm are sent as 0.
function M.mix(u, caps, min_rpm)
  local rpm = { bow_stbd = 0, stern_port = 0, bow_port = 0, stern_stbd = 0 }
  local a, b = "bow_stbd", "stern_port"
  if u < 0 then a, b = "bow_port", "stern_stbd" end
  local f = math.min(1, math.abs(u))
  for _, id in ipairs({ a, b }) do
    local r = math.sqrt(f) * caps[id]
    if r < min_rpm then r = 0 end
    rpm[id] = r
  end
  return rpm
end

function M:saved()
  return { yaw_b = self.yaw.ok and self.yaw.est.t[1] or nil }
end

return M
