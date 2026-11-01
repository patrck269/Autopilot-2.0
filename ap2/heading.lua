-- Heading hold and sideslip cancelling on the four RCS pullers.
--
-- The RCS only ever pulls (positive RPM); no controller reverses. Corner
-- forces for a turn force u (> 0 = right) and a sway force s (> 0 = toward
-- starboard), each corner clamped at 0:
--   bow stbd = u + s    stern port = u - s
--   bow port = -u - s   stern stbd = s - u
-- A pure turn is a diagonal pair (right: bow stbd + stern port), a pure
-- strafe a same-side pair (starboard: bow stbd + stern stbd), and any blend
-- gives exactly 2u of turn and 2s of sway. Every controller write costs
-- flicker on every controller sharing its input shaft (see ap2/actuators.lua),
-- so both loops are kept to as few writes as possible.
--
-- Both authorities are learned in flight, sign included. Every 0.5 s the
-- measured yaw and sideways accelerations are fitted to
--     yaw_accel  = b  * u - d  * yaw_rate
--     side_accel = bs * s - ds * side_speed
-- (u, s = forces actually applied). Commands are computed as accelerations
-- and divided by b / bs, so pullers that act the other way are flipped and a
-- weak or strong RCS gets the right gain. Until a direction has been
-- confirmed by a few samples, that loop is held to probe_limit.
--
-- Heading: bearing error sets a yaw-rate command; a PI loop on yaw rate gives
-- the yaw acceleration. The integral holds off steady torques such as a rear
-- thrust line that misses the centre of mass. A correction starts past the
-- deadzone and stops within settle_deg; in between the learned trim is held.
--
-- Sideslip: drift past sway_engage (m/s) starts a brake that runs until the
-- drift is under sway_settle, then the sway force rests. That cancels drift
-- the ship already had when the autopilot took over, as well as crabbing at
-- speed. Heading has first call on the RCS; sway gets what is left.
local util = require("ap2.util")
local rls = require("ap2.rls")

local M = {}
M.__index = M

local RAD = math.pi / 180

local function learner(guess, damping)
  return {
    -- Wide starting variance: the real authority may be many times the guess.
    est = rls.new({ guess, damping }, { math.max(100, 100 * guess * guess), 0.25 }, 0.95),
    ok = false,   -- direction confirmed
    count = 0,    -- informative samples since the last direction change
  }
end

-- saved: { yaw_b =, sway_b = } learned on an earlier flight, or nil.
function M.new(cfg, saved)
  local c = cfg.heading
  local yaw = learner(c.yaw_authority, 0.1)
  local sway = learner(c.sway_authority, 0.05)
  if saved and util.finite(saved.yaw_b) and math.abs(saved.yaw_b) >= c.min_authority then
    yaw.est.t[1], yaw.ok = saved.yaw_b, true
  end
  if saved and util.finite(saved.sway_b) and math.abs(saved.sway_b) >= c.min_sway_authority then
    sway.est.t[1], sway.ok = saved.sway_b, true
  end
  return setmetatable({
    c = c, yaw = yaw, sway = sway, integ = 0, err = 0, r_cmd = 0, win = nil,
    active = true, sway_active = false,
  }, M)
end

function M:reset()
  self.integ = 0
end

-- Fold one fitted sample into a learner and track direction confirmation.
local function learn(l, x, damp, y, informative, min_b)
  local before = l.est.t[1]
  l.est:update(x, damp, y)
  if l.est.t[2] < 0 then l.est.t[2] = 0 end
  local b = l.est.t[1]
  if util.sign(b) ~= util.sign(before) then
    l.ok, l.count = false, 0
  end
  if informative then l.count = l.count + 1 end
  if not l.ok and l.count >= 4 and math.abs(b) >= min_b then l.ok = true end
end

-- Learn both authorities from 0.5 s windows. u and sw are the forces the
-- controllers are actually applying (from their current RPM).
function M:observe(s, u, sw, now)
  local w = self.win
  if w == nil then
    self.win = { t0 = now, r0 = s.yaw_rate, v0 = s.v_side, u = 0, sw = 0, r = 0, v = 0, f = 0, n = 0 }
    return
  end
  w.u, w.sw, w.r, w.v, w.f, w.n = w.u + u, w.sw + sw, w.r + s.yaw_rate, w.v + s.v_side, w.f + s.v_fwd, w.n + 1
  local span = now - w.t0
  if span < 0.5 then return end
  self.win = { t0 = now, r0 = s.yaw_rate, v0 = s.v_side, u = 0, sw = 0, r = 0, v = 0, f = 0, n = 0 }

  local n, c = w.n, self.c
  local mu, msw, mr, mv, mf = w.u / n, w.sw / n, w.r / n, w.v / n, w.f / n
  if math.abs(mu) >= 0.03 or math.abs(mr) >= 0.3 then
    learn(self.yaw, mu, -mr, (s.yaw_rate - w.r0) / span, math.abs(mu) >= 0.05, c.min_authority)
  end
  if math.abs(msw) >= 0.03 or math.abs(mv) >= 0.2 then
    -- The body frame turns under the velocity: take that out of the change
    -- in side speed (d v_side/dt = a_side - yaw_rate * v_fwd).
    local a_side = (s.v_side - w.v0) / span + mr * RAD * mf
    learn(self.sway, msw, -mv, a_side, math.abs(msw) >= 0.05, c.min_sway_authority)
  end
end

local function signed_floor(b, floor)
  return (b >= 0 and 1 or -1) * math.max(math.abs(b), floor)
end

-- Returns the turn force u and the sway force sw, both normalised.
-- deadzone: bearing error (deg) tolerated before an active correction starts.
function M:command(s, bearing, dt, deadzone)
  local c = self.c
  local err = util.wrap180(bearing - s.hdg)
  local dist = math.abs(err)
  local b = self.yaw.est.t[1]
  local limit = self.yaw.ok and 1 or c.probe_limit
  local b_eff = signed_floor(b, c.min_authority)

  if self.active then
    if dist < c.settle_deg and math.abs(s.yaw_rate) < c.settle_rate then self.active = false end
  elseif dist > deadzone or math.abs(s.yaw_rate) > c.engage_rate then
    self.active = true
  end

  local u
  if not self.active then
    self.r_cmd = 0
    u = util.clamp(self.integ / b_eff, -limit, limit)
  else
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
    self.r_cmd = r_cmd
    u = util.clamp((self.integ + c.rate_gain * r_err) / b_eff, -limit, limit)
  end
  self.err = err

  -- Sideslip brake with engage/settle hysteresis.
  local drift = math.abs(s.v_side)
  if self.sway_active then
    if drift < c.sway_settle then self.sway_active = false end
  elseif drift > c.sway_engage then
    self.sway_active = true
  end
  local sw = 0
  if self.sway_active then
    local sb = signed_floor(self.sway.est.t[1], c.min_sway_authority)
    local slimit = self.sway.ok and 1 or c.probe_limit
    sw = util.clamp(-c.sway_gain * s.v_side / sb, -slimit, slimit)
    local room = 1 - math.abs(u)
    sw = util.clamp(sw, -room, room)
  end
  return u, sw
end

-- Turn and sway forces currently applied, recovered from the four RPMs.
function M.applied(rpm, caps)
  local function f(id) return util.ssq(rpm[id] or 0) / (caps[id] * caps[id]) end
  local bs, sp, bp, ss = f("bow_stbd"), f("stern_port"), f("bow_port"), f("stern_stbd")
  return (bs + sp - bp - ss) / 2, (bs + ss - bp - sp) / 2
end

-- Map turn and sway forces to the four pullers' RPM (pull only). Corners
-- under min_rpm are sent as 0.
function M.mix(u, sw, caps, min_rpm)
  local force = {
    bow_stbd = u + sw, stern_port = u - sw,
    bow_port = -u - sw, stern_stbd = sw - u,
  }
  local rpm = {}
  for id, f in pairs(force) do
    local r = math.sqrt(util.clamp(f, 0, 1)) * caps[id]
    if r < min_rpm then r = 0 end
    rpm[id] = r
  end
  return rpm
end

function M:saved()
  return {
    yaw_b = self.yaw.ok and self.yaw.est.t[1] or nil,
    sway_b = self.sway.ok and self.sway.est.t[1] or nil,
  }
end

return M
