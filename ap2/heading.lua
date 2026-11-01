-- Heading hold and sideslip damping on the four RCS pullers.
--
-- The RCS authority is learned in flight, sign included. Every 0.5 s the
-- measured yaw acceleration is fitted to
--     yaw_accel = b * u - d * yaw_rate
-- (u = yaw force actually applied, -1..1), and the same for sideways
-- acceleration against the applied sway force. Commands are computed as an
-- acceleration and divided by b, so pullers that turn the hull the other way
-- (b < 0) are flipped automatically and a weak or strong RCS gets the right
-- gain. Until the direction has been confirmed by a few samples, RCS output is
-- held to probe_limit, so a backwards guess cannot spin the ship up.
--
-- Bearing error sets a yaw-rate command (proportional, capped by max_rate and
-- by the rate the learned authority can still stop); a PI loop on yaw rate
-- gives the yaw acceleration. The integral holds off steady torques such as a
-- rear thrust line that misses the centre of mass; above high_speed it works
-- harder because those torques grow with thrust and airspeed.
--
-- Mixing (positive RPM pulls the hull toward that thruster, negative pushes):
--   right turn  u > 0:  bow stbd +u, stern port +u, bow port -u, stern stbd -u
--   to starboard s > 0: bow stbd +s, stern stbd +s, bow port -s, stern port -s
local util = require("ap2.util")
local rls = require("ap2.rls")

local M = {}
M.__index = M

local RAD = math.pi / 180

local function learner(guess, damping, p_damping)
  return {
    -- Wide starting variance: the real authority may be many times the guess.
    est = rls.new({ guess, damping }, { math.max(100, 100 * guess * guess), p_damping }, 0.95),
    ok = false,   -- direction confirmed
    count = 0,    -- informative samples since the last direction change
  }
end

-- saved: { yaw_b =, sway_b = } learned on an earlier flight, or nil.
function M.new(cfg, saved)
  local c = cfg.heading
  local yaw = learner(c.yaw_authority, 0.1, 0.25)
  local sway = learner(c.sway_authority, 0.2, 0.25)
  if saved and util.finite(saved.yaw_b) and math.abs(saved.yaw_b) >= c.min_authority then
    yaw.est.t[1], yaw.ok = saved.yaw_b, true
  end
  if saved and util.finite(saved.sway_b) and math.abs(saved.sway_b) >= c.min_sway_authority then
    sway.est.t[1], sway.ok = saved.sway_b, true
  end
  return setmetatable({ c = c, yaw = yaw, sway = sway, integ = 0, err = 0, r_cmd = 0, win = nil }, M)
end

function M:reset()
  self.integ = 0
end

-- Fold one fitted sample into a learner and track direction confirmation.
local function learn(l, x1, x2, y, informative, min_b)
  local before = l.est.t[1]
  l.est:update(x1, x2, y)
  if l.est.t[2] < 0 then l.est.t[2] = 0 end
  local b = l.est.t[1]
  if util.sign(b) ~= util.sign(before) then
    l.ok, l.count = false, 0
  end
  if informative then l.count = l.count + 1 end
  if not l.ok and l.count >= 4 and math.abs(b) >= min_b then l.ok = true end
end

-- Learn RCS authority from 0.5 s windows. u and sway are the forces the
-- controllers are actually applying (from their current RPM).
function M:observe(s, u, sway, now)
  local w = self.win
  if w == nil then
    self.win = { t0 = now, r0 = s.yaw_rate, v0 = s.v_side, u = 0, sw = 0, r = 0, v = 0, f = 0, n = 0 }
    return
  end
  w.u, w.sw, w.r, w.v, w.f, w.n = w.u + u, w.sw + sway, w.r + s.yaw_rate, w.v + s.v_side, w.f + s.v_fwd, w.n + 1
  local span = now - w.t0
  if span < 0.5 then return end
  self.win = { t0 = now, r0 = s.yaw_rate, v0 = s.v_side, u = 0, sw = 0, r = 0, v = 0, f = 0, n = 0 }

  local n = w.n
  local mu, msw, mr, mv, mf = w.u / n, w.sw / n, w.r / n, w.v / n, w.f / n
  local c = self.c
  if math.abs(mu) >= 0.03 or math.abs(mr) >= 0.3 then
    learn(self.yaw, mu, -mr, (s.yaw_rate - w.r0) / span, math.abs(mu) >= 0.05, c.min_authority)
  end
  if math.abs(msw) >= 0.03 or math.abs(mv) >= 0.2 then
    -- Turning rotates the body frame under a fixed velocity: take that out.
    local a_side = (s.v_side - w.v0) / span + mr * RAD * mf
    learn(self.sway, msw, -mv, a_side, math.abs(msw) >= 0.05, c.min_sway_authority)
  end
end

local function signed_floor(b, floor)
  return (b >= 0 and 1 or -1) * math.max(math.abs(b), floor)
end

-- Returns yaw force u and sway force s, both normalised to [-1, 1].
function M:command(s, bearing, dt, sway_on)
  local c = self.c
  local err = util.wrap180(bearing - s.hdg)
  local dist = math.abs(err)
  local b = self.yaw.est.t[1]
  local decel = math.min(c.yaw_accel, 0.5 * math.max(math.abs(b), c.min_authority))
  local max_rate = self.yaw.ok and c.max_rate or 0.5 * c.max_rate
  local r_cmd = util.sign(err) * math.min(max_rate, c.bearing_gain * dist, math.sqrt(2 * decel * dist))
  local r_err = r_cmd - s.yaw_rate
  local resting = dist < c.deadband and math.abs(s.yaw_rate) < 0.15

  local limit = self.yaw.ok and 1 or c.probe_limit
  local b_eff = signed_floor(b, c.min_authority)
  if not resting then
    local int_gain = c.int_gain
    if s.gs > c.high_speed then int_gain = int_gain * c.high_speed_gain end
    local grown = util.clamp(self.integ + int_gain * r_err * dt, -c.int_limit, c.int_limit)
    -- No wind-up while the output is pinned at its limit.
    local u_try = (grown + c.rate_gain * r_err) / b_eff
    if math.abs(u_try) < limit or math.abs(grown) < math.abs(self.integ) then self.integ = grown end
  end
  local accel = self.integ
  if not resting then accel = accel + c.rate_gain * r_err end
  local u = util.clamp(accel / b_eff, -limit, limit)

  local sway = 0
  if sway_on and math.abs(s.v_side) > 0.2 then
    local sb = signed_floor(self.sway.est.t[1], c.min_sway_authority)
    local slimit = self.sway.ok and 1 or c.probe_limit
    sway = util.clamp(-c.sway_gain * s.v_side / sb, -slimit, slimit)
  end
  -- Heading has priority: sway gets what is left.
  local room = 1 - math.abs(u)
  sway = util.clamp(sway, -room, room)

  self.err, self.r_cmd = err, r_cmd
  return u, sway
end

-- Yaw and sway forces currently applied, recovered from the four RPMs.
function M.applied(rpm, caps)
  local function f(id) return util.ssq(rpm[id] or 0) / (caps[id] * caps[id]) end
  local bs, bp, ss, sp = f("bow_stbd"), f("bow_port"), f("stern_stbd"), f("stern_port")
  return (bs + sp - bp - ss) / 4, (bs + ss - bp - sp) / 4
end

-- Map yaw/sway forces to the four pullers' RPM.
function M.mix(u, sway, caps, min_rpm)
  local f = {
    bow_stbd = u + sway,
    stern_port = u - sway,
    bow_port = -u - sway,
    stern_stbd = -u + sway,
  }
  local rpm = {}
  for id, force in pairs(f) do
    local r = util.ssqrt(util.clamp(force, -1, 1)) * caps[id]
    if math.abs(r) < min_rpm then r = 0 end
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
