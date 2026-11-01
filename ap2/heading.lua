-- Heading hold and sideslip cancelling on the four RCS pullers.
--
-- The RCS only ever pulls (positive RPM); no controller reverses. Corner
-- forces for a turn force u (> 0 = right) and a sway force s (> 0 = toward
-- starboard), each corner clamped at 0:
--   bow stbd = u + s    stern port = u - s
--   bow port = -u - s   stern stbd = s - u
-- A pure turn is a diagonal pair (right: bow stbd + stern port), a pure
-- strafe a same-side pair (starboard: bow stbd + stern stbd), and any blend
-- gives exactly 2u of turn and 2s of sway.
--
-- The turning authority is learned in flight, sign included: every 0.5 s the
-- measured yaw acceleration is fitted to yaw_accel = b * u - d * yaw_rate
-- (u = turn force the props are really applying). Commands are computed as an
-- acceleration and divided by b. Until the direction has been confirmed by a
-- few samples, the turn is held to probe_limit.
--
-- Heading: bearing error sets a yaw-rate command; a PI loop on yaw rate gives
-- the yaw acceleration. The integral holds off steady torques such as a rear
-- thrust line that misses the centre of mass. A correction starts past the
-- deadzone and stops within settle_deg; in between the learned trim is held.
--
-- Sideslip: the direction a sway force really moves the hull, and how hard,
-- are learned by ap2/motion.lua; the brake works along that direction, so it
-- does not depend on the configured frame or on which way the pullers act.
-- Drift past sway_engage starts the brake, which runs until the drift is
-- under sway_settle; the brake also stops early when the props' remaining
-- spin will finish the job (they take over a second to spin down). Heading
-- has first call on the RCS; sway gets what is left.
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
  return setmetatable({
    c = c, yaw = yaw, integ = 0, err = 0, r_cmd = 0, win = nil,
    active = true, sway_active = false, drift = 0,
  }, M)
end

function M:reset()
  self.integ = 0
end

-- Learn the turning authority from 0.5 s windows. u is the turn force the
-- props are really applying.
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
  l.est:update({ mu, -mr }, (s.yaw_rate - w.r0) / span)
  if l.est.t[2] < 0 then l.est.t[2] = 0 end
  local b = l.est.t[1]
  if util.sign(b) ~= util.sign(before) then
    l.ok, l.count = false, 0
  end
  if math.abs(mu) >= 0.05 then l.count = l.count + 1 end
  if not l.ok and l.count >= 4 and math.abs(b) >= self.c.min_authority then l.ok = true end
end

local function signed_floor(b, floor)
  return (b >= 0 and 1 or -1) * math.max(math.abs(b), floor)
end

-- Returns the turn force u and the sway force sw, both normalised.
-- deadzone: bearing error (deg) tolerated before an active correction starts.
-- sway: { drift = m/s along the learned strafe direction, auth = m/s^2 at
-- full sway force, ok = learned, coming = m/s the props' current spin will
-- still add to the drift } or nil to leave sideslip alone.
-- pending: turn force the controllers are set to but the props have not yet
-- reached (they spin up and down at 32 RPM a tick); it adds a lead to the
-- yaw-rate loop so it does not keep pushing what is already on its way.
function M:command(s, bearing, dt, deadzone, sway, pending)
  local c = self.c
  local err = util.wrap180(bearing - s.hdg)
  local dist = math.abs(err)
  local b = self.yaw.est.t[1]
  local limit = self.yaw.ok and 1 or c.probe_limit
  local b_eff = signed_floor(b, c.min_authority)
  local rate = s.yaw_rate + (self.yaw.ok and pending and b * pending * c.yaw_lag or 0)

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
    local r_err = r_cmd - rate
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

  -- Sideslip brake with engage/settle hysteresis, on the drift the hull will
  -- have once the props' current spin has played out.
  local sw = 0
  self.drift = 0
  if sway then
    local drift = sway.drift + (sway.coming or 0)
    self.drift = sway.drift
    if self.sway_active then
      if math.abs(drift) < c.sway_settle or math.abs(sway.drift) < c.sway_settle then self.sway_active = false end
    elseif math.abs(sway.drift) > c.sway_engage then
      self.sway_active = true
    end
    if self.sway_active then
      local auth = math.max(sway.auth, c.min_sway_authority)
      local slimit = sway.ok and 1 or c.probe_limit
      sw = util.clamp(-c.sway_gain * drift / auth, -slimit, slimit)
      local room = 1 - math.abs(u)
      sw = util.clamp(sw, -room, room)
    end
  end
  return u, sw
end

-- Turn and sway forces from four corner RPMs.
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
  return { yaw_b = self.yaw.ok and self.yaw.est.t[1] or nil }
end

return M
