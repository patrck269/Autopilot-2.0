-- Altitude hold with automatic hover calibration.
--
-- Clockwork blade propellers make thrust 0.5 * rho(y) * v_tip^2 * C_L * area,
-- and blade tip speed is proportional to RPM (inflow from climbing is tiny next
-- to tip speed). So the vertical acceleration of the hull is
--     a = k * rho(y) * rpm * |rpm| - g
-- with a single ship constant k (blade area / mass). Once k is known the hover
-- RPM at any height h is sqrt(g / (k * rho(h))), taking rho from VS's
-- atmosphere (aero.getAirDensity). k is learned in flight from 1-second
-- windows of measured acceleration against the RPM actually applied, so it
-- follows cargo and hull changes too. On the ground, while asked to climb,
-- RPM is raised step by step until the hull lifts off.
local util = require("ap2.util")

local M = {}
M.__index = M

-- cfg: full config. saved: { k = number } from a previous flight, or nil.
-- sea_rho: air density at sea level, used to seed k from default_hover_rpm.
function M.new(cfg, saved, sea_rho)
  local a = cfg.altitude
  local g = cfg.gravity
  local k
  if saved and util.finite(saved.k) and saved.k > 0 then
    k = saved.k
  else
    k = g / ((sea_rho or 1) * a.default_hover_rpm * a.default_hover_rpm)
  end
  return setmetatable({
    a = a,
    g = g,
    k = k,
    integ = 0,
    grounded = false,
    samples = 0,
    win = nil,
    phase = "IDLE",
    saturated = false,
    last_sample = nil,
  }, M)
end

-- Hover RPM at air density rho.
function M:hover_rpm(rho)
  return math.sqrt(self.g / (self.k * rho))
end

-- RPM that produces vertical acceleration accel at density rho (negative RPM =
-- reverse lift, only when more than gravity is needed downward).
function M:rpm_for(accel, rho)
  return util.ssqrt((self.g + accel) / (self.k * rho))
end

-- Seed k from an RPM that is holding the hull level right now.
function M:seed_from_hover(rpm, rho)
  if rpm and rpm > 0 and rho and rho > 0 then
    self.k = self.g / (rho * rpm * rpm)
    self.win = nil
  end
end

function M:reset_calibration(sea_rho)
  self.k = self.g / ((sea_rho or 1) * self.a.default_hover_rpm * self.a.default_hover_rpm)
  self.samples = 0
  self.win = nil
  self.integ = 0
end

-- Calibration. Call every control step with the RPM currently on the lift
-- controller. want_up: the controller is asking to climb.
function M:observe(s, applied_rpm, now, want_up)
  local w = self.win
  if w == nil then
    self.win = { t0 = now, vy0 = s.vy, f = 0, rho = 0, vy = 0, n = 0 }
    return
  end
  w.f = w.f + util.ssq(applied_rpm)
  w.rho = w.rho + s.rho
  w.vy = w.vy + s.vy
  w.n = w.n + 1
  local span = now - w.t0
  if span < self.a.cal_window then return end
  self.win = { t0 = now, vy0 = s.vy, f = 0, rho = 0, vy = 0, n = 0 }

  local accel = (s.vy - w.vy0) / span
  local force = w.f / w.n
  local rho = w.rho / w.n
  local vy = w.vy / w.n
  local predicted = self.k * rho * force - self.g

  -- Held up by something (ground, dock): still while the model says it should
  -- sink harder than the altitude integral alone can ask for.
  if math.abs(accel) < 0.15 and math.abs(vy) < 0.08 and predicted < -1.05 then
    self.grounded = true
  elseif vy > 0.25 or vy < -0.5 then
    self.grounded = false
  end

  if self.grounded then
    if want_up then
      self.k = self.k * self.a.liftoff_step
    end
    return
  end
  if force <= 0 or math.abs(vy) > self.a.cal_max_vy then return end
  local sample = (accel + self.g) / (rho * force)
  if not util.finite(sample) or sample <= 0 then return end
  local ratio = util.clamp(sample / self.k, 0.67, 1.5)
  self.k = self.k * ratio ^ self.a.cal_rate
  self.samples = self.samples + 1
  self.last_sample = sample
end

-- Lift RPM to fly toward target world Y. Returns rpm.
-- Settled inside hold_band, the RPM is held as it is (no new writes) until
-- the hull drifts past hold_release or starts moving; calibration keeps
-- learning from the held RPM, so each re-settle lands closer.
function M:command(s, target, dt, cap)
  local a = self.a
  local err = target - s.y
  local dist = math.abs(err)
  if self.held then
    if self.grounded or dist > a.hold_release or math.abs(s.vy) > 0.4 or math.abs(self.held) > cap then
      self.held = nil
    else
      self.phase = "HOLD"
      return self.held, 0, 0
    end
  end
  local vmax = err > 0 and a.climb_rate or a.descent_rate
  local v_cmd = util.sign(err) * math.min(vmax, a.pos_gain * dist, math.sqrt(2 * a.brake_accel * dist))

  if dist < a.int_zone and not self.grounded then
    self.integ = util.clamp(self.integ + a.int_gain * err * dt, -a.int_limit, a.int_limit)
  end

  local accel = a.vel_gain * (v_cmd - s.vy) + self.integ
  accel = util.clamp(accel, a.min_accel, a.max_accel)

  local reverse_ok = a.reverse_lift and not self.grounded
  if self.grounded then
    if err > a.hold_band then accel = a.liftoff_accel else accel = math.max(accel, -0.5 * self.g) end
  end

  local rpm = self:rpm_for(accel, s.rho)
  if rpm < 0 and not reverse_ok then rpm = 0 end
  self.saturated = math.abs(rpm) > cap
  rpm = util.clamp(rpm, -cap, cap)

  if self.grounded then
    self.phase = err > a.hold_band and "LIFTOFF" or "GROUND"
  elseif dist <= a.hold_band and math.abs(s.vy) < 0.4 then
    self.phase = "HOLD"
  elseif dist <= 5 then
    self.phase = "CAPTURE"
  elseif err > 0 then
    self.phase = "CLIMB"
  else
    self.phase = "DESCEND"
  end
  if not self.grounded and dist <= a.hold_band and math.abs(s.vy) < 0.25 then
    -- Hold what keeps the hull level: the calibrated hover RPM here plus the
    -- altitude trim, not this instant's correction.
    self.held = util.clamp(self:rpm_for(self.integ, s.rho), -cap, cap)
  end
  return rpm, v_cmd, accel
end

return M
