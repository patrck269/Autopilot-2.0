-- Learns how the hull really moves, in its own horizontal plane (telemetry
-- axes e1, e2, fixed to the hull), so control does not depend on the
-- configured frame being right:
--   * the rear thrusters' push (positive RPM pushes toward the bow) gives the
--     bow axis. Once it is clear, it is snapped to the block grid and, if it
--     differs from the configured frame.forward, the autopilot switches to it
--   * the strafe pairs' push gives the direction a sway force really moves
--     the hull; the sideslip brake works along that direction
-- Every 0.5 s the hull-plane acceleration is fitted, per axis k, to
--     a_k = h_k * thrust + g_k * sway - d_k * v_k
-- (thrust, sway = forces the props are really applying, from the modelled
-- spin; v_k = hull-plane velocity).
local util = require("ap2.util")
local rls = require("ap2.rls")

local M = {}
M.__index = M

local DEG = 180 / math.pi
local atan2 = math.atan2 or math.atan

-- tel: telemetry (for its plane axes and current forward).
function M.new(cfg, tel)
  local c = cfg.heading
  local self = setmetatable({ cfg = cfg, tel = tel, win = nil, n_thrust = 0, n_sway = 0,
    bow = nil, streak = 0, candidate = nil }, M)
  self:reset_axes()
  return self
end

-- Start the fits from the configured frame (forward = bow, right = strafe).
function M:reset_axes()
  local cfg, tel = self.cfg, self.tel
  local f1, f2 = tel.plane(tel.forward)
  local r = util.cross(tel.forward, tel.up)
  local r1, r2 = tel.plane(r)
  local A = cfg.speed.accel_full
  local S = cfg.heading.sway_authority
  local p = { 4 * A * A + 1, 25 * S * S + 0.1, 0.05 }
  self.fit = {
    rls.new({ A * f1, S * r1, 0.02 }, p, 0.97),
    rls.new({ A * f2, S * r2, 0.02 }, p, 0.97),
  }
  self.n_sway = 0
end

-- Feed one control step. thrust, sway: normalised forces really applied.
function M:observe(s, thrust, sway, now)
  local w = self.win
  if w == nil then
    self.win = { t0 = now, vx = s.vx, vy = s.vy, vz = s.vz, f = 0, sw = 0, v1 = 0, v2 = 0, n = 0 }
    return
  end
  w.f, w.sw, w.v1, w.v2, w.n = w.f + thrust, w.sw + sway, w.v1 + s.lv1, w.v2 + s.lv2, w.n + 1
  local span = now - w.t0
  if span < 0.5 then return end
  self.win = { t0 = now, vx = s.vx, vy = s.vy, vz = s.vz, f = 0, sw = 0, v1 = 0, v2 = 0, n = 0 }

  local mf, msw = w.f / w.n, w.sw / w.n
  if math.abs(mf) < 0.05 and math.abs(msw) < 0.05 then return end
  local aw = { x = (s.vx - w.vx) / span, y = (s.vy - w.vy) / span, z = (s.vz - w.vz) / span }
  local a1, a2 = self.tel.plane(util.rotate_inv(s.q, aw))
  self.fit[1]:update({ mf, msw, -w.v1 / w.n }, a1)
  self.fit[2]:update({ mf, msw, -w.v2 / w.n }, a2)
  if math.abs(mf) >= 0.1 then self.n_thrust = self.n_thrust + 1 end
  if math.abs(msw) >= 0.05 then self.n_sway = self.n_sway + 1 end
  self:detect_bow()
end

-- Thrust and strafe response vectors in the hull plane (m/s^2 at full force).
function M:thrust_vec()
  return self.fit[1].t[1], self.fit[2].t[1]
end

function M:strafe_vec()
  return self.fit[1].t[2], self.fit[2].t[2]
end

-- Snap the learned thrust direction to the nearest grid axis once it is
-- clear: enough thrust samples, a real response, within snap_deg of an axis,
-- and the same answer bow_streak times running.
function M:detect_bow()
  local c = self.cfg.frame
  local h1, h2 = self:thrust_vec()
  local mag = math.sqrt(h1 * h1 + h2 * h2)
  if self.n_thrust < c.bow_samples or mag < c.bow_min_accel then return end
  local ang = atan2(h2, h1) * DEG
  local snap = util.round(ang / 90) * 90
  if math.abs(util.wrap180(ang - snap)) > c.bow_snap_deg then
    self.streak = 0
    return
  end
  local cand = util.round(snap) % 360
  if cand == self.candidate then
    self.streak = self.streak + 1
  else
    self.candidate, self.streak = cand, 1
  end
  if self.streak >= c.bow_streak then
    local t = self.tel
    local r = math.rad(cand)
    local c1, c2 = util.round(math.cos(r)), util.round(math.sin(r))
    self.bow = {
      x = c1 * t.e1.x + c2 * t.e2.x, y = c1 * t.e1.y + c2 * t.e2.y, z = c1 * t.e1.z + c2 * t.e2.z,
    }
  end
end

-- Is the learned strafe direction trustworthy yet?
function M:sway_ok()
  local g1, g2 = self:strafe_vec()
  return self.n_sway >= 4 and math.sqrt(g1 * g1 + g2 * g2) >= self.cfg.heading.min_sway_authority
end

-- Drift along the direction a positive sway force moves the hull (m/s), and
-- the sway authority (m/s^2 at full force).
function M:strafe_drift(s)
  local g1, g2 = self:strafe_vec()
  local mag = math.sqrt(g1 * g1 + g2 * g2)
  if mag < 1e-6 then return 0, 0 end
  return (s.lv1 * g1 + s.lv2 * g2) / mag, mag
end

-- Bow axis in hull-plane degrees from e1 toward e2 (clockwise seen from above).
function M.plane_angle(tel, v)
  local a, b = tel.plane(v)
  return atan2(b, a) * DEG
end

return M
