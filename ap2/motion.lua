-- Learns how the hull really moves, in its own horizontal plane (telemetry
-- axes e1, e2, fixed to the hull):
--   * the rear thrusters push toward the bow (positive RPM) and the starboard
--     pullers pull toward starboard (positive RPM, checked in game). So the
--     direction each of them really moves the hull shows where the bow is.
--     Once that is clear it is snapped to the block grid; if it differs from
--     the configured frame.forward the autopilot switches to it, and if it
--     agrees the frame counts as confirmed
--   * the sideways strength of the RCS (m/s^2 at full sway force)
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
  local self = setmetatable({ cfg = cfg, tel = tel, win = nil, n_thrust = 0, n_sway = 0,
    bow = nil, frame_ok = false, votes = {} }, M)
  local f1, f2 = tel.plane(tel.forward)
  local r1, r2 = -f2, f1
  local A, S = cfg.speed.accel_full, cfg.heading.sway_authority
  local p = { 4 * A * A + 1, 25 * S * S + 0.1, 0.05 }
  self.fit = {
    rls.new({ A * f1, S * r1, 0.02 }, p, 0.97),
    rls.new({ A * f2, S * r2, 0.02 }, p, 0.97),
  }
  return self
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
  local c = self.cfg.frame
  if math.abs(mf) >= 0.1 then
    self.n_thrust = self.n_thrust + 1
    -- Thrust pushes toward the bow.
    self:vote("thrust", self.fit[1].t[1], self.fit[2].t[1], self.n_thrust, c.bow_min_accel)
  end
  if math.abs(msw) >= 0.05 then
    self.n_sway = self.n_sway + 1
    -- Sway pushes toward starboard; the bow is 90 deg anticlockwise of that.
    local g1, g2 = self.fit[1].t[2], self.fit[2].t[2]
    self:vote("sway", g2, -g1, self.n_sway, self.cfg.heading.min_sway_authority)
  end
end

-- One source's view of the bow axis (b1, b2 in hull-plane coordinates). Once
-- it has enough samples, a real response, lies within bow_snap_deg of a grid
-- axis and gives the same axis bow_streak times running, it decides.
function M:vote(src, b1, b2, n, min_mag)
  local c = self.cfg.frame
  local v = self.votes[src] or { cand = nil, streak = 0 }
  self.votes[src] = v
  if n < c.bow_samples or math.sqrt(b1 * b1 + b2 * b2) < min_mag then return end
  local ang = atan2(b2, b1) * DEG
  local snap = util.round(ang / 90) * 90
  if math.abs(util.wrap180(ang - snap)) > c.bow_snap_deg then
    v.streak = 0
    return
  end
  local cand = snap % 360
  if cand == v.cand then v.streak = v.streak + 1 else v.cand, v.streak = cand, 1 end
  if v.streak < c.bow_streak then return end
  local t = self.tel
  local r = math.rad(cand)
  local c1, c2 = util.round(math.cos(r)), util.round(math.sin(r))
  local bow = { x = c1 * t.e1.x + c2 * t.e2.x, y = c1 * t.e1.y + c2 * t.e2.y, z = c1 * t.e1.z + c2 * t.e2.z }
  if util.dot(bow, t.forward) > 0.9 then
    self.frame_ok = true
  elseif src == "thrust" or not (self.votes.thrust and self.votes.thrust.streak >= c.bow_streak) then
    -- The rear thrusters' view wins if both have decided.
    self.bow = bow
  end
end

-- Sideways strength of the RCS along the hull's starboard axis (m/s^2 at full).
function M:sway_authority()
  local f1, f2 = self.tel.plane(self.tel.forward)
  local g = self.fit[1].t[2] * -f2 + self.fit[2].t[2] * f1
  return math.max(g, self.cfg.heading.min_sway_authority)
end

-- Bow axis in hull-plane degrees from e1 toward e2 (clockwise seen from above).
function M.plane_angle(tel, v)
  local a, b = tel.plane(v)
  return atan2(b, a) * DEG
end

return M
