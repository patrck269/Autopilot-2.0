-- Reads ship state from the CC:VS `ship` and `aero` APIs and expresses it in
-- pilot terms: world Y, vertical speed, compass heading, forward/side speed,
-- pitch, roll and yaw rate. It also reports velocity in the hull's own
-- horizontal plane (axes e1, e2 fixed to the hull), which the autopilot uses
-- to learn which way the hull's bow and strafe really point.
local util = require("ap2.util")

local M = {}

local atan2 = math.atan2 or math.atan
local DEG = 180 / math.pi

local function unit(v)
  local l = util.len(v)
  if l <= 0 then return { x = 0, y = 0, z = 0 } end
  return { x = v.x / l, y = v.y / l, z = v.z / l }
end

-- api = { ship = <CC:VS ship API>, aero = <CC:VS aero API or nil>, clock = os.clock }
function M.new(cfg, api)
  local up = unit(cfg.frame.up)
  local fwd, right
  -- Hull-fixed horizontal basis: e1 = the grid axis most across "up", e2 = e1 x up.
  local e1 = math.abs(up.x) < 0.9 and { x = 1, y = 0, z = 0 } or { x = 0, y = 0, z = 1 }
  e1 = unit({ x = e1.x - util.dot(e1, up) * up.x, y = e1.y - util.dot(e1, up) * up.y, z = e1.z - util.dot(e1, up) * up.z })
  local e2 = util.cross(e1, up)
  local ship, aero, clock = api.ship, api.aero, api.clock
  local last_t, last_pitch, last_hdg, last_rate

  local self = { e1 = e1, e2 = e2, up = up }

  -- Use hull-local vector v as the bow (snapped bow axes come from learning).
  function self.set_forward(v)
    fwd = unit(v)
    right = util.cross(fwd, up)
    self.forward = fwd
    last_t = nil -- the heading jumps: no yaw-rate spike
  end
  self.set_forward(cfg.frame.forward)

  -- Components of a hull-local vector in the hull's horizontal plane.
  function self.plane(v)
    return util.dot(v, e1), util.dot(v, e2)
  end

  -- Air density at world Y; 1.0 when the aero API is unavailable (the hover
  -- calibration then absorbs the density change as it flies).
  function self.rho_at(y)
    if aero and aero.getAirDensity then
      local ok, rho = pcall(aero.getAirDensity, y)
      if ok and util.finite(rho) and rho > 0 then return rho end
    end
    return 1.0
  end

  function self.read()
    local now = clock()
    local pos = ship.getWorldspacePosition()
    local vel = ship.getVelocity()
    local q = ship.getQuaternion()
    local mass = ship.getMass and ship.getMass() or nil

    local f = util.rotate(q, fwd)
    local r = util.rotate(q, right)
    local fh = unit({ x = f.x, y = 0, z = f.z })
    local rh = { x = -fh.z, y = 0, z = fh.x }

    local heading = (atan2(f.x, -f.z) * DEG) % 360
    local pitch = math.asin(util.clamp(f.y, -1, 1)) * DEG
    local roll = math.asin(util.clamp(-r.y, -1, 1)) * DEG
    -- Rates come from the change in attitude between samples, so heading and
    -- yaw rate always agree in sign (no reliance on the angular velocity frame).
    local pitch_rate, yaw_rate = 0, 0
    if last_t and now > last_t then
      local dt = now - last_t
      pitch_rate = (pitch - last_pitch) / dt
      yaw_rate = util.wrap180(heading - last_hdg) / dt
      if last_rate and dt < 0.5 then yaw_rate = 0.5 * last_rate + 0.5 * yaw_rate end
    end
    last_t, last_pitch, last_hdg, last_rate = now, pitch, heading, yaw_rate

    local vl = util.rotate_inv(q, vel)
    local lv1, lv2 = self.plane(vl)
    local s = {
      q = q,
      lv1 = lv1, lv2 = lv2, -- velocity along the hull's own horizontal axes
      t = now,
      x = pos.x, y = pos.y, z = pos.z,
      vx = vel.x, vy = vel.y, vz = vel.z,
      gs = math.sqrt(vel.x * vel.x + vel.z * vel.z),
      v_fwd = util.dot(vel, fh),
      v_side = util.dot(vel, rh),
      hdg = heading,
      pitch = pitch,
      roll = roll,
      pitch_rate = pitch_rate,
      yaw_rate = yaw_rate, -- deg/s, clockwise (compass increasing) positive
      mass = mass,
      rho = self.rho_at(pos.y),
    }
    if s.gs > 0.5 then
      s.track = (atan2(vel.x, -vel.z) * DEG) % 360
    end
    return s
  end

  return self
end

return M
