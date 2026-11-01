-- Reads ship state from the CC:VS `ship` and `aero` APIs and expresses it in
-- pilot terms: world Y, vertical speed, compass heading, forward/side speed,
-- pitch, roll and yaw rate.
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
  local fwd = unit(cfg.frame.forward)
  local up = unit(cfg.frame.up)
  local right = util.cross(fwd, up)
  local ship, aero, clock = api.ship, api.aero, api.clock
  local last_t, last_pitch, last_hdg, last_rate

  local self = {}

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

    local s = {
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
