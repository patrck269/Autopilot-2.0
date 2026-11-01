-- Heading hold and sideslip damping on the four RCS pullers.
--
-- Bearing error sets a yaw-rate command (proportional, limited by max_rate and
-- by the rate that can still stop at the bearing); a PI loop on yaw rate gives
-- a normalised yaw force. The integral holds off steady torques such as a rear
-- thrust line that misses the estimated centre of mass. Above high_speed the
-- gains are scaled up because those torques grow with thrust and airspeed.
--
-- Mixing (positive RPM pulls the hull toward that thruster, negative pushes):
--   right turn  u > 0:  bow stbd +u, stern port +u, bow port -u, stern stbd -u
--   to starboard s > 0: bow stbd +s, stern stbd +s, bow port -s, stern port -s
local util = require("ap2.util")

local M = {}
M.__index = M

function M.new(cfg)
  return setmetatable({ c = cfg.heading, integ = 0, err = 0, r_cmd = 0 }, M)
end

function M:reset()
  self.integ = 0
end

-- Returns yaw force u and sway force s, both normalised to [-1, 1].
function M:command(s, bearing, dt, sway_on)
  local c = self.c
  local err = util.wrap180(bearing - s.hdg)
  local dist = math.abs(err)
  local gain = 1
  if s.gs > c.high_speed then gain = c.high_speed_gain end

  local r_cmd = util.sign(err) * math.min(c.max_rate, c.bearing_gain * dist, math.sqrt(2 * c.yaw_accel * dist))
  local r_err = r_cmd - s.yaw_rate
  local resting = dist < c.deadband and math.abs(s.yaw_rate) < 0.15
  if not resting then
    self.integ = util.clamp(self.integ + c.int_gain * gain * r_err * dt, -c.int_limit, c.int_limit)
  end
  local u = self.integ
  if not resting then u = u + c.rate_gain * gain * r_err end

  local sway = 0
  if sway_on and math.abs(s.v_side) > 0.2 then
    sway = -c.sway_gain * gain * s.v_side
  end

  u = util.clamp(u, -1, 1)
  -- Heading has priority: sway gets what is left.
  local room = 1 - math.abs(u)
  sway = util.clamp(sway, -room, room)

  self.err, self.r_cmd = err, r_cmd
  return u, sway
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

return M
