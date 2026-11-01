-- Physics stand-in for a Valkyrien Skies hull on Clockwork propellers, with
-- fake Create rotation speed controllers and CC:VS ship/aero APIs. No
-- requires, so the same file runs in desktop Lua and inside CraftOS-PC.
--   * lift   = k * rho(y) * rpm|rpm|       (Clockwork blade thrust law)
--   * thrust = b * rho(y) * rpm|rpm| along the bow, linear + quadratic drag
--   * RCS pullers: sway force and yaw torque, yaw damping
--   * rear thrust that misses the centre of mass: yaw torque and nose dip
--   * props follow the controller with a short lag; the ground holds the hull
--   * Create flicker score per controller (+10 per change, -1 per tick)
local W = {}

local RAD = math.pi / 180

W.DEFAULTS = {
  hover_rpm_sea = 900,  -- true hover RPM at sea-level density
  thrust_full = 4.0,    -- forward m/s^2 at 1500 RPM, sea level
  drag1 = 0.01,         -- linear drag, 1/s
  drag2 = 0.001,        -- quadratic drag, 1/m
  side_drag = 0.3,
  rcs_accel = 0.6,      -- sway m/s^2 from one RCS at full RPM
  yaw_accel = 2.0,      -- deg/s^2 from one RCS at full RPM
  yaw_damp = 0.15,
  misalign = 0.0,       -- deg/s^2 of yaw per m/s^2 of rear thrust
  dip = 0.0,            -- deg/s^2 nose down per m/s^2 of rear thrust
  pitch_spring = 0.4,
  pitch_damp = 0.8,
  prop_lag = 0.15,
  ground_y = 64,
  noise = 0,            -- +- m/s added to each velocity reading
}

local function ssq(x) return x * math.abs(x) end

local function qmul(a, b)
  return {
    w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  }
end

local function axis_angle(x, y, z, ang)
  local s = math.sin(ang / 2)
  return { x = x * s, y = y * s, z = z * s, w = math.cos(ang / 2) }
end

-- Orientation for compass heading psi and pitch theta (degrees), bow = local +X.
function W.quat(psi, theta)
  return qmul(axis_angle(0, 1, 0, (90 - psi) * RAD), axis_angle(0, 0, 1, theta * RAD))
end

function W.rho(y)
  return 1.225 * math.exp(-(y - 62) / 700)
end

-- controllers: cfg.controllers. opts: overrides of DEFAULTS plus y, psi, rpm.
function W.new(controllers, opts, gravity)
  opts = opts or {}
  local p = {}
  for k, v in pairs(W.DEFAULTS) do p[k] = v end
  for k, v in pairs(opts) do p[k] = v end
  gravity = gravity or 10

  local sea = W.rho(62)
  local w = {
    p = p,
    x = 0, y = opts.y or p.ground_y, z = 0,
    vx = 0, vy = 0, vz = 0,
    psi = opts.psi or 0, r = 0, theta = 0, q = 0,
    rpm = {}, cmd = {}, flick = {}, peak = {}, writes = 0,
    devices = {},
  }
  local k_lift = gravity / (sea * p.hover_rpm_sea ^ 2)
  local k_thrust = p.thrust_full / (sea * 1500 ^ 2)

  for id, c in pairs(controllers) do
    w.rpm[id] = opts.rpm and opts.rpm[id] or 0
    w.cmd[id] = w.rpm[id]
    w.flick[id] = 0
    w.peak[id] = 0
    w.devices[id] = {
      setTargetSpeed = function(v)
        assert(type(v) == "number", "setTargetSpeed expects a number")
        assert(v == math.floor(v), "setTargetSpeed should get an integer, got " .. v)
        assert(math.abs(v) <= c.cap, id .. " over cap: " .. v)
        if v ~= w.cmd[id] then
          w.flick[id] = w.flick[id] + 10
          if w.flick[id] > w.peak[id] then w.peak[id] = w.flick[id] end
        end
        w.cmd[id] = v
        w.writes = w.writes + 1
      end,
      getTargetSpeed = function() return w.cmd[id] end,
    }
  end

  w.ship = {
    getId = function() return 7 end,
    getSlug = function() return "test-hull" end,
    getWorldspacePosition = function() return { x = w.x, y = w.y, z = w.z } end,
    getVelocity = function()
      local function jitter() return (math.random() * 2 - 1) * p.noise end
      return { x = w.vx + jitter(), y = w.vy + jitter(), z = w.vz + jitter() }
    end,
    getQuaternion = function() return W.quat(w.psi, w.theta) end,
    getAngularVelocity = function() return { x = 0, y = -w.r * RAD, z = 0 } end,
    getMass = function() return 50000 end,
  }
  w.aero = { getAirDensity = function(y) return W.rho(y or w.y) end }

  -- Advance the hull by dt seconds (one game tick is 0.05).
  function w.step(dt)
    local rho = W.rho(w.y)
    for id, v in pairs(w.cmd) do
      w.rpm[id] = w.rpm[id] + (v - w.rpm[id]) * math.min(1, dt / p.prop_lag)
      w.flick[id] = math.max(0, w.flick[id] - 1)
    end
    local function f(id) return ssq(w.rpm[id]) end
    local a_lift = k_lift * rho * f("lift") - gravity
    local a_thr = k_thrust * rho * f("thrust")
    local rc = 1 / (2500 * 2500)
    local bs, bp = f("bow_stbd") * rc, f("bow_port") * rc
    local ss, sp = f("stern_stbd") * rc, f("stern_port") * rc
    local a_side = p.rcs_accel * (bs + ss - bp - sp) * rho / sea
    local yaw_acc = p.yaw_accel * ((bs - bp) - (ss - sp)) * rho / sea
      - p.yaw_damp * w.r + p.misalign * a_thr

    local fx, fz = math.sin(w.psi * RAD), -math.cos(w.psi * RAD)
    local rx, rz = -fz, fx
    local vf = w.vx * fx + w.vz * fz
    local vs = w.vx * rx + w.vz * rz
    local af = a_thr - p.drag1 * vf - p.drag2 * vf * math.abs(vf)
    local as = a_side - p.side_drag * vs
    w.vx = w.vx + (af * fx + as * rx) * dt
    w.vz = w.vz + (af * fz + as * rz) * dt
    w.vy = w.vy + a_lift * dt
    w.x = w.x + w.vx * dt
    w.z = w.z + w.vz * dt
    w.y = w.y + w.vy * dt
    if w.y <= p.ground_y then
      w.y = p.ground_y
      if w.vy < 0 then w.vy = 0 end
      w.vx, w.vz = w.vx * 0.9, w.vz * 0.9
    end
    w.r = w.r + yaw_acc * dt
    w.psi = (w.psi + w.r * dt) % 360
    local q_acc = -p.dip * a_thr - p.pitch_spring * w.theta - p.pitch_damp * w.q
    w.q = w.q + q_acc * dt
    w.theta = w.theta + w.q * dt
  end

  function w.peak_flicker()
    local m = 0
    for _, v in pairs(w.peak) do if v > m then m = v end end
    return m
  end

  return w
end

return W
