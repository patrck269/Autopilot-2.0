-- Desktop closed loop: the real telemetry, autopilot and actuator modules
-- flying tests/world.lua at 20 ticks per second, with one controller write
-- per tick (setTargetSpeed runs on the server thread in game).
package.path = "./?.lua;" .. package.path

local world = dofile("tests/world.lua")
local telemetry = require("ap2.telemetry")
local autopilot = require("ap2.autopilot")
local actuators = require("ap2.actuators")

local P = {}
P.rho = world.rho

-- opts: world overrides (see world.DEFAULTS, plus y, psi, rpm), saved
-- calibration, cfg(cfg) to edit the config.
function P.new(opts)
  opts = opts or {}
  local cfg = dofile("ap2/config.lua")
  if opts.cfg then opts.cfg(cfg) end
  local w = world.new(cfg.controllers, opts, cfg.gravity)
  w.t, w.tick = 0, 0

  local tel = telemetry.new(cfg, { ship = w.ship, aero = w.aero, clock = function() return w.t end })
  local bank = actuators.new(cfg, w.devices)
  bank:seed()
  local ap = autopilot.new(cfg, opts.saved, tel.rho_at(62), tel)
  local sim = { w = w, p = w.p, cfg = cfg, ap = ap, bank = bank, tel = tel }
  local ctl_every = math.floor(cfg.loop_period / 0.05 + 0.5)

  -- Advance seconds of game time. hook(sim, s) runs after each control step.
  function sim.run(seconds, hook)
    local n = math.floor(seconds / 0.05 + 0.5)
    for _ = 1, n do
      w.tick = w.tick + 1
      w.t = w.tick * 0.05
      if w.tick % ctl_every == 0 then
        local s = tel.read()
        sim.s = s
        local out = ap:step(s, w.t, cfg.loop_period, bank)
        for _, id in ipairs(actuators.ORDER) do bank:set(id, out[id]) end
        if hook then hook(sim, s) end
      end
      -- The flight computer polls the stressometer every 0.5 s.
      if w.tick % 10 == 0 and bank.set_stress then
        local su, cap = w.stress()
        if su then bank:set_stress(su, cap, w.t) end
      end
      bank:service(w.t)
      w.step(0.05)
    end
  end

  function sim.cmd(c, v)
    local ok, msg = ap:command({ c = c, v = v }, sim.s or tel.read())
    assert(ok, msg)
    if c == "mode" and v == "KILL" then bank:kill() end
  end

  sim.peak_flicker = w.peak_flicker
  sim.s = tel.read()
  return sim
end

return P
