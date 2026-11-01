-- Closed-loop flight checks against tests/plant.lua. Run from the repo root:
--   lua tests/test_flight.lua
package.path = "./?.lua;" .. package.path
local plant = require("tests.plant")
local util = require("ap2.util")

-- Station keeping holds inside deliberate bands (fewer controller writes):
-- altitude within hold_release, heading within the mode's deadzone, plus about
-- half a unit for actuator lag. Clockwork props take ~1.6 s to spin from 0 to
-- 1000 RPM (world.lua models that), which costs a little more at speed.
local CFG = dofile("ap2/config.lua")
local ALT_TOL = CFG.altitude.hold_release + 0.5
local HDG_HOVER = CFG.heading.hover_deadzone + 0.5
local HDG_CRUISE = 4

local failures = 0
local function check(cond, label, detail)
  if cond then
    print("  ok    " .. label)
  else
    failures = failures + 1
    print("  FAIL  " .. label .. (detail and ("  (" .. detail .. ")") or ""))
  end
end

local function hover_rpm_true(sim, y)
  return sim.p.hover_rpm_sea * math.sqrt(plant.rho(62) / plant.rho(y))
end

-- Track altitude behaviour from a moment on.
local function watch()
  local m = { max_y = -1e9, min_y = 1e9, max_vy = 0, reached = nil }
  m.hook = function(sim, s)
    if s.y > m.max_y then m.max_y = s.y end
    if s.y < m.min_y then m.min_y = s.y end
    if math.abs(s.vy) > m.max_vy then m.max_vy = math.abs(s.vy) end
  end
  return m
end

print("1. Ground start, no calibration, climb to 150 and hold")
do
  -- Real hover is 900 RPM at sea level; the untrained guess is 430.
  local sim = plant.new({ hover_rpm_sea = 900 })
  sim.cmd("alt", 150)
  sim.cmd("mode", "HOVER")
  local reached
  sim.run(120, function(s2, s)
    if not reached and math.abs(s.y - 150) < 1 then reached = s2.w.t end
  end)
  check(reached ~= nil and reached < 80, "reaches 150 +-1", "t=" .. tostring(reached))
  local m = watch()
  sim.run(60, m.hook)
  check(m.max_y - 150 < ALT_TOL and 150 - m.min_y < ALT_TOL, "holds 150 within " .. ALT_TOL .. " for 60 s",
    string.format("min %.2f max %.2f", m.min_y, m.max_y))
  local hov = hover_rpm_true(sim, 150)
  local learned = sim.ap.alt:hover_rpm(plant.rho(150))
  check(math.abs(learned - hov) / hov < 0.03, "learned hover RPM within 3%",
    string.format("learned %.1f true %.1f", learned, hov))
  check(sim.peak_flicker() <= 60, "flicker score stays <= 60", "peak " .. sim.peak_flicker())
  print(string.format("     liftoff->150 in %.1f s, lift %d RPM, writes %d", reached or -1, sim.w.cmd.lift, sim.w.writes))

  print("2. Same flight: climb to 400 (thinner air) then descend to 90")
  sim.cmd("alt", 400)
  local m2 = watch()
  local t0 = sim.w.t
  local r400
  sim.run(150, function(s2, s)
    m2.hook(s2, s)
    if not r400 and math.abs(s.y - 400) < 1 then r400 = s2.w.t - t0 end
  end)
  check(r400 ~= nil, "reaches 400", "t=" .. tostring(r400))
  check(m2.max_y < 403, "overshoot under 3 blocks", string.format("max %.2f", m2.max_y))
  check(math.abs(sim.w.y - 400) < 0.8, "holds 400", string.format("y %.2f", sim.w.y))
  local hov4 = hover_rpm_true(sim, 400)
  check(math.abs(sim.w.cmd.lift - hov4) / hov4 < 0.04, "lift RPM at 400 near true hover",
    string.format("cmd %d true %.1f", sim.w.cmd.lift, hov4))
  print(string.format("     150->400 in %.1f s, max climb %.2f m/s", r400 or -1, m2.max_vy))

  sim.cmd("alt", 90)
  local m3 = watch()
  sim.run(150, m3.hook)
  check(m3.min_y > 87, "descent undershoot under 3 blocks", string.format("min %.2f", m3.min_y))
  check(math.abs(sim.w.y - 90) < 0.8, "holds 90", string.format("y %.2f", sim.w.y))
  check(sim.peak_flicker() <= 60, "flicker score stays <= 60", "peak " .. sim.peak_flicker())
end

print("3. Heavy hull (hover 2200 RPM) from the ground")
do
  local sim = plant.new({ hover_rpm_sea = 2200 })
  sim.cmd("alt", 120)
  sim.cmd("mode", "HOVER")
  sim.run(150)
  check(math.abs(sim.w.y - 120) < 1, "reaches and holds 120", string.format("y %.2f", sim.w.y))
end

print("4. Light hull (hover 300 RPM), calibration guess too high")
do
  local sim = plant.new({ hover_rpm_sea = 300 })
  sim.cmd("alt", 100)
  sim.cmd("mode", "HOVER")
  local m = watch()
  sim.run(120, m.hook)
  check(math.abs(sim.w.y - 100) < 1, "reaches and holds 100", string.format("y %.2f", sim.w.y))
  check(m.max_y < 106, "overshoot under 6 blocks on the first climb", string.format("max %.2f", m.max_y))
end

print("5. Cruise: 0 -> 30 m/s within 15 s, hold bearing and altitude")
do
  -- Off-centre thrust line: at full rear thrust it needs about half of what
  -- one RCS pair can give at 1000 RPM.
  local sim = plant.new({ hover_rpm_sea = 900, y = 200, rpm = { lift = 900 }, misalign = 0.08, psi = 10,
    saved = { k = 10 / (plant.rho(62) * 900 * 900) } })
  sim.cmd("alt", 200)
  sim.cmd("brg", 90)
  sim.cmd("mode", "HOVER")
  sim.run(40)
  check(math.abs(util.wrap180(sim.s.hdg - 90)) < HDG_HOVER, "hover turns to bearing 90", string.format("hdg %.2f", sim.s.hdg))
  sim.cmd("spd", 30)
  sim.cmd("mode", "CRUISE")
  local t0 = sim.w.t
  local reach
  local max_hdg_err, max_alt_err, max_v = 0, 0, 0
  sim.run(40, function(s2, s)
    if not reach and math.abs(s.v_fwd - 30) < 1 then reach = s2.w.t - t0 end
    if s2.w.t - t0 > 3 then
      max_hdg_err = math.max(max_hdg_err, math.abs(util.wrap180(s.hdg - 90)))
    end
    max_alt_err = math.max(max_alt_err, math.abs(s.y - 200))
    max_v = math.max(max_v, s.v_fwd)
  end)
  check(reach ~= nil and reach <= 15, "30 m/s (+-1) within 15 s", "t=" .. tostring(reach))
  check(max_v < 31.5, "speed overshoot under 1.5 m/s", string.format("max %.2f", max_v))
  check(max_hdg_err < HDG_CRUISE, "heading within " .. HDG_CRUISE .. " deg at speed with thrust misalignment",
    string.format("%.2f", max_hdg_err))
  check(max_alt_err < 2, "altitude within 2 blocks", string.format("%.2f", max_alt_err))

  print("6. Cap at 35 m/s")
  local ok = sim.ap:command({ c = "spd", v = 50 }, sim.s)
  check(ok and sim.ap.tgt.spd == 35, "target 50 clamps to 35", tostring(sim.ap.tgt.spd))
  local max_v2 = 0
  sim.run(40, function(_, s) max_v2 = math.max(max_v2, s.v_fwd) end)
  check(max_v2 <= 35.5, "speed never above 35.5", string.format("%.2f", max_v2))
  check(math.abs(sim.s.v_fwd - 35) < 1, "settles at 35", string.format("%.2f", sim.s.v_fwd))

  print("7. Slow to 0 with reverse thrust")
  sim.cmd("spd", 0)
  local reverse, t0b, stopped = false, sim.w.t, nil
  sim.run(40, function(s2, s)
    if s2.w.cmd.thrust < 0 then reverse = true end
    if not stopped and math.abs(s.v_fwd) < 0.5 then stopped = s2.w.t - t0b end
  end)
  check(reverse, "rear thrusters ran in reverse")
  check(stopped ~= nil and stopped <= 15, "stopped (<0.5 m/s) within 15 s", "t=" .. tostring(stopped))
  check(sim.peak_flicker() <= 60, "flicker score stays <= 60", "peak " .. sim.peak_flicker())
end

print("8. Nose dips under acceleration: thrust held back")
do
  -- Unprotected, full thrust here would pitch the nose about 8 deg down; cruise
  -- drag alone about 3 deg.
  local sim = plant.new({ hover_rpm_sea = 900, y = 200, rpm = { lift = 900 }, dip = 1.0,
    saved = { k = 10 / (plant.rho(62) * 900 * 900) } })
  sim.cmd("alt", 200)
  sim.cmd("spd", 30)
  sim.cmd("mode", "CRUISE")
  local min_pitch, held, reach = 0, false, nil
  sim.run(60, function(s2, s)
    min_pitch = math.min(min_pitch, s.pitch)
    if s2.ap.spd.pitch_hold then held = true end
    if not reach and math.abs(s.v_fwd - 30) < 1 then reach = s2.w.t end
  end)
  check(held, "pitch protection engaged")
  local limit = sim.cfg.speed.pitch_limit
  check(min_pitch > -(limit + 2), "nose dip kept within limit + 2 deg", string.format("%.2f", min_pitch))
  check(reach ~= nil, "still reaches 30 m/s", "t=" .. tostring(reach))
  print(string.format("     reached 30 m/s at %.1f s with the nose limited", reach or -1))
end

print("9. KILL zeroes every controller at once")
do
  local sim = plant.new({ hover_rpm_sea = 900, y = 200, rpm = { lift = 900 },
    saved = { k = 10 / (plant.rho(62) * 900 * 900) } })
  sim.cmd("spd", 20)
  sim.cmd("mode", "CRUISE")
  sim.run(10)
  sim.cmd("mode", "KILL")
  sim.run(0.4)
  local all_zero = true
  for _, v in pairs(sim.w.cmd) do if v ~= 0 then all_zero = false end end
  check(all_zero, "all controllers at 0 within 0.4 s")
end

print("10. Noisy velocity readings (+-0.15 m/s): calibrate and hold")
do
  math.randomseed(7)
  local sim = plant.new({ hover_rpm_sea = 1200, noise = 0.15 })
  sim.cmd("alt", 140)
  sim.cmd("mode", "HOVER")
  sim.run(100)
  local m = watch()
  sim.run(60, m.hook)
  check(m.max_y - 140 < ALT_TOL and 140 - m.min_y < ALT_TOL, "holds 140 within " .. ALT_TOL,
    string.format("min %.2f max %.2f", m.min_y, m.max_y))
  check(sim.peak_flicker() <= 60, "flicker score stays <= 60", "peak " .. sim.peak_flicker())
end

-- Hover with the bearing 40 deg off. Returns max |yaw rate|, final heading
-- error, and the largest heading error over the last 20 s.
local function turn_in_hover(opts)
  opts.y = 200
  opts.rpm = { lift = 900 }
  opts.psi = 0
  opts.saved = { k = 10 / (plant.rho(62) * 900 * 900) }
  local sim = plant.new(opts)
  sim.cmd("alt", 200)
  sim.cmd("brg", 40)
  sim.cmd("mode", "HOVER")
  local max_rate, late_err = 0, 0
  sim.run(opts.seconds or 90, function(s2, s)
    max_rate = math.max(max_rate, math.abs(s.yaw_rate))
    if s2.w.t > (opts.seconds or 90) - 20 then
      late_err = math.max(late_err, math.abs(util.wrap180(s.hdg - 40)))
    end
  end)
  return sim, max_rate, math.abs(util.wrap180(sim.s.hdg - 40)), late_err
end

-- Longest run of ticks with an RCS corner pulling without its diagonal
-- partner (with the helm's stabilisation that strafes the hull instead of
-- turning it). Pair writes go back to back, so at most one tick.
local function lonely_run(sim)
  return sim.w.lonely_max
end

print("11. RCS pushes on positive RPM (turn direction backwards): no spin")
do
  local sim, max_rate, err, late = turn_in_hover({ rcs_reverse = true })
  check(max_rate < 10, "yaw rate stays under 10 deg/s", string.format("max %.1f", max_rate))
  check(late < HDG_HOVER, "settles on the bearing", string.format("err %.2f, late max %.2f", err, late))
  check(lonely_run(sim) <= 1, "diagonal pairs run together", lonely_run(sim) .. " ticks alone at most")
end

print("12. Very strong RCS: no limit cycle")
do
  local sim, max_rate, err, late = turn_in_hover({ yaw_accel = 60 })
  check(max_rate < 10, "yaw rate stays under 10 deg/s", string.format("max %.1f", max_rate))
  check(late < HDG_HOVER, "settles on the bearing", string.format("err %.2f, late max %.2f", err, late))
  check(lonely_run(sim) <= 1, "diagonal pairs run together", lonely_run(sim) .. " ticks alone at most")
end

print("13. Weak RCS: still turns")
do
  local sim, max_rate, err, late = turn_in_hover({ yaw_accel = 1.0, seconds = 150 })
  check(late < HDG_HOVER, "settles on the bearing", string.format("err %.2f, late max %.2f", err, late))
  check(lonely_run(sim) <= 1, "diagonal pairs run together", lonely_run(sim) .. " ticks alone at most")
end

print("14. RCS writes never exceed 1000 RPM, even saturated")
do
  local sim = plant.new({ y = 200, rpm = { lift = 900 }, psi = 0, yaw_accel = 0.3,
    saved = { k = 10 / (plant.rho(62) * 900 * 900), yaw_b = 0.2 } })
  sim.cmd("alt", 200)
  sim.cmd("brg", 170)
  sim.cmd("mode", "HOVER")
  local top = 0
  sim.run(20, function(s2)
    for _, id in ipairs({ "bow_port", "bow_stbd", "stern_port", "stern_stbd" }) do
      top = math.max(top, math.abs(s2.w.cmd[id]))
    end
  end)
  check(top == 1000, "RCS reaches but does not pass 1000 RPM", "max " .. top)
end

print("15. DEBUG: every controller at +15 RPM on the block, calibration untouched")
do
  local sim = plant.new({ y = 200, rpm = { lift = 900 },
    saved = { k = 10 / (plant.rho(62) * 900 * 900) },
    cfg = function(cfg) cfg.controllers.bow_port.sign = -1 end })
  sim.cmd("alt", 200)
  sim.cmd("mode", "HOVER")
  sim.run(5)
  local k = sim.ap.alt.k
  sim.cmd("mode", "DEBUG")
  sim.run(6) -- six writes through the shared flicker budget
  local all = true
  for id, v in pairs(sim.w.cmd) do
    if v ~= 15 then all = false; print("     " .. id .. " = " .. v) end
  end
  check(all, "all six blocks at +15 (bow_port has sign -1 in config)")
  sim.run(5) -- the hull drops on 15 RPM elevation
  check(sim.ap.alt.k == k, "hover calibration unchanged by the drop")
  sim.cmd("mode", "HOVER")
  sim.run(40)
  check(math.abs(sim.w.y - 200) < 1.5, "HOVER afterwards climbs back to 200", string.format("y %.2f", sim.w.y))
end

-- Climb to 150, turn, cruise, turn and stop on an engine of the given
-- capacity (SU). Hovering at 150 costs about 115000 SU. Returns the sim and
-- the list of destroyed controllers.
local function stressed_flight(capacity, opts)
  opts = opts or {}
  opts.stress = { capacity = capacity, lift = 120, thrust = 60, rcs = 30 }
  local sim = plant.new(opts)
  sim.cmd("alt", 150)
  sim.cmd("brg", 60)
  sim.cmd("mode", "HOVER")
  sim.run(90)
  sim.cmd("spd", 25)
  sim.cmd("mode", "CRUISE")
  sim.run(60)
  sim.cmd("brg", 200)
  sim.cmd("spd", 0)
  sim.run(60)
  local broken = {}
  for id in pairs(sim.w.destroyed) do broken[#broken + 1] = id end
  table.sort(broken)
  return sim, broken
end

print("16. Engine with room for the hover (77%): flies, nothing destroyed")
do
  local sim, broken = stressed_flight(150000)
  check(#broken == 0, "no rotation speed controller destroyed", table.concat(broken, ", "))
  check(sim.w.peak_flicker() <= 70, "worst flicker score at or under 70 (Create breaks at 128)",
    "peak " .. sim.w.peak_flicker())
  check(sim.w.toggles == 0, "engine never overstressed", sim.w.toggles .. " switches")
  check(math.abs(sim.w.y - 150) < 2, "holds altitude 150", string.format("y %.2f", sim.w.y))
end

print("17. Engine too small to hover at 150: held back, nothing destroyed")
do
  for _, start in ipairs({ "ground", "air" }) do
    local opts = {}
    if start == "air" then
      opts = { y = 150, rpm = { lift = 900 }, saved = { k = 10 / (plant.rho(62) * 900 * 900) } }
    end
    local sim, broken = stressed_flight(118000, opts)
    check(#broken == 0, start .. " start: no controller destroyed", table.concat(broken, ", "))
    check(sim.w.peak_flicker() <= 70, start .. " start: worst flicker at or under 70", "peak " .. sim.w.peak_flicker())
    local alerts = table.concat(sim.bank:alerts(sim.w.t), " | ")
    check(alerts:find("STRESS LIMIT", 1, true) ~= nil or alerts:find("OVERSTRESSED", 1, true) ~= nil,
      start .. " start: the stress limit is reported", alerts)
  end
end

print("18. Another computer changes a controller: left alone, reported")
do
  local sim = plant.new({ y = 200, rpm = { lift = 900 }, saved = { k = 10 / (plant.rho(62) * 900 * 900) },
    stress = { capacity = 300000, lift = 120, thrust = 60, rcs = 30 } })
  sim.cmd("alt", 200)
  sim.cmd("brg", 90)
  sim.cmd("mode", "HOVER")
  sim.run(5)
  sim.w.devices.bow_port.setTargetSpeed(321) -- someone else
  local writes_before = sim.w.writes
  local fought = false
  sim.run(3, function(s2)
    if s2.w.cmd.bow_port ~= 321 then fought = true end
  end)
  check(not fought, "the foreign value is not overwritten for the hold time")
  local alerts = table.concat(sim.bank:alerts(sim.w.t), " | ")
  check(alerts:find("CHANGED BY ANOTHER COMPUTER", 1, true) ~= nil, "reported", alerts)
  sim.run(10)
  check(sim.w.cmd.bow_port ~= 321, "control resumes after the hold")
  check(sim.w.writes > writes_before, "other controllers kept flying meanwhile")
end

print("19. Already drifting sideways when HOVER engages: drift cancelled")
for _, case in ipairs({ { "pullers as assumed", {} }, { "RCS acting backwards", { rcs_reverse = true } } }) do
  -- 3 m/s to starboard, hardly any side drag (a VS hull keeps sliding).
  local opts = { y = 150, rpm = { lift = 958 }, psi = 30, v_side = 3, side_drag = 0.02,
    saved = { k = 10 / (plant.rho(62) * 900 * 900) } }
  for k, v in pairs(case[2]) do opts[k] = v end
  local sim = plant.new(opts)
  sim.cmd("alt", 150)
  sim.cmd("brg", 30)
  sim.cmd("mode", "HOVER")
  local stopped, worst_hdg = nil, 0
  sim.run(90, function(s2, s)
    if not stopped and math.abs(s.v_side) < 0.3 then stopped = s2.w.t end
    worst_hdg = math.max(worst_hdg, math.abs(util.wrap180(s.hdg - 30)))
  end)
  check(stopped ~= nil, case[1] .. ": drift brought under 0.3 m/s", "t=" .. tostring(stopped))
  check(math.abs(sim.s.v_side) < 0.5, case[1] .. ": and kept there", string.format("%.2f m/s", sim.s.v_side))
  check(worst_hdg < 6, case[1] .. ": heading held while braking", string.format("worst %.2f deg", worst_hdg))
  check(sim.w.peak_flicker() <= 60, case[1] .. ": flicker at or under 60", "peak " .. sim.w.peak_flicker())
  print(string.format("     %s: drift stopped after %.1f s, sideways authority %.3f", case[1], stopped or -1,
    select(2, sim.ap.motion:strafe_drift(sim.s))))
end

print("20. frame.forward set wrong: bow axis learned, drift cancelled, cruise works")
for _, case in ipairs({ { "90 deg off", { x = 0, y = 0, z = -1 } }, { "180 deg off", { x = -1, y = 0, z = 0 } } }) do
  -- The hull's real bow is +X; the config says otherwise. Drifting 3 m/s at start.
  local sim = plant.new({ y = 150, rpm = { lift = 958 }, psi = 30, v_side = 3, side_drag = 0.02,
    saved = { k = 10 / (plant.rho(62) * 900 * 900) },
    cfg = function(cfg) cfg.frame.forward = case[2] end })
  sim.cmd("alt", 150)
  sim.cmd("mode", "HOVER")
  local switches, last = 0, 0
  local function true_side()
    local a = math.rad(sim.w.psi)
    return sim.w.vx * math.cos(a) + sim.w.vz * math.sin(a)
  end
  sim.run(90, function(s2)
    local stb = s2.w.cmd.bow_stbd > 0 and s2.w.cmd.stern_stbd > 0
    local prt = s2.w.cmd.bow_port > 0 and s2.w.cmd.stern_port > 0
    local side = stb and 1 or (prt and -1 or 0)
    if side ~= 0 and side ~= last then switches = switches + 1; last = side end
  end)
  check(math.abs(true_side()) < 0.5, case[1] .. ": real sideways drift cancelled", string.format("%.2f m/s", true_side()))
  check(switches <= 4, case[1] .. ": brake does not flap port/starboard", switches .. " side changes")
  -- The bow is learned from the rear thrusters' push, so it may need a cruise.
  local brg = util.round(sim.s.hdg) % 360
  sim.cmd("brg", brg)
  sim.cmd("spd", 15)
  sim.cmd("mode", "CRUISE")
  local top = 0
  sim.run(40, function(s2) top = math.max(top, math.sqrt(s2.w.vx ^ 2 + s2.w.vz ^ 2)) end)
  check(util.dot(sim.tel.forward, { x = 1, y = 0, z = 0 }) > 0.99, case[1] .. ": bow axis learned as +X",
    util.axis_name(sim.tel.forward))
  check(math.abs(util.wrap180(sim.s.hdg - sim.w.psi)) < 1, case[1] .. ": heading reads true",
    string.format("reads %.1f, true %.1f", sim.s.hdg, sim.w.psi))
  local a = math.rad(sim.w.psi)
  local true_fwd = sim.w.vx * math.sin(a) - sim.w.vz * math.cos(a)
  check(math.abs(true_fwd - 15) < 1.5, case[1] .. ": cruises at 15 m/s along the real bow", string.format("%.2f m/s", true_fwd))
  check(top < 18, case[1] .. ": no runaway before the axis is learned", string.format("top %.1f m/s", top))
  check(sim.peak_flicker() <= 60, case[1] .. ": flicker at or under 60", "peak " .. sim.peak_flicker())
end

print(failures == 0 and "ALL FLIGHT CHECKS PASSED" or (failures .. " FLIGHT CHECK(S) FAILED"))
os.exit(failures == 0 and 0 or 1)
