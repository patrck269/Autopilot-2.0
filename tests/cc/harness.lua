-- Runs inside CraftOS-PC (see tests/cc/run.py). Installs fake Create
-- controllers, gauges, a wired modem and four monitors into the real CC
-- peripheral API, flies tests/world.lua in a coroutine, starts the real
-- /ap2_start.lua (the repo's startup.lua) and drives the MFDs through
-- monitor_touch events and the terminal through char/key events.
-- Writes /report.txt and shuts the computer down.
package.path = "/?.lua;/?/init.lua;" .. package.path

local scenario = "both"
if fs.exists("/scenario.txt") then
  local h = fs.open("/scenario.txt", "r")
  scenario = h.readAll():gsub("%s", "")
  h.close()
end

local cfg = dofile("/ap2/config.lua")
local W = dofile("/tests/world.lua")
local mfd = require("ap2.mfd")
local w = W.new(cfg.controllers, { hover_rpm_sea = 900 }, cfg.gravity)
_G.ship = w.ship
_G.aero = w.aero

-- Log every controller write with the score after it.
local write_log = {}
for id, dev in pairs(w.devices) do
  local set = dev.setTargetSpeed
  dev.setTargetSpeed = function(v)
    set(v)
    write_log[#write_log + 1] = { os.clock(), id, v, w.flick[id] }
  end
end

local report = {}
local failures = 0
local function check(cond, label, detail)
  if not cond then failures = failures + 1 end
  report[#report + 1] = (cond and "ok    " or "FAIL  ") .. label .. (detail and ("  (" .. tostring(detail) .. ")") or "")
end

-- Fake monitor: records text written by blit/write.
local monitors = {}
local function fake_monitor(name, mw, mh)
  local m = { rows = {}, x = 1, y = 1 }
  local function blank()
    for y = 1, mh do m.rows[y] = string.rep(" ", mw) end
  end
  blank()
  local function put(text)
    local row = m.rows[m.y]
    if not row then return end
    local before = row:sub(1, m.x - 1)
    local after = row:sub(m.x + #text)
    m.rows[m.y] = (before .. text .. after):sub(1, mw)
    m.x = m.x + #text
  end
  m.obj = {
    setTextScale = function() end, getTextScale = function() return 0.5 end,
    getSize = function() return mw, mh end,
    setBackgroundColor = function() end, setTextColor = function() end,
    setBackgroundColour = function() end, setTextColour = function() end,
    clear = blank, setCursorBlink = function() end,
    setCursorPos = function(x, y) m.x, m.y = x, y end,
    write = function(t) put(tostring(t)) end,
    blit = function(t) put(t) end,
    isColor = function() return true end, isColour = function() return true end,
  }
  m.w, m.h = mw, mh
  monitors[name] = m
  return m
end
fake_monitor("monitor_4", 57, 24)
fake_monitor("monitor_5", 57, 24)
fake_monitor("monitor_6", 36, 24)
fake_monitor("monitor_7", 36, 24)

local function screen(name)
  return table.concat(monitors[name].rows, "\n")
end
local function shows(name, text)
  return screen(name):find(text, 1, true) ~= nil
end

local transmitted = 0
local opened = {}
local modem = {
  isWireless = function() return false end,
  open = function(ch) opened[ch] = true end,
  close = function(ch) opened[ch] = nil end,
  isOpen = function(ch) return opened[ch] == true end,
  closeAll = function() opened = {} end,
  transmit = function() transmitted = transmitted + 1 end,
  getNamesRemote = function() return {} end,
}

local fake = {}
for id, c in pairs(cfg.controllers) do
  fake[c.name] = { "Create_RotationSpeedController", w.devices[id] }
end
fake[cfg.speedometer] = { "Create_Speedometer", { getSpeed = function() return 256 end } }
fake[cfg.stressometer] = { "Create_Stressometer", {
  getStress = function() return 120000 end, getStressCapacity = function() return 479231 end } }
fake.back = { "modem", modem }
for name, m in pairs(monitors) do fake[name] = { "monitor", m.obj } end

local held_back = {}
if scenario == "missing" then
  held_back[cfg.controllers.lift.name] = true
  held_back.back = true -- the wired modem
end

local function visible(name)
  return fake[name] ~= nil and not held_back[name]
end
peripheral.getNames = function()
  local list = {}
  for name in pairs(fake) do if visible(name) then list[#list + 1] = name end end
  table.sort(list)
  return list
end
peripheral.isPresent = function(name) return visible(name) end
peripheral.getType = function(name)
  if visible(name) then return fake[name][1] end
  return nil
end
peripheral.hasType = function(name, t)
  if not visible(name) then return nil end
  return fake[name][1] == t
end
peripheral.wrap = function(name)
  if visible(name) then return fake[name][2] end
  return nil
end
peripheral.call = function(name, method, ...)
  if not visible(name) then error("No peripheral attached: " .. tostring(name)) end
  return fake[name][2][method](...)
end

-- Press the centre of an OSB, or a content point, on a monitor.
local function press(name, pos)
  local m = monitors[name]
  local r = mfd.layout(m.w, m.h).osb[pos]
  os.queueEvent("monitor_touch", name, r.x + math.floor(r.w / 2), r.y + math.floor(r.h / 2))
  sleep(0.3)
end

local function type_line(text)
  for i = 1, #text do
    os.queueEvent("char", text:sub(i, i))
  end
  os.queueEvent("key", keys.enter, false)
  sleep(0.5)
end

-- Keypad digits on the UFC page (same grid as ap2/pages.lua).
local KEYS = { { "7", "8", "9" }, { "4", "5", "6" }, { "1", "2", "3" }, { "-", "0", "." } }
local function keypad(name, digits)
  local m = monitors[name]
  local a = mfd.layout(m.w, m.h).content
  local top = a.y + 3
  local kh = math.max(1, math.floor((a.y + a.h - top) / 4) - 1)
  local kw = math.max(3, math.floor(a.w / 3) - 1)
  for i = 1, #digits do
    local d = digits:sub(i, i)
    for r, row in ipairs(KEYS) do
      for col, k in ipairs(row) do
        if k == d then
          os.queueEvent("monitor_touch", name, a.x + (col - 1) * (kw + 1) + 1, top + (r - 1) * (kh + 1))
          sleep(0.2)
        end
      end
    end
  end
end

-- In game os.clock() counts server ticks, so Create's flicker decay and the
-- autopilot's pacing share one clock. CraftOS-PC's os.clock is wall time, so
-- the world steps once per 0.05 s of os.clock (catching up after a stall)
-- instead of once per wake-up.
local steps, start_clock = 0, os.clock()
local function physics()
  while true do
    sleep(0.05)
    local due = math.floor((os.clock() - start_clock) / 0.05)
    while steps < due do
      w.step(0.05)
      steps = steps + 1
    end
  end
end

local function dump(title)
  report[#report + 1] = "---- " .. title
  for _, name in ipairs({ "monitor_4", "monitor_5", "monitor_6", "monitor_7" }) do
    report[#report + 1] = "[" .. name .. "]"
    report[#report + 1] = screen(name)
  end
end

local function both_script()
  sleep(2)
  check(shows("monitor_4", "PFD") and shows("monitor_4", "MENU") and shows("monitor_4", "SWAP"),
    "monitor_4 shows PFD with MENU/SWAP buttons")
  check(shows("monitor_5", "HSI"), "monitor_5 shows HSI")
  check(shows("monitor_6", "ENGINE"), "monitor_6 shows ENG")
  check(shows("monitor_7", "SYSTEM"), "monitor_7 shows SYS")
  check(shows("monitor_6", "MAIN ENGINE 256 RPM"), "speedometer reading on ENG")
  dump("boot")

  press("monitor_4", "B5") -- SWAP 4 <-> 5
  check(shows("monitor_4", "HSI") and shows("monitor_5", "PFD"), "SWAP exchanges monitor_4 and monitor_5")

  press("monitor_6", "B1") -- MENU
  check(shows("monitor_6", "FORMAT MENU"), "MENU opens the format menu")
  press("monitor_6", "T1") -- first page in the list: PFD (shown on monitor_5)
  check(shows("monitor_6", "PFD") and shows("monitor_5", "ENGINE"),
    "choosing PFD moves it to monitor_6 and sends ENG to monitor_5")

  press("monitor_4", "B3") -- monitor_4 now holds display 2's formats: HSI, AP, CAL
  check(shows("monitor_4", "AUTOPILOT"), "format button B3 shows AP on monitor_4")

  press("monitor_7", "B3") -- SYS, UFC, CAL -> UFC
  check(shows("monitor_7", "KEYPAD"), "monitor_7 shows the keypad")
  keypad("monitor_7", "120")
  check(shows("monitor_7", "> 120_"), "keypad scratchpad reads 120", screen("monitor_7"):match("> [^\n]*"))
  press("monitor_7", "L1") -- ALT SET
  sleep(0.6)
  check(shows("monitor_4", "TGT   120"), "ALT SET from the keypad reaches the flight computer")

  press("monitor_4", "T1") -- HOVR
  sleep(1)
  check(shows("monitor_4", " HOVER "), "HOVR engages HOVER")
  local t0 = os.clock()
  local reached
  while os.clock() - t0 < 70 do
    if math.abs(w.y - 120) < 1 and math.abs(w.vy) < 0.3 then reached = os.clock() - t0 break end
    sleep(0.5)
  end
  check(reached ~= nil, "ship lifts off and holds Y 120", string.format("y %.2f after %s s", w.y, tostring(reached)))
  sleep(5)
  check(math.abs(w.y - 120) < 1, "still at 120 five seconds later", string.format("y %.2f", w.y))

  type_line("brg 90")
  type_line("spd 20")
  check(shows("monitor_4", "TGT   090") and shows("monitor_4", "TGT    20"),
    "terminal commands set bearing 90 and speed 20")
  press("monitor_4", "T2") -- CRZ
  sleep(25)
  local hdg_err = math.abs(((w.psi - 90 + 180) % 360) - 180)
  local speed = math.sqrt(w.vx * w.vx + w.vz * w.vz)
  check(hdg_err < 2, "CRUISE turns to bearing 90", string.format("hdg %.2f", w.psi))
  check(math.abs(speed - 20) < 1.5, "CRUISE holds 20 m/s", string.format("%.2f", speed))
  check(math.abs(w.y - 120) < 2, "CRUISE holds altitude", string.format("y %.2f", w.y))
  dump("cruise")

  press("monitor_7", "B2") -- SYS
  press("monitor_7", "T3") -- DEBUG (arm)
  check(shows("monitor_7", "CONF") and not shows("monitor_4", " DEBUG "), "first DEBUG press only arms")
  press("monitor_7", "T3") -- confirm
  sleep(2)
  local all15 = true
  for _, v in pairs(w.cmd) do if v ~= 15 then all15 = false end end
  check(all15, "confirmed DEBUG puts every controller at +15 RPM")
  check(shows("monitor_4", " DEBUG "), "mode shows DEBUG")
  press("monitor_7", "T5") -- KILL (arm)
  check(shows("monitor_7", "CONF"), "first KILL press arms the confirm")
  press("monitor_7", "T5") -- confirm
  sleep(1)
  local all_zero = true
  for _, v in pairs(w.cmd) do if v ~= 0 then all_zero = false end end
  check(all_zero, "confirmed KILL zeroes every controller")
  check(shows("monitor_4", " KILL "), "mode shows KILL")
  local worst, worst_id = 0, nil
  for id, v in pairs(w.peak) do if v > worst then worst, worst_id = v, id end end
  check(worst <= 60, "flicker score stayed <= 60", tostring(worst) .. " on " .. tostring(worst_id))
  if worst > 60 then
    for _, e in ipairs(write_log) do
      if e[2] == worst_id then report[#report + 1] = string.format("  t=%.2f %s %d score %d", e[1], e[2], e[3], e[4]) end
    end
    report[#report + 1] = string.format("  physics steps %d in %.1f s", steps, os.clock() - start_clock)
  end
  check(fs.exists("/ap2_mfd.dat"), "MFD layout saved")
  dump("after kill")
end

local function missing_script()
  sleep(3)
  local function either(text) return shows("monitor_4", text) or shows("monitor_5", text) end
  check(either("START-UP FAILED"), "monitors show the flight start-up failure")
  check(either("No wired modem"), "the missing wired modem is reported")
  check(either("Missing RSC16 ELEV"), "the missing elevation controller is named")
  check(w.writes == 0, "nothing is written to the controllers", w.writes)
  dump("missing")
  held_back = {}
  os.queueEvent("peripheral", "back")
  os.queueEvent("peripheral", cfg.controllers.lift.name)
  sleep(3)
  check(shows("monitor_4", "PFD") and not shows("monitor_4", "START-UP FAILED"),
    "after attaching it the flight computer starts")
  dump("attached")
end

local function wait_y(target, seconds)
  local t0 = os.clock()
  while os.clock() - t0 < seconds do
    if math.abs(w.y - target) < 1 and math.abs(w.vy) < 0.3 then return os.clock() - t0 end
    sleep(0.5)
  end
  return nil
end

-- Flight computer alone: terminal commands, rednet status out, rednet
-- commands in (and commands for another ship ignored).
local function flight_script()
  sleep(2)
  check(transmitted > 0, "status is broadcast over the wired modem", transmitted)
  check(not shows("monitor_4", "PFD"), "flight role leaves the monitors alone")
  type_line("alt 90")
  type_line("hover")
  local t = wait_y(90, 60)
  check(t ~= nil, "typed 'alt 90' + 'hover' climbs to and holds 90", string.format("y %.2f", w.y))
  local sent = transmitted
  os.queueEvent("rednet_message", 42, { ap2 = "cmd", c = "alt", v = 100, ship = 7, seq = 1 }, cfg.protocol)
  sleep(0.5)
  check(transmitted > sent, "a rednet command gets a reply")
  t = wait_y(100, 40)
  check(t ~= nil, "rednet 'alt 100' from this ship's command center is flown", string.format("y %.2f", w.y))
  os.queueEvent("rednet_message", 43, { ap2 = "cmd", c = "alt", v = 200, ship = 99, seq = 1 }, cfg.protocol)
  sleep(5)
  check(math.abs(w.y - 100) < 1.5, "a command for another ship is ignored", string.format("y %.2f", w.y))
end

local role = scenario == "flight" and "flight" or "both"
local scripts = { both = both_script, missing = missing_script, flight = flight_script }
local ok, err = pcall(parallel.waitForAny,
  physics,
  function() shell.run("/ap2_start.lua", role) end,
  scripts[scenario])
if not ok then
  failures = failures + 1
  report[#report + 1] = "FAIL  harness error: " .. tostring(err)
end
report[#report + 1] = failures == 0 and "ALL CC CHECKS PASSED" or (failures .. " CC CHECK(S) FAILED")
local h = fs.open("/report.txt", "w")
h.write(table.concat(report, "\n"))
h.close()
os.shutdown()
