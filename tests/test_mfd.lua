-- MFD format slots: every display keeps three different formats and every
-- page is shown on at most one display, whatever is pressed or loaded.
-- Run from the repo root: lua tests/test_mfd.lua
package.path = "./?.lua;" .. package.path
colors = setmetatable({}, { __index = function() return 1 end })
local mfd = require("ap2.mfd")

local IDS = { "PFD", "AP", "HSI", "ENG", "CAL", "SYS", "UFC" }
local pages = {}
for _, id in ipairs(IDS) do pages[#pages + 1] = { id = id, title = id, draw = function() end } end
local function displays(n)
  local list = {}
  for i = 1, n do list[i] = { name = "monitor_" .. i, w = 57, h = 24, mon = {} } end
  return list
end

local failures = 0
local function check(m, label)
  local shown = {}
  for _, d in ipairs(m.displays) do
    local s = d.slots
    if s[1] == s[2] or s[1] == s[3] or s[2] == s[3] then
      failures = failures + 1
      print(string.format("FAIL  %s: %s holds %s/%s/%s", label, d.name, s[1], s[2], s[3]))
    end
    local id = m:shown(d)
    if shown[id] then
      failures = failures + 1
      print(string.format("FAIL  %s: %s shown on %s and %s", label, id, shown[id], d.name))
    end
    shown[id] = d.name
  end
end

-- Every page picked from the menu on every display, then every format
-- button, on four- and six-display cockpits.
for _, n in ipairs({ 4, 6 }) do
  local m = mfd.new(displays(n), pages, nil)
  check(m, n .. " displays at start")
  for _, d in ipairs(m.displays) do
    for _, id in ipairs(IDS) do
      m:show(d, id)
      check(m, n .. " displays, " .. id .. " from the menu on " .. d.name)
    end
    for k = 1, 3 do
      m:select(d, k)
      check(m, n .. " displays, format " .. k .. " on " .. d.name)
    end
    m:swap(d)
    check(m, n .. " displays, SWAP on " .. d.name)
  end
end

-- A layout saved by the old menu, with repeated formats, loads clean and
-- keeps the pages that were on screen.
local saved = {
  monitor_1 = { slots = { "HSI", "HSI", "CAL" }, sel = 2 },
  monitor_2 = { slots = { "UFC", "UFC", "UFC" }, sel = 2 },
  monitor_3 = { slots = { "SYS", "ENG", "ENG" }, sel = 2 },
}
local m = mfd.new(displays(3), pages, saved)
check(m, "saved layout with repeats")
local was = { "HSI", "UFC", "ENG" }
for i, d in ipairs(m.displays) do
  if m:shown(d) ~= was[i] then
    failures = failures + 1
    print("FAIL  saved layout: " .. d.name .. " shows " .. m:shown(d) .. ", was " .. was[i])
  end
end

print(failures == 0 and "ALL MFD CHECKS PASSED" or (failures .. " MFD CHECK(S) FAILED"))
os.exit(failures == 0 and 0 or 1)
