-- Start-up checks: every modem and peripheral the autopilot depends on.
-- Each function returns errors (fatal: the program will not fly or connect)
-- and warnings (shown, but the program runs).
local M = {}

local TYPES = {
  rsc = "Create_RotationSpeedController",
  speedometer = "Create_Speedometer",
  stressometer = "Create_Stressometer",
}

local function has_type(P, name, wanted)
  if P.hasType then
    local ok, yes = pcall(P.hasType, name, wanted)
    if ok and yes ~= nil then return yes end
  end
  local types = { P.getType(name) }
  for _, t in ipairs(types) do
    if t == wanted then return true end
  end
  return false
end

-- Lists attached modems: { wired = {names}, wireless = {names} }
function M.modems(P)
  local found = { wired = {}, wireless = {} }
  for _, name in ipairs(P.getNames()) do
    if has_type(P, name, "modem") then
      local m = P.wrap(name)
      if m and m.isWireless and m.isWireless() then
        found.wireless[#found.wireless + 1] = name
      else
        found.wired[#found.wired + 1] = name
      end
    end
  end
  return found
end

local function device(P, errors, label, name, wanted)
  if not P.isPresent(name) then
    errors[#errors + 1] = "Missing " .. label .. " (" .. name .. ")"
  elseif not has_type(P, name, wanted) then
    errors[#errors + 1] = label .. " (" .. name .. ") is a " .. tostring(P.getType(name)) .. ", expected " .. wanted
  end
end

-- env = { peripheral = P, ship = ship API or nil, aero = aero API or nil }
function M.flight(cfg, env, role)
  local P = env.peripheral
  local errors, warnings = {}, {}
  if env.ship == nil then
    errors[#errors + 1] = "No CC:VS ship API: this computer is not on a Valkyrien Skies ship"
  end
  if env.aero == nil then
    warnings[#warnings + 1] = "No CC:VS aero API: hover calibration will not scale with air density"
  end
  local modems = M.modems(P)
  if #modems.wired == 0 then
    errors[#errors + 1] = "No wired modem: the controllers and gauges are reached over a wired modem"
  end
  if role == "flight" and #modems.wired + #modems.wireless == 0 then
    errors[#errors + 1] = "No modem for the command center link (wired or wireless)"
  end
  local ids = { "thrust", "lift", "bow_port", "bow_stbd", "stern_port", "stern_stbd" }
  for _, id in ipairs(ids) do
    local c = cfg.controllers[id]
    device(P, errors, c.label, c.name, TYPES.rsc)
  end
  device(P, errors, "Speedometer (main engine speed)", cfg.speedometer, TYPES.speedometer)
  device(P, errors, "Stressometer (main engine stress)", cfg.stressometer, TYPES.stressometer)
  return errors, warnings
end

-- Returns errors, warnings, list of monitor names to use.
function M.command(cfg, env, role)
  local P = env.peripheral
  local errors, warnings = {}, {}
  local modems = M.modems(P)
  if role == "command" and #modems.wired + #modems.wireless == 0 then
    errors[#errors + 1] = "No modem for the flight computer link (wired or wireless)"
  end
  local names = {}
  local wanted = cfg.monitors or {}
  if #wanted == 0 then
    for _, name in ipairs(P.getNames()) do
      if has_type(P, name, "monitor") then names[#names + 1] = name end
    end
    table.sort(names)
  else
    for _, name in ipairs(wanted) do
      if P.isPresent(name) and has_type(P, name, "monitor") then
        names[#names + 1] = name
      else
        warnings[#warnings + 1] = "Missing MFD monitor " .. name
      end
    end
  end
  for _, name in ipairs(names) do
    local m = P.wrap(name)
    if m and m.isColor and not m.isColor() then
      warnings[#warnings + 1] = name .. " is not an advanced monitor: no touch or colour"
    end
  end
  if #names == 0 then
    errors[#errors + 1] = "No MFD monitors found"
  end
  return errors, warnings, names
end

return M
