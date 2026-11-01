-- Autopilot 2.0 installer. In game:
--   wget run https://raw.githubusercontent.com/patrck269/Autopilot-2.0/master/install.lua <flight|command|both> [commit]
-- Installs the latest commit (or the one given), pinned by SHA so every file
-- comes from the same revision, sets the role in ap2/config.lua, and reboots.
local REPO = "patrck269/Autopilot-2.0"
local FILES = {
  "startup.lua",
  "ap2/actuators.lua",
  "ap2/altitude.lua",
  "ap2/autopilot.lua",
  "ap2/canvas.lua",
  "ap2/checks.lua",
  "ap2/cmdline.lua",
  "ap2/command.lua",
  "ap2/config.lua",
  "ap2/flight.lua",
  "ap2/heading.lua",
  "ap2/link.lua",
  "ap2/mfd.lua",
  "ap2/motion.lua",
  "ap2/pages.lua",
  "ap2/rls.lua",
  "ap2/speed.lua",
  "ap2/store.lua",
  "ap2/telemetry.lua",
  "ap2/util.lua",
}
local ROLES = { flight = true, command = true, both = true }

local role, commit = ...

if not http then
  printError("HTTP is disabled in the ComputerCraft server config.")
  return
end

if not ROLES[role] then
  print("Role for this computer: flight, command or both?")
  write("> ")
  role = read()
  if not ROLES[role] then
    printError("Unknown role '" .. tostring(role) .. "'.")
    return
  end
end

local function get(url)
  local h, err = http.get(url, nil, true)
  if not h then return nil, err end
  local body = h.readAll()
  h.close()
  return body
end

if not commit then
  write("Finding latest version... ")
  local body, err = get("https://api.github.com/repos/" .. REPO .. "/commits/master")
  local info = body and textutils.unserialiseJSON(body)
  commit = info and info.sha
  if not commit then
    print("failed")
    printError(tostring(err or "no commit in the GitHub reply"))
    return
  end
  print(commit:sub(1, 7))
end

-- Download everything before touching the disk, so a failed download leaves
-- the current install as it was.
local base = "https://raw.githubusercontent.com/" .. REPO .. "/" .. commit .. "/"
local got = {}
for i, file in ipairs(FILES) do
  write(string.format("[%2d/%d] %s ", i, #FILES, file))
  local body, err = get(base .. file)
  if not body then
    print("failed")
    printError(tostring(err))
    printError("Nothing was changed.")
    return
  end
  got[file] = body
  print("ok")
end

local cfg, n = got["ap2/config.lua"]:gsub('role = "%a+",', 'role = "' .. role .. '",', 1)
if n ~= 1 then
  printError("Could not set the role in ap2/config.lua. Nothing was changed.")
  return
end
got["ap2/config.lua"] = cfg

local function backup(path)
  if fs.exists(path) then
    local old = fs.open(path, "rb")
    local text = old.readAll()
    old.close()
    if text ~= got[path:sub(2)] then
      if fs.exists(path .. ".bak") then fs.delete(path .. ".bak") end
      fs.copy(path, path .. ".bak")
      print("Kept the old " .. path:sub(2) .. " as " .. path:sub(2) .. ".bak")
    end
  end
end
backup("/startup.lua")
backup("/ap2/config.lua")

for _, file in ipairs(FILES) do
  local path = "/" .. file
  fs.makeDir(fs.getDir(path))
  local h = fs.open(path, "wb")
  h.write(got[file])
  h.close()
end
local v = fs.open("/ap2/VERSION", "w")
v.write(commit .. "\n")
v.close()

print("Autopilot 2.0 " .. commit:sub(1, 7) .. " installed as '" .. role .. "'.")
print("Peripheral names are in ap2/config.lua.")
print("Rebooting in 3 seconds...")
sleep(3)
os.reboot()
