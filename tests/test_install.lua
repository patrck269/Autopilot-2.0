-- The installer's FILES list must hold every module startup.lua can load
-- (following require("ap2.x") through all modules), and nothing that is
-- missing from the repo. Run from the repo root: lua tests/test_install.lua
local function read(path)
  local h = io.open(path, "rb")
  if not h then return nil end
  local text = h:read("a")
  h:close()
  return text
end

local listed = {}
local block = read("install.lua"):match("local FILES = (%b{})")
for file in block:gmatch('"([^"]+)"') do listed[file] = true end

local failures = 0
for file in pairs(listed) do
  if not read(file) then
    failures = failures + 1
    print("FAIL  install.lua lists " .. file .. ", which is not in the repo")
  end
end

local seen, queue = {}, { "startup.lua" }
while #queue > 0 do
  local file = table.remove(queue)
  if not seen[file] then
    seen[file] = true
    if not listed[file] then
      failures = failures + 1
      print("FAIL  " .. file .. " is loaded at run time but install.lua does not download it")
    end
    for mod in (read(file) or ""):gmatch('require%("(ap2%.[%w_]+)"%)') do
      queue[#queue + 1] = mod:gsub("%.", "/") .. ".lua"
    end
  end
end

print(failures == 0 and "INSTALL LIST OK" or (failures .. " INSTALL LIST PROBLEM(S)"))
os.exit(failures == 0 and 0 or 1)
