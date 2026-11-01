-- Tiny persistence for learned calibration and MFD layout (textutils format).
local M = {}

function M.load(path)
  if not fs or not fs.exists(path) then return nil end
  local h = fs.open(path, "r")
  if not h then return nil end
  local text = h.readAll()
  h.close()
  local ok, value = pcall(textutils.unserialize, text)
  if ok and type(value) == "table" then return value end
  return nil
end

function M.save(path, value)
  if not fs then return false end
  local h = fs.open(path, "w")
  if not h then return false end
  h.write(textutils.serialize(value))
  h.close()
  return true
end

return M
