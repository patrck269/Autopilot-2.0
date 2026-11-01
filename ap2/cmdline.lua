-- Typed commands, shared by the flight and command center terminals.
local M = {}

M.HELP = {
  "alt <Y>     target altitude (world Y)",
  "brg <deg>   target bearing (0=N 90=E)",
  "spd <m/s>   target speed (0-35)",
  "hover       hold altitude, stop, hold heading",
  "cruise      altitude + bearing + speed",
  "stby        autopilot off (lift left as is)",
  "kill        cut every controller to 0",
  "debug       every prop at +15 RPM (drops!)",
  "sync        targets = current alt/heading",
  "recal       forget the hover calibration",
}

local MODES = { hover = "HOVER", h = "HOVER", cruise = "CRUISE", c = "CRUISE", stby = "STBY", standby = "STBY", off = "STBY", kill = "KILL", debug = "DEBUG" }
local VALUES = { alt = "alt", a = "alt", y = "alt", brg = "brg", b = "brg", hdg = "brg", spd = "spd", s = "spd", speed = "spd" }

-- Returns a command { c =, v = }, or nil and an error, or "help".
function M.parse(line)
  local words = {}
  for w in string.gmatch(string.lower(line or ""), "%S+") do words[#words + 1] = w end
  local head = words[1]
  if head == nil then return nil, "" end
  if head == "help" or head == "?" then return "help" end
  if MODES[head] then return { c = "mode", v = MODES[head] } end
  if head == "sync" or head == "recal" then return { c = head } end
  local c = VALUES[head]
  if c then
    local v = tonumber(words[2])
    if v == nil then return nil, head .. " needs a number" end
    return { c = c, v = v }
  end
  return nil, "unknown command '" .. head .. "' (type help)"
end

-- A one-line text prompt driven by os.pullEvent events, so the caller's event
-- loop never blocks in read().
function M.prompt()
  local p = { text = "" }
  -- Returns the submitted line when Enter is pressed, else nil.
  function p.handle(ev)
    if ev[1] == "char" then
      p.text = p.text .. ev[2]
    elseif ev[1] == "paste" then
      p.text = p.text .. ev[2]
    elseif ev[1] == "key" then
      if ev[2] == keys.backspace then
        p.text = p.text:sub(1, -2)
      elseif ev[2] == keys.enter or ev[2] == keys.numPadEnter then
        local line = p.text
        p.text = ""
        return line
      end
    end
    return nil
  end
  return p
end

return M
