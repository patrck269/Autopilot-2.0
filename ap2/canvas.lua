-- Off-screen character buffer for a monitor. Pages draw into it; flush()
-- blits only the rows that changed since the last flush, so redrawing at 5 Hz
-- does not flicker or flood the monitor with peripheral calls.
local M = {}
M.__index = M

local HEX = "0123456789abcdef"

-- colors.* value (1, 2, 4, ... 32768) -> blit hex digit
local function hex(color)
  local i = 0
  while color > 1 do
    color = color / 2
    i = i + 1
  end
  return HEX:sub(i + 1, i + 1)
end

local CACHE = {}
local function blit(color)
  local h = CACHE[color]
  if not h then
    h = hex(color)
    CACHE[color] = h
  end
  return h
end

function M.new(w, h)
  local self = setmetatable({ w = w, h = h, text = {}, fg = {}, bg = {}, shown = {} }, M)
  self:clear(colors.black, colors.white)
  return self
end

function M:clear(bg, fg)
  local b, f = blit(bg or colors.black), blit(fg or colors.white)
  for y = 1, self.h do
    local t, fr, br = {}, {}, {}
    for x = 1, self.w do
      t[x], fr[x], br[x] = " ", f, b
    end
    self.text[y], self.fg[y], self.bg[y] = t, fr, br
  end
end

-- Write text at x, y; clipped to the canvas (and to the optional clip box).
function M:write(x, y, str, fg, bg, clip)
  str = tostring(str)
  if y < 1 or y > self.h then return end
  local x0, x1 = 1, self.w
  if clip then
    if y < clip.y or y > clip.y + clip.h - 1 then return end
    x0, x1 = math.max(1, clip.x), math.min(self.w, clip.x + clip.w - 1)
  end
  local f = fg and blit(fg)
  local b = bg and blit(bg)
  local row, frow, brow = self.text[y], self.fg[y], self.bg[y]
  for i = 1, #str do
    local cx = x + i - 1
    if cx >= x0 and cx <= x1 then
      row[cx] = str:sub(i, i)
      if f then frow[cx] = f end
      if b then brow[cx] = b end
    end
  end
end

function M:fill(x, y, w, h, bg, ch, fg)
  local line = string.rep(ch or " ", math.max(0, w))
  for yy = y, y + h - 1 do
    self:write(x, yy, line, fg, bg)
  end
end

-- Centre text inside a box.
function M:center(x, y, w, str, fg, bg)
  str = tostring(str)
  if #str > w then str = str:sub(1, w) end
  self:write(x + math.floor((w - #str) / 2), y, str, fg, bg)
end

-- Plot one character (for compass and attitude drawings).
function M:put(x, y, ch, fg, bg)
  x, y = math.floor(x + 0.5), math.floor(y + 0.5)
  if x >= 1 and x <= self.w and y >= 1 and y <= self.h then
    self:write(x, y, ch, fg, bg)
  end
end

function M:flush(mon)
  for y = 1, self.h do
    local t = table.concat(self.text[y])
    local f = table.concat(self.fg[y])
    local b = table.concat(self.bg[y])
    local key = t .. f .. b
    if self.shown[y] ~= key then
      mon.setCursorPos(1, y)
      mon.blit(t, f, b)
      self.shown[y] = key
    end
  end
end

-- Forget what is on screen (after a resize or re-attach).
function M:invalidate()
  self.shown = {}
end

return M
