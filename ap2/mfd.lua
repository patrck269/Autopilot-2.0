-- Fighter-style multifunction displays on ComputerCraft monitors.
--
-- Every monitor is one MFD with option select buttons (OSBs) around its edge:
--   top row T1-T5 and side columns L1-L4 / R1-R4: actions of the shown page
--   bottom row: B1 MENU, B2-B4 the three formats programmed on this MFD
--               (the lit one is shown), B5 SWAP
-- Pressing an unlit format button shows that format; pressing the lit one (or
-- MENU) opens the format menu, where any page can be put on this MFD. A page
-- is only ever shown on one MFD: choosing one that another MFD is showing
-- moves it here and sends this MFD's old page there. SWAP exchanges the whole
-- MFD with its partner (1<->2, 3<->4, ...), like the left/right MFD swap in a
-- fighter cockpit.
local canvas = require("ap2.canvas")

local M = {}
M.__index = M

local SIDE_POS = { "T1", "T2", "T3", "T4", "T5", "L1", "L2", "L3", "L4", "R1", "R2", "R3", "R4" }
M.SIDE_POS = SIDE_POS

-- Button and content rectangles for a w x h monitor.
function M.layout(w, h)
  local bh = (h >= 30 and 3) or (h >= 16 and 2) or 1
  local sw = (w >= 50 and 7) or (w >= 34 and 6) or 5
  local osb = {}
  local bw = math.floor((w - 4) / 5)
  local extra = (w - 4) - 5 * bw
  local x = 1
  for i = 1, 5 do
    local ww = bw + ((i <= extra) and 1 or 0)
    osb["T" .. i] = { x = x, y = 1, w = ww, h = bh }
    osb["B" .. i] = { x = x, y = h - bh + 1, w = ww, h = bh }
    x = x + ww + 1
  end
  local top, bottom = bh + 2, h - bh - 1
  local sh = math.max(1, math.floor((bottom - top + 1 - 3) / 4))
  for i = 1, 4 do
    local y = top + (i - 1) * (sh + 1)
    osb["L" .. i] = { x = 1, y = y, w = sw, h = sh }
    osb["R" .. i] = { x = w - sw + 1, y = y, w = sw, h = sh }
  end
  local content = { x = sw + 2, y = bh + 2, w = w - 2 * sw - 2, h = h - 2 * bh - 2 }
  return { osb = osb, content = content, w = w, h = h }
end

local function inside(r, x, y)
  return x >= r.x and x < r.x + r.w and y >= r.y and y < r.y + r.h
end

function M.hit(lay, x, y)
  for pos, r in pairs(lay.osb) do
    if inside(r, x, y) then return pos end
  end
  if inside(lay.content, x, y) then return "content" end
  return nil
end

-- Default formats per MFD: the first entry is shown.
local DEFAULTS = {
  { "PFD", "AP", "UFC" },
  { "HSI", "AP", "CAL" },
  { "ENG", "CAL", "SYS" },
  { "SYS", "UFC", "CAL" },
  { "AP", "UFC", "PFD" },
  { "CAL", "SYS", "ENG" },
}

-- displays: list of { name =, mon =, w =, h = }; pages: list of page tables.
-- saved: { [monitor name] = { slots = {...}, sel = n } } or nil.
function M.new(displays, pages, saved)
  local self = setmetatable({ displays = {}, pages = pages, by_id = {}, by_name = {} }, M)
  for _, p in ipairs(pages) do self.by_id[p.id] = p end
  for i, d in ipairs(displays) do
    local def = DEFAULTS[(i - 1) % #DEFAULTS + 1]
    local mfd = {
      index = i, name = d.name, mon = d.mon, w = d.w, h = d.h,
      lay = M.layout(d.w, d.h), canvas = canvas.new(d.w, d.h),
      slots = { def[1], def[2], def[3] }, sel = 1, menu = false, flash = nil,
    }
    local s = saved and saved[d.name]
    if s and type(s.slots) == "table" and self.by_id[s.slots[1]] and self.by_id[s.slots[2]] and self.by_id[s.slots[3]] then
      mfd.slots = { s.slots[1], s.slots[2], s.slots[3] }
      mfd.sel = (s.sel == 1 or s.sel == 2 or s.sel == 3) and s.sel or 1
    end
    self.displays[i] = mfd
    self.by_name[d.name] = mfd
  end
  self:dedupe()
  return self
end

function M:shown(d)
  return d.slots[d.sel]
end

-- Who shows page id (other than skip)?
function M:showing(id, skip)
  for _, d in ipairs(self.displays) do
    if d ~= skip and self:shown(d) == id then return d end
  end
  return nil
end

-- Make every shown page unique (after loading a saved layout).
function M:dedupe()
  local used = {}
  for _, d in ipairs(self.displays) do
    local id = self:shown(d)
    if used[id] then
      for _, p in ipairs(self.pages) do
        if not used[p.id] then
          d.slots[d.sel] = p.id
          id = p.id
          break
        end
      end
    end
    used[id] = true
  end
end

-- Show page id on MFD d; the MFD that was showing it gets d's old page.
function M:show(d, id)
  local old = self:shown(d)
  if old == id then return end
  local other = self:showing(id, d)
  d.slots[d.sel] = id
  if other then other.slots[other.sel] = old end
end

-- Select format slot k on MFD d.
function M:select(d, k)
  local id = d.slots[k]
  local old = self:shown(d)
  local other = self:showing(id, d)
  d.sel = k
  if other then other.slots[other.sel] = old end
end

function M:partner(d)
  local n = #self.displays
  if n < 2 then return nil end
  local i = d.index
  local j = (i % 2 == 1) and i + 1 or i - 1
  if j > n then j = 1 end
  if j == i then return nil end
  return self.displays[j]
end

function M:swap(d)
  local e = self:partner(d)
  if not e then return end
  d.slots, e.slots = e.slots, d.slots
  d.sel, e.sel = e.sel, d.sel
  d.menu, e.menu = false, false
end

function M:layout_state()
  local out = {}
  for _, d in ipairs(self.displays) do
    out[d.name] = { slots = { d.slots[1], d.slots[2], d.slots[3] }, sel = d.sel }
  end
  return out
end

-- Page OSBs for MFD d: either the page's actions or the format menu.
function M:side_buttons(d, ctx)
  if d.menu then
    local list = {}
    for i, p in ipairs(self.pages) do
      local pos = SIDE_POS[i]
      if pos then
        list[pos] = { label = p.id, lit = self:shown(d) == p.id, page = p.id }
      end
    end
    return list
  end
  local page = self.by_id[self:shown(d)]
  return page.osb and page.osb(ctx) or {}
end

function M:bottom_buttons(d)
  local b = {
    B1 = { label = "MENU", lit = d.menu },
    B5 = { label = "SWAP" },
  }
  for k = 1, 3 do
    b["B" .. (k + 1)] = { label = d.slots[k], lit = (k == d.sel) and not d.menu, sel = true }
  end
  return b
end

-- Handle a monitor_touch. Returns true when anything changed (layout or a
-- command was sent), "layout" when the MFD layout should be saved.
function M:touch(name, x, y, ctx)
  local d = self.by_name[name]
  if not d then return false end
  local pos = M.hit(d.lay, x, y)
  if not pos then return false end
  d.flash = { pos = pos, t = ctx.now }
  if pos == "B1" then
    d.menu = not d.menu
    return "layout"
  elseif pos == "B5" then
    self:swap(d)
    return "layout"
  elseif pos == "B2" or pos == "B3" or pos == "B4" then
    local k = tonumber(pos:sub(2)) - 1
    if k == d.sel and not d.menu then
      d.menu = true
    else
      self:select(d, k)
      d.menu = false
    end
    return "layout"
  elseif pos == "content" then
    if d.menu then return false end
    local page = self.by_id[self:shown(d)]
    if page.touch then
      local c = d.lay.content
      page.touch(ctx, x - c.x + 1, y - c.y + 1, c)
      return true
    end
    return false
  else
    local b = self:side_buttons(d, ctx)[pos]
    if not b then return false end
    if d.menu then
      self:show(d, b.page)
      d.menu = false
      return "layout"
    end
    if b.action then
      b.action(ctx)
      return true
    end
  end
  return false
end

local function draw_button(c, r, b, flash)
  local bg, fg = colors.gray, colors.white
  if b.lit then bg, fg = colors.lime, colors.black end
  if b.warn then bg, fg = colors.red, colors.white end
  if b.sel and not b.lit then bg, fg = colors.lightGray, colors.black end
  if flash then bg, fg = colors.white, colors.black end
  c:fill(r.x, r.y, r.w, r.h, bg)
  local lines = {}
  for part in string.gmatch(b.label, "[^\n]+") do lines[#lines + 1] = part end
  local y0 = r.y + math.floor((r.h - #lines) / 2)
  for i, text in ipairs(lines) do
    if i <= r.h then c:center(r.x, math.max(r.y, y0 + i - 1), r.w, text, fg, bg) end
  end
end

-- Content of the format menu.
function M:draw_menu(d, c, area)
  c:write(area.x, area.y, "FORMAT MENU", colors.yellow)
  c:write(area.x, area.y + 1, "Slot " .. d.sel .. ": pick a page for", colors.lightGray)
  c:write(area.x, area.y + 2, "this display.", colors.lightGray)
  local y = area.y + 4
  for _, p in ipairs(self.pages) do
    local where = self:showing(p.id, nil)
    local tag = ""
    if where == d then tag = "  (here)" elseif where then tag = "  (on " .. where.name .. ")" end
    c:write(area.x, y, string.format("%-4s %s%s", p.id, p.title, tag), where == d and colors.lime or colors.white)
    y = y + 1
  end
end

function M:draw(d, ctx)
  local c = d.canvas
  c:clear(colors.black, colors.white)
  local area = d.lay.content
  if d.menu then
    self:draw_menu(d, c, area)
  else
    local page = self.by_id[self:shown(d)]
    page.draw(c, area, ctx, d)
  end
  local flash = d.flash and ctx.now - d.flash.t < 0.25 and d.flash.pos or nil
  for pos, b in pairs(self:side_buttons(d, ctx)) do
    local r = d.lay.osb[pos]
    if r then draw_button(c, r, b, flash == pos) end
  end
  for pos, b in pairs(self:bottom_buttons(d)) do
    draw_button(c, d.lay.osb[pos], b, flash == pos)
  end
  c:flush(d.mon)
end

function M:draw_all(ctx)
  for _, d in ipairs(self.displays) do self:draw(d, ctx) end
end

return M
