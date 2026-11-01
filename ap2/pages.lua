-- MFD pages. Each page: id (button label), title, osb(ctx) -> buttons,
-- draw(canvas, area, ctx), optional touch(ctx, x, y, area) for its content.
--
-- ctx fields: st (latest flight status or nil), age (s since it arrived),
-- now, send(c, v), ufc = { text }, arm = {}, reply (last answer text),
-- local_errors / local_warnings (command center checks), flight_id.
local util = require("ap2.util")

local M = {}

local fmt = util.fmt
local compass = util.compass
local sin, cos, rad = math.sin, math.cos, math.rad

local MODE_COLOR = { STBY = colors.lightGray, HOVER = colors.cyan, CRUISE = colors.lime, KILL = colors.red, DEBUG = colors.orange }

local function stale(ctx)
  return ctx.st == nil or ctx.st.s == nil or ctx.age > 3
end

-- Title row: page name on the left, mode on the right.
local function header(c, a, title, ctx)
  c:write(a.x, a.y, title, colors.yellow)
  local st = ctx.st
  if st and st.mode then
    local m = " " .. st.mode .. " "
    c:write(a.x + a.w - #m, a.y, m, colors.black, MODE_COLOR[st.mode] or colors.white)
  end
end

-- Split text into lines of at most width characters, breaking at spaces.
local function wrap(text, width)
  local lines, cur = {}, ""
  for word in string.gmatch(text, "%S+") do
    if cur == "" then
      cur = word
    elseif #cur + 1 + #word <= width then
      cur = cur .. " " .. word
    else
      lines[#lines + 1] = cur
      cur = word
    end
    while #cur > width do
      lines[#lines + 1] = cur:sub(1, width)
      cur = cur:sub(width + 1)
    end
  end
  if cur ~= "" then lines[#lines + 1] = cur end
  return lines
end

-- Draw a "no data" panel. Returns true when the page should stop drawing.
local function no_data(c, a, ctx)
  if not stale(ctx) then return false end
  local y = a.y + 2
  if ctx.st and ctx.st.errors then
    c:write(a.x, y, "FLIGHT COMPUTER START-UP FAILED", colors.red)
    for _, e in ipairs(ctx.st.errors) do
      for _, part in ipairs(wrap(e, a.w)) do
        y = y + 1
        if y < a.y + a.h then c:write(a.x, y, part, colors.orange) end
      end
    end
    return true
  end
  c:write(a.x, y, "NO FLIGHT DATA", colors.red)
  if ctx.st then
    c:write(a.x, y + 1, "Last status " .. fmt(ctx.age, 0) .. " s ago", colors.orange)
  else
    c:write(a.x, y + 1, "Waiting for the flight computer", colors.orange)
  end
  return true
end

local function mode_buttons(ctx, b)
  local mode = ctx.st and ctx.st.mode
  b.T1 = { label = "HOVR", lit = mode == "HOVER", action = function(x) x.send("mode", "HOVER") end }
  b.T2 = { label = "CRZ", lit = mode == "CRUISE", action = function(x) x.send("mode", "CRUISE") end }
  b.T3 = { label = "STBY", lit = mode == "STBY", action = function(x) x.send("mode", "STBY") end }
  return b
end

local function step_button(label, c, v)
  return { label = label, action = function(x) x.send(c, v) end }
end

-- Signed bar: zero in the middle, filled toward the sign of value/cap.
local function bar(c, x, y, w, value, cap, color)
  c:fill(x, y, w, 1, colors.gray)
  local mid = x + math.floor(w / 2)
  c:put(mid, y, "|", colors.white, colors.gray)
  if not util.finite(value) or cap <= 0 then return end
  local n = math.floor(math.abs(value) / cap * (w / 2) + 0.5)
  if n <= 0 then return end
  if value > 0 then c:fill(mid + 1, y, math.min(n, x + w - mid - 1), 1, color) end
  if value < 0 then c:fill(mid - n, y, n, 1, color) end
end

------------------------------------------------------------------ PFD
M.PFD = {
  id = "PFD", title = "Primary flight display",
  osb = function(ctx)
    local b = mode_buttons(ctx, {})
    b.T5 = { label = "SYNC", action = function(x) x.send("sync") end }
    return b
  end,
  draw = function(c, a, ctx)
    header(c, a, "PFD", ctx)
    if no_data(c, a, ctx) then return end
    local st, s = ctx.st, ctx.st.s
    -- Attitude: sky above, ground below; horizon moves with pitch, tilts with roll.
    local top, bottom = a.y + 2, a.y + a.h - 2
    local cx = a.x + a.w / 2
    local cy = (top + bottom) / 2
    local slope = math.tan(rad(util.clamp(s.roll, -60, 60))) * 0.67
    for y = top, bottom do
      for x = a.x, a.x + a.w - 1 do
        local horizon = cy + s.pitch / 2.5 - (x - cx) * slope
        c:put(x, y, " ", nil, y < horizon and colors.lightBlue or colors.brown)
      end
    end
    -- Pitch ladder every 5 degrees (numbers only when there is room).
    local wide = a.w >= 34
    for deg = -20, 20, 5 do
      if deg ~= 0 then
        local y = cy + (s.pitch - deg) / 2.5
        if y >= top and y <= bottom then
          local mark = wide and (string.format("%3d", deg) .. "--") or "--"
          c:write(math.floor(cx) - (wide and 3 or 1), math.floor(y + 0.5), mark, colors.white)
        end
      end
    end
    c:write(math.floor(cx) - 2, math.floor(cy + 0.5), "-(^)-", colors.yellow)
    -- Heading row.
    c:center(a.x, a.y + 1, a.w, "HDG " .. compass(s.hdg) .. "  BRG " .. compass(st.tgt.brg), colors.white, colors.black)
    -- Speed box left, altitude box right.
    local function box(x, lines)
      for i, l in ipairs(lines) do c:write(x, top + i - 1, l[1], l[2] or colors.white, colors.black) end
    end
    box(a.x, {
      { "GS " .. fmt(s.gs, 1) },
      { "FWD" .. fmt(s.v_fwd, 1, 5) },
      { "TGT" .. fmt(st.tgt.spd, 0, 5), colors.cyan },
    })
    local ax = a.x + a.w - 9
    box(ax, {
      { "Y " .. fmt(s.y, 1, 7) },
      { "TGT" .. fmt(st.tgt.alt, 0, 6), colors.cyan },
      { "VS" .. fmt(s.vy, 1, 7), math.abs(s.vy) > 0.3 and colors.yellow or colors.white },
    })
    c:write(a.x, a.y + a.h - 1, "PITCH " .. fmt(s.pitch, 1) .. "  ROLL " .. fmt(s.roll, 1) ..
      "  " .. st.alt.phase .. "/" .. st.spd.phase, colors.lightGray, colors.black)
  end,
}

------------------------------------------------------------------ AP
M.AP = {
  id = "AP", title = "Autopilot targets and modes",
  osb = function(ctx)
    local b = mode_buttons(ctx, {})
    b.T4 = step_button("BRG\n-5", "brg_step", -5)
    b.T5 = step_button("BRG\n+5", "brg_step", 5)
    b.L1 = step_button("ALT\n+10", "alt_step", 10)
    b.L2 = step_button("ALT\n+1", "alt_step", 1)
    b.L3 = step_button("ALT\n-1", "alt_step", -1)
    b.L4 = step_button("ALT\n-10", "alt_step", -10)
    b.R1 = step_button("SPD\n+5", "spd_step", 5)
    b.R2 = step_button("SPD\n+1", "spd_step", 1)
    b.R3 = step_button("SPD\n-1", "spd_step", -1)
    b.R4 = step_button("SPD\n-5", "spd_step", -5)
    return b
  end,
  draw = function(c, a, ctx)
    header(c, a, "AUTOPILOT", ctx)
    if no_data(c, a, ctx) then return end
    local st, s = ctx.st, ctx.st.s
    local x, y = a.x, a.y + 2
    local function row(label, tgt, now, extra, color)
      c:write(x, y, label, colors.yellow)
      if tgt then c:write(x + 4, y, "TGT " .. tgt, colors.cyan) end
      c:write(x + 14, y, now, colors.white)
      if extra then c:write(x + 26, y, extra, color or colors.lightGray) end
      y = y + 1
    end
    row("ALT", fmt(st.tgt.alt, 0, 5), "Y " .. fmt(s.y, 1), st.alt.phase,
      st.alt.phase == "HOLD" and colors.lime or colors.orange)
    row("VS", nil, fmt(s.vy, 2) .. " m/s")
    row("BRG", "  " .. compass(st.tgt.brg), "HDG " .. fmt(s.hdg, 1), "ERR " .. fmt(st.hdg.err, 1))
    row("SPD", fmt(st.tgt.spd, 0, 5), "FWD " .. fmt(s.v_fwd, 1), st.spd.phase,
      st.spd.phase == "PITCH" and colors.red or colors.lightGray)
    row("REF", nil, fmt(st.spd.v_ref, 1) .. " m/s", "GS " .. fmt(s.gs, 1))
    y = y + 1
    c:write(x, y, "Hover " .. fmt(st.alt.hover_now, 0) .. " RPM here, " .. fmt(st.alt.hover_tgt, 0) .. " at target", colors.lightGray)
    y = y + 2
    for i = 1, math.min(#st.alerts, a.y + a.h - 2 - y) do
      c:write(x, y, st.alerts[i], colors.red)
      y = y + 1
    end
    if ctx.reply then c:write(x, a.y + a.h - 1, ctx.reply, colors.lightGray) end
  end,
}

------------------------------------------------------------------ HSI
local CARDINAL = { [0] = "N", [30] = "3", [60] = "6", [90] = "E", [120] = "12", [150] = "15",
  [180] = "S", [210] = "21", [240] = "24", [270] = "W", [300] = "30", [330] = "33" }

M.HSI = {
  id = "HSI", title = "Compass, bearing and position",
  osb = function(ctx)
    return {
      T1 = { label = "BRG\nSYNC", action = function(x)
        if x.st and x.st.s then x.send("brg", util.round(x.st.s.hdg)) end
      end },
      T2 = step_button("BRG\n-10", "brg_step", -10),
      T3 = step_button("BRG\n-1", "brg_step", -1),
      T4 = step_button("BRG\n+1", "brg_step", 1),
      T5 = step_button("BRG\n+10", "brg_step", 10),
    }
  end,
  draw = function(c, a, ctx)
    header(c, a, "HSI", ctx)
    if no_data(c, a, ctx) then return end
    local st, s = ctx.st, ctx.st.s
    local cx = a.x + a.w / 2
    local cy = a.y + 1 + (a.h - 2) / 2
    local ry = math.max(2, (a.h - 4) / 2)
    local rx = math.min(ry * 1.5, a.w / 2 - 2)
    local function at(bearing, scale)
      local rel = rad(bearing - s.hdg)
      return cx + rx * scale * sin(rel), cy - ry * scale * cos(rel)
    end
    for deg = 0, 355, 10 do
      if not CARDINAL[deg] then
        local px, py = at(deg, 1)
        c:put(px, py, ".", colors.gray)
      end
    end
    for deg, label in pairs(CARDINAL) do
      local px, py = at(deg, 1)
      c:put(px - (#label - 1) / 2, py, label, deg % 90 == 0 and colors.white or colors.lightGray)
    end
    if st.tgt.brg then
      local bx, by = at(st.tgt.brg, 0.75)
      c:put(bx, by, "B", colors.cyan)
    end
    if s.track then
      local tx, ty = at(s.track, 0.5)
      c:put(tx, ty, "o", colors.magenta)
    end
    c:put(cx, cy - ry - 1 < a.y + 1 and a.y + 1 or cy - ry - 1, "v", colors.yellow)
    c:put(cx, cy, "^", colors.yellow)
    c:write(a.x, a.y + 1, "HDG " .. compass(s.hdg), colors.white)
    local brg = "BRG " .. compass(st.tgt.brg)
    c:write(a.x + a.w - #brg, a.y + 1, brg, colors.cyan)
    c:write(a.x, a.y + a.h - 2, "TRK " .. compass(s.track) .. " GS " .. fmt(s.gs, 1), colors.magenta)
    c:write(a.x, a.y + a.h - 1, "DRIFT " .. fmt(s.v_side, 1) .. " m/s", colors.lightGray)
    local pos = "X" .. fmt(s.x, 0) .. " Z" .. fmt(s.z, 0)
    c:write(a.x + a.w - #pos, a.y + a.h - 1, pos, colors.lightGray)
  end,
}

------------------------------------------------------------------ ENG
local ORDER = { "lift", "thrust", "bow_port", "bow_stbd", "stern_port", "stern_stbd" }
local SHORT = { lift = "ELEV", thrust = "THR", bow_port = "BOW P", bow_stbd = "BOW S",
  stern_port = "STN P", stern_stbd = "STN S" }

M.ENG = {
  id = "ENG", title = "Engine and rotation controllers",
  draw = function(c, a, ctx)
    header(c, a, "ENGINE", ctx)
    if no_data(c, a, ctx) then return end
    local st = ctx.st
    local e = st.eng or {}
    local y = a.y + 2
    c:write(a.x, y, "MAIN ENGINE " .. fmt(e.rpm, 0) .. " RPM", colors.white)
    y = y + 1
    local load = (e.su and e.cap and e.cap > 0) and e.su / e.cap or nil
    c:write(a.x, y, "STRESS " .. fmt(e.su, 0) .. " / " .. fmt(e.cap, 0) .. " SU", colors.white)
    y = y + 1
    if load then
      local w = a.w
      local n = math.floor(util.clamp(load, 0, 1) * w + 0.5)
      c:fill(a.x, y, w, 1, colors.gray)
      c:fill(a.x, y, n, 1, load > 0.9 and colors.red or (load > 0.75 and colors.orange or colors.green))
      c:write(a.x + 1, y, fmt(load * 100, 0) .. "%", colors.white)
    end
    y = y + 2
    local label_w = a.w >= 40 and 16 or 6
    local num_w = 12
    local bar_w = a.w - label_w - num_w - 1
    for _, id in ipairs(ORDER) do
      local r = st.rsc and st.rsc[id]
      if r and y < a.y + a.h then
        -- red: missing, magenta: changed by another computer, orange: held by the stress limit
        local color = colors.white
        if not r.present then color = colors.red
        elseif r.foreign then color = colors.magenta
        elseif r.held then color = colors.orange end
        local label = label_w >= 16 and (r.label or id) or SHORT[id]
        c:write(a.x, y, label:sub(1, label_w - 1), color)
        c:write(a.x + label_w, y, fmt(r.app, 0, 5) .. "/" .. fmt(r.des, 0, 5), colors.lightGray)
        if bar_w >= 6 then
          bar(c, a.x + label_w + num_w + 1, y, bar_w, r.app, r.cap, r.fault and colors.red or colors.lime)
        end
        y = y + 1
      end
    end
    if y < a.y + a.h then
      c:write(a.x, a.y + a.h - 1, "applied/desired RPM", colors.gray)
    end
  end,
}

------------------------------------------------------------------ CAL
M.CAL = {
  id = "CAL", title = "Hover and thrust calibration",
  osb = function(ctx)
    local armed = ctx.arm.recal and ctx.now - ctx.arm.recal < 3
    return {
      T1 = { label = armed and "CONF\nRECAL" or "RECAL", warn = armed, action = function(x)
        if armed then
          x.arm.recal = nil
          x.send("recal")
        else
          x.arm.recal = x.now
        end
      end },
    }
  end,
  draw = function(c, a, ctx)
    header(c, a, "CALIBRATION", ctx)
    if no_data(c, a, ctx) then return end
    local st, s, al, sp = ctx.st, ctx.st.s, ctx.st.alt, ctx.st.spd
    local rows = {
      { "State", (al.grounded and "ON GROUND" or "AIRBORNE") .. "  " .. al.phase },
      { "Samples", fmt(al.samples, 0) },
      { "Hover RPM here", fmt(al.hover_now, 0) },
      { "Hover RPM @ tgt", fmt(al.hover_tgt, 0) .. "  (Y " .. fmt(st.tgt.alt, 0) .. ")" },
      { "Air density", fmt(s.rho, 4) .. " / tgt " .. fmt(al.rho_tgt, 4) },
      { "Lift constant k", al.k and string.format("%.4g", al.k) or "---" },
      { "Alt integral", fmt(al.integ, 2) .. " m/s2" },
      { "Ship mass", fmt(s.mass, 0) .. " kg" },
      { "Thrust accel", fmt(sp.A, 2) .. " m/s2 at full" },
      { "Drag at 35", fmt(sp.drag35, 2) .. " m/s2" },
      { "RCS yaw", fmt(st.hdg.yaw_b, 2) .. " deg/s2 at full" },
      { "RCS sideways", fmt(st.hdg.sway_auth, 2) .. " m/s2 at full" },
      { "Bow axis", tostring(st.hdg.bow) .. (st.hdg.bow_learned and " (learned)"
        or (st.hdg.frame_ok and " (config, confirmed)" or " (config)")) },
    }
    local y = a.y + 2
    for _, r in ipairs(rows) do
      if y >= a.y + a.h then break end
      c:write(a.x, y, r[1], colors.lightGray)
      c:write(a.x + 17, y, r[2], colors.white)
      y = y + 1
    end
    if al.saturated then c:write(a.x, a.y + a.h - 1, "ELEVATION AT RPM CAP", colors.red) end
  end,
}

------------------------------------------------------------------ SYS
M.SYS = {
  id = "SYS", title = "Status, alerts, kill and debug",
  osb = function(ctx)
    local armed = ctx.arm.kill and ctx.now - ctx.arm.kill < 3
    local dbg_armed = ctx.arm.debug and ctx.now - ctx.arm.debug < 3
    local mode = ctx.st and ctx.st.mode
    return {
      T1 = { label = "STBY", lit = mode == "STBY", action = function(x) x.send("mode", "STBY") end },
      -- DEBUG drops the elevation to debug RPM too, so it needs a second press.
      T3 = { label = dbg_armed and "CONF\nDEBUG" or "DEBUG", lit = mode == "DEBUG", warn = dbg_armed,
        action = function(x)
          if dbg_armed then
            x.arm.debug = nil
            x.send("mode", "DEBUG")
          else
            x.arm.debug = x.now
          end
        end },
      T5 = { label = armed and "CONF\nKILL" or "KILL", warn = true, action = function(x)
        if armed then
          x.arm.kill = nil
          x.send("mode", "KILL")
        else
          x.arm.kill = x.now
        end
      end },
    }
  end,
  draw = function(c, a, ctx)
    header(c, a, "SYSTEM", ctx)
    local y = a.y + 2
    local function line(text, color)
      for _, part in ipairs(wrap(text, a.w)) do
        if y < a.y + a.h then
          c:write(a.x, y, part, color)
          y = y + 1
        end
      end
    end
    local st = ctx.st
    if st then
      line("Flight #" .. tostring(ctx.flight_id) .. " " .. tostring(st.slug or "") .. "  " .. fmt(ctx.age, 1) .. " s",
        ctx.age > 3 and colors.red or colors.lime)
    else
      line("No flight computer heard", colors.red)
    end
    for _, e in ipairs(ctx.local_errors or {}) do line("CMD ERROR " .. e, colors.red) end
    for _, w in ipairs(ctx.local_warnings or {}) do line("CMD " .. w, colors.orange) end
    if st then
      for _, e in ipairs(st.errors or {}) do line("FLT ERROR " .. e, colors.red) end
      for _, al in ipairs(st.alerts or {}) do line(al, colors.red) end
      for _, w in ipairs(st.warnings or {}) do line(w, colors.orange) end
      if st.msg then line("Last: " .. st.msg, colors.lightGray) end
      local ev = st.events or {}
      for i = #ev, 1, -1 do line(ev[i], colors.gray) end
    end
  end,
}

------------------------------------------------------------------ UFC
local KEYS = { { "7", "8", "9" }, { "4", "5", "6" }, { "1", "2", "3" }, { "-", "0", "." } }

local function key_grid(a)
  local top = a.y + 3
  local rows, cols = #KEYS, 3
  local kh = math.max(1, math.floor((a.y + a.h - top) / rows) - 1)
  local kw = math.max(3, math.floor(a.w / cols) - 1)
  return top, kw, kh
end

M.UFC = {
  id = "UFC", title = "Keypad: type a value, then set it",
  osb = function(ctx)
    local function set(c)
      return function(x)
        local v = tonumber(x.ufc.text)
        if v then
          x.send(c, v)
          x.ufc.text = ""
        else
          x.reply = "keypad: enter a number first"
        end
      end
    end
    return {
      L1 = { label = "ALT\nSET", action = set("alt") },
      L2 = { label = "BRG\nSET", action = set("brg") },
      L3 = { label = "SPD\nSET", action = set("spd") },
      R1 = { label = "CLR", action = function(x) x.ufc.text = "" end },
      R2 = { label = "BKSP", action = function(x) x.ufc.text = x.ufc.text:sub(1, -2) end },
    }
  end,
  draw = function(c, a, ctx)
    header(c, a, "KEYPAD", ctx)
    c:fill(a.x, a.y + 1, a.w, 1, colors.gray)
    c:write(a.x + 1, a.y + 1, "> " .. ctx.ufc.text .. "_", colors.white, colors.gray)
    local top, kw, kh = key_grid(a)
    for r, row in ipairs(KEYS) do
      for col, k in ipairs(row) do
        local x = a.x + (col - 1) * (kw + 1)
        local y = top + (r - 1) * (kh + 1)
        if y + kh - 1 < a.y + a.h then
          c:fill(x, y, kw, kh, colors.gray)
          c:center(x, y + math.floor((kh - 1) / 2), kw, k, colors.white, colors.gray)
        end
      end
    end
    if ctx.reply then c:write(a.x, a.y + 2, ctx.reply, colors.lightGray) end
  end,
  touch = function(ctx, x, y, a)
    local top, kw, kh = key_grid(a)
    local gx = x - 1
    local gy = (a.y + y - 1) - top
    if gy < 0 then return end
    local col = math.floor(gx / (kw + 1)) + 1
    local r = math.floor(gy / (kh + 1)) + 1
    local k = KEYS[r] and KEYS[r][col]
    if not k then return end
    if #ctx.ufc.text < 8 then ctx.ufc.text = ctx.ufc.text .. k end
  end,
}

M.LIST = { M.PFD, M.AP, M.HSI, M.ENG, M.CAL, M.SYS, M.UFC }

return M
