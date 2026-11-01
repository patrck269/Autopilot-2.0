-- Command center program: MFDs on the monitors, typed commands on the
-- terminal, telemetry from the flight computer over the link.
local util = require("ap2.util")
local checks = require("ap2.checks")
local mfd = require("ap2.mfd")
local pages = require("ap2.pages")
local store = require("ap2.store")
local cmdline = require("ap2.cmdline")

local M = {}

local function paint_errors(P, cfg, errors, warnings)
  local lines = { { "AUTOPILOT 2.0 COMMAND: START-UP CHECK FAILED", colors.red } }
  for _, e in ipairs(errors) do lines[#lines + 1] = { "ERROR: " .. e, colors.red } end
  for _, w in ipairs(warnings) do lines[#lines + 1] = { "WARN:  " .. w, colors.orange } end
  lines[#lines + 1] = { "Checking again when a peripheral is attached.", colors.lightGray }
  local targets = { term }
  for _, name in ipairs(cfg.monitors or {}) do
    if P.isPresent(name) and P.getType(name) == "monitor" then targets[#targets + 1] = P.wrap(name) end
  end
  for _, t in ipairs(targets) do
    if t ~= term and t.setTextScale then t.setTextScale(cfg.monitor_scale) end
    t.setBackgroundColor(colors.black)
    t.clear()
    for i, l in ipairs(lines) do
      t.setCursorPos(1, i)
      t.setTextColor(l[2])
      t.write(l[1])
    end
  end
  term.setTextColor(colors.white)
end

-- opts = { link = link object, role = "command"|"both", terminal = bool }
function M.run(cfg, opts)
  local P = peripheral
  local link = opts.link
  local env = { peripheral = P }
  local errors, warnings, names

  local function wait_peripheral()
    local timer = os.startTimer(2)
    while true do
      local ev = os.pullEvent()
      if ev == "peripheral" or ev == "peripheral_detach" or ev == "timer" then break end
    end
    os.cancelTimer(timer)
  end

  while true do
    errors, warnings, names = checks.command(cfg, env, opts.role)
    if #errors == 0 then break end
    paint_errors(P, cfg, errors, warnings)
    wait_peripheral()
  end

  local mgr
  local function build()
    local displays = {}
    for _, name in ipairs(names) do
      local mon = P.wrap(name)
      mon.setTextScale(cfg.monitor_scale)
      mon.setBackgroundColor(colors.black)
      mon.clear()
      local w, h = mon.getSize()
      displays[#displays + 1] = { name = name, mon = mon, w = w, h = h }
    end
    local saved = mgr and mgr:layout_state() or store.load(cfg.mfd_file)
    mgr = mfd.new(displays, pages.LIST, saved)
  end
  build()

  local own_ship = ship and ship.getId and ship.getId() or nil
  local flight_id = cfg.flight_id
  if opts.role == "both" then flight_id = "local" end
  local st_time = -math.huge
  local seq = 0

  local ctx = {
    st = nil, age = math.huge, now = os.clock(),
    ufc = { text = "" }, arm = {}, reply = nil,
    local_errors = errors, local_warnings = warnings, flight_id = flight_id,
  }
  function ctx.send(c, v)
    if flight_id == nil then
      ctx.reply = "no flight computer heard yet"
      return false
    end
    seq = seq + 1
    link:send_command({ c = c, v = v, seq = seq, ship = own_ship }, flight_id)
    ctx.reply = "sent " .. c .. (v ~= nil and (" " .. tostring(v)) or "")
    return true
  end

  local prompt = cmdline.prompt()
  local help = false

  local function draw_terminal()
    if not opts.terminal then return end
    local w, h = term.getSize()
    term.setBackgroundColor(colors.black)
    term.clear()
    local function line(y, text, color)
      term.setCursorPos(1, y)
      term.setTextColor(color or colors.white)
      term.write(text:sub(1, w))
    end
    line(1, "AUTOPILOT 2.0 COMMAND CENTER", colors.yellow)
    local st = ctx.st
    local y = 2
    if st and st.s and ctx.age < 3 then
      local s = st.s
      line(2, "Flight #" .. tostring(flight_id) .. " " .. tostring(st.slug) .. "  mode " .. st.mode, colors.lime)
      line(3, string.format("Y %s -> %s  VS %s", util.fmt(s.y, 1), util.fmt(st.tgt.alt, 0), util.fmt(s.vy, 2)))
      line(4, string.format("HDG %s -> %s  SPD %s -> %s", util.compass(s.hdg), util.compass(st.tgt.brg),
        util.fmt(s.v_fwd, 1), util.fmt(st.tgt.spd, 0)))
      y = 5
      for _, a in ipairs(st.alerts or {}) do
        if y < h - 2 then line(y, a, colors.red); y = y + 1 end
      end
    elseif st and st.errors then
      line(2, "Flight computer start-up failed:", colors.red)
      y = 3
      for _, e in ipairs(st.errors) do
        if y < h - 2 then line(y, e, colors.orange); y = y + 1 end
      end
    else
      line(2, "No flight data", colors.red)
      y = 3
    end
    for _, wmsg in ipairs(warnings) do
      if y < h - 2 then line(y, wmsg, colors.orange); y = y + 1 end
    end
    if help then
      for _, hl in ipairs(cmdline.HELP) do
        if y < h - 1 then line(y, hl, colors.lightGray); y = y + 1 end
      end
    end
    line(h - 1, ctx.reply or "type help for commands", colors.lightGray)
    line(h, "> " .. prompt.text, colors.white)
    term.setCursorPos(math.min(w, 3 + #prompt.text), h)
    term.setCursorBlink(true)
  end

  local redraw = os.startTimer(0.2)
  local term_at = 0
  while true do
    local ev = { os.pullEvent() }
    ctx.now = os.clock()
    ctx.age = ctx.now - st_time
    local kind, msg, from = link:parse(ev)
    if kind == "status" then
      local mine = true
      if msg.ship and own_ship and msg.ship ~= own_ship then mine = false end
      if cfg.flight_id and from ~= cfg.flight_id then mine = false end
      if opts.role == "both" and from ~= "local" then mine = false end
      if mine then
        flight_id = from
        ctx.flight_id = from
        ctx.st = msg
        st_time = ctx.now
        ctx.age = 0
      end
    elseif kind == "reply" then
      ctx.reply = (msg.ok and "" or "REJECTED: ") .. tostring(msg.text)
    elseif ev[1] == "timer" and ev[2] == redraw then
      redraw = os.startTimer(0.2)
      mgr:draw_all(ctx)
      if ctx.now - term_at >= 0.5 then
        term_at = ctx.now
        draw_terminal()
      end
    elseif ev[1] == "monitor_touch" then
      local changed = mgr:touch(ev[2], ev[3], ev[4], ctx)
      if changed == "layout" then store.save(cfg.mfd_file, mgr:layout_state()) end
      if changed then mgr:draw_all(ctx) end
    elseif ev[1] == "monitor_resize" then
      build()
      mgr:draw_all(ctx)
    elseif ev[1] == "peripheral" or ev[1] == "peripheral_detach" then
      local errs, warns, found = checks.command(cfg, env, opts.role)
      ctx.local_errors, ctx.local_warnings, warnings = errs, warns, warns
      if #found > 0 then
        names = found
        build()
        mgr:draw_all(ctx)
      end
    else
      local line = prompt.handle(ev)
      if line then
        local cmd, err = cmdline.parse(line)
        help = false
        if cmd == "help" then
          help = true
        elseif cmd then
          ctx.send(cmd.c, cmd.v)
        elseif err ~= "" then
          ctx.reply = err
        end
        draw_terminal()
      elseif ev[1] == "char" or ev[1] == "key" or ev[1] == "paste" then
        draw_terminal()
      end
    end
  end
end

return M
