-- Flight computer program: checks peripherals, then runs the control loop,
-- the controller writer, gauge polling, the command link and the terminal.
local util = require("ap2.util")
local checks = require("ap2.checks")
local telemetry = require("ap2.telemetry")
local actuators = require("ap2.actuators")
local autopilot = require("ap2.autopilot")
local store = require("ap2.store")
local cmdline = require("ap2.cmdline")

local M = {}

local function show_errors(title, errors, warnings)
  term.setBackgroundColor(colors.black)
  term.clear()
  term.setCursorPos(1, 1)
  term.setTextColor(colors.red)
  print(title)
  for _, e in ipairs(errors) do print("  ERROR: " .. e) end
  term.setTextColor(colors.orange)
  for _, w in ipairs(warnings) do print("  WARN:  " .. w) end
  term.setTextColor(colors.lightGray)
  print("")
  print("Fix the wiring or ap2/config.lua; checking again")
  print("when a peripheral is attached (and every 2 s).")
  term.setTextColor(colors.white)
end

-- opts = { link = link object, role = "flight"|"both", terminal = bool }
function M.run(cfg, opts)
  local P = peripheral
  local link = opts.link
  local env = { peripheral = P, ship = ship, aero = aero }
  local warnings

  -- 1. Required modems and peripherals. Nothing is driven until all are present.
  while true do
    local errors
    errors, warnings = checks.flight(cfg, env, opts.role)
    if #errors == 0 then break end
    if opts.terminal then show_errors("AUTOPILOT 2.0 FLIGHT: START-UP CHECK FAILED", errors, warnings) end
    link:send_status({ errors = errors, warnings = warnings })
    local timer = os.startTimer(2)
    while true do
      local ev = os.pullEvent()
      if ev == "peripheral" or ev == "peripheral_detach" or ev == "timer" then break end
    end
    os.cancelTimer(timer)
  end

  local devices = {}
  for id, c in pairs(cfg.controllers) do devices[id] = P.wrap(c.name) end
  local bank = actuators.new(cfg, devices)
  bank:seed()
  local tel = telemetry.new(cfg, { ship = ship, aero = aero, clock = os.clock })
  local saved = store.load(cfg.cal_file)
  local ap = autopilot.new(cfg, saved, tel.rho_at(62))
  local ship_id = ship.getId and ship.getId() or nil
  local slug = ship.getSlug and ship.getSlug() or "ship"

  local last_s = tel.read()
  -- A reboot in the air leaves the elevation spinning: take over in HOVER at
  -- the current altitude instead of letting the hull drift.
  local lift = bank:applied("lift")
  if lift > 0 then
    if math.abs(last_s.vy) < 0.5 then ap.alt:seed_from_hover(lift, last_s.rho) end
    ap.tgt.alt = util.round(last_s.y)
    ap:engage("HOVER", last_s)
  end

  local gauges = { rpm = nil, su = nil, cap = nil }
  local faults = {}
  local message = "ready"
  local last_saved = ap:saved()
  local sent_at = -math.huge

  local function status(now)
    local s = last_s
    local a = ap.alt
    local alerts = ap:alerts(s, now)
    for _, f in pairs(faults) do alerts[#alerts + 1] = f end
    for _, id in ipairs(actuators.ORDER) do
      local c = bank.ch[id]
      if c.dev == nil then alerts[#alerts + 1] = "MISSING " .. c.label end
      if c.fault then alerts[#alerts + 1] = c.label .. " WRITE FAILED" end
    end
    local tgt_alt = ap.tgt.alt or s.y
    local rho_t = tel.rho_at(tgt_alt)
    return {
      ship = ship_id, slug = slug, t = now,
      mode = ap.mode,
      tgt = { alt = ap.tgt.alt, brg = ap.tgt.brg, spd = ap.tgt.spd },
      s = {
        x = s.x, y = s.y, z = s.z, vy = s.vy, gs = s.gs, v_fwd = s.v_fwd, v_side = s.v_side,
        hdg = s.hdg, track = s.track, pitch = s.pitch, roll = s.roll, yaw_rate = s.yaw_rate,
        rho = s.rho, mass = s.mass,
      },
      alt = {
        phase = (ap.mode == "HOVER" or ap.mode == "CRUISE") and a.phase or "OFF",
        k = a.k, hover_now = a:hover_rpm(s.rho), hover_tgt = a:hover_rpm(rho_t), rho_tgt = rho_t,
        grounded = a.grounded, samples = a.samples, saturated = a.saturated, integ = a.integ,
      },
      spd = {
        phase = ap.mode == "CRUISE" and ap.spd.phase or (ap.mode == "HOVER" and "STOP" or "OFF"),
        v_ref = ap.spd.v_ref, A = ap.spd.A, drag35 = ap.spd:drag(35), pitch_hold = ap.spd.pitch_hold,
      },
      hdg = {
        err = ap.hdg.err, r_cmd = ap.hdg.r_cmd,
        yaw_b = ap.hdg.yaw.est.t[1], yaw_ok = ap.hdg.yaw.ok,
        sway_b = ap.hdg.sway.est.t[1], sway_ok = ap.hdg.sway.ok,
      },
      rsc = bank:snapshot(now),
      eng = gauges,
      alerts = alerts,
      warnings = warnings,
      events = ap.events,
      msg = message,
    }
  end

  local function handle(msg, from)
    if type(msg) ~= "table" then return end
    if msg.ship and ship_id and msg.ship ~= ship_id then return end
    local ok, text = ap:command({ c = msg.c, v = msg.v }, last_s)
    if ok and msg.c == "mode" and msg.v == "KILL" then bank:kill() end
    message = text
    if from and from ~= "terminal" then link:reply(from, { ok = ok, text = text, seq = msg.seq }) end
    link:send_status(status(os.clock()))
    sent_at = os.clock()
    return ok, text
  end

  local function control()
    local period = cfg.loop_period
    local timer = os.startTimer(period)
    local last = os.clock()
    local save_at = os.clock() + 30
    while true do
      local ev, id = os.pullEvent("timer")
      if id == timer then
        timer = os.startTimer(period)
        local now = os.clock()
        local dt = util.clamp(now - last, 0.05, 0.5)
        last = now
        local ok, s = pcall(tel.read)
        if ok then
          faults.telemetry = nil
          last_s = s
          local out = ap:step(s, now, dt, bank)
          for _, ch in ipairs(actuators.ORDER) do bank:set(ch, out[ch]) end
        else
          faults.telemetry = "SHIP TELEMETRY: " .. tostring(s)
        end
        if now - sent_at >= cfg.status_period then
          link:send_status(status(now))
          sent_at = now
        end
        if now >= save_at then
          save_at = now + 30
          local cur = ap:saved()
          local function moved(a, b, tol)
            if a == nil or b == nil then return a ~= b end
            return math.abs(a - b) > tol * math.max(math.abs(b), 1e-9)
          end
          if moved(cur.k, last_saved.k, 0.005) or moved(cur.A, last_saved.A, 0.01)
            or moved(cur.yaw_b, last_saved.yaw_b, 0.05) or moved(cur.sway_b, last_saved.sway_b, 0.05) then
            store.save(cfg.cal_file, cur)
            last_saved = cur
          end
        end
      end
    end
  end

  local function writer()
    while true do
      if bank:service(os.clock()) == nil then sleep(0.05) end
    end
  end

  local function gauge_poll()
    while true do
      local speedo, stress = P.wrap(cfg.speedometer), P.wrap(cfg.stressometer)
      local rpm, su, cap
      if speedo then
        local ok, v = pcall(speedo.getSpeed)
        if ok then rpm = v end
      end
      if stress then
        local ok, v = pcall(stress.getStress)
        if ok then su = v end
        local ok2, c = pcall(stress.getStressCapacity)
        if ok2 then cap = c end
      end
      gauges.rpm, gauges.su, gauges.cap = rpm, su, cap
      faults.gauges = (rpm == nil or su == nil) and "ENGINE GAUGE READ FAILED" or nil
      sleep(0.5)
    end
  end

  local function receiver()
    while true do
      local ev = { os.pullEvent() }
      local kind, msg, from = link:parse(ev)
      if kind == "cmd" then
        handle(msg, from)
      elseif ev[1] == "peripheral" or ev[1] == "peripheral_detach" then
        for id, c in pairs(cfg.controllers) do
          local present = P.isPresent(c.name)
          if present and bank.ch[id].dev == nil then
            bank:set_device(id, P.wrap(c.name))
            bank:seed()
          elseif not present and bank.ch[id].dev ~= nil then
            bank:set_device(id, nil)
          end
        end
      end
    end
  end

  local function terminal()
    local prompt = cmdline.prompt()
    local help = false
    local redraw = os.startTimer(0.5)
    local function draw()
      local st = status(os.clock())
      local w, h = term.getSize()
      term.setBackgroundColor(colors.black)
      term.clear()
      local function line(y, text, color)
        term.setCursorPos(1, y)
        term.setTextColor(color or colors.white)
        term.write(text:sub(1, w))
      end
      local s = st.s
      line(1, "AUTOPILOT 2.0 FLIGHT  " .. st.slug, colors.yellow)
      line(2, "Mode " .. st.mode .. "   ALT " .. st.alt.phase .. "   SPD " .. st.spd.phase,
        st.mode == "KILL" and colors.red or colors.lime)
      line(3, string.format("Y %s -> %s  VS %s m/s", util.fmt(s.y, 1), util.fmt(st.tgt.alt, 0), util.fmt(s.vy, 2)))
      line(4, string.format("HDG %s -> %s  SPD %s -> %s m/s", util.compass(s.hdg), util.compass(st.tgt.brg),
        util.fmt(s.v_fwd, 1), util.fmt(st.tgt.spd, 0)))
      line(5, string.format("Pitch %s  Roll %s  Hover %s RPM", util.fmt(s.pitch, 1), util.fmt(s.roll, 1),
        util.fmt(st.alt.hover_now, 0)))
      local y = 6
      for _, id in ipairs(actuators.ORDER) do
        local r = st.rsc[id]
        line(y, string.format("%-16s %6s / %6s", r.label, util.fmt(r.app, 0), util.fmt(r.des, 0)),
          r.present and colors.lightGray or colors.red)
        y = y + 1
      end
      local list = help and cmdline.HELP or st.alerts
      for i = 1, math.min(#list, h - y - 2) do
        line(y, list[i], help and colors.lightGray or colors.orange)
        y = y + 1
      end
      line(h - 1, message, colors.lightGray)
      line(h, "> " .. prompt.text, colors.white)
      term.setCursorPos(math.min(w, 3 + #prompt.text), h)
      term.setCursorBlink(true)
    end
    while true do
      local ev = { os.pullEvent() }
      if ev[1] == "timer" and ev[2] == redraw then
        redraw = os.startTimer(0.5)
        draw()
      else
        local line = prompt.handle(ev)
        if line then
          local cmd, err = cmdline.parse(line)
          help = false
          if cmd == "help" then
            help = true
          elseif cmd then
            handle(cmd, "terminal")
          elseif err ~= "" then
            message = err
          end
          draw()
        elseif ev[1] == "char" or ev[1] == "key" then
          draw()
        end
      end
    end
  end

  local tasks = { control, writer, gauge_poll, receiver }
  if opts.terminal then tasks[#tasks + 1] = terminal end
  parallel.waitForAny(table.unpack(tasks))
end

return M
