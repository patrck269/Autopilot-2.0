-- Autopilot modes and the glue between sensors, controllers and the RSC bank.
--
--   STBY    autopilot off: rear thrust and RCS to 0, elevation left as it is
--   HOVER   hold target altitude (climb/descend to it), brake to a stop with
--           reverse thrust, hold heading, damp sideways drift
--   CRUISE  hold target altitude and bearing, fly at target speed (cap 35 m/s,
--           reached through a ramp that arrives within 15 s), slow down with
--           reverse thrust, RCS keeps heading and cancels sideslip
--   KILL    every controller to 0 at once; stays until another mode is chosen
--   DEBUG   every controller at +debug_rpm as written to the block (the
--           config sign is not applied), to see which way each prop pushes.
--           The elevation drops to that RPM too. Learning is paused.
local util = require("ap2.util")
local altitude = require("ap2.altitude")
local speed = require("ap2.speed")
local heading = require("ap2.heading")
local motion = require("ap2.motion")

local M = {}
M.__index = M

M.MODES = { STBY = true, HOVER = true, CRUISE = true, KILL = true, DEBUG = true }
local RCS = { "bow_port", "bow_stbd", "stern_port", "stern_stbd" }

-- saved: table from a previous flight ({ k =, A =, Dn =, yaw_b =, bow = }) or nil.
-- sea_rho: air density at sea level (seeds the hover guess).
-- tel: telemetry; the learned bow axis is applied to it.
function M.new(cfg, saved, sea_rho, tel)
  if saved and type(saved.bow) == "table" and util.finite(saved.bow.x) then
    tel.set_forward(saved.bow)
  end
  local self = setmetatable({
    cfg = cfg,
    tel = tel,
    mode = "STBY",
    tgt = { alt = nil, brg = nil, spd = 0 },
    alt = altitude.new(cfg, saved, sea_rho),
    spd = speed.new(cfg, saved),
    hdg = heading.new(cfg, saved),
    motion = motion.new(cfg, tel),
    bow_learned = saved ~= nil and saved.bow ~= nil,
    sea_rho = sea_rho,
    caps = {},
    accel_limited_since = nil,
    events = {},
  }, M)
  for id, c in pairs(cfg.controllers) do self.caps[id] = c.cap end
  return self
end

function M:log(text)
  local e = self.events
  e[#e + 1] = text
  if #e > 12 then table.remove(e, 1) end
end

function M:engage(mode, s)
  local prev = self.mode
  self.mode = mode
  -- Learning pauses in KILL and DEBUG; start fresh windows either side.
  self.alt.win, self.spd.win, self.hdg.win, self.motion.win = nil, nil, nil, nil
  if mode == "HOVER" or mode == "CRUISE" then
    if self.tgt.alt == nil then self.tgt.alt = util.round(s.y) end
    if self.tgt.brg == nil then self.tgt.brg = util.round(s.hdg) % 360 end
    if prev ~= "HOVER" and prev ~= "CRUISE" then
      self.hdg:reset()
      self.spd:reset(s.v_fwd)
    end
    if mode == "HOVER" then
      self.spd:set_target(0, s.v_fwd)
    else
      self.spd:set_target(self.tgt.spd, s.v_fwd)
    end
  end
  self:log("MODE " .. mode)
end

-- Handle one command { c = name, v = value }. Returns ok, message.
function M:command(msg, s)
  local c, v = msg.c, msg.v
  if c == "mode" then
    if not M.MODES[v] then return false, "unknown mode " .. tostring(v) end
    if s == nil and (v == "HOVER" or v == "CRUISE") then return false, "no telemetry" end
    self:engage(v, s)
    return true, "mode " .. v
  elseif c == "alt" then
    if not util.finite(v) then return false, "altitude must be a number" end
    self.tgt.alt = util.clamp(v, -64, 2000)
    self:log("ALT " .. util.fmt(self.tgt.alt))
    return true, "altitude " .. util.fmt(self.tgt.alt)
  elseif c == "alt_step" then
    if not util.finite(v) then return false, "step must be a number" end
    local base = self.tgt.alt or (s and util.round(s.y)) or 0
    return self:command({ c = "alt", v = base + v }, s)
  elseif c == "brg" then
    if not util.finite(v) then return false, "bearing must be a number" end
    self.tgt.brg = util.wrap360(v)
    self:log("BRG " .. util.fmt(self.tgt.brg))
    return true, "bearing " .. util.fmt(self.tgt.brg)
  elseif c == "brg_step" then
    if not util.finite(v) then return false, "step must be a number" end
    local base = self.tgt.brg or (s and util.round(s.hdg)) or 0
    return self:command({ c = "brg", v = base + v }, s)
  elseif c == "spd" then
    if not util.finite(v) then return false, "speed must be a number" end
    self.tgt.spd = util.clamp(v, 0, self.cfg.speed.max)
    if self.mode == "CRUISE" and s then self.spd:set_target(self.tgt.spd, s.v_fwd) end
    self:log("SPD " .. util.fmt(self.tgt.spd))
    return true, "speed " .. util.fmt(self.tgt.spd)
  elseif c == "spd_step" then
    if not util.finite(v) then return false, "step must be a number" end
    return self:command({ c = "spd", v = (self.tgt.spd or 0) + v }, s)
  elseif c == "sync" then
    if s == nil then return false, "no telemetry" end
    self.tgt.alt = util.round(s.y)
    self.tgt.brg = util.round(s.hdg) % 360
    self:log("SYNC")
    return true, "targets set to current altitude and heading"
  elseif c == "recal" then
    -- Holding steady on a spinning lift: that RPM is the hover. Otherwise start
    -- again from the default guess and learn in flight.
    local lift = self.lift_applied or 0
    if s and lift > 0 and math.abs(s.vy) < 0.3 and not self.alt.grounded then
      self.alt:reset_calibration(self.sea_rho)
      self.alt:seed_from_hover(lift, s.rho)
      self:log("HOVER CAL FROM " .. lift .. " RPM")
      return true, "hover calibration reset to " .. lift .. " RPM here"
    end
    self.alt:reset_calibration(self.sea_rho)
    self:log("HOVER CAL RESET")
    return true, "hover calibration reset to default"
  end
  return false, "unknown command " .. tostring(c)
end

-- One control step. Returns desired RPM per controller (nil = leave alone).
function M:step(s, now, dt, bank)
  local out = {}
  local cfg = self.cfg
  local lift_applied = bank:applied("lift")
  self.lift_applied = lift_applied
  bank.exact = self.mode == "DEBUG"
  local thr_applied = bank:applied("thrust")
  local thr_cap = self.caps.thrust

  if self.mode == "KILL" then
    for id in pairs(self.caps) do out[id] = 0 end
    return out
  end
  if self.mode == "DEBUG" then
    -- The bank multiplies by sign on write; pre-multiply so the block gets +rpm.
    for id, c in pairs(cfg.controllers) do out[id] = cfg.debug_rpm * (c.sign or 1) end
    return out
  end

  local flying = self.mode == "HOVER" or self.mode == "CRUISE"
  local want_up = flying and self.tgt.alt ~= nil and self.tgt.alt > s.y + cfg.altitude.hold_band
  -- Learning uses the force the props are really making (their modelled
  -- spin, which lags the controller by up to ~1.6 s), not the commanded RPM.
  local lift_prop = bank:prop("lift", now)
  local thr_prop = bank:prop("thrust", now)
  local rcs_prop = {}
  for _, id in ipairs(RCS) do rcs_prop[id] = bank:prop(id, now) end
  local u_now, sw_now = heading.applied(rcs_prop, self.caps)
  local thr_f = util.ssq(thr_prop) / (thr_cap * thr_cap)
  -- While the engine is overstressed nothing turns: learning from it would
  -- teach every model that the props do nothing. Restart the windows after.
  if bank:overstressed(now) then
    self.alt.win, self.spd.win, self.hdg.win, self.motion.win = nil, nil, nil, nil
  else
    if lift_prop ~= 0 or flying then
      self.alt:observe(s, lift_prop, now, want_up)
    end
    self.spd:observe(s, thr_f, now)
    self.hdg:observe(s, u_now, now)
    self.motion:observe(s, thr_f, sw_now, now)
  end

  -- The rear thrusters or the strafe pairs showed the bow is on another axis
  -- than configured: switch to it, keep pointing where the ship points now,
  -- relearn speed.
  local bow = self.motion.bow
  if bow and util.dot(bow, self.tel.forward) < 0.9 then
    local delta = util.wrap180(motion.plane_angle(self.tel, bow) - motion.plane_angle(self.tel, self.tel.forward))
    self.tel.set_forward(bow)
    if self.tgt.brg then self.tgt.brg = (self.tgt.brg + delta) % 360 end
    self.bow_learned, self.bow_changed_at, self.motion.frame_ok = true, now, true
    self:log("BOW AXIS CORRECTED " .. util.fmt(delta) .. " DEG")
    s = self.tel.read()
    self.spd:forget(s.v_fwd)
    self.hdg:reset()
  end

  if not flying then
    out.thrust = 0
    for _, id in ipairs(RCS) do out[id] = 0 end
    return out
  end

  out.lift = self.alt:command(s, self.tgt.alt, dt, self.caps.lift)
  out.thrust = self.spd:command(s, dt, thr_cap, thr_applied)
  local deadzone = self.mode == "HOVER" and cfg.heading.hover_deadzone or cfg.heading.cruise_deadzone
  -- Until the frame is confirmed (or learned), a wrong axis would make the
  -- sideways brake push the wrong way: keep it gentle.
  local auth = self.motion:sway_authority()
  local sway = {
    drift = s.v_side, auth = auth, coming = auth * sw_now * cfg.heading.sway_lag,
    limit = (self.motion.frame_ok or self.bow_learned) and 1 or cfg.heading.probe_limit,
  }
  local rcs_set = {}
  for _, id in ipairs(RCS) do rcs_set[id] = bank:applied(id) end
  local u_set = heading.applied(rcs_set, self.caps)
  local u, sw = self.hdg:command(s, self.tgt.brg, dt, deadzone, sway, u_set - u_now)
  -- Pulling at full and still off the bearing: the RCS can't hold this
  -- (e.g. thrust line too far off centre for the RCS cap).
  if math.abs(u) >= 0.99 and math.abs(self.hdg.err) > deadzone then
    self.rcs_full_since = self.rcs_full_since or now
  else
    self.rcs_full_since = nil
  end
  local rcs = heading.mix(u, sw, self.caps, cfg.actuator.rcs_min_rpm)
  for _, id in ipairs(RCS) do out[id] = rcs[id] end

  if self.spd.saturated ~= 0 and self.spd.phase ~= "HOLD" then
    self.accel_limited_since = self.accel_limited_since or now
  else
    self.accel_limited_since = nil
  end
  return out
end

-- Current warnings for the displays.
function M:alerts(s, now)
  local list = {}
  if self.mode == "KILL" then list[#list + 1] = "KILL: ALL THRUST CUT" end
  if self.mode == "DEBUG" then
    list[#list + 1] = "DEBUG: ALL PROPS AT +" .. self.cfg.debug_rpm .. " RPM, ELEVATION TOO"
  end
  local flying = self.mode == "HOVER" or self.mode == "CRUISE"
  if self.rcs_full_since and now - self.rcs_full_since > 3 then
    list[#list + 1] = "RCS AT FULL: HEADING NOT HELD"
  end
  if self.bow_changed_at and now - self.bow_changed_at < 30 then
    list[#list + 1] = "BOW AXIS CORRECTED: HEADING NOW TRUE"
  end
  if flying and self.hdg.sway_active and not (self.motion.frame_ok or self.bow_learned) then
    list[#list + 1] = "BOW AXIS NOT CONFIRMED YET: SIDEWAYS BRAKE GENTLE"
  end
  if s and math.abs(s.yaw_rate) > 3 * self.cfg.heading.max_rate then
    list[#list + 1] = "SPINNING " .. util.fmt(s.yaw_rate, 0) .. " DEG/S"
  end
  if self.alt.saturated and (self.mode == "HOVER" or self.mode == "CRUISE") then
    list[#list + 1] = "ELEVATION AT RPM CAP"
  end
  if self.spd.pitch_hold then list[#list + 1] = "PITCH LIMIT: THRUST HELD" end
  if self.accel_limited_since and now - self.accel_limited_since > 2 then
    list[#list + 1] = "REAR THRUST AT CAP"
  end
  if s and s.gs > self.cfg.speed.max + 1 then list[#list + 1] = "OVERSPEED" end
  return list
end

function M:saved()
  local h = self.hdg:saved()
  local f = self.tel.forward
  return {
    k = self.alt.k, A = self.spd.A, Dn = self.spd.Dn, yaw_b = h.yaw_b,
    bow = self.bow_learned and { x = f.x, y = f.y, z = f.z } or nil,
  }
end

return M
