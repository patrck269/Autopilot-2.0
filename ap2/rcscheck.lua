-- RCS check: finds where each RCS controller's props really sit on the hull.
--
-- Run in HOVER. One controller at a time pulls at check.rpm for check.on
-- seconds, then rests for check.coast (its props spin down) while the change
-- in heading and in sideways / fore-aft speed is measured, then everything
-- settles for check.settle seconds before the next one. Every corner pulls
-- (positive RPM), so the side it slides the hull toward is its side, and of
-- the two corners on one side, the bow corner swings the nose toward that
-- side more than the stern corner does (the stern one swings it away). The
-- comparison is relative, so a heavily damped helm or a slow yaw drift does
-- not hide it. The heading loop, the sideslip brake and the rear thrust rest
-- meanwhile; the elevation keeps holding altitude.
local util = require("ap2.util")

local M = {}
M.__index = M

M.CORNERS = { "bow_port", "bow_stbd", "stern_port", "stern_stbd" }

function M.new(cfg)
  return setmetatable({ c = cfg.rcs_check, i = 0, phase = nil, results = {}, done = false }, M)
end

-- One control step. Returns RPM for the four corners while running, nil when
-- finished. log(text) records progress.
function M:step(s, now, log)
  local c = self.c
  if self.done then return nil end
  if self.phase == nil then
    self.phase, self.t = "settle", now
  end
  local rpm = { bow_port = 0, bow_stbd = 0, stern_port = 0, stern_stbd = 0 }
  local id = M.CORNERS[self.i]
  if self.phase == "settle" and now - self.t >= c.settle then
    self.i = self.i + 1
    id = M.CORNERS[self.i]
    if not id then
      self.done = true
      return nil
    end
    self.phase, self.t = "on", now
    self.h0, self.side0, self.fwd0 = s.hdg, s.v_side, s.v_fwd
  elseif self.phase == "on" and now - self.t >= c.on then
    self.phase, self.t = "coast", now
  elseif self.phase == "coast" and now - self.t >= c.coast then
    local dh = util.wrap180(s.hdg - self.h0)
    local dside, dfwd = s.v_side - self.side0, s.v_fwd - self.fwd0
    self.results[id] = { dh = dh, dside = dside, dfwd = dfwd }
    log(string.format("%s: side %+.2f m/s, turn %+.1f deg", id:upper(), dside, dh))
    self.phase, self.t = "settle", now
  end
  if self.phase == "on" then rpm[id] = c.rpm end
  return rpm
end
-- After the check: layout[corner] = controller id found there; or nil and why.
function M:layout()
  local c = self.c
  local side = { stbd = {}, port = {} }
  for _, id in ipairs(M.CORNERS) do
    local r = self.results[id]
    if not r then return nil, id:upper() .. " not tested" end
    if math.max(math.abs(r.dside), math.abs(r.dfwd)) < c.min_v then return nil, id:upper() .. " did not move the hull" end
    if math.abs(r.dfwd) > math.abs(r.dside) then return nil, id:upper() .. " pushed fore/aft, not sideways" end
    local list = r.dside > 0 and side.stbd or side.port
    list[#list + 1] = id
  end
  if #side.stbd ~= 2 then return nil, #side.stbd .. " corners slid the hull starboard, not 2" end
  local layout = {}
  for name, list in pairs(side) do
    local a, b = self.results[list[1]], self.results[list[2]]
    if math.abs(a.dh - b.dh) < c.min_deg then return nil, name:upper() .. " pair turned the hull the same way" end
    -- Starboard bow corner swings the nose right (+), port bow corner left (-).
    local first_is_bow = (a.dh > b.dh) == (name == "stbd")
    layout[(first_is_bow and "bow_" or "stern_") .. name] = list[1]
    layout[(first_is_bow and "stern_" or "bow_") .. name] = list[2]
  end
  return layout
end

function M:testing()
  return not self.done and M.CORNERS[self.i] or nil
end

return M
