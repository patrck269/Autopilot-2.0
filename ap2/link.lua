-- Flight computer <-> command center messages.
--
-- Over rednet (protocol cfg.protocol) on every attached modem, and, when one
-- computer runs both roles, through a local event queue ("ap2_local").
-- Message kinds (field ap2): "status" flight -> all, "cmd" command -> flight,
-- "reply" flight -> the sender of a command.
local checks = require("ap2.checks")

local M = {}
M.__index = M

-- opts = { loopback = bool, rednet = bool }
function M.new(cfg, opts)
  local self = setmetatable({ protocol = cfg.protocol, loopback = opts.loopback, rednet = false, modems = {} }, M)
  if opts.rednet ~= false and rednet and peripheral then
    local found = checks.modems(peripheral)
    for _, list in pairs(found) do
      for _, name in ipairs(list) do
        if not rednet.isOpen(name) then rednet.open(name) end
        self.modems[#self.modems + 1] = name
      end
    end
    self.rednet = #self.modems > 0
  end
  return self
end

function M:send_status(msg)
  msg.ap2 = "status"
  if self.loopback then os.queueEvent("ap2_local", msg) end
  if self.rednet then rednet.broadcast(msg, self.protocol) end
end

-- to: computer ID of the flight computer, or "local".
function M:send_command(msg, to)
  msg.ap2 = "cmd"
  if to == "local" then
    os.queueEvent("ap2_local", msg)
    return true
  end
  if self.rednet and to then
    rednet.send(to, msg, self.protocol)
    return true
  end
  return false
end

function M:reply(to, msg)
  msg.ap2 = "reply"
  if to == "local" then
    os.queueEvent("ap2_local", msg)
  elseif self.rednet and to then
    rednet.send(to, msg, self.protocol)
  end
end

-- Decode an event from os.pullEvent. Returns kind, message, sender or nil.
function M:parse(ev)
  if ev[1] == "ap2_local" and type(ev[2]) == "table" then
    return ev[2].ap2, ev[2], "local"
  end
  if ev[1] == "rednet_message" and ev[4] == self.protocol and type(ev[3]) == "table" then
    return ev[3].ap2, ev[3], ev[2]
  end
  return nil
end

return M
