-- Autopilot 2.0 entry point. Runs at boot; the role comes from ap2/config.lua
-- or from the first argument: startup flight | startup command | startup both
package.path = "/?.lua;/?/init.lua;" .. package.path

local cfg = require("ap2.config")
local role = (...) or cfg.role
local link_mod = require("ap2.link")

if role == "flight" then
  local link = link_mod.new(cfg, { loopback = false })
  require("ap2.flight").run(cfg, { link = link, role = role, terminal = true })
elseif role == "command" then
  local link = link_mod.new(cfg, { loopback = false })
  require("ap2.command").run(cfg, { link = link, role = role, terminal = true })
elseif role == "both" then
  local link = link_mod.new(cfg, { loopback = true })
  parallel.waitForAny(
    function() require("ap2.flight").run(cfg, { link = link, role = role, terminal = false }) end,
    function() require("ap2.command").run(cfg, { link = link, role = role, terminal = true }) end
  )
else
  printError("Unknown role '" .. tostring(role) .. "'. Use flight, command or both.")
end
