-- Two-parameter recursive least squares with forgetting, for models of the
-- form y = t1 * x1 + t2 * x2. Used to learn thrust/drag and RCS authority.
local util = require("ap2.util")

local M = {}
M.__index = M

-- theta: starting {t1, t2}; p0: starting variance {p1, p2}; lambda: forgetting.
function M.new(theta, p0, lambda)
  return setmetatable({
    t = { theta[1], theta[2] },
    p0 = { p0[1], p0[2] },
    P = { p0[1], 0, 0, p0[2] },
    lambda = lambda or 0.98,
    n = 0,
  }, M)
end

function M:update(x1, x2, y)
  local P, lambda = self.P, self.lambda
  local p1 = P[1] * x1 + P[2] * x2
  local p2 = P[3] * x1 + P[4] * x2
  local denom = lambda + x1 * p1 + x2 * p2
  local k1, k2 = p1 / denom, p2 / denom
  local err = y - (self.t[1] * x1 + self.t[2] * x2)
  local t1, t2 = self.t[1] + k1 * err, self.t[2] + k2 * err
  local r1 = x1 * P[1] + x2 * P[3]
  local r2 = x1 * P[2] + x2 * P[4]
  P[1] = (P[1] - k1 * r1) / lambda
  P[2] = (P[2] - k1 * r2) / lambda
  P[3] = (P[3] - k2 * r1) / lambda
  P[4] = (P[4] - k2 * r2) / lambda
  -- Forgetting with no excitation lets P grow without bound; cap it.
  if P[1] + P[4] > 12.5 * (self.p0[1] + self.p0[2]) or not util.finite(P[1] + P[2] + P[3] + P[4]) then
    self.P = { self.p0[1], 0, 0, self.p0[2] }
  end
  if util.finite(t1) and util.finite(t2) then
    self.t[1], self.t[2] = t1, t2
    self.n = self.n + 1
  end
  return err
end

return M
