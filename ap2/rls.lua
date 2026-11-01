-- Recursive least squares with forgetting for linear models
--     y = t[1] * x[1] + ... + t[n] * x[n]
-- Used to learn thrust/drag, RCS authority and the hull's axes in flight.
local util = require("ap2.util")

local M = {}
M.__index = M

-- theta: starting parameters; p0: starting variance per parameter;
-- lambda: forgetting factor.
function M.new(theta, p0, lambda)
  local n = #theta
  local self = setmetatable({ n = n, t = {}, p0 = {}, lambda = lambda or 0.98, count = 0 }, M)
  for i = 1, n do
    self.t[i] = theta[i]
    self.p0[i] = p0[i]
  end
  self:reset_variance()
  return self
end

function M:reset_variance()
  local P = {}
  for i = 1, self.n do
    P[i] = {}
    for j = 1, self.n do P[i][j] = (i == j) and self.p0[i] or 0 end
  end
  self.P = P
end

-- One sample: inputs x (list of n numbers) and output y. Returns the error.
function M:update(x, y)
  local n, P, lambda = self.n, self.P, self.lambda
  local px = {}
  local denom = lambda
  for i = 1, n do
    local s = 0
    for j = 1, n do s = s + P[i][j] * x[j] end
    px[i] = s
    denom = denom + x[i] * s
  end
  local err = y
  for i = 1, n do err = err - self.t[i] * x[i] end
  local k = {}
  for i = 1, n do k[i] = px[i] / denom end
  local t = {}
  local ok = true
  for i = 1, n do
    t[i] = self.t[i] + k[i] * err
    if not util.finite(t[i]) then ok = false end
  end
  -- P = (P - k * (x' P)) / lambda; x' P = px' because P is symmetric.
  local trace = 0
  for i = 1, n do
    for j = 1, n do
      P[i][j] = (P[i][j] - k[i] * px[j]) / lambda
      if not util.finite(P[i][j]) then ok = false end
    end
    trace = trace + P[i][i]
  end
  -- Forgetting with no excitation lets P grow without bound; cap it.
  local trace0 = 0
  for i = 1, n do trace0 = trace0 + self.p0[i] end
  if not ok or trace > 12.5 * trace0 then self:reset_variance() end
  if ok then
    self.t = t
    self.count = self.count + 1
  end
  return err
end

return M
