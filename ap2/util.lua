-- Small math helpers shared by every module. Lua 5.1/5.2 safe (no // or bit ops).
local M = {}

function M.clamp(v, lo, hi)
  if v < lo then return lo end
  if v > hi then return hi end
  return v
end

function M.sign(v)
  if v > 0 then return 1 end
  if v < 0 then return -1 end
  return 0
end

function M.round(v)
  if v >= 0 then return math.floor(v + 0.5) end
  return -math.floor(-v + 0.5)
end

function M.finite(v)
  return type(v) == "number" and v == v and v ~= math.huge and v ~= -math.huge
end

-- Angle helpers in degrees. wrap180 returns (-180, 180].
function M.wrap360(d)
  return d % 360
end

function M.wrap180(d)
  d = d % 360
  if d > 180 then d = d - 360 end
  return d
end

-- Propeller force grows with RPM squared. ssq maps RPM to signed force units,
-- ssqrt maps signed force units back to RPM.
function M.ssq(x)
  return x * math.abs(x)
end

function M.ssqrt(x)
  if x >= 0 then return math.sqrt(x) end
  return -math.sqrt(-x)
end

-- Move cur toward target by at most step.
function M.approach(cur, target, step)
  if cur < target then
    cur = cur + step
    if cur > target then cur = target end
  elseif cur > target then
    cur = cur - step
    if cur < target then cur = target end
  end
  return cur
end

function M.dot(a, b)
  return a.x * b.x + a.y * b.y + a.z * b.z
end

function M.cross(a, b)
  return {
    x = a.y * b.z - a.z * b.y,
    y = a.z * b.x - a.x * b.z,
    z = a.x * b.y - a.y * b.x,
  }
end

function M.len(a)
  return math.sqrt(M.dot(a, a))
end

-- Rotate vector v by quaternion q (normalised here). q is either CC:VS's
-- quaternion object (vector part q.v, scalar part q.a, what ship.getQuaternion
-- returns in game) or a plain {x, y, z, w}.
function M.rotate(q, v)
  local u = q.v or q
  local qx, qy, qz, qw = u.x, u.y, u.z, q.a or q.w
  local n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
  if n <= 0 or not M.finite(n) then
    return { x = v.x, y = v.y, z = v.z }
  end
  qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
  local tx = 2 * (qy * v.z - qz * v.y)
  local ty = 2 * (qz * v.x - qx * v.z)
  local tz = 2 * (qx * v.y - qy * v.x)
  return {
    x = v.x + qw * tx + (qy * tz - qz * ty),
    y = v.y + qw * ty + (qz * tx - qx * tz),
    z = v.z + qw * tz + (qx * ty - qy * tx),
  }
end

function M.fmt(v, decimals, width)
  local s
  if not M.finite(v) then
    s = "---"
  else
    s = string.format("%." .. (decimals or 0) .. "f", v)
    if s:match("^%-0%.?0*$") then s = s:sub(2) end
  end
  if width and #s < width then s = string.rep(" ", width - #s) .. s end
  return s
end

-- Compass value as three digits, 000-359 ("---" when unknown).
function M.compass(v)
  if not M.finite(v) then return "---" end
  return string.format("%03d", M.round(v) % 360)
end

return M
