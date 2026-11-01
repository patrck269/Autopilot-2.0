-- Autopilot 2.0 configuration. Edit peripheral names and tuning here.
-- Every value is read at program start; reboot the computer after editing.
return {
  -- "flight":  this computer drives the controllers (flight computer).
  -- "command": this computer runs the MFD command center.
  -- "both":    one computer does both (flight control plus MFDs).
  role = "flight",

  -- rednet protocol shared by flight and command computers.
  protocol = "ap2",
  -- Optional: lock the command center to one flight computer ID. nil = the
  -- flight computer on the same VS ship (matched by ship ID), else the first heard.
  flight_id = nil,

  -- Rotation speed controllers on the flight computer's wired network.
  -- cap is the absolute RPM limit; sign flips a controller that runs backwards.
  controllers = {
    thrust     = { name = "Create_RotationSpeedController_17", cap = 1500, sign = 1, label = "RSC17 REAR THR" },
    lift       = { name = "Create_RotationSpeedController_16", cap = 2750, sign = 1, label = "RSC16 ELEV" },
    bow_port   = { name = "Create_RotationSpeedController_13", cap = 2500, sign = 1, label = "RSC13 BOW PORT" },
    bow_stbd   = { name = "Create_RotationSpeedController_14", cap = 2500, sign = 1, label = "RSC14 BOW STBD" },
    stern_port = { name = "Create_RotationSpeedController_15", cap = 2500, sign = 1, label = "RSC15 STRN PORT" },
    stern_stbd = { name = "Create_RotationSpeedController_12", cap = 2500, sign = 1, label = "RSC12 STRN STBD" },
  },
  speedometer = "Create_Speedometer_0",   -- main engine speed
  stressometer = "Create_Stressometer_0", -- main engine stress

  -- Command center monitors used as MFDs, in display order. SWAP exchanges
  -- displays 1<->2 and 3<->4 (an odd last display swaps with display 1).
  -- An empty list uses every attached monitor.
  monitors = { "monitor_4", "monitor_5", "monitor_6", "monitor_7" },
  monitor_scale = 0.5,

  -- Ship-local axes as they were when the hull was assembled. forward points
  -- out of the bow, up out of the deck. Compass bearing: 0 = north (-Z),
  -- 90 = east (+X), 180 = south, 270 = west.
  frame = {
    forward = { x = 1, y = 0, z = 0 },
    up = { x = 0, y = 1, z = 0 },
  },

  loop_period = 0.1,    -- control loop period, seconds
  status_period = 0.25, -- telemetry broadcast period, seconds
  gravity = 10,         -- VS gravity, m/s^2

  -- Rotation speed controller protection. Every target change rebuilds the
  -- Create kinetic network and adds flicker points (Create breaks the block
  -- above 128; the score decays 1 per tick). Writes are paced to stay far
  -- below that, and a sign reversal always passes through 0 first.
  actuator = {
    min_interval = 0.5,   -- seconds between writes to one controller
    flicker_cost = 10,    -- points charged per write
    flicker_decay = 20,   -- points recovered per second
    flicker_limit = 60,   -- writes wait while the score would pass this
    deadband = { thrust = 4, lift = 2, rcs = 10 }, -- smallest RPM change worth a write
    slew = { thrust = 400, lift = 300, rcs = 2500 }, -- largest RPM change per write
    rcs_min_rpm = 60,     -- smaller RCS commands are sent as 0
  },

  altitude = {
    climb_rate = 6,       -- max climb, m/s
    descent_rate = 5,     -- max descent, m/s
    brake_accel = 1.0,    -- deceleration planned when arriving, m/s^2
    pos_gain = 0.5,       -- (m/s) per block of altitude error
    vel_gain = 1.0,       -- (m/s^2) per m/s of vertical speed error
    max_accel = 3,        -- largest commanded climb acceleration, m/s^2
    min_accel = -6,       -- largest commanded sink acceleration (reverse lift below -g)
    int_gain = 0.02,      -- altitude integral, (m/s^2) per block-second
    int_limit = 1.0,      -- integral limit, m/s^2
    int_zone = 6,         -- integral runs inside this many blocks
    hold_band = 1.0,      -- "HOLD" once within this many blocks and |vy| < 0.4
    reverse_lift = true,  -- allow negative elevation RPM (pull down) when airborne
    -- Hover calibration. Lift = k * airDensity(y) * rpm^2, learned in flight.
    default_hover_rpm = 430, -- first guess at sea-level density, before any learning
    cal_window = 1.0,     -- seconds per calibration sample
    cal_rate = 0.5,       -- fraction of each sample applied (0..1)
    cal_max_vy = 15,      -- samples taken only while |vy| is below this, m/s
    liftoff_step = 0.88,  -- grounded and asked to climb: k *= this per window (+6.6% RPM)
    liftoff_accel = 1.0,  -- climb acceleration asked for while still on the ground, m/s^2
  },

  speed = {
    max = 35,             -- hard speed cap, m/s
    ramp_time = 12,       -- reference reaches the new target in this many s (requirement: within 15 s)
    min_ramp = 0.3,       -- slowest reference ramp, m/s^2
    vel_gain = 0.8,       -- (m/s^2) per m/s of speed error
    int_gain = 0.15,      -- integral (m/s^2) per (m/s * s)
    int_limit = 2.0,      -- integral limit, m/s^2
    stop_band = 0.3,      -- below this speed with target 0 the rear thrusters idle
    pitch_limit = 6,      -- degrees of acceleration-induced nose dip before thrust is held back
    -- Thrust model a = A*f - D*v|v| (f = normalised force, rpm^2/cap^2), learned in flight.
    accel_full = 1.5,     -- first guess: m/s^2 at full rear thrust
    top_speed = 40,       -- first guess: speed where full thrust equals drag
  },

  heading = {
    bearing_gain = 0.4,   -- (deg/s) of yaw rate per degree of bearing error
    max_rate = 6,         -- max yaw rate, deg/s
    yaw_accel = 1.5,      -- yaw deceleration planned when arriving, deg/s^2
    rate_gain = 0.12,     -- normalised RCS force per deg/s of yaw-rate error
    int_gain = 0.02,      -- integral force per (deg/s * s)
    int_limit = 0.4,
    deadband = 0.25,      -- degrees; inside it (and nearly still) RCS rests
    sway_gain = 0.08,     -- normalised RCS force per m/s of sideslip
    high_speed = 20,      -- above this ground speed the RCS gains are scaled up
    high_speed_gain = 2.0,
  },

  -- Files on the flight computer.
  cal_file = "/ap2_cal.dat",
  mfd_file = "/ap2_mfd.dat",
}
