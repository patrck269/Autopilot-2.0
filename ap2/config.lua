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
    bow_port   = { name = "Create_RotationSpeedController_13", cap = 1000, sign = 1, label = "RSC13 BOW PORT" },
    bow_stbd   = { name = "Create_RotationSpeedController_14", cap = 1000, sign = 1, label = "RSC14 BOW STBD" },
    stern_port = { name = "Create_RotationSpeedController_15", cap = 1000, sign = 1, label = "RSC15 STRN PORT" },
    stern_stbd = { name = "Create_RotationSpeedController_12", cap = 1000, sign = 1, label = "RSC12 STRN STBD" },
  },
  speedometer = "Create_Speedometer_1",   -- main engine speed
  stressometer = "Create_Stressometer_2", -- main engine stress

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

  -- Rotation speed controller protection (see ap2/actuators.lua for how Create
  -- destroys them). Create breaks a controller above 128 flicker points; the
  -- score decays 1 per tick (20 a second). A write can cost every controller
  -- that shares its input shaft 10 points, so the whole ship shares one budget:
  -- about two writes a second in total.
  actuator = {
    flicker_cost = 10,    -- shared points per write with the engine clearly below its limit
    flicker_cost_risky = 20, -- per write near the stress limit (overstress may flip) or with no reading
    flicker_decay = 20,   -- points recovered per second
    flicker_limit = 50,   -- writes wait while the shared score would pass this. KILL bypasses
                          -- the budget (up to 6 zero writes = +60), so the worst case stays at 110
    lift_reserve = 20,    -- top of the budget kept for the elevation
    min_interval = { lift = 1.0, thrust = 1.0, rcs = 1.0 }, -- seconds between writes to one controller
    deadband = { lift = 5, thrust = 15, rcs = 100 },        -- smallest RPM change worth a write
    weight = { lift = 3, thrust = 2, rcs = 1.5 },            -- who gets the next write
    slew = { thrust = 400, lift = 300, rcs = 1000 },         -- largest RPM change per write
    stress_high = 0.85,   -- increases may not push engine stress past this share of capacity
    stress_quiet = 0.75,  -- below this share a write cannot flip overstress
    foreign_hold = 5,     -- seconds to leave alone a controller something else changed
    rcs_min_rpm = 150,    -- smaller RCS commands are sent as 0
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
    hold_band = 1.0,      -- settled within this many blocks (and |vy| < 0.25): lift RPM is held
    hold_release = 1.5,   -- held lift RPM is recomputed once the hull drifts this many blocks
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
    cap_margin = 0.5,     -- the speed reference stops this far under the cap
    ramp_time = 12,       -- reference reaches the new target in this many s (requirement: within 15 s)
    min_ramp = 0.3,       -- slowest reference ramp, m/s^2
    vel_gain = 0.8,       -- (m/s^2) per m/s of speed error
    int_gain = 0.15,      -- integral (m/s^2) per (m/s * s)
    int_limit = 2.0,      -- integral limit, m/s^2
    stop_band = 0.2,      -- stopped: brake drift down to this (m/s), then the rear thrusters rest
    hover_engage = 0.8,   -- stopped: drift (m/s) that wakes the rear thrusters again
    hold_band = 0.3,      -- cruising within this of the target speed (m/s): thrust is held
    hold_release = 1.0,   -- held thrust is recomputed once speed drifts this far (m/s)
    pitch_limit = 6,      -- degrees of acceleration-induced nose dip before thrust is held back
    -- Thrust model a = A*f - D*v|v| (f = normalised force, rpm^2/cap^2), learned in flight.
    accel_full = 1.5,     -- first guess: m/s^2 at full rear thrust
    top_speed = 40,       -- first guess: speed where full thrust equals drag
  },

  heading = {
    bearing_gain = 0.4,   -- (deg/s) of yaw rate per degree of bearing error
    max_rate = 4,         -- max yaw rate, deg/s
    yaw_accel = 0.5,      -- yaw deceleration planned when arriving, deg/s^2 (RCS updates lag ~1-2 s)
    rate_gain = 0.4,      -- (deg/s^2) per deg/s of yaw-rate error
    int_gain = 0.3,       -- integral (deg/s^2) per (deg/s * s)
    int_limit = 3,        -- integral limit, deg/s^2
    -- Station keeping: a correction starts when the heading is off by more than
    -- the deadzone (or the hull turns faster than engage_rate), and stops once
    -- within settle_deg and turning slower than settle_rate. In between the RCS
    -- holds a steady trim against any steady torque, or rests.
    hover_deadzone = 2,   -- degrees, HOVER
    cruise_deadzone = 1,  -- degrees, CRUISE
    settle_deg = 0.5,
    settle_rate = 0.2,    -- deg/s
    engage_rate = 0.5,    -- deg/s
    high_speed = 20,      -- above this ground speed the heading integral works harder
    high_speed_gain = 2.0,
    -- RCS authority is learned in flight, direction included. These are the
    -- first guesses at full RCS; the direction is confirmed within a few seconds.
    yaw_authority = 1.0,  -- deg/s^2 of yaw at full RCS
    min_authority = 0.1,  -- smallest yaw authority believed (deg/s^2)
    probe_limit = 0.2,    -- RCS share allowed until the direction is confirmed
  },

  -- DEBUG mode RPM for every controller (written to the block as is).
  debug_rpm = 15,

  -- Files on the flight computer.
  cal_file = "/ap2_cal.dat",
  mfd_file = "/ap2_mfd.dat",
}
