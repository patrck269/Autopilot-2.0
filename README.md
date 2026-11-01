# Autopilot 2.0

A ComputerCraft autopilot for a Valkyrien Skies airship driven by Clockwork propellers through Create rotation speed controllers. It climbs to a typed block height and holds it, learning hover power on its own at any height. It flies a bearing at a set speed up to 35 m/s, with the target speed reached through a ramp that arrives within 15 s, and slows down with reverse thrust. A command center shows fighter-style MFDs on monitors.

Needs CC: Tweaked and CC:VS (`ship` and `aero` APIs) on the ship's computers.

## Hardware

| Controller | Role | Cap |
|---|---|---|
| `Create_RotationSpeedController_17` | Rear thrusters. Positive RPM pushes forward; negative is reverse thrust | ±1500 |
| `Create_RotationSpeedController_16` | Elevation | ±2750 |
| `Create_RotationSpeedController_13` | Bow port puller | ±1000 |
| `Create_RotationSpeedController_14` | Bow starboard puller | ±1000 |
| `Create_RotationSpeedController_15` | Stern port puller | ±1000 |
| `Create_RotationSpeedController_12` | Stern starboard puller | ±1000 |
| `Create_Speedometer_1` | Main engine speed | |
| `Create_Stressometer_2` | Main engine stress | |

Names, caps, and a per-controller `sign` flip are in `ap2/config.lua`. The MFD monitors default to `monitor_4` through `monitor_7`.

## Install

On each computer, run one line in the shell (not the Lua prompt), with that computer's role:

```
wget run https://raw.githubusercontent.com/patrck269/Autopilot-2.0/master/install.lua flight
```

The installer looks up the latest commit, downloads every file from that exact commit, sets `role` in `ap2/config.lua`, and reboots. If any download fails, nothing on the computer changes. An existing `startup.lua` or `ap2/config.lua` that differs from the new one is kept as `.bak`, so copy any peripheral-name edits back from `ap2/config.lua.bak`. Re-run the same line to update. Add a commit SHA after the role to install that exact version. The installed commit is written to `ap2/VERSION`.

The role can also be changed later in `ap2/config.lua`, or passed to the program (`startup flight`):

| Role | Computer | Needs |
|---|---|---|
| `flight` | Flight computer | Wired modem to the six controllers and both gauges, plus a modem for the command link |
| `command` | Command center | A modem for the link, plus advanced monitors |
| `both` | One computer does both | Everything above except a separate link modem |

At start-up each program checks every modem and peripheral it needs. Anything missing is listed on the terminal, and on the MFDs when they can be drawn. The flight computer writes nothing to the controllers until the list is clear, then starts on its own as soon as the missing device is attached.

Before the first flight, check controller direction with DEBUG (every prop at +15 RPM): positive RPM must pull on all four RCS corners and push the rear thrusters forward. A controller that turns backwards gets `sign = -1`. The bow axis doesn't have to be right in `frame.forward` (default +X). The flight computer checks it against which way the rear thrusters push the hull, and which way the starboard pullers pull it. Either one is enough. It snaps the result to the block grid; if that differs, it switches to it while keeping the ship pointing where it is, and remembers it. SYS shows "BOW AXIS CORRECTED: HEADING NOW TRUE", and CAL shows the axis as learned, or as "config, confirmed". Until the axis is confirmed, the sideways brake is held to 20%.

## Flying

| Command (terminal) | MFD | Effect |
|---|---|---|
| `alt 150` | AP: ALT ±1/±10, UFC: ALT SET | Target altitude (world Y) |
| `brg 90` | AP: BRG ±5, HSI: BRG ±1/±10/SYNC, UFC: BRG SET | Target bearing, 0 = N, 90 = E |
| `spd 30` | AP: SPD ±1/±5, UFC: SPD SET | Target speed, clamped to 0–35 m/s |
| `hover` | HOVR | Climb or descend to the altitude and hold it. Brake to a stop, hold heading, cancel drift |
| `cruise` | CRZ | Altitude, bearing, and speed together |
| `stby` | STBY | Autopilot off: rear thrust and RCS to 0, elevation left as it is |
| `kill` | SYS: KILL then CONF KILL | Every controller to 0 immediately |
| `debug` | SYS: DEBUG then CONF DEBUG | Every controller at +15 RPM as written to the block (config `sign` not applied), to see which way each prop pushes. **The elevation drops to 15 RPM too, so an airborne ship falls.** Learning is paused. Leave with any other mode |
| `sync` | PFD: SYNC | Targets become the current altitude and heading |
| `recal` | CAL: RECAL then CONF | Forget the hover calibration. In a steady hover it restarts from the current RPM |

If the flight computer reboots while the elevation is spinning, it takes over in HOVER at the current altitude.

## MFDs

Each monitor is one MFD with option-select buttons around the edge. Top-row and side buttons belong to the page on screen. The bottom row is **MENU**, three format buttons (the lit one is shown), and **SWAP**.

- A format button shows that page. Pressing the lit one, or MENU, opens the format menu, which lists every page.
- A page is only shown on one MFD at a time. Picking a page that another MFD is showing moves it here and sends this MFD's old page there.
- SWAP exchanges two MFDs: 1↔2 and 3↔4, with an odd last display swapping with display 1.
- The layout is saved and comes back after a reboot.

| Page | Shows |
|---|---|
| PFD | Attitude (pitch ladder, roll), heading and bearing, ground and forward speed, altitude, target, vertical speed, altitude and speed phases |
| AP | Mode, every target against its actual value, bearing error, speed reference, hover RPM here and at the target, alerts |
| HSI | Heading-up compass rose with bearing bug and track, ground speed, drift, X/Z position |
| ENG | Main engine RPM, stress against capacity, each controller's applied and desired RPM against its cap. Orange: held by the stress limit. Magenta: changed by another computer. Red: missing |
| CAL | Ground or airborne, sample count, hover RPM here and at target, air density, lift constant, altitude trim, mass, learned thrust and drag, learned RCS turning and sideways strength, bow axis (learned or config) |
| SYS | Link health, start-up errors and warnings, live alerts, event log, DEBUG, KILL |
| UFC | Keypad: type a number, then ALT, BRG, or SPD SET |

## How it flies

**Hover calibration.** Clockwork blade thrust is `0.5·ρ·v_tip²·C_L·area`, and tip speed is proportional to RPM, so lift is `k·ρ(y)·RPM²`. Inflow from climbing is negligible next to tip speed. The flight computer measures vertical acceleration over 1-second windows against the RPM actually on the controller and refines the single constant `k`. Hover RPM at any height is then `sqrt(g / (k·ρ(h)))`, with `ρ` from `aero.getAirDensity`. On the ground, while asked to climb, RPM steps up until the hull lifts. The learned values are saved to `/ap2_cal.dat`.

**Altitude.** The altitude error sets a vertical-speed command. That command is capped at 6 m/s up and 5 m/s down, and further limited near the target so the ship can still stop in time. A vertical-speed loop turns it into an acceleration, and the calibration turns that acceleration into RPM. Negative elevation RPM pulls down when the airborne hull needs more than gravity to slow a climb. Once settled within 1 block, the elevation is held at the calibrated hover RPM (plus its altitude trim) and only rewritten if the hull drifts past 1.5 blocks or starts moving.

**Speed.** The speed reference ramps to the target in 12 s and stops 0.5 m/s under the 35 m/s cap. Control uses normalised force (`RPM²`) with acceleration and drag learned in flight. Slowing down uses negative RPM on the rear thrusters. If acceleration pitches the nose past `pitch_limit` (6°), thrust is held back and the ramp pauses until the nose recovers. Once at cruise speed (within 0.3 m/s), the rear thrust is held at the RPM that balances drag and only recomputed if the speed drifts 1 m/s. Stopped (HOVER), the rear thrusters rest until drift passes 0.8 m/s, brake it under 0.2 m/s, and rest again.

**Heading and sideslip.** The RCS only pulls; no controller ever reverses. For a turn force `u` and a sideways force `s`, each corner gets `max(0, …)` of: bow starboard `u+s`, stern port `u−s`, bow port `−u−s`, stern starboard `s−u`. A pure turn is a diagonal pair (right: bow starboard + stern port; left: bow port + stern starboard), a pure strafe is a same-side pair (both starboard corners pull to starboard), and blends stay proportional. One corner on its own mostly strafes the hull, because the Eureka helm's stabilisation resists the turn. So RCS changes are written as one batch: corners turning down first, then corners turning up, each group on consecutive ticks and only started when the whole group fits the budget.

The bearing error sets a yaw-rate command, and a PI loop on yaw rate (taken from the change in heading between samples) gives a yaw acceleration. The directions are fixed: positive RPM pulls on every corner. Only the turning and sideways *strength* is learned in flight, and commands are divided by it. If the RCS pulls at full for 3 s and the heading is still off, SYS shows "RCS AT FULL: HEADING NOT HELD".

Station keeping: a heading correction starts when the heading is more than 2° off (1° in CRUISE) or the hull turns faster than 0.5°/s, and stops within 0.5°. In between, the RCS holds the trim its integral learned (an off-centre thrust line, say) or rests. Sideways drift over 0.5 m/s, including drift the ship already had when the autopilot took over, is braked with the same-side pair until it is under 0.15 m/s; then the sideways force rests. The smallest RCS pulse is 100 RPM. Heading gets first call on the RCS.

Clockwork props don't follow the controller at once: the bearing changes spin by at most 32 RPM a tick, so 0 → 1000 RPM takes about 1.6 s, and spinning down takes as long. The flight computer models that spin. Every learner uses the force the props are really making, not the commanded RPM. The heading loop gets a lead for turn force still on its way, and the sideslip brake lets go early when the remaining spin will finish the job.

**Controller protection.** In Create 6.0.8 a rotation speed controller is destroyed when its flicker score passes 128; the score decays 1 per tick (20 a second). A target change detaches the controller and re-attaches it. Every block that gets its rotation *through* that controller drops to 0 (+5) and comes back (+5): its own props, and every controller downstream of it. On this ship the four RCS controllers are fed through the elevation controller but not through each other. So an elevation write costs the elevation and all four RCS +10 each, and an RCS write costs only itself. Every switch of the engine network between overstressed and not also costs every block +5. The layout is `actuator.downstream` in `ap2/config.lua`. The flight computer:

- models every controller's flicker score. A write charges 10 to the controller and everything downstream of it, plus 10 to every controller near the stress limit or with no stress reading. No write may take any score past 60. RCS and rear-thrust increases leave 20 of each RCS score free, so the elevation can always be written. KILL bypasses the budget; its zero writes add at most about 30, so the worst case stays near 90, under Create's 128
- writes each controller at most once a second. It skips changes under 5 RPM (elevation), 15 (rear thrust) and 100 (RCS), but always writes a start or a stop. A running RCS batch goes first, then the elevation, then the largest need. Sign reversals pass through 0
- holds still when nothing needs doing: two minutes of steady hover in the test world make no writes at all
- reads the stressometer and learns each controller's stress per RPM. It never lets an increase push the engine past 85% of capacity, and the elevation gets first call on that room. If the engine is overstressed anyway, rear thrust and RCS are cut to 0 and the elevation is brought back under the limit
- re-reads every controller's target. If something else changed one (a player, another computer, the old autopilot), it leaves that controller alone for 5 s and shows it on SYS and ENG
- stops writing if another Autopilot 2.0 flight computer on the same ship is heard. The lower computer ID keeps control

If the engine can't hold a hover inside 85% at the target height, the ship stays lower (where the air is denser) and SYS shows "STRESS LIMIT: … HELD". Learning pauses while the engine is overstressed.

Before flying with Autopilot 2.0, make sure no other program writes to these controllers: shut down or clear the old autopilot computers, including its watchdog.

## Limits

- No controller can pitch the hull: the pullers are horizontal and the elevation is one shared controller. Nose dip is handled by limiting rear thrust, which can stretch an acceleration past 15 s on a hull that dips hard.
- RCS turning force is limited: at the 1000 RPM cap, a pulling pair gives about a sixth of what 2500 RPM pushing and pulling did. If the rear thrust line is far off the centre of mass, the heading drifts during hard acceleration (SYS shows "RCS AT FULL").
- Gains are tuned against a simulated hull. In-flight learning covers lift, thrust, drag, and RCS strength and direction; the remaining gains are in `ap2/config.lua` if the real hull needs different ones.

## Tests

```
lua tests/test_flight.lua
lua tests/test_install.lua
python tests/cc/run.py <path to CraftOS-PC_console.exe>
```

`tests/test_flight.lua` flies the real control modules against `tests/world.lua`, a hull model with the Clockwork thrust law, air density, drag, RCS yaw, thrust misalignment, nose dip, prop lag, ground contact, and Create's flicker score. It checks liftoff, climbing and holding at several heights from wrong calibrations, cruise to 30 m/s within 15 s, the 35 m/s cap, reverse-thrust braking, heading hold, the pitch limit, KILL, and noisy telemetry.

`tests/cc/run.py` boots the real `startup.lua` in [CraftOS-PC](https://www.craftos-pc.cc) with fake Create peripherals and monitors. It presses MFD buttons, types terminal and keypad commands, sends rednet commands (including one for another ship), and checks the missing-peripheral path.
