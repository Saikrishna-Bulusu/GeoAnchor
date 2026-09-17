# The whole system, on a fixed wing, in Gazebo

One command puts a fixed-wing aircraft with a nadir camera over real satellite
imagery of an Australian regional town, matches every frame against a **different,
older** satellite capture of the same ground, and feeds the resulting fixes to
PX4's EKF2 with GNSS available but the fix arriving as an independent absolute
position source.

```bash
bash scripts/sim_fixedwing.sh
```

Everything below is how to get to the point where that command works, and what
each piece is doing.

---

## What this rig is for, and what the other rigs are for

Every other harness in this repo exercises one seam:

| rig | seam |
|---|---|
| `scripts/sitl_openloop.py` | the MAVLink write path into ArduPilot |
| `scripts/px4_sitl.sh` | PX4's estimator, with a synthetic fix |
| `scripts/verify_architecture.py` | the three-layer boundaries |
| the replay harness | the matcher, on recorded frames |
| **`scripts/sim_fixedwing.sh`** | **all of them at once, with nothing stubbed** |

This is the only place where a camera on a moving aircraft produces frames that
become fixes that reach a flight controller's estimator, with no stage replaced
by a recording or a stub.

## The ground is NOT the reference map

This is the single most important thing about this rig, and it is enforced in
code — `sim_fixedwing.sh` exits if the two paths are equal.

    ground texture   data/sim_rural/ref_tile_2026-08-05.tif
    reference map    data/sim_rural/ref_tile_2024-06-06.tif

Texturing the simulated ground with the same image the pipeline matches against
makes the pipeline match a picture to itself. It then reports centimetres,
which say nothing about whether the system works and everything about floating
point. It is a hard rule in `CLAUDE.md` for that reason.

Because the two captures are 2.2 years apart, **this run is also a genuine
cross-date test**. `results/crossdate_cities_2026-09-17.md` measured this exact
area and this exact pair, tile against tile:

| capture | gap | inliers | solved | median |
|---|---|---|---|---|
| 2026-01-29 | 0.5 yr | 443 | 24/24 | 0.02 m |
| **2024-06-06** | **2.2 yr** | **165** | **24/24** | **0.40 m** |
| 2023-06-29 | 3.1 yr | 167 | 24/24 | 0.45 m |
| 2017-10-25 | 8.8 yr | 118 | 24/24 | 0.53 m |

**Those numbers are NOT a target for this rig.** They come from matching a
512 px crop of one tile against another tile — no camera, no projection, no
renderer. Here a simulated camera looks at a textured 3D plane through a
perspective transform and a mipmap chain, so the frame is a resampled view of
the texture rather than the texture. Expect fewer inliers, and read the table
as evidence the *area and the date gap* are workable, not as a score to match.

What the table is genuinely good for is the opposite direction: **if this rig
ever reports centimetres and hundreds of inliers, the ground and the reference
have become the same file.** The runner refuses the obvious version of that
mistake, but only for files passed through it.

## Why Griffith and not a capital city

The same cross-date study found the opposite of the intuition. Buildings do not
move, so a built-up area ought to be the stable one. The *tallest* built-up
areas are the hard case — Brisbane, Perth and Melbourne CBDs all fail at gaps
of months, while Griffith works out to 8.8 years. What changes over a CBD
between two satellite passes is not the ground, it is the apparent geometry of
tall structures: facade parallax with view angle, shadow with sun angle and
season. Both scale with height, and Griffith's centre is one and two storeys.

A demonstration flown over a CBD would therefore fail for a reason that has
nothing to do with this pipeline.

**A note on the name.** The study calls this area `rural_griffith` and
originally described it as farmland. It is not: the tile is 2.17 km of
suburban streets, shop awnings and industrial sheds, which is what you will see
on the camera feed. The label was wrong and is corrected in that document. The
comparison it supports is *low-rise town vs high-rise CBD*, which isolates
building height. **Nothing in this project has measured open country**, so if
your flight area is farmland, treat it as untested.

## What it produced

`results/sim_fixedwing_2026-09-17.md`, 139 accepted fixes:

    error   median 3.35 m, p90 6.32, p99 9.89, max 10.30
            73% within 5 m, 99% within 10 m, 100% within 20 m
    inliers median 86 (9-128)
    latency median 97.6 ms, p95 159.8 ms against a 250 ms budget

and in PX4:

    observation_variance  [64.0, 64.0]   <- the 8.0 m sigma, squared, intact
    fused                 True
    cs_aux_gpos           True

**Do not compare that median with the env80 table in `CLAUDE.md`.** A rendered
view of a flat textured plane has no relief displacement, no cloud, no seasonal
change and no exposure difference, which is why this run contains no
catastrophic fixes and real satellite runs do.

## Is it actually matching? A check that isolates the renderer

The rig has three things that can each look like "the matcher is broken": the
renderer, the date gap, and the flight path. This separates them, and it is
worth running before believing any negative result.

Crop the reference at the aircraft's **current** position — not the tile centre
— and match a live frame against both tiles:

```bash
curl -s -o /tmp/f.jpg http://localhost:8000/api/frame.jpg
```

then, for each of the two tiles, crop an 800 px window centred on the lat/lon
from `px4-listener vehicle_global_position` and run any matcher. Measured
17 Sept 2026 at 91 m AGL, plain ORB:

| reference | inliers | what it means |
|---|---|---|
| `ref_tile_2026-08-05.tif` (textures the ground) | **268** | control — camera, projection, rectification and GSD scaling are all correct |
| `ref_tile_2024-06-06.tif` (the pipeline's map) | **96** | a genuine cross-date match, 2.2 years |

**And where it flies matters more than anything else.** The same check at two
positions inside this one tile:

| position | vs the ground's own imagery | vs the 2.2-year reference |
|---|---|---|
| commercial centre | 270 | **30** |
| industrial fringe, 800 m south | 268 | **96** |

Flat control column, 3.2x difference across dates. An acceptance rate from this
rig is a statement about the flight path as much as about the method — parking
on the town centre takes it to zero with every upstream stage working. That is
also why `NAV_LOITER_RAD` is 500 m: a tight orbit both banks the camera off
nadir and samples one patch of ground.

**Centre the crop on the aircraft, not on the tile.** The first version of this
check cropped the tile centre while the aircraft was 794 m away, got 5 inliers
against *both* tiles — including the one the ground is literally textured with —
and pointed at the renderer. The renderer was fine; the crop had no overlap.
Five inliers against the control tile means the two images are not of the same
ground, and nothing else.

---

## Requirements

| | version used | notes |
|---|---|---|
| Ubuntu | 24.04 | Python 3.12 system-wide |
| Gazebo | Harmonic, `gz-sim8` 8.15.0 | `sudo apt install gz-harmonic` |
| PX4 | v1.16.2 | built as `px4_sitl_default` |
| `MicroXRCEAgent` | any recent | only needed for the closed loop |
| this repo's venv | `.venv` | `uv venv && uv pip install -e .` |

### Gazebo's Python bindings are APT packages, and PYTHONPATH is the wrong way in

`gz.transport13` and `gz.msgs10` install as `python3-gz-*` under
`/usr/lib/python3/dist-packages` and a venv does not see them. Both
interpreters are 3.12 on 24.04, so the ABI matches and the system path simply
works.

**Do not reach for `PYTHONPATH`.** It goes in *front* of the venv's
site-packages, so the system `typing_extensions` shadows the venv's, `pydantic`
dies on `cannot import name 'Sentinel'`, `fastapi` fails to import, and the
dashboard never starts — with an error message about pydantic that says nothing
about Gazebo. `GzFeed.__init__` **appends** the path to `sys.path` instead, so
venv packages keep winning and the caller has to do nothing.

### ROS 2 shadows the `gz` binary, and PX4 hangs on it

If ROS 2 is installed, it puts a vendored `gz` shim first on `PATH`. The shim
reports `I cannot find any available 'gz' command` on a machine where Gazebo is
installed and working. PX4's `rcS` polls for the world with a bare `gz topic -l`,
gets that, and sits in **`Waiting for Gazebo world...` forever** against a world
that is up and publishing.

`sim_fixedwing.sh` resolves the real binary and puts its directory first on
`PATH` before starting anything, because resolving it for its own use is not
enough — every child process needs the same `PATH`.

---

## Building the pieces

Each step is idempotent and only needs running once.

### 1. PX4 SITL

```bash
git clone --depth 1 --branch v1.16.2 https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot && make px4_sitl_default
```

Point the runner at it with `--px4 /path/to/PX4-Autopilot`, or set `PX4_DIR`.
It defaults to `/home/sai/thesis_2.0/PX4-Autopilot`.

### 2. The reference imagery

Two captures of the same ground, from Esri World Imagery Wayback:

```bash
.venv/bin/python scripts/fetch_wayback_tile.py --list --lat -34.2896 --lon 146.0502
.venv/bin/python scripts/fetch_wayback_tile.py --lat -34.2896 --lon 146.0502 \
    --size 2200 --zoom 19 --release 2026-08-05 --out data/sim_rural/ref_tile_2026-08-05.tif
.venv/bin/python scripts/fetch_wayback_tile.py --lat -34.2896 --lon 146.0502 \
    --size 2200 --zoom 19 --release 2024-06-06 --out data/sim_rural/ref_tile_2024-06-06.tif
```

**Licence.** Esri World Imagery is not CC BY. These pixels are measurement-only
working data and cannot go into a paper. For a publishable figure use the NSW
Historical Imagery Viewer (CC BY), which also carries a real *acquisition* date
where a Wayback release date is when Esri published.

### 3. The Gazebo world

```bash
.venv/bin/python scripts/make_gz_world.py \
    --tif data/sim_rural/ref_tile_2026-08-05.tif --name geoanchor_rural
```

This writes `sim/gz/worlds/geoanchor_rural.sdf` plus a texture, and:

- sets the world's `spherical_coordinates` to the **image centre**, so PX4's
  simulated GPS and the ground agree. Getting this wrong offsets every fix by a
  constant nobody can see;
- sizes the ground plane `width_px * gsd` by `height_px * gsd` — for this tile
  2168.5 x 2159.2 m at 0.4926 m/px;
- **refuses a geographic CRS.** A tile in EPSG:4326 has degrees for units and
  the ground plane would come out 0.02 m across. Reproject to UTM first;
- loads `Magnetometer` and `AirSpeed` alongside the usual `Imu`, `AirPressure`
  and `NavSat` systems. Gazebo's *default* server config carries all of these,
  but a world that declares any plugin list gets only what it declares — and a
  missing magnetometer shows up as PX4's `Preflight Fail: Found 0 compass`,
  which reads like a model fault rather than a world one.

### 4. The aircraft

`sim/gz/models/geoanchor_cessna/` is committed, so nothing to build. It merges
two models PX4 already ships:

- `rc_cessna` — a fixed wing, because that is the airframe class this work
  targets;
- `mono_cam` — rotated to look straight down.

**The camera points down, and the `1.5707` rad pitch in both the include pose
and the joint is how.** Gazebo's camera looks along its own +X; pitching the
link by pi/2 turns +X into -Z. Getting this wrong points the camera at the
horizon, every frame matches nothing, and it reads as a matcher failure rather
than a mounting one.

---

## Running it

```bash
bash scripts/sim_fixedwing.sh                # everything, GUI
bash scripts/sim_fixedwing.sh --headless     # no GUI
bash scripts/sim_fixedwing.sh --no-pipeline  # simulator only, to look at it
bash scripts/sim_fixedwing.sh --open-loop    # no write to the flight controller
bash scripts/sim_fixedwing.sh --no-fly       # leave it on the ground for QGC
```

Seven steps:

0. **preconditions** — refuse to start on top of a live run, resolve the real
   `gz`, refuse ground == reference, read the world origin out of the SDF and
   hand it to PX4 as home.
1. **Gazebo** — start the world, wait for topics.
2. **PX4 SITL** — `PX4_SYS_AUTOSTART=4003` (rc_cessna), spawning
   `geoanchor_cessna` into `geoanchor_rural`.
3. **camera topic** — resolved from `gz topic -l` by matching `/image$`. Not
   `image|camera`: that also matches `camera_info`, which sorts first and
   carries no pixels.
4. **the pipeline** — all three layers, on the Gazebo feed.
5. **the AGP write path** — parameters, agent, bridge.
6. **launch** — mission upload, arm, climb into the envelope, then hold a
   120 m orbit over the map centre. The orbit is commanded with
   `DO_REPOSITION` + `AUTO.LOITER` rather than left to the mission's own
   `LOITER_UNLIM`, which this PX4 runs straight past.
7. **running** — dashboard at `http://localhost:8000`.

### Why the runner waits for `Startup script returned successfully`

Not for `Ready for takeoff`. That line waits on *every* arming check, including
`No connection to the ground control station`, which cannot pass until a GCS
connects and heartbeats — which is step 6. Waiting for it at step 2 deadlocks
for the full timeout on a PX4 that is perfectly healthy.

---

## How a fix reaches PX4: AGP, not external vision

**This is the finding that made the PX4 integration possible at all**, and it
is written up in `docs/px4_ekf2_extnav_2026-09-17.md`.

PX4's external-vision path has a hard rate floor. `EV_MAX_INTERVAL` is 200000
microseconds, so a fix arriving slower than 5 Hz never starts fusion. Measured
across four passes: 12 of 12 fused at 5 Hz and above, 4 of 4 failed at 4 Hz.
This pipeline runs at 0.3-5 Hz depending on matcher and board, so on a Pi it
sits below the floor and the EV path is closed to it.

**Aux Global Position has no rate check.** Its starting condition is a finite
lat/lon plus yaw alignment, and the only timing rule is a 5 s timeout. Measured
fusing at 2 Hz with GNSS denied.

The cost is that AGP is reachable **only over uXRCE-DDS** — there is no MAVLink
message for it. So the closed loop needs:

```
pipeline  --ZeroMQ-->  scripts/agp_bridge.py  --ROS 2-->  /fmu/in/aux_global_position
```

`agp_bridge.py` publishes `VehicleGlobalPosition` with BEST_EFFORT QoS at depth
1, sets `eph` to `max(sigma, EKF2_AGP_NOISE)`, and stamps `timestamp_sample`
with the **capture** time, never the fix-completion time.

Parameters go in from `configs/px4_agp.params` and
`configs/px4_sim_fixedwing.params`, applied at runtime by
`scripts/px4_set_params.py` on top of the stock `4003` (rc_cessna) airframe.
**Both files go to one invocation** — `--file a b` — because PX4 locks a
mavlink instance to the first peer and a second invocation gets `no heartbeat`
and applies nothing while the first still reports success.

`configs/px4_sim_fixedwing.params` holds two settings that are correct for a
simulator and **wrong for an aircraft**:

| param | value | why, and why only here |
|---|---|---|
| `MIS_TKO_LAND_REQ` | 0 | no landing item required. With the default 2, a landing item must clear both the loiter radius and the 8° `FW_LND_ANG` glide limit, which puts it ~740 m from the loiter centre — and PX4 then flies there, taking the camera off the middle of the map. A real aircraft wants the landing item. |
| `NAV_DLL_ACT` | 0 | no datalink-loss failsafe. The default RTL fires 10 s after `sim_fly.py` exits. On a real airframe, no failsafe is how you lose it. |

There is deliberately **no custom PX4 airframe file.** One existed and was
removed: it set the same four parameters a second time, which means two places
to edit and one of them silently losing. Runtime application also needs no PX4
rebuild, so this rig works against an unmodified PX4 checkout.

**Give the parameter script its own MAVLink link.** PX4's UDP mavlink locks to
one peer per instance, so a second client on the pipeline's port steals its
telemetry stream and then exits, leaving PX4 sending to a socket nobody holds —
and the data layer receives nothing for the rest of the run, with no error. The
runner uses the separate 14280/14030 instance for parameters.

| param | value | why |
|---|---|---|
| `EKF2_AGP_CTRL` | 1 | bit 0, horizontal only. **Defaults to 0 — AGP is off until this is set.** Bit 1 stays clear: this pipeline produces no altitude. |
| `EKF2_AGP_NOISE` | 0.1 | a *lower bound* on the `eph` supplied. The default 0.9 would silently floor every sigma tighter than 90 cm, which defeats the covariance estimator. |
| `EKF2_AGP_GATE` | 3.0 | innovation gate, sigma. Note this is tighter than the EV path's 5.0. |
| `EKF2_AGP_DELAY` | 50.0 | ms, **reboot required**. A placeholder. Replace with your own measured capture-to-publish median. |

### Confirming it actually fused

In the PX4 console:

```
listener estimator_aid_src_aux_global_position
```

`observation_variance` should be the square of the `eph` supplied — 8.0 m in,
`[64.0, 64.0]` out — and `estimator_status_flags` should show `cs_aux_gpos:
True`. Those two together are the proof the fix was consumed rather than merely
delivered.

### INT32 parameters arrive as a bit pattern

`scripts/px4_set_params.py` packs INT32 values as a reinterpreted bit pattern
in the float field, and `check_extnav.py` honours `param_type` when reading
back. Both directions matter, and the bug they cause cancels out: setting
`EKF2_AGP_CTRL` to 1 stored 1065353216, reading it back as a float said 1.0,
and the check printed `ok` while AGP was off.

---

## The dashboard

`http://localhost:8000`, live while the sim runs.

### Adding a map, the way a user on the ground would

`POST /api/map` takes a GeoTIFF upload, reports the georeferencing it found,
and **warns when the file is not projected**. A plain image renamed `.tif`
loads without complaint and matches against nothing, so the endpoint reads the
CRS rather than the extension. Upload from the control bar; the dashboard asks
for confirmation before switching the live reference.

### The learned covariance estimator, as a switch

It is plug-and-play — `GET /api/covariance` lists every trained model with the
matcher it was `trained_on` and whether it `matches_current`, and the control
bar shows a warning when they disagree. That warning is the whole point of the
project's headline finding: **a system that swaps matchers silently inherits a
covariance model that no longer works.** The dashboard makes the mismatch
visible instead of silent.

The alternative backend is the inlier-count threshold, which cannot produce
metres at all — it outputs a bit. Both are selectable so the difference is
something you can see rather than something you have to read about.

---

## Troubleshooting

| symptom | cause |
|---|---|
| `Waiting for Gazebo world...` forever | ROS 2's `gz` shim is first on `PATH`. The runner fixes this; if you start PX4 by hand, you must too. |
| `Preflight Fail: Found 0 compass` | the world declares a plugin list without `gz-sim-magnetometer-system`. |
| `Preflight Fail: No connection to the ground control station` | expected until step 6 connects. `sim_fly.py` runs a heartbeat thread; pymavlink does not do this on its own. |
| no image topic | `gz topic -l | grep /image$`. If `camera_info` exists but `image` does not, Gazebo is still loading the texture. |
| centimetre errors | the ground and the reference have become the same file. The runner refuses this, but only if both are passed through it. |
| few inliers, `prior=cold`, `scale N is more than 0.35 from 1.0` | the aircraft is not over the ground the tile search is looking at. Check `vehicle_global_position` against the world origin before suspecting the matcher. |
| the aircraft drifts off the map centre | a landing item has to sit `alt/tan(FW_LND_ANG)` ≈ 740 m away, and PX4 flies to it — through an unlimited loiter with `autocontinue: 0` and through a commanded `AUTO.LOITER`. The rig sets `MIS_TKO_LAND_REQ 0` so the mission needs no landing item at all. |
| `vehicle_status.nav_state` is 5, not 4 | that is `AUTO_RTL`, a datalink-loss failsafe firing 10 s after `sim_fly.py` exits and stops heartbeating. Over this world home *is* the map centre, so it looks correct. `NAV_DLL_ACT 0` in the sim params turns it off. |
| `no heartbeat` from `px4_set_params.py` | a second invocation against a port PX4 has already locked to the first. Pass every file to one invocation: `--file a.params b.params`. |
| a previous run seems to still be feeding the flight controller | orphaned `agp_bridge.py` processes. They run under ROS via `bash -c ... exec python3`, answer to no obvious name, and seven accumulated before `--kill` learned to match them — each still subscribed and still willing to publish to `/fmu/in/aux_global_position`. `ps -eo args \| grep [a]gp_bridge` to check. |
| `cs_aux_gpos` stays False with everything else healthy | the AGP bridge is not running. It needs **both** ROS (`rclpy`, `px4_msgs`) and the venv (`zmq`, `geoanchor`); `.venv/bin/python` fails on `rclpy` and bare `python3` on `zmq`. Read `/tmp/agp_bridge.log`. |
| `ModuleNotFoundError: gz` | `sudo apt install python3-gz-transport13 python3-gz-msgs10`. Do not set `PYTHONPATH` — see above. |
| `cannot import name 'Sentinel' from 'typing_extensions'` | something put the system dist-packages on `PYTHONPATH`, ahead of the venv. |
| `end of feed` after one published frame | a live feed returned `None` for "nothing new yet", which a finite feed uses for end-of-file. `GzFeed.read()` blocks instead. |
| ten identical `arm refused (result 1)` | the EKF had not converged. `sim_fly.py` waits for `MAV_STATE_STANDBY` first; a fixed wing in SITL needs 20-60 s. |
| `No valid data from Accel 0` / `angular velocity no longer valid` | something is starving lockstep. `gz topic -e` on the 1280x960 30 Hz image topic writes ~450 MB/s and will do it. |
| publisher advertised on `172.17.0.1` | that is the docker0 bridge, and it is harmless — gz-transport advertises on every interface and subscribers still connect. |
| PX4 will not restart | a stale lock. `rm -f /tmp/px4-sock-0 /tmp/px4_lock-0`. |

**Do not use `pkill -f px4`** to clean up. It matches its own shell and kills
the script that ran it, which surfaces as a bare exit code 144. Use `pkill -x
px4`, or an explicit PID loop.

---

## What this rig does not do

- **No wind, no turbulence, no rolling shutter, no motion blur.** The camera is
  a pinhole on a rigid mount. Real imagery at 5 m/s has all four.
- **The ground is flat.** `make_gz_world.py` builds a plane, not a DEM, so
  there is no terrain parallax and no relief displacement — one of the two
  effects that makes cities hard in the cross-date study.
- **GNSS stays on.** The fix goes in as an independent absolute source
  alongside it. Denying GNSS is a separate experiment, `px4_sitl.sh`.
- **The latency is not the flight latency.** The pipeline runs on a laptop.
  The 250 ms budget that matters is the one measured on the target board.
