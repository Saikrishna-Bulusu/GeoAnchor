# Keeping every device in sync

Written 10 Sept 2026. Every board pushes its session transcripts to a shared
repo; every board pulls everyone else's back; the dashboard on the laptop shows
all of them in one list.

---

## Two repos, on purpose

| repo | holds | who writes |
|---|---|---|
| `geoanchor-rt` | the code, configs, docs | whoever is developing |
| `GeoAnchor-logs` | session transcripts, one directory per device | every device, automatically |

**Why not one repo.** A session is ~100 KB and every board writes them
continuously. In the code repo they would bloat history permanently, and every
device would be committing to the same branch on every run — a merge conflict
per sync. In the logs repo each device owns `<device>/`, so two devices can
never touch the same file and the merge is always trivial.

The parent research repo (`~/GeoAnchor`) is **not** synced. It is 27 GB —
17 GB of datasets, 5.3 GB of venv, 2.2 GB of cloned third-party code. Those
are rebuildable or downloadable and do not belong in git.

---

## One-time setup

Create the logs repo once, from any machine:

```bash
gh repo create Saikrishna-Bulusu/GeoAnchor-logs --private
```

Then on **each** device:

```bash
cd ~/geoanchor-rt
bash scripts/sync_logs.sh
```

The first run clones, the rest is automatic. The device names itself from
`hostname -s`; override with `--device <name>` or `GEOANCHOR_DEVICE`.

If the remote is empty the script says so and pushes anyway — bootstrapping the
first device is the one case where `git pull` fails rather than no-opping, and
it is handled.

---

## Running it

```bash
bash scripts/sync_logs.sh              # push mine, pull theirs
bash scripts/sync_logs.sh --pull-only  # just refresh the fleet view
bash scripts/sync_logs.sh --dry-run    # show what would be copied
```

It is **idempotent**: a second run with nothing new prints
`nothing changed -- already in sync` and exits without a commit.

### Automatically, after every run

```bash
crontab -e
```

```
*/30 * * * * cd $HOME/geoanchor-rt && bash scripts/sync_logs.sh >> /tmp/geoanchor-sync.log 2>&1
```

Half-hourly is deliberate. Syncing on every run start would push half-written
sessions; the script already skips a run directory with no `session.json`, but
a schedule that is slower than a flight is simpler to reason about.

### What gets pushed

    <device>/board.json           model, cores, RAM, max clock
    <device>/<run>/session.json   the full export, including the embedded basemap
    <device>/<run>/*.jsonl        per-layer transcripts

Not pushed: `stores/` (16–41 MB each and rebuildable from the source raster),
`.venv`, `data/`, and the raw `*.out` console logs.

**`board.json` is the reason a number can be read months later.** A session's
timings mean nothing without knowing which board produced them, and the
hostname is not enough — it does not say how many cores were online, which is
exactly the trap that has cost this project two runs on Jetson hardware.

---

## Seeing it on the dashboard

The API exposes the clone read-only:

    GET /api/fleet                      devices + every session, newest first
    GET /api/fleet/<device>/<run>       one session.json

The dashboard's **Replay** panel lists them under a "Fleet" heading with a
device filter. Click one and it loads exactly as a dropped file would.

The API never runs git and never writes into the clone. Syncing is the cron
job's business, so a board that is off, or a laptop with no network, degrades
to "its runs are not listed yet" rather than to a failed request.

Point the API at the clone with `session.fleet_dir` in `configs/system.yaml`
(default `fleet/`, which is gitignored).

### A replayed session carries its own map

`session.json` embeds the map packet **and** the basemap as a ~570 KB data URI,
so a session from a Pi renders its track on the Pi's reference tile while
displayed on a laptop that has never built that store.

**Sessions written before this feature do not have it**, and they draw their
tracks on a blank ground. That is the export's age, not a sync failure — check
for a `map` key before concluding anything is broken:

```bash
python -c "import json,sys; d=json.load(open(sys.argv[1])); print('map:', bool(d.get('map')), '| basemap:', bool(d.get('basemap')))" fleet/<device>/<run>/session.json
```

Turn the embed off with `output_layer.export.embed_basemap: false` if the
transcripts ever get too big; the track then needs a board that has the store.

---

## When it goes wrong

**Push rejected / behind.** The script always pulls with `--rebase --autostash`
before pushing, so this means two devices synced at the same instant. Re-run
it; the second attempt rebases cleanly because they wrote different
directories.

**A push fails with no network.** The commit is local in `fleet/` and nothing
is lost. `runs/` is the source of truth and `fleet/` is a mirror. Re-run when
the network is back.

**A device's runs are missing.** Check it has actually synced —
`fleet/<device>/board.json` should exist. If a run has no `session.json` it was
never flushed, which means the output layer did not close cleanly.
