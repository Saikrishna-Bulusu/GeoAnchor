# Xavier AGX: from JetPack 5.0.1 DP to a working GeoAnchor board

Verified against NVIDIA documentation on 3 Sept 2026. Every version number and
button sequence below was checked rather than recalled.

## Where you are and where this ends

| | now | after |
|---|---|---|
| JetPack | 5.0.1 **Developer Preview** (L4T 34.1.1) | 5.1.7 (L4T 35.6.5) |
| root filesystem | 32 GB eMMC, 3.3 GB free | 512 GB NVMe |
| Ubuntu / Python | 20.04 / 3.8 | 20.04 / 3.8 (unchanged, and that is fine) |

**JetPack 5.1.7 is the last release for AGX Xavier.** JetPack 6 is Orin-only.
You are not missing anything by staying on 5.

**One thing to understand before you start, because it changes what "boot from
the SSD" means.** The AGX Xavier keeps its boot firmware on the internal eMMC.
It physically cannot boot without a flashed eMMC. So the arrangement is:

    eMMC   bootloader, kernel, device tree, extlinux.conf     ~2 GB used
    NVMe   the entire root filesystem, everything you install  512 GB

That is the standard and supported arrangement, and it solves your space
problem completely: after step 5, `/` is the SSD and the eMMC never fills up
again. You cannot remove the eMMC from the picture, and you do not want to.

---

# Step 0 — Fit the SSD, and prove it works before you destroy anything

**Do this first.** Flashing before the drive is in means doing the work twice.

1. Shut down, unplug the power, and unplug everything else.
2. The M.2 Key M slot is under the side cover. One Phillips screw holds the
   card down at the far end. Seat the card at its angle, press flat, screw down.
3. **It must be PCIe NVMe, not M.2 SATA.** They look identical and the Xavier
   cannot boot from SATA. If yours has two notches in the connector edge it is
   probably SATA; NVMe (M key) has one.
4. Reassemble, power on, and boot the JetPack 5.0.1 that is already there.
5. Check the drive is seen:

```
lsblk -o NAME,SIZE,TYPE,TRAN,MODEL
```

You want a line like `nvme0n1  476.9G  disk  nvme  <model>`. If you see `sda`
with `TRAN=sata`, it is a SATA card and you need a different drive. **Sort this
out now** — every later step assumes `/dev/nvme0n1` exists.

Nothing else on this old install matters. Once `lsblk` is happy, you are done
with JetPack 5.0.1 forever.

---

# Step 1 — Turn the Legion into a flashing host

Your Legion runs **Ubuntu 24.04**, which NVIDIA does not support as a host for
JetPack 5. SDK Manager will refuse to install natively. The supported way round
this is NVIDIA's own SDK Manager Docker image, which carries a supported Ubuntu
inside it.

You need roughly **40 GB free** on the Legion for the downloaded images.

**1a. Docker, if you do not already have it:**

```
sudo apt update && sudo apt install -y docker.io
sudo usermod -aG docker $USER
getent group docker && sg docker -c 'docker run --rm hello-world'
```

`getent` should print `docker:x:127:sai`, confirming the group change landed.
`sg` runs the test with that group applied, which your current shell does not
yet have. **Log out and back in** before the next step so plain `docker` works
without the `sg` wrapper.

**1b. Get the SDK Manager container.** Download the **Docker image** variant
(not the .deb) from <https://developer.nvidia.com/sdk-manager>. It needs an
NVIDIA developer account, which is free.

Three Docker variants are offered — Ubuntu 24.04, 22.04 and 20.04 — and **only
20.04 works.** NVIDIA's host-OS compatibility matrix on that same page ticks
JetPack 5.x for Ubuntu 18.04 and 20.04 only; the 22.04 and 24.04 columns are
blank for that row. Take a 22.04 container and it installs, runs, detects the
board, and then simply does not offer JetPack 5 for the Xavier — which looks
like a recovery-mode fault and is not one. The file is
`sdkmanager-2.4.1.13536-ubuntu_20.04_docker.tar.gz` as of 3 Sep 2026.

Note the extension: NVIDIA ships this one as a plain **`.tar`**, not `.tar.gz`,
and the `Ubuntu` in the name is capitalised. A glob written for `.tar.gz` will
silently match nothing.

```
docker load -i ~/Downloads/sdkmanager-*_docker.tar && docker tag "$(docker image ls --format '{{.Repository}}:{{.Tag}}' | grep '^sdkmanager:' | head -1)" sdkmanager:latest && docker image ls | grep sdkmanager
```

That loads the image, reads back whatever version tag it carries, and re-tags it
`sdkmanager:latest`, so nothing downstream depends on the version number. For
2.4.1 the loaded tag is `sdkmanager:2.4.1.13536-Ubuntu_20.04`.

---

# Step 2 — Flash JetPack 5.1.7 to the eMMC

**2a. Put the Xavier into Force Recovery Mode.** This exact sequence, from the
Jetson Linux Developer Guide:

1. Make sure the developer kit is **powered off**.
2. **Press and hold** the Force Recovery button (the middle of the three).
3. **Press, then release** the Power button.
4. **Release** the Force Recovery button.

**2b. Connect it to the Legion** with a USB-C cable, using **the USB-C port next
to the power button**. There is more than one USB-C port and the other one will
not work for flashing.

**2c. Confirm it is actually in recovery mode** before going further:

```
lsusb | grep 0955
```

You must see `ID 0955:7019 NVIDIA Corp.` — that ID is the AGX Xavier in
recovery. If nothing appears, repeat 2a; the timing on the button press is
fussy and it is normal to need two or three attempts.

**2d. Flash.** Run the container with `--cli` and nothing else, and let its
wizard walk you through the choices. The non-interactive flag names (`--login-type`
vs `--logintype`, `--target-os` vs `--targetos`) and the target identifiers both
change between SDK Manager versions, so a fully-specified command line is the
most fragile way to do this, not the most reliable:

```
docker run -it --privileged --network host -v /dev/bus/usb:/dev/bus/usb -v /dev:/dev -v /media/$USER:/media/nvidia:slave --name jetpack_xavier sdkmanager --cli
```

It will ask for your NVIDIA developer login first. Type those yourself — they
are your credentials and nobody else should be handling them.

The container is deliberately **named and not `--rm`'d**, so the tens of
gigabytes it downloads survive a failure. If the flash dies partway, do not
re-run the `docker run` line — it will collide on the name and, worse, start the
download from zero. Resume the same container instead:

```
docker start -ai jetpack_xavier
```

When it asks:

- **Target hardware**: Jetson AGX Xavier (the devkit entry)
- **Storage device / OEM configuration**: **eMMC**, and **runtime** OEM config
- **Do NOT choose NVMe as the storage target.** NVIDIA's own documentation says
  the SDK Manager container "does not currently support flashing to external
  storages on all Jetson devices". We move the root filesystem across ourselves
  in step 5, which is the part of this job the container is bad at, and it is a
  twenty-minute job you can watch.
- **System configuration: Target Hardware only — untick Host Machine.** The host
  half installs x86 CUDA 11.4, cuDNN, TensorRT and Nsight onto the Legion. None
  of it is used by a CUDA-free pipeline, it is tens of gigabytes, JetPack 5 host
  components are not supported on Ubuntu 24.04 anyway, and it can collide with
  the CUDA and ROS toolchain already installed there.
- **Components: Jetson OS only — untick Jetson SDK Components.** This is a 32 GB
  eMMC. A fresh JetPack 5.1.7 rootfs uses roughly 16 GB of it; the SDK components
  want another 8-12 GB, which very nearly fills the card and then has to be
  rsync'd across in step 5. Flash the OS, move the rootfs to the NVMe, and only
  then install what you actually want with `sudo apt install nvidia-jetpack` on
  the roomy filesystem. It also cuts most of the download time out of the flash.
- **If it offers to pre-set a username and password for the new system, do it.**
  That pre-seeds oem-config, which otherwise demands an HDMI monitor and a USB
  keyboard on the Jetson before first boot will complete.
- **If no JetPack 5 release is listed at all**, that is SDK Manager 2.4.1 hiding
  older SDKs, not a hardware fault. Quit, and re-run the same line with
  `--cli --archived-versions` appended in place of `--cli`.

This takes 45–90 minutes, most of it download. If the flash itself fails partway,
put the board back into recovery mode and re-run — a half-flashed eMMC is not a
brick, it just needs doing again.

---

# Step 3 — First boot

Monitor on HDMI or DisplayPort, USB keyboard and mouse, then power on. You get
the Ubuntu first-boot wizard.

- **Username**: use `sai`, so paths match your Legion and every command below.
- **Hostname**: `xavier` — used in the SSH commands later.
- **Timezone**: Australia/Sydney.
- Accept the licence, skip anything optional.

Then confirm you got what you asked for:

```
cat /etc/nv_tegra_release          # want: R35 ... REVISION: 6.5
lsblk | grep nvme                  # the SSD is still there
free -h                            # 32 GB
nvpmodel -q                        # power mode
```

`R35 REVISION: 6.5` means JetPack 5.1.7. If it says R34, the flash used the
wrong version and you should redo step 2.

---

# Step 4 — Network: direct Ethernet, with internet

Here is the thing that will bite you if you skip it: **a bare Ethernet cable
between the two machines gives the Jetson no internet.** Steps 5 to 8 all need
it — apt, pip, git clone, npm, and Claude Code's login. So set the Legion up to
share its wifi over the cable.

Modern network ports auto-detect cable type, so an ordinary straight cable is
fine. No crossover cable needed.

**On the Legion**, with the cable plugged into both machines:

```
nmcli device status
```

That prints your wired device — something like `enp5s0`, or `enx...` if you are
using a USB-Ethernet adapter. This next line finds it itself and sets up the
share, so you do not have to substitute anything:

```
WIRED=$(nmcli -t -f DEVICE,TYPE device status | awk -F: '$2=="ethernet"{print $1; exit}') && echo "sharing over: $WIRED" && sudo nmcli connection add type ethernet ifname "$WIRED" con-name jetson-share ipv4.method shared && sudo nmcli connection up jetson-share && ip -4 addr show "$WIRED" | grep inet
```

The last line should show `10.42.0.1`. If `$WIRED` comes out empty, the cable is
not detected — reseat it at both ends and run it again.

The Legion becomes `10.42.0.1` and hands the Jetson an address by DHCP, with
NAT out through your wifi. Nothing on the Jetson needs configuring — its wired
connection defaults to DHCP.

**On the Jetson**, check you got an address and can reach the world:

```
ip -4 addr show | grep 10.42
ping -c3 8.8.8.8
ping -c3 github.com
```

If the ping to `8.8.8.8` works but `github.com` does not, it is DNS: add
`nameserver 8.8.8.8` via `nmcli connection modify <conn> ipv4.dns 8.8.8.8`.

**Then, from the Legion**, check SSH:

```
ssh sai@xavier.local && ssh-copy-id sai@xavier.local
```

If `xavier.local` does not resolve, use the address directly — find it on the
Jetson with `ip -4 addr show`, then `ssh sai@10.42.0.<n>`.

If SSH is refused outright the server is not installed. Run this on the Jetson,
then retry the line above:

```
sudo apt install -y openssh-server
```

---

# Step 5 — Move the root filesystem to the NVMe

Twenty minutes, entirely on the Jetson, and reversible by editing one line back.

**5a. Partition and format** (this erases the SSD, which is empty anyway):

```
sudo parted /dev/nvme0n1 mklabel gpt
sudo parted -a optimal /dev/nvme0n1 mkpart primary ext4 0% 100%
sudo mkfs.ext4 -L JETSONROOT /dev/nvme0n1p1
```

**5b. Copy the running root filesystem across:**

```
sudo mkdir -p /mnt/nvme && sudo mount /dev/nvme0n1p1 /mnt/nvme
sudo rsync -aAXv --exclude={"/dev/*","/proc/*","/sys/*","/tmp/*","/run/*","/mnt/*","/media/*","/lost+found"} / /mnt/nvme/
```

Ten minutes or so. The excludes matter: they are the pseudo-filesystems the
kernel provides at runtime, and copying them is at best pointless and at worst
a recursive loop through `/mnt`.

**5c. Point the bootloader at the SSD.** Back up first, then edit:

```
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.emmc-backup
sudo nano /boot/extlinux/extlinux.conf
```

Find the `APPEND` line under `LABEL primary`. It contains `root=/dev/mmcblk0p1`.
Change **only that** to `root=/dev/nvme0n1p1`. Leave everything else alone.

Then add a fallback block at the end of the file, so a failed boot is
recoverable from the serial console rather than a reflash:

```
LABEL emmc
      MENU LABEL fallback to eMMC
      LINUX /boot/Image
      INITRD /boot/initrd
      APPEND ${cbootargs} root=/dev/mmcblk0p1 rw rootwait rootfstype=ext4 console=ttyTCU0,115200n8 console=tty0 fbcon=map:0 net.ifnames=0
```

**5d. Reboot and confirm:**

```
sudo reboot
```

When it comes back up, check what `/` actually is:

```
df -h /
```

You want `/dev/nvme0n1p1` mounted on `/` with about 470 GB available. If you
still see `mmcblk0p1`, the APPEND line did not take — check you edited the block
that `DEFAULT` points to at the top of the file.

**One warning you need to carry forward.** The bootloader reads the kernel from
the **eMMC**, but apt now installs kernel updates onto the **NVMe**. A kernel
upgrade would leave the eMMC's kernel and the NVMe's `/lib/modules` out of step,
and the board would boot into a system with no working modules. For a
benchmarking machine you want a fixed kernel anyway, so pin it:

```
sudo apt-mark hold nvidia-l4t-kernel nvidia-l4t-kernel-dtbs nvidia-l4t-kernel-headers
```

---

# Step 6 — Claude Code on the Jetson

Ubuntu 20.04's apt carries a Node far too old for Claude Code, so use nvm:

```
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
exec $SHELL -l
nvm install --lts
node -v                       # want v20 or newer
npm install -g @anthropic-ai/claude-code
```

Then `claude` in any directory to log in the first time.

---

# Step 7 — Get the project across

The runtime is a git repo now, so **clone it rather than rsyncing** — that
carries the history, and you can push results back to the Legion afterwards.

**On the Jetson:**

```
git clone ssh://sai@10.42.0.1/home/sai/GeoAnchor/geoanchor-rt ~/geoanchor-rt
```

The data is deliberately not in the repo (the NSW tile and AnyVisLoc are large
and not ours to redistribute), so bring it separately.

**On the Legion:**

```
rsync -avz --progress ~/GeoAnchor/data/sydney/ sai@xavier.local:~/geoanchor-rt/data/sydney/
```

**Do not** add `--delete` to that command. On 2 Sept a `--delete` without the
right excludes wiped the Pi's git history and two run transcripts. There is no
reason to ever use it here.

---

# Step 8 — Bring the pipeline up

```
cd ~/geoanchor-rt && bash bootstrap.sh
```

`bootstrap.sh` already knows this board. It will detect the Xavier, tell you
JetPack 5.1.x is the ceiling, notice Python 3.8, and pin torch to 2.4.1 — the
newest release with a cp38 aarch64 wheel — while asserting the wheel is CPU-only.
Expect 15–30 minutes, most of it the torch download.

Then, **once per boot, before any timing measurement**:

```
sudo nvpmodel -m 0 && sudo jetson_clocks && sudo nvpmodel -q
```

Record what `nvpmodel -q` prints in your run notes. A timing number without a
power mode beside it is not comparable with anything.

Then:

```
python3 scripts/preflight.py                          # what this board can do
python3 -m geoanchor.data_layer --build-map           # once, ~1-2 min
bash verify.sh                                        # 46 checks, ~3 min
```

If `verify.sh` comes back 46/46 the board is ready, and `CLAUDE.md` in that
directory will bring Claude Code fully up to speed on the first prompt.

---

# What to measure first

From `docs/bringup.md`, and in this order:

1. **`python3 scripts/measure_overhead.py --device 0 --n 200`** — OVERHEAD_MS.
   Still a 40 ms guess, and the whole deployment question sits downstream of it.
2. **`bash sweep.sh`** — the 326-frame env80 table, with the Xavier's own
   timings and INA3221 power readings.

The Xavier is the bench upper bound, not a flight board. Its numbers set the
top of the compute curve; the Pi 5 numbers you already have set the bottom.

---

# If something goes wrong

| symptom | cause | fix |
|---|---|---|
| `lsusb` shows no `0955:7019` | recovery mode did not take | repeat the button sequence; check you used the USB-C port next to the power button |
| SDK Manager refuses to start | it detected Ubuntu 24.04 | you are running it natively — use the Docker image |
| flash fails partway | usually a USB or power hiccup | back into recovery, run it again; the eMMC is not bricked |
| boots to `mmcblk0p1` after step 5 | edited the wrong `LABEL` block | check which block `DEFAULT` names at the top of extlinux.conf |
| will not boot at all after step 5 | bad `root=` | serial console on the USB-C port at 115200 baud, pick the `emmc` fallback entry |
| `bootstrap.sh` fails on torch | wrong Python or arch | `python3 -V` should be 3.8.x and `uname -m` `aarch64` |
| no internet on the Jetson | sharing not set up | step 4; `nmcli connection show --active` on the Legion should list `jetson-share` |

## Rough timings

| step | time |
|---|---|
| 0 — fit and verify the SSD | 20 min |
| 1 — Docker and SDK Manager image | 30–45 min, mostly download |
| 2 — flash the eMMC | 45–90 min |
| 3 — first boot | 15 min |
| 4 — network | 15 min |
| 5 — rootfs to NVMe | 20 min |
| 6 — Claude Code | 10 min |
| 7 — project across | 10 min |
| 8 — bootstrap and verify | 30 min |

Half a day if it goes well. Step 2 is the one that eats time.
