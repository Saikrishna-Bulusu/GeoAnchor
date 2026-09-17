#!/usr/bin/env bash
# Put a Pi 5 on a Pixhawk's telemetry port and fan the link out to two UDP ports.
#
#     bash scripts/pixhawk_link.sh                      # /dev/serial0 at 921600
#     bash scripts/pixhawk_link.sh /dev/ttyACM0 115200  # USB cable, bench only
#
# WHY A FAN-OUT AND NOT A DIRECT SERIAL ENDPOINT
# The data layer opens its own MAVLink connection to read GLOBAL_POSITION_INT
# and ATTITUDE, and the output layer opens a second one to write ODOMETRY and
# to drain HEARTBEAT. That separation is deliberate (gpsin.py header) and it is
# free over UDP, where two sockets see the same datagrams. Over a serial port it
# is not: two readers on one tty split the byte stream between them and both see
# corrupt MAVLink. One process owns the tty, everything else talks to it over
# UDP.
#
# 14560 is the data layer's read port, 14561 the output layer's write port.
# configs/flight.yaml already points at both. A third --out can feed QGC.
#
# ponytail: MAVProxy because it is a pip install. mavlink-routerd is lighter and
# starts faster, and is worth building if this ever becomes a boot service.
set -euo pipefail

PORT="${1:-/dev/serial0}"
BAUD="${2:-921600}"

if [ ! -e "$PORT" ]; then
    echo "no $PORT."
    case "$PORT" in
      /dev/serial0|/dev/ttyAMA*)
        echo "The Pi's UART is off or the console owns it. Enable it, then reboot:"
        echo "    sudo raspi-config nonint do_serial_hw 0"
        echo "    sudo raspi-config nonint do_serial_cons 1"
        ;;
      *) echo "Check the cable and 'ls /dev/tty*' with it unplugged and plugged." ;;
    esac
    exit 1
fi

if ! command -v mavproxy.py >/dev/null 2>&1; then
    echo "installing MAVProxy"
    python3 -m pip install --user MAVProxy
fi

echo "$PORT @ $BAUD  ->  udp 14560 (data layer)  udp 14561 (output layer)"
exec mavproxy.py --master="$PORT,$BAUD" \
     --out=udp:127.0.0.1:14560 \
     --out=udp:127.0.0.1:14561 \
     --streamrate=10 --daemon
