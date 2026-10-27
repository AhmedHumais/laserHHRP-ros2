#!/usr/bin/env bash
# Finds a scanCONTROL laser on a wired interface and puts that interface on the scanner's subnet
# (persistent: NetworkManager if it manages the interface, otherwise netplan/systemd-networkd).
# Works on the development PC and on the Raspberry Pi 5 (Ubuntu 24.04). Needs root.
#   sudo scripts/setup_scanner_network.sh [-i IFACE] [-s SCANNER_IP] [-H HOST_IP] [-t SECONDS] [--force]
#     -i  wired interface (default: first wired interface that has a link: eno1, eth0, ...)
#     -s  scanner IP, if known: skips discovery
#     -H  host IP to use (default: .1 on the scanner's /24, .2 if the scanner is .1)
#     -t  how long to wait for the scanner to announce itself (default 90 s)
#     --force  configure even if this SSH session runs over that interface
# Discovery is passive: scanCONTROL scanners have a fixed IP and announce it (gratuitous ARP)
# when they boot, so power-cycle the scanner while the script waits. Micro-Epsilon MACs start 00:0c:12.
# At the end the scanner IP is written to config/hhrp.yaml (laser.interface).
set -euo pipefail

IFACE="" SCANNER_IP="" HOST_IP="" TIMEOUT=90 FORCE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    -i) IFACE="$2"; shift 2 ;;
    -s) SCANNER_IP="$2"; shift 2 ;;
    -H) HOST_IP="$2"; shift 2 ;;
    -t) TIMEOUT="$2"; shift 2 ;;
    --force) FORCE=1; shift ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

die() { echo "error: $*" >&2; exit 1; }
[[ $EUID -eq 0 ]] || die "run with sudo"

# --- interface -----------------------------------------------------------------------------------
if [[ -z "$IFACE" ]]; then
  for dev in /sys/class/net/*; do
    name=$(basename "$dev")
    [[ -d "$dev/device" && ! -d "$dev/wireless" ]] || continue   # physical, not Wi-Fi
    [[ "$(cat "$dev/carrier" 2>/dev/null)" == 1 ]] || continue    # has a link
    IFACE="$name"; break
  done
  [[ -n "$IFACE" ]] || die "no wired interface with a link found (cable plugged in?); use -i"
fi
[[ -e /sys/class/net/$IFACE ]] || die "interface $IFACE does not exist"
ip link set "$IFACE" up
echo "interface: $IFACE ($(cat /sys/class/net/"$IFACE"/speed 2>/dev/null || echo ?) Mb/s)"
speed=$(cat /sys/class/net/"$IFACE"/speed 2>/dev/null || echo 0)
[[ "$speed" -ge 1000 ]] || echo "warning: link is below 1000 Mb/s: check the cable (profiles may drop at high rates)"

# Don't cut off the session we're running in.
if [[ -n "${SSH_CONNECTION:-}" && $FORCE -eq 0 ]]; then
  local_ip=$(awk '{print $3}' <<<"$SSH_CONNECTION")
  if ip -4 -br addr show dev "$IFACE" | grep -qw "$local_ip"; then
    die "this SSH session runs over $IFACE; reconfiguring it would disconnect you (use Wi-Fi, or --force)"
  fi
fi

# --- discovery -----------------------------------------------------------------------------------
if [[ -z "$SCANNER_IP" ]]; then
  command -v tcpdump >/dev/null || { echo "installing tcpdump..."; apt-get install -y tcpdump >/dev/null; }
  echo "waiting up to ${TIMEOUT}s for the scanner: POWER-CYCLE THE SCANNER NOW"
  # ARP from a Micro-Epsilon MAC; the announcement is "who-has X tell X" (sender IP = scanner IP).
  line=$(timeout "$TIMEOUT" tcpdump -l -n -e -i "$IFACE" -c 1 \
           'arp and ether[6:4] & 0xffffff00 = 0x000c1200' 2>/dev/null || true)
  [[ -n "$line" ]] || die "no scanner seen on $IFACE (power on? cable? try: tcpdump -ni $IFACE -e arp)"
  SCANNER_MAC=$(awk '{print $2}' <<<"$line")
  SCANNER_IP=$(grep -oE 'tell [0-9.]+' <<<"$line" | awk '{print $2}')
  [[ -n "$SCANNER_IP" ]] || SCANNER_IP=$(grep -oE '[0-9]+(\.[0-9]+){3} is-at' <<<"$line" | awk '{print $1}')
  [[ -n "$SCANNER_IP" ]] || die "could not read the IP from: $line"
  echo "scanner: $SCANNER_IP (MAC $SCANNER_MAC)"
fi

# --- host address --------------------------------------------------------------------------------
prefix=${SCANNER_IP%.*}
if [[ -z "$HOST_IP" ]]; then
  [[ "${SCANNER_IP##*.}" == 1 ]] && HOST_IP="$prefix.2" || HOST_IP="$prefix.1"
fi
echo "host:    $HOST_IP/24 on $IFACE"

# --- persistent configuration --------------------------------------------------------------------
nm_managed=0
if command -v nmcli >/dev/null && systemctl is-active --quiet NetworkManager; then
  state=$(nmcli -g GENERAL.STATE device show "$IFACE" 2>/dev/null || true)
  [[ -n "$state" && "$state" != *unmanaged* ]] && nm_managed=1
fi

if [[ $nm_managed -eq 1 ]]; then
  con=$(nmcli -g GENERAL.CONNECTION device show "$IFACE" 2>/dev/null || true)
  if [[ -z "$con" ]]; then
    con="scancontrol-$IFACE"
    nmcli con add type ethernet ifname "$IFACE" con-name "$con" >/dev/null
  fi
  echo "NetworkManager connection: $con"
  nmcli con mod "$con" ipv4.method manual ipv4.addresses "$HOST_IP/24" ipv4.gateway "" \
        ipv4.never-default yes connection.autoconnect yes
  nmcli con up "$con" >/dev/null
else
  file=/etc/netplan/90-scancontrol-$IFACE.yaml
  echo "netplan: $file"
  cat >"$file" <<EOF
# Written by scripts/setup_scanner_network.sh: $IFACE on the scanCONTROL subnet.
network:
  version: 2
  ethernets:
    $IFACE:
      dhcp4: false
      dhcp6: false
      addresses: [$HOST_IP/24]
EOF
  chmod 600 "$file"
  netplan apply
fi

# Larger socket receive buffers, so GigE profile streams don't drop packets.
cat >/etc/sysctl.d/90-scancontrol.conf <<EOF
net.core.rmem_max = 33554432
net.core.rmem_default = 33554432
EOF
sysctl -q --system

# --- check ---------------------------------------------------------------------------------------
sleep 2
if ping -c 3 -W 1 -I "$IFACE" "$SCANNER_IP" >/dev/null; then
  echo "OK: scanner $SCANNER_IP reachable from $HOST_IP on $IFACE"
else
  die "configured $HOST_IP/24 but the scanner $SCANNER_IP does not answer ping"
fi

# --- ROS 2 parameter -----------------------------------------------------------------------
# laser.interface in config/hhrp.yaml: the line is replaced (also when commented out) or
# appended. Written in place (cat >) so the file keeps its owner although this runs as root.
params="$(cd "$(dirname "$0")/.." && pwd)/config/hhrp.yaml"
if [[ -f "$params" ]]; then
  line="    laser.interface: \"$SCANNER_IP\"  # scanner IP (set by scripts/setup_scanner_network.sh)"
  if grep -qE '^[#[:space:]]*laser\.interface:' "$params"; then
    updated=$(sed -E "s|^[#[:space:]]*laser\.interface:.*|$line|" "$params")
  else
    updated=$(cat "$params"; echo "$line")
  fi
  printf '%s\n' "$updated" >"$params"
  echo "params: laser.interface = $SCANNER_IP in $params"
else
  echo "note: $params not found: set laser.interface: $SCANNER_IP yourself"
fi
