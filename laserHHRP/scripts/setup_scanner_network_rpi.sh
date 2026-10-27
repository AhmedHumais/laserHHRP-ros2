#!/usr/bin/env bash
# Raspberry Pi 5 (Ubuntu 24.04): scanCONTROL on eth0 (netplan, or NetworkManager on desktop images).
# Run it over Wi-Fi or locally: reconfiguring eth0 drops an SSH session on eth0. Extra options pass through.
#   sudo scripts/setup_scanner_network_rpi.sh [-s SCANNER_IP] ...
exec "$(dirname "$0")/setup_scanner_network.sh" -i eth0 "$@"
