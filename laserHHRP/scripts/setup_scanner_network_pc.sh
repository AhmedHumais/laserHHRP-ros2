#!/usr/bin/env bash
# Development PC: scanCONTROL on the wired port eno1 (NetworkManager). Extra options pass through.
#   sudo scripts/setup_scanner_network_pc.sh [-s SCANNER_IP] ...
exec "$(dirname "$0")/setup_scanner_network.sh" -i eno1 "$@"
