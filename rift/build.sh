#!/usr/bin/env bash
# Build the band (Smol Slime tracker fork) or receiver fork on Pi 1.
#   ~/smol/build.sh band [-Doptions]      xiao_ble/nrf52840/sense
#   ~/smol/build.sh receiver [-Doptions]  xiao_ble/nrf52840
set -euo pipefail
NAME=${1:?band or receiver}; shift
case $NAME in
	band) BOARD=xiao_ble/nrf52840/sense ;;
	receiver) BOARD=xiao_ble/nrf52840 ;;
	*) echo "band or receiver" >&2; exit 1 ;;
esac
W=~/smol/$NAME
export ZEPHYR_SDK_INSTALL_DIR=~/smol/zephyr-sdk-0.16.9
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_BASE=$W/zephyr
export PATH=$W/venv/bin:$PATH
cd $W
west build --board $BOARD --pristine=always app --build-dir $W/build -- \
	-DNCS_TOOLCHAIN_VERSION=NONE -DBOARD_ROOT=$W/app "$@"
UF2=$W/build/app/zephyr/zephyr.uf2
echo "Built $UF2"
sha256sum $UF2
