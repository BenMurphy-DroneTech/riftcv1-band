#!/usr/bin/env bash
# Set up nRF Connect SDK workspaces for the band and receiver forks
# under ~/smol, with Zephyr SDK 0.16.9 (as Smol Slime's CI uses).
set -euo pipefail
TOP=~/smol
SDKV=0.16.9
SDK=$TOP/zephyr-sdk-$SDKV
mkdir -p $TOP
cd $TOP

if [ ! -x "$SDK/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc" ]; then
	url=https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v$SDKV
	curl -fsSL -o sdk.tar.xz $url/zephyr-sdk-${SDKV}_linux-aarch64_minimal.tar.xz
	curl -fsSL -o arm.tar.xz $url/toolchain_linux-aarch64_arm-zephyr-eabi.tar.xz
	tar -xf sdk.tar.xz
	tar -xf arm.tar.xz -C "$SDK"
	rm sdk.tar.xz arm.tar.xz
fi
"$SDK/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc" --version | head -1

ws() { # name repo
	local name=$1 repo=$2
	local W=$TOP/$name
	mkdir -p $W
	[ -x $W/venv/bin/west ] || { python3 -m venv $W/venv; $W/venv/bin/pip install -q --upgrade pip; $W/venv/bin/pip install -q west cmake ninja; }
	[ -d $W/app/.git ] || git clone -q $repo $W/app
	cd $W
	[ -d .west ] || $W/venv/bin/west init -l app
	$W/venv/bin/west update --narrow -o=--depth=1 >/dev/null
	$W/venv/bin/pip install -q -r zephyr/scripts/requirements-base.txt
	[ -f nrf/scripts/requirements-build.txt ] && $W/venv/bin/pip install -q -r nrf/scripts/requirements-build.txt
	echo "$name ready: $(git -C nrf describe --tags 2>/dev/null || git -C nrf rev-parse --short HEAD)"
}
ws band https://github.com/BenMurphy-DroneTech/riftcv1-band.git
ws receiver https://github.com/BenMurphy-DroneTech/riftcv1-receiver.git
du -sh $TOP/*
echo SETUP DONE
