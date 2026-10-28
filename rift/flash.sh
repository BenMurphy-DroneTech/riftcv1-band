#!/usr/bin/env bash
# Flash the band or the receiver with its last build: the dfu console command
# starts the XIAO's UF2 bootloader, then the .uf2 is copied to its drive.
#   ~/smol/flash.sh band|receiver
set -euo pipefail
NAME=${1:?band or receiver}
case $NAME in
	band) CON=band ;;
	receiver) CON=rx ;;
	*) echo "band or receiver" >&2; exit 1 ;;
esac
UF2=~/smol/$NAME/build/app/zephyr/zephyr.uf2
DRIVE=/media/ben/XIAO-SENSE
python3 ~/smol/con.py $CON dfu >/dev/null 2>&1 || true
for i in $(seq 1 40); do
	[ -d $DRIVE ] && break
	sleep 0.5
done
[ -d $DRIVE ] || { echo "$DRIVE didn't appear: double-tap the XIAO's reset, then run this again" >&2; exit 1; }
cp $UF2 $DRIVE/
sync
echo "Flashed $UF2"
sha256sum $UF2
