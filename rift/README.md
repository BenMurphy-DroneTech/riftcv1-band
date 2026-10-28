# riftcv1-mocap band tools

This fork of Smol Slime is the band firmware for
[riftcv1-mocap](https://github.com/BenMurphy-DroneTech/riftcv1-mocap). Its
receiver is [riftcv1-receiver](https://github.com/BenMurphy-DroneTech/riftcv1-receiver),
which also sends the camera beacon (riftcv1-mocap's `firmware/rift_common`,
pulled in as a west project). The design lives in the Marker spec's Band
firmware tab (Claude Docs).

Both build on Pi 1 in `~/smol`, apart from riftcv1-mocap's own Zephyr:

- `setup.sh`: first run only. Zephyr SDK 0.16.9 and one nRF Connect SDK
  workspace per repo (`~/smol/band`, `~/smol/receiver`; the app is cloned as
  `app/`), as each repo's `west.yml` pins it (band v3.1, receiver
  v3.3.0-preview3). About 7 GB.
- `build.sh band|receiver [-Doptions]`: builds for `xiao_ble/nrf52840/sense`
  (band) or `xiao_ble/nrf52840` (receiver) and prints the .uf2 and its sha256.
  The receiver builds the riftcv1-mocap revision its `west.yml` pins: after
  moving the pin, `cd ~/smol/receiver && venv/bin/west update riftcv1-mocap`.
- `flash.sh band|receiver`: the `dfu` console command, then the last build
  copied to `/media/ben/XIAO-SENSE` (if `dfu` can't reach it, double-tap
  reset first).
- `con.py band|rx "cmd" ...`: sends console commands (`wait N` listens for N s).
- `statlog.py SECONDS [PERIOD]`: logs both ends' `stats` every PERIOD seconds.

The band's `pairs` command prints its timestamp pairs (its own time at each
packet's transmit ADDRESS, the receiver's beacon-clock arrival from the
acknowledgement) and their residuals from a fitted line; the receiver's
`stats` adds the beacon and acknowledgement-timestamp counts.

Pairing by console: on the receiver `add <band device address>`, which prints
`Pair the device with <code>`; on the band `set <code>`.
