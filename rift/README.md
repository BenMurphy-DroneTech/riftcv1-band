# riftcv1-mocap band tools

This fork of Smol Slime is the band firmware for
[riftcv1-mocap](https://github.com/BenMurphy-DroneTech/riftcv1-mocap). Its
receiver is [riftcv1-receiver](https://github.com/BenMurphy-DroneTech/riftcv1-receiver).
The design lives in the Marker spec's Band firmware tab (Claude Docs).

Both build on Pi 1 in `~/smol`, apart from riftcv1-mocap's own Zephyr:

- `setup.sh`: first run only. Zephyr SDK 0.16.9 and one nRF Connect SDK
  workspace per repo (`~/smol/band`, `~/smol/receiver`; the app is cloned as
  `app/`), as each repo's `west.yml` pins it (band v3.1, receiver
  v3.3.0-preview3). About 7 GB.
- `build.sh band|receiver [-Doptions]`: builds for `xiao_ble/nrf52840/sense`
  (band) or `xiao_ble/nrf52840` (receiver) and prints the .uf2 and its sha256.
  Flash: the `dfu` console command (or a double-tap of reset), then copy the
  .uf2 to `/media/ben/XIAO-SENSE`.
- `con.py band|rx "cmd" ...`: sends console commands (`wait N` listens for N s).
- `statlog.py SECONDS [PERIOD]`: logs both ends' `stats` every PERIOD seconds.

Pairing by console: on the receiver `add <band device address>`, which prints
`Pair the device with <code>`; on the band `set <code>`.
