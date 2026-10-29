# SkunkCrafts Updater development feed

This repository publishes an experimental A380X XP12 feed using the depot
format documented by SkunkCrafts Updater 4.2.7. Protocol level 1 is used for
compatibility with both 3.x and 4.x clients.

## Install and use

Install the aircraft in `X-Plane 12/Aircraft/A380X-XP12`. Close X-Plane,
start SkunkCrafts Updater, scan the simulator, and select **A380X XP12 Port
(Development)**. The module is:

```text
https://github.com/Sava227/A380X/raw/refs/heads/skunkcrafts/output/A380X-XP12/
```

Bundled livery repair is opt-in (`liveries|false`). Custom and otherwise
unlisted liveries are never deleted.

## Managed-file policy

`tools/build_skunkcrafts_depot.py` calculates unsigned decimal CRC32 and byte
sizes from hydrated payload bytes. It rejects LFS pointers, symlinks,
non-regular files, unsafe paths, and case/Unicode-normalization collisions.
The aircraft preference file is not published because X-Plane writes local
state into it. The authored X-Camera profile and FCU calibration use CRC `-1`:
install if missing, then never overwrite.

Backups, hidden metadata, build/runtime reports, source inventories, updater
state, and control files are excluded. The out-of-process macOS display helper
is also excluded until its parent plugin can restore execute permission lost
by SkunkCrafts downloads. Deleted or renamed managed files must be added to
`skunkcrafts_updater_blacklist.txt` for a release.

Generate and verify:

```sh
python3 tools/build_skunkcrafts_depot.py
python3 tools/build_skunkcrafts_depot.py --check
PYTHONPATH=tools python3 -m unittest tools/test_build_skunkcrafts_depot.py
```

The repository `VERSION`, package `VERSION`, and first changelog header must
match. The generator writes `skunkcrafts_updater.cfg`, whitelist, sizes list,
and blacklist atomically. It never places updater control files in the
whitelist.

## Publication safety

For a static host, publish `locked|true` first, then payload and lists, purge
metadata caches, and publish `locked|false` last. Verify remote GET bytes and
CRC rather than relying on HEAD requests. Disable image optimization,
transcoding, and minification.

This GitHub/LFS endpoint is a development-only fallback. Direct
`raw.githubusercontent.com` URLs return LFS pointer text for this repository;
the `github.com/.../raw/` endpoint redirects normal and LFS blobs correctly.
GitHub CDN caches are not atomically updated with a branch ref, and the
included LFS bandwidth permits only a few full repairs of this multi-gigabyte
plane. Move a public production feed to static object storage/CDN.

Official references:

- <https://github.com/lzamouth/sku_client/blob/master/docs/PUBLISHING.md>
- <https://1228098873.rsc.cdn77.org/updater/USER_GUIDE.html>
- <https://forums.x-plane.org/forums/topic/352063-20261004-skunkcrafts-updater-v4xy-is-available/>
