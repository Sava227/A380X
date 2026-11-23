/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Pixel density of the 768x1024 display units, per kind.
 *
 * Every display keeps its 768x1024 logical layout; its cockpit device and, in xp-kit builds, the
 * WebKit/WebView2 capture are SCALE times larger, so a screen stays sharp when it covers more than
 * 768 pixels of the simulator window (about 1300 px on a full-screen 3024x1964 display).
 *
 * The helper captures one view at a time, so pixels cost frame rate everywhere. Measured first with
 * tools/probe_xp_kit_mfd_cache.py (both PFDs and NDs at one scale, Retina-scale painting and a CPU
 * resample): PFD 19 / 12 / 8 / 5 fps and ND capture 55 / 101 / 132 / 199 ms at 1, 1.25, 1.5 and 1.75.
 * Since the helper paints at the capture scale (no resample, vImage copy) and the plugin publishes the
 * PFD/ND state at up to 75 Hz, the real helper with moving PFD/ND content and 60 Hz state on an M1 Pro
 * (bench_displays.py, 35 s warm-up) captured PFD 60 / ND 52 fps with the PFD at 1, and PFD 45-52 /
 * ND 43 fps with both at 1.5 (ND 31 fps at 2). Hence the defaults: PFDs and NDs 3/2 (1152x1536), and
 * the MFDs / ECAM SD, captured only when their page changes, 7/4. Override with
 * -DA380X_<KIND>_SCALE_NUM=n -DA380X_<KIND>_SCALE_DEN=d.
 */
#ifndef A380X_DU_SCALE_H
#define A380X_DU_SCALE_H

#ifndef A380X_PFD_SCALE_NUM
#define A380X_PFD_SCALE_NUM 3
#define A380X_PFD_SCALE_DEN 2
#endif
#ifndef A380X_ND_SCALE_NUM
#define A380X_ND_SCALE_NUM 3
#define A380X_ND_SCALE_DEN 2
#endif
#ifndef A380X_MFD_SCALE_NUM      /* MFDs and the ECAM SD */
#define A380X_MFD_SCALE_NUM 7
#define A380X_MFD_SCALE_DEN 4
#endif

/* Logical display units -> device / capture pixels (exact for 768 and 1024 at these ratios). */
#define A380X_PFD_PX(v) ((v) * A380X_PFD_SCALE_NUM / A380X_PFD_SCALE_DEN)
#define A380X_ND_PX(v) ((v) * A380X_ND_SCALE_NUM / A380X_ND_SCALE_DEN)
#define A380X_MFD_PX(v) ((v) * A380X_MFD_SCALE_NUM / A380X_MFD_SCALE_DEN)
#define A380X_PFD_SCALE ((float)A380X_PFD_SCALE_NUM / (float)A380X_PFD_SCALE_DEN)
#define A380X_ND_SCALE ((float)A380X_ND_SCALE_NUM / (float)A380X_ND_SCALE_DEN)
#define A380X_MFD_SCALE ((float)A380X_MFD_SCALE_NUM / (float)A380X_MFD_SCALE_DEN)
/* Device texel (bottom-left origin) -> logical unit, at the texel centre. */
#define A380X_DU_LOGICAL(px, scale) (((float)(px) + 0.5f) / (scale))

#endif
