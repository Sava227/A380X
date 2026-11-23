/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef A380X_XPKIT_SHARED_H
#define A380X_XPKIT_SHARED_H
#ifdef __cplusplus
#include <atomic>
#define XPKIT_ATOMIC(type) std::atomic<type>
#else
#include <stdatomic.h>
#define XPKIT_ATOMIC(type) _Atomic type
#endif
#include <stdint.h>
#include "a380x_mfd_types.h"
#include "a380x_du_scale.h"
#define XPKIT_MAGIC 0x58333830u
/* v7 (GitHub): every PFD/ND/MFD/SD view is captured at its kind's scale (a380x_du_scale.h).
 * v8: v7 plus views 7/8, the two cockpit EFB tablets (1440x1000 at scale 1, the aspect of the old
 * 720x500 atlas region), each with its own JSON frame and a page -> plugin action ring
 * (docs/EFB_DEVELOPMENT.md).
 * v9: v8 plus zero-copy frames (macOS), appended after the v8 fields so every v8 offset is kept.
 *     Per view XPKIT_SURFACES IOSurface IDs, created by the PLUGIN (global, 'BGRA', 4 bytes per
 *     element, view width x height) once per plugin enable and kept across helper restarts.
 *     surface_mode[view] = 1 (set by the plugin only after its GL capability probe passed): the helper
 *     writes each frame into back = (surface_front + 1) % XPKIT_SURFACES under IOSurfaceLock and
 *     publishes it with the frame seqlock (frame_seq odd -> surface_front = back -> frame_time ->
 *     frame_seq even). surface_mode 0: the helper writes the pixel arrays as in v8 and publishes
 *     surface_front = XPKIT_SURFACE_PIXELS. The Windows host never uses surfaces.
 * Both helpers and tools/xp_kit_transport.py must agree on this layout. */
#define XPKIT_VERSION 9u
#define XPKIT_VIEWS 9
#define XPKIT_SD_CAP 65536
/* v6: PFDs are captured at their full 768x1024 logical size (were 384x512).
 * v7: every view is captured at its kind's scale times its 768x1024 logical size (a380x_du_scale.h);
 * the pages keep their 768x1024 CSS layout. */
#define XPKIT_W A380X_PFD_PX(768)
#define XPKIT_H A380X_PFD_PX(1024)
#define XPKIT_BYTES (XPKIT_W * XPKIT_H * 4)
#define XPKIT_ND_W A380X_ND_PX(768)
#define XPKIT_ND_H A380X_ND_PX(1024)
#define XPKIT_ND_BYTES (XPKIT_ND_W * XPKIT_ND_H * 4)
#define XPKIT_JSON_CAP 262144
#define XPKIT_MFD_W A380X_MFD_PX(MFD_W)
#define XPKIT_MFD_H A380X_MFD_PX(MFD_H)
#define XPKIT_MFD_BYTES (XPKIT_MFD_W * XPKIT_MFD_H * 4)
#define XPKIT_EFB_VIEW0 7
#define XPKIT_EFB_W 1440
#define XPKIT_EFB_H 1000
#define XPKIT_EFB_BYTES (XPKIT_EFB_W * XPKIT_EFB_H * 4)
#define XPKIT_EFB_CAP 49152
#define XPKIT_EFB_ACTIONS 32
#define XPKIT_EFB_ACTION_LEN 128
#define XPKIT_SURFACES 3
#define XPKIT_SURFACE_PIXELS XPKIT_SURFACES   /* surface_front: this frame is in the v8 pixel arrays */
typedef struct {
    XPKIT_ATOMIC(uint32_t) sequence;
    uint32_t powered;
    char uri[64];
    MfdDrawList list;
} A380XXpKitMfd;
typedef struct { XPKIT_ATOMIC(uint32_t) sequence; uint32_t size; char json[XPKIT_SD_CAP]; } A380XXpKitSd;
/* One cockpit tablet. The plugin is the only writer of sequence/size/json and of action_tail; the
 * helper is the only writer of the action slots, action_head and actions_dropped (single producer,
 * single consumer; head - tail never exceeds XPKIT_EFB_ACTIONS). A slot is a NUL-terminated
 * "<id> <verb> [args]" string from the page; the native EFB parses and validates every one. */
typedef struct {
    XPKIT_ATOMIC(uint32_t) sequence;
    uint32_t size;
    char json[XPKIT_EFB_CAP];
    XPKIT_ATOMIC(uint32_t) action_head, action_tail, actions_dropped;
    char action[XPKIT_EFB_ACTIONS][XPKIT_EFB_ACTION_LEN];
} A380XXpKitEfb;
/* Read-only diagnostics: helper writes capture/apply fields; plugin writes upload/draw fields. */
typedef struct {
    XPKIT_ATOMIC(uint64_t) snapshot_count, snapshot_ns, apply_count, apply_ns, skipped_count;
    XPKIT_ATOMIC(uint64_t) upload_count, upload_bytes, draw_count;
} A380XXpKitNdProfile;
typedef struct {
    uint32_t magic, version;
    XPKIT_ATOMIC(uint32_t) state_seq;
    uint32_t state_size;
    double state_time;
    char state[XPKIT_JSON_CAP];
    XPKIT_ATOMIC(uint32_t) ready[XPKIT_VIEWS], frame_seq[XPKIT_VIEWS], errors;
    double frame_time[XPKIT_VIEWS];
    XPKIT_ATOMIC(int) zoom_delta[2];
    XPKIT_ATOMIC(uint32_t) nd_status[2][8];
    char error[512];
    unsigned char pixels[2][XPKIT_BYTES];
    unsigned char nd_pixels[2][XPKIT_ND_BYTES];
    unsigned char mfd_pixels[3][XPKIT_MFD_BYTES];
    A380XXpKitMfd mfd[2];
    A380XXpKitSd sd;
    A380XXpKitNdProfile nd_profile[2];
    unsigned char efb_pixels[2][XPKIT_EFB_BYTES];
    A380XXpKitEfb efb[2];
    /* v9: plugin writes surface_id and surface_mode; helper writes surface_front (see above). */
    XPKIT_ATOMIC(uint32_t) surface_id[XPKIT_VIEWS][XPKIT_SURFACES];
    XPKIT_ATOMIC(uint32_t) surface_mode[XPKIT_VIEWS];
    XPKIT_ATOMIC(uint32_t) surface_front[XPKIT_VIEWS];
} A380XXpKitShared;
/* The largest view sizes the plugin's texture staging buffer (an MFD/SD at its v7 scale, or an EFB). */
#define XPKIT_MAX_VIEW_BYTES (XPKIT_EFB_BYTES > XPKIT_MFD_BYTES ? XPKIT_EFB_BYTES : XPKIT_MFD_BYTES)
static inline int xpkit_view_width(int view) {
    return view < 2 ? XPKIT_W : view < 4 ? XPKIT_ND_W : view < XPKIT_EFB_VIEW0 ? XPKIT_MFD_W : XPKIT_EFB_W;
}
static inline int xpkit_view_height(int view) {
    return view < 2 ? XPKIT_H : view < 4 ? XPKIT_ND_H : view < XPKIT_EFB_VIEW0 ? XPKIT_MFD_H : XPKIT_EFB_H;
}
/* Capture scale of a view (pixels per CSS pixel): 768x1024 logical units for the display units; the
 * EFB page is laid out at its full 1440x1000. */
static inline float xpkit_view_scale(int view) {
    return view < 2 ? A380X_PFD_SCALE : view < 4 ? A380X_ND_SCALE : view < XPKIT_EFB_VIEW0 ? A380X_MFD_SCALE : 1.0f;
}
/* CSS (layout) size of a view: its capture pixels divided by its scale, i.e. 768x1024 for every display
 * unit and 1440x1000 for an EFB tablet. The helpers size their web views with these. */
static inline int xpkit_view_css_width(int view) { return (int)(xpkit_view_width(view) / xpkit_view_scale(view) + 0.5f); }
static inline int xpkit_view_css_height(int view) { return (int)(xpkit_view_height(view) / xpkit_view_scale(view) + 0.5f); }
static inline size_t xpkit_view_bytes(int view) {
    return view < 2 ? XPKIT_BYTES : view < 4 ? XPKIT_ND_BYTES : view < XPKIT_EFB_VIEW0 ? XPKIT_MFD_BYTES : XPKIT_EFB_BYTES;
}
static inline unsigned char *xpkit_view_pixels(A380XXpKitShared *s, int view) {
    return view < 2 ? s->pixels[view] : view < 4 ? s->nd_pixels[view - 2] :
           view < XPKIT_EFB_VIEW0 ? s->mfd_pixels[view - 4] : s->efb_pixels[view - XPKIT_EFB_VIEW0];
}
#endif
