/* SPDX-License-Identifier: GPL-3.0-or-later
 * Web EFB transport (docs/EFB_DEVELOPMENT.md): the native EFB (a380x_efb.c, other translation unit)
 * builds one JSON frame per cockpit tablet; this file publishes it to the aircraft-local WebKit
 * helper (xp-kit views 7/8), drains the page -> plugin action rings into the EFB's validator and
 * lets the EFB cockpit devices draw the captured page texture. No network, no server.
 *
 * Unity-build include of a380x_xpkit_adapter.inc (needs xpkit_shared, xpkit_health, xpkit_now,
 * xpkit_draw_view, xpkit_uploaded).
 */
#define XPKIT_EFB_HEALTH 22   /* xpkit_health[22..23] active, [24..25] presented frames */
static uint64_t xpkit_efb_hash[2];
static double xpkit_efb_time[2];
static unsigned xpkit_efb_serial[2];
static char xpkit_efb_json[XPKIT_EFB_CAP];

static int xpkit_efb_live(int tablet)
{
    return tablet >= 0 && tablet < 2 && xpkit_shared && xpkit_pid && xpkit_health[0] &&
           xpkit_health[XPKIT_EFB_HEALTH + tablet];
}

static int xpkit_efb_draw(int tablet, float width, float height)
{
    return xpkit_efb_live(tablet) && xpkit_draw_view(XPKIT_EFB_VIEW0 + tablet, 0, 0, width, height);
}

static const A380XEfbWebHost xpkit_efb_host = {xpkit_efb_live, xpkit_efb_draw};

static void xpkit_efb_init(void) { a380x_efb_set_web_host(&xpkit_efb_host); }
static void xpkit_efb_shutdown(void) { a380x_efb_set_web_host(NULL); }

static void xpkit_efb_reset(void)
{
    memset(xpkit_efb_hash, 0, sizeof(xpkit_efb_hash));
    memset(xpkit_efb_time, 0, sizeof(xpkit_efb_time));
    memset(xpkit_efb_serial, 0, sizeof(xpkit_efb_serial));
    for (int i = 0; i < 4; ++i) xpkit_health[XPKIT_EFB_HEALTH + i] = 0;
}

/* Page -> plugin: every complete slot written by the helper, in order, then released. */
static void xpkit_efb_drain(void)
{
    for (int t = 0; t < 2; ++t) {
        A380XXpKitEfb *efb = &xpkit_shared->efb[t];
        uint32_t head = atomic_load_explicit(&efb->action_head, memory_order_acquire);
        uint32_t tail = atomic_load_explicit(&efb->action_tail, memory_order_relaxed);
        if (head - tail > XPKIT_EFB_ACTIONS) tail = head - XPKIT_EFB_ACTIONS;   /* corrupt ring: resync */
        for (int n = 0; tail != head && n < XPKIT_EFB_ACTIONS; ++n, ++tail) {
            char text[XPKIT_EFB_ACTION_LEN];
            memcpy(text, efb->action[tail % XPKIT_EFB_ACTIONS], sizeof(text));
            text[sizeof(text) - 1] = 0;
            a380x_efb_web_action(t, text);
        }
        atomic_store_explicit(&efb->action_tail, tail, memory_order_release);
    }
}

/* Plugin -> page: rebuilt on any EFB change (touch, action, state) and at least every 250 ms for the
 * live aircraft values; published only when the bytes differ (no WebKit capture for an idle page). */
static void xpkit_efb_publish(void)
{
    double now = xpkit_now();
    unsigned serial = a380x_efb_change_serial();
    for (int t = 0; t < 2; ++t) {
        if (serial == xpkit_efb_serial[t] && now - xpkit_efb_time[t] < .25) continue;
        long size = a380x_efb_frame_json(t, xpkit_efb_json, sizeof(xpkit_efb_json));
        xpkit_efb_serial[t] = serial;
        xpkit_efb_time[t] = now;
        if (size <= 0 || size >= XPKIT_EFB_CAP) { ++xpkit_health[3]; continue; }
        uint64_t hash = 1469598103934665603ULL;
        for (long i = 0; i < size; ++i) hash = (hash ^ (unsigned char)xpkit_efb_json[i]) * 1099511628211ULL;
        A380XXpKitEfb *out = &xpkit_shared->efb[t];
        if (hash == xpkit_efb_hash[t] && atomic_load(&out->sequence)) continue;
        uint32_t seq = atomic_load_explicit(&out->sequence, memory_order_relaxed);
        atomic_store_explicit(&out->sequence, seq + 1, memory_order_release);
        memcpy(out->json, xpkit_efb_json, (size_t)size + 1);
        out->size = (uint32_t)size;
        atomic_store_explicit(&out->sequence, seq + 2, memory_order_release);
        xpkit_efb_hash[t] = hash;
    }
}

/* Every frame (xpkit_present_poll), also without a helper. */
static int xpkit_frame_unclaimed(int view, double now);
static void xpkit_efb_poll(double now)
{
    for (int t = 0; t < 2; ++t) {
        int view = XPKIT_EFB_VIEW0 + t;
        int live = xpkit_shared && xpkit_health[0] && atomic_load(&xpkit_shared->ready[view]) &&
                   atomic_load(&xpkit_shared->frame_seq[view]) > 0 && now - xpkit_shared->frame_time[view] < 1.0;
        xpkit_health[XPKIT_EFB_HEALTH + t] = live;
        /* A newly captured page frame: the on-demand device draws (and presents) it once. */
        if (live && xpkit_frame_unclaimed(view, now)) a380x_efb_needs_drawing(t);
    }
}
