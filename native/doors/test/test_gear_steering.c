/* Offline test of A380XDoors' gear steering visuals (a380x/xp12/gear/steer_vis_deg) with a mock XPLM. */
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
extern float m_steer[10]; extern int m_gground[10]; extern double m_t; extern XPLMFlightLoop_f loop_cb;
typedef struct { const char *name; XPLMGetDatavf_f gvf; XPLMGetDataf_f gf; void *ref; } Acc; extern Acc acc[16]; extern int nacc;
PLUGIN_API int XPluginStart(char *, char *, char *);
static float vis(int k) { int i; float v = -999; for (i = 0; i < nacc; ++i) if (!strcmp(acc[i].name, "a380x/xp12/gear/steer_vis_deg")) acc[i].gvf(acc[i].ref, &v, k, 1); return v; }
static void run(double secs) { int i, n = (int)(secs / 0.02); for (i = 0; i < n; ++i) { m_t += 0.02; loop_cb(0.02f, 0.02f, 0, NULL); } }
int main(void) {
    char a[256], b[256], c[256]; int k;
    assert(XPluginStart(a, b, c));
    for (k = 0; k < 5; ++k) m_gground[k] = 1;
    run(3);                                                        /* finds the datarefs, gates open */
    m_steer[0] = 45.0f; m_steer[3] = -8.0f; m_steer[4] = -8.0f; m_steer[1] = 3.0f;
    run(0.1);
    printf("on ground: nose %.2f body L %.2f R %.2f wing %.2f\n", vis(0), vis(3), vis(4), vis(1));
    assert(fabsf(vis(0) - 45.0f) < 1e-3f && fabsf(vis(3) + 8.0f) < 1e-3f && fabsf(vis(1) - 3.0f) < 1e-3f);
    m_steer[0] = 200.0f; run(0.1); printf("clamped: %.1f\n", vis(0)); assert(vis(0) == 90.0f);
    m_steer[0] = 30.0f; m_gground[0] = 0;                          /* nose lifts off (rotation), still steering in X-Plane */
    run(0.5); printf("0.5 s after nose liftoff: %.2f\n", vis(0)); assert(vis(0) > 13.0f && vis(0) < 17.0f);
    run(0.6); printf("1.1 s after liftoff: %.2f\n", vis(0)); assert(vis(0) == 0.0f);
    m_steer[0] = NAN; m_gground[0] = 1; run(2); printf("NaN input: %.2f\n", vis(0)); assert(vis(0) == 0.0f);
    puts("gear steering OK");
    return 0;
}
