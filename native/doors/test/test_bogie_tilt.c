/* Offline test of A380XDoors' main gear bogie tilt (a380x/xp12/gear/bogie_tilt_deg) with a mock XPLM. */
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
extern float m_steer[10], m_tx[10], m_ty[10], m_tz[10], m_dep[10], m_defl[10], m_agl, m_theta, m_phi;
extern int m_gground[10]; extern double m_t; extern XPLMFlightLoop_f loop_cb;
typedef struct { const char *name; XPLMGetDatavf_f gvf; XPLMGetDataf_f gf; void *ref; } Acc; extern Acc acc[16]; extern int nacc;
PLUGIN_API int XPluginStart(char *, char *, char *);
static float get(const char *n, int k) { int i; float v = -999; for (i = 0; i < nacc; ++i) if (!strcmp(acc[i].name, n)) acc[i].gvf(acc[i].ref, &v, k, 1); return v; }
static float tilt(int k) { return get("a380x/xp12/gear/bogie_tilt_deg", k); }
static float hgt(int k) { return get("a380x/xp12/gear/height_m", k); }
static void run(double secs) { int i, n = (int)(secs / 0.02); for (i = 0; i < n; ++i) { m_t += 0.02; loop_cb(0.02f, 0.02f, 0, NULL); } }
static void ground(int on) { int k; for (k = 0; k < 5; ++k) m_gground[k] = on; }
int main(void) {
    char a[256], b[256], c[256]; int k;
    assert(XPluginStart(a, b, c));
    /* gear wheel points relative to the CG (airplane coords): wings +-6 m, body +-2.6 m, 5 m below; the wheel point
     * is reported 0.7 m above the true contact (as if it were the axle): the ground calibration must absorb it */
    float xs[5] = {0, -6, 6, -2.6f, 2.6f}, zs[5] = {-30, 1, 1, 6, 6};
    for (k = 0; k < 5; ++k) { m_tx[k] = xs[k]; m_ty[k] = -4.3f; m_tz[k] = zs[k]; m_dep[k] = 1; m_defl[k] = 0.1f; }
    m_agl = 5.0f - 0.7f - 0.1f + 0.0f;   /* contact point 0.1 m into the ground (strut deflection 0.1) */
    ground(1); run(3);
    printf("taxi: tilt wing %.2f body %.2f height %.3f\n", tilt(1), tilt(3), hgt(3));
    assert(fabsf(tilt(1)) < 0.05f && fabsf(tilt(3)) < 0.05f && tilt(0) == 0.0f);
    m_theta = 9.0f; run(0.5);                           /* rotation, mains still on the ground */
    printf("rotation 9 deg on the ground: tilt body %.2f wing %.2f\n", tilt(3), tilt(1)); assert(fabsf(tilt(3) + 9.0f) < 0.05f);
    ground(0); for (k = 0; k < 5; ++k) m_defl[k] = 0;
    run(0.5); printf("0.5 s after lift-off (height %.2f): body %.2f\n", hgt(3), tilt(3));
    assert(tilt(3) <= -9.0f + 0.01f && tilt(3) >= -10.0f - 0.01f);
    m_agl += 20; run(4); printf("climbing: body %.2f wing %.2f (front wheels down)\n", tilt(3), tilt(1));
    assert(fabsf(tilt(3) + 10) < 0.01f && fabsf(tilt(1) + 10) < 0.01f);
    m_theta = 0.0f; run(2); printf("level flight: body %.2f\n", tilt(3)); assert(fabsf(tilt(3) + 10) < 0.01f);
    m_dep[3] = 0.9f; run(0.1); printf("retracting, deploy 0.9: body %.2f\n", tilt(3)); assert(fabsf(tilt(3) + 10.0f / 3) < 0.05f);
    m_dep[3] = 0.5f; run(0.1); assert(tilt(3) == 0.0f);
    m_dep[3] = 1.0f; m_theta = 4.0f;                   /* approach, 4 deg nose up */
    /* descend to each body gear height (the mock's rotation about the CG left a calibration offset, so steer by
     * the computed height); check both axles of the body and wing gears */
    float hs[] = {3.0f, 0.5f, 0.2f, 0.1f, 0.05f, 0.02f, 0.0f};
    int n = (int)(sizeof(hs) / sizeof(hs[0])), j, gk[2] = {3, 1};
    float arm[2] = {1.66f, 0.867f};
    for (k = 0; k < n; ++k) {
        run(0.02); m_agl += hs[k] - hgt(3); run(0.3);
        for (j = 0; j < 2; ++j) {
            float h = hgt(gk[j]), g = (tilt(gk[j]) + 4.0f) * 0.0174533f;   /* bogie angle to the ground, + = rear down */
            float front = h + arm[j] * sinf(g), rear = h - arm[j] * sinf(g);
            printf("approach: %s height %.3f -> tilt %6.2f, front axle %.3f, rear axle %.3f above the runway\n",
                   j ? "wing" : "body", h, tilt(gk[j]), front, rear);
            assert(front > -0.005f && rear > -0.005f);
            if (h > 0.01f) assert(front < rear);                      /* front wheels lowest: they touch first */
        }
        if (hs[k] >= 3.0f) assert(fabsf(tilt(3) + 10) < 0.01f);
        if (hs[k] == 0.05f) assert(tilt(3) > -6.0f && tilt(3) < -5.0f);   /* front axle limit: -asin(.05/1.66)-4 */
    }
    ground(1); m_defl[3] = 0.05f; run(0.1); printf("touchdown: body %.2f\n", tilt(3)); assert(fabsf(tilt(3) + 4.0f) < 0.05f);
    m_theta = 0.0f; run(0.5); printf("nose wheel down: body %.2f\n", tilt(3)); assert(fabsf(tilt(3)) < 0.05f);
    ground(0); m_defl[3] = 0; m_agl += 20; run(0.5);              /* go-around: free again, trimmer rate */
    printf("go-around 0.5 s: body %.2f\n", tilt(3)); assert(fabsf(tilt(3) + 4.0f) < 0.1f);
    run(1.0); printf("go-around 1.5 s: body %.2f\n", tilt(3)); assert(fabsf(tilt(3) + 10) < 0.01f);
    puts("bogie tilt OK");
    return 0;
}
