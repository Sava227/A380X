#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
extern float m_door[20]; extern int m_armed[16]; extern double m_t; extern XPLMFlightLoop_f loop_cb; extern XPLMCommandCallback_f m_stow;
typedef struct { const char *name; XPLMGetDatavf_f gvf; XPLMGetDataf_f gf; void *ref; } Acc; extern Acc acc[16]; extern int nacc;
PLUGIN_API int XPluginStart(char *, char *, char *);
static float slide(int k) { int i; float v = -1; for (i = 0; i < nacc; ++i) if (!strcmp(acc[i].name, "a380x/xp12/doors/slide_ratio")) acc[i].gvf(acc[i].ref, &v, k, 1); return v; }
static void run(double secs, int door, float door_rate) {
    int i, n = (int)(secs / 0.02);
    for (i = 0; i < n; ++i) {
        m_t += 0.02;
        if (door >= 0) { m_door[door] += door_rate * 0.02f; if (m_door[door] > 1) m_door[door] = 1; if (m_door[door] < 0) m_door[door] = 0; }
        for (int k = 0; k < 16; ++k) if (m_door[k] > 0.01f) m_armed[k] = 0;   /* the avionics disarms an opened door */
        loop_cb(0.02f, 0.02f, 0, NULL);
    }
}
int main(void) {
    char a[256], b[256], c[256];
    assert(XPluginStart(a, b, c));
    run(3, -1, 0);                                   /* finds the datarefs */
    m_armed[2] = 1; m_armed[12] = 1;                 /* M2L and U2L armed, M1L not */
    run(0.5, -1, 0);
    run(2, 2, 1.0f / 8);                              /* M2L opens: 0.25 of its travel after 2 s */
    printf("M2L after 2 s of door travel: door %.3f slide %.3f\n", m_door[2], slide(2));
    run(6.5, 2, 1.0f / 8);
    printf("M2L 6.5 s later: door %.3f slide %.3f\n", m_door[2], slide(2));
    assert(slide(2) > 0.99f);
    run(9, 0, 1.0f / 8);                              /* M1L (not armed) opens */
    printf("M1L opened disarmed: slide %.3f\n", slide(0)); assert(slide(0) == 0.0f);
    run(2.7, 12, 1.0f / 8);                           /* U2L: 0.34 of travel, below the upper threshold */
    printf("U2L at door %.3f: slide %.3f\n", m_door[12], slide(12)); assert(slide(12) == 0.0f);
    run(7, 12, 1.0f / 8);
    printf("U2L later: slide %.3f\n", slide(12)); assert(slide(12) > 0.5f);
    m_stow(NULL, xplm_CommandBegin, NULL); run(3.1, -1, 0);
    printf("after slides_stow_all: M2L %.3f U2L %.3f\n", slide(2), slide(12)); assert(slide(2) == 0.0f && slide(12) == 0.0f);
    m_armed[2] = 0; run(9, 2, -1.0f / 8);              /* close M2L: the slide stays stowed, nothing re-deploys */
    run(8, 2, 1.0f / 8); printf("M2L reopened after closing disarmed: slide %.3f\n", slide(2)); assert(slide(2) == 0.0f);
    puts("slides OK");
    return 0;
}
