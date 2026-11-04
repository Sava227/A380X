/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Producer of the aircraft-local engine indications the native ECAM SD ENGINE page reads
 * (a380x_sd_adapter.inc "optional aircraft-local producers"):
 *   a380x/xp12/engine/n3_percent[4]           FBW N3 = X-Plane's HP core N2
 *   a380x/xp12/engine/oil_quantity_quarts[4]  FBW oil quantity (a380x_engine_indications.h)
 * Read-only float arrays computed on read, so they are never stale. Vibration and nacelle
 * temperature have no X-Plane source and stay unpublished (the page shows XX for them).
 */
#include "a380x_engine_indications.h"

static XPLMDataRef engi_out[2], engi_n2, engi_oil_ratio, engi_thrust;

static int engi_read(float *values, int offset, int max, int which)
{
    float n2[8] = {0}, ratio[8] = {0}, thrust[8] = {0};
    int i, n;
    if (!values) return 4;
    if (offset < 0 || offset >= 4) return 0;
    n = 4 - offset < max ? 4 - offset : max;
    if (!engi_n2) engi_n2 = XPLMFindDataRef("sim/cockpit2/engine/indicators/N2_percent");
    if (!engi_oil_ratio) engi_oil_ratio = XPLMFindDataRef("sim/cockpit2/engine/indicators/oil_quantity_ratio");
    if (!engi_thrust) engi_thrust = XPLMFindDataRef("sim/flightmodel/engine/POINT_thrust");
    if (which == 0 && (!engi_n2 || XPLMGetDatavf(engi_n2, n2, 0, 4) < 4)) return 0;
    if (which == 1 && (!engi_oil_ratio || XPLMGetDatavf(engi_oil_ratio, ratio, 0, 4) < 4)) return 0;
    if (which == 1 && engi_thrust) XPLMGetDatavf(engi_thrust, thrust, 0, 4);
    for (i = 0; i < n; ++i) {
        int e = offset + i;
        values[i] = (float)(which == 0 ? a380x_eng_n3_from_xp_n2(n2[e])
                                       : a380x_eng_oil_quantity_qt(ratio[e], thrust[e]));
    }
    return n;
}
static int engi_get_n3(void *refcon, float *v, int o, int m) { (void)refcon; return engi_read(v, o, m, 0); }
static int engi_get_oil(void *refcon, float *v, int o, int m) { (void)refcon; return engi_read(v, o, m, 1); }

static void engi_shutdown(void)
{
    int i;
    for (i = 0; i < 2; ++i)
        if (engi_out[i]) { XPLMUnregisterDataAccessor(engi_out[i]); engi_out[i] = NULL; }
}

/* Never fatal: a name another plugin already owns is left to it. */
static void engi_init(void)
{
    static const char *const names[2] = {"a380x/xp12/engine/n3_percent", "a380x/xp12/engine/oil_quantity_quarts"};
    int i;
    for (i = 0; i < 2; ++i)
        if (!engi_out[i] && !XPLMFindDataRef(names[i]))
            engi_out[i] = XPLMRegisterDataAccessor(names[i], xplmType_FloatArray, 0, NULL, NULL, NULL, NULL, NULL,
                NULL, NULL, NULL, i ? engi_get_oil : engi_get_n3, NULL, NULL, NULL, NULL, NULL);
}
