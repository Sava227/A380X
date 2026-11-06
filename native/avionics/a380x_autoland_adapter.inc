/* SPDX-License-Identifier: GPL-3.0-or-later
 * Distance to the tuned localizer antenna for the LAND / ROLL OUT laws (docs/AUTOLAND.md).
 * The FBW ROLL OUT law turns the localizer angle into a lateral deviation with nav_dme_nmi. X-Plane's
 * ILS DME is usually co-located with the glide antenna, abeam the touchdown zone, which would make
 * the roll-out centreline term several times too weak. The X-Plane navaid database gives the
 * localizer position by receiver frequency; nothing is written and no stock navigation is used.
 * Unity include of a380x_avionics.c after a380x_fcu_adapter.inc (fcu_localizer_distance_provider).
 */
typedef struct {
    XPLMDataRef latitude, longitude;
    int frequency[2], found[2];
    float search_at[2];
    double loc_lat[2], loc_lon[2];
} AutolandLocalizer;
static AutolandLocalizer autoland_loc;

static double autoland_localizer_distance(int side)
{
    AutolandLocalizer *a = &autoland_loc;
    double lat, lon, dlat, dlon, h;
    int frequency;
    float now = XPLMGetElapsedTime();
    if (side < 0 || side > 1) return -1;
    if (!a->latitude) a->latitude = XPLMFindDataRef("sim/flightmodel/position/latitude");
    if (!a->longitude) a->longitude = XPLMFindDataRef("sim/flightmodel/position/longitude");
    if (!a->latitude || !a->longitude) return -1;
    lat = XPLMGetDatad(a->latitude); lon = XPLMGetDatad(a->longitude);
    frequency = XPLMGetDatai(refs.side[side].nav_frequency);
    if (!isfinite(lat) || !isfinite(lon) || frequency <= 0) return -1;
    /* Re-resolve on a frequency change, and every 20 s (a nearer same-frequency localizer). */
    if (frequency != a->frequency[side] || now >= a->search_at[side] || now + 60.0f < a->search_at[side]) {
        float flat = (float)lat, flon = (float)lon, olat = 0, olon = 0;
        int freq = frequency;
        XPLMNavRef ref = XPLMFindNavAid(NULL, NULL, &flat, &flon, &freq, xplm_Nav_ILS | xplm_Nav_Localizer);
        a->frequency[side] = frequency; a->search_at[side] = now + 20.0f; a->found[side] = 0;
        if (ref != XPLM_NAV_NOT_FOUND) {
            XPLMGetNavAidInfo(ref, NULL, &olat, &olon, NULL, NULL, NULL, NULL, NULL, NULL);
            if (isfinite(olat) && isfinite(olon)) { a->loc_lat[side] = olat; a->loc_lon[side] = olon; a->found[side] = 1; }
        }
    }
    if (!a->found[side]) return -1;
    dlat = (a->loc_lat[side] - lat) * 0.017453292519943295;
    dlon = (a->loc_lon[side] - lon) * 0.017453292519943295;
    h = sin(dlat / 2) * sin(dlat / 2) + cos(lat * 0.017453292519943295) *
        cos(a->loc_lat[side] * 0.017453292519943295) * sin(dlon / 2) * sin(dlon / 2);
    return 2.0 * asin(fmin(1.0, sqrt(h))) * 3440.065;   /* mean Earth radius in nm */
}
