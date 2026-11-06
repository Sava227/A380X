/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef A380X_EFIS_ND_H
#define A380X_EFIS_ND_H
#include <math.h>
#include "a380x_terrain.h"
#define EFIS_ND_NAV_KINDS 4
#define EFIS_ND_NAV_MAX 64
#define EFIS_ND_ROUTE_MAX 100
/* Vertical display terrain: highest terrain in a corridor along the VD axis, per column. */
#define EFIS_VD_COLUMNS 96
#define EFIS_VD_CORRIDOR_NM 0.5f

typedef struct {
    double lat, lon;
    float distance_nm;
    char id[16];
} EfisNdNavPoint;
typedef struct {
    double lat, lon;
    int altitude_ft;
    char id[16];
} EfisNdRoutePoint;
typedef struct {
    float elevation_ft[EFIS_VD_COLUMNS];  /* highest corridor terrain, valid where valid[] */
    unsigned char valid[EFIS_VD_COLUMNS], water[EFIS_VD_COLUMNS];
    float range_nm, axis_true;            /* profile geometry the columns were sampled for */
    int next_column;
} EfisVdTerrain;
typedef struct {
    char id[16];
    float frequency_mhz, bearing_mag, distance_nm, deviation_dots, course_mag;
    int valid, dme_valid;
} EfisNdVor;
typedef struct {
    double lat, lon;
    float true_heading, altitude_ft;
    int position_valid, nav_scanning;
    EfisNdNavPoint nav[EFIS_ND_NAV_KINDS][EFIS_ND_NAV_MAX];
    int nav_count[EFIS_ND_NAV_KINDS];
    EfisNdRoutePoint route[EFIS_ND_ROUTE_MAX];
    int route_count, route_active, route_displayed;
    float traffic_bearing[64], traffic_distance_m[64], traffic_altitude_m[64];
    int traffic_available;
    EfisNdVor vor[2];
    TerrDisplay terrain[2];            /* TERRAIN on ND raster per ND (a380x_terrain_adapter.inc) */
    TerrPointer terrain_pointer[2];    /* simulator-aid elevation under the ND pointer */
    /* FBW ND header inputs: NAN when unavailable. Wind direction is true, "from". */
    float tas_knots, wind_dir_true, wind_speed_kt, zulu_seconds, magnetic_variation;
    EfisVdTerrain vd[2];
    /* TAWS look-ahead alerts (a380x_taws.h via a380x_terrain_adapter.inc), read by the aurals, the PFD
     * and the EGPWS discrete words; gpws_pull_up = GPWS modes 1/2 PULL UP from the aurals core. */
    int taws_caution, taws_warning, gpws_pull_up;
} EfisNdData;

/* Great-circle distance/bearing avoids longitude wrapping and polar-map errors.
 * x=east/right and y=north/up, in nautical miles relative to map heading. */
static inline int efis_nd_project(double origin_lat, double origin_lon,
                                  double target_lat, double target_lon,
                                  float heading, float *x, float *y)
{
    const double radians = 0.01745329251994329577;
    const double earth_radius_nm = 3440.065;
    double lat1, lat2, dlat, dlon, a, distance, bearing;
    if (!isfinite(origin_lat) || !isfinite(origin_lon) ||
        !isfinite(target_lat) || !isfinite(target_lon) || !isfinite(heading) ||
        fabs(origin_lat) > 90.0 || fabs(target_lat) > 90.0) return 0;
    lat1 = origin_lat * radians; lat2 = target_lat * radians;
    dlat = lat2-lat1; dlon = (target_lon-origin_lon)*radians;
    a = sin(dlat*.5)*sin(dlat*.5) + cos(lat1)*cos(lat2)*sin(dlon*.5)*sin(dlon*.5);
    a = fmin(1.0,fmax(0.0,a));
    distance = 2.0*earth_radius_nm*atan2(sqrt(a),sqrt(1.0-a));
    bearing = atan2(sin(dlon)*cos(lat2),
        cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlon)) - heading*radians;
    *x = (float)(distance*sin(bearing));
    *y = (float)(distance*cos(bearing));
    return isfinite(*x) && isfinite(*y);
}

static inline int efis_nd_in_map(float x, float y, float radius, int arc)
{
    if (!isfinite(x) || !isfinite(y) || x*x+y*y>radius*radius) return 0;
    /* ARC has exactly 100 degrees: 50 either side of the heading line. */
    return !arc || (y>=0.0f && fabsf(x)<=y*1.1917535926f);
}
#endif
