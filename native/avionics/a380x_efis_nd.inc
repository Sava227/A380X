/* SPDX-License-Identifier: GPL-3.0-or-later
 * Read-only, cached X-Plane navigation/FMS/TCAS/scenery sources for both NDs.
 * Include after a380x_efis.inc and before the flight loop. No synthetic traffic,
 * terrain or airport-layout geometry is generated when source data is absent. TERRAIN on ND has its
 * own cache and sweep (a380x_terrain_adapter.inc); nd_data.terrain[] holds its rasters.
 */
#include "a380x_efis_nd.h"
static EfisNdData nd_data;
static struct {
    XPLMDataRef latitude,longitude,true_heading,elevation;
    XPLMDataRef traffic_bearing,traffic_distance,traffic_altitude,traffic_count;
    XPLMDataRef nav_type;
    XPLMDataRef tas,wind_dir,wind_speed,zulu,magvar;
    XPLMDataRef vor_id[2],vor_frequency[2],vor_bearing[2],vor_dme[2],vor_has_dme[2];
    XPLMDataRef vor_deviation[2],vor_course[2];
    XPLMNavRef cursor;
    XPLMProbeRef probe;
    double scan_lat,scan_lon;
    float nav_age,slow_timer;
    int initialized;
} nd_source;

static void efis_nd_cache_nav(XPLMNavRef ref)
{
    XPLMNavType type=0;
    float lat=0.0f,lon=0.0f,dx,dy;
    int kind=-1,count,at;
    char id[256]={0};
    EfisNdNavPoint point;
    if (ref==XPLM_NAV_NOT_FOUND) return;
    XPLMGetNavAidInfo(ref,&type,&lat,&lon,NULL,NULL,NULL,id,NULL,NULL);
    if (type==xplm_Nav_Airport) kind=0;
    else if (type==xplm_Nav_VOR || type==xplm_Nav_DME || type==xplm_Nav_TACAN) kind=1;
    else if (type==xplm_Nav_NDB) kind=2;
    else if (type==xplm_Nav_Fix) kind=3;
    if (kind<0 || !efis_nd_project(nd_source.scan_lat,nd_source.scan_lon,
                                  lat,lon,0.0f,&dx,&dy)) return;
    point.distance_nm=hypotf(dx,dy);
    if (point.distance_nm>700.0f) return;
    point.lat=lat; point.lon=lon;
    snprintf(point.id,sizeof(point.id),"%.15s",id);
    count=nd_data.nav_count[kind];
    for (at=0;at<count;++at) {
        if (fabs(nd_data.nav[kind][at].lat-lat)<0.00001 &&
            fabs(nd_data.nav[kind][at].lon-lon)<0.00001 &&
            strcmp(nd_data.nav[kind][at].id,point.id)==0) return;
        if (point.distance_nm<nd_data.nav[kind][at].distance_nm) break;
    }
    if (at==EFIS_ND_NAV_MAX) return;
    if (count<EFIS_ND_NAV_MAX) ++count;
    memmove(&nd_data.nav[kind][at+1],&nd_data.nav[kind][at],
            (size_t)(count-at-1)*sizeof(point));
    nd_data.nav[kind][at]=point;
    nd_data.nav_count[kind]=count;
}

static void efis_nd_begin_scan(void)
{
    static const XPLMNavType types[]={xplm_Nav_Airport,xplm_Nav_VOR,xplm_Nav_NDB,xplm_Nav_Fix};
    float lat=(float)nd_data.lat,lon=(float)nd_data.lon;
    int i;
    nd_source.scan_lat=nd_data.lat; nd_source.scan_lon=nd_data.lon;
    memset(nd_data.nav_count,0,sizeof(nd_data.nav_count));
    /* Seed one real nearby item per layer immediately, then fill gradually. */
    for (i=0;i<4;++i)
        efis_nd_cache_nav(XPLMFindNavAid(NULL,NULL,&lat,&lon,NULL,types[i]));
    nd_source.cursor=XPLMGetFirstNavAid();
    nd_source.nav_age=0.0f;
    nd_data.nav_scanning=nd_source.cursor!=XPLM_NAV_NOT_FOUND;
}

static void efis_nd_read_route(void)
{
    int i,count=XPLMCountFMSEntries();
    nd_data.route_count=count<0?0:(count>EFIS_ND_ROUTE_MAX?EFIS_ND_ROUTE_MAX:count);
    nd_data.route_active=XPLMGetDestinationFMSEntry();
    nd_data.route_displayed=XPLMGetDisplayedFMSEntry();
    for (i=0;i<nd_data.route_count;++i) {
        char id[256]={0};
        float lat=NAN,lon=NAN;
        XPLMNavRef nav=XPLM_NAV_NOT_FOUND;
        EfisNdRoutePoint *point=&nd_data.route[i];
        XPLMGetFMSEntryInfo(i,NULL,id,&nav,&point->altitude_ft,&lat,&lon);
        point->lat=lat; point->lon=lon;
        snprintf(point->id,sizeof(point->id),"%.15s",id);
    }
}

static void efis_nd_read_radios(void)
{
    int i,types[12]={0};
    if (nd_source.nav_type) XPLMGetDatavi(nd_source.nav_type,types,0,12);
    for (i=0;i<2;++i) {
        EfisNdVor *vor=&nd_data.vor[i];
        memset(vor->id,0,sizeof(vor->id));
        if (nd_source.vor_id[i]) XPLMGetDatab(nd_source.vor_id[i],vor->id,0,15);
        vor->frequency_mhz=nd_source.vor_frequency[i]?
            XPLMGetDatai(nd_source.vor_frequency[i])*0.01f:0.0f;
        vor->bearing_mag=nd_source.vor_bearing[i]?XPLMGetDataf(nd_source.vor_bearing[i]):NAN;
        vor->distance_nm=nd_source.vor_dme[i]?XPLMGetDataf(nd_source.vor_dme[i]):NAN;
        vor->deviation_dots=nd_source.vor_deviation[i]?XPLMGetDataf(nd_source.vor_deviation[i]):NAN;
        vor->course_mag=nd_source.vor_course[i]?XPLMGetDataf(nd_source.vor_course[i]):NAN;
        /* nav_type uses XPLMNavType bit values, received station only. */
        vor->valid=(types[i]&xplm_Nav_VOR)!=0 && vor->id[0] && isfinite(vor->bearing_mag);
        vor->dme_valid=vor->valid && nd_source.vor_has_dme[i] &&
            XPLMGetDatai(nd_source.vor_has_dme[i]) && isfinite(vor->distance_nm);
    }
}

/* VD terrain profile: drawn only by the native FlyByWire-layout ND (a380x_nd_fbw.inc), which xp-kit
 * builds replace with the FlyByWire library ND. */
#if !defined(A380X_XPKIT)
/* Great-circle destination from (lat, lon) along true bearing `bearing_deg` for `distance_nm`. */
static void efis_nd_destination(double lat_deg, double lon_deg, double bearing_deg, double distance_nm,
                                double *out_lat, double *out_lon)
{
    double d = distance_nm / 3440.065, lat = lat_deg * M_PI / 180.0, lon = lon_deg * M_PI / 180.0;
    double b = bearing_deg * M_PI / 180.0;
    double lat2 = asin(sin(lat) * cos(d) + cos(lat) * sin(d) * cos(b));
    *out_lat = lat2 * 180.0 / M_PI;
    *out_lon = (lon + atan2(sin(b) * sin(d) * cos(lat), cos(d) - sin(lat) * sin(lat2))) * 180.0 / M_PI;
}

/* Terrain elevation (ft) and water flag at a point; 0 when the scenery there is not loaded. */
static int efis_nd_probe_ground(double lat, double lon, float *elevation_ft, int *water)
{
    double local_x, local_y, local_z, hit_lat, hit_lon, hit_alt;
    XPLMProbeInfo_t hit;
    XPLMWorldToLocal(lat, lon, 20000.0, &local_x, &local_y, &local_z);
    memset(&hit, 0, sizeof(hit)); hit.structSize = sizeof(hit);
    if (XPLMProbeTerrainXYZ(nd_source.probe, (float)local_x, (float)local_y, (float)local_z, &hit) != xplm_ProbeHitTerrain)
        return 0;
    XPLMLocalToWorld(hit.locationX, hit.locationY, hit.locationZ, &hit_lat, &hit_lon, &hit_alt);
    if (!isfinite(hit_alt)) return 0;
    *elevation_ft = (float)(hit_alt * 3.280839895);
    *water = hit.is_wet;
    return 1;
}

/* Vertical display terrain (A380 VD): EFIS_VD_COLUMNS columns from the aircraft to the selected
 * range along the VD axis (ground track; heading when slow). Each column keeps the highest terrain
 * of three probes across a +-EFIS_VD_CORRIDOR_NM corridor and is water only if all three are wet.
 * Six columns per side per frame refresh the whole profile about twice a second; a range change or
 * an axis change of more than 5 degrees restarts it. Only displayed in ROSE NAV and ARC. */
static void efis_nd_update_vd_terrain(int side)
{
    EfisVdTerrain *vd = &nd_data.vd[side];
    float range = efis[side].range_nm, axis;
    int n;
    if ((efis[side].mode != 2 && efis[side].mode != 3) || efis[side].zoom || !nd_data.position_valid ||
        !nd_source.probe || !isfinite(range) || range <= 0.0f) return;
    axis = ground_speed_knots > 40.0f && isfinite(ground_track_degrees) ? ground_track_degrees : nd_data.true_heading;
    if (!isfinite(axis)) return;
    if (vd->range_nm != range || fabsf(remainderf(axis - vd->axis_true, 360.0f)) > 5.0f) {
        memset(vd, 0, sizeof(*vd));
        vd->range_nm = range;
    }
    vd->axis_true = axis;
    for (n = 0; n < 6; ++n) {
        int column = vd->next_column, k, hits = 0, wet = 0;
        float distance = range * ((float)column + 0.5f) / (float)EFIS_VD_COLUMNS, highest = -1e9f;
        double centre_lat, centre_lon;
        vd->next_column = (column + 1) % EFIS_VD_COLUMNS;
        efis_nd_destination(nd_data.lat, nd_data.lon, axis, distance, &centre_lat, &centre_lon);
        for (k = -1; k <= 1; ++k) {
            double lat = centre_lat, lon = centre_lon;
            float elevation;
            int water;
            if (k) efis_nd_destination(centre_lat, centre_lon, axis + 90.0f * k, EFIS_VD_CORRIDOR_NM, &lat, &lon);
            if (!efis_nd_probe_ground(lat, lon, &elevation, &water)) continue;
            ++hits; wet += water != 0;
            if (elevation > highest) highest = elevation;
        }
        vd->valid[column] = hits > 0;
        vd->water[column] = hits > 0 && wet == hits;
        vd->elevation_ft[column] = hits > 0 ? highest : 0.0f;
    }
}
#endif

static void efis_nd_init(void)
{
    int i;
    char name[160];
    memset(&nd_source,0,sizeof(nd_source)); memset(&nd_data,0,sizeof(nd_data));
    nd_source.cursor=XPLM_NAV_NOT_FOUND;
#define ND_REF(field,path) nd_source.field=XPLMFindDataRef(path)
    ND_REF(latitude,"sim/flightmodel/position/latitude");
    ND_REF(longitude,"sim/flightmodel/position/longitude");
    ND_REF(true_heading,"sim/flightmodel/position/true_psi");
    ND_REF(elevation,"sim/flightmodel/position/elevation");
    ND_REF(traffic_bearing,"sim/cockpit2/tcas/indicators/relative_bearing_degs");
    ND_REF(traffic_distance,"sim/cockpit2/tcas/indicators/relative_distance_mtrs");
    ND_REF(traffic_altitude,"sim/cockpit2/tcas/indicators/relative_altitude_mtrs");
    ND_REF(traffic_count,"sim/cockpit2/tcas/indicators/tcas_num_acf");
    ND_REF(nav_type,"sim/cockpit2/radios/indicators/nav_type");
    ND_REF(tas,"sim/flightmodel/position/true_airspeed");
    ND_REF(wind_dir,"sim/weather/aircraft/wind_now_direction_degt");
    ND_REF(wind_speed,"sim/weather/aircraft/wind_now_speed_msc");
    ND_REF(zulu,"sim/time/zulu_time_sec");
    ND_REF(magvar,"sim/flightmodel/position/magnetic_variation");
#undef ND_REF
    for (i=0;i<2;++i) {
#define VOR_REF(field,section,stem) do { \
        snprintf(name,sizeof(name),"sim/cockpit2/radios/" section "/nav%d_" stem,i+1); \
        nd_source.field[i]=XPLMFindDataRef(name); \
    } while(0)
        VOR_REF(vor_id,"indicators","nav_id");
        VOR_REF(vor_frequency,"actuators","frequency_hz");
        VOR_REF(vor_bearing,"indicators","bearing_deg_mag");
        VOR_REF(vor_dme,"indicators","dme_distance_nm");
        VOR_REF(vor_has_dme,"indicators","has_dme");
        VOR_REF(vor_deviation,"indicators","hdef_dots_pilot");
        VOR_REF(vor_course,"actuators","obs_deg_mag_pilot");
#undef VOR_REF
    }
    nd_source.probe=XPLMCreateProbe(xplm_ProbeY);
    nd_source.slow_timer=1.0f;
}

static void efis_nd_shutdown(void)
{
    if (nd_source.probe) XPLMDestroyProbe(nd_source.probe);
    memset(&nd_source,0,sizeof(nd_source));
    memset(&nd_data,0,sizeof(nd_data));
}

static void efis_nd_update(float dt)
{
    int i;
    float dx=0,dy=0;
    dt=isfinite(dt)?fminf(1.0f,fmaxf(0.0f,dt)):0.0f;
    nd_data.lat=nd_source.latitude?XPLMGetDatad(nd_source.latitude):NAN;
    nd_data.lon=nd_source.longitude?XPLMGetDatad(nd_source.longitude):NAN;
    nd_data.true_heading=nd_source.true_heading?XPLMGetDataf(nd_source.true_heading):NAN;
    nd_data.altitude_ft=nd_source.elevation?(float)(XPLMGetDatad(nd_source.elevation)*3.280839895):NAN;
    nd_data.tas_knots=nd_source.tas?XPLMGetDataf(nd_source.tas)*1.94384449f:NAN;
    nd_data.wind_dir_true=nd_source.wind_dir?XPLMGetDataf(nd_source.wind_dir):NAN;
    nd_data.wind_speed_kt=nd_source.wind_speed?XPLMGetDataf(nd_source.wind_speed)*1.94384449f:NAN;
    nd_data.zulu_seconds=nd_source.zulu?XPLMGetDataf(nd_source.zulu):NAN;
    nd_data.magnetic_variation=nd_source.magvar?XPLMGetDataf(nd_source.magvar):NAN;
    nd_data.position_valid=isfinite(nd_data.lat)&&isfinite(nd_data.lon)&&fabs(nd_data.lat)<=90.0&&
        isfinite(nd_data.true_heading)&&isfinite(nd_data.altitude_ft);
    if (!nd_data.position_valid) return;
    nd_source.nav_age+=dt; nd_source.slow_timer+=dt;
    efis_nd_project(nd_source.scan_lat,nd_source.scan_lon,nd_data.lat,nd_data.lon,0,&dx,&dy);
    if (!nd_source.initialized || (!nd_data.nav_scanning && nd_source.nav_age>20.0f && hypotf(dx,dy)>5.0f)) {
        efis_nd_begin_scan(); nd_source.initialized=1;
    }
    /* Bound navigation API work to 512 navaids per frame, never a full global
     * navdb scan in one callback. Keep the nearest 64 of each display type. */
    for (i=0;i<512 && nd_source.cursor!=XPLM_NAV_NOT_FOUND;++i) {
        efis_nd_cache_nav(nd_source.cursor);
        nd_source.cursor=XPLMGetNextNavAid(nd_source.cursor);
    }
    nd_data.nav_scanning=nd_source.cursor!=XPLM_NAV_NOT_FOUND;
    if (nd_source.slow_timer>=0.5f) {
        nd_source.slow_timer=0.0f;
        efis_nd_read_route(); efis_nd_read_radios();
        nd_data.traffic_available=nd_source.traffic_bearing&&nd_source.traffic_distance&&nd_source.traffic_altitude;
        memset(nd_data.traffic_distance_m,0,sizeof(nd_data.traffic_distance_m));
        if (nd_data.traffic_available && (!nd_source.traffic_count || XPLMGetDatai(nd_source.traffic_count)>0)) {
            XPLMGetDatavf(nd_source.traffic_bearing,nd_data.traffic_bearing,0,64);
            XPLMGetDatavf(nd_source.traffic_distance,nd_data.traffic_distance_m,0,64);
            XPLMGetDatavf(nd_source.traffic_altitude,nd_data.traffic_altitude_m,0,64);
        }
    }
#if !defined(A380X_XPKIT)
    efis_nd_update_vd_terrain(0); efis_nd_update_vd_terrain(1);
#endif
}
