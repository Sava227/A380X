/* SPDX-License-Identifier: GPL-3.0-or-later
 * FMGC pseudo waypoints on the native NDs (both layouts; included by a380x_pfd_render.inc): the
 * symbols of FBW's ND PseudoWaypointLayer (fbw-a380x ND map PseudoWaypointLayer.ts paths, FBW px,
 * y-down) for the list the FMS adapter copies from the FMGC (fmsa_nd_pwp, a380x_fmgc_guidance.h):
 * T/C and level-off arrows, start of climb / descent (T/D), DECEL, speed change dot, profile
 * intercept. On the path at their position (NAV), or along the present track (HDG/TRK level-off and
 * intercept arrows, rotated with the track as FBW does). Colours: managed magenta, selected cyan,
 * white. Scale: FBW px on the 768 x 1024 layout (sym 3), proportionally smaller on the native one.
 */
static const float ND_PWP_LEVEL_CLIMB[] = {-42.0f, 16.2f, -22.2f, 0.0f, 0.0f, 0.0f};
static const float ND_PWP_LEVEL_DESCENT[] = {-42.0f, -16.2f, -22.2f, 0.0f, 0.0f, 0.0f};
static const float ND_PWP_ARROW_HEAD[] = {-4.2f, -4.2f, 0.0f, 0.0f, -4.2f, 4.2f};
static const float ND_PWP_START_CLIMB[] = {0.0f, 0.0f, 22.2f, 0.0f, 42.0f, -16.2f};
static const float ND_PWP_START_CLIMB_HEAD[] = {36.0f, -16.2f, 42.0f, -16.2f, 42.0f, -10.2f};
static const float ND_PWP_START_DESCENT[] = {0.0f, 0.0f, 22.2f, 0.0f, 42.0f, 16.2f};
static const float ND_PWP_START_DESCENT_HEAD[] = {36.0f, 16.2f, 42.0f, 16.2f, 42.0f, 10.2f};
static const float ND_PWP_INTERCEPT[] = {-38.0f, 0.0f, -24.0f, -17.0f, -24.0f, 17.0f, -10.0f, 0.0f, 0.0f, 0.0f};
static const float ND_PWP_INTERCEPT_HEAD[] = {-5.0f, -5.0f, 0.0f, 0.0f, -5.0f, 5.0f};
static const float ND_PWP_DECEL_D[] = {2.3625f, -9.7875f, -5.4f, -9.7875f, -5.4f, 8.4375f, 2.3625f, 8.4375f,
                                       5.0625f, 5.0625f, 5.0625f, -6.4125f, 2.3625f, -9.7875f};

/* FBW path points (y-down) at (x, y), scale k, rotated clockwise by rot_deg. */
static void nd_pwp_polyline(float x, float y, float k, float rot_deg, const float *xy, int n, Color color)
{
    const float c = cosf(rot_deg * (float)M_PI / 180.0f), s = sinf(rot_deg * (float)M_PI / 180.0f);
    int i;
    set_color(color);
    glBegin(GL_LINE_STRIP);
    for (i = 0; i < n; ++i) {
        const float fx = xy[2 * i] * k, fy = -xy[2 * i + 1] * k;
        glVertex2f(x + fx * c + fy * s, y - fx * s + fy * c);
    }
    glEnd();
}

static void nd_pwp_symbol(float x, float y, float k, float rot, unsigned symbol, Color color)
{
    int i;
#define ND_PWP_POLY(points) nd_pwp_polyline(x, y, k, rot, points, (int)(sizeof(points) / sizeof(points[0]) / 2), color)
    switch (symbol) {
    case A380X_PWP_CLIMB_LEVEL_OFF: ND_PWP_POLY(ND_PWP_LEVEL_CLIMB); ND_PWP_POLY(ND_PWP_ARROW_HEAD); break;
    case A380X_PWP_DESCENT_LEVEL_OFF: ND_PWP_POLY(ND_PWP_LEVEL_DESCENT); ND_PWP_POLY(ND_PWP_ARROW_HEAD); break;
    case A380X_PWP_START_OF_CLIMB: ND_PWP_POLY(ND_PWP_START_CLIMB); ND_PWP_POLY(ND_PWP_START_CLIMB_HEAD); break;
    case A380X_PWP_START_OF_DESCENT: ND_PWP_POLY(ND_PWP_START_DESCENT); ND_PWP_POLY(ND_PWP_START_DESCENT_HEAD); break;
    case A380X_PWP_INTERCEPT_PROFILE: ND_PWP_POLY(ND_PWP_INTERCEPT); ND_PWP_POLY(ND_PWP_INTERCEPT_HEAD); break;
    case A380X_PWP_DECEL: {
        float ring[66];
        for (i = 0; i <= 32; ++i) {
            ring[2 * i] = 14.5125f * cosf(i * (float)M_PI / 16.0f);
            ring[2 * i + 1] = 14.5125f * sinf(i * (float)M_PI / 16.0f);
        }
        nd_pwp_polyline(x, y, k, 0.0f, ring, 33, color);
        nd_pwp_polyline(x, y, k, 0.0f, ND_PWP_DECEL_D, 7, color);
        break;
    }
    case A380X_PWP_SPEED_CHANGE:
        set_color(color);
        glBegin(GL_TRIANGLE_FAN);
        glVertex2f(x, y);
        for (i = 0; i <= 16; ++i) glVertex2f(x + 8.0f * k * cosf(i * (float)M_PI / 8.0f), y + 8.0f * k * sinf(i * (float)M_PI / 8.0f));
        glEnd();
        break;
    default:
        break;
    }
#undef ND_PWP_POLY
}

/* The ACTIVE plan's pseudo waypoints over the route (drawn by nd_draw_fms_route). */
static void nd_draw_fmgc_pwps(const EfisNdCanvas *map, int side)
{
    const float k = map->sym / 3.0f, px_per_nm = map->radius / map->range_nm;
    int i;
    for (i = 0; i < fmsa_nd_pwp_count && i < A380X_FMGC_MAX_ND_PWP; ++i) {
        const A380XFmgcNdPwp *w = &fmsa_nd_pwp[i];
        const Color color = w->colour == A380X_PWP_MAGENTA ? MAGENTA : w->colour == A380X_PWP_CYAN ? CYAN
                          : w->colour == A380X_PWP_AMBER ? AMBER : WHITE;
        float px, py, rot = 0.0f;
        if (w->nd == A380X_PWP_ND_PATH) {
            if (!isfinite(w->lat) || !isfinite(w->lon) || !nd_map_point(map, w->lat, w->lon, &px, &py)) continue;
        } else if (w->nd == A380X_PWP_ND_TRACK && isfinite(w->dist_ahead_nm)) {
            /* along the track line from the aircraft: true track = true heading + drift, map-relative */
            const Side *s = &refs.side[side ? 1 : 0];
            float ox, oy, drift = s->heading_valid ? remainderf(ground_track_degrees - s->heading_value, 360.0f) : 0.0f;
            if (!nd_map_point(map, nd_data.lat, nd_data.lon, &ox, &oy)) continue;
            rot = remainderf(nd_data.true_heading + drift - map->heading, 360.0f);
            px = ox + (float)w->dist_ahead_nm * px_per_nm * sinf(rot * (float)M_PI / 180.0f);
            py = oy + (float)w->dist_ahead_nm * px_per_nm * cosf(rot * (float)M_PI / 180.0f);
        } else continue;
        if (!nd_map_visible(map, px, py, 4.0f * map->sym)) continue;
        nd_pwp_symbol(map->cx + px, map->cy + py, k, rot, w->symbol, color);
    }
}
