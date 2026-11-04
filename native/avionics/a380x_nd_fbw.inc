/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * A380 ND in ARC, ROSE (NAV/VOR/LS) and PLAN modes, laid out on the FlyByWire 768 x 1024 ND
 * canvas (768 x 768 map plus the
 * A380 vertical display below it). Geometry, colours and font sizes are taken from the pinned
 * FBW sources (fbw-common/src/systems/instruments/src/ND: pages/arc/ArcModeUnderlay.tsx,
 * pages/rose/RoseModeUnderlay.tsx, pages/rose/RoseNavPage.tsx, pages/plan/PlanModeUnderlay.tsx,
 * shared/Airplane.tsx, pages/arc/LubberLine.tsx, shared/TrackBug.tsx,
 * pages/arc/SelectedHeadingBug.tsx, ND.tsx header layers, shared/RadioNavInfo.tsx;
 * fbw-a380x/.../ND/VerticalDisplay/VerticalDisplay.tsx). FBW coordinates are y-down; NDY()
 * converts them for the y-up cockpit device. Map layers (route, navaids, traffic, terrain)
 * reuse the native ND layers through an EfisNdCanvas sized for this canvas.
 *
 * Not yet ported: wind arrow, off-scale selected-heading readout, ROSE VOR/LS course pointers and
 * deviation scales, the VD flight-path profile and the TCAS range-ring variants.
 */

#define ND_FBW_W 768.0f
#define ND_FBW_H 1024.0f
#define NDY(y) (ND_FBW_H - (y))
#define ND_FBW_CX 384.0f
#define ND_FBW_CY 620.0f            /* ARC aircraft reference, FBW y-down */
#define ND_FBW_ARC_RADIUS 492.0f    /* ARC compass ring */
#define ND_FBW_ARC_MAP_RADIUS 498.0f /* arc/index.tsx set_map_pixel_radius: selected range */
#define ND_FBW_ROSE_CY 384.0f       /* ROSE/PLAN centre */
#define ND_FBW_ROSE_RADIUS 250.0f   /* ROSE: range/2, PLAN: range */

/* FBW ND font classes ($font-factor 5): FontSmallest 22.5, FontSmall 25, FontIntermediate 27.5,
 * FontMedium 30, FontLarge 32.5. */
#define ND_FONT_SMALLEST 22.5f
#define ND_FONT_SMALL 25.0f
#define ND_FONT_INTERMEDIATE 27.5f
#define ND_FONT_LARGE 32.5f

static void ndf_line(float x1, float y1, float x2, float y2, float width, Color color)
{
    glLineWidth(width);
    line(x1, NDY(y1), x2, NDY(y2), color);
}

static void ndf_text(float x, float y, const char *value, float size, Color color, int anchor)
{
    ewd_font_draw(x, NDY(y), value, size, color, 0, anchor);
}

/* Point at `deg` clockwise from straight up, `r` from centre (cx, cy) (FBW y-down). */
static void ndf_polar_at(float cx, float cy, float deg, float r, float *x, float *y)
{
    float a = deg * (float)M_PI / 180.0f;
    *x = cx + r * sinf(a);
    *y = cy - r * cosf(a);
}


static float ndf_wrap180(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/* Polyline through `n` points rotated by `deg` (clockwise) about (cx, cy), FBW y-down. */
static void ndf_rotated_polyline(float cx, float cy, const float *xy, int n, int closed, float deg, float width, Color color)
{
    float a = deg * (float)M_PI / 180.0f, c = cosf(a), s = sinf(a);
    int i;
    glLineWidth(width);
    set_color(color);
    glBegin(GL_LINE_STRIP);
    for (i = 0; i < n + (closed ? 1 : 0); ++i) {
        float dx = xy[2 * (i % n)] - cx, dy = xy[2 * (i % n) + 1] - cy;
        glVertex2f(cx + dx * c - dy * s, NDY(cy + dx * s + dy * c));
    }
    glEnd();
}

/* Filled triangle (FBW y-down points) rotated by `deg` about (cx, cy). */
static void ndf_rotated_triangle(float cx, float cy, const float *xy, float deg, Color color)
{
    float a = deg * (float)M_PI / 180.0f, c = cosf(a), s = sinf(a);
    int i;
    set_color(color);
    glBegin(GL_TRIANGLE_FAN);
    for (i = 0; i < 3; ++i) {
        float dx = xy[2 * i] - cx, dy = xy[2 * i + 1] - cy;
        glVertex2f(cx + dx * c - dy * s, NDY(cy + dx * s + dy * c));
    }
    glEnd();
}

/* Upper limit (FBW y) of the V-shaped range-ring clip: left edge, centre, right edge. */
static float ndf_v_limit(float x, float left, float centre, float right)
{
    return x < ND_FBW_CX ? left + (centre - left) * (x / ND_FBW_CX)
                         : centre + (right - centre) * ((x - ND_FBW_CX) / ND_FBW_CX);
}

/* Dashed range ring clipped to the FBW overlay polygon above a V line. */
static void ndf_dashed_ring(float r, float dash, float gap, float left, float centre, float right, Color color)
{
    const float step = 2.0f / r; /* ~2 px along the arc */
    float a, travelled = 0.0f;
    glLineWidth(2.0f);
    set_color(color);
    for (a = -(float)M_PI; a < (float)M_PI; a += step, travelled += 2.0f) {
        float x1 = ND_FBW_CX + r * sinf(a), y1 = ND_FBW_CY - r * cosf(a);
        float x2 = ND_FBW_CX + r * sinf(a + step), y2 = ND_FBW_CY - r * cosf(a + step);
        if (fmodf(travelled, dash + gap) >= dash) continue;
        if (y1 > ndf_v_limit(x1, left, centre, right) || y2 > ndf_v_limit(x2, left, centre, right)) continue;
        glBegin(GL_LINES);
        glVertex2f(x1, NDY(y1)); glVertex2f(x2, NDY(y2));
        glEnd();
    }
}

/* Dashed circle about (cx, cy), FBW y-down. */
static void ndf_dashed_circle(float cx, float cy, float r, float dash, float gap, float width, Color color)
{
    const float step = 2.0f / r;
    float a, travelled = 0.0f;
    glLineWidth(width);
    set_color(color);
    glBegin(GL_LINES);
    for (a = 0.0f; a < 2.0f * (float)M_PI; a += step, travelled += 2.0f) {
        if (fmodf(travelled, dash + gap) >= dash) continue;
        glVertex2f(cx + r * sinf(a), NDY(cy - r * cosf(a)));
        glVertex2f(cx + r * sinf(a + step), NDY(cy - r * cosf(a + step)));
    }
    glEnd();
}

static void ndf_range_label(float value, float x, float y, int anchor)
{
    char text[16];
    if (fabsf(value - roundf(value)) < 0.01f) snprintf(text, sizeof(text), "%.0f", value);
    else snprintf(text, sizeof(text), "%.1f", value);
    ndf_text(x, y, text, ND_FONT_SMALLEST, CYAN, anchor);
}

/* Heading ring about (cx, cy): ticks every 5 degrees (10-degree ticks longer), labels every
 * `label_step` degrees rotated with the ring, drawn within +-limit of the top (180 = full circle). */
static void ndf_compass_ring(float cx, float cy, float radius, float major_r, float minor_r, float label_r,
                             float limit, int label_step, float big_size, float small_size, float width,
                             float heading_mag)
{
    int h;
    float a;
    glLineWidth(width);
    set_color(WHITE);
    glBegin(GL_LINE_STRIP);
    for (a = -limit; a <= limit + 0.01f; a += 0.5f) {
        float x, y;
        ndf_polar_at(cx, cy, a, radius, &x, &y);
        glVertex2f(x, NDY(y));
    }
    glEnd();
    for (h = 0; h < 360; h += 5) {
        float rel = ndf_wrap180((float)h - heading_mag), x1, y1, x2, y2;
        if (fabsf(rel) > limit) continue;
        ndf_polar_at(cx, cy, rel, radius, &x1, &y1);
        ndf_polar_at(cx, cy, rel, h % 10 == 0 ? major_r : minor_r, &x2, &y2);
        ndf_line(x1, y1, x2, y2, width, WHITE);
        if (h % label_step == 0) {
            char label[8];
            float lx, ly;
            snprintf(label, sizeof(label), "%d", h / 10);
            ndf_polar_at(cx, cy, rel, label_r, &lx, &ly);
            glPushMatrix();
            glTranslatef(lx, NDY(ly), 0.0f);
            glRotatef(-rel, 0.0f, 0.0f, 1.0f);
            ewd_font_draw(0.0f, 0.0f, label, h % 30 == 0 ? big_size : small_size, WHITE, 0, 1);
            glPopMatrix();
        }
    }
}

/* ARC: R = 492 ring, visible where FBW's overlay clip (x 6..762, y <= 768) keeps it: +-50 deg. */
static void ndf_compass(float heading_mag)
{
    ndf_compass_ring(ND_FBW_CX, ND_FBW_CY, ND_FBW_ARC_RADIUS, 521.0f, 507.0f, 529.0f, 50.5f, 10, 34.0f, 22.0f, 2.0f, heading_mag);
}

static void ndf_aircraft_and_bugs(const Side *side, float heading_mag)
{
    /* Aircraft symbol (shared/Airplane.tsx): wings, fuselage, tail. */
    ndf_line(343.0f, ND_FBW_CY, 425.0f, ND_FBW_CY, 5.0f, YELLOW);
    ndf_line(384.0f, 590.5f, 384.0f, 660.75f, 5.0f, YELLOW);
    ndf_line(372.5f, 651.0f, 396.0f, 651.0f, 5.0f, YELLOW);
    /* Lubber line (pages/arc/LubberLine.tsx). */
    ndf_line(384.0f, 108.0f, 384.0f, 148.0f, 5.0f, YELLOW);
    if (!side->heading_valid) return;
    /* Track diamond (shared/TrackBug.tsx), green, from the ground track converted to magnetic. */
    {   /* FBW IR track: the heading below 50 kt ground speed (adirs.rs), so the diamond sits at the lubber line. */
        static const float diamond[] = {384, 128, 378, 138, 384, 148, 390, 138};
        float rel = 0.0f;
        if (ground_speed_knots >= 50.0f && isfinite(ground_track_degrees) && isfinite(nd_data.true_heading))
            rel = ndf_wrap180(ground_track_degrees - (nd_data.true_heading - heading_mag) - heading_mag);
        if (fabsf(rel) <= 50.0f) ndf_rotated_polyline(ND_FBW_CX, ND_FBW_CY, diamond, 4, 1, rel, 3.0f, GREEN);
    }
    /* Selected heading bug (pages/arc/SelectedHeadingBug.tsx), cyan, while HDG is selected. */
    if (!pfd_guidance.heading_managed && isfinite(pfd_guidance.selected_heading_deg)) {
        static const float bug[] = {382, 126, 370, 99, 398, 99, 386, 126};
        float rel = ndf_wrap180((float)pfd_guidance.selected_heading_deg - heading_mag);
        if (fabsf(rel) <= 50.0f) ndf_rotated_polyline(ND_FBW_CX, ND_FBW_CY, bug, 4, 0, rel, 3.0f, CYAN);
    }
}

/* ND.tsx SpeedIndicator (Layer 2,25) and WindIndicator (Layer 23,58). */
static void ndf_header_left(const Side *side)
{
    char value[16];
    int air_valid = isfinite(nd_data.tas_knots) && nd_data.tas_knots >= 60.0f;
    ndf_text(2.0f, 25.0f, "GS", ND_FONT_SMALLEST, WHITE, 0);
    if (side->navigation_valid) snprintf(value, sizeof(value), "%.0f", fmaxf(0.0f, ground_speed_knots));
    else snprintf(value, sizeof(value), "---");
    ndf_text(91.0f, 25.0f, value, ND_FONT_INTERMEDIATE, GREEN, 2);
    ndf_text(97.0f, 25.0f, "TAS", ND_FONT_SMALLEST, WHITE, 0);
    if (air_valid) snprintf(value, sizeof(value), "%.0f", nd_data.tas_knots);
    else snprintf(value, sizeof(value), "---");
    ndf_text(203.0f, 25.0f, value, ND_FONT_INTERMEDIATE, GREEN, 2);
    /* Wind: magnetic "from" direction / speed; dashes below 100 kt TAS as on FBW. */
    if (air_valid && nd_data.tas_knots >= 100.0f && isfinite(nd_data.wind_dir_true) && isfinite(nd_data.wind_speed_kt)) {
        float dir = fmodf(nd_data.wind_dir_true - (isfinite(nd_data.magnetic_variation) ? nd_data.magnetic_variation : 0.0f) + 360.0f, 360.0f);
        snprintf(value, sizeof(value), "%03.0f", dir);
        ndf_text(48.0f, 58.0f, value, ND_FONT_SMALL, GREEN, 2);
        snprintf(value, sizeof(value), "%.0f", nd_data.wind_speed_kt);
        ndf_text(73.0f, 58.0f, value, ND_FONT_SMALL, GREEN, 0);
    } else {
        ndf_text(48.0f, 58.0f, "---", ND_FONT_SMALL, GREEN, 2);
        ndf_text(73.0f, 58.0f, "---", ND_FONT_SMALL, GREEN, 0);
    }
    ndf_text(54.0f, 57.0f, "/", ND_FONT_SMALLEST, WHITE, 0);
}

/* ND.tsx ToWaypointIndicator (Layer 690,25): TO waypoint ident, magnetic bearing, distance, ETA. */
static void ndf_header_to_waypoint(const Side *side)
{
    const A380XFplnPlan *p;
    const A380XFplnLeg *to;
    float east, north, dist, bearing_true, bearing_mag, mag_offset;
    char value[24];
    if (!fmsa_fpln || !nd_data.position_valid) return;
    p = a380x_fpln_plan(fmsa_fpln, A380X_PLAN_ACTIVE);
    if (!p || !p->exists || p->active_leg_index < 0 || p->active_leg_index >= p->pri.count) return;
    to = &p->pri.el[p->active_leg_index];
    if (a380x_fpln_is_disco(to) || !isfinite(to->term_lat) || !isfinite(to->term_lon)) return;
    if (!efis_nd_project(nd_data.lat, nd_data.lon, to->term_lat, to->term_lon, 0.0f, &east, &north)) return;
    dist = hypotf(east, north);
    bearing_true = atan2f(east, north) * 180.0f / (float)M_PI;
    mag_offset = side->heading_valid && isfinite(nd_data.true_heading) ? nd_data.true_heading - side->heading_value : 0.0f;
    bearing_mag = fmodf(bearing_true - mag_offset + 720.0f, 360.0f);
    ndf_text(677.0f, 25.0f, to->ident[0] ? to->ident : "PPOS", ND_FONT_INTERMEDIATE, WHITE, 2);
    snprintf(value, sizeof(value), "%03.0f", bearing_mag);
    ndf_text(747.0f, 25.0f, value, ND_FONT_INTERMEDIATE, GREEN, 2);
    ndf_text(763.0f, 27.0f, "\xc2\xb0", ND_FONT_INTERMEDIATE, CYAN, 2);
    if (dist < 20.0f) snprintf(value, sizeof(value), "%.1f", dist);
    else snprintf(value, sizeof(value), "%.0f", dist);
    ndf_text(729.0f, 57.0f, value, ND_FONT_INTERMEDIATE, GREEN, 2);
    ndf_text(762.0f, 57.0f, "NM", ND_FONT_SMALLEST, CYAN, 2);
    if (ground_speed_knots > 20.0f && isfinite(nd_data.zulu_seconds)) {
        int eta = (int)lroundf(nd_data.zulu_seconds + dist / ground_speed_knots * 3600.0f) % 86400;
        snprintf(value, sizeof(value), "%02d:%02d", eta / 3600, (eta / 60) % 60);
        ndf_text(762.0f, 91.0f, value, ND_FONT_INTERMEDIATE, GREEN, 2);
    }
}

/* shared/RadioNavInfo.tsx: VOR1 bottom-left (x 37), VOR2 bottom-right, when selected on the EFIS. */
static void ndf_radio_info(int index)
{
    int i;
    for (i = 0; i < 2; ++i) {
        const EfisNdVor *vor = &nd_data.vor[i];
        int selected = i == 0 ? efis[index].vor1 : efis[index].vor2;
        int anchor = i == 0 ? 0 : 2;
        float x = i == 0 ? 37.0f : 731.0f;
        char value[24];
        if (!selected) continue;
        ndf_text(x, 692.0f, i == 0 ? "VOR1" : "VOR2", 24.0f, WHITE, anchor);
        if (vor->id[0]) ndf_text(x, 722.0f, vor->id, 24.0f, GREEN, anchor);
        else if (vor->frequency_mhz > 0.0f) {
            snprintf(value, sizeof(value), "%.2f", vor->frequency_mhz);
            ndf_text(x, 722.0f, value, 24.0f, GREEN, anchor);
        }
        if (vor->id[0] && isfinite(vor->distance_nm) && vor->distance_nm > 0.0f) {
            snprintf(value, sizeof(value), "%.0f", vor->distance_nm);
            if (i == 0) {
                ndf_text(x + 60.0f, 759.0f, value, 24.0f, GREEN, 2);
                ndf_text(x + 66.0f, 759.0f, "NM", 20.0f, CYAN, 0);
            } else {
                ndf_text(x - 34.0f, 759.0f, value, 24.0f, GREEN, 2);
                ndf_text(x, 759.0f, "NM", 20.0f, CYAN, 2);
            }
        }
    }
}

/* VD terrain (efis_nd_update_vd_terrain): brown fill under the corridor profile, cyan where the
 * whole corridor is water, magenta MORA (highest terrain + 1000 ft, + 2000 ft above 5000 ft,
 * in hundreds of feet) under its column. Columns not yet sampled or outside loaded scenery are
 * left empty rather than guessed. */
static void ndf_vd_terrain(int index, float range_nm, float x0, float x1, float top, float bottom,
                           float zero_y, float lo, float span)
{
    static const Color VD_TERRAIN = {0.48f, 0.24f, 0.09f};
    const EfisVdTerrain *vd = &nd_data.vd[index];
    float highest = -1e9f, highest_x = 0.0f;
    int i;
    if (vd->range_nm != range_nm) return;   /* profile still being rebuilt for this range */
    for (i = 0; i < EFIS_VD_COLUMNS; ++i) {
        float left = x0 + (x1 - x0) * i / EFIS_VD_COLUMNS, right = x0 + (x1 - x0) * (i + 1) / EFIS_VD_COLUMNS;
        float y;
        if (!vd->valid[i]) continue;
        y = zero_y - (vd->elevation_ft[i] - lo) * (zero_y - top) / span;
        y = fmaxf(top, fminf(bottom, y));
        rectangle(left, NDY(bottom), right, NDY(y), vd->water[i] ? CYAN : VD_TERRAIN);
        if (vd->elevation_ft[i] > highest) { highest = vd->elevation_ft[i]; highest_x = 0.5f * (left + right); }
    }
    if (highest > -1e8f) {
        char value[16];
        float mora = ceilf((fmaxf(0.0f, highest) + (highest > 5000.0f ? 2000.0f : 1000.0f)) / 100.0f);
        float x = fmaxf(x0 + 40.0f, fminf(x1 - 60.0f, highest_x));
        snprintf(value, sizeof(value), "%.0f", mora);
        ndf_text(x, 1021.0f, value, 20.0f, MAGENTA, 2);
        ndf_text(x + 6.0f, 1021.0f, "MORA", 20.0f, MAGENTA, 0);
    }
}

/* A380 vertical display frame (VerticalDisplay.tsx): altitude tape x 105, distance scale
 * x 150..690 over y 800..1000, own-altitude marker, terrain profile. The FMS vertical profile is
 * not drawn yet. */
static void ndf_vertical_display(float range_nm, int index)
{
    const float x0 = 150.0f, x1 = 690.0f, top = 800.0f, bottom = 1000.0f, zero_y = 960.0f, span = 24000.0f;
    float alt = isfinite(nd_data.altitude_ft) ? nd_data.altitude_ft : 0.0f;
    float lo = alt < 18000.0f ? 0.0f : floorf((alt - 12000.0f) / 5000.0f) * 5000.0f;
    float a;
    int k;
    char value[16];
    ndf_vd_terrain(index, range_nm, x0, x1, top, bottom, zero_y, lo, span);
    /* Distance scale: solid baseline and ends, dashed quarter lines, cyan range labels. */
    ndf_line(x0, top, x1, top, 2.0f, WHITE);
    ndf_line(x1, top, x1, bottom, 2.0f, WHITE);
    for (k = 0; k < 4; ++k) {
        float x = x0 + (x1 - x0) * k / 4.0f, y;
        for (y = top; y < bottom; y += 16.0f) ndf_line(x, y, x, fminf(y + 8.0f, bottom), 2.0f, WHITE);
    }
    for (k = 0; k <= 4; ++k) ndf_range_label(range_nm * k / 4.0f, x0 + (x1 - x0) * k / 4.0f, 797.0f, 1);
    /* Altitude tape: ticks every 5000 ft, labels every 10000 ft. */
    ndf_line(105.0f, top, 105.0f, bottom, 2.0f, WHITE);
    for (a = lo; a <= lo + span; a += 5000.0f) {
        float y = zero_y - (a - lo) * (zero_y - top) / span;
        if (y < top || y > bottom) continue;
        ndf_line(105.0f, y, 120.0f, y, 2.0f, WHITE);
        if (fmodf(a, 10000.0f) == 0.0f) {
            snprintf(value, sizeof(value), "%.0f", a);
            ndf_text(95.0f, y + 8.0f, value, ND_FONT_SMALLEST, WHITE, 2);
        }
    }
    if (isfinite(nd_data.altitude_ft)) {
        float y = zero_y - (alt - lo) * (zero_y - top) / span;
        if (y >= top && y <= bottom) ndf_line(97.0f, y, 115.0f, y, 4.0f, YELLOW);
    }
}

/* Whole ARC page on the 768 x 1024 device canvas. */
static void draw_nd_fbw_arc(const Side *side, int index)
{
    const int position_valid = side->navigation_valid && nd_data.position_valid;
    const float heading_mag = side->heading_valid ? side->heading_value : 0.0f;
    float range_nm = isfinite(side->nd_range_nm) && side->nd_range_nm > 0 ? side->nd_range_nm : 80.0f;
    const int saved_atlas = text_use_atlas;
    const float saved_size = text_atlas_size;
    EfisNdCanvas map;
    rectangle(0.0f, 0.0f, ND_FBW_W, ND_FBW_H, BLACK);
    if (!side->display_valid) return;
    /* Map-layer labels (text_at) use the EIS atlas at FBW's waypoint label size. */
    text_use_atlas = 1; text_atlas_size = 22.0f;
    memset(&map, 0, sizeof(map));
    map.cx = ND_FBW_CX; map.cy = NDY(ND_FBW_CY);
    map.radius = ND_FBW_ARC_MAP_RADIUS; map.range_nm = range_nm;
    map.heading = side->heading_valid && isfinite(nd_data.true_heading) ? nd_data.true_heading : 0.0f;
    map.lat = nd_data.lat; map.lon = nd_data.lon;
    map.arc = 0; /* the FBW ARC map is clipped by the box below, not to the +-50 degree wedge */
    map.clip_l = 6.0f; map.clip_r = 762.0f; map.clip_b = NDY(690.0f); map.clip_t = NDY(80.0f);
    map.label_r = 700.0f; map.sym = 3.0f;
    if (position_valid) {
        glLineWidth(2.5f);
        nd_draw_terrain(&map, index);
        nd_draw_route(&map, index);
        nd_draw_nav_layers(&map, index);
        nd_draw_traffic(&map, index);
    }
    /* Range rings R = 369 (3/4), 246 (1/2), 123 (1/4) with FBW's V-shaped lower clips. */
    ndf_dashed_ring(369.0f, 15.0f, 10.5f, 564.0f, 709.0f, 564.0f, WHITE);
    ndf_dashed_ring(246.0f, 15.0f, 10.0f, 532.0f, 687.0f, 541.0f, WHITE);
    ndf_dashed_ring(123.0f, 15.0f, 10.0f, 519.0f, 664.0f, 578.0f, WHITE);
    ndf_range_label(range_nm * 0.75f, 58.0f, 482.0f, 0);
    ndf_range_label(range_nm * 0.75f, 709.0f, 482.0f, 2);
    ndf_range_label(range_nm * 0.5f, 175.0f, 528.0f, 0);
    ndf_range_label(range_nm * 0.5f, 592.0f, 528.0f, 2);
    if (side->heading_valid) ndf_compass(heading_mag);
    else ndf_text(384.0f, 241.0f, "HDG", ND_FONT_LARGE, RED, 1);
    ndf_aircraft_and_bugs(side, heading_mag);
    ndf_header_left(side);
    ndf_header_to_waypoint(side);
    ndf_radio_info(index);
    if (!position_valid) ndf_text(384.0f, 320.0f, "MAP NOT AVAIL", ND_FONT_INTERMEDIATE, AMBER, 1);
    ndf_vertical_display(range_nm, index);
    glLineWidth(1.0f);
    text_use_atlas = saved_atlas; text_atlas_size = saved_size;
}

/* ---- ROSE (NAV / VOR / LS) and PLAN ------------------------------------------------------- */

/* shared/Airplane.tsx symbol with its wing line centred on (x, y), rotated `deg` clockwise. */
static void ndf_aircraft_symbol(float x, float y, float deg)
{
    static const float wings[] = {-41, 0, 41, 0}, body[] = {0, -29.5f, 0, 40.75f}, tail[] = {-11.5f, 31, 12, 31};
    const float *parts[3] = {wings, body, tail};
    float a = deg * (float)M_PI / 180.0f, c = cosf(a), s = sinf(a);
    int k;
    glLineWidth(5.0f);
    set_color(YELLOW);
    glBegin(GL_LINES);
    for (k = 0; k < 3; ++k) {
        int i;
        for (i = 0; i < 2; ++i) {
            float dx = parts[k][2 * i], dy = parts[k][2 * i + 1];
            glVertex2f(x + dx * c - dy * s, NDY(y + dx * s + dy * c));
        }
    }
    glEnd();
}

/* Map canvas centred on (cx, cy) FBW y-down, `radius` px = `range_nm`, clipped to FBW's map box. */
static void ndf_map_canvas(EfisNdCanvas *map, float cx, float cy, float radius, float range_nm, float heading,
                           double lat, double lon, float clip_l, float clip_r, float clip_top_y, float clip_bottom_y)
{
    memset(map, 0, sizeof(*map));
    map->cx = cx; map->cy = NDY(cy);
    map->radius = radius; map->range_nm = range_nm;
    map->heading = heading; map->lat = lat; map->lon = lon;
    map->arc = 0;
    map->clip_l = clip_l; map->clip_r = clip_r; map->clip_b = NDY(clip_bottom_y); map->clip_t = NDY(clip_top_y);
    map->label_r = clip_r - 60.0f; map->sym = 3.0f;
}

static void ndf_map_layers(const EfisNdCanvas *map, int index, int with_route)
{
    glLineWidth(2.5f);
    nd_draw_terrain(map, index);
    if (with_route) { nd_draw_route(map, index); nd_draw_nav_layers(map, index); }
    nd_draw_traffic(map, index);
}

/* ROSE underlay (pages/rose/RoseModeUnderlay.tsx): R = 250 heading ring about (384,384), dashed
 * R = 125 ring, range/2 and range/4 labels, fixed triangles every 45 degrees except at 12 o'clock;
 * lubber line, track diamond and selected-heading bug about the same centre. */
static void ndf_rose_underlay(const Side *side, float heading_mag, float range_nm)
{
    static const float triangle[] = {384, 132, 379, 123, 389, 123};
    static const float diamond[] = {384, 134, 379, 143, 384, 152, 389, 143};
    static const float bug[] = {382, 126, 370, 99, 398, 99, 386, 126};
    int k;
    ndf_dashed_circle(ND_FBW_CX, ND_FBW_ROSE_CY, 125.0f, 15.0f, 10.0f, 2.0f, WHITE);
    ndf_range_label(range_nm / 2.0f, 212.0f, 556.0f, 0);
    ndf_range_label(range_nm / 4.0f, 310.0f, 474.0f, 0);
    for (k = 1; k < 8; ++k) ndf_rotated_triangle(ND_FBW_CX, ND_FBW_ROSE_CY, triangle, 45.0f * k, WHITE);
    if (side->heading_valid)
        ndf_compass_ring(ND_FBW_CX, ND_FBW_ROSE_CY, ND_FBW_ROSE_RADIUS, 262.0f, 256.0f, 272.0f, 180.0f, 30, 22.0f, 22.0f,
                         2.5f, heading_mag);
    else ndf_text(384.0f, 241.0f, "HDG", ND_FONT_LARGE, RED, 1);
    ndf_line(384.0f, 116.0f, 384.0f, 152.0f, 5.0f, YELLOW);                       /* lubber line */
    ndf_aircraft_symbol(ND_FBW_CX, ND_FBW_ROSE_CY, 0.0f);
    if (!side->heading_valid) return;
    {   /* FBW IR track: the heading below 50 kt ground speed (adirs.rs). */
        float rel = 0.0f;
        if (ground_speed_knots >= 50.0f && isfinite(ground_track_degrees) && isfinite(nd_data.true_heading))
            rel = ndf_wrap180(ground_track_degrees - (nd_data.true_heading - heading_mag) - heading_mag);
        ndf_rotated_polyline(ND_FBW_CX, ND_FBW_ROSE_CY, diamond, 4, 1, rel, 3.0f, GREEN);
    }
    if (!pfd_guidance.heading_managed && isfinite(pfd_guidance.selected_heading_deg))
        ndf_rotated_polyline(ND_FBW_CX, ND_FBW_ROSE_CY, bug, 4, 0,
                             ndf_wrap180((float)pfd_guidance.selected_heading_deg - heading_mag), 3.0f, CYAN);
}

static void draw_nd_fbw_rose(const Side *side, int index, int mode)
{
    const int position_valid = side->navigation_valid && nd_data.position_valid;
    const float heading_mag = side->heading_valid ? side->heading_value : 0.0f;
    const float range_nm = isfinite(side->nd_range_nm) && side->nd_range_nm > 0 ? side->nd_range_nm : 80.0f;
    EfisNdCanvas map;
    rectangle(0.0f, 0.0f, ND_FBW_W, ND_FBW_H, BLACK);
    if (!side->display_valid) return;
    /* RoseNavPage.tsx: map pixel radius 250 = range / 2; clip box M45,155 .. 723,625. */
    ndf_map_canvas(&map, ND_FBW_CX, ND_FBW_ROSE_CY, ND_FBW_ROSE_RADIUS, range_nm / 2.0f,
                   side->heading_valid && isfinite(nd_data.true_heading) ? nd_data.true_heading : 0.0f,
                   nd_data.lat, nd_data.lon, 45.0f, 723.0f, 155.0f, 625.0f);
    if (position_valid) ndf_map_layers(&map, index, mode == 2);
    ndf_rose_underlay(side, heading_mag, range_nm);
    ndf_header_left(side);
    ndf_header_to_waypoint(side);
    ndf_radio_info(index);
    if (!position_valid) ndf_text(384.0f, 320.0f, "MAP NOT AVAIL", ND_FONT_INTERMEDIATE, AMBER, 1);
    if (mode == 2) ndf_vertical_display(range_nm, index);
}

/* PLAN (pages/plan/PlanModeUnderlay.tsx): north up, centred on the F-PLN; solid R = 250 ring = range,
 * dashed R = 125 = range/2, N/E/S/W with triangles; the aircraft drawn at its position and heading. */
static void draw_nd_fbw_plan(const Side *side, int index)
{
    static const float tri_n[] = {384, 141.5f, 390, 151, 378, 151};
    const int position_valid = side->navigation_valid && nd_data.position_valid;
    const float range_nm = isfinite(side->nd_range_nm) && side->nd_range_nm > 0 ? side->nd_range_nm : 80.0f;
    double centre_lat = nd_data.lat, centre_lon = nd_data.lon;
    EfisNdCanvas map;
    float px, py;
    int k;
    static const char *const letters[4] = {"N", "E", "S", "W"};
    rectangle(0.0f, 0.0f, ND_FBW_W, ND_FBW_H, BLACK);
    if (!side->display_valid) return;
    nd_fms_plan_centre(&centre_lat, &centre_lon);
    ndf_map_canvas(&map, ND_FBW_CX, ND_FBW_ROSE_CY, ND_FBW_ROSE_RADIUS, range_nm, 0.0f,
                   centre_lat, centre_lon, 45.0f, 723.0f, 155.0f, 625.0f);
    if (position_valid) ndf_map_layers(&map, index, 1);
    glLineWidth(3.0f);
    set_color(WHITE);
    glBegin(GL_LINE_STRIP);
    for (k = 0; k <= 360; k += 2) {
        float x, y;
        ndf_polar_at(ND_FBW_CX, ND_FBW_ROSE_CY, (float)k, ND_FBW_ROSE_RADIUS, &x, &y);
        glVertex2f(x, NDY(y));
    }
    glEnd();
    ndf_dashed_circle(ND_FBW_CX, ND_FBW_ROSE_CY, 125.0f, 14.0f, 13.0f, 3.0f, WHITE);
    ndf_range_label(range_nm / 2.0f, 310.0f, 474.0f, 0);
    ndf_range_label(range_nm, 212.0f, 556.0f, 0);
    for (k = 0; k < 4; ++k) {
        float lx, ly;
        ndf_polar_at(ND_FBW_CX, ND_FBW_ROSE_CY, 90.0f * k, 214.0f, &lx, &ly);
        ndf_text(lx, ly + 9.0f, letters[k], ND_FONT_SMALL, WHITE, 1);   /* alignment-baseline central */
        ndf_rotated_triangle(ND_FBW_CX, ND_FBW_ROSE_CY, tri_n, 90.0f * k, WHITE);
    }
    /* Own aircraft at its position, rotated by true heading (north-up map). */
    if (position_valid && nd_map_point(&map, nd_data.lat, nd_data.lon, &px, &py) && nd_map_visible(&map, px, py, 0))
        ndf_aircraft_symbol(map.cx + px, ND_FBW_H - (map.cy + py), nd_data.true_heading);
    ndf_header_left(side);
    ndf_header_to_waypoint(side);
    ndf_radio_info(index);
    if (!position_valid) ndf_text(384.0f, 320.0f, "MAP NOT AVAIL", ND_FONT_INTERMEDIATE, AMBER, 1);
}

/* Entry point for the ND device: modes 0 ROSE LS, 1 ROSE VOR, 2 ROSE NAV, 3 ARC, 4 PLAN. Map-layer
 * labels use the EIS atlas at FBW's waypoint size; the caller's text settings are restored. */
static void draw_nd_fbw(const Side *side, int index)
{
    const int saved_atlas = text_use_atlas;
    const float saved_size = text_atlas_size;
    const int mode = side->nd_mode >= 0 && side->nd_mode <= 4 ? side->nd_mode : 2;
    if (mode == 3) { draw_nd_fbw_arc(side, index); return; }
    text_use_atlas = 1; text_atlas_size = 22.0f;
    if (mode == 4) draw_nd_fbw_plan(side, index);
    else draw_nd_fbw_rose(side, index, mode);
    glLineWidth(1.0f);
    text_use_atlas = saved_atlas; text_atlas_size = saved_size;
}
