/* SPDX-License-Identifier: GPL-3.0-or-later
 * Native rendering translated from pinned FlyByWire A380 PFD components.
 * Included by the X-Plane adapter after its shared display/guidance state.
 */
#include "a380x_elec_power.h"
#ifndef SIDE_ND_VALID
#define SIDE_ND_VALID(side) ((side)->display_valid)
#endif
static const Color MAGENTA = {1.0f, 0.22f, 0.95f};
/* The same pinned Airbus EIS font used by the EWD is shared by the PFD. */
static void ewd_font_draw(float x, float y, const char *text, float size,
                          Color color, float tracking, int anchor);
static void ewd_font_clip_y(float y0, float y1);
#include "a380x_terrain_render.inc"   /* TERRAIN on ND image, TERR box, pointer elevation readout */

typedef struct { int mach_visible, excessive_attitude; } PfdDisplayState;
static PfdDisplayState pfd_display_state[2];

static int pfd_excessive_attitude(const Side *side)
{
    PfdDisplayState *state=&pfd_display_state[side==&refs.side[1]];
    if (!side->attitude_valid) return 0;
    if (side->pitch_value>25.0f || side->pitch_value< -13.0f || fabsf(side->roll_value)>45.0f)
        state->excessive_attitude=1;
    else if (side->pitch_value> -10.0f && side->pitch_value<22.0f && fabsf(side->roll_value)<40.0f)
        state->excessive_attitude=0;
    return state->excessive_attitude;
}

static float pfd_clamp(float v, float lo, float hi) { return fminf(hi, fmaxf(lo, v)); }

static void set_color(Color c) { glColor3f(c.r, c.g, c.b); }

static void rectangle(float left, float bottom, float right, float top, Color color)
{
    set_color(color);
    glBegin(GL_QUADS);
    glVertex2f(left, bottom); glVertex2f(right, bottom);
    glVertex2f(right, top); glVertex2f(left, top);
    glEnd();
}

static void line(float x1, float y1, float x2, float y2, Color color)
{
    set_color(color);
    glBegin(GL_LINES);
    glVertex2f(x1, y1); glVertex2f(x2, y2);
    glEnd();
}

/* Nonzero while drawing into a scaled cockpit device (a380x_nd_device.inc): XPLMDrawString
 * ignores the device scale, so text uses the FBW EIS atlas at the X-Plane UI font's height. */
static int text_use_atlas;
static float text_atlas_size = 9.0f;

static void text_at(float x, float y, const char *value, Color color)
{
    float rgb[3] = {color.r, color.g, color.b};
    if (text_use_atlas) {
        ewd_font_draw(x, y, value, text_atlas_size, color, 0, 0);
        return;
    }
    XPLMDrawString(rgb, (int)lroundf(x), (int)lroundf(y), value, NULL, xplmFont_Proportional);
    /* XPLMDrawString may change OpenGL state.  Restore the documented panel
     * drawing state before a later primitive is emitted in this callback. */
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
}

typedef struct { float x, y; } Point;

static int clip_half_plane(const Point *input, int count, Point *output,
                           float nx, float ny, float offset, int positive)
{
    int i, out_count = 0;
    Point previous;
    float previous_distance;
    int previous_inside;
    if (count <= 0) return 0;
    previous = input[count - 1];
    previous_distance = nx * previous.x + ny * previous.y + offset;
    previous_inside = positive ? previous_distance >= 0.0f : previous_distance <= 0.0f;
    for (i = 0; i < count; ++i) {
        Point current = input[i];
        float distance = nx * current.x + ny * current.y + offset;
        int inside = positive ? distance >= 0.0f : distance <= 0.0f;
        if (inside != previous_inside) {
            float t = previous_distance / (previous_distance - distance);
            output[out_count++] = (Point){previous.x + t * (current.x - previous.x),
                                         previous.y + t * (current.y - previous.y)};
        }
        if (inside) output[out_count++] = current;
        previous = current;
        previous_distance = distance;
        previous_inside = inside;
    }
    return out_count;
}

static void filled_polygon(const Point *points, int count, float ox, float oy, Color color)
{
    int i;
    if (count < 3) return;
    set_color(color);
    glBegin(GL_TRIANGLE_FAN);
    for (i = 0; i < count; ++i) glVertex2f(ox + points[i].x, oy + points[i].y);
    glEnd();
}

#define FBW_PFD_WIDTH 158.75f
#define FBW_PFD_HEIGHT 211.6f
#define PFD_PIXEL_WIDTH 320.0f
#define PFD_PIXEL_HEIGHT 416.0f
/* Line widths are authored for the 416-texel-high canvas. A cockpit device that scales the PFD
 * up (a380x_pfd_device.inc) raises this so lines keep their thickness relative to the display;
 * glLineWidth itself is not affected by the modelview scale. */
static float pfd_line_scale = 1.0f;

static void pfd_line_width(float width) { glLineWidth(width * pfd_line_scale); }

typedef struct { float left, bottom, scale; } PfdCanvas;

static PfdCanvas pfd_canvas(float x, float y)
{
    PfdCanvas p;
    p.scale = PFD_PIXEL_HEIGHT / FBW_PFD_HEIGHT;
    p.left = x + (PFD_PIXEL_WIDTH - FBW_PFD_WIDTH * p.scale) * 0.5f;
    p.bottom = y;
    return p;
}

static float pfd_x(const PfdCanvas *p, float x) { return p->left + x * p->scale; }
static float pfd_y(const PfdCanvas *p, float y) { return p->bottom + (FBW_PFD_HEIGHT - y) * p->scale; }

static void pfd_rect(const PfdCanvas *p, float x1, float y1, float x2, float y2, Color color)
{
    rectangle(pfd_x(p, x1), pfd_y(p, y2), pfd_x(p, x2), pfd_y(p, y1), color);
}

static void pfd_line(const PfdCanvas *p, float x1, float y1, float x2, float y2, Color color)
{
    line(pfd_x(p, x1), pfd_y(p, y1), pfd_x(p, x2), pfd_y(p, y2), color);
}

static void pfd_polygon(const PfdCanvas *p, const Point *source, int count, Color color)
{
    Point mapped[24];
    int i;
    if (count > ARRAY_LEN(mapped)) return;
    for (i = 0; i < count; ++i) {
        mapped[i].x = pfd_x(p, source[i].x);
        mapped[i].y = pfd_y(p, source[i].y);
    }
    filled_polygon(mapped, count, 0.0f, 0.0f, color);
}

static void pfd_polygon_outline(const PfdCanvas *p, const Point *source, int count, Color color)
{
    int i;
    for (i = 0; i < count; ++i) {
        int next = (i + 1) % count;
        pfd_line(p, source[i].x, source[i].y, source[next].x, source[next].y, color);
    }
}

static void pfd_circle(const PfdCanvas *p, float x, float y, float radius, Color color)
{
    int i;
    float px = x + radius, py = y;
    for (i = 1; i <= 24; ++i) {
        float a = (float)i * (float)(2.0 * M_PI / 24.0);
        float nx = x + cosf(a) * radius, ny = y + sinf(a) * radius;
        pfd_line(p,px,py,nx,ny,color);
        px = nx; py = ny;
    }
}

static void pfd_box(const PfdCanvas *p, float x1, float y1, float x2, float y2, Color color)
{
    pfd_line(p,x1,y1,x2,y1,color); pfd_line(p,x2,y1,x2,y2,color);
    pfd_line(p,x2,y2,x1,y2,color); pfd_line(p,x1,y2,x1,y1,color);
}

static void pfd_text(const PfdCanvas *p, float x, float y, const char *value, Color color)
{
    ewd_font_draw(pfd_x(p,x),pfd_y(p,y),value,6.0f*p->scale,color,0,0);
}

static void pfd_text_center(const PfdCanvas *p, float x, float y, const char *value, Color color)
{
    ewd_font_draw(pfd_x(p,x),pfd_y(p,y),value,6.0f*p->scale,color,0,1);
}

static void pfd_text_sized(const PfdCanvas *p,float x,float y,const char *value,Color color,float size,int anchor)
{
    ewd_font_draw(pfd_x(p,x),pfd_y(p,y),value,size*p->scale,color,0,anchor);
}

/* Exact nonlinear pitch mapping from FlyByWire A380 PFDUtils.tsx. */
static float fbw_horizon_offset(float pitch)
{
    if (pitch > -5.0f && pitch <= 20.0f) return pitch * 1.8f;
    if (pitch > 20.0f && pitch <= 30.0f) return -0.04f * pitch * pitch + 3.4f * pitch - 16.0f;
    if (pitch > 30.0f) return 20.0f + pitch;
    if (pitch < -5.0f && pitch >= -15.0f) return 0.04f * pitch * pitch + 2.2f * pitch + 1.0f;
    return pitch - 8.0f;
}

static void rotate_point(float x, float y, float angle, float *out_x, float *out_y)
{
    float cosine = cosf(angle), sine = sinf(angle);
    *out_x = cosine * x - sine * y;
    *out_y = sine * x + cosine * y;
}

/* Clip rotated ladder lines to the FBW round window with flat sides on the CPU.
 * Scissor/stencil
 * state belongs to X-Plane's panel renderer, so do not change it here. */
static void attitude_line(const PfdCanvas *p, float x1, float y1, float x2, float y2, Color color)
{
    const float radius = 42.0815f * p->scale, half_width = 36.768f * p->scale;
    float dx = (x2 - x1) / radius, dy = (y2 - y1) / radius;
    float x = x1 / radius, y = y1 / radius;
    float a = dx * dx + dy * dy;
    float b = 2.0f * (x * dx + y * dy), c = x * x + y * y - 1.0f;
    float discriminant = b * b - 4.0f * a * c;
    float lo, hi;
    if (a < 1e-7f || discriminant < 0.0f) return;
    lo = fmaxf(0.0f, (-b - sqrtf(discriminant)) / (2.0f * a));
    hi = fminf(1.0f, (-b + sqrtf(discriminant)) / (2.0f * a));
    if (fabsf(x2-x1)>1e-7f) {
        float side1=(-half_width-x1)/(x2-x1), side2=(half_width-x1)/(x2-x1);
        lo=fmaxf(lo,fminf(side1,side2));
        hi=fminf(hi,fmaxf(side1,side2));
    } else if (fabsf(x1)>half_width) return;
    if (lo > hi) return;
    line(pfd_x(p,68.906f) + x1 + (x2 - x1) * lo,
         pfd_y(p,80.823f) + y1 + (y2 - y1) * lo,
         pfd_x(p,68.906f) + x1 + (x2 - x1) * hi,
         pfd_y(p,80.823f) + y1 + (y2 - y1) * hi,color);
}

static void draw_pitch_ladder(const PfdCanvas *p, float pitch, float roll)
{
    int index;
    const float cx = pfd_x(p, 68.906f), cy = pfd_y(p, 80.823f);
    /* X-Plane roll is positive right; the horizon rotates oppositely.
     * SVG used -roll in Y-down coordinates, equivalent to +roll in GL Y-up. */
    const float angle = roll * (float)M_PI / 180.0f;
    const float pitch_offset = fbw_horizon_offset(pitch);
    for (index = -12; index <= 12; ++index) {
        float mark = (float)index * 2.5f;
        float local_y, half_source, half, x1, y1, x2, y2;
        char label[8];
        if (index == 0) continue;
        local_y = (fbw_horizon_offset(mark) - pitch_offset) * p->scale;
        if (fabsf(local_y) > 48.0f * p->scale) continue;
        if (index % 4 == 0) half_source = fabsf(mark) >= 30.0f ? 13.0f : 9.5f;
        else if (index % 2 == 0) half_source = 4.5f;
        else half_source = 2.5f;
        half = half_source * p->scale;
        rotate_point(-half, local_y, angle, &x1, &y1);
        rotate_point(half, local_y, angle, &x2, &y2);
        attitude_line(p, x1, y1, x2, y2, WHITE);
        if (index % 4 == 0 && fabsf(roll) < 35.0f && fabsf(local_y) < 33.0f * p->scale) {
            snprintf(label, sizeof(label), "%d", (int)fabsf(mark));
            text_at(cx + x1 - 17.0f, cy + y1 - 4.0f, label, WHITE);
            text_at(cx + x2 + 4.0f, cy + y2 - 4.0f, label, WHITE);
        }
    }
}

static void mask_attitude_window(const PfdCanvas *p)
{
    int i;
    const int strips = 168;
    const float cx = 68.906f, cy = 80.8205f, rx = 36.768f, ry = 42.0815f;
    for (i = 0; i < strips; ++i) {
        float y1 = cy - ry + (2.0f * ry * i) / strips;
        float y2 = cy - ry + (2.0f * ry * (i + 1)) / strips;
        float ym = (y1 + y2) * 0.5f;
        float dy = ym-cy;
        /* PFD.tsx mask: 42.08-unit circular ends, clipped at x=32.138/105.674. */
        float half = fminf(rx,sqrtf(fmaxf(0.0f,ry*ry-dy*dy)));
        pfd_rect(p, cx - rx - 0.5f, y1, cx - half, y2, BLACK);
        pfd_rect(p, cx + half, y1, cx + rx + 0.5f, y2, BLACK);
    }
}

static void draw_bank_scale(const PfdCanvas *p, float roll)
{
    static const int marks[] = {-30, -20, -10, 10, 20, 30};
    int i;
    float previous_x = 0.0f, previous_y = 0.0f;
    const float cx = 68.906f, cy = 80.823f, radius = 42.158f;
    for (i = -32; i <= 32; ++i) {
        float angle = (float)i * (float)M_PI / 180.0f;
        float x = cx + sinf(angle) * radius;
        float y = cy - cosf(angle) * radius;
        if (i > -32) pfd_line(p, previous_x, previous_y, x, y, WHITE);
        previous_x = x;
        previous_y = y;
    }
    for (i = 0; i < ARRAY_LEN(marks); ++i) {
        float angle = marks[i] * (float)M_PI / 180.0f;
        float inner = radius - (abs(marks[i]) == 30 ? 3.7f : 2.7f);
        pfd_line(p, cx + sinf(angle) * inner, cy - cosf(angle) * inner,
                 cx + sinf(angle) * radius, cy - cosf(angle) * radius, WHITE);
    }
    pfd_line(p,98.645f,51.067f,101.4942f,48.2161f,WHITE);
    pfd_line(p,39.168f,51.067f,36.3188f,48.2161f,WHITE);
    {
        Point fixed[3] = {{68.906f, 38.650f}, {66.3876f, 34.950f}, {71.4244f, 34.950f}};
        Point moving[3];
        const Point base[3] = {{-2.832f, 36.840f}, {0.028f, 41.073f}, {2.888f, 36.840f}};
        float angle = roll * (float)M_PI / 180.0f;
        for (i = 0; i < 3; ++i) {
            float sx, sy;
            rotate_point(base[i].x, base[i].y, angle, &sx, &sy);
            moving[i].x = cx + sx;
            moving[i].y = cy - sy;
        }
        pfd_polygon_outline(p, fixed, 3, YELLOW);
        pfd_polygon_outline(p, moving, 3, YELLOW);
    }
}

static void draw_fixed_aircraft(const PfdCanvas *p)
{
    static const Point left[] = {
        {34.153f,79.563f},{49.263f,79.563f},{49.263f,86.1146f},
        {46.7446f,86.1146f},{46.7446f,82.0829f},{34.153f,82.0829f}
    };
    static const Point right[] = {
        {88.55f,86.114f},{91.0684f,86.114f},{91.0684f,82.0823f},
        {103.6604f,82.0823f},{103.6604f,79.5625f},{88.55f,79.5625f}
    };
    static const Point center[] = {
        {67.647f,82.083f},{67.647f,79.5632f},{70.1654f,79.5632f},{70.1654f,82.083f}
    };
    pfd_polygon(p, left, ARRAY_LEN(left), BLACK);
    pfd_polygon(p, right, ARRAY_LEN(right), BLACK);
    pfd_polygon(p, center, ARRAY_LEN(center), BLACK);
    pfd_line_width(2.0f);
    pfd_polygon_outline(p, left, ARRAY_LEN(left), YELLOW);
    pfd_polygon_outline(p, right, ARRAY_LEN(right), YELLOW);
    pfd_polygon_outline(p, center, ARRAY_LEN(center), YELLOW);
    pfd_line_width(1.0f);
}

static int rising_ground_polygon(const Side *side, float scale, Point *clipped, Point *outline)
{
    Point scratch[12];
    const float half_width=36.768f*scale, half_height=42.0815f*scale;
    float target_pitch=side->radio_altitude_valid && isfinite(side->radio_altitude_value)
        ? 0.1f*side->radio_altitude_value : 200.0f;
    float offset=pfd_clamp(fbw_horizon_offset(side->pitch_value+target_pitch)-31.563f,-63.093f,0.0f);
    float angle=side->roll_value*(float)M_PI/180.0f;
    Point source[]={{23.87f,112.383f+offset},{113.95f,112.383f+offset},
                    {113.95f,157.74f+offset},{23.87f,157.74f+offset}};
    int i,count=4;
    /* RisingGround is in the source roll group, outside its pitch group.
     * Rotate the finite rectangle once, then clip before emitting vertices.
     * A half-plane fill alone would incorrectly extend its bottom boundary
     * during strongly nose-down attitudes. */
    for(i=0;i<4;++i) {
        rotate_point((source[i].x-68.906f)*scale,(80.823f-source[i].y)*scale,
                     angle,&outline[i].x,&outline[i].y);
        clipped[i]=outline[i];
    }
    count=clip_half_plane(clipped,count,scratch,1,0,half_width,1);
    count=clip_half_plane(scratch,count,clipped,1,0,-half_width,0);
    count=clip_half_plane(clipped,count,scratch,0,1,half_height,1);
    return clip_half_plane(scratch,count,clipped,0,1,-half_height,0);
}

static void draw_rising_ground(const PfdCanvas *p,const Side *side)
{
    Point clipped[12],outline[4];
    int i,count;
    if(!side->attitude_valid || !isfinite(side->pitch_value) || !isfinite(side->roll_value))return;
    count=rising_ground_polygon(side,p->scale,clipped,outline);
    filled_polygon(clipped,count,pfd_x(p,68.906f),pfd_y(p,80.823f),EARTH);
    /* The subsequent attitude mask hides fill outside its rounded aperture;
     * borders use the same CPU clipping as the pitch ladder. */
    pfd_line_width(3.0f);
    for(i=0;i<4;++i) attitude_line(p,outline[i].x,outline[i].y,
                                  outline[(i+1)%4].x,outline[(i+1)%4].y,BLACK);
    pfd_line_width(1.0f);
    for(i=0;i<4;++i) attitude_line(p,outline[i].x,outline[i].y,
                                  outline[(i+1)%4].x,outline[(i+1)%4].y,WHITE);
}

static void draw_horizon(const PfdCanvas *p, const Side *side)
{
    Point box[4], sky[8], ground[8];
    float width, height, cx, cy, angle, nx, ny, dy, offset;
    int sky_count, ground_count;
    if (!side->attitude_valid || !isfinite(side->pitch_value) || !isfinite(side->roll_value)) {
        pfd_text_center(p, 68.906f, 83.137f, "ATT", RED);
        return;
    }
    width = (105.674f - 32.138f) * p->scale;
    height = (122.902f - 38.739f) * p->scale;
    cx = pfd_x(p, 68.906f);
    cy = pfd_y(p, 80.823f);
    box[0] = (Point){-width * 0.5f, -height * 0.5f};
    box[1] = (Point){ width * 0.5f, -height * 0.5f};
    box[2] = (Point){ width * 0.5f,  height * 0.5f};
    box[3] = (Point){-width * 0.5f,  height * 0.5f};
    angle = side->roll_value * (float)M_PI / 180.0f;
    nx = -sinf(angle);
    ny = cosf(angle);
    dy = -fbw_horizon_offset(side->pitch_value) * p->scale;
    /* Pitch displacement is along the rotated horizon normal, not the screen Y axis. */
    offset = -dy;
    sky_count = clip_half_plane(box, 4, sky, nx, ny, offset, 1);
    ground_count = clip_half_plane(box, 4, ground, nx, ny, offset, 0);
    filled_polygon(sky, sky_count, cx, cy, SKY);
    filled_polygon(ground, ground_count, cx, cy, EARTH);
    draw_pitch_ladder(p, side->pitch_value, side->roll_value);
    draw_rising_ground(p,side);
    mask_attitude_window(p);
    draw_bank_scale(p, side->roll_value);
    draw_fixed_aircraft(p);
}

static void draw_speed_tape(const PfdCanvas *p, const Side *side)
{
    static const Point pointer[] = {
        {13.994f,80.460f},{13.994f,81.1857f},{20.5418f,81.1857f},
        {23.6646f,82.3348f},{23.6646f,79.3110f},{20.5418f,80.460f}
    };
    int speed;
    float indicated = fmaxf(30.0f, side->ias_value);
    char value[16];
    pfd_rect(p, 1.9058f, 38.086f, 19.031f, 123.56f, TAPE_BACKGROUND);
    pfd_line(p, 1.9058f, 38.086f, 23.765f, 38.086f, side->air_valid ? WHITE : RED);
    pfd_line(p, 1.9058f, 123.56f, 23.765f, 123.56f, side->air_valid ? WHITE : RED);
    if (!side->air_valid || !isfinite(side->ias_value)) {
        pfd_text_center(p, 12.0f, 83.386f, "SPD", RED);
        return;
    }
    speed = (int)floorf((indicated - 48.0f) / 10.0f) * 10;
    for (; speed <= indicated + 48.0f; speed += 10) {
        float y;
        if (speed < 30 || speed > 660) continue;
        y = 80.818f - ((float)speed - indicated);
        if (y < 38.4f || y > 123.2f) continue;
        pfd_line(p, 16.2104f, y, 19.031f, y, WHITE);
        if (speed % 20 == 0) {
            snprintf(value, sizeof(value), "%03d", speed);
            pfd_text_center(p, 9.535f, y + 2.12f, value, WHITE);
        }
    }
    if (side->ias_trend_visible && isfinite(side->ias_trend_value)) {
        float offset = pfd_clamp(-side->ias_trend_value, -40.0f, 40.0f);
        float head = offset < 0.0f ? 2.4607f : -2.4607f;
        pfd_line(p,15.455f,80.823f,15.455f,80.823f+offset,YELLOW);
        pfd_line(p,15.455f,80.823f+offset,14.2019f,80.823f+offset+head,YELLOW);
        pfd_line(p,15.455f,80.823f+offset,16.7081f,80.823f+offset+head,YELLOW);
    }
    pfd_polygon(p, pointer, ARRAY_LEN(pointer), YELLOW);
    pfd_polygon_outline(p, pointer, ARRAY_LEN(pointer), BLACK);
    pfd_rect(p, 0.0926f, 80.460f, 2.1073f, 81.1857f, YELLOW);
    {
        int permille = (int)lroundf(side->mach_value * 1000.0f);
        PfdDisplayState *state=&pfd_display_state[side==&refs.side[1]];
        if (permille>500) state->mach_visible=1;
        else if (permille<450) state->mach_visible=0;
        if (state->mach_visible) {
            snprintf(value, sizeof(value), ".%03d", permille);
            pfd_text(p, 5.5668f, 137.030f, value, GREEN);
        }
    }
}

/* FBW DigitalAltitudeReadout.tsx drums. Big digits (ten-thousands, thousands, hundreds) roll by
 * 7 units per digit inside the 117.754..132.254 x 76.337..85.308 window; the hundreds roll only
 * while the tens pass 80 ft, and higher digits follow when the one below shows 9. The tens drum
 * rolls continuously in 20 ft steps (4.7 units apart) inside 130.85..139.715 x 73.666..87.979. */
static void pfd_drum_digit_text(int type, int value, int show_zero, char *out, size_t n)
{
    out[0] = '\0';
    if (type == 0) {                       /* tens: TensDigits */
        int v = value < 0 ? value + 100 : value >= 100 ? value - 100 : value;
        snprintf(out, n, "%02d", v);
        return;
    }
    if (type == 1) value = value < 0 ? value + 10 : value >= 10 ? value - 10 : value;  /* hundreds */
    else value = ((value % 10) + 10) % 10;
    if (!show_zero && value == 0) return;
    snprintf(out, n, "%d", value);
}

static void pfd_big_drum(const PfdCanvas *p, float x, int type, float value, float position, int show_zero, Color color)
{
    int i, highest_value = (int)roundf(value + 1.0f), highest_position = (int)roundf(position + 1.0f);
    if ((float)highest_position > position + 1.0f) --highest_position;
    if ((float)highest_value > value + 1.0f) --highest_value;
    for (i = 0; i < 3; ++i) {
        char text[16];
        float baseline = 76.3374f + 6.8f + (position - (float)(highest_position - i)) * 7.0f;
        pfd_drum_digit_text(type, highest_value - i, show_zero, text, sizeof(text));
        if (text[0]) pfd_text_sized(p, x, baseline, text, color, 6.0f, 1);
    }
}

static void pfd_altitude_drums(const PfdCanvas *p, float altitude)
{
    const float abs_alt = fabsf(fmaxf(fminf(altitude, 50000.0f), -1500.0f));
    const float tens = fmodf(abs_alt, 100.0f);
    const int hundreds = (int)floorf(fmodf(abs_alt / 100.0f, 10.0f));
    const int thousands = (int)floorf(fmodf(abs_alt / 1000.0f, 10.0f));
    const int ten_thousands = (int)floorf(fmodf(abs_alt / 10000.0f, 10.0f));
    const float hundreds_position = tens > 80.0f ? tens / 20.0f - 4.0f : 0.0f;
    const float thousands_position = hundreds >= 9 ? hundreds_position : 0.0f;
    const float ten_thousands_position = thousands >= 9 ? thousands_position : 0.0f;
    int i, highest;
    /* Big digits, clipped to their window (FBW svg x=117.754 y=76.3374 w=14.5 h=8.9706). */
    ewd_font_clip_y(pfd_y(p, 85.308f), pfd_y(p, 76.3374f));
    pfd_big_drum(p, 117.754f + 6.05f, 3, (float)ten_thousands, ten_thousands_position, 0, GREEN);
    pfd_big_drum(p, 117.754f + 9.7f, 2, (float)thousands, thousands_position, ten_thousands != 0, GREEN);
    pfd_big_drum(p, 117.754f + 13.2f, 1, (float)hundreds, hundreds_position, 1, GREEN);
    /* Tens drum (FBW svg x=130.85 y=73.6664 w=8.8647 h=14.313). */
    ewd_font_clip_y(pfd_y(p, 87.979f), pfd_y(p, 73.6664f));
    highest = (int)roundf((tens + 40.0f) / 20.0f) * 20;
    if ((float)highest > tens + 40.0f) highest -= 20;
    for (i = 0; i < 4; ++i) {
        char text[16];
        int value = highest - 20 * i;
        float baseline = 73.6664f + 8.9133f + (tens - (float)value) * 4.7f / 20.0f;
        pfd_drum_digit_text(0, value, 1, text, sizeof(text));
        pfd_text_sized(p, 130.85f + 5.5f, baseline, text, GREEN, 4.5f, 1);
    }
    ewd_font_clip_y(0.0f, 0.0f);
    if (altitude < 0.0f) {                 /* AltNegativeText */
        pfd_text_sized(p, 121.51714f, 77.956947f, "N", WHITE, 6.5f, 2);
        pfd_text_sized(p, 121.7f, 83.251389f, "E", WHITE, 6.5f, 2);
        pfd_text_sized(p, 121.63675f, 88.486031f, "G", WHITE, 6.5f, 2);
    }
}

static void draw_altitude_tape(const PfdCanvas *p, const Side *side)
{
    static const Point readout[] = {
        {130.85f,85.308f},{117.72f,85.308f},{117.72f,76.3374f},
        {130.85f,76.3374f},{130.85f,73.6664f},{139.7147f,73.6664f},
        {139.7147f,87.9794f},{130.85f,87.9794f}
    };
    int altitude;
    char value[16];
    pfd_rect(p, 117.754f, 38.087f, 130.85f, 123.56f, TAPE_BACKGROUND);
    pfd_line(p, 117.754f, 38.087f, 137.58f, 38.087f, side->altitude_valid ? WHITE : RED);
    pfd_line(p, 117.754f, 123.56f, 137.58f, 123.56f, side->altitude_valid ? WHITE : RED);
    if (!side->altitude_valid || !isfinite(side->altitude_value)) {
        pfd_rect(p, 117.72f, 76.3374f, 131.35f, 85.308f, BLACK);
        pfd_text_center(p, 124.5f, 83.433f, "ALT", RED);
        return;
    }
    altitude = (int)floorf((side->altitude_value - 650.0f) / 100.0f) * 100;
    for (; altitude <= side->altitude_value + 650.0f; altitude += 100) {
        float y = 80.819f - ((float)altitude - side->altitude_value) * 0.075f;
        if (altitude < -1500 || altitude > 50000 || y < 38.4f || y > 123.2f) continue;
        pfd_line(p, 128.8353f, y, 130.85f, y, WHITE);
        if (altitude % 500 == 0 && y - 2.4f > 38.09f && y + 2.2f < 123.56f) {
            snprintf(value, sizeof(value), "%03d", abs(altitude / 100));
            pfd_text_center(p, 123.288f, y + 1.82f, value, WHITE);
            pfd_line(p, 115.79f, y + 1.0783f, 117.1216f, y, WHITE);
            pfd_line(p, 117.1216f, y, 115.79f, y - 1.0783f, WHITE);
        }
    }
    pfd_polygon(p, readout, ARRAY_LEN(readout), BLACK);
    pfd_line_width(1.5f);
    pfd_line(p, 117.75f,76.337f,130.846f,76.337f,YELLOW);
    pfd_line(p,130.846f,76.337f,130.846f,73.666f,YELLOW);
    pfd_line(p,130.846f,73.666f,139.711f,73.666f,YELLOW);
    pfd_line(p,139.711f,73.666f,139.711f,87.979f,YELLOW);
    pfd_line(p,139.711f,87.979f,130.846f,87.979f,YELLOW);
    pfd_line(p,130.846f,87.979f,130.846f,85.308f,YELLOW);
    pfd_line(p,130.846f,85.308f,117.75f,85.308f,YELLOW);
    pfd_line_width(1.0f);
    pfd_altitude_drums(p, side->altitude_value);
}

static float vertical_speed_offset(float fpm)
{
    float sign = fpm < 0.0f ? -1.0f : 1.0f;
    float value = fabsf(fpm);
    if (value < 1000.0f) return (fpm / 1000.0f) * -27.22f;
    if (value < 2000.0f) return ((fpm - sign * 1000.0f) / 1000.0f) * -10.1f - sign * 27.22f;
    if (value < 6000.0f) return ((fpm - sign * 2000.0f) / 4000.0f) * -10.1f - sign * 37.32f;
    return sign * -47.37f;
}

static void draw_vertical_speed(const PfdCanvas *p, const Side *side)
{
    static const Point background[] = {
        {151.84f,131.72f},{155.9701f,116.097f},{155.9701f,45.541f},
        {151.84f,29.918f},{146.2996f,29.918f},{146.2996f,131.72f}
    };
    static const float tick_y[] = {32.844f,42.798f,52.878f,67.216f,94.430f,108.772f,118.852f,128.799f};
    int i;
    char value[16];
    pfd_polygon(p, background, ARRAY_LEN(background), TAPE_BACKGROUND);
    if (!side->vertical_speed_valid || !isfinite(side->vertical_speed_value)) {
        pfd_text_center(p, 151.2f, 77.50f, "V", RED);
        pfd_text_center(p, 151.2f, 83.21f, "/", RED);
        pfd_text_center(p, 151.2f, 88.87f, "S", RED);
        return;
    }
    for (i = 0; i < ARRAY_LEN(tick_y); ++i)
        pfd_line(p, i == 3 || i == 4 ? 150.12f : 149.92f, tick_y[i], 151.84f, tick_y[i], WHITE);
    pfd_text_sized(p,148.47f,55.316f,"1",WHITE,4.5f,1);
    pfd_text_sized(p,148.26f,45.356f,"2",WHITE,4.5f,1);
    pfd_text_sized(p,148.21f,35.195f,"6",WHITE,4.5f,1);
    pfd_text_sized(p,148.47f,109.728f,"1",WHITE,4.5f,1);
    pfd_text_sized(p,148.24f,119.880f,"2",WHITE,4.5f,1);
    pfd_text_sized(p,148.27f,129.906f,"6",WHITE,4.5f,1);
    pfd_rect(p,145.79f,80.067f,151.8376f,81.5789f,YELLOW);
    {
        float y_offset = vertical_speed_offset(side->vertical_speed_value);
        float y1 = 80.822f + (5.0f / 12.0f) * y_offset;
        float y2 = 80.822f + y_offset;
        int low_altitude_sink = side->radio_altitude_valid &&
            ((side->radio_altitude_value <= 2500.0f && side->radio_altitude_value >= 1000.0f &&
              side->vertical_speed_value <= -2000.0f) ||
             (side->radio_altitude_value <= 1000.0f && side->vertical_speed_value <= -1200.0f));
        Color needle = fabsf(side->vertical_speed_value) >= 6000.0f || low_altitude_sink ? AMBER : GREEN;
        pfd_line_width(3.0f);
        pfd_line(p,157.74f,y1,150.74f,y2,BLACK);
        pfd_line_width(2.0f);
        pfd_line(p,157.74f,y1,150.74f,y2,needle);
        pfd_line_width(1.0f);
        if (fabsf(side->vertical_speed_value) >= 200.0f) {
            float shift=y_offset-(side->vertical_speed_value>0.0f ? 2.4f : -2.4f);
            snprintf(value, sizeof(value), "%02d", (int)lroundf(fabsf(side->vertical_speed_value) / 100.0f));
            pfd_rect(p,151.3486f,78.6121f+shift,158.4f,83.011f+shift,BLACK);
            pfd_text_sized(p,155.14055f,82.554756f+shift,value,needle,4.5f,1);
        }
    }
}

static void draw_selected_vertical_speed(const PfdCanvas *p, const Side *side)
{
    float target;
    if (!side->vertical_speed_valid || (published.fma_b1 != A380X_B1_VS && published.fma_b1 != A380X_B1_FPA)) return;
    target = (float)pfd_guidance.selected_vs_fpm;
    if (published.fma_b1 == A380X_B1_FPA)
        target = tanf((float)pfd_guidance.selected_fpa_deg*(float)M_PI/180.0f)*ground_speed_knots*101.2686f;
    if (isfinite(target)) {
        float y = 80.822f+vertical_speed_offset(target);
        pfd_line(p,151.9f,y-2.0f,154.0f,y,CYAN);
        pfd_line(p,154.0f,y,151.9f,y+2.0f,CYAN);
    }
}

static float smallest_angle(float value, float reference)
{
    float delta = value - reference;
    if (delta > 180.0f) delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;
    return delta;
}

static void draw_heading_tape(const PfdCanvas *p, const Side *side)
{
    int tick;
    char value[16];
    const float center = 68.906f;
    float heading_reference = side->heading_value + (float)pfd_guidance.heading_reference_offset_deg;
    pfd_rect(p,32.138f,145.34f,105.674f,155.722f,TAPE_BACKGROUND);
    pfd_line(p,32.138f,156.23f,32.138f,145.34f,side->heading_valid ? WHITE : RED);
    pfd_line(p,32.138f,145.34f,105.674f,145.34f,side->heading_valid ? WHITE : RED);
    pfd_line(p,105.674f,145.34f,105.674f,156.23f,side->heading_valid ? WHITE : RED);
    if (!side->heading_valid || !isfinite(side->heading_value)) {
        pfd_text_center(p,68.906f,151.955f,"HDG",RED);
        return;
    }
    /* Align ticks to fixed magnetic headings and translate continuously.  Anchoring
     * ticks to the current heading produces a stationary, falsely labelled tape. */
    tick = (int)floorf((heading_reference - 25.0f) / 5.0f) * 5;
    for (; tick <= heading_reference + 25.0f; tick += 5) {
        float x = center + ((float)tick - heading_reference) * (7.555f / 5.0f);
        int heading = ((tick % 360) + 360) % 360;
        float length = heading % 10 == 0 ? 4.0f : 1.68f;
        if (x < 32.5f || x > 105.3f) continue;
        pfd_line(p,x,145.34f,x,145.34f + length,WHITE);
        if (heading % 10 == 0) {
            snprintf(value,sizeof(value),"%d",heading / 10);
            pfd_text_center(p,x,154.642f,value,WHITE);
        }
    }
    if (pfd_guidance.true_heading) {
        pfd_box(p,62.439f,134.468f,75.374f,139.043f,CYAN);
        pfd_text_center(p,68.9065f,138.2f,"TRUE",CYAN);
    }
    pfd_rect(p,68.0981f,139.2465f,69.61f,147.31f,YELLOW);
    if (side->navigation_valid) {
        float track_delta = smallest_angle(ground_track_degrees, side->heading_value);
        if (fabsf(track_delta) <= 24.0f) {
            float x = center + track_delta * (7.555f / 5.0f);
            Point diamond[4] = {{x,145.75f},{x-1.2592f,147.514f},{x,149.278f},{x+1.2592f,147.514f}};
            pfd_polygon_outline(p,diamond,4,GREEN);
        }
    }
}

typedef struct {
    int land_mode, autoland_warning, autoland_mask;
    int chrono_mode[2], chrono_seconds[2], chrono_frozen[2];
    float flash_time;
} PfdGlareState;
static PfdGlareState pfd_glare;

static int pfd_fd_selected(const Side *side)
{
    return side == &refs.side[1] ? pfd_guidance.fd2 : pfd_guidance.fd1;
}

static void draw_flight_director(const PfdCanvas *p, const Side *side)
{
    float x, y;
    if (pfd_glare.land_mode && side->radio_altitude_value<200.0f &&
        ((side->radio_altitude_value>15.0f && !side->loc_valid) ||
         (side->radio_altitude_value>100.0f && !side->gs_valid)) &&
        fmodf(pfd_glare.flash_time,1.0f)>=0.5f) return;
    if (!side->attitude_valid || !isfinite(side->pitch_value) || !isfinite(side->roll_value)) return;
    if ((pfd_guidance.trk_fpa_mode || efis[side == &refs.side[1]].vv) && side->vertical_speed_valid && side->heading_valid && ground_speed_knots > 30.0f) {
        Color vector_color = pfd_guidance.trk_fpa_mode ? GREEN : BLACK;
        /* FlightPathVector.tsx: drift limited to 21 degrees, rotated with attitude. */
        float drift = pfd_clamp(remainderf(ground_track_degrees-side->heading_value,360.0f),-21.0f,21.0f) * 1.5f;
        float fpa = atanf(side->vertical_speed_value / (ground_speed_knots * 101.2686f)) * (180.0f / (float)M_PI);
        float vertical = fbw_horizon_offset(side->pitch_value) - fbw_horizon_offset(fpa);
        float angle = -side->roll_value * (float)M_PI / 180.0f;
        x = 68.906f + drift*cosf(angle) - vertical*sinf(angle);
        y = 80.823f + vertical*cosf(angle) + drift*sinf(angle);
        if (x > 40.0f && x < 98.0f && y > 48.0f && y < 110.0f) {
            pfd_circle(p,x,y,2.25f,vector_color);
            pfd_line(p,x-7.25f,y,x-2.25f,y,vector_color);
            pfd_line(p,x+2.25f,y,x+7.25f,y,vector_color);
            pfd_line(p,x,y-2.25f,x,y-4.25f,vector_color);
        }
    }
    if (!pfd_fd_selected(side)) return;
    if (!pfd_guidance.guidance_valid) {
        if (pfd_guidance.guidance_fault) pfd_text(p,43.0f,56.065f,"FD",RED);
        return;
    }
    /* Do not draw guidance bars during unusual attitudes (FBW PFD declutter). */
    if (pfd_excessive_attitude(side)) return;
    x = 68.903f + pfd_clamp((float)pfd_guidance.fd_roll_error_deg,-45.0f,45.0f)*0.44f;
    y = 80.823f - pfd_clamp((float)pfd_guidance.fd_pitch_error_deg,-22.5f,22.5f)*0.89f;
    pfd_line_width(4.0f);
    pfd_line(p,x,61.672f,x,99.974f,BLACK);
    pfd_line(p,49.263f,y,88.550f,y,BLACK);
    pfd_line_width(2.0f);
    pfd_line(p,x,61.672f,x,99.974f,GREEN);
    pfd_line(p,49.263f,y,88.550f,y,GREEN);
    pfd_line_width(1.0f);
    draw_fixed_aircraft(p);
}

static void draw_speed_targets(const PfdCanvas *p, const Side *side)
{
    char text[16];
    float indicated = fmaxf(side->ias_value,30.0f);
    float target = (float)pfd_guidance.selected_speed_knots;
    Color target_color = pfd_guidance.speed_managed ? MAGENTA : CYAN;
    if (!side->air_valid || !isfinite(side->ias_value)) return;
    if (isfinite(target) && target > 0.0f) {
        float y = 80.818f - (target-indicated);
        if (y > 40.0f && y < 121.0f) {
            Point bug[] = {{19.274f,y+1.077f},{24.6317f,y+3.0282f},
                           {24.6317f,y-3.0194f},{19.274f,y-1.0682f}};
            pfd_polygon_outline(p,bug,4,target_color);
        } else {
            snprintf(text,sizeof(text),"%03d",(int)lroundf(target));
            pfd_text_center(p,16.0f,y <= 40.0f ? 36.67f : 128.28f,text,target_color);
        }
    }
    if (pfd_guidance.vmax_knots > 0.0) {
        float end = pfd_clamp(80.818f-((float)pfd_guidance.vmax_knots-indicated),38.1f,123.5f);
        float y;
        for (y=38.1f;y<end;y+=5.04f)
            pfd_rect(p,19.031f,y,22.053f,fminf(y+2.419f,end),RED);
        pfd_line(p,22.053f,38.1f,22.053f,end,RED);
    }
    if (pfd_guidance.vls_knots > 0.0) {
        float y = 80.818f-((float)pfd_guidance.vls_knots-indicated);
        float floor_speed=(float)(pfd_guidance.alpha_prot_knots>0.0 ?
            pfd_guidance.alpha_prot_knots : pfd_guidance.stall_warning_knots);
        if (y>38.1f && y<123.5f) {
            pfd_line(p,19.031f,y,21.006f,y,AMBER);
            if (floor_speed>0 && floor_speed<pfd_guidance.vls_knots)
                pfd_line(p,21.006f,y,21.006f,pfd_clamp(80.818f-(floor_speed-indicated),y,123.5f),AMBER);
        }
    }
    if (pfd_guidance.envelope_valid && !pfd_guidance.on_ground) {
        float prot=(float)pfd_guidance.alpha_prot_knots;
        float limit=(float)pfd_guidance.alpha_limit_knots;
        float stall=(float)pfd_guidance.stall_warning_knots;
        if (limit>0) {
            float top=pfd_clamp(80.818f-(limit-indicated),38.1f,123.5f);
            pfd_rect(p,19.031f,top,22.053f,123.5f,RED);
        }
        if (prot>limit && limit>0) {
            float top=pfd_clamp(80.818f-(prot-indicated),38.1f,123.5f);
            float bottom=pfd_clamp(80.818f-(limit-indicated),38.1f,123.5f);
            for(float y=top;y<bottom;y+=5.04f) pfd_rect(p,19.031f,y,22.053f,fminf(y+2.419f,bottom),AMBER);
        } else if (prot<=0 && stall>0) {
            float top=pfd_clamp(80.818f-(stall-indicated),38.1f,123.5f);
            for(float y=top;y<123.5f;y+=5.04f) pfd_rect(p,19.031f,y,22.053f,fminf(y+2.419f,123.5f),RED);
        }
    }
    {
        const double marks[]={pfd_guidance.f_speed_knots,pfd_guidance.s_speed_knots};
        const char *names[]={"F","S"};
        for(int i=0;i<2;++i) if(marks[i]>0) {
            float y=80.818f-((float)marks[i]-indicated);
            if(y>=40&&y<=121) pfd_text(p,19.5f,y+2.0f,names[i],GREEN);
        }
        if(pfd_guidance.vfe_next_knots>0 && !pfd_guidance.on_ground && guidance_inputs.pressure_altitude_ft<20000) {
            float y=80.818f-((float)pfd_guidance.vfe_next_knots-indicated);
            if(y>=40&&y<=121) {
                pfd_line(p,19.0f,y-1.0f,22.0f,y-1.0f,AMBER);
                pfd_line(p,19.0f,y+1.0f,22.0f,y+1.0f,AMBER);
            }
        }
    }
    if (published.flight_phase <= 5 && pfd_guidance.v1_knots > 0.0) {
        float y = 80.818f-((float)pfd_guidance.v1_knots-indicated);
        if (y>=40.0f && y<=121.0f) pfd_text(p,22.0f,y+2.0f,"1",CYAN);
        else if (y<40.0f) {
            snprintf(text,sizeof(text),"%.0f",pfd_guidance.v1_knots);
            pfd_text(p,22.0f,43.23f,text,CYAN);
        }
    }
    if (published.flight_phase <= 5 && pfd_guidance.vr_knots > 0.0) {
        float y = 80.818f-((float)pfd_guidance.vr_knots-indicated);
        if (y>=40.0f && y<=121.0f) pfd_circle(p,19.0f,y,1.5f,CYAN);
    }
    if (pfd_guidance.green_dot_knots > 0.0 && pfd_guidance.flap_config==0 && !pfd_guidance.on_ground) {
        float y = 80.818f-((float)pfd_guidance.green_dot_knots-indicated);
        if (y>=40.0f && y<=121.0f) pfd_circle(p,19.0f,y,1.5f,GREEN);
    }
}

static void draw_selected_altitude(const PfdCanvas *p, const Side *side)
{
    float target = (float)pfd_guidance.selected_altitude_ft;
    float y;
    char value[24];
    if (!side->altitude_valid || !isfinite(side->altitude_value) || !isfinite(target)) return;
    if (side->baro_std) snprintf(value,sizeof(value),"FL%03d",(int)lroundf(target / 100.0f));
    else snprintf(value,sizeof(value),"%05d",(int)lroundf(target));
    y = 80.819f - (target-side->altitude_value)*0.075f;
    if (y < 41.0f || y > 121.0f) {
        pfd_text_center(p,127.0f,y < 41.0f ? 36.9f : 128.70f,value,CYAN);
    } else {
        /* Target symbol remains legible even when it crosses the altitude drums. */
        pfd_line(p,122.79f,y+3.012f,122.79f,y+9.564f,CYAN);
        pfd_line(p,122.79f,y+9.564f,115.74f,y+9.564f,CYAN);
        pfd_line(p,115.74f,y+9.564f,115.74f,y+0.996f,CYAN);
        pfd_line(p,115.74f,y+0.996f,117.75f,y,CYAN);
        pfd_line(p,122.79f,y-3.035f,122.79f,y-9.586f,CYAN);
        pfd_line(p,122.79f,y-9.586f,115.93f,y-9.586f,CYAN);
        pfd_line(p,115.93f,y-9.586f,115.93f,y-1.018f,CYAN);
        pfd_line(p,115.93f,y-1.018f,117.75f,y,CYAN);
        if (fabsf(y-80.819f)>11.0f) {
            pfd_rect(p,117.75f,y-3.02f,139.0f,y+3.02f,BLACK);
            pfd_text(p,118.23f,y+2.25f,value,CYAN);
        }
    }
    if (pfd_guidance.metric_altitude) {
        snprintf(value,sizeof(value),"%.0f",roundf(target*0.3048f/10.0f)*10.0f);
        pfd_text_center(p,95.0f,37.93f,value,CYAN);
        pfd_text(p,104.0f,37.93f,"M",CYAN);
        pfd_box(p,116.56f,140.22f,145.77f,147.28f,YELLOW);
        snprintf(value,sizeof(value),"%.0f",roundf(side->altitude_value*0.3048f/10.0f)*10.0f);
        pfd_text_center(p,128.65f,145.86f,value,GREEN);
        pfd_text_center(p,142.03f,145.86f,"M",CYAN);
    }
}

static void draw_baro(const PfdCanvas *p, const Side *side)
{
    char value[16];
    if (side->baro_std) {
        pfd_box(p,124.79f,131.74f,137.89f,138.80f,YELLOW);
        pfd_text_center(p,131.3f,137.36f,"STD",CYAN);
    } else if (isfinite(side->baro_hpa_value)) {
        pfd_text(p,117.0f,138.1f,"QNH",WHITE);
        if (efis[side == &refs.side[1]].baro_inhg)
            snprintf(value,sizeof(value),"%.2f",side->baro_hpa_value/33.8638867f);
        else snprintf(value,sizeof(value),"%.0f",side->baro_hpa_value);
        pfd_text(p,129.5f,138.1f,value,CYAN);
    }
}

static void draw_selected_heading(const PfdCanvas *p, const Side *side)
{
    float delta;
    char value[8];
    if (!side->heading_valid || !isfinite(side->heading_value) || !isfinite(pfd_guidance.selected_heading_deg)) return;
    delta = smallest_angle((float)pfd_guidance.selected_heading_deg,side->heading_value);
    if (fabsf(delta)<22.0f) {
        float x=68.906f+delta*(7.555f/5.0f);
        pfd_line(p,x+1.072f,145.1f,x+3.022f,139.739f,CYAN);
        pfd_line(p,x+3.022f,139.739f,x-3.022f,139.739f,CYAN);
        pfd_line(p,x-3.022f,139.739f,x-1.072f,145.1f,CYAN);
    } else {
        snprintf(value,sizeof(value),"%03d",((int)lround(pfd_guidance.selected_heading_deg+pfd_guidance.heading_reference_offset_deg)%360+360)%360);
        pfd_text_center(p,delta<0.0f ? 36.418f : 101.704f,144.348f,value,CYAN);
    }
}

static void draw_landing_system(const PfdCanvas *p, const Side *side)
{
    static const float offsets[] = {-30.221f,-15.1105f,15.1105f,30.221f};
    int i, hide_loc, hide_gs;
    char nav_text[24];
    if (!side->ls_visible && !pfd_glare.land_mode) return;
    hide_loc=pfd_glare.land_mode && (pfd_glare.autoland_mask&2) &&
        fmodf(pfd_glare.flash_time,1.0f)>=0.5f;
    hide_gs=pfd_glare.land_mode && (pfd_glare.autoland_mask&4) &&
        fmodf(pfd_glare.flash_time,1.0f)>=0.5f;
    if (side->nav_ident[0]) pfd_text(p,1.184f,143.115f,side->nav_ident,MAGENTA);
    if (side->nav_frequency_mhz>=108.0f && side->nav_frequency_mhz<=118.0f) {
        snprintf(nav_text,sizeof(nav_text),"%.2f",side->nav_frequency_mhz);
        pfd_text(p,1.184f,149.5f,nav_text,MAGENTA);
    }
    if (side->nav_dme_valid && isfinite(side->nav_dme_nm)) {
        if (side->nav_dme_nm<20.0f) snprintf(nav_text,sizeof(nav_text),"%.1fNM",side->nav_dme_nm);
        else snprintf(nav_text,sizeof(nav_text),"%.0fNM",side->nav_dme_nm);
        pfd_text(p,1.184f,155.5f,nav_text,MAGENTA);
    }
    for (i=0;i<4;++i) {
        if (!hide_loc) pfd_circle(p,68.906f+offsets[i],130.51f,1.0074f,WHITE);
        if (!hide_gs) pfd_circle(p,109.7f,80.823f+offsets[i],1.0074f,WHITE);
    }
    if (!hide_loc) pfd_rect(p,68.098f,126.437f,69.610f,134.500f,YELLOW);
    if (!hide_gs) pfd_rect(p,106.549f,80.067f,115.520f,81.579f,YELLOW);
    if (!hide_loc && side->loc_valid && isfinite(side->loc_dots)) {
        float x=68.906f+pfd_clamp(side->loc_dots,-2.0f,2.0f)*15.1105f;
        if (side->loc_dots>2.0f) {
            pfd_line(p,x,127.990f,x+3.778f,130.510f,MAGENTA);
            pfd_line(p,x+3.778f,130.510f,x,133.030f,MAGENTA);
        } else if (side->loc_dots< -2.0f) {
            pfd_line(p,x,127.990f,x-3.778f,130.510f,MAGENTA);
            pfd_line(p,x-3.778f,130.510f,x,133.030f,MAGENTA);
        } else {
            Point diamond[]={{x-3.778f,130.51f},{x,133.030f},{x+3.778f,130.51f},{x,127.99f}};
            pfd_polygon_outline(p,diamond,4,MAGENTA);
        }
    }
    if (!hide_gs && side->gs_valid && isfinite(side->gs_dots)) {
        float y=80.823f+pfd_clamp(side->gs_dots,-2.0f,2.0f)*15.119f;
        if (fabsf(side->gs_dots)>2.0f) {
            float tip=side->gs_dots>0.0f ? 3.780f : -3.780f;
            pfd_line(p,107.182f,y,109.7f,y+tip,MAGENTA);
            pfd_line(p,109.7f,y+tip,112.218f,y,MAGENTA);
        } else {
            Point diamond[]={{109.7f,y-3.780f},{107.182f,y},{109.7f,y+3.780f},{112.218f,y}};
            pfd_polygon_outline(p,diamond,4,MAGENTA);
        }
    }
}

/* FBW FMA highlights newly engaged modes for ten seconds.  Track each
 * display independently so powering the second display does not reset CAPT. */
typedef struct { int initialized, signature[8]; float changed_at[8]; } PfdFmaHistory;
static PfdFmaHistory pfd_fma_history[2];

static void fma_change_box(const PfdCanvas *p, int side_index, int cell, int signature,
                           float left, float top, float right, float bottom, Color color)
{
    PfdFmaHistory *history = &pfd_fma_history[side_index];
    float now = XPLMGetElapsedTime();
    if (!history->initialized || history->signature[cell] != signature) {
        history->signature[cell] = signature;
        history->changed_at[cell] = now;
    }
    if (signature && now - history->changed_at[cell] < 10.0f)
        pfd_box(p,left,top,right,bottom,color);
}

static void draw_fma(const PfdCanvas *p, const Side *side)
{
    int active_vertical = pfd_guidance.guidance_valid ? published.fma_b1 : A380X_B1_NONE;
    int active_lateral = pfd_guidance.guidance_valid ? published.fma_c1 : A380X_C1_NONE;
    int active_thrust = published.fma_a1a2;
    if (!pfd_guidance.athr_active && active_thrust >= A380X_A1A2_SPEED && active_thrust <= A380X_A1A2_TOGA_LK)
        active_thrust = A380X_A1A2_NONE;
    const char *a1 = a380x_a1a2_text((A380XA1A2Message)active_thrust);
    const char *a3 = a380x_a3_text((A380XA3Message)published.fma_a3);
    const char *b1 = a380x_b1_text((A380XB1Message)active_vertical);
    const char *c1 = a380x_c1_text((A380XC1Message)active_lateral);
    const char *d1 = a380x_d1d2_text((A380XD1D2Message)published.fma_d1d2);
    static const char *brake_modes[] = {"", "BTV", "BRK LO", "BRK 2", "BRK 3", "BRK HI", "BRK RTO"};
    const char *brake_mode = (center_right.autobrake_armed >= 0 && center_right.autobrake_armed < 7)
        ? brake_modes[center_right.autobrake_armed] : "";
    static const float borders[] = {33.117f,66.241f,102.52f,133.72f};
    char vertical_text[24], armed_vertical[20], fd_status[8];
    const char *ap_status = pfd_guidance.ap1 ? (pfd_guidance.ap2 ? "AP1+2" : "AP1") :
                            (pfd_guidance.ap2 ? "AP2" : "");
    int i;
    int side_index = side == &refs.side[1] ? 1 : 0;
    for (i = 0; i < ARRAY_LEN(borders); ++i) pfd_line(p,borders[i],0.337f,borders[i],21.201f,GREY);
    if (center_right.autobrake_active) {
        pfd_text_center(p,16.782f,7.128f,brake_mode,GREEN);
    } else if (strncmp(a1,"MAN ",4) == 0) {
        pfd_text_center(p,16.782f,7.128f,"MAN",WHITE);
        pfd_text_center(p,16.782f,14.352f,a1 + 4,WHITE);
        pfd_box(p,1.0f,1.0f,32.0f,15.5f,WHITE);
    } else {
        pfd_text_center(p,16.782f,7.128f,a1,GREEN);
        if (active_thrust==A380X_A1A2_A_FLOOR || active_thrust==A380X_A1A2_TOGA_LK) {
            if (fmodf(XPLMGetElapsedTime(),1.0f)<0.5f) pfd_box(p,1.0f,1.0f,32.0f,8.7f,AMBER);
        } else fma_change_box(p,side_index,0,active_thrust,1.0f,1.0f,32.0f,8.7f,WHITE);
    }
    if (!center_right.autobrake_active && !center_right.rto_arm &&
        active_thrust == A380X_A1A2_NONE)
        pfd_text_center(p,16.782f,14.329653f,brake_mode,CYAN);
    if (center_right.rto_arm && !center_right.autobrake_active)
        pfd_text_center(p,16.99f,21.641f,"RTO",CYAN);
    else pfd_text_center(p,16.99f,21.641f,a3,AMBER);
    if (active_vertical == A380X_B1_VS) {
        snprintf(vertical_text,sizeof(vertical_text),"V/S %+.0f",pfd_guidance.selected_vs_fpm);
        b1=vertical_text;
    } else if (active_vertical == A380X_B1_FPA) {
        snprintf(vertical_text,sizeof(vertical_text),"FPA %+.1f",pfd_guidance.selected_fpa_deg);
        b1=vertical_text;
    }
    if (!pfd_guidance.mode_reversion || ((int)(XPLMGetElapsedTime()*2.0f))%2==0)
        pfd_text_center(p,49.922f,7.104f,b1,GREEN);
    if (pfd_guidance.guidance_valid && pfd_guidance.mode_reversion) pfd_box(p,34.0f,1.0f,65.4f,8.7f,WHITE);
    snprintf(armed_vertical,sizeof(armed_vertical),"%s%s%s",
             pfd_guidance.altitude_armed ? "ALT" : "",
             pfd_guidance.altitude_armed && pfd_guidance.gs_armed ? " " : "",
             pfd_guidance.gs_armed ? "G/S" : "");
    if (pfd_guidance.guidance_valid) pfd_text_center(p,49.922f,14.36f,armed_vertical,CYAN);
    if (pfd_guidance.guidance_valid && pfd_guidance.loc_armed) pfd_text_center(p,84.857f,14.36f,"LOC",CYAN);
    pfd_text_center(p,84.857f,6.987f,c1,GREEN);
    pfd_text_center(p,118.459f,7.126f,d1,WHITE);
    fma_change_box(p,side_index,1,active_vertical,34.0f,1.0f,65.4f,8.7f,WHITE);
    fma_change_box(p,side_index,2,active_lateral,67.1f,1.0f,101.7f,8.7f,WHITE);
    pfd_text_center(p,145.62f,6.956f,ap_status,WHITE);
    if (pfd_guidance.ap1 || pfd_guidance.ap2 || pfd_guidance.fd1 || pfd_guidance.fd2) {
        snprintf(fd_status,sizeof(fd_status),"%cFD%c",pfd_guidance.fd1 ? '1' : '-',pfd_guidance.fd2 ? '2' : '-');
        pfd_text_center(p,145.95f,14.417f,fd_status,WHITE);
    }
    if (pfd_guidance.athr_armed || pfd_guidance.athr_active)
        pfd_text_center(p,145.76f,21.435f,"A/THR",pfd_guidance.athr_active ? WHITE : CYAN);
    fma_change_box(p,side_index,3,pfd_guidance.ap1+2*pfd_guidance.ap2,
                   135.32f,1.814f,156.13f,7.862f,WHITE);
    fma_change_box(p,side_index,4,pfd_guidance.fd1+2*pfd_guidance.fd2,
                   135.32f,9.072f,156.13f,15.12f,WHITE);
    fma_change_box(p,side_index,5,pfd_guidance.athr_armed+2*pfd_guidance.athr_active,
                   135.32f,16.329f,156.13f,22.377f,WHITE);
    pfd_fma_history[side_index].initialized = 1;
}

#include "a380x_pfd_configuration.inc"

static void draw_lower_area(const PfdCanvas *p)
{
    pfd_line(p,2.1f,157.7f,156.5f,157.7f,WHITE);
    pfd_line(p,67.0f,158.0f,67.0f,209.8f,WHITE);
    /* Port capability/disconnect advisories occupy the lower area rather than
     * inventing an Airbus FMA mode or approach-capability annunciation. */
    if (pfd_guidance.disconnect_warning) pfd_text_center(p,113.0f,173.0f,"AP OFF",RED);
    if (pfd_guidance.managed_unavailable) pfd_text_center(p,113.0f,182.0f,"FMS GUIDANCE N/A",AMBER);
    draw_pfd_configuration(p);
}

static void draw_pfd(const Side *side, float x, float y, const char *side_name)
{
    char value[16];
    PfdCanvas p = pfd_canvas(x,y);
    (void)side_name;
    rectangle(x,y,x + PFD_PIXEL_WIDTH,y + PFD_PIXEL_HEIGHT,BLACK);
    if (!side->display_valid) {
        pfd_fma_history[side == &refs.side[1] ? 1 : 0].initialized = 0;
        memset(&pfd_display_state[side==&refs.side[1]],0,sizeof(PfdDisplayState));
        return;
    }
    pfd_line_width(1.0f);
    draw_horizon(&p,side);
    draw_flight_director(&p,side);
    draw_heading_tape(&p,side);
    draw_selected_heading(&p,side);
    draw_altitude_tape(&p,side);
    draw_speed_tape(&p,side);
    draw_speed_targets(&p,side);
    draw_selected_altitude(&p,side);
    draw_baro(&p,side);
    draw_vertical_speed(&p,side);
    draw_selected_vertical_speed(&p,side);
    draw_landing_system(&p,side);
    if (pfd_guidance.stall_warning && !pfd_guidance.on_ground)
        pfd_text_sized(&p,68.906f,100.0f,"STALL",RED,7.0f,1);
    if (pfd_excessive_attitude(side)) {
        /* FBW attitude declutter hides radio height until the recovery bounds. */
    } else if (!side->radio_altitude_valid || !isfinite(side->radio_altitude_value)) {
        pfd_text_center(&p,69.202f,121.5f,"RA",RED);
    } else if (side->radio_altitude_value < 2500.0f) {
        float ra = side->radio_altitude_value;
        float increment = ra < 5.0f ? 1.0f : ra <= 50.0f ? 5.0f : 10.0f;
        snprintf(value,sizeof(value),"%d",(int)(floorf(ra / increment + 0.5f) * increment));
        int dh_valid=isfinite(side->decision_height_value) && side->decision_height_value>=0;
        int above_dh_margin=dh_valid && ra>side->decision_height_value+100.0f;
        pfd_text_sized(&p,69.202f,121.5f,value,ra>400.0f || above_dh_margin ? GREEN : AMBER,
                       ra<400.0f ? 7.0f : 6.5f,1);
        if (dh_valid && ra<=side->decision_height_value)
            pfd_text_sized(&p,73.511879f,115.0f,"DH",AMBER,7.0f,2);
    }
    draw_fma(&p,side);
    draw_lower_area(&p);
    if (efis[side == &refs.side[1]].taxi_requested)
        pfd_text_center(&p,78.0f,166.0f,"ETACS NOT AVAILABLE",AMBER);
    if (pfd_glare.autoland_warning && fmodf(pfd_glare.flash_time,1.0f)<0.5f)
        pfd_text_center(&p,113.0f,192.0f,"AUTOLAND",RED);
    glLineWidth(1.0f);
}

/* EFIS map rendering is fed only by cached simulator source data. */
typedef struct {
    float x,y,cx,cy,radius,range_nm,heading;
    double lat,lon;
    int arc;
    /* Map clip box and label limit in drawing units, and the symbol/offset scale (1 for the
     * 180 x 234 native layout, larger for the FBW 768 x 1024 layout of a380x_nd_fbw.inc). */
    float clip_l,clip_r,clip_b,clip_t,label_r,sym;
} EfisNdCanvas;

static int nd_map_point(const EfisNdCanvas *map,double lat,double lon,float *px,float *py)
{
    float east,north;
    if (!efis_nd_project(map->lat,map->lon,lat,lon,map->heading,&east,&north)) return 0;
    *px=east*map->radius/map->range_nm;
    *py=north*map->radius/map->range_nm;
    return 1;
}

static int nd_map_visible(const EfisNdCanvas *map,float px,float py,float margin)
{
    return efis_nd_in_map(px,py,map->radius-margin,map->arc) &&
        map->cx+px>map->clip_l+margin && map->cx+px<map->clip_r-margin &&
        map->cy+py>map->clip_b+margin && map->cy+py<map->clip_t-margin;
}

static void nd_map_segment(const EfisNdCanvas *map,float x1,float y1,float x2,float y2,Color color)
{
    int n=(int)ceilf(hypotf(x2-x1,y2-y1)/3.0f),i;
    float previous_x=x1,previous_y=y1;
    if (n<1) n=1;
    if (n>256) n=256;
    for (i=1;i<=n;++i) {
        float px=x1+(x2-x1)*i/n,py=y1+(y2-y1)*i/n;
        if (nd_map_visible(map,previous_x,previous_y,0) && nd_map_visible(map,px,py,0))
            line(map->cx+previous_x,map->cy+previous_y,map->cx+px,map->cy+py,color);
        previous_x=px; previous_y=py;
    }
}

static int nd_airport_available(void)
{
    return efis_airport.found && (efis_airport.runway_count>0 || efis_airport.ring_count>0);
}

static void nd_map_filled_quad(const EfisNdCanvas *map,const Point points[4],Color color)
{
    Point a[100],b[100];
    int count=4,i;
    memcpy(a,points,4*sizeof(Point));
#define ND_CLIP(nx,ny,offset) do { \
    count=clip_half_plane(a,count,b,nx,ny,offset,1); \
    memcpy(a,b,(size_t)count*sizeof(Point)); \
} while(0)
    ND_CLIP(1,0,map->cx-map->clip_l);
    ND_CLIP(-1,0,map->clip_r-map->cx);
    ND_CLIP(0,1,map->cy-map->clip_b);
    ND_CLIP(0,-1,map->clip_t-map->cy);
    for(i=0;i<32 && count>0;++i) {
        float angle=i*(float)M_PI/16;
        ND_CLIP(-cosf(angle),-sinf(angle),map->radius*cosf((float)M_PI/32));
    }
    if(map->arc) {
        ND_CLIP(0,1,0);ND_CLIP(1,1.1917535926f,0);ND_CLIP(-1,1.1917535926f,0);
    }
#undef ND_CLIP
    filled_polygon(a,count,map->cx,map->cy,color);
}

static void nd_draw_airport(const EfisNdCanvas *map)
{
    int i,j;
    static const Color pavement={.16f,.21f,.26f};
    static const Color runway_fill={.26f,.30f,.34f};
    if(!nd_airport_available()) return;
    /* apt.dat pavement rings remain outlines: separate hole rings are kept,
     * never filled across taxiway holes or concave apron boundaries. */
    for(i=0;i<efis_airport.ring_count;++i) {
        int first=efis_airport.rings[i].first,count=efis_airport.rings[i].count;
        if(first<0||count<2||first+count>efis_airport.node_count) continue;
        for(j=0;j<count;++j) {
            int a=first+j,b=first+(j+1)%count,k;
            float x0,y0,x1,y1,cx0,cy0,cx1,cy1,previous_x,previous_y;
            int curved=efis_airport.nodes[a].bezier||efis_airport.nodes[b].bezier;
            int steps=curved?8:1;
            if(!nd_map_point(map,efis_airport.nodes[a].lat,efis_airport.nodes[a].lon,&x0,&y0)||
               !nd_map_point(map,efis_airport.nodes[b].lat,efis_airport.nodes[b].lon,&x1,&y1)) continue;
            cx0=x0;cy0=y0;cx1=x1;cy1=y1;
            if(efis_airport.nodes[a].bezier)
                nd_map_point(map,efis_airport.nodes[a].control_lat,efis_airport.nodes[a].control_lon,&cx0,&cy0);
            if(efis_airport.nodes[b].bezier) {
                float control_x=x1,control_y=y1;
                nd_map_point(map,efis_airport.nodes[b].control_lat,efis_airport.nodes[b].control_lon,&control_x,&control_y);
                cx1=2*x1-control_x;cy1=2*y1-control_y;
            }
            /* Reject wholly off-map Bezier hulls before emitting any GL. */
            if(fminf(fminf(x0,x1),fminf(cx0,cx1))>map->radius ||
               fmaxf(fmaxf(x0,x1),fmaxf(cx0,cx1))<-map->radius ||
               fminf(fminf(y0,y1),fminf(cy0,cy1))>map->radius ||
               fmaxf(fmaxf(y0,y1),fmaxf(cy0,cy1))<-map->radius) continue;
            previous_x=x0;previous_y=y0;
            for(k=1;k<=steps;++k) {
                float t=(float)k/steps,u=1-t;
                float px=u*u*u*x0+3*u*u*t*cx0+3*u*t*t*cx1+t*t*t*x1;
                float py=u*u*u*y0+3*u*u*t*cy0+3*u*t*t*cy1+t*t*t*y1;
                nd_map_segment(map,previous_x,previous_y,px,py,pavement);
                previous_x=px;previous_y=py;
            }
        }
    }
    for(i=0;i<efis_airport.runway_count;++i) {
        float ax,ay,bx,by,dx,dy,length,rx,ry,half_width;
        Point quad[4];
        if(!nd_map_point(map,efis_airport.runways[i].lat1,efis_airport.runways[i].lon1,&ax,&ay)||
           !nd_map_point(map,efis_airport.runways[i].lat2,efis_airport.runways[i].lon2,&bx,&by)) continue;
        dx=bx-ax;dy=by-ay;length=hypotf(dx,dy);
        if(length<.01f||!isfinite(efis_airport.runways[i].width_m)||efis_airport.runways[i].width_m<=0) continue;
        half_width=efis_airport.runways[i].width_m*.5f/1852.0f*map->radius/map->range_nm;
        rx=dy/length*half_width;ry=-dx/length*half_width;
        quad[0]=(Point){ax+rx,ay+ry};quad[1]=(Point){bx+rx,by+ry};
        quad[2]=(Point){bx-rx,by-ry};quad[3]=(Point){ax-rx,ay-ry};
        nd_map_filled_quad(map,quad,runway_fill);
        nd_map_segment(map,ax,ay,bx,by,WHITE);
        if(nd_map_visible(map,ax,ay,8)&&map->cx+ax<map->label_r+6*map->sym)
            text_at(map->cx+ax+3,map->cy+ay+3,efis_airport.runways[i].end1,WHITE);
        if(nd_map_visible(map,bx,by,8)&&map->cx+bx<map->label_r+6*map->sym)
            text_at(map->cx+bx+3,map->cy+by+3,efis_airport.runways[i].end2,WHITE);
    }
}

static void nd_diamond(float x,float y,float size,Color color)
{
    line(x,y+size,x+size,y,color); line(x+size,y,x,y-size,color);
    line(x,y-size,x-size,y,color); line(x-size,y,x,y+size,color);
}

/* TERRAIN on ND (a380x_terrain_render.inc): the side's swept EGPWS raster under the map symbols,
 * scaled to this map and clipped to its box. Nothing while TERR is off or unavailable. */
static void nd_draw_terrain(const EfisNdCanvas *map,int side)
{
    if (!efis[side].terr || efis[side].zoom) return;
    terrain_gl_draw(side,map->cx,map->cy,map->radius/map->range_nm,map->clip_l,map->clip_r,map->clip_b,map->clip_t);
}

/* ---- Native FMS flight plan on the ND (FBW: ACTIVE solid green, TMPY dashed yellow, missed
 * approach cyan, TO waypoint white). Geometry comes from a380x_fpln_leg_path(). ---------------- */

static void nd_map_dashed(const EfisNdCanvas *map,float x1,float y1,float x2,float y2,Color color)
{
    const float dash=4.0f*map->sym,gap=3.0f*map->sym;
    float length=hypotf(x2-x1,y2-y1),at=0.0f;
    if (length<0.01f) return;
    while (at<length) {
        float end=fminf(at+dash,length);
        nd_map_segment(map,x1+(x2-x1)*at/length,y1+(y2-y1)*at/length,
                       x1+(x2-x1)*end/length,y1+(y2-y1)*end/length,color);
        at=end+gap;
    }
}

static void nd_map_path_segment(const EfisNdCanvas *map,float x1,float y1,float x2,float y2,Color color,int dashed)
{
    if (dashed) nd_map_dashed(map,x1,y1,x2,y2,color);
    else nd_map_segment(map,x1,y1,x2,y2,color);
}

static void nd_draw_leg_path(const EfisNdCanvas *map,const A380XFplnLegPath *path,Color color,int dashed)
{
    float sx,sy,ex,ey;
    if (!nd_map_point(map,path->start_lat,path->start_lon,&sx,&sy) ||
        !nd_map_point(map,path->end_lat,path->end_lon,&ex,&ey)) return;
    if (path->kind==A380X_PATH_ARC) {
        float cx,cy,a0,a1,sweep,radius;
        int k,steps=16;
        if (!nd_map_point(map,path->center_lat,path->center_lon,&cx,&cy)) {
            nd_map_path_segment(map,sx,sy,ex,ey,color,dashed);
            return;
        }
        radius=hypotf(sx-cx,sy-cy);
        a0=atan2f(sy-cy,sx-cx); a1=atan2f(ey-cy,ex-cx);
        sweep=a1-a0;
        /* Screen angles grow counterclockwise; a right (clockwise) turn sweeps negative. */
        if (path->clockwise) { while (sweep>0) sweep-=2*(float)M_PI; }
        else { while (sweep<0) sweep+=2*(float)M_PI; }
        for (k=0;k<steps;++k) {
            float t0=a0+sweep*k/steps,t1=a0+sweep*(k+1)/steps;
            nd_map_path_segment(map,cx+radius*cosf(t0),cy+radius*sinf(t0),
                                cx+radius*cosf(t1),cy+radius*sinf(t1),color,dashed);
        }
        return;
    }
    /* LINE, and HOLD drawn as its inbound course to the fix until racetracks are ported. */
    nd_map_path_segment(map,sx,sy,ex,ey,color,dashed);
}

static int nd_fpln_plan_drawable(int plan_index)
{
    const A380XFplnPlan *p=fmsa_fpln?a380x_fpln_plan(fmsa_fpln,plan_index):NULL;
    return p && p->exists && p->pri.count>0;
}

/* Draws one plan's primary route from its FROM waypoint onwards. */
static void nd_draw_fpln_plan(const EfisNdCanvas *map,int side,int plan_index)
{
    const A380XFplnPlan *p=a380x_fpln_plan(fmsa_fpln,plan_index);
    const A380XFplnRoute *r=&p->pri;
    const int tmpy=plan_index==A380X_PLAN_TEMPORARY;
    const int from=a380x_fpln_from_leg_index(p);
    int i,labels=0,start=from>0?from:0;
    for (i=start+1;i<r->count;++i) {
        A380XFplnLegPath path;
        if (!a380x_fpln_leg_path(fmsa_fpln,plan_index,0,i,&path)) continue;
        nd_draw_leg_path(map,&path,tmpy?YELLOW:path.is_missed?CYAN:GREEN,tmpy);
    }
    for (i=start;i<r->count;++i) {
        const A380XFplnLeg *leg=&r->el[i];
        float px,py;
        Color color;
        if (a380x_fpln_is_disco(leg) || !isfinite(leg->term_lat) || !isfinite(leg->term_lon)) continue;
        if (!nd_map_point(map,leg->term_lat,leg->term_lon,&px,&py) || !nd_map_visible(map,px,py,6)) continue;
        color=tmpy?YELLOW:i==p->active_leg_index?WHITE:i>=r->first_missed_appr_index?CYAN:GREEN;
        nd_diamond(map->cx+px,map->cy+py,3*map->sym,color);
        if (labels<10 &&  px<map->label_r-map->cx) {
            char label[24];
            A380XFplnAltCstr alt=a380x_fpln_eff_alt(leg);
            snprintf(label,sizeof(label),"%.7s",leg->ident[0]?leg->ident:"LATLON");
            text_at(map->cx+px+4*map->sym,map->cy+py+3*map->sym,label,color);
            if (efis[side].cstr && !tmpy && alt.desc && isfinite(alt.alt1_ft) && alt.alt1_ft>0) {
                snprintf(label,sizeof(label),"%.0f",alt.alt1_ft);
                text_at(map->cx+px+4*map->sym,map->cy+py-8*map->sym,label,MAGENTA);
            }
            ++labels;
        }
    }
}

/* Returns 1 when the native FMS has a route to show (the stock X-Plane FMS route is then ignored). */
static int nd_draw_fms_route(const EfisNdCanvas *map,int side)
{
    int drew=0;
    if (nd_fpln_plan_drawable(A380X_PLAN_ACTIVE)) { nd_draw_fpln_plan(map,side,A380X_PLAN_ACTIVE); drew=1; }
    if (nd_fpln_plan_drawable(A380X_PLAN_TEMPORARY)) { nd_draw_fpln_plan(map,side,A380X_PLAN_TEMPORARY); drew=1; }
    return drew;
}

/* PLAN mode centre: the TO waypoint of the plan shown on the F-PLN page (TMPY first), else its
 * first located waypoint. Returns 0 when the native FMS has no located waypoint. */
static int nd_fms_plan_centre(double *lat,double *lon)
{
    const int order[2]={A380X_PLAN_TEMPORARY,A380X_PLAN_ACTIVE};
    int k,i;
    for (k=0;k<2;++k) {
        const A380XFplnPlan *p;
        if (!nd_fpln_plan_drawable(order[k])) continue;
        p=a380x_fpln_plan(fmsa_fpln,order[k]);
        i=p->active_leg_index>=0?p->active_leg_index:0;
        for (;i<p->pri.count;++i) {
            const A380XFplnLeg *leg=&p->pri.el[i];
            if (!a380x_fpln_is_disco(leg) && isfinite(leg->term_lat) && isfinite(leg->term_lon)) {
                *lat=leg->term_lat; *lon=leg->term_lon;
                return 1;
            }
        }
    }
    return 0;
}

static void nd_draw_route(const EfisNdCanvas *map,int side)
{
    int i,labels=0;
    float previous_x=0,previous_y=0;
    int previous_valid=0;
    if (nd_draw_fms_route(map,side)) return; /* the stock X-Plane route is only a fallback */
    for (i=0;i<nd_data.route_count;++i) {
        const EfisNdRoutePoint *point=&nd_data.route[i];
        float px=0,py=0;
        int valid=nd_map_point(map,point->lat,point->lon,&px,&py);
        Color color=i==nd_data.route_active?WHITE:GREEN;
        if (valid && previous_valid) nd_map_segment(map,previous_x,previous_y,px,py,GREEN);
        if (valid && nd_map_visible(map,px,py,6)) {
            char label[24];
            nd_diamond(map->cx+px,map->cy+py,3*map->sym,color);
            if (labels<6 &&  px<map->label_r-map->cx) {
                snprintf(label,sizeof(label),"%.6s",point->id[0]?point->id:"LATLON");
                text_at(map->cx+px+4*map->sym,map->cy+py+3*map->sym,label,color);
                if (efis[side].cstr && point->altitude_ft>0) {
                    snprintf(label,sizeof(label),"%d",point->altitude_ft);
                    text_at(map->cx+px+4*map->sym,map->cy+py-8*map->sym,label,MAGENTA);
                }
                ++labels;
            }
        }
        previous_valid=valid; previous_x=px; previous_y=py;
    }
}

static void nd_draw_nav_layers(const EfisNdCanvas *map,int side)
{
    const int selected[]={efis[side].arpt,efis[side].vord,efis[side].ndb,efis[side].wpt};
    int kind,i;
    for (kind=0;kind<EFIS_ND_NAV_KINDS;++kind) {
        int labels=0;
        if (!selected[kind]) continue;
        for (i=0;i<nd_data.nav_count[kind];++i) {
            const EfisNdNavPoint *point=&nd_data.nav[kind][i];
            float px,py,sx,sy;
            char id[8];
            if (!nd_map_point(map,point->lat,point->lon,&px,&py) || !nd_map_visible(map,px,py,5)) continue;
            sx=map->cx+px; sy=map->cy+py;
            if (kind==0) { /* Airport: circular position symbol. */
                int k;
                set_color(MAGENTA); glBegin(GL_LINE_STRIP);
                for (k=0;k<=8;++k) glVertex2f(sx+3*map->sym*cosf(k*(float)M_PI/4),sy+3*map->sym*sinf(k*(float)M_PI/4));
                glEnd();
            } else if (kind==1) {
                const float k=map->sym;
                line(sx-3*k,sy-2*k,sx+3*k,sy-2*k,CYAN); line(sx+3*k,sy-2*k,sx,sy+3*k,CYAN);
                line(sx,sy+3*k,sx-3*k,sy-2*k,CYAN);
            } else if (kind==2) {
                nd_diamond(sx,sy,3*map->sym,CYAN); rectangle(sx-map->sym,sy-map->sym,sx+map->sym,sy+map->sym,CYAN);
            } else nd_diamond(sx,sy,2*map->sym,MAGENTA);
            if (labels<4 && sx<map->label_r) {
                snprintf(id,sizeof(id),"%.6s",point->id);
                text_at(sx+4*map->sym,sy+3*map->sym,id,kind==0||kind==3?MAGENTA:CYAN); ++labels;
            }
        }
    }
}

static void nd_draw_traffic(const EfisNdCanvas *map,int side)
{
    int i;
    if (!efis[side].traf || !nd_data.traffic_available) return;
    for (i=0;i<64;++i) {
        float distance=nd_data.traffic_distance_m[i]/1852.0f;
        float angle=(nd_data.traffic_bearing[i]+nd_data.true_heading-map->heading)*(float)M_PI/180.0f;
        float relative=nd_data.traffic_altitude_m[i]*3.280839895f;
        float px,py;
        char label[8];
        /* Zero-distance empty TCAS slots must never become ownship traffic. */
        if (!isfinite(distance)||distance<=0.01f||!isfinite(angle)||!isfinite(relative)||fabsf(relative)>9900.0f) continue;
        if (map->lat==nd_data.lat && map->lon==nd_data.lon) {
            px=sinf(angle)*distance*map->radius/map->range_nm;
            py=cosf(angle)*distance*map->radius/map->range_nm;
        } else {
            /* PLAN's map may be centred on another waypoint. Offset targets
             * from ownship using the same north-up chart scale. */
            float own_x,own_y;
            if (!nd_map_point(map,nd_data.lat,nd_data.lon,&own_x,&own_y)) continue;
            px=own_x+sinf(angle)*distance*map->radius/map->range_nm;
            py=own_y+cosf(angle)*distance*map->radius/map->range_nm;
        }
        if (!nd_map_visible(map,px,py,6)) continue;
        nd_diamond(map->cx+px,map->cy+py,3*map->sym,CYAN);
        snprintf(label,sizeof(label),"%+03d",(int)lroundf(relative/100));
        if (map->cx+px<map->label_r+10*map->sym) text_at(map->cx+px+4*map->sym,map->cy+py+3*map->sym,label,CYAN);
    }
}

static void nd_draw_radio(const EfisNdCanvas *map,const Side *side,int index,int mode)
{
    int receiver;
    for (receiver=0;receiver<2;++receiver) {
        const EfisNdVor *vor=&nd_data.vor[receiver];
        float left=map->x+(receiver?95.0f:5.0f);
        float footer_y=map->y+(mode==4?16.0f:0.0f);
        char label[32];
        if (!(receiver?efis[index].vor2:efis[index].vor1)) continue;
        snprintf(label,sizeof(label),"VOR%d %.5s",receiver+1,vor->valid?vor->id:"---");
        text_at(left,footer_y+18,label,vor->valid?CYAN:AMBER);
        if (vor->dme_valid) snprintf(label,sizeof(label),"%.1f NM",vor->distance_nm);
        else snprintf(label,sizeof(label),"%.2f",vor->frequency_mhz);
        text_at(left,footer_y+6,label,CYAN);
        if (side->heading_valid && vor->valid && mode!=4) {
            float angle=(vor->bearing_mag-side->heading_value)*(float)M_PI/180.0f;
            float px=sinf(angle)*(map->radius-12),py=cosf(angle)*(map->radius-12);
            nd_map_segment(map,-px,-py,px,py,CYAN);
            if (nd_map_visible(map,px,py,3)) nd_diamond(map->cx+px,map->cy+py,receiver?4:2,CYAN);
        }
    }
    if (mode==0 || mode==1) {
        const EfisNdVor *vor=&nd_data.vor[index];
        int valid=mode==0?side->loc_valid:vor->valid;
        float dots=mode==0?side->loc_dots:vor->deviation_dots;
        if (side->heading_valid && valid && isfinite(dots) && isfinite(vor->course_mag)) {
            float angle=(vor->course_mag-side->heading_value)*(float)M_PI/180.0f;
            float ux=sinf(angle),uy=cosf(angle),rx=cosf(angle),ry=-sinf(angle);
            float deflection=pfd_clamp(dots,-2.5f,2.5f)*9;
            int i;
            nd_map_segment(map,-ux*55,-uy*55,ux*55,uy*55,MAGENTA);
            nd_map_segment(map,rx*deflection-ux*24,ry*deflection-uy*24,
                           rx*deflection+ux*24,ry*deflection+uy*24,MAGENTA);
            for(i=-2;i<=2;++i) if(i) nd_diamond(map->cx+rx*i*9,map->cy+ry*i*9,1,WHITE);
        } else text_at(map->x+47,map->y+40,mode==0?"LS NO DATA":"VOR NO DATA",AMBER);
    }
}

static void draw_nd(const Side *side, float x, float y, const char *side_name)
{
    int i,index=side==&refs.side[1]?1:0;
    int position_valid=side->navigation_valid && nd_data.position_valid;
    char value[48];
    static const char *modes[]={"ROSE LS","ROSE VOR","ROSE NAV","ARC","PLAN"};
    int mode=side->nd_mode>=0&&side->nd_mode<=4?side->nd_mode:2;
    EfisNdCanvas map;
    float own_x=0,own_y=0;
    map.x=x; map.y=y; map.arc=mode==3;
    map.clip_l=x+5; map.clip_r=x+175; map.clip_b=y+32; map.clip_t=y+183; map.label_r=x+140; map.sym=1;
    map.cx=x+90; map.cy=y+(map.arc?75:104); map.radius=map.arc?96:76;
    map.range_nm=isfinite(side->nd_range_nm)&&side->nd_range_nm>0?side->nd_range_nm:80;
    map.heading=mode==4?0:nd_data.true_heading;
    map.lat=nd_data.lat; map.lon=nd_data.lon;
    if (mode==4 && !efis[index].zoom && nd_fms_plan_centre(&map.lat,&map.lon)) {
        /* PLAN centred on the native F-PLN */
    } else if (mode==4 && !efis[index].zoom && nd_data.route_displayed>=0 && nd_data.route_displayed<nd_data.route_count &&
        isfinite(nd_data.route[nd_data.route_displayed].lat) && isfinite(nd_data.route[nd_data.route_displayed].lon)) {
        map.lat=nd_data.route[nd_data.route_displayed].lat;
        map.lon=nd_data.route[nd_data.route_displayed].lon;
    }
    rectangle(x,y,x+180,y+234,(Color){.006f,.012f,.020f});
    /* An unpowered ND display unit is dark (A380 network); without it the legacy marker stays. */
    if (!SIDE_ND_VALID(side)) { if (!a380x_power.available) text_at(x+55,y+116,"ND OFF",AMBER); return; }
    if (position_valid && efis[index].zoom) nd_draw_airport(&map);
    if (position_valid && !efis[index].zoom) {
        nd_draw_terrain(&map,index);
        if (mode>=2) { nd_draw_route(&map,index); nd_draw_nav_layers(&map,index); }
    }
    if (position_valid) nd_draw_traffic(&map,index);
    set_color(WHITE); glBegin(GL_LINE_STRIP);
    for(i=0;i<=72;++i) {
        float degrees=map.arc?-50.0f+100.0f*i/72:-180.0f+360.0f*i/72;
        float a=degrees*(float)M_PI/180;
        glVertex2f(map.cx+sinf(a)*map.radius,map.cy+cosf(a)*map.radius);
    }
    glEnd();
    for(i=0;side->heading_valid && i<360;i+=10) {
        float relative=(float)i-(mode==4?0:side->heading_value);
        float a,inner;
        while(relative>180) relative-=360;
        while(relative<-180) relative+=360;
        if(map.arc&&fabsf(relative)>50) continue;
        a=relative*(float)M_PI/180; inner=map.radius-(i%30==0?8:4);
        line(map.cx+sinf(a)*inner,map.cy+cosf(a)*inner,
             map.cx+sinf(a)*map.radius,map.cy+cosf(a)*map.radius,WHITE);
        if(i%30==0) {
            float tx=map.cx+sinf(a)*(map.radius-15)-5,ty=map.cy+cosf(a)*(map.radius-15)-4;
            snprintf(value,sizeof(value),"%02d",i/10);
            if(tx>x+5&&tx<x+160&&ty>y+30) text_at(tx,ty,value,WHITE);
        }
    }
    nd_draw_radio(&map,side,index,mode);
    if(position_valid) nd_map_point(&map,nd_data.lat,nd_data.lon,&own_x,&own_y);
    if(position_valid && nd_map_visible(&map,own_x,own_y,4)) {
        float angle=(nd_data.true_heading-map.heading)*(float)M_PI/180;
        float hx=sinf(angle),hy=cosf(angle),rx=cosf(angle),ry=-sinf(angle);
        float sx=map.cx+own_x,sy=map.cy+own_y;
        line(sx-hx*5,sy-hy*5,sx+hx*8,sy+hy*8,YELLOW);
        line(sx-rx*6,sy-ry*6,sx+rx*6,sy+ry*6,YELLOW);
        if(side->heading_valid&&ground_speed_knots>1) {
            float a=(ground_track_degrees-map.heading)*(float)M_PI/180;
            nd_map_segment(&map,own_x,own_y,own_x+sinf(a)*map.radius*.6f,
                           own_y+cosf(a)*map.radius*.6f,GREEN);
        }
    }
    if(side->heading_valid) {
        snprintf(value,sizeof(value),"HDG %03d",((int)lroundf(side->heading_value)%360+360)%360);
        text_at(x+7,y+217,value,WHITE);
    } else text_at(x+7,y+217,"HDG DATA",AMBER);
    if(side->navigation_valid) snprintf(value,sizeof(value),"GS %.0f",ground_speed_knots);
    else snprintf(value,sizeof(value),"GS ---");
    text_at(x+124,y+217,value,side->navigation_valid?GREEN:AMBER);
    text_at(x+7,y+195,efis[index].zoom?"ZOOM":modes[mode],GREEN);
    snprintf(value,sizeof(value),"%.0f NM",map.range_nm); text_at(x+126,y+195,value,CYAN);
    if(efis[index].zoom) {
        if(!position_valid) text_at(x+49,y+113,"MAP NO DATA",AMBER);
        else if(nd_airport_available()) {
            text_at(x+7,y+183,efis_airport.id,CYAN);
            if(efis_airport.truncated) text_at(x+65,y+18,"MAP LIMIT",AMBER);
        }
        else text_at(x+25,y+113,efis_airport.loading?"AIRPORT MAP LOAD":"AIRPORT MAP N/A",AMBER);
    }
    else if(!position_valid) text_at(x+49,y+113,"MAP NO DATA",AMBER);
    else if(nd_data.route_count==0&&mode>=2&&!nd_fpln_plan_drawable(A380X_PLAN_ACTIVE)&&
            !nd_fpln_plan_drawable(A380X_PLAN_TEMPORARY)) text_at(x+8,y+183,"NO F-PLN",GREY);
    if(efis[index].wx_requested) text_at(x+116,y+183,"WX INOP",AMBER);
    else if(efis[index].terr && !efis[index].zoom) {
        /* TERRAIN on ND: highest / lowest elevation in hundreds of feet (FBW TERR box values). */
        char upper[16],lower[16];
        if(!nd_data.terrain[index].on) text_at(x+98,y+183,"TERR N/A",AMBER);
        else if(terr_threshold_text(&nd_data.terrain[index].shown,upper,lower)) {
            snprintf(value,sizeof(value),"TERR%s%s%s",upper,lower[0]?"/":"",lower);
            text_at(x+86,y+183,value,CYAN);
        } else text_at(x+120,y+183,"TERR",CYAN);
    }
    else if(efis[index].traf) text_at(x+129,y+183,nd_data.traffic_available?"TRAF":"TCAS N/A",CYAN);
    if(pfd_glare.chrono_mode[index]) {
        int seconds=pfd_glare.chrono_mode[index]==2?pfd_glare.chrono_frozen[index]:pfd_glare.chrono_seconds[index];
        snprintf(value,sizeof(value),"CHR %02d:%02d",seconds/60,seconds%60);
        text_at(x+105,y+30,value,WHITE);
    }
    if(mode==4) {
        text_at(x+5,y+4,"PREV",CYAN);text_at(x+149,y+4,"NEXT",CYAN);
    } else if(!efis[index].vor1&&!efis[index].vor2) text_at(x+7,y+2,side_name,GREY);
}
/* FlyByWire-layout native ND for the native cockpit devices (a380x_native_displays.inc). */
#if !defined(A380X_XPKIT)
#include "a380x_nd_fbw.inc"
#endif
