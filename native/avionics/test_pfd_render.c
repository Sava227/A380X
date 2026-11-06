/* SPDX-License-Identifier: GPL-3.0-or-later
 * Headless rendering regression harness. Exercises the actual panel renderer;
 * XPLM/OpenGL calls capture draw commands without a running simulator.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "a380x_avionics_core.h"
#include "a380x_pfd_data.h"
#include "a380x_guidance.h"
#include "a380x_efis.h"
#include "a380x_efis_nd.h"
#include "a380x_efis_airport.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define ARRAY_LEN(a) ((int)(sizeof(a)/sizeof((a)[0])))
#define GL_QUADS 1
#define GL_LINES 2
#define GL_TRIANGLE_FAN 3
#define GL_LINE_STRIP 4
#define xplmFont_Proportional 0

typedef struct { float r,g,b; } Color;
static const Color WHITE={.92f,.96f,1}, GREEN={.24f,.95f,.36f}, CYAN={.10f,.88f,.96f};
static const Color AMBER={1,.65f,0}, RED={1,.12f,.08f}, YELLOW={1,.82f,0};
static const Color GREY={.48f,.55f,.60f}, BLACK={.004f,.008f,.012f};
static const Color TAPE_BACKGROUND={79.f/255,84.f/255,103.f/255}, SKY={0,91.f/255,196.f/255}, EARTH={101.f/255,41.f/255,10.f/255};
typedef struct {
    float ias_value,altitude_value,vertical_speed_value,pitch_value,roll_value;
    float heading_value,radio_altitude_value,mach_value,ias_trend_value,baro_hpa_value,decision_height_value;
    float nd_range_nm;
    int nd_mode;
    float loc_dots,gs_dots,nav_frequency_mhz,nav_dme_nm;
    char nav_ident[16];
    int nav_dme_valid;
    int navigation_valid;
    int air_valid,altitude_valid,vertical_speed_valid,attitude_valid,heading_valid;
    int radio_altitude_valid,display_valid,ias_trend_visible,baro_std,loc_valid,gs_valid,ls_visible;
} Side;
static struct { Side side[2]; } refs;
static struct { int flight_phase,fma_a1a2,fma_a3,fma_b1,fma_c1,fma_d1d2; } published;
static A380XPfdGuidance pfd_guidance;
static A380XGuidanceState guidance;
static EfisSide efis[2];
static EfisNdData nd_data;
static EfisAirport efis_airport;
static A380XGuidanceInputs guidance_inputs;
static float ground_speed_knots,ground_track_degrees;

typedef struct { float x,y; Color color; } CapturedVertex;
typedef struct { float x,y; char value[80]; Color color; } CapturedText;
static CapturedVertex vertices[100000];
static CapturedText labels[2000];
static int vertex_count,label_count,primitive,primitive_start;
static Color current_color;
static float line_width=1.0f,clock_seconds=1.0f;
static FILE *svg;
static float svg_y_origin=448;

/* Translate/rotate stack for the FBW ND compass labels (no scaling is used in these tests). */
static float xf_tx,xf_ty,xf_rot,xf_stack[8][3];
static int xf_depth;
static void glPushMatrix(void) { assert(xf_depth<8); xf_stack[xf_depth][0]=xf_tx;xf_stack[xf_depth][1]=xf_ty;xf_stack[xf_depth++][2]=xf_rot; }
static void glPopMatrix(void) { assert(xf_depth>0); --xf_depth; xf_tx=xf_stack[xf_depth][0];xf_ty=xf_stack[xf_depth][1];xf_rot=xf_stack[xf_depth][2]; }
static void glTranslatef(float x,float y,float z) {
    float a=xf_rot*(float)M_PI/180; (void)z;
    xf_tx+=x*cosf(a)-y*sinf(a); xf_ty+=x*sinf(a)+y*cosf(a);
}
static void glRotatef(float deg,float x,float y,float z) { (void)x;(void)y;assert(z==1.0f); xf_rot+=deg; }
static void xf_apply(float *x,float *y) {
    float a=xf_rot*(float)M_PI/180,px=*x,py=*y;
    *x=xf_tx+px*cosf(a)-py*sinf(a); *y=xf_ty+px*sinf(a)+py*cosf(a);
}
static void glColor3f(float r,float g,float b) { current_color=(Color){r,g,b}; }
static void glLineWidth(float width) { line_width=width; }
static void glBegin(int kind) { primitive=kind; primitive_start=vertex_count; }
/* textured-quad capture (TERRAIN on ND), set by glTexCoord2f below */
static float mock_tex_u[4],mock_tex_v[4],mock_quad_x[4],mock_quad_y[4];
static int mock_quad_vertex=-1;
static void glVertex2f(float x,float y) {
    assert(isfinite(x) && isfinite(y));
    if(mock_quad_vertex>=0) { mock_quad_x[mock_quad_vertex]=x; mock_quad_y[mock_quad_vertex]=y; if(mock_quad_vertex==3) mock_quad_vertex=-1; }
    assert(vertex_count<ARRAY_LEN(vertices));
    xf_apply(&x,&y);
    vertices[vertex_count++]=(CapturedVertex){x,y,current_color};
}
static void svg_color(Color c) { fprintf(svg,"rgb(%d,%d,%d)",(int)(255*c.r),(int)(255*c.g),(int)(255*c.b)); }
static void glEnd(void) {
    int i;
    if (!svg) return;
    if (primitive==GL_LINES) {
        for (i=primitive_start;i+1<vertex_count;i+=2) {
            fprintf(svg,"<path d=\"M%.3f %.3fL%.3f %.3f\" fill=\"none\" stroke=\"",vertices[i].x,svg_y_origin-vertices[i].y,vertices[i+1].x,svg_y_origin-vertices[i+1].y);
            svg_color(current_color);fprintf(svg,"\" stroke-width=\"%.2f\"/>\n",line_width);
        }
    } else {
        fprintf(svg,primitive==GL_LINE_STRIP ? "<polyline points=\"" : "<polygon points=\"");
        for(i=primitive_start;i<vertex_count;++i) fprintf(svg,"%.3f,%.3f ",vertices[i].x,svg_y_origin-vertices[i].y);
        fprintf(svg,"\" fill=\"");
        if (primitive==GL_LINE_STRIP) fprintf(svg,"none\" stroke=\"");
        svg_color(current_color);fprintf(svg,"\"/>\n");
    }
}
static void XPLMSetGraphicsState(int a,int b,int c,int d,int e,int f,int g) { (void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g; }
/* Texture mock for the TERRAIN on ND image (a380x_terrain_render.inc): RGBA storage per texture,
 * textured quads recorded with their bounds and texture coordinates. */
typedef unsigned int GLuint;
#define GL_TEXTURE_2D 0x0DE1
#define GL_TEXTURE_MIN_FILTER 0x2801
#define GL_TEXTURE_MAG_FILTER 0x2800
#define GL_TEXTURE_WRAP_S 0x2802
#define GL_TEXTURE_WRAP_T 0x2803
#define GL_NEAREST 0x2600
#define GL_CLAMP_TO_EDGE 0x812F
#define GL_RGBA 0x1908
#define GL_UNSIGNED_BYTE 0x1401
static struct { int width,height; unsigned char *rgba; } mock_textures[8];
static int mock_texture_count,mock_bound,mock_textured_quads,mock_sub_uploads;
static void XPLMGenerateTextureNumbers(int *ids,int count) { int i;for(i=0;i<count;++i) ids[i]=++mock_texture_count; assert(mock_texture_count<8); }
static void XPLMBindTexture2d(int id,int unit) { (void)unit; mock_bound=id; }
static void glTexParameteri(int target,int name,int value) { (void)target;(void)name;(void)value; }
static void glTexImage2D(int target,int level,int internal,int width,int height,int border,int format,int type,const void *data) {
    (void)target;(void)level;(void)internal;(void)border;(void)format;(void)type;
    assert(mock_bound>0 && !data);
    free(mock_textures[mock_bound].rgba);
    mock_textures[mock_bound].rgba=calloc((size_t)width*height,4);
    mock_textures[mock_bound].width=width;mock_textures[mock_bound].height=height;
}
static void glTexSubImage2D(int target,int level,int x,int y,int width,int height,int format,int type,const void *data) {
    int row;(void)target;(void)level;(void)format;(void)type;
    assert(mock_bound>0 && x==0 && width==mock_textures[mock_bound].width && y+height<=mock_textures[mock_bound].height);
    for(row=0;row<height;++row) memcpy(mock_textures[mock_bound].rgba+((size_t)(y+row)*width)*4,(const unsigned char *)data+(size_t)row*width*4,(size_t)width*4);
    ++mock_sub_uploads;
}
static void glDeleteTextures(int count,const GLuint *ids) { (void)count;(void)ids; }
static void glColor4f(float r,float g,float b,float a) { (void)a; current_color=(Color){r,g,b}; }
static void glTexCoord2f(float u,float v) {
    mock_quad_vertex=(mock_quad_vertex+1)%4; mock_tex_u[mock_quad_vertex]=u; mock_tex_v[mock_quad_vertex]=v;
    if(mock_quad_vertex==3) ++mock_textured_quads;
}
static float XPLMGetElapsedTime(void) { return clock_seconds; }
static void XPLMDrawString(float *rgb,int x,int y,const char *text,void *wrap,int font) {
    Color c={rgb[0],rgb[1],rgb[2]}; (void)wrap;(void)font;
    assert(label_count<ARRAY_LEN(labels));
    labels[label_count].x=(float)x;labels[label_count].y=(float)y;labels[label_count].color=c;
    snprintf(labels[label_count++].value,80,"%s",text);
    if(svg) {
        fprintf(svg,"<text x=\"%d\" y=\"%d\" font-family=\"monospace\" font-size=\"10\" fill=\"",x,(int)svg_y_origin-y);
        svg_color(c);fprintf(svg,"\">%s</text>\n",text);
    }
}
/* Vertical clip window of the real atlas font: text entirely outside it is not drawn. */
static float clip_y0,clip_y1;
static void ewd_font_clip_y(float y0,float y1) { clip_y0=y0; clip_y1=y1; }
static void ewd_font_draw(float x,float y,const char *text,float size,Color color,float tracking,int anchor) {
    float width=(float)strlen(text)*(size*.6f+tracking);
    if (clip_y0<clip_y1 && (y+size*.72f<=clip_y0 || y>=clip_y1)) return;
    assert(label_count<ARRAY_LEN(labels));
    xf_apply(&x,&y);
    labels[label_count].x=x-width*(anchor==2 ? 1.0f : anchor==1 ? .5f : 0);
    labels[label_count].y=y;labels[label_count].color=color;
    snprintf(labels[label_count++].value,80,"%s",text);
    if(svg) {
        fprintf(svg,"<text x=\"%.3f\" y=\"%.3f\" transform=\"rotate(%.2f %.3f %.3f)\" font-family=\"EIS\" font-size=\"%.3f\" text-anchor=\"%s\" fill=\"",
                x,svg_y_origin-y,-xf_rot,x,svg_y_origin-y,size,anchor==2 ? "end" : anchor==1 ? "middle" : "start");
        svg_color(color);fprintf(svg,"\">%s</text>\n",text);
    }
}
static struct {
    int rto_arm, gear_not_down_warning;
    int autobrake_armed, autobrake_active;
    int gear_not_locked_down_warning, gear_not_locked_up_warning;
} center_right;
/* Minimal native flight-plan store for the ND route tests: plan 0 ACTIVE, 1 TMPY, straight legs
 * between consecutive termination points, the same rules as a380x_fms_fpln.c for disco/missed. */
#include "a380x_fms_fpln.h"
static A380XFlightPlans *fmsa_fpln;
static A380XFplnPlan test_plans[2];
const A380XFplnPlan *a380x_fpln_plan(const A380XFlightPlans *fp, int plan_index)
{
    (void)fp;
    return plan_index == 0 || plan_index == 1 ? &test_plans[plan_index] : NULL;
}
int a380x_fpln_is_disco(const A380XFplnLeg *l) { return (l->flags & A380X_LEGF_DISCONTINUITY) != 0; }
int a380x_fpln_from_leg_index(const A380XFplnPlan *p)
{
    int i;
    for (i = p->active_leg_index - 1; i >= 0; --i) if (!a380x_fpln_is_disco(&p->pri.el[i])) return i;
    return -1;
}
A380XFplnAltCstr a380x_fpln_eff_alt(const A380XFplnLeg *l) { return l->alt_pilot.desc ? l->alt_pilot : l->alt_db; }
int a380x_fpln_leg_path(const A380XFlightPlans *fp, int plan, int altn, int idx, A380XFplnLegPath *out)
{
    const A380XFplnRoute *r = &test_plans[plan].pri;
    const A380XFplnLeg *l = &r->el[idx], *prev = idx > 0 ? &r->el[idx - 1] : NULL;
    (void)fp; (void)altn;
    if (a380x_fpln_is_disco(l) || !prev || a380x_fpln_is_disco(prev)) return 0;
    memset(out, 0, sizeof(*out));
    out->kind = A380X_PATH_LINE;
    out->start_lat = prev->term_lat; out->start_lon = prev->term_lon;
    out->end_lat = l->term_lat; out->end_lon = l->term_lon;
    out->is_missed = idx >= r->first_missed_appr_index;
    out->is_tmpy = plan == 1;
    return 1;
}
#include "a380x_pfd_render.inc"
/* FCU LCD text: capture each string (the GL atlas path is a380x_fcu_font.inc,
 * covered pixel-for-pixel by test_fcu_render.c). */
#include "a380x_fcu_font.h"
static int captured_face[2000];
static void fcu_text_draw(int face,float x,float y,const char *text,Color color) {
    assert(label_count<ARRAY_LEN(labels));
    captured_face[label_count]=face;
    labels[label_count].x=x;labels[label_count].y=y;labels[label_count].color=color;
    snprintf(labels[label_count++].value,80,"%s",text);
    if(svg) {
        fprintf(svg,"<text x=\"%.1f\" y=\"%.1f\" font-family=\"%s\" font-size=\"%.0f\" fill=\"",
                x,svg_y_origin-y,face==2?"Poppins":"Digital",fcu_face_size[face]);
        svg_color(color);fprintf(svg,"\">%s</text>\n",text);
    }
}
#include "a380x_fcu_render.inc"
#include "a380x_efis_render.inc"

static void clear_capture(void) { vertex_count=label_count=0; }
static int has_label(const char *wanted) {
    int i;for(i=0;i<label_count;++i) if(!strcmp(labels[i].value,wanted)) return 1;
    return 0;
}
static int has_vertex(float x,float y,Color color) {
    int i;for(i=0;i<vertex_count;++i)
        if(fabsf(vertices[i].x-x)<.02f && fabsf(vertices[i].y-y)<.02f &&
           fabsf(vertices[i].color.r-color.r)<.01f && fabsf(vertices[i].color.g-color.g)<.01f) return 1;
    return 0;
}
static int count_color(Color color) {
    int i,n=0;for(i=0;i<vertex_count;++i)
        if(fabsf(vertices[i].color.r-color.r)<.01f && fabsf(vertices[i].color.g-color.g)<.01f) ++n;
    return n;
}
static void normal_state(void) {
    Side side={0};
    memset(efis,0,sizeof(efis));
    memset(&nd_data,0,sizeof(nd_data));
    memset(&efis_airport,0,sizeof(efis_airport));
    side.ias_value=270;side.altitude_value=12440;side.vertical_speed_value=1600;side.pitch_value=6;side.roll_value=15;
    side.nd_mode=3;side.nd_range_nm=80;
    side.heading_value=359;side.radio_altitude_value=7500;side.mach_value=.52f;side.baro_hpa_value=1013;
    side.decision_height_value=-1;
    side.navigation_valid=1;
    side.air_valid=side.altitude_valid=side.vertical_speed_valid=side.attitude_valid=side.heading_valid=1;
    side.radio_altitude_valid=side.display_valid=1;side.ias_trend_value=9;side.ias_trend_visible=1;
    refs.side[0]=refs.side[1]=side;ground_speed_knots=320;ground_track_degrees=2;
    memset(&pfd_guidance,0,sizeof(pfd_guidance));
    memset(pfd_display_state,0,sizeof(pfd_display_state));
    pfd_guidance.selected_speed_knots=285;pfd_guidance.selected_heading_deg=15;pfd_guidance.selected_altitude_ft=18000;
    pfd_guidance.selected_vs_fpm=1800;pfd_guidance.fd1=pfd_guidance.fd2=pfd_guidance.ap1=1;
    pfd_guidance.athr_armed=pfd_guidance.athr_active=pfd_guidance.altitude_armed=pfd_guidance.guidance_valid=1;
    pfd_guidance.flap_valid=pfd_guidance.slat_valid=pfd_guidance.speedbrake_valid=1;
    pfd_guidance.fd_roll_error_deg=10;pfd_guidance.fd_pitch_error_deg=5;
    published.flight_phase=8;published.fma_a1a2=A380X_A1A2_THR_CLB;published.fma_b1=A380X_B1_OP_CLB;
    published.fma_c1=A380X_C1_HDG;published.fma_d1d2=0;
    clear_capture();
}
static void export_svg(const char *path) {
    svg=fopen(path,"w");assert(svg);svg_y_origin=32.0f+PFD_PIXEL_HEIGHT;
    fprintf(svg,"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%.0f\" height=\"%.0f\" viewBox=\"0 0 %.0f %.0f\">\n",
            2*PFD_PIXEL_WIDTH,2*PFD_PIXEL_HEIGHT,PFD_PIXEL_WIDTH,PFD_PIXEL_HEIGHT);
    fprintf(svg,"<style>@font-face{font-family:EIS;src:url('PFD_FONT_DATA_URI')}</style>\n");
    draw_pfd(&refs.side[0],0,32,"CAPT");
    fprintf(svg,"</svg>\n");fclose(svg);svg=NULL;svg_y_origin=448;
}
static void export_fcu_svg(const char *path) {
    svg=fopen(path,"w");assert(svg);svg_y_origin=FCU_GLASS_Y+FCU_GLASS_H;
    fprintf(svg,"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"2048\" height=\"212\" viewBox=\"0 0 2048 212\">\n");
    clear_capture();draw_fcu();
    fprintf(svg,"</svg>\n");fclose(svg);svg=NULL;svg_y_origin=448;
}
static const CapturedText *fcu_text(const char *wanted) {
    int i;for(i=0;i<label_count;++i) if(!strcmp(labels[i].value,wanted)) return &labels[i];
    return NULL;
}
static int same_color(Color a,Color b) { return fabsf(a.r-b.r)<.002f&&fabsf(a.g-b.g)<.002f&&fabsf(a.b-b.b)<.002f; }
static void test_fcu(void) {
    const CapturedText *t;
    guidance_inputs.powered=0;clear_capture();draw_fcu();assert(label_count==0&&vertex_count==8);
    memset(&guidance,0,sizeof(guidance));guidance_inputs.powered=1;
    guidance.selected_speed_knots=250;guidance.selected_heading_deg=359;
    guidance.selected_altitude_ft=12000;guidance.selected_vs_fpm=-1800;
    guidance_inputs.ias_knots=320; /* Actual speed must not replace selected speed. */
    FcuGlass v=fcu_glass_values(&guidance);
    assert(!strcmp(v.value[0],"250")&&!strcmp(v.value[1],"359"));
    /* FBW VsDisplay: sign, two hundreds digits, then two small zeros. */
    assert(!strcmp(v.value[2],"12000")&&!strcmp(v.value[3],"-18oo"));
    assert(!strcmp(v.title[0],"SPD")&&!strcmp(v.title[3],"V/S")&&v.knots&&v.degrees);
    clear_capture();draw_fcu();
    /* Two full-width LCD backgrounds in the 1024 x 212 slices, FBW reference colour. */
    for(int screen=0;screen<2;++screen) {
        assert(vertices[screen*4].x==FCU_GLASS_X+FCU_LCD_W*screen);
        for(int corner=0;corner<4;++corner) assert(same_color(vertices[screen*4+corner].color,FCU_LCD_BACKGROUND));
    }
    for(int i=0;i<label_count;++i) {
        assert(labels[i].x>=FCU_GLASS_X&&labels[i].x+fcu_text_width(captured_face[i],labels[i].value)<=FCU_GLASS_X+FCU_GLASS_W);
        assert(labels[i].y>FCU_GLASS_Y&&labels[i].y<FCU_GLASS_Y+FCU_GLASS_H);
    }
    t=fcu_text("250");assert(t&&same_color(t->color,FCU_LCD_INK)&&t->x<FCU_LCD_W);
    t=fcu_text("SPD");assert(t&&same_color(t->color,FCU_LCD_INK));
    t=fcu_text("MACH");assert(t&&same_color(t->color,FCU_LCD_GHOST));          /* ghosted legend */
    assert(t->x+fcu_text_width(FCU_FACE_LABEL,"MACH")<fcu_text("SPD")->x);  /* MACH left of SPD */
    t=fcu_text("TRK");assert(t&&same_color(t->color,FCU_LCD_GHOST)&&t->x>fcu_text("HDG")->x);
    t=fcu_text("FPA");assert(t&&same_color(t->color,FCU_LCD_GHOST)&&t->x>fcu_text("V/S")->x);
    assert(fcu_text("KT")&&fcu_text("\xb0")&&!fcu_text("TRUE"));
    t=fcu_text("12000");assert(t&&t->x>=FCU_LCD_W);
    /* Managed targets: dashes, no units; the ALT window never shows dashes. */
    guidance.speed_dashes=guidance.heading_dashes=guidance.vs_dashes=1;
    guidance.speed_managed=guidance.heading_managed=guidance.altitude_managed=1;
    v=fcu_glass_values(&guidance);
    assert(!strcmp(v.value[0],"---")&&!strcmp(v.value[1],"---")&&!strcmp(v.value[2],"12000")&&!strcmp(v.value[3],"~----"));
    clear_capture();draw_fcu();assert(!fcu_text("KT")&&!fcu_text("\xb0"));
    guidance.speed_dashes=guidance.heading_dashes=guidance.vs_dashes=0;
    guidance.speed_managed=guidance.heading_managed=guidance.altitude_managed=0;
    guidance.mach_mode=guidance.trk_fpa_mode=guidance.true_heading=1;
    guidance.selected_mach=.78;guidance.selected_fpa_deg=-3.2;guidance.selected_heading_deg=359.8;
    v=fcu_glass_values(&guidance);
    assert(!strcmp(v.value[0],"0.78")&&!strcmp(v.value[1],"000")&&!strcmp(v.value[3],"~-3.2"));
    assert(!strcmp(v.title[0],"MACH")&&!strcmp(v.title[1],"TRK")&&!strcmp(v.title[3],"FPA"));
    assert(v.true_reference&&!v.knots);
    clear_capture();draw_fcu();
    assert(same_color(fcu_text("MACH")->color,FCU_LCD_INK)&&same_color(fcu_text("SPD")->color,FCU_LCD_GHOST));
    assert(same_color(fcu_text("TRK")->color,FCU_LCD_INK)&&same_color(fcu_text("HDG")->color,FCU_LCD_GHOST));
    assert(same_color(fcu_text("FPA")->color,FCU_LCD_INK)&&fcu_text("TRUE"));
    guidance.selected_fpa_deg=0;v=fcu_glass_values(&guidance);assert(!strcmp(v.value[3],"~+0.0"));
    guidance.trk_fpa_mode=0;guidance.selected_vs_fpm=0;v=fcu_glass_values(&guidance);assert(!strcmp(v.value[3],"+00oo"));
    guidance.selected_vs_fpm=6000;v=fcu_glass_values(&guidance);assert(!strcmp(v.value[3],"+60oo"));
    guidance.selected_altitude_ft=100;v=fcu_glass_values(&guidance);assert(!strcmp(v.value[2],"00100"));
    /* Every drawn character exists in the embedded atlas. */
    for(const char *p="0123456789-+.~o\xb0";*p;++p)assert(fcu_glyph_find(FCU_FACE_VALUE,(unsigned char)*p));
    for(const char *p="0123456789-+.~o";*p;++p)assert(fcu_glyph_find(FCU_FACE_VS,(unsigned char)*p));
    for(const char *p="MACHSPDTRKALV/FUEGT";*p;++p)assert(fcu_glyph_find(FCU_FACE_LABEL,(unsigned char)*p));
    fcu_lcd_level=0;clear_capture();draw_fcu();assert(vertex_count==8&&label_count==0);
    fcu_lcd_level=.25f;clear_capture();draw_fcu();
    assert(vertices[0].color.b<=.25f&&labels[0].color.r<=.25f);
    fcu_lcd_level=1;
}
static const CapturedText *find_label(const char *wanted) {
    int i;
    for(i=0;i<label_count;++i) if(!strcmp(labels[i].value,wanted)) return &labels[i];
    return NULL;
}
static void nd_test_state(void) {
    normal_state();
    refs.side[0].nd_mode=refs.side[1].nd_mode=2;
    refs.side[0].heading_value=refs.side[1].heading_value=0;
    nd_data.position_valid=1; nd_data.lat=nd_data.lon=0;
    nd_data.true_heading=0;nd_data.altitude_ft=5000;
    ground_speed_knots=0;
}
static void nd_test_airport(void) {
    efis_airport.found=efis_airport.complete=1;
    snprintf(efis_airport.id,sizeof(efis_airport.id),"TEST");
    efis_airport.runway_count=1;
    efis_airport.runways[0].lat1=-.004;efis_airport.runways[0].lon1=.002;
    efis_airport.runways[0].lat2=.010;efis_airport.runways[0].lon2=.002;
    efis_airport.runways[0].width_m=45;
    snprintf(efis_airport.runways[0].end1,sizeof(efis_airport.runways[0].end1),"34");
    snprintf(efis_airport.runways[0].end2,sizeof(efis_airport.runways[0].end2),"16");
    efis_airport.ring_count=1;efis_airport.node_count=4;
    efis_airport.rings[0].first=0;efis_airport.rings[0].count=4;
    efis_airport.nodes[0].lat=-.008;efis_airport.nodes[0].lon=-.005;
    efis_airport.nodes[1].lat=.010;efis_airport.nodes[1].lon=-.005;
    efis_airport.nodes[2].lat=.010;efis_airport.nodes[2].lon=-.002;
    efis_airport.nodes[3].lat=-.008;efis_airport.nodes[3].lon=-.002;
}
static void test_nd(void) {
    static const char *ids[]={"APT001","VOR001","NDB001","FIX001"};
    int *flags[]={&efis[0].arpt,&efis[0].vord,&efis[0].ndb,&efis[0].wpt};
    int i,j;
    const CapturedText *label;
    float nav_x,nav_y,plan_x,plan_y;
    nd_test_state();
    for(i=0;i<4;++i) {
        nd_data.nav_count[i]=1;
        nd_data.nav[i][0].lat=.25;nd_data.nav[i][0].lon=.25;
        snprintf(nd_data.nav[i][0].id,sizeof(nd_data.nav[i][0].id),"%s",ids[i]);
    }
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");
    for(i=0;i<4;++i) assert(!has_label(ids[i]));
    for(i=0;i<4;++i) {
        *flags[i]=1;
        clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");
        for(j=0;j<4;++j) assert(has_label(ids[j])==(i==j));
        clear_capture();draw_nd(&refs.side[1],0,0,"F/O");
        for(j=0;j<4;++j) assert(!has_label(ids[j]));
        *flags[i]=0;
    }
    /* Route geometry is always displayed in NAV; the CSTR control gates only
     * its sourced waypoint altitude, independently on each side. */
    nd_data.route_count=1;nd_data.route_active=0;
    nd_data.route[0]=(EfisNdRoutePoint){.4,.15,9000,"ROUTE1"};
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");
    assert(has_label("ROUTE1")&&!has_label("9000"));
    efis[0].cstr=1;
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");
    assert(has_label("9000"));
    clear_capture();draw_nd(&refs.side[1],0,0,"F/O");
    assert(has_label("ROUTE1")&&!has_label("9000"));
    /* A northward fix is left on an east-heading NAV display, but directly
     * above the centre on north-up PLAN regardless of aircraft heading. */
    nd_test_state();nd_data.nav_count[3]=1;efis[0].wpt=1;
    nd_data.nav[3][0]=(EfisNdNavPoint){.25,0,15,"NORTH"};
    refs.side[0].heading_value=nd_data.true_heading=90;
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");
    label=find_label("NORTH");assert(label);nav_x=label->x;nav_y=label->y;
    refs.side[0].nd_mode=4;
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");
    label=find_label("NORTH");assert(label);plan_x=label->x;plan_y=label->y;
    assert(nav_x<90&&fabsf(nav_y-107)<2);
    assert(fabsf(plan_x-94)<2&&plan_y>115);
    assert(has_label("PREV")&&has_label("NEXT"));
    /* The selected FMS point, rather than ownship, centres PLAN. */
    nd_data.route_count=2;nd_data.route_displayed=1;
    nd_data.route[0]=(EfisNdRoutePoint){-.25,0,0,"SOUTH"};
    nd_data.route[1]=(EfisNdRoutePoint){.25,0,0,"CENTRE"};
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");
    label=find_label("CENTRE");assert(label&&label->x==94&&label->y==107);
    /* Traffic is not invented when source refs or slots are absent. An actual
     * nonzero TCAS slot appears only while TRAF is selected on that side. */
    nd_test_state();efis[0].traf=1;
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");
    assert(has_label("TCAS N/A")&&!has_label("+00"));
    nd_data.traffic_available=1;
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");
    assert(has_label("TRAF")&&!has_label("+00"));
    nd_data.traffic_distance_m[3]=18520;nd_data.traffic_bearing[3]=20;
    nd_data.traffic_altitude_m[3]=304.8f;
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");assert(has_label("+10"));
    clear_capture();draw_nd(&refs.side[1],0,0,"F/O");assert(!has_label("+10"));
    /* TERRAIN on ND (a380x_terrain_render.inc): nothing until the side's raster is on; then one
     * textured quad at the raster's place on this map (legacy 180 x 234 ROSE: radius 76 = 80 NM), the
     * pattern pixels uploaded row by row, the FBW TERR box values in the status line. */
    nd_test_state();efis[0].terr=1;
    clear_capture();mock_textured_quads=0;draw_nd(&refs.side[0],0,0,"CAPT");
    assert(has_label("TERR N/A")&&mock_textured_quads==0);
    terr_display_reset(&nd_data.terrain[0]);
    nd_data.terrain[0].on=1;assert(terr_geometry(2,80,&nd_data.terrain[0].geo));
    nd_data.terrain[0].cls[20][42]=TERR_RED_HIGH;nd_data.terrain[0].cls[20][43]=TERR_GREEN_SOLID;
    nd_data.terrain[0].shown=(TerrThresholds){3000,TERR_LEVEL_GREEN,9100,TERR_LEVEL_RED,0};
    clear_capture();mock_textured_quads=mock_sub_uploads=0;draw_nd(&refs.side[0],0,0,"CAPT");
    assert(has_label("TERR 92/ 30")&&mock_textured_quads==1&&mock_sub_uploads==nd_data.terrain[0].geo.rows);
    {   /* the image's aircraft reference (384, 384) at the legacy ROSE centre, 76/80 units per NM */
        const float k=(76.0f/80.0f)/nd_data.terrain[0].geo.px_per_nm;
        const unsigned char *rgba=mock_textures[mock_bound].rgba;
        const int w=mock_textures[mock_bound].width;
        assert(fabsf(mock_quad_x[0]-fmaxf(90+(45-384)*k,5))<.01f && fabsf(mock_quad_y[0]-fminf(104+(384-128)*k,183))<.01f);
        assert(w==85*8 && mock_textures[mock_bound].height==32*8);
        /* row 20 col 43 solid green, col 42 high-density red: pixel (336, 160) tile (0, 4) on */
        assert(rgba[((size_t)(20*8)*w+43*8)*4+1]==255&&rgba[((size_t)(20*8)*w+43*8)*4+3]==255);
        assert(rgba[((size_t)(20*8)*w+42*8)*4+0]==255&&rgba[((size_t)(20*8)*w+42*8)*4+1]==0);
    }
    clear_capture();mock_sub_uploads=0;draw_nd(&refs.side[0],0,0,"CAPT");assert(mock_sub_uploads==0);   /* unchanged rows */
    nd_data.terrain[0].cls[5][5]=TERR_WATER;
    clear_capture();mock_sub_uploads=0;draw_nd(&refs.side[0],0,0,"CAPT");assert(mock_sub_uploads==1);   /* one changed row */
    clear_capture();mock_textured_quads=0;draw_nd(&refs.side[1],0,0,"F/O");
    assert(!has_label("TERR 92/ 30")&&mock_textured_quads==0);
    /* Pointer elevation readout (simulator aid) beside the pointer, flipped left near the right edge. */
    memset(&nd_data.terrain_pointer[0],0,sizeof(nd_data.terrain_pointer[0]));
    clear_capture();terrain_draw_overlay(0);assert(label_count==0&&vertex_count==0);
    nd_data.terrain_pointer[0].inside=nd_data.terrain_pointer[0].shown=1;
    nd_data.terrain_pointer[0].x=400;nd_data.terrain_pointer[0].y=300;
    snprintf(nd_data.terrain_pointer[0].text,sizeof(nd_data.terrain_pointer[0].text),"SIM ELEV 4250 FT");
    clear_capture();terrain_draw_overlay(0);
    label=find_label("SIM ELEV 4250 FT");
    assert(label&&label->x>400&&label->y<1024-300&&count_color((Color){.10f,.11f,.13f})==4);
    nd_data.terrain_pointer[0].x=740;
    clear_capture();terrain_draw_overlay(0);
    label=find_label("SIM ELEV 4250 FT");assert(label&&label->x<740);
    terrain_gl_shutdown();assert(!terrain_gl[0].texture);
    efis[0].zoom=1;clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");
    assert(has_label("AIRPORT MAP N/A")&&count_color((Color){.44f,.25f,.01f})==0);
    nd_test_state();efis[0].zoom=1;refs.side[0].nd_range_nm=1;
    efis_airport.loading=1;
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");assert(has_label("AIRPORT MAP LOAD"));
    efis_airport.loading=0;nd_test_airport();
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");
    assert(has_label("TEST")&&has_label("34")&&has_label("16")&&!has_label("AIRPORT MAP N/A"));
    assert(count_color((Color){.26f,.30f,.34f})>=4); /* actual runway width */
    assert(count_color((Color){.16f,.21f,.26f})>0); /* sourced pavement boundaries */
    label=find_label("34");assert(label);nav_y=label->y;
    nd_data.lat=.002;
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");
    label=find_label("34");assert(label&&label->y<nav_y-5); /* moving ownship */
    clear_capture();draw_nd(&refs.side[1],0,0,"F/O");assert(!has_label("TEST"));
    efis_airport.truncated=1;
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");assert(has_label("MAP LIMIT"));
    nd_test_state();efis[0].wx_requested=1;
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");assert(has_label("WX INOP"));
    refs.side[0].navigation_valid=0;
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");
    assert(has_label("MAP NO DATA") && has_label("GS ---"));
    clear_capture();draw_nd(&refs.side[1],0,0,"F/O");
    assert(!has_label("MAP NO DATA") && !has_label("GS ---"));
    refs.side[0].navigation_valid=1;
    nd_data.position_valid=0;
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");assert(has_label("MAP NO DATA"));
    refs.side[0].display_valid=0;
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");assert(label_count==1&&has_label("ND OFF"));
}
static void export_nd_svg(const char *path) {
    int i;
    static const char *ids[]={"KAAA","SEA","ABC","SPARK"};
    nd_test_state();
    refs.side[0].nd_mode=3;refs.side[0].nd_range_nm=40;
    refs.side[1].nd_mode=4;refs.side[1].nd_range_nm=40;
    efis[0].arpt=efis[0].vord=efis[0].ndb=efis[0].wpt=efis[0].cstr=efis[0].traf=1;
    efis[1].wpt=efis[1].cstr=1;
    nd_data.route_count=3;nd_data.route_active=1;nd_data.route_displayed=1;
    nd_data.route[0]=(EfisNdRoutePoint){0,0,0,"ORIGIN"};
    nd_data.route[1]=(EfisNdRoutePoint){.2,.04,5000,"DRAKE"};
    nd_data.route[2]=(EfisNdRoutePoint){.48,-.1,9000,"NORTH"};
    for(i=0;i<4;++i) {
        nd_data.nav_count[i]=1;
        nd_data.nav[i][0].lat=.1+.125*i;nd_data.nav[i][0].lon=(i%2?.18:-.16);
        snprintf(nd_data.nav[i][0].id,sizeof(nd_data.nav[i][0].id),"%s",ids[i]);
    }
    nd_data.traffic_available=1;
    nd_data.traffic_distance_m[2]=37040;nd_data.traffic_bearing[2]=35;
    nd_data.traffic_altitude_m[2]=-609.6f;
    svg=fopen(path,"w");assert(svg);svg_y_origin=234;
    fprintf(svg,"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1620\" height=\"702\" viewBox=\"0 0 540 234\">\n");
    clear_capture();draw_nd(&refs.side[0],0,0,"CAPT");draw_nd(&refs.side[1],180,0,"F/O");
    nd_test_airport();efis[0].zoom=1;refs.side[0].nd_mode=2;refs.side[0].nd_range_nm=1;
    draw_nd(&refs.side[0],360,0,"MOCK APT");
    fprintf(svg,"</svg>\n");fclose(svg);svg=NULL;svg_y_origin=448;
}
static int convex_contains(const Point *polygon,int count,float x,float y) {
    int sign=0;
    if(count<3)return 0;
    for(int i=0;i<count;++i) {
        Point a=polygon[i],b=polygon[(i+1)%count];
        float cross=(b.x-a.x)*(y-a.y)-(b.y-a.y)*(x-a.x);
        if(fabsf(cross)<.0001f)continue;
        if(sign && (cross>0 ? 1 : -1)!=sign)return 0;
        sign=cross>0 ? 1 : -1;
    }
    return 1;
}
static int color_is(Color a, Color b) { return a.r == b.r && a.g == b.g && a.b == b.b; }
static int vertices_in(Color c) {
    int i, n = 0;
    for (i = 0; i < vertex_count; ++i) if (color_is(vertices[i].color, c)) ++n;
    return n;
}
static void fms_leg(A380XFplnRoute *r, int i, const char *id, double lat, double lon) {
    memset(&r->el[i], 0, sizeof(r->el[i]));
    snprintf(r->el[i].ident, sizeof(r->el[i].ident), "%s", id);
    r->el[i].term_lat = lat; r->el[i].term_lon = lon;
    r->el[i].alt_db.alt1_ft = r->el[i].alt_pilot.alt1_ft = NAN;
}
static void test_nd_fms_route(void) {
    A380XFplnRoute *r = &test_plans[0].pri;
    int base_green, base_yellow;
    memset(test_plans, 0, sizeof(test_plans));
    fmsa_fpln = (A380XFlightPlans *)test_plans; /* any non-NULL store */
    nd_test_state();
    /* Baseline: the ND without any route (own-ship symbol etc. share these colours). */
    nd_data.route_count = 0;
    clear_capture(); draw_nd(&refs.side[0], 0, 0, "CAPT");
    base_green = vertices_in(GREEN); base_yellow = vertices_in(YELLOW);
    /* The stock X-Plane route must not be drawn while the native FMS has a plan. */
    nd_data.route_count = 1; nd_data.route_active = 0;
    nd_data.route[0] = (EfisNdRoutePoint){.35, -.1, 0, "STOCK"};
    /* ORIG - WPA (TO) - WPB - disco - WPC (missed approach) */
    test_plans[0].exists = 1; test_plans[0].active_leg_index = 1;
    fms_leg(r, 0, "ORIG", 0, 0); fms_leg(r, 1, "WPA", .1, .05); fms_leg(r, 2, "WPB", .2, .1);
    fms_leg(r, 3, "", NAN, NAN); r->el[3].flags = A380X_LEGF_DISCONTINUITY;
    fms_leg(r, 4, "WPC", .3, .1);
    r->count = 5; r->first_missed_appr_index = 4;
    r->el[2].alt_db.desc = '+'; r->el[2].alt_db.alt1_ft = 7000;
    clear_capture(); draw_nd(&refs.side[0], 0, 0, "CAPT");
    assert(!has_label("STOCK") && !has_label("NO F-PLN"));
    assert(find_label("WPA") && color_is(find_label("WPA")->color, WHITE));   /* TO waypoint */
    assert(find_label("WPB") && color_is(find_label("WPB")->color, GREEN));
    assert(find_label("WPC") && color_is(find_label("WPC")->color, CYAN));    /* missed approach */
    assert(vertices_in(GREEN) > base_green && vertices_in(YELLOW) == base_yellow);
    assert(!has_label("7000"));                                                /* CSTR off */
    efis[0].cstr = 1;
    clear_capture(); draw_nd(&refs.side[0], 0, 0, "CAPT");
    assert(has_label("7000"));
    efis[0].cstr = 0;
    /* A temporary plan is drawn yellow next to the active one. */
    test_plans[1] = test_plans[0];
    snprintf(test_plans[1].pri.el[2].ident, sizeof(test_plans[1].pri.el[2].ident), "TMPB");
    test_plans[1].pri.el[2].term_lat = .2; test_plans[1].pri.el[2].term_lon = -.1;
    clear_capture(); draw_nd(&refs.side[0], 0, 0, "CAPT");
    assert(find_label("TMPB") && color_is(find_label("TMPB")->color, YELLOW));
    assert(vertices_in(YELLOW) > base_yellow && find_label("WPB"));
    /* Without native plans the stock route is the fallback; with neither, NO F-PLN. */
    test_plans[0].exists = test_plans[1].exists = 0;
    clear_capture(); draw_nd(&refs.side[0], 0, 0, "CAPT");
    assert(has_label("STOCK"));
    nd_data.route_count = 0;
    clear_capture(); draw_nd(&refs.side[0], 0, 0, "CAPT");
    assert(has_label("NO F-PLN"));
    fmsa_fpln = NULL;
}
/* FBW-layout ARC page: cruise state with an active leg to SABAR (~30 NM north). */
static void nd_fbw_arc_state(void) {
    A380XFplnRoute *r = &test_plans[0].pri;
    memset(test_plans, 0, sizeof(test_plans));
    fmsa_fpln = (A380XFlightPlans *)test_plans;
    nd_test_state();
    refs.side[0].nd_mode = 3; refs.side[0].nd_range_nm = 40;
    ground_speed_knots = 300; ground_track_degrees = 4;
    nd_data.tas_knots = 250; nd_data.wind_dir_true = 270; nd_data.wind_speed_kt = 30;
    nd_data.magnetic_variation = 0; nd_data.zulu_seconds = 3600; nd_data.altitude_ft = 12000;
    nd_data.route_count = 0;
    efis[0].vor1 = 1;
    snprintf(nd_data.vor[0].id, sizeof(nd_data.vor[0].id), "KUE");
    nd_data.vor[0].distance_nm = 25;
    test_plans[0].exists = 1; test_plans[0].active_leg_index = 1;
    fms_leg(r, 0, "ORIG", -.05, 0); fms_leg(r, 1, "SABAR", .5, 0); fms_leg(r, 2, "AILEY", .8, .05);
    r->count = 3; r->first_missed_appr_index = 3;
    /* VD terrain: sea for the first third, a ridge rising to 8000 ft, then unloaded scenery. */
    {
        int i;
        EfisVdTerrain *vd = &nd_data.vd[0];
        memset(vd, 0, sizeof(*vd));
        vd->range_nm = 40;
        for (i = 0; i < EFIS_VD_COLUMNS * 2 / 3; ++i) {
            vd->valid[i] = 1;
            vd->water[i] = i < EFIS_VD_COLUMNS / 3;
            vd->elevation_ft[i] = vd->water[i] ? 0.0f : 8000.0f * (float)(i - EFIS_VD_COLUMNS / 3) / (EFIS_VD_COLUMNS / 3 - 1);
        }
    }
}
/* TERRAIN on ND in the FBW-layout native ND (a380x_nd_fbw.inc): the raster quad placed about the ARC
 * aircraft reference (384, 620) at 498 px per range, under the map; the FBW TERR box values. */
static void test_nd_fbw_terrain(void) {
    TerrDisplay *d = &nd_data.terrain[0];
    nd_fbw_arc_state();
    efis[0].terr = 1;
    clear_capture(); mock_textured_quads = 0; draw_nd_fbw_arc(&refs.side[0], 0);
    assert(mock_textured_quads == 0 && !has_label("TERR"));
    terr_display_reset(d);
    d->on = 1; assert(terr_geometry(3, 40, &d->geo));
    d->cls[30][40] = TERR_RED_HIGH;
    d->shown = (TerrThresholds){3000, TERR_LEVEL_YELLOW, 9100, TERR_LEVEL_RED, 0};
    clear_capture(); mock_textured_quads = 0; draw_nd_fbw_arc(&refs.side[0], 0);
    assert(mock_textured_quads == 1 && has_label("TERR") && has_label(" 92") && has_label(" 30"));
    /* 498 px per range on both layouts: the image keeps its pixel size, its top edge 498 px above the
     * aircraft line minus the FBW offset (626 - 128), clipped by the map box at y 80 */
    assert(fabsf(mock_quad_x[0] - 6.0f) < .01f && fabsf(mock_quad_y[0] - fminf(NDY(620) + (626 - 128), NDY(80))) < .01f);
    assert(fabsf(mock_tex_u[0]) < 1e-6f && fabsf(mock_tex_u[1] - 756.0f / 760.0f) < 1e-5f);   /* clipped at x 762 */
    d->shown = (TerrThresholds){-1, 0, -1, 0, 0};
    clear_capture(); draw_nd_fbw_arc(&refs.side[0], 0);
    assert(!has_label("TERR"));   /* no terrain in view: box hidden */
    terr_display_reset(d); efis[0].terr = 0;
    fmsa_fpln = NULL;
}
static void test_vd_terrain(void) {
    static const Color VD_TERRAIN = {0.48f, 0.24f, 0.09f};
    nd_fbw_arc_state();
    clear_capture(); draw_nd_fbw_arc(&refs.side[0], 0);
    assert(vertices_in(VD_TERRAIN) > 0);                    /* brown ridge */
    assert(has_label("100") && has_label("MORA"));          /* 8000 + 2000 ft = 100 */
    {   /* water quads sit on the sea-level line of the VD altitude scale (y 960 FBW) */
        int i, water = 0;
        for (i = 0; i < vertex_count; ++i)
            if (color_is(vertices[i].color, CYAN) && fabsf(vertices[i].y - NDY(960)) < .5f && vertices[i].x < 330) ++water;
        assert(water > 0);
    }
    /* A profile sampled for another range is not shown. */
    nd_data.vd[0].range_nm = 80;
    clear_capture(); draw_nd_fbw_arc(&refs.side[0], 0);
    assert(vertices_in(VD_TERRAIN) == 0 && !has_label("MORA"));
    /* ROSE NAV shows the VD too; PLAN does not. */
    nd_data.vd[0].range_nm = 40; refs.side[0].nd_mode = 2;
    clear_capture(); draw_nd_fbw(&refs.side[0], 0);
    assert(has_label("MORA"));
    refs.side[0].nd_mode = 4;
    clear_capture(); draw_nd_fbw(&refs.side[0], 0);
    assert(!has_label("MORA"));
    fmsa_fpln = NULL;
}
static void test_nd_fbw_arc(void) {
    nd_fbw_arc_state();
    clear_capture(); draw_nd_fbw_arc(&refs.side[0], 0);
    /* Header: GS/TAS, wind, TO waypoint with bearing, distance and ETA (01:00 + 30 NM at 300 kt). */
    assert(has_label("GS") && has_label("TAS") && has_label("300") && has_label("250"));
    assert(has_label("270") && has_label("/"));
    assert(find_label("SABAR") && color_is(find_label("SABAR")->color, WHITE));
    assert(has_label("000") && has_label("NM") && has_label("01:06"));
    /* Range rings at 3/4 and 1/2 of 40 NM; VD distance scale 0..40. */
    assert(has_label("30") && has_label("20") && has_label("10") && has_label("40"));
    /* Compass: heading 0 shows 0 (large) through +-50 degrees, nothing at 90. */
    assert(has_label("0") && has_label("5") && has_label("31") && !has_label("9"));
    assert(has_label("VOR1") && has_label("KUE"));
    /* Aircraft symbol wing line at the FBW reference (384,620). */
    {
        int i, found = 0;
        for (i = 0; i < vertex_count; ++i)
            if (color_is(vertices[i].color, YELLOW) && fabsf(vertices[i].x - 343) < .5f &&
                fabsf(vertices[i].y - NDY(620)) < .5f) found = 1;
        assert(found);
    }
    /* No display: nothing but the background. */
    refs.side[0].display_valid = 0;
    clear_capture(); draw_nd_fbw_arc(&refs.side[0], 0);
    assert(label_count == 0);
    refs.side[0].display_valid = 1;
    fmsa_fpln = NULL;
}
static void test_nd_fbw_rose_plan(void) {
    /* ROSE NAV: full heading ring (180 visible behind), range/2 and range/4 labels, VD present. */
    nd_fbw_arc_state(); refs.side[0].nd_mode = 2;
    clear_capture(); draw_nd_fbw(&refs.side[0], 0);
    assert(has_label("GS") && has_label("SABAR") && has_label("0") && has_label("18") && has_label("9"));
    assert(!has_label("1") && !has_label("35"));                           /* ROSE labels every 30 degrees */
    assert(has_label("20") && has_label("10") && has_label("40"));     /* 40 = VD scale end */
    assert(find_label("SABAR")->color.g == WHITE.g);
    /* ROSE VOR: no route, no VD. */
    refs.side[0].nd_mode = 1;
    clear_capture(); draw_nd_fbw(&refs.side[0], 0);
    assert(has_label("SABAR") && !has_label("AILEY") && !has_label("40"));
    /* PLAN: N/E/S/W, range and range/2, route labels, no compass numbers. */
    refs.side[0].nd_mode = 4;
    clear_capture(); draw_nd_fbw(&refs.side[0], 0);
    assert(has_label("N") && has_label("E") && has_label("S") && has_label("W"));
    assert(has_label("40") && has_label("20") && !has_label("18"));
    fmsa_fpln = NULL;
}
static void export_nd_fbw_one(const char *nd_path, const char *name, int mode) {
    char path[1024];
    const char *slash = strrchr(nd_path, '/');
    snprintf(path, sizeof(path), "%.*s%s", slash ? (int)(slash - nd_path + 1) : 0, nd_path, name);
    nd_fbw_arc_state(); refs.side[0].nd_mode = mode;
    svg = fopen(path, "w"); assert(svg); svg_y_origin = 1024;
    fprintf(svg, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"768\" height=\"1024\" viewBox=\"0 0 768 1024\">\n");
    fprintf(svg, "<style>@font-face{font-family:EIS;src:url('PFD_FONT_DATA_URI')}</style>\n");
    clear_capture(); draw_nd_fbw(&refs.side[0], 0);
    fprintf(svg, "</svg>\n"); fclose(svg); svg = NULL; svg_y_origin = 448;
    fmsa_fpln = NULL;
}
static void export_nd_fbw_svg(const char *nd_path) {
    export_nd_fbw_one(nd_path, "nd-fbw-arc.svg", 3);
    export_nd_fbw_one(nd_path, "nd-fbw-rose.svg", 2);
    export_nd_fbw_one(nd_path, "nd-fbw-plan.svg", 4);
}
static int only_labels(const char *const *wanted,int n) {
    int i,j;
    if (label_count!=n) return 0;
    for (i=0;i<n;++i) { for (j=0;j<label_count&&strcmp(labels[j].value,wanted[i]);++j) {} if (j==label_count) return 0; }
    return 1;
}
static void test_altitude_drums(void) {
    static const char *steady[]={"1","2","4","60","40","20"};
    static const char *rolling[]={"1","2","5","4","20","00","80","60"}; /* 20 and 60 partly in view */
    static const char *low[]={"8","60","40","20"};
    PfdCanvas canvas=pfd_canvas(0,32);
    clear_capture(); pfd_altitude_drums(&canvas,12440); assert(only_labels(steady,6));
    /* Tens past 80: the hundreds roll between 4 and 5 and the tens wrap through 00. */
    clear_capture(); pfd_altitude_drums(&canvas,12490); assert(only_labels(rolling,8));
    /* Below 1000 ft no leading zeros. */
    clear_capture(); pfd_altitude_drums(&canvas,840); assert(only_labels(low,4));
    assert(clip_y0==0 && clip_y1==0);  /* clip released */
}
static void test_rising_ground(void) {
    PfdCanvas p=pfd_canvas(0,32);
    Point polygon[12],outline[4];
    Side side;
    int count;
    normal_state();side=refs.side[0];side.pitch_value=side.roll_value=0;
    side.radio_altitude_value=-3;
    count=rising_ground_polygon(&side,1,polygon,outline);
    /* At ground level the earth overlay hides the negative-pitch ladder,
     * while retaining the positive-pitch part above the aircraft reference. */
    assert(convex_contains(polygon,count,0,-5));
    assert(!convex_contains(polygon,count,0,5));
    side.radio_altitude_value=100;
    count=rising_ground_polygon(&side,1,polygon,outline);
    assert(!convex_contains(polygon,count,0,-5));
    assert(convex_contains(polygon,count,0,-25));
    side.radio_altitude_valid=0;
    count=rising_ground_polygon(&side,1,polygon,outline);
    assert(!convex_contains(polygon,count,0,-25));
    assert(convex_contains(polygon,count,0,-35));
    side.radio_altitude_valid=1;side.radio_altitude_value=0;side.roll_value=45;
    count=rising_ground_polygon(&side,1,polygon,outline);
    assert(convex_contains(polygon,count,20,0));
    assert(!convex_contains(polygon,count,-20,0));
    side.roll_value=0;side.pitch_value=-80;
    count=rising_ground_polygon(&side,1,polygon,outline);
    /* Preserve the source rectangle's finite bottom at its upper clamp. */
    assert(convex_contains(polygon,count,0,0));
    assert(!convex_contains(polygon,count,0,-20));
    for(int pitch=-80;pitch<=80;pitch+=40)for(int roll=-180;roll<=180;roll+=30) {
        side.pitch_value=(float)pitch;side.roll_value=(float)roll;
        clear_capture();draw_rising_ground(&p,&side);
        for(int i=0;i<vertex_count;++i) {
            assert(fabsf(vertices[i].x-pfd_x(&p,68.906f))<=36.768f*p.scale+.001f);
            assert(fabsf(vertices[i].y-pfd_y(&p,80.823f))<=42.0815f*p.scale+.001f);
        }
    }
}
static void test_pfd_bank_direction(void) {
    PfdCanvas p=pfd_canvas(0,32);
    const float cx=pfd_x(&p,68.906f),cy=pfd_y(&p,80.823f),half_width=36.768f*p.scale;
    /* X-Plane positive roll is right-wing-down. In the fixed-aircraft PFD,
     * earth therefore rises on the right, and the sky reference moves left.
     * Check physical directions independently of the renderer's angle formula. */
    for(int sign=-1;sign<=1;sign+=2) {
        float left_top=-HUGE_VALF,right_top=-HUGE_VALF,pointer_x=0;
        int pointer_vertices=0;
        normal_state();refs.side[0].pitch_value=0;refs.side[0].roll_value=sign*15.0f;
        clear_capture();draw_horizon(&p,&refs.side[0]);
        for(int i=0;i<vertex_count;++i) {
            CapturedVertex v=vertices[i];
            if(fabsf(v.color.r-EARTH.r)>.001f || fabsf(v.color.g-EARTH.g)>.001f)continue;
            if(fabsf(v.x-(cx-half_width))<.02f)left_top=fmaxf(left_top,v.y);
            if(fabsf(v.x-(cx+half_width))<.02f)right_top=fmaxf(right_top,v.y);
        }
        assert(isfinite(left_top)&&isfinite(right_top));
        assert((right_top-cy)*sign>0 && (left_top-cy)*sign<0);
        clear_capture();draw_pitch_ladder(&p,0,sign*15.0f);
        assert(vertex_count>=2);
        for(int i=0;i+1<vertex_count;i+=2) {
            assert(vertices[i+1].x>vertices[i].x);
            assert((vertices[i+1].y-vertices[i].y)*sign>0);
        }
        clear_capture();draw_bank_scale(&p,sign*15.0f);
        for(int i=0;i<vertex_count;++i) {
            CapturedVertex v=vertices[i];
            if(fabsf(v.color.r-YELLOW.r)<.001f && fabsf(v.color.g-YELLOW.g)<.001f &&
               v.y<pfd_y(&p,38.7f)) {pointer_x+=v.x;++pointer_vertices;}
        }
        assert(pointer_vertices==6);
        assert((pointer_x/pointer_vertices-cx)*sign<0);
    }
}
static void test_pfd_boundaries(void) {
    PfdCanvas p=pfd_canvas(0,32);
    const CapturedText *label;
    normal_state();
    refs.side[0].vertical_speed_value=-1200;
    refs.side[0].radio_altitude_value=1000;
    clear_capture();draw_vertical_speed(&p,&refs.side[0]);
    label=find_label("12");assert(label&&label->color.g==AMBER.g);
    assert(fabsf(label->y-pfd_y(&p,82.554756f+vertical_speed_offset(-1200)+2.4f))<.01f);
    refs.side[0].radio_altitude_value=1000.1f;
    clear_capture();draw_vertical_speed(&p,&refs.side[0]);assert(find_label("12")->color.g==GREEN.g);
    refs.side[0].vertical_speed_value=-2000;
    refs.side[0].radio_altitude_value=2500;
    clear_capture();draw_vertical_speed(&p,&refs.side[0]);assert(find_label("20")->color.g==AMBER.g);
    refs.side[0].radio_altitude_valid=0;
    clear_capture();draw_vertical_speed(&p,&refs.side[0]);assert(find_label("20")->color.g==GREEN.g);
    for(int sign=-1;sign<=1;sign+=2) {
        refs.side[0].vertical_speed_value=sign*6000;
        clear_capture();draw_vertical_speed(&p,&refs.side[0]);assert(find_label("60")->color.g==AMBER.g);
    }
    refs.side[0].vertical_speed_value=199;
    clear_capture();draw_vertical_speed(&p,&refs.side[0]);assert(!has_label("02"));
    normal_state();refs.side[0].radio_altitude_value=-1;
    clear_capture();draw_pfd(&refs.side[0],0,32,"CAPT");assert(has_label("-1"));
    refs.side[0].radio_altitude_value=-.49f;
    clear_capture();draw_pfd(&refs.side[0],0,32,"CAPT");assert(has_label("0")&&!has_label("-0"));
    refs.side[0].radio_altitude_value=310;refs.side[0].decision_height_value=100;
    clear_capture();draw_pfd(&refs.side[0],0,32,"CAPT");assert(find_label("310")->color.g==GREEN.g);
    refs.side[0].decision_height_value=250;
    clear_capture();draw_pfd(&refs.side[0],0,32,"CAPT");assert(find_label("310")->color.g==AMBER.g);
    refs.side[0].radio_altitude_value=200;
    clear_capture();draw_pfd(&refs.side[0],0,32,"CAPT");
    label=find_label("DH");assert(label&&fabsf(label->y-pfd_y(&p,115.0f))<.01f);
    assert(fabsf(label->x-(pfd_x(&p,73.511879f)-2*7.0f*p.scale*.6f))<.01f);
    refs.side[0].pitch_value=26;assert(pfd_excessive_attitude(&refs.side[0]));
    refs.side[0].pitch_value=23;assert(pfd_excessive_attitude(&refs.side[0]));
    refs.side[0].pitch_value=21;assert(!pfd_excessive_attitude(&refs.side[0]));
    refs.side[0].mach_value=.501f;
    clear_capture();draw_speed_tape(&p,&refs.side[0]);assert(has_label(".501"));
    refs.side[0].mach_value=.450f;
    clear_capture();draw_speed_tape(&p,&refs.side[0]);assert(has_label(".450"));
    refs.side[0].mach_value=.449f;
    clear_capture();draw_speed_tape(&p,&refs.side[0]);assert(!has_label(".449"));
    normal_state();published.fma_a1a2=A380X_A1A2_A_FLOOR;
    clock_seconds=12.2f;clear_capture();draw_fma(&p,&refs.side[0]);
    assert(has_label("A.FLOOR")&&count_color(AMBER)>0);
    clock_seconds=12.7f;clear_capture();draw_fma(&p,&refs.side[0]);assert(count_color(AMBER)==0);
    clock_seconds=1;
    normal_state();
    pfd_guidance.flap_handle_index=2;pfd_guidance.flap_config=3;
    pfd_guidance.flap_ratio=pfd_guidance.flap_target_ratio=.5;
    pfd_guidance.slat_ratio=pfd_guidance.slat_target_ratio=1;
    clear_capture();draw_pfd_configuration(&p);assert(find_label("2")->color.g==GREEN.g);
    pfd_guidance.flap_ratio=.25;
    clear_capture();draw_pfd_configuration(&p);assert(find_label("2")->color.g==CYAN.g);
    pfd_guidance.flap_fault=1;
    clear_capture();draw_pfd_configuration(&p);assert(count_color(AMBER)>0);
    /* FBW SFCC display (docs/FLAPS_LEVER.md): "1+F" settled green, target cyan, F RELIEF / A LOCK. */
    normal_state();
    pfd_guidance.sfcc_valid=1;pfd_guidance.flap_handle_index=1;pfd_guidance.flap_config=2;
    pfd_guidance.flap_fppu=215.68;pfd_guidance.slat_fppu=286.48;
    pfd_guidance.flap_target_index=pfd_guidance.slat_target_index=-1;
    clear_capture();draw_pfd_configuration(&p);assert(find_label("1+F")->color.g==GREEN.g&&has_label("S")&&has_label("F"));
    pfd_guidance.flap_config=1;pfd_guidance.flap_target_index=0;
    clear_capture();draw_pfd_configuration(&p);assert(find_label("1")->color.g==CYAN.g);
    pfd_guidance.flap_handle_index=3;pfd_guidance.flap_config=4;pfd_guidance.flap_relief=1;pfd_guidance.flap_target_index=3;
    pfd_guidance.flap_fppu=259.28;pfd_guidance.slat_fppu=327.39;
    clear_capture();draw_pfd_configuration(&p);assert(has_label("F RELIEF")&&!has_label("F")&&find_label("3")->color.g==CYAN.g);
    pfd_guidance.flap_handle_index=0;pfd_guidance.flap_config=0;pfd_guidance.flap_relief=0;pfd_guidance.alpha_lock=1;
    pfd_guidance.flap_fppu=215.68;pfd_guidance.slat_fppu=286.48;pfd_guidance.flap_target_index=0;pfd_guidance.slat_target_index=0;
    clear_capture();draw_pfd_configuration(&p);assert(has_label("A LOCK")&&!has_label("S"));
    pfd_guidance.alpha_lock=0;pfd_guidance.flap_fppu=pfd_guidance.slat_fppu=0;
    pfd_guidance.flap_target_index=pfd_guidance.slat_target_index=-1;
    clear_capture();draw_pfd_configuration(&p);assert(!has_label("0"));   /* clean: hidden */
    pfd_guidance.sfcc_valid=0;
    normal_state();pfd_guidance.stall_warning=1;
    clear_capture();draw_pfd(&refs.side[0],0,32,"CAPT");assert(has_label("STALL"));
    pfd_guidance.on_ground=1;
    clear_capture();draw_pfd(&refs.side[0],0,32,"CAPT");assert(!has_label("STALL"));
}
static void test_fms_flap_speeds(void) {
    /* FBW PRIM rule: S in CONF 1 and 1+F, F in CONF 2 and 3, neither clean or FULL. */
    const int expect_f[6]={0,0,0,1,1,0}, expect_s[6]={0,1,1,0,0,0};
    A380XFmsFeed feed;
    A380XPfdGuidance p;
    memset(&feed,0,sizeof(feed));
    feed.valid=1;feed.f_speed_knots=180;feed.s_speed_knots=210;
    for(int config=0;config<=5;++config) {
        memset(&p,0,sizeof(p));p.flap_config=config;
        a380x_pfd_apply_fms_flap_speeds(&p,&feed);
        assert((p.f_speed_knots==180)==expect_f[config] && (p.f_speed_knots==0)==!expect_f[config]);
        assert((p.s_speed_knots==210)==expect_s[config] && (p.s_speed_knots==0)==!expect_s[config]);
    }
    /* Without FMS data a provider value is left alone. */
    feed.valid=0;memset(&p,0,sizeof(p));p.flap_config=3;p.f_speed_knots=175;
    a380x_pfd_apply_fms_flap_speeds(&p,&feed);
    assert(p.f_speed_knots==175);
    /* Lines keep their authored thickness relative to the canvas a device scales the PFD to. */
    pfd_line_scale=1024.0f/PFD_PIXEL_HEIGHT;
    pfd_line_width(2.0f);
    assert(fabsf(line_width-2.0f*1024.0f/416.0f)<1e-4f);
    pfd_line_scale=1.0f;
    glLineWidth(1.0f);
}
int main(int argc,char **argv) {
    PfdCanvas canvas=pfd_canvas(0,32);
    test_fms_flap_speeds();
    test_nd_fms_route();
    test_nd_fbw_arc();
    test_nd_fbw_rose_plan();
    test_vd_terrain();
    test_nd_fbw_terrain();
    test_altitude_drums();
    test_rising_ground();
    test_pfd_bank_direction();
    test_pfd_boundaries();
    test_nd();
    normal_state();
    draw_heading_tape(&canvas,&refs.side[0]);
    assert(has_vertex(pfd_x(&canvas,68.906f+1.511f),pfd_y(&canvas,145.34f),WHITE));
    refs.side[0].heading_value=359.5f;clear_capture();draw_heading_tape(&canvas,&refs.side[0]);
    assert(has_vertex(pfd_x(&canvas,68.906f+.7555f),pfd_y(&canvas,145.34f),WHITE));
    pfd_guidance.true_heading=1;pfd_guidance.heading_reference_offset_deg=10;
    clear_capture();draw_heading_tape(&canvas,&refs.side[0]);assert(has_label("TRUE"));
    assert(has_label("1") && has_vertex(pfd_x(&canvas,68.906f+.7555f),pfd_y(&canvas,145.34f),WHITE));
    pfd_guidance.true_heading=0;pfd_guidance.heading_reference_offset_deg=0;
    clear_capture();draw_bank_scale(&canvas,0);
    assert(has_vertex(pfd_x(&canvas,68.934f),pfd_y(&canvas,39.750f),YELLOW));
    clear_capture();draw_flight_director(&canvas,&refs.side[0]);
    assert(has_vertex(pfd_x(&canvas,73.303f),pfd_y(&canvas,61.672f),GREEN));
    assert(has_vertex(pfd_x(&canvas,49.263f),pfd_y(&canvas,76.373f),GREEN));
    normal_state();draw_pfd(&refs.side[0],0,32,"CAPT");
    assert(has_label("AP1") && has_label("1FD2") && has_label("A/THR") && has_label("OP CLB") && has_label("ALT"));
    assert(!has_label("APPR 1") && !has_label("LAND 3"));
    /* LAND / FLARE / ROLL OUT span B1+C1 (FMA.tsx BC1Cell) and replace G/S and LOC (a380x_autoland.inc). */
    published.fma_b1=A380X_B1_GS;published.fma_c1=A380X_C1_LOC;published.fma_d1d2=A380X_D1D2_LAND_2;
    pfd_guidance.fma_bc1=A380X_BC1_LAND;clear_capture();draw_fma(&canvas,&refs.side[0]);
    assert(has_label("LAND") && !has_label("G/S") && !has_label("LOC") && has_label("LAND 2"));
    pfd_guidance.fma_bc1=A380X_BC1_ROLL_OUT;clear_capture();draw_fma(&canvas,&refs.side[0]);
    assert(has_label("ROLL OUT") && !has_label("LAND"));
    pfd_guidance.fma_bc1=A380X_BC1_NONE;clear_capture();draw_fma(&canvas,&refs.side[0]);
    assert(has_label("G/S") && has_label("LOC") && !has_label("ROLL OUT"));
    published.fma_b1=A380X_B1_ALT;published.fma_c1=A380X_C1_HDG;published.fma_d1d2=0;
    pfd_guidance.guidance_valid=0;pfd_guidance.athr_active=0;clear_capture();draw_fma(&canvas,&refs.side[0]);
    assert(!has_label("OP CLB") && !has_label("HDG") && !has_label("THR CLB") && !has_label("ALT"));
    center_right.rto_arm=1;clear_capture();draw_fma(&canvas,&refs.side[0]);
    assert(has_label("RTO"));center_right.rto_arm=0;
    center_right.autobrake_armed=5;
    clear_capture();draw_fma(&canvas,&refs.side[0]);
    assert(has_label("BRK HI") && find_label("BRK HI")->color.b == CYAN.b);
    center_right.autobrake_active=1;
    clear_capture();draw_fma(&canvas,&refs.side[0]);
    assert(has_label("BRK HI") && find_label("BRK HI")->color.b == GREEN.b);
    center_right.autobrake_armed=0;center_right.autobrake_active=0;
    clear_capture();draw_flight_director(&canvas,&refs.side[0]);assert(!has_label("FD"));
    pfd_guidance.guidance_fault=1;clear_capture();draw_flight_director(&canvas,&refs.side[0]);assert(has_label("FD"));
    pfd_guidance.guidance_valid=1;pfd_guidance.guidance_fault=0;pfd_guidance.athr_active=1;
    /* Below sea level FBW keeps the absolute drums and stacks N/E/G beside them. */
    refs.side[0].altitude_value=-20;clear_capture();draw_altitude_tape(&canvas,&refs.side[0]);
    assert(has_label("N")&&has_label("E")&&has_label("G")&&has_label("0")&&has_label("20"));
    refs.side[0].radio_altitude_value=54;clear_capture();draw_pfd(&refs.side[0],0,32,"CAPT");assert(has_label("50"));
    refs.side[0].ls_visible=1;refs.side[0].loc_valid=refs.side[0].gs_valid=0;
    clear_capture();draw_landing_system(&canvas,&refs.side[0]);assert(count_color(MAGENTA)==0);
    refs.side[0].loc_valid=refs.side[0].gs_valid=1;refs.side[0].loc_dots=3;refs.side[0].gs_dots=-3;
    clear_capture();draw_landing_system(&canvas,&refs.side[0]);assert(count_color(MAGENTA)==8);
    snprintf(refs.side[0].nav_ident,sizeof(refs.side[0].nav_ident),"IBOS");
    refs.side[0].nav_frequency_mhz=110.3f;refs.side[0].nav_dme_valid=1;refs.side[0].nav_dme_nm=8.4f;
    clear_capture();draw_landing_system(&canvas,&refs.side[0]);
    assert(has_label("IBOS") && has_label("110.30") && has_label("8.4NM"));
    refs.side[0].air_valid=refs.side[0].altitude_valid=refs.side[0].heading_valid=0;
    refs.side[0].attitude_valid=refs.side[0].radio_altitude_valid=0;
    clear_capture();draw_pfd(&refs.side[0],0,32,"CAPT");
    assert(has_label("SPD") && has_label("ALT") && has_label("HDG") && has_label("ATT") && has_label("RA"));
    refs.side[0].display_valid=0;clear_capture();draw_pfd(&refs.side[0],0,32,"CAPT");assert(label_count==0);
    normal_state();draw_nd(&refs.side[0],328,136,"CAPT");
    assert(has_label("ARC") && has_label("80 NM"));
    refs.side[0].nd_mode=4;refs.side[0].nd_range_nm=160;
    clear_capture();draw_nd(&refs.side[0],328,136,"CAPT");
    assert(has_label("PLAN") && has_label("160 NM"));
    if(argc>1) {normal_state();export_svg(argv[1]);}
    if(argc>2) {
        normal_state();refs.side[0].ias_value=145;refs.side[0].altitude_value=2440;refs.side[0].radio_altitude_value=650;
        refs.side[0].vertical_speed_value=-750;refs.side[0].pitch_value=2;refs.side[0].roll_value=-3;
        refs.side[0].ls_visible=refs.side[0].loc_valid=refs.side[0].gs_valid=1;refs.side[0].loc_dots=.4f;refs.side[0].gs_dots=-.3f;
        pfd_guidance.selected_speed_knots=140;pfd_guidance.selected_altitude_ft=3000;pfd_guidance.fd_pitch_error_deg=-.7;
        pfd_guidance.fd_roll_error_deg=2;pfd_guidance.altitude_armed=0;pfd_guidance.metric_altitude=1;
        pfd_guidance.flap_handle_index=4;pfd_guidance.flap_config=5;
        pfd_guidance.flap_ratio=pfd_guidance.slat_ratio=1;
        pfd_guidance.flap_target_ratio=pfd_guidance.slat_target_ratio=1;
        pfd_guidance.spoilers_armed=pfd_guidance.gear_down=1;
        pfd_guidance.envelope_valid=1;pfd_guidance.vmax_knots=182;pfd_guidance.vls_knots=132;
        pfd_guidance.alpha_prot_knots=120;pfd_guidance.alpha_limit_knots=112;
        published.fma_a1a2=A380X_A1A2_SPEED;published.fma_b1=A380X_B1_GS;published.fma_c1=A380X_C1_LOC;
        published.fma_d1d2=A380X_D1D2_RAW_ONLY;export_svg(argv[2]);
    }
    /* EFIS VV is side-specific and uses a black vector in HDG/V/S. */
    normal_state();pfd_guidance.fd1=pfd_guidance.fd2=0;
    clear_capture();draw_flight_director(&canvas,&refs.side[0]);assert(vertex_count==0);
    efis[0].vv=1;
    clear_capture();draw_flight_director(&canvas,&refs.side[0]);assert(count_color(BLACK)>40);
    clear_capture();draw_flight_director(&canvas,&refs.side[1]);assert(vertex_count==0);
    efis[0].baro_inhg=1;refs.side[0].baro_hpa_value=1013.2075f;
    clear_capture();draw_baro(&canvas,&refs.side[0]);assert(has_label("29.92"));
    clear_capture();draw_baro(&canvas,&refs.side[1]);assert(has_label("1013"));
    efis[0].taxi_requested=1;
    clear_capture();draw_pfd(&refs.side[0],0,32,"CAPT");assert(has_label("ETACS NOT AVAILABLE"));
    memset(efis,0,sizeof(efis));
    /* Unpowered: dark bed only (upper row, five lower cells, baro) per side. */
    clear_capture();draw_efis_cp();assert(vertex_count==2*7*4&&label_count==0);
    efis[0].powered=efis[1].powered=1;efis[0].qnh_hpa=1013.2075f;
    efis[0].baro_inhg=1;efis[0].wpt=1;
    efis[1].baro_std=1;efis[1].baro_preset_hpa=1001;
    clear_capture();draw_efis_cp();assert(vertex_count>1000);
    /* FBW dot-matrix legends and baro window stay inside the EFIS atlas band. */
    for(int i=0;i<vertex_count;++i) {
        assert(vertices[i].y>=1500&&vertices[i].y<=1996);
        assert(vertices[i].x>=0&&vertices[i].x<=1928);
    }
    assert(has_label("29.92")&&has_label("Std")&&has_label("QNH")&&has_label("QFE"));
    assert(!has_label("1001")); /* STD preset hidden until the knob is turned */
    efis[1].baro_preset_visible=1;efis[1].baro_inhg=0;
    clear_capture();draw_efis_cp();assert(has_label("1001"));
    efis[0].baro_inhg=0;efis[0].qnh_hpa=998.4f;
    clear_capture();draw_efis_cp();assert(has_label("0998"));
    if(argc>5) {
        svg=fopen(argv[5],"w");assert(svg);svg_y_origin=1024;
        fprintf(svg,"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1024\" height=\"504\" viewBox=\"0 0 1024 504\">\n");
        clear_capture();draw_efis_cp();fprintf(svg,"</svg>\n");fclose(svg);svg=NULL;svg_y_origin=448;
    }
    if(argc>6) { export_nd_svg(argv[6]); export_nd_fbw_svg(argv[6]); }
    test_fcu();
    if(argc>3) export_fcu_svg(argv[3]);
    if(argc>4) {
        guidance.speed_dashes=guidance.heading_dashes=guidance.vs_dashes=1;
        guidance.speed_managed=guidance.heading_managed=guidance.altitude_managed=1;
        export_fcu_svg(argv[4]);
    }
    puts("FCU renderer: FBW fonts, power blanking, selected values, managed dashes, ghost legends, MACH, TRK/FPA, TRUE and FBW V/S format passed");
    puts("PFD renderer: heading wrap/scroll, bank pointer, FD direction, FMA, altitude sign, RA, LS validity/offscale and failures passed");
    puts("ND renderer: independent EFIS filters, FMS constraints, PLAN projection/centre, real traffic/terrain validity and airport moving map passed");
    return 0;
}
