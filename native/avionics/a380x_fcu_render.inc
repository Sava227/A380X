/* SPDX-License-Identifier: GPL-3.0-or-later
 * AFS CP LCDs, translated from the pinned FlyByWire A380X FCU display
 * (systems/instruments/src/FCU/Components/{Spd,Hdg,Alt,Vs}Display.tsx): the FBW
 * "Digital" value font (A380X_FCU.ttf) and Poppins-SemiBold labels, rasterised at
 * their exact texel size into a380x_fcu_font.h. Each physical LCD owns a
 * 1024 x 212 texel atlas slice, which matches the authored glass aspect (4.8:1).
 *
 * The includer provides fcu_text_draw(face, x, baseline_y, text, color): the GL
 * atlas implementation is a380x_fcu_font.inc; the offline tests capture/raster it.
 */
#include "a380x_fcu_font.h"
static float fcu_lcd_level=1.0f;
/* Reference capture of the FBW A380X AFS CP in MSFS: background #3F23F3,
 * inactive (ghosted) mode legends slightly darker. Shared with the EFIS LCDs. */
/* LCD backlight #16145E and its inactive-segment ghost at the original ratio (2026-11-12 user choice;
 * was 63,35,243 / 54,32,214). */
static const Color FCU_LCD_BACKGROUND={22.f/255,20.f/255,94.f/255};
static const Color FCU_LCD_GHOST={0.0739f,0.0717f,0.3246f};
static const Color FCU_LCD_INK={1,1,1};
enum {FCU_GLASS_X=0,FCU_GLASS_Y=1280,FCU_GLASS_W=2048,FCU_GLASS_H=212,FCU_GLASS_SCREEN_COUNT=2,
      FCU_LCD_W=FCU_GLASS_W/FCU_GLASS_SCREEN_COUNT};
static void fcu_text_draw(int face,float x,float baseline_y,const char *text,Color color);

typedef struct {
    char value[4][24];           /* FBW strings: Digital font, '~' = blank cell, 'o' = small zero */
    const char *title[4];        /* active legends */
    const char *ghost[4];        /* paired inactive legends, NULL when none */
    int knots,degrees,true_reference;
} FcuGlass;

static Color fcu_tint(Color c,float level) {
    float v=fminf(1,fmaxf(0,isfinite(level)?level:0));
    return (Color){c.r*v,c.g*v,c.b*v};
}

/* Display strings exactly as the FBW components build them (lights test omitted:
 * the port has no annunciator-test input for the AFS CP). */
static FcuGlass fcu_glass_values(const A380XGuidanceState *s) {
    FcuGlass v;memset(&v,0,sizeof(v));
    int spd_dashes=s->speed_dashes||!isfinite(s->selected_speed_knots)||!isfinite(s->selected_mach);
    int hdg_dashes=s->heading_dashes||!isfinite(s->selected_heading_deg);
    int vs_dashes=s->vs_dashes||!isfinite(s->selected_vs_fpm)||!isfinite(s->selected_fpa_deg);
    v.title[0]=s->mach_mode?"MACH":"SPD";  v.ghost[0]=s->mach_mode?"SPD":"MACH";
    v.title[1]=s->trk_fpa_mode?"TRK":"HDG"; v.ghost[1]=s->trk_fpa_mode?"HDG":"TRK";
    v.title[2]="ALT";                      v.ghost[2]=NULL;
    v.title[3]=s->trk_fpa_mode?"FPA":"V/S"; v.ghost[3]=s->trk_fpa_mode?"V/S":"FPA";
    if(spd_dashes)snprintf(v.value[0],sizeof(v.value[0]),"---");
    else if(s->mach_mode)snprintf(v.value[0],sizeof(v.value[0]),"%.2f",fmin(.99,fmax(0,s->selected_mach)));
    else snprintf(v.value[0],sizeof(v.value[0]),"%03d",(int)lround(fmin(999,fmax(0,s->selected_speed_knots))));
    if(hdg_dashes)snprintf(v.value[1],sizeof(v.value[1]),"---");
    else snprintf(v.value[1],sizeof(v.value[1]),"%03d",((int)lround(s->selected_heading_deg)%360+360)%360);
    if(isfinite(s->selected_altitude_ft))
        snprintf(v.value[2],sizeof(v.value[2]),"%05d",(int)lround(fmin(99999,fmax(0,s->selected_altitude_ft))));
    else snprintf(v.value[2],sizeof(v.value[2]),"00000");          /* the ALT window never shows dashes */
    if(vs_dashes)snprintf(v.value[3],sizeof(v.value[3]),"~----");
    else if(s->trk_fpa_mode) {
        double a=round(fabs(s->selected_fpa_deg)*10)/10;
        snprintf(v.value[3],sizeof(v.value[3]),"~%c%.1f",s->selected_fpa_deg<0&&a>0?'-':'+',fmin(9.9,a));
    } else {
        int hundreds=(int)floor(fabs(s->selected_vs_fpm)*.01+1e-9);
        snprintf(v.value[3],sizeof(v.value[3]),"%c%02doo",s->selected_vs_fpm<0?'-':'+',hundreds>99?99:hundreds);
    }
    v.knots=!s->mach_mode&&!spd_dashes;
    v.degrees=!hdg_dashes;
    v.true_reference=s->true_heading;
    return v;
}

/* Layout in texels from each LCD's top-left (y down), measured from the reference
 * capture and scaled to the 1024 x 212 slice: labels sit on a common baseline,
 * values on the lower baseline. Mode legends have fixed positions; the inactive
 * one of each pair stays faintly visible, as on the physical LCD. */
enum { FCU_LABEL_BASELINE=55, FCU_VALUE_BASELINE=193, FCU_LEGEND_GAP=6 };
static void fcu_lcd_text(int screen,int face,float x,float y_down,const char *text,Color color) {
    fcu_text_draw(face,FCU_GLASS_X+screen*FCU_LCD_W+x,FCU_GLASS_Y+FCU_GLASS_H-y_down,text,color);
}
static void draw_fcu(void) {
    int lit=guidance_inputs.powered&&isfinite(fcu_lcd_level)&&fcu_lcd_level>0;
    Color background=lit?fcu_tint(FCU_LCD_BACKGROUND,fcu_lcd_level):BLACK;
    for(int screen=0;screen<FCU_GLASS_SCREEN_COUNT;++screen) {
        float left=FCU_GLASS_X+screen*FCU_LCD_W;
        rectangle(left,FCU_GLASS_Y,left+FCU_LCD_W,FCU_GLASS_Y+FCU_GLASS_H,background);
    }
    if(!lit)return;
    FcuGlass v=fcu_glass_values(&guidance);
    Color ink=fcu_tint(FCU_LCD_INK,fcu_lcd_level),ghost=fcu_tint(FCU_LCD_GHOST,fcu_lcd_level);
    /* Left LCD: MACH ends just left of SPD; TRK starts just right of HDG. */
    {
        float spd=237,mach=spd-FCU_LEGEND_GAP-fcu_text_width(FCU_FACE_LABEL,"MACH");
        float hdg=673,trk=hdg+fcu_text_width(FCU_FACE_LABEL,"HDG")+FCU_LEGEND_GAP;
        fcu_lcd_text(0,FCU_FACE_LABEL,mach,FCU_LABEL_BASELINE,"MACH",guidance.mach_mode?ink:ghost);
        fcu_lcd_text(0,FCU_FACE_LABEL,spd,FCU_LABEL_BASELINE,"SPD",guidance.mach_mode?ghost:ink);
        fcu_lcd_text(0,FCU_FACE_LABEL,hdg,FCU_LABEL_BASELINE,"HDG",guidance.trk_fpa_mode?ghost:ink);
        fcu_lcd_text(0,FCU_FACE_LABEL,trk,FCU_LABEL_BASELINE,"TRK",guidance.trk_fpa_mode?ink:ghost);
        if(v.true_reference)
            fcu_lcd_text(0,FCU_FACE_LABEL,hdg-3*FCU_LEGEND_GAP-fcu_text_width(FCU_FACE_LABEL,"TRUE"),
                         FCU_LABEL_BASELINE,"TRUE",ink);
    }
    fcu_lcd_text(0,FCU_FACE_VALUE,77,FCU_VALUE_BASELINE,v.value[0],ink);
    if(v.knots)fcu_lcd_text(0,FCU_FACE_LABEL,356,FCU_VALUE_BASELINE,"KT",ink);
    fcu_lcd_text(0,FCU_FACE_VALUE,659,FCU_VALUE_BASELINE,v.value[1],ink);
    if(v.degrees)fcu_lcd_text(0,FCU_FACE_VALUE,940,FCU_VALUE_BASELINE,"\xb0",ink);
    /* Right LCD: ALT, then V/S with FPA just to its right. */
    fcu_lcd_text(1,FCU_FACE_LABEL,226,FCU_LABEL_BASELINE,"ALT",ink);
    fcu_lcd_text(1,FCU_FACE_VALUE,53,FCU_VALUE_BASELINE,v.value[2],ink);
    {
        float vs=739,fpa=vs+fcu_text_width(FCU_FACE_LABEL,"V/S")+FCU_LEGEND_GAP;
        fcu_lcd_text(1,FCU_FACE_LABEL,vs,FCU_LABEL_BASELINE,"V/S",guidance.trk_fpa_mode?ghost:ink);
        fcu_lcd_text(1,FCU_FACE_LABEL,fpa,FCU_LABEL_BASELINE,"FPA",guidance.trk_fpa_mode?ink:ghost);
    }
    fcu_lcd_text(1,FCU_FACE_VS,573,FCU_VALUE_BASELINE,v.value[3],ink);
}
