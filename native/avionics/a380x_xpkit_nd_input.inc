/* SPDX-License-Identifier: GPL-3.0-or-later
 * Device texels are 768x1024, bottom-left. JS client coordinates are top-left.
 * Every ordered button event is retained across host frames; hover is sampled.
 */
#include "a380x_elec_power.h"
#include "a380x_avionics_popup.h"
/* CDS boot / self-test screen hook (a380x_avionics.c defines it before including the xp-kit adapter). */
#ifndef XPKIT_ND_OVERLAY
#define XPKIT_ND_OVERLAY(side) 0
#endif
#define XPKIT_ND_EVENTS 32
#define XPKIT_ND_ZOOM_MAX 6
#define XPKIT_ND_ZOOM_DEFAULT 1 /* 0.1 NM compass diameter: close airport/stand view. */
typedef struct { unsigned seq; int type,x,y,amount; } XpKitNdEvent;
typedef struct {
    int side,x,y,inside,down,zoom,seen,draw_live,draw_powered;
    unsigned serial,presented_frame;
    XpKitNdEvent events[XPKIT_ND_EVENTS];
    XPLMAvionicsID device;
    XPLMCommandRef command[5];
    XPLMDataRef cursor_ref[3];
} XpKitNd;
static XpKitNd xpkit_nd[2];
static const char *xpkit_nd_actions[]={"popup","map_data","center","zoom_in","zoom_out"};
static int xpkit_nd_du(const XpKitNd *d) { return d->side ? A380X_ELN_DU_FO_ND : A380X_ELN_DU_CAPT_ND; }
/* ND display unit On (A380 network); without the network the EFIS power as before. */
static int xpkit_nd_on(XpKitNd *d) {
    return plugin_enabled && a380x_power_du_on(xpkit_nd_du(d), efis[d->side].powered) &&
           du_channels[d->side*DU_CHANNELS_PER_SIDE+DU_ND].level>0;
}
static int xpkit_nd_input_on(XpKitNd *d) { return xpkit_nd_on(d)&&xpkit_health[d->side+6]; }
static int xpkit_nd_frame_pending(XpKitNd *d,unsigned sequence,int live,int powered) {
    /* The legacy atlas can upload this texture before the dedicated surface
     * draws it. Track presentation separately from that shared GL upload. */
    return (live && sequence!=d->presented_frame) || live!=d->draw_live || powered!=d->draw_powered;
}
static void xpkit_nd_renderer_lost(void) {
    for(int side=0;side<2;++side) {
        XpKitNd *d=&xpkit_nd[side];
        xpkit_health[side+6]=0;
        d->inside=d->down=d->seen=d->draw_live=0;
        d->presented_frame=0;
        /* A dead helper cannot publish another frame. Explicitly repaint the
         * on-demand surface so it presents fallback instead of its last ND. */
        if(d->device&&mfda_av.loaded)mfda_av.needs_drawing(d->device);
    }
}
static void xpkit_nd_event(XpKitNd *d,int type,int x,int y,int amount) {
    d->x=x<0?0:x>767?767:x; d->y=y<0?0:y>1023?1023:y;
    if(type!=4) d->inside=1;
    if(type==1) d->down=1;
    if(type==2||type==4) d->down=0;
    if(type==4) d->inside=0;
    unsigned seq=++d->serial;
    d->events[(seq-1)%XPKIT_ND_EVENTS]=(XpKitNdEvent){seq,type,d->x,d->y,amount};
    /* Cursor geometry is composed over the retained texture. Pointer motion
     * requests a cheap device draw, never a WebKit capture or pixel upload. */
    if(d->device&&mfda_av.loaded)mfda_av.needs_drawing(d->device);
}
static int xpkit_nd_touch(int x,int y,XPLMMouseStatus phase,void *refcon) {
    XpKitNd *d=refcon;
    if(!xpkit_nd_input_on(d)) return 1;
    /* Releases/drags may arrive with an off-screen sentinel (-16,-16) or (0,0):
     * finish the gesture where it was instead of in the corner. */
    if(phase!=xplm_MouseDown&&d->down&&((x==0&&y==0)||x<0||y<0||x>767||y>1023)) {x=d->x;y=1023-d->y;}
    xpkit_nd_event(d,phase==xplm_MouseDown?1:phase==xplm_MouseUp?2:0,x,1023-y,0);
    return 1;
}
static int xpkit_nd_right(int x,int y,XPLMMouseStatus phase,void *refcon) {
    (void)x;(void)y;(void)phase;(void)refcon; return 0; /* Preserve right-drag camera look. */
}
static int xpkit_nd_scroll(int x,int y,int wheel,int clicks,void *refcon) {
    XpKitNd *d=refcon;
    if(!xpkit_nd_input_on(d)||wheel||!efis[d->side].zoom) return 0;
    xpkit_nd_event(d,3,x,1023-y,clicks); return 1;
}
static XPLMCursorStatus xpkit_nd_cursor(int x,int y,void *refcon) {
    XpKitNd *d=refcon;d->seen=1;
    if(!xpkit_nd_input_on(d)) return xplm_CursorDefault;
    if(!d->inside||d->x!=x||d->y!=1023-y) xpkit_nd_event(d,0,x,1023-y,0);
    return xplm_CursorHidden;
}
/* Analytic stroke quads with a one-pixel alpha fringe. Avoid driver-dependent
 * wide-line smoothing and preserve the original FlyByWire cursor proportions. */
static void xpkit_nd_cursor_line(float ax,float ay,float bx,float by,float width) {
    float dx=bx-ax,dy=by-ay,length=sqrtf(dx*dx+dy*dy);
    if(length<=0)return;
    float nx=-dy/length,ny=dx/length,half=width*.5f,outer=half+.75f;
    glBegin(GL_QUADS);
    glColor4f(1,1,0,1);
    glVertex2f(ax+nx*half,ay+ny*half);glVertex2f(bx+nx*half,by+ny*half);
    glVertex2f(bx-nx*half,by-ny*half);glVertex2f(ax-nx*half,ay-ny*half);
    for(int sign=-1;sign<=1;sign+=2) {
        glColor4f(1,1,0,1);
        glVertex2f(ax+sign*nx*half,ay+sign*ny*half);glVertex2f(bx+sign*nx*half,by+sign*ny*half);
        glColor4f(1,1,0,0);
        glVertex2f(bx+sign*nx*outer,by+sign*ny*outer);glVertex2f(ax+sign*nx*outer,ay+sign*ny*outer);
    }
    glEnd();
}
static void xpkit_nd_draw_cursor(XpKitNd *d) {
    if(!d->inside || !xpkit_nd_input_on(d))return;
    XPLMSetGraphicsState(0,0,0,0,1,0,0);
    glPushMatrix();glTranslatef((float)d->x,1023.0f-(float)d->y,0);
    if(d->side)glRotatef(-90,0,0,1);
    xpkit_nd_cursor_line(-40,40,0,5,3);xpkit_nd_cursor_line(0,5,40,40,3);
    xpkit_nd_cursor_line(-40,-40,0,-5,3);xpkit_nd_cursor_line(0,-5,40,-40,3);
    xpkit_nd_cursor_line(-1,0,1,0,2);xpkit_nd_cursor_line(0,-1,0,1,2);
    glPopMatrix();glColor4f(1,1,1,1);
}
static void xpkit_nd_draw_device(void *refcon) {
    XpKitNd *d=refcon;
    GLboolean cull=glIsEnabled(GL_CULL_FACE);glDisable(GL_CULL_FACE);
    XPLMSetGraphicsState(0,0,0,0,1,0,0);
    glClearColor(0,0,0,1);glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);glClear(GL_COLOR_BUFFER_BIT);
    /* Keep alpha = 1, as the MFD/SD do: the device is drawn with X-Plane's leftover blend state,
     * which can zero the alpha, and a see-through ND shows the 180x234 panel-atlas copy behind it
     * (the "blurry ND") and a transparent pop-up. */
    glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_FALSE);glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    if(XPKIT_ND_OVERLAY(d->side)) { d->presented_frame=0; glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE); if(cull)glEnable(GL_CULL_FACE); return; }
    int rendered=plugin_enabled && xpkit_draw_view(d->side+2,0,0,768,1024);
    if(plugin_enabled && !rendered) {
        glPushMatrix();glScalef(768.0f/180.0f,1024.0f/234.0f,1);
        draw_nd(&refs.side[d->side],0,0,d->side?"F/O":"CAPT");glPopMatrix();
    }
    d->presented_frame=rendered?xpkit_uploaded[d->side+2]:0;
    xpkit_nd_draw_cursor(d);
    glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
    if(xpkit_shared)atomic_fetch_add_explicit(&xpkit_shared->nd_profile[d->side].draw_count,1,memory_order_relaxed);
    if(cull)glEnable(GL_CULL_FACE);
}
static float xpkit_nd_brightness(float rheo,float ambient,float bus,void *refcon) {
    XpKitNd *d=refcon;(void)rheo;(void)ambient;(void)bus;
    int lit=xpkit_nd_on(d) || (plugin_enabled && a380x_power_du_lit(xpkit_nd_du(d),0));
    return lit?du_channels[d->side*DU_CHANNELS_PER_SIDE+DU_ND].level:0;
}
static int xpkit_nd_command(XPLMCommandRef cmd,XPLMCommandPhase phase,void *refcon) {
    XpKitNd *d=refcon;
    if(phase!=xplm_CommandBegin||!plugin_enabled)return 1;
    for(int a=0;a<5;++a) if(cmd==d->command[a]) {
        if(a==0&&d->device&&mfda_av.loaded) {
            int show=!mfda_av.is_popup_visible(d->device);
            if(show)a380x_fit_popup(d->device,d->side,mfda_av.get_geometry,mfda_av.set_geometry);
            mfda_av.set_popup_visible(d->device,show);
        }
        else if(a==1&&xpkit_nd_input_on(d))xpkit_nd_event(d,5,d->x,d->y,0);
        else if(a==2&&xpkit_nd_input_on(d))xpkit_nd_event(d,6,d->x,d->y,0);
        else if(a==3&&d->zoom>0)--d->zoom;
        else if(a==4&&d->zoom<XPKIT_ND_ZOOM_MAX)++d->zoom;
    }
    return 1;
}
static float xpkit_nd_get_cursor(void *refcon) {
    int *value=refcon;return (float)*value;
}
static void xpkit_nd_set_x(void *refcon,float value) {
    XpKitNd *d=refcon;if(isfinite(value)&&xpkit_nd_input_on(d))xpkit_nd_event(d,0,(int)value,d->y,0);
}
static void xpkit_nd_set_y(void *refcon,float value) {
    XpKitNd *d=refcon;if(isfinite(value)&&xpkit_nd_input_on(d))xpkit_nd_event(d,0,d->x,(int)value,0);
}
static void xpkit_nd_reset(void) {
    for(int s=0;s<2;++s) {xpkit_nd[s].serial=xpkit_nd[s].presented_frame=0;xpkit_nd[s].inside=xpkit_nd[s].down=0;
        xpkit_nd[s].draw_live=xpkit_nd[s].draw_powered=0;memset(xpkit_nd[s].events,0,sizeof(xpkit_nd[s].events));}
}
static void xpkit_nd_init(void) {
    for(int s=0;s<2;++s) {
        XpKitNd *d=&xpkit_nd[s]; d->side=s;d->zoom=XPKIT_ND_ZOOM_DEFAULT;d->x=d->y=384;
        char name[128];
        for(int a=0;a<5;++a) {
            snprintf(name,sizeof(name),"a380x/xp12/nd/%s/%s",s?"first_officer":"captain",xpkit_nd_actions[a]);
            d->command[a]=XPLMCreateCommand(name,"A380X FlyByWire ND/OANS control");
            XPLMRegisterCommandHandler(d->command[a],xpkit_nd_command,1,d);
        }
        for(int a=0;a<3;++a) {
            snprintf(name,sizeof(name),"a380x/xp12/nd/%s/cursor_%s",s?"first_officer":"captain",a==0?"x":a==1?"y":"visible");
            d->cursor_ref[a]=XPLMRegisterDataAccessor(name,xplmType_Float,a<2,NULL,NULL,xpkit_nd_get_cursor,
                a==0?xpkit_nd_set_x:a==1?xpkit_nd_set_y:NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
                a==0?&d->x:a==1?&d->y:&d->inside,d);
        }
        if(mfda_av.loaded) {
            XPLMCreateAvionics_t p;memset(&p,0,sizeof(p));p.structSize=sizeof(p);
            p.screenWidth=p.bezelWidth=768;p.screenHeight=p.bezelHeight=1024;
            p.drawOnDemand=1;p.drawCallback=xpkit_nd_draw_device;p.bezelDrawCallback=a380x_black_bezel;p.screenTouchCallback=xpkit_nd_touch;
            p.screenRightTouchCallback=xpkit_nd_right;p.screenScrollCallback=xpkit_nd_scroll;p.screenCursorCallback=xpkit_nd_cursor;
            p.brightnessCallback=xpkit_nd_brightness;p.refcon=d;
            p.deviceID=s?"a380x.xp12.nd.first_officer":"a380x.xp12.nd.captain";
            p.deviceName=s?"A380X FlyByWire ND F/O":"A380X FlyByWire ND CAPT";
            d->device=mfda_av.create(&p);
        }
    }
    xpkit_health[10]=xpkit_nd[0].device&&xpkit_nd[1].device;
}
static void xpkit_nd_poll(void) {
    for(int s=0;s<2;++s) {
        XpKitNd *d=&xpkit_nd[s];int x,y;
        xpkit_health[11+s]=d->device&&mfda_av.loaded&&mfda_av.is_bound(d->device);
        if(xpkit_shared) {int delta=atomic_exchange(&xpkit_shared->zoom_delta[s],0);d->zoom=d->zoom+delta<0?0:d->zoom+delta>XPKIT_ND_ZOOM_MAX?XPKIT_ND_ZOOM_MAX:d->zoom+delta;}
        if(d->device&&mfda_av.loaded&&xpkit_nd_input_on(d)&&mfda_av.cursor_over(d->device,&x,&y)) {
            if(!d->inside||d->x!=x||d->y!=1023-y)xpkit_nd_event(d,0,x,1023-y,0);
        }else if(d->inside&&(!xpkit_nd_input_on(d)||(!d->seen&&!d->down)))xpkit_nd_event(d,4,d->x,d->y,0);
        d->seen=0;
    }
}
static void xpkit_nd_shutdown(void) {
    for(int s=0;s<2;++s) {
        XpKitNd *d=&xpkit_nd[s];
        if(d->device&&mfda_av.loaded)mfda_av.destroy(d->device);
        for(int a=0;a<5;++a)if(d->command[a])XPLMUnregisterCommandHandler(d->command[a],xpkit_nd_command,1,d);
        for(int a=0;a<3;++a)if(d->cursor_ref[a])XPLMUnregisterDataAccessor(d->cursor_ref[a]);
        memset(d,0,sizeof(*d));
    }
}
