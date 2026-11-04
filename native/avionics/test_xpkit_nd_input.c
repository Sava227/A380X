/* SPDX-License-Identifier: GPL-3.0-or-later
 * Exercise production pointer routing and retained-texture cursor composition
 * without an X-Plane process, browser, or OpenGL context.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <OpenGL/gl.h>
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"
#include "a380x_xpkit_shared.h"

enum {DU_CHANNELS_PER_SIDE=3,DU_ND=1};
static int plugin_enabled=1,xpkit_health[19];
static unsigned xpkit_uploaded[XPKIT_VIEWS];
static struct {int powered,zoom;} efis[2]={{1,1},{1,1}};
static struct {float level;} du_channels[6]={{1},{1},{1},{1},{1},{1}};
static A380XXpKitShared memory;
static A380XXpKitShared *xpkit_shared=&memory;
typedef struct {int unused;} Side;
static struct {Side side[2];} refs;
static int redraws,views,vertices,matrices,rotations,fallbacks;
static float translation_x,translation_y;
static void redraw(XPLMAvionicsID id){assert(id);++redraws;}
static int absent(XPLMAvionicsID id,int *x,int *y){(void)id;(void)x;(void)y;return 0;}
static int bound(XPLMAvionicsID id){assert(id);return 1;}
static struct {
    int loaded;
    void (*needs_drawing)(XPLMAvionicsID);
    void (*set_popup_visible)(XPLMAvionicsID,int);
    int (*is_popup_visible)(XPLMAvionicsID);
    XPLMAvionicsID (*create)(XPLMCreateAvionics_t *);
    int (*cursor_over)(XPLMAvionicsID,int *,int *);
    int (*is_bound)(XPLMAvionicsID);
    void (*destroy)(XPLMAvionicsID);
    void (*get_geometry)(XPLMAvionicsID,int *,int *,int *,int *);
    void (*set_geometry)(XPLMAvionicsID,int,int,int,int);
} mfda_av={.loaded=1,.needs_drawing=redraw,.cursor_over=absent,.is_bound=bound};
/* Pop-up window: X-Plane's default geometry is the whole 1512x982 screen. */
static int popup_shown,geom[4]={0,982,1512,0};
static void show_popup(XPLMAvionicsID id,int v){assert(id);popup_shown=v;}
static int popup_visible(XPLMAvionicsID id){assert(id);return popup_shown;}
static void get_geom(XPLMAvionicsID id,int *l,int *t,int *r,int *b){assert(id);*l=geom[0];*t=geom[1];*r=geom[2];*b=geom[3];}
static void set_geom(XPLMAvionicsID id,int l,int t,int r,int b){assert(id);geom[0]=l;geom[1]=t;geom[2]=r;geom[3]=b;}
void XPLMGetScreenBoundsGlobal(int *l,int *t,int *r,int *b){*l=0;*t=982;*r=1512;*b=0;}
static int xpkit_draw_view(int side,float x,float y,float w,float h) {
    assert(side==2||side==3);assert(x==0&&y==0&&w==768&&h==1024);++views;
    if(xpkit_health[side+4])xpkit_uploaded[side]=2;
    return xpkit_health[side+4];
}
static void draw_nd(const Side *s,float x,float y,const char *label){(void)s;(void)x;(void)y;(void)label;++fallbacks;}
static void vertex(float x,float y){assert(isfinite(x)&&isfinite(y));assert(fabsf(x)<=44&&fabsf(y)<=44);++vertices;}
static void translate(float x,float y,float z){assert(z==0);translation_x=x;translation_y=y;}
static void rotate(float angle,float x,float y,float z){assert(angle==-90&&x==0&&y==0&&z==1);++rotations;}
#define glBegin(...) ((void)0)
#define glEnd() ((void)0)
#define glColor4f(...) ((void)0)
#define glVertex2f(x,y) vertex(x,y)
#define glPushMatrix() (++matrices)
#define glPopMatrix() (--matrices)
#define glTranslatef(x,y,z) translate(x,y,z)
#define glRotatef(a,x,y,z) rotate(a,x,y,z)
#define glScalef(...) ((void)0)
#define glIsEnabled(...) GL_FALSE
#define glDisable(...) ((void)0)
#define glEnable(...) ((void)0)
#define glClearColor(...) ((void)0)
#define glColorMask(...) ((void)0)
#define glBlendFunc(...) ((void)0)
#define glClear(...) ((void)0)
#define XPLMSetGraphicsState(...) ((void)0)
#include "a380x_xpkit_nd_input.inc"

int main(void) {
    XpKitNd *d=&xpkit_nd[0];d->device=(XPLMAvionicsID)1;
    xpkit_health[6]=xpkit_health[7]=1;
    assert(xpkit_nd_cursor(320,700,d)==xplm_CursorHidden);
    assert(d->x==320&&d->y==323&&d->inside&&d->serial==1&&redraws==1);
    assert(xpkit_nd_cursor(320,700,d)==xplm_CursorHidden);
    assert(d->serial==1&&redraws==1);
    assert(xpkit_nd_touch(320,700,xplm_MouseDown,d)==1);
    assert(d->down&&d->events[1].type==1);
    assert(xpkit_nd_touch(325,695,xplm_MouseDrag,d)==1);
    assert(d->down&&d->events[2].type==0);
    assert(xpkit_nd_touch(325,695,xplm_MouseUp,d)==1);
    assert(!d->down&&d->events[3].type==2);
    assert(xpkit_nd_right(0,0,xplm_MouseDown,d)==0);
    int count=redraws;
    xpkit_nd_draw_device(d);
    assert(views==1&&vertices==72&&matrices==0&&rotations==0);
    assert(translation_x==325&&translation_y==695&&redraws==count);
    assert(atomic_load(&memory.nd_profile[0].draw_count)==1);
    assert(d->presented_frame==2);
    d->draw_live=d->draw_powered=1;
    assert(!xpkit_nd_frame_pending(d,2,1,1));
    xpkit_uploaded[2]=4; /* Legacy atlas uploaded a new texture first. */
    assert(xpkit_nd_frame_pending(d,4,1,1));
    assert(xpkit_nd_frame_pending(d,2,0,1));
    assert(xpkit_nd_frame_pending(d,2,1,0));
    assert(!atomic_load(&memory.nd_profile[0].snapshot_count)&&!atomic_load(&memory.nd_profile[0].upload_count));
    d->side=1;xpkit_nd_draw_cursor(d);assert(rotations==1&&matrices==0);
    xpkit_health[7]=0;vertices=0;xpkit_nd_draw_cursor(d);assert(vertices==0);
    assert(xpkit_nd_cursor(100,100,d)==xplm_CursorDefault);
    d->side=0;d->seen=0;xpkit_nd_poll();assert(!d->inside&&d->events[4].type==4);
    assert(xpkit_nd_scroll(100,100,1,1,d)==0);
    assert(xpkit_nd_scroll(100,100,0,1,d)==1&&d->events[5].type==3);
    xpkit_nd[1].side=1;xpkit_nd[1].device=(XPLMAvionicsID)2;
    xpkit_nd[0].inside=xpkit_nd[0].down=xpkit_nd[0].seen=1;
    xpkit_nd[1].inside=xpkit_nd[1].down=xpkit_nd[1].seen=1;
    xpkit_health[6]=xpkit_health[7]=1;count=redraws;
    xpkit_nd_renderer_lost();
    assert(redraws==count+2&&!xpkit_health[6]&&!xpkit_health[7]);
    for(int side=0;side<2;++side)assert(!xpkit_nd[side].inside&&!xpkit_nd[side].down&&!xpkit_nd[side].seen);
    vertices=0;xpkit_nd_draw_device(d);assert(fallbacks==1&&vertices==0&&matrices==0);
    assert(!d->presented_frame&&!xpkit_nd_frame_pending(d,4,0,1));
    assert(!atomic_load(&memory.nd_profile[0].snapshot_count)&&!atomic_load(&memory.nd_profile[0].upload_count));
    assert(XPKIT_ND_ZOOM_DEFAULT==1&&XPKIT_ND_ZOOM_MAX==6);
    d->zoom=XPKIT_ND_ZOOM_DEFAULT;
    d->command[3]=(XPLMCommandRef)3;d->command[4]=(XPLMCommandRef)4;
    for(int i=0;i<10;++i)xpkit_nd_command(d->command[4],xplm_CommandBegin,d);
    assert(d->zoom==6);
    for(int i=0;i<10;++i)xpkit_nd_command(d->command[3],xplm_CommandBegin,d);
    assert(d->zoom==0);
    atomic_store(&memory.zoom_delta[0],99);xpkit_nd_poll();assert(d->zoom==6);
    atomic_store(&memory.zoom_delta[0],-99);xpkit_nd_poll();assert(d->zoom==0);
    /* Pop-up: 3:4 at 80 % of the screen height instead of X-Plane's full-screen default. */
    mfda_av.set_popup_visible=show_popup;mfda_av.is_popup_visible=popup_visible;
    mfda_av.get_geometry=get_geom;mfda_av.set_geometry=set_geom;
    d->command[0]=(XPLMCommandRef)10;xpkit_nd_command(d->command[0],xplm_CommandBegin,d);
    assert(popup_shown&&geom[1]-geom[3]==785&&geom[2]-geom[0]==785*768/1024);
    xpkit_nd_command(d->command[0],xplm_CommandBegin,d);assert(!popup_shown);
    puts("Native ND input: ordered clicks, seven airport zoom levels, native cursor geometry, separate texture/presentation scheduling, no cursor uploads, immediate helper-exit fallback and 3:4 pop-up passed");
    return 0;
}
