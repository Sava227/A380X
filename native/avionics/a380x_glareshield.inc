/* SPDX-License-Identifier: GPL-3.0-or-later
 * Aircraft-local glareshield side-panel controls and alert indications.
 * ATC/stick-motion inputs are explicit until their FBW producers exist. land_mode is produced by
 * the native guidance (a380x_autoland.inc): LAND engaged with an AP in it, before ROLL OUT.
 */
#include "a380x_elec_power.h"
/* Native FWS master lights (a380x_fws_adapter.inc, included later in the unity build). */
static int fwsa_available(void);
static int fwsa_master_warning(void);
static int fwsa_master_caution(void);
typedef struct {
    int chrono_mode[2], chrono_seconds[2], chrono_frozen[2];
    float chrono_fraction[2], flash_time, atc_age;
    int stick_motion[2], takeover_held, priority;
    int warning, caution, warning_source, caution_source, warning_ack, caution_ack;
    int land_mode, autoland_warning, autoland_mask;
    int atc_pending, atc_urgent, atc_rang;
    int warning_lit, caution_lit, autoland_lit, atc_lit;
    int arrow[2], green[2];
    int volume[2], ptt[2];
    int clear_warning_pending, clear_caution_pending;
    XPLMDataRef master_warning, master_caution;
    XPLMCommandRef clear_warning, clear_caution, radio_ptt;
} GlareState;
static GlareState glare;

typedef struct { const char *name; int *value; int writable; XPLMDataRef handle; } GlareRef;
#define GR(name, field, write) {"a380x/xp12/glareshield/" name, &glare.field, write, NULL}
static GlareRef glare_refs[] = {
    GR("chrono_left_mode", chrono_mode[0], 0), GR("chrono_right_mode", chrono_mode[1], 0),
    GR("chrono_left_seconds", chrono_seconds[0], 0), GR("chrono_right_seconds", chrono_seconds[1], 0),
    GR("stick_left_moving", stick_motion[0], 1), GR("stick_right_moving", stick_motion[1], 1),
    GR("priority", priority, 0), GR("master_warning", warning, 0),
    GR("master_caution", caution, 0), GR("autoland_warning", autoland_warning, 0),
    GR("land_mode", land_mode, 0), GR("atc_pending", atc_pending, 0),
    GR("volume_left", volume[0], 1), GR("volume_right", volume[1], 1),
    GR("ptt_left", ptt[0], 0), GR("ptt_right", ptt[1], 0),
    GR("master_warning_lit", warning_lit, 0), GR("master_caution_lit", caution_lit, 0),
    GR("autoland_lit", autoland_lit, 0), GR("atc_lit", atc_lit, 0),
    GR("priority_arrow_left", arrow[0], 0), GR("priority_arrow_right", arrow[1], 0),
    GR("priority_green_left", green[0], 0), GR("priority_green_right", green[1], 0),
};
#undef GR
static int glare_get(void *refcon) { return *(((GlareRef *)refcon)->value); }
static void glare_set(void *refcon, int value)
{
    GlareRef *ref=(GlareRef *)refcon;
    if (!ref->writable) return;
    if (ref->value==&glare.volume[0] || ref->value==&glare.volume[1])
        *ref->value=value<0 ? 0 : value>100 ? 100 : value;
    else *ref->value=value!=0;
}

typedef struct { const char *name; int action, side; XPLMCommandRef handle; } GlareCommand;
enum { GL_CHRONO, GL_TAKEOVER, GL_WARN, GL_CAUT, GL_ATC_ACK,
       GL_ATC_NORMAL, GL_ATC_URGENT, GL_VOL_UP, GL_VOL_DOWN, GL_PTT };
#define GC(name, action, side) {"a380x/xp12/glareshield/" name, action, side, NULL}
static GlareCommand glare_commands[] = {
    GC("chrono_left",GL_CHRONO,0), GC("chrono_right",GL_CHRONO,1),
    GC("takeover_left",GL_TAKEOVER,0), GC("takeover_right",GL_TAKEOVER,1),
    GC("master_warning_left",GL_WARN,0), GC("master_warning_right",GL_WARN,1),
    GC("master_caution_left",GL_CAUT,0), GC("master_caution_right",GL_CAUT,1),
    GC("atc_ack_left",GL_ATC_ACK,0), GC("atc_ack_right",GL_ATC_ACK,1),
    GC("atc_receive_normal",GL_ATC_NORMAL,0), GC("atc_receive_urgent",GL_ATC_URGENT,0),
    GC("volume_left_up",GL_VOL_UP,0), GC("volume_left_down",GL_VOL_DOWN,0),
    GC("volume_right_up",GL_VOL_UP,1), GC("volume_right_down",GL_VOL_DOWN,1),
    GC("ptt_left",GL_PTT,0), GC("ptt_right",GL_PTT,1),
};
#undef GC

static int aurals_crc_covers(void);   /* a380x_aurals.inc: the FWS CRC sample plays this warning */
static short glare_tone[22050];
static FMOD_CHANNEL *glare_audio;
typedef struct { short *data; uint32_t bytes; } GlareVoice;
static GlareVoice glare_voice[2];
static void glare_audio_done(void *refcon, FMOD_RESULT status)
{
    (void)refcon; (void)status; glare_audio=NULL;
}
static void glare_stop_audio(void)
{
    if (glare_audio) { XPLMStopAudio(glare_audio); glare_audio=NULL; }
}
static void glare_play_tone(int frequency, int pulses, int side)
{
    int i, count=0, rate=22050;
    float volume=(float)(side>=0 ? glare.volume[side] :
        (glare.volume[0]+glare.volume[1])/2)/100.0f;
    if (volume<=0.0f) return;
    glare_stop_audio();
    for (i=0;i<rate && count<rate;++i) {
        float t=(float)i/rate;
        int pulse=(int)(t*4.0f);
        float in_pulse=fmodf(t,0.25f);
        if (pulse>=pulses) break;
        glare_tone[i]=in_pulse<0.11f ? (short)(4800.0f*sinf(2.0f*(float)M_PI*frequency*t)) : 0;
        ++count;
    }
    if (count) {
        glare_audio=XPLMPlayPCMOnBus(glare_tone,(uint32_t)(count*sizeof(short)),
            FMOD_SOUND_FORMAT_PCM16,rate,1,0,xplm_AudioInterior,glare_audio_done,NULL);
        if (glare_audio) XPLMSetAudioVolume(glare_audio,volume);
    }
}
static void glare_play_voice(int side)
{
    GlareVoice *voice=&glare_voice[side];
    float volume=(float)glare.volume[side]/100.0f;
    if (!voice->data || !voice->bytes || volume<=0.0f) return;
    glare_stop_audio();
    glare_audio=XPLMPlayPCMOnBus(voice->data,voice->bytes,FMOD_SOUND_FORMAT_PCM16,
        22050,1,0,xplm_AudioInterior,glare_audio_done,NULL);
    if (glare_audio) XPLMSetAudioVolume(glare_audio,volume);
}
static void glare_load_voices(void)
{
    char path[1024], root[1024];
    char *cut;
    int i;
    XPLMGetPluginInfo(XPLMGetMyID(),NULL,root,NULL,NULL);
    cut=strrchr(root,'/'); if (!cut) cut=strrchr(root,'\\');
    if (!cut) return;
    *cut=0;
    cut=strrchr(root,'/'); if (!cut) cut=strrchr(root,'\\');
    if (!cut) return;
    *cut=0;
    for (i=0;i<2;++i) {
        FILE *file;
        long length;
        if (snprintf(path,sizeof(path),"%s/sounds/priority_%s.pcm",root,
            i ? "right" : "left") >= (int)sizeof(path)) continue;
        file=fopen(path,"rb");
        if (!file) continue;
        if (fseek(file,0,SEEK_END)!=0) { fclose(file); continue; }
        length=ftell(file);
        if (length<=0 || length>200000 || (length&1) || fseek(file,0,SEEK_SET)!=0) {
            fclose(file); continue;
        }
        glare_voice[i].data=(short *)malloc((size_t)length);
        if (glare_voice[i].data && fread(glare_voice[i].data,1,(size_t)length,file)==(size_t)length)
            glare_voice[i].bytes=(uint32_t)length;
        else { free(glare_voice[i].data); glare_voice[i].data=NULL; }
        fclose(file);
    }
}
static void glare_take_priority(int side)
{
    glare.priority=side+1;
    glare_play_voice(side);
}
static int glare_command_handler(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    GlareCommand *c=(GlareCommand *)refcon;
    int other=1-c->side;
    (void)command;
    if (!plugin_enabled) return 1;
    if (c->action==GL_PTT) {
        if (phase==xplm_CommandBegin && !glare.ptt[c->side]) {
            glare.ptt[c->side]=1;
            if (!glare.radio_ptt) glare.radio_ptt=XPLMFindCommand("xpilot/ptt");
            if (glare.radio_ptt && !glare.ptt[other]) XPLMCommandBegin(glare.radio_ptt);
        } else if (phase==xplm_CommandEnd && glare.ptt[c->side]) {
            glare.ptt[c->side]=0;
            if (glare.radio_ptt && !glare.ptt[other]) XPLMCommandEnd(glare.radio_ptt);
        }
        return 1;
    }
    if (c->action==GL_TAKEOVER) {
        if (phase==xplm_CommandBegin) {
            if (glare.priority==c->side+1) glare.priority=0;
            else { glare.takeover_held=c->side+1;
                if (glare.stick_motion[other]) glare_take_priority(c->side);
            }
            a380x_guidance_command(&guidance,&guidance_inputs,A380X_FCU_AP_DISCONNECT,0);
            release_guidance_controls();
        } else if (phase==xplm_CommandEnd && glare.takeover_held==c->side+1)
            glare.takeover_held=0;
        return 1;
    }
    if (phase!=xplm_CommandBegin) return 1;
    switch (c->action) {
    case GL_CHRONO:
        if (glare.chrono_mode[c->side]==0) {
            glare.chrono_seconds[c->side]=0; glare.chrono_fraction[c->side]=0;
            glare.chrono_mode[c->side]=1;
        } else if (glare.chrono_mode[c->side]==1) {
            glare.chrono_frozen[c->side]=glare.chrono_seconds[c->side];
            glare.chrono_mode[c->side]=2;
        } else glare.chrono_mode[c->side]=0;
        break;
    case GL_WARN:
        glare.warning_ack=1; glare_stop_audio();
        if (glare.clear_warning && !glare.clear_warning_pending) {
            XPLMCommandBegin(glare.clear_warning); glare.clear_warning_pending=1;
        }
        break;
    case GL_CAUT:
        glare.caution_ack=1; glare_stop_audio();
        if (glare.clear_caution && !glare.clear_caution_pending) {
            XPLMCommandBegin(glare.clear_caution); glare.clear_caution_pending=1;
        }
        break;
    case GL_ATC_ACK:
        glare.atc_pending=0; glare.atc_rang=0; glare_stop_audio(); break;
    case GL_ATC_NORMAL: case GL_ATC_URGENT:
        glare.atc_pending=1; glare.atc_urgent=c->action==GL_ATC_URGENT;
        glare.atc_age=0; glare.atc_rang=0; break;
    case GL_VOL_UP: case GL_VOL_DOWN:
        glare.volume[c->side]+=c->action==GL_VOL_UP ? 5 : -5;
        if (glare.volume[c->side]<0) glare.volume[c->side]=0;
        if (glare.volume[c->side]>100) glare.volume[c->side]=100;
        break;
    default: break;
    }
    return 1;
}

static void glare_update(float dt)
{
    int i, mask=0, suppressed;
    float ra=refs.side[0].radio_altitude_value;
    if (dt<0 || dt>0.25f) dt=0;
    if (glare.clear_warning_pending) {
        XPLMCommandEnd(glare.clear_warning); glare.clear_warning_pending=0;
    }
    if (glare.clear_caution_pending) {
        XPLMCommandEnd(glare.clear_caution); glare.clear_caution_pending=0;
    }
    glare.flash_time+=dt;
    for (i=0;i<2;++i) if (glare.chrono_mode[i]==1) {
        glare.chrono_fraction[i]+=dt;
        while (glare.chrono_fraction[i]>=1.0f) {
            ++glare.chrono_seconds[i]; glare.chrono_fraction[i]-=1.0f;
        }
    }
    if (glare.takeover_held && glare.stick_motion[2-glare.takeover_held]
        && glare.priority!=glare.takeover_held)
        glare_take_priority(glare.takeover_held-1);
    /* With the native FWS the lights are its masterWarningOutput / masterCautionOutput: the pbs
     * reach the FWS (pass-through tap) which keeps non-cancellable warnings lit, so no local ack. */
    i=fwsa_available() ? fwsa_master_warning() : XPLMGetDatai(glare.master_warning)!=0;
    if (i && !glare.warning_source) { glare.warning_ack=0; if (!aurals_crc_covers()) glare_play_tone(640,3,-1); }
    if (!i) glare.warning_ack=0;
    glare.warning_source=i;
    glare.warning=i && (fwsa_available() || !glare.warning_ack);
    i=fwsa_available() ? fwsa_master_caution() : XPLMGetDatai(glare.master_caution)!=0;
    if (i && !glare.caution_source) { glare.caution_ack=0; glare_play_tone(520,2,-1); }
    if (!i) glare.caution_ack=0;
    glare.caution_source=i;
    glare.caution=i && (fwsa_available() || !glare.caution_ack);
    /* AUTOLAND warning window: an autoland (AP engaged in LAND at some point) until ROLL OUT. */
    glare.land_mode=guidance.land.land_active && guidance.land.autoland && !guidance.land.rollout_active
        && !guidance_inputs.on_ground;
    if (glare.land_mode && refs.side[0].radio_altitude_valid && ra<200.0f) {
        if (!guidance.ap1 && !guidance.ap2) mask|=1;
        if (ra>15.0f && (!refs.side[0].loc_valid || fabsf(refs.side[0].loc_dots)>0.25f)) mask|=2;
        if (ra>100.0f && (!refs.side[0].gs_valid || fabsf(refs.side[0].gs_dots)>1.0f)) mask|=4;
        if (refs.side[1].radio_altitude_valid &&
            fabsf(ra-refs.side[1].radio_altitude_value)>15.0f) mask|=8;
    }
    if (mask & ~glare.autoland_mask) glare_play_tone(1500,3,-1);
    glare.autoland_mask=mask; glare.autoland_warning=mask!=0;
    /* FBW phases 3-7 cover takeoff/climbout, 9-10 landing/rollout. */
    suppressed=(published.flight_phase>=3 && published.flight_phase<=7) ||
        published.flight_phase==9 || published.flight_phase==10;
    if (glare.atc_pending) {
        glare.atc_age+=dt;
        if (!suppressed && !glare.atc_rang &&
            (glare.atc_urgent || glare.atc_age>=10.0f)) {
            glare_play_tone(840,2,-1); glare.atc_rang=1;
        }
    }
    i=fmodf(glare.flash_time,1.0f)<0.5f;
    {
        /* A380 network: FWS 1/2 on DC ESS / DC 2 light the attention getters; sidestick
         * priority lamps on AC 1 (CAPT) / AC 2 (F/O) (spec 6.4). Always lit without it. */
        int fws=a380x_power_any(A380X_ELN_BIT(A380X_ELN_DC_ESS)|A380X_ELN_BIT(A380X_ELN_DC2),1);
        int left=a380x_power_bus(A380X_ELN_AC1,1), right=a380x_power_bus(A380X_ELN_AC2,1);
        glare.warning_lit=fws && glare.warning && i;
        glare.caution_lit=fws && glare.caution;
        glare.autoland_lit=fws && glare.autoland_warning && i;
        glare.atc_lit=fws && glare.atc_pending && !suppressed && i;
        glare.arrow[0]=left && glare.priority==2;
        glare.arrow[1]=right && glare.priority==1;
        glare.green[0]=left && (glare.priority==1 ? glare.stick_motion[1] :
            glare.priority==0 && glare.stick_motion[0] && glare.stick_motion[1] && i);
        glare.green[1]=right && (glare.priority==2 ? glare.stick_motion[0] :
            glare.priority==0 && glare.stick_motion[0] && glare.stick_motion[1] && i);
    }
}
static void glare_shutdown(void)
{
    int i;
    if (glare.clear_warning_pending) XPLMCommandEnd(glare.clear_warning);
    if (glare.clear_caution_pending) XPLMCommandEnd(glare.clear_caution);
    if ((glare.ptt[0] || glare.ptt[1]) && glare.radio_ptt) XPLMCommandEnd(glare.radio_ptt);
    glare_stop_audio();
    for (i=0;i<2;++i) { free(glare_voice[i].data); glare_voice[i].data=NULL; glare_voice[i].bytes=0; }
    for (i=0;i<ARRAY_LEN(glare_commands);++i) if (glare_commands[i].handle) {
        XPLMUnregisterCommandHandler(glare_commands[i].handle,glare_command_handler,1,&glare_commands[i]);
        glare_commands[i].handle=NULL;
    }
    for (i=0;i<ARRAY_LEN(glare_refs);++i) if (glare_refs[i].handle) {
        XPLMUnregisterDataAccessor(glare_refs[i].handle); glare_refs[i].handle=NULL;
    }
}
static int glare_init(void)
{
    int i;
    memset(&glare,0,sizeof(glare)); glare.volume[0]=glare.volume[1]=70;
    glare.master_warning=XPLMFindDataRef("sim/cockpit2/annunciators/master_warning");
    glare.master_caution=XPLMFindDataRef("sim/cockpit2/annunciators/master_caution");
    glare.clear_warning=XPLMFindCommand("sim/annunciator/clear_master_warning");
    glare.clear_caution=XPLMFindCommand("sim/annunciator/clear_master_caution");
    glare.radio_ptt=XPLMFindCommand("xpilot/ptt");
    if (!glare.master_warning || !glare.master_caution) return 0;
    glare_load_voices();
    for (i=0;i<ARRAY_LEN(glare_refs);++i) {
        GlareRef *r=&glare_refs[i];
        if (XPLMFindDataRef(r->name)) { glare_shutdown(); return 0; }
        r->handle=XPLMRegisterDataAccessor(r->name,xplmType_Int,r->writable,
            glare_get,r->writable ? glare_set : NULL,NULL,NULL,NULL,NULL,NULL,NULL,
            NULL,NULL,NULL,NULL,r,NULL);
        if (!r->handle) { glare_shutdown(); return 0; }
    }
    for (i=0;i<ARRAY_LEN(glare_commands);++i) {
        GlareCommand *c=&glare_commands[i];
        c->handle=XPLMCreateCommand(c->name,c->name);
        if (!c->handle) { glare_shutdown(); return 0; }
        XPLMRegisterCommandHandler(c->handle,glare_command_handler,1,c);
    }
    return 1;
}
