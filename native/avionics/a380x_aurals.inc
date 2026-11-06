/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * X-Plane adapter of the cockpit aurals (a380x_aurals.h, docs/COCKPIT_AURALS.md).
 *
 * Samples: plugins/A380XAvionics/sounds/cockpit/<name>.wav (mono 16-bit PCM, prepared from the user's
 * "A380 sounds cockpit" set, see sounds/cockpit/provenance.json). They play on X-Plane's interior bus
 * and only while the camera is in the cockpit: never in an outside view, in the cabin, paused or in
 * replay. Loops (CRC, continuous C-chord, stall) stop when the view leaves the cockpit and resume
 * when it returns while still requested; one-shot callouts that fall outside are not replayed.
 *
 * Unity-build include after a380x_fws_adapter.inc (FWS aural requests), a380x_fms_adapter.inc
 * (minimums), the guidance state and a380x_cockpit_lever.inc (view / pause / replay helpers).
 */
#include "a380x_aurals.h"

typedef struct { short *data; uint32_t bytes; int rate; } AuralSample;

static const struct { const char *file; float volume; } aural_files[AUR_COUNT] = {
    {"callout_2500.wav", 0.9f}, {"callout_2000.wav", 0.9f}, {"callout_1000.wav", 0.9f},
    {"callout_500.wav", 0.9f}, {"callout_400.wav", 0.9f}, {"callout_300.wav", 0.9f},
    {"callout_200.wav", 0.9f}, {"callout_100.wav", 0.9f}, {"callout_90.wav", 0.9f},
    {"callout_80.wav", 0.9f}, {"callout_70.wav", 0.9f}, {"callout_60.wav", 0.9f},
    {"callout_50.wav", 0.9f}, {"callout_40.wav", 0.9f}, {"callout_30.wav", 0.9f},
    {"callout_20.wav", 0.9f}, {"callout_10.wav", 0.9f}, {"callout_5.wav", 0.9f},
    {"hundred_above.wav", 0.9f}, {"minimum.wav", 0.9f}, {"retard.wav", 0.9f}, {"v1.wav", 0.9f},
    {"stall.wav", 1.0f}, {"crc_master_warning.wav", 0.8f}, {"cchord.wav", 0.45f},
    {"triple_click.wav", 0.6f}, {"seatbelt_chime.wav", 0.5f}, {"cavalry_charge.wav", 0.8f},
};

/* Channel slots: 0 = the voice, 1 + i = one-shot sample i, 1 + AUR_COUNT + i = loop of sample i. */
#define AUR_SLOTS (1 + 2 * AUR_COUNT)
#define AUR_VOICE_SLOT 0
#define AUR_SHOT_SLOT(i) (1 + (i))
#define AUR_LOOP_SLOT(i) (1 + AUR_COUNT + (i))

static struct {
    Aurals core;
    AuralSample sample[AUR_COUNT];
    int loaded;
    FMOD_CHANNEL *channel[AUR_SLOTS];
    unsigned generation[AUR_SLOTS];   /* a completion callback only clears the play it belongs to */
    XPLMDataRef seatbelt, tla[4], stall_raw;
    int audible;
} aur;

static int aural_parse_wav(AuralSample *s, const unsigned char *b, size_t n)
{
    unsigned bytes = 0;
    memset(s, 0, sizeof(*s));
    if (!aurals_parse_wav(b, n, &s->data, &bytes, &s->rate)) return 0;
    s->bytes = bytes;
    return 1;
}

static int aural_load(AuralSample *s, const char *file)
{
    char root[1024], path[1200];
    unsigned char *buffer;
    FILE *f;
    long length;
    int i, ok = 0;
    memset(s, 0, sizeof(*s));
    root[0] = 0;
    XPLMGetPluginInfo(XPLMGetMyID(), NULL, root, NULL, NULL);
    for (i = 0; i < 2; ++i) { /* .../A380XAvionics/<platform>/A380XAvionics.xpl -> .../A380XAvionics */
        char *cut = strrchr(root, '/');
        if (!cut) cut = strrchr(root, '\\');
        if (!cut) return 0;
        *cut = 0;
    }
    if (snprintf(path, sizeof(path), "%s/sounds/cockpit/%s", root, file) >= (int)sizeof(path)) return 0;
    f = fopen(path, "rb");
    if (!f) return 0;
    if (fseek(f, 0, SEEK_END) == 0 && (length = ftell(f)) > 44 && length <= 2000000 && fseek(f, 0, SEEK_SET) == 0) {
        buffer = (unsigned char *)malloc((size_t)length);
        if (buffer && fread(buffer, 1, (size_t)length, f) == (size_t)length) ok = aural_parse_wav(s, buffer, (size_t)length);
        free(buffer);
    }
    fclose(f);
    return ok;
}

static void aural_done(void *refcon, FMOD_RESULT status)
{
    uintptr_t r = (uintptr_t)refcon;
    unsigned slot = (unsigned)(r & 0xffu), generation = (unsigned)(r >> 8);
    (void)status;
    if (slot < AUR_SLOTS && aur.generation[slot] == generation) aur.channel[slot] = NULL;
}

static void aural_stop(int slot)
{
    FMOD_CHANNEL *c = aur.channel[slot];
    aur.channel[slot] = NULL;
    aur.generation[slot] = (aur.generation[slot] + 1u) & 0xffffffu;   /* a late callback is ignored */
    if (c) XPLMStopAudio(c);
}

static void aural_stop_all(void)
{
    int slot;
    for (slot = 0; slot < AUR_SLOTS; ++slot) aural_stop(slot);
}

static void aural_start(int s, int loop, int slot)
{
    const AuralSample *x = &aur.sample[s];
    aural_stop(slot);
    if (!x->data) return;
    aur.channel[slot] = XPLMPlayPCMOnBus(x->data, x->bytes, FMOD_SOUND_FORMAT_PCM16, x->rate, 1, loop,
        xplm_AudioInterior, aural_done, (void *)(((uintptr_t)aur.generation[slot] << 8) | (uintptr_t)slot));
    if (aur.channel[slot]) XPLMSetAudioVolume(aur.channel[slot], aural_files[s].volume);
}

/* The crew hears the cockpit loudspeakers only with the camera in the cockpit. */
static int aural_cockpit_view(void)
{
    /* Inside view with the camera within 4 m of the pilot's eye point: the flight deck, not the cabin
     * (cockpit_sound_world measures that distance for inside views). */
    static const float eye[3] = {0.0f, 0.0f, 0.0f};
    FMOD_VECTOR at;
    if (!cockpit_sound_allowed() || cockpit_sound_value(CS_VIEW_EXTERNAL) != 0.0) return 0;
    return cockpit_sound_world(eye, &at);
}

/* MASTER WARNING: the FWS CRC, or the cavalry charge of an A/P OFF warning, replaces the
 * glareshield's synthetic tone. */
static int aurals_crc_covers(void)
{
    if (!fwsa_available() || !fwsa_fws.powered) return 0;
    return (aur.sample[AUR_CRC].data && fwsa_fws.aural_crc_active) ||
           (aur.sample[AUR_CAVALRY].data && fwsa_fws.out.aural_cavalry_charge);
}

static void aurals_reset(void)
{
    int i;
    aural_stop_all();
    aurals_init(&aur.core);
    for (i = 0; i < AUR_COUNT; ++i)
        if (aur.sample[i].data && aur.sample[i].rate > 0)
            aurals_set_length(&aur.core, (AurSample)i, (double)aur.sample[i].bytes / 2.0 / aur.sample[i].rate);
}

static void aurals_start(void)
{
    char line[160];
    int i;
    cockpit_sound_bind();
    if (!aur.loaded) {
        for (i = 0; i < AUR_COUNT; ++i) aur.loaded += aural_load(&aur.sample[i], aural_files[i].file);
        snprintf(line, sizeof(line), "A380X cockpit aurals: %d/%d samples loaded (interior bus, cockpit view only)\n",
                 aur.loaded, AUR_COUNT);
        XPLMDebugString(line);
    }
    aurals_reset();
}

static void aurals_stop(void)
{
    int i;
    aural_stop_all();
    for (i = 0; i < AUR_COUNT; ++i) { free(aur.sample[i].data); memset(&aur.sample[i], 0, sizeof(aur.sample[i])); }
    aur.loaded = 0;
}

static void aurals_update_frame(float dt)
{
    static const char *const tla_names[4] = {"a380x/xp12/thrust/tla_1", "a380x/xp12/thrust/tla_2",
                                             "a380x/xp12/thrust/tla_3", "a380x/xp12/thrust/tla_4"};
    AurInputs in;
    AurOutputs out;
    int i, audible;
    if (!aur.loaded) return;
    memset(&in, 0, sizeof(in));
    if (!aur.seatbelt) aur.seatbelt = XPLMFindDataRef("sim/cockpit2/switches/fasten_seat_belts");
    for (i = 0; i < 4; ++i) {
        if (!aur.tla[i]) aur.tla[i] = XPLMFindDataRef(tla_names[i]);
        if (aur.tla[i] && XPLMGetDataf(aur.tla[i]) > 2.0f) in.thrust_above_idle = 1;
    }
    in.dt = dt;
    in.powered = fwsa_available() && fwsa_fws.powered;
    in.ra_valid = refs.side[0].radio_altitude_valid; in.ra_ft = refs.side[0].radio_altitude_value;
    in.on_ground = refs.on_ground && XPLMGetDatai(refs.on_ground) != 0;
    in.baro_valid = refs.side[0].altitude_valid; in.baro_ft = refs.side[0].altitude_value;
    in.dh_valid = fmsa_fms.out.dh_valid; in.dh_ft = fmsa_fms.out.dh_ft;
    in.mda_valid = fmsa_fms.out.mda_valid; in.mda_ft = fmsa_fms.out.mda_ft;
    in.ap_engaged = guidance.ap1 || guidance.ap2;
    in.crc = fwsa_fws.aural_crc_active;
    in.cchord_once = fwsa_fws.out.aural_cchord_once; in.cchord_cont = fwsa_fws.out.aural_cchord_cont;
    in.v1 = fwsa_fws.out.aural_v1; in.stall = fwsa_fws.out.stall_warning;
    in.cavalry = fwsa_fws.out.aural_cavalry_charge;
    if (!aur.stall_raw) aur.stall_raw = XPLMFindDataRef("sim/cockpit2/annunciators/stall_warning");
    in.aoa_stall_warning = aur.stall_raw && XPLMGetDatai(aur.stall_raw) != 0;
    in.cas_valid = guidance_inputs.airdata_valid; in.cas_kt = guidance_inputs.ias_knots;
    in.vsw_kt = pfd_guidance.stall_warning_knots;
    in.mode_reversion = guidance.mode_reversion;
    in.seatbelt_valid = aur.seatbelt != NULL;
    in.seatbelt_sign = aur.seatbelt ? XPLMGetDatai(aur.seatbelt) : 0;
    aurals_update(&aur.core, &in, &out);

    audible = aural_cockpit_view();
    if (!audible) {
        /* Outside the cockpit, paused or in replay: silence; nothing is replayed later. */
        if (aur.audible) aural_stop_all();
        aurals_voice_silenced(&aur.core);
        aur.audible = 0;
        return;
    }
    aur.audible = 1;
    if (out.stop_voice) aural_stop(AUR_VOICE_SLOT);
    for (i = 0; i < AUR_COUNT; ++i) {
        if (out.start[i]) aural_start(i, 0, aurals_is_voice((AurSample)i) ? AUR_VOICE_SLOT : AUR_SHOT_SLOT(i));
        if (out.loop[i] && !aur.channel[AUR_LOOP_SLOT(i)]) aural_start(i, 1, AUR_LOOP_SLOT(i));
        else if (!out.loop[i] && aur.channel[AUR_LOOP_SLOT(i)]) aural_stop(AUR_LOOP_SLOT(i));
    }
}
