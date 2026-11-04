/* SPDX-License-Identifier: GPL-3.0-or-later
 * Overhead annunciator lamps (every overhead pushbutton legend outside the ELEC / EMER ELEC PWR
 * panel, a380x_elec_panel.h, and the ADIRS panel, a380x_adirs.c): XPLM-free lamp logic.
 *
 * FlyByWire A380X (GPL-3.0) model: a legend is lit when
 *     (its condition || ANN LT TEST) && its supply is powered,
 * at full brightness with ANN LT BRT and A380X_OVHD_DIM_LEVEL with ANN LT DIM
 * (FBW FBW_Push_Toggle SEQn_EMISSIVE_CODE, A32NX_Interior_Generics.xml). The supply of every
 * pushbutton is FBW's SEQ_POWERED bus (A380_Cockpit_Behavior.xml, translated to the real bus
 * names, docs/ELEC_SYSTEM_SPEC.md 0.3); the table is generated from tools/ovhd_lamps_contract.py
 * into a380x_ovhd_lamps_table.h. The X-Plane adapter is a380x_ovhd_lamps.inc
 * (docs/OVERHEAD_LAMPS.md).
 */
#ifndef A380X_OVHD_LAMPS_H
#define A380X_OVHD_LAMPS_H
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "a380x_elec.h"

typedef struct {
    const char *id, *node;          /* dataref id (lamp/<id>_upper|_lower), FBW glTF pushbutton */
    unsigned char legend[2];        /* A380X_OVHD_LEGEND_*: [0] SEQ1 upper, [1] SEQ2 lower */
    unsigned char source[2];        /* A380X_OVHD_SRC_* condition of each legend */
    unsigned char index[2];         /* engine / pump / pack index of an indexed condition */
    uint32_t any_of, all_of;        /* supply: any of these buses powered, and all of these */
} A380XOvhdLampInfo;
#include "a380x_ovhd_lamps_table.h"

#define A380X_OVHD_DIM_LEVEL 0.3f   /* same as the ELEC panel (a380x_elec_panel.h) */
/* Lamp supply setting (a380x/xp12/ovhd/settings/lamp_supply). */
enum { A380X_OVHD_SUPPLY_FBW = 0, A380X_OVHD_SUPPLY_SIMPLE = 1 };
/* ANN LT switch positions (FBW L:A32NX_OVHD_INTLT_ANN). */
enum { A380X_OVHD_ANN_TEST = 0, A380X_OVHD_ANN_BRT = 1, A380X_OVHD_ANN_DIM = 2 };
#define A380X_OVHD_ANY_DC (A380X_ELN_BIT(A380X_ELN_DC_ESS) | A380X_ELN_BIT(A380X_ELN_DC1) | \
                           A380X_ELN_BIT(A380X_ELN_DC2) | A380X_ELN_BIT(A380X_ELN_DC_APU))

typedef struct {
    /* system states (adapter: native network / X-Plane datarefs the pushbuttons drive) */
    unsigned char apu_master, apu_failed, apu_start, apu_avail, apu_bleed;
    unsigned char eng_bleed[4], eng_bleed_failed[4], pack[2];
    unsigned char eng_ai[4], eng_ai_failed[4], wing_ai, wing_ai_failed, window_heat;
    unsigned char feed_pump[4], crew_oxy;
    unsigned char fault[A380X_OVHD_PB_COUNT];   /* writable fail/<id> inputs */
    unsigned char pb[A380X_OVHD_PB_COUNT];      /* latched position of a pushbutton with no simulated
                                                   system (1 = pressed in), a380x/xp12/ovhd/pb/<id> */
    /* supply */
    uint32_t buses;        /* powered native buses (A380X_ELN_BIT) */
    int network;           /* buses valid (native A380 network running) */
    int legacy_power;      /* without the network: any stock bus >= 18 V */
    int supply_mode;       /* A380X_OVHD_SUPPLY_* */
    int simple_power;      /* simplified supply: any DC bus live or a battery pb AUTO */
    int ann_lt;            /* A380X_OVHD_ANN_* */
} A380XOvhdInputs;

static inline int a380x_ovhd_find(const char *id)
{
    int pb;
    for (pb = 0; id && pb < A380X_OVHD_PB_COUNT; ++pb) if (!strcmp(a380x_ovhd_pbs[pb].id, id)) return pb;
    return -1;
}

/* FBW per-legend supply, or the simplified / legacy supply. */
static inline int a380x_ovhd_powered(const A380XOvhdInputs *in, int pb)
{
    const A380XOvhdLampInfo *info;
    if (!in || pb < 0 || pb >= A380X_OVHD_PB_COUNT) return 0;
    if (!in->network) return in->legacy_power != 0;
    if (in->supply_mode == A380X_OVHD_SUPPLY_SIMPLE) return in->simple_power != 0;
    info = &a380x_ovhd_pbs[pb];
    if (info->all_of && (in->buses & info->all_of) != info->all_of) return 0;
    return !info->any_of || (in->buses & info->any_of) != 0;
}

/* The legend's own condition (without ANN LT TEST and supply). */
static inline int a380x_ovhd_condition(const A380XOvhdInputs *in, int pb, int seq)
{
    const A380XOvhdLampInfo *info;
    int i;
    if (!in || pb < 0 || pb >= A380X_OVHD_PB_COUNT || seq < 0 || seq > 1) return 0;
    info = &a380x_ovhd_pbs[pb];
    i = info->index[seq] & 3;
    switch (info->source[seq]) {
    case A380X_OVHD_SRC_FAULT: return in->fault[pb] != 0;
    case A380X_OVHD_SRC_APU_MASTER_ON: return in->apu_master != 0;
    /* FBW: the ECB fault (auto shutdown) shows while MASTER SW is on; port: X-Plane APU failure. */
    case A380X_OVHD_SRC_APU_MASTER_FAULT: return in->fault[pb] || (in->apu_master && in->apu_failed);
    case A380X_OVHD_SRC_APU_START_ON: return in->apu_start && !in->apu_avail;
    case A380X_OVHD_SRC_APU_AVAIL: return in->apu_avail != 0;
    case A380X_OVHD_SRC_APU_BLEED_ON: return in->apu_bleed != 0;
    case A380X_OVHD_SRC_ENG_BLEED_OFF: return !in->eng_bleed[i];
    case A380X_OVHD_SRC_ENG_BLEED_FAULT: return in->fault[pb] || (in->eng_bleed[i] && in->eng_bleed_failed[i]);
    case A380X_OVHD_SRC_PACK_OFF: return !in->pack[i & 1];
    case A380X_OVHD_SRC_ENG_AI_ON: return in->eng_ai[i] != 0;
    case A380X_OVHD_SRC_ENG_AI_FAULT: return in->fault[pb] || (in->eng_ai[i] && in->eng_ai_failed[i]);
    case A380X_OVHD_SRC_WING_AI_ON: return in->wing_ai != 0;
    case A380X_OVHD_SRC_WING_AI_FAULT: return in->fault[pb] || (in->wing_ai && in->wing_ai_failed);
    case A380X_OVHD_SRC_WINDOW_HEAT_ON: return in->window_heat != 0;
    case A380X_OVHD_SRC_FEED_PUMP_OFF: return !in->feed_pump[i];
    case A380X_OVHD_SRC_CREW_OXY_OFF: return !in->crew_oxy;
    case A380X_OVHD_SRC_PB_OFF: return !in->pb[pb];        /* OFF while released */
    case A380X_OVHD_SRC_PB_ON: return in->pb[pb] != 0;     /* ON / MAN / OVRD / DISC while pressed in */
    default: return 0;                                   /* NONE, TEST: ANN LT TEST only */
    }
}

/* Lamp brightness 0..1 (0 = dark) of a pushbutton's upper (seq 0) or lower (seq 1) legend. */
static inline float a380x_ovhd_lamp(const A380XOvhdInputs *in, int pb, int seq)
{
    float level;
    if (!in || pb < 0 || pb >= A380X_OVHD_PB_COUNT || seq < 0 || seq > 1) return 0.0f;
    if (a380x_ovhd_pbs[pb].legend[seq] == A380X_OVHD_LEGEND_NONE || !a380x_ovhd_powered(in, pb)) return 0.0f;
    level = in->ann_lt == A380X_OVHD_ANN_DIM ? A380X_OVHD_DIM_LEVEL : 1.0f;
    if (in->ann_lt == A380X_OVHD_ANN_TEST) return level;
    return a380x_ovhd_condition(in, pb, seq) ? level : 0.0f;
}

/* Pushbuttons with their own latched position (contract pb_state_ids) and its Airbus normal default:
 * OFF-type pressed in, ON-type released, so a normal cockpit is dark. */
static inline int a380x_ovhd_has_pb(int pb)
{
    const A380XOvhdLampInfo *info = &a380x_ovhd_pbs[pb];
    return info->source[0] == A380X_OVHD_SRC_PB_OFF || info->source[0] == A380X_OVHD_SRC_PB_ON ||
           info->source[1] == A380X_OVHD_SRC_PB_OFF || info->source[1] == A380X_OVHD_SRC_PB_ON;
}
static inline int a380x_ovhd_pb_default(int pb) { return a380x_ovhd_pbs[pb].source[1] == A380X_OVHD_SRC_PB_OFF; }

/* ANN LT switch: one step toward TEST (up) or DIM (down); returns the new position. */
static inline int a380x_ovhd_ann_step(int position, int up)
{
    position += up ? -1 : 1;
    return position < A380X_OVHD_ANN_TEST ? A380X_OVHD_ANN_TEST : position > A380X_OVHD_ANN_DIM ? A380X_OVHD_ANN_DIM : position;
}
#endif
