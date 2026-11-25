/* SPDX-License-Identifier: GPL-3.0-or-later
 * Native FWS signal side: see a380x_fws_signals.h and docs/FWS_SIGNALS.md.
 *
 * Source line references ("FwsCore:3570") point into the pinned FlyByWire A380X tree,
 * fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/. Port deviations are tagged [DEV n]
 * and listed in docs/FWS_SIGNALS.md; port-only alert logic is tagged [EXT].
 * XPLM-free plain C11.
 */
#include "a380x_fws_signals.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* ============================================================================================== */
/* NXLogic nodes (fbw-common/src/systems/instruments/src/NXLogic.ts), dt in seconds               */
/* ============================================================================================== */

static double fwss_max0(double v) { return v > 0.0 ? v : 0.0; }

static void fwss_conf_init(FwsConfNode *n, double t, int rising)
{
    memset(n, 0, sizeof *n);
    n->t = t;
    n->rising = (unsigned char)(rising != 0);
}

/* NXLogicConfirmNode.write (previousInput null behaves like false) */
static int fwss_conf(FwsConfNode *n, int value, double dt)
{
    value = value != 0;
    if (n->rising) {
        if (!value) {
            n->timer = 0.0;
        } else if (n->timer > 0.0) {
            n->timer = fwss_max0(n->timer - dt);
            n->prev_in = 1;
            n->prev_out = 0;
            return 0;
        } else if (!n->prev_in) {
            n->timer = n->t;
            n->prev_in = 1;
            n->prev_out = 0;
            return 0;
        }
    } else if (value) {
        n->timer = 0.0;
    } else if (n->timer > 0.0) {
        n->timer = fwss_max0(n->timer - dt);
        n->prev_in = 0;
        n->prev_out = 1;
        return 1;
    } else if (n->prev_in) {
        n->timer = n->t;
        n->prev_in = 0;
        n->prev_out = 1;
        return 1;
    }
    n->prev_in = (unsigned char)value;
    n->prev_out = (unsigned char)value;
    return value;
}

static int fwss_conf_read(const FwsConfNode *n) { return n->prev_out; }

static void fwss_mtrig_init(FwsMtrigNode *n, double t, int rising, int retrig)
{
    memset(n, 0, sizeof *n);
    n->t = t;
    n->rising = (unsigned char)(rising != 0);
    n->retrig = (unsigned char)(retrig != 0);
}

/* NXLogicTriggeredMonostableNode.write */
static int fwss_mtrig(FwsMtrigNode *n, int value, double dt)
{
    value = value != 0;
    if (n->timer > 0.0)
        n->timer = fwss_max0(n->timer - dt);
    if ((n->retrig || n->timer <= 0.0) &&
        ((n->rising && value && !n->prev) || (!n->rising && !value && n->prev)))
        n->timer = n->t;
    n->prev = (unsigned char)value;
    n->out = n->timer > 0.0;
    return n->out;
}

static int fwss_mtrig_read(const FwsMtrigNode *n) { return n->out; }

static void fwss_mem_init(FwsMemNode *n, int set_star)
{
    n->set_star = (unsigned char)(set_star != 0);
    n->value = 0;
}

/* NXLogicMemoryNode.write */
static int fwss_mem(FwsMemNode *n, int set, int reset)
{
    if (set && reset)
        n->value = n->set_star;
    else if (set && !n->value)
        n->value = 1;
    else if (reset && n->value)
        n->value = 0;
    return n->value;
}

static void fwss_pulse_init(FwsPulseNode *n, int rising)
{
    memset(n, 0, sizeof *n);
    n->rising = (unsigned char)(rising != 0);
}

/* NXLogicPulseNode.write */
static int fwss_pulse(FwsPulseNode *n, int input)
{
    input = input != 0;
    if (n->out)
        n->out = 0;
    if ((n->rising && input && !n->last) || (!n->rising && !input && n->last))
        n->out = 1;
    n->last = (unsigned char)input;
    return n->out;
}

/* ============================================================================================== */
/* Alert descriptor table                                                                        */
/* ============================================================================================== */

#define B(p) FWS_PHASE_BIT(p)
#define AL(ID, TITLE, FAIL, PH, PAGE, N, CONF, CANCEL, NONPROC, AURAL, ACT, TIMER, REC, EXT, DEF, ...) \
    { ID, TITLE, FAIL, PH, CONF, CANCEL, PAGE, N, NONPROC, AURAL, ACT, TIMER, REC, EXT, DEF, { __VA_ARGS__ } }
/* FBW alert, default monitor confirm time / cancel */
#define STD(ID, TITLE, FAIL, PH, PAGE, N, ...) AL(ID, TITLE, FAIL, PH, PAGE, N, 0.6f, 1, 0, 0, 0, 0, 0, 0, 0, __VA_ARGS__)
/* FBW alert with an auralWarning override */
#define AUR(ID, TITLE, FAIL, PH, PAGE, N, ...) AL(ID, TITLE, FAIL, PH, PAGE, N, 0.6f, 1, 0, 1, 0, 0, 0, 0, 0, __VA_ARGS__)
/* port extension [EXT] */
#define EXT(ID, TITLE, FAIL, PH, PAGE, N, ...) AL(ID, TITLE, FAIL, PH, PAGE, N, 0.6f, 1, 0, 0, 0, 0, 0, 1, 0, __VA_ARGS__)

/* phase sets */
#define PH_CAUT (B(3) | B(4) | B(5) | B(6) | B(7) | B(9) | B(10))                 /* typical FBW caution */
#define PH_LVL1 (B(3) | B(4) | B(5) | B(6) | B(7) | B(8) | B(9) | B(10) | B(11))  /* typical FBW level 1 */
#define PH_56 (B(5) | B(6))                                                          /* phase56Inhibition */
#define PH_TO_CFG (B(5) | B(6) | B(7) | B(8) | B(9) | B(10) | B(12))
#define PH_TO_SPD (B(1) | B(4) | B(5) | B(6) | B(7) | B(8) | B(9) | B(10) | B(12))
#define PH_HYD (B(4) | B(5) | B(6) | B(7) | B(9) | B(10))
#define PH_EMP (B(2) | B(3) | B(4) | B(5) | B(6) | B(7) | B(8) | B(9) | B(10) | B(11))   /* ELEC PMP FAULT */
#define PH_FEED (B(1) | B(3) | B(4) | B(5) | B(6) | B(7) | B(9) | B(10) | B(12))
#define PH_DOOR (B(1) | B(4) | B(5) | B(6) | B(7) | B(9) | B(10) | B(12))
#define PH_LG (B(1) | B(2) | B(3) | B(4) | B(5) | B(6) | B(7) | B(10) | B(11) | B(12))
#define PH_AIRBORNE (B(1) | B(2) | B(3) | B(4) | B(5) | B(10) | B(11) | B(12))
#define PH_ALL_BUT_8 (B(1) | B(2) | B(3) | B(4) | B(5) | B(6) | B(7) | B(9) | B(10) | B(11) | B(12))

static const FwsAlertDesc fwss_alerts[] = {
    /* ---- ATA 21 (FwsAbnormalSensed:487-509, 815-870, 990-1081) ---- */
    STD(211800011u, "PACK 1 OFF", 2, PH_ALL_BUT_8, FWS_SD_BLEED, 0, 211800009u, 211800021u),
    STD(211800012u, "PACK 2 OFF", 2, PH_ALL_BUT_8, FWS_SD_BLEED, 0, 211800010u, 211800021u),
    AL(211800021u, "PACK 1+2 FAULT", 2, B(4) | B(5) | B(6) | B(7) | B(9) | B(10), FWS_SD_BLEED, 15,
       0.6f, 1, 0, 0, 0, 0, 2, 0, 0, 0),
    STD(211800051u, "APU BLEED FAULT", 2, PH_CAUT, FWS_SD_BLEED, 6, 0),
    STD(211800052u, "ENG 1 BLEED OFF", 2, PH_CAUT, FWS_SD_BLEED, 0, 0),
    STD(211800053u, "ENG 2 BLEED OFF", 2, PH_CAUT, FWS_SD_BLEED, 0, 0),
    STD(211800054u, "ENG 3 BLEED OFF", 2, PH_CAUT, FWS_SD_BLEED, 0, 0),
    STD(211800055u, "ENG 4 BLEED OFF", 2, PH_CAUT, FWS_SD_BLEED, 0, 0),
    STD(213800001u, "EXCESS CAB ALT", 3, PH_ALL_BUT_8, FWS_SD_PRESS, 23, 0),
    STD(213800002u, "EXCESS DIFF PRESS", 3, PH_ALL_BUT_8, FWS_SD_PRESS, 7, 0),
    STD(213800004u, "EXCESS RESIDUAL DIFF PRESS", 3, B(2) | B(3) | B(4) | B(5) | B(6) | B(7) | B(8) | B(9) | B(10),
        FWS_SD_PRESS, 5, 0),
    STD(213800009u, "INHIBITED BY DOORS", 2, B(1) | B(4) | B(5) | B(6) | B(7) | B(9) | B(10) | B(11) | B(12),
        FWS_SD_PRESS, 1, 0),
    /* ---- ATA 22 (FwsAbnormalSensed:1661-1840) ---- */
    AL(220000002u, "A/THR OFF (caution, non-procedure)", 2, 0, FWS_SD_NONE, 0, 0.0f, 1, 1, 0, 0, 0, 0, 0, 0, 0),
    AL(220800001u, "AP OFF (involuntary)", 3, 0, FWS_SD_NONE, 0, 0.0f, 1, 0, 1, 0, 0, 0, 0, 0, 0),
    AL(220800003u, "A/THR LIMITED", 2, B(2) | B(3) | B(4) | B(5) | B(6) | B(10) | B(11), FWS_SD_NONE, 2,
       0.0f, 1, 0, 0, 0, 0, 0, 0, 0, 0),
    AL(220800004u, "A/THR OFF (involuntary)", 2, B(3) | B(4) | B(5) | B(10), FWS_SD_NONE, 1, 0.0f, 1, 0, 1, 0, 0,
       0, 0, 0, 0),
    STD(221800007u, "TO SPEEDS NOT INSERTED", 2, PH_TO_SPD, FWS_SD_NONE, 0, 0),
    STD(221800008u, "TO SPEEDS TOO LOW", 2, PH_TO_SPD, FWS_SD_NONE, 1, 0),
    STD(221800009u, "TO V1/VR/V2 DISAGREE", 2, PH_TO_SPD, FWS_SD_NONE, 0, 0),
    /* ---- ATA 24 [EXT] port extensions from the native network (texts: AbnormalSensed/ata24.ts) ---- */
    EXT(240800002u, "ELEC AC BUS 1 FAULT", 2, PH_CAUT, FWS_SD_ELEC_AC, 1, 0),
    EXT(240800004u, "ELEC AC BUS 2 FAULT", 2, PH_CAUT, FWS_SD_ELEC_AC, 8, 0),
    EXT(240800007u, "ELEC AC BUS 3 FAULT", 2, PH_CAUT, FWS_SD_ELEC_AC, 12, 0),
    EXT(240800009u, "ELEC AC BUS 4 FAULT", 2, PH_CAUT, FWS_SD_ELEC_AC, 9, 0),
    EXT(240800010u, "ELEC AC EMER BUS FAULT", 2, PH_CAUT, FWS_SD_ELEC_AC, 4, 0),
    EXT(240800011u, "ELEC AC ESS BUS ALTN", 1, PH_LVL1, FWS_SD_ELEC_AC, 0, 0),
    EXT(240800012u, "ELEC AC ESS BUS FAULT", 2, PH_CAUT, FWS_SD_ELEC_AC, 11, 0),
    EXT(240800013u, "ELEC APU BAT FAULT", 2, PH_CAUT, FWS_SD_ELEC_DC, 1, 0),
    EXT(240800014u, "ELEC APU GEN A FAULT", 1, PH_LVL1, FWS_SD_ELEC_AC, 1, 0),
    EXT(240800015u, "ELEC APU GEN B FAULT", 1, PH_LVL1, FWS_SD_ELEC_AC, 1, 0),
    EXT(240800016u, "ELEC APU TR FAULT", 1, PH_LVL1, FWS_SD_ELEC_DC, 2, 0),
    EXT(240800017u, "ELEC BAT 1 (ESS) FAULT", 2, PH_CAUT, FWS_SD_ELEC_DC, 1, 0),
    EXT(240800018u, "ELEC BAT 2 (ESS) FAULT", 2, PH_CAUT, FWS_SD_ELEC_DC, 1, 0),
    EXT(240800019u, "ELEC BUS TIE OFF", 2, PH_CAUT, FWS_SD_ELEC_AC, 0, 0),
    EXT(240800026u, "ELEC DC BUS 1 FAULT", 2, PH_CAUT, FWS_SD_ELEC_DC, 11, 240800027u, 240800028u),
    EXT(240800027u, "ELEC DC BUS 1+2 FAULT", 2, PH_CAUT, FWS_SD_ELEC_DC, 22, 0),
    AL(240800028u, "ELEC DC BUS 1+ESS FAULT", 2, PH_CAUT, FWS_SD_ELEC_DC, 21, 0.6f, 1, 0, 0, 0, 0, 2, 1, 0, 0),
    EXT(240800029u, "ELEC DC BUS 2 FAULT", 2, PH_CAUT, FWS_SD_ELEC_DC, 12, 240800027u),
    EXT(240800030u, "ELEC DC ESS BUS FAULT", 2, PH_CAUT, FWS_SD_ELEC_DC, 20, 240800028u),
    EXT(240800036u, "ELEC DRIVE 1 DISCONNECTED", 2, PH_CAUT, FWS_SD_ELEC_AC, 1, 0),
    EXT(240800037u, "ELEC DRIVE 2 DISCONNECTED", 2, PH_CAUT, FWS_SD_ELEC_AC, 1, 0),
    EXT(240800038u, "ELEC DRIVE 3 DISCONNECTED", 2, PH_CAUT, FWS_SD_ELEC_AC, 1, 0),
    EXT(240800039u, "ELEC DRIVE 4 DISCONNECTED", 2, PH_CAUT, FWS_SD_ELEC_AC, 1, 0),
    AL(240800055u, "ELEC EMER CONFIG", 3, B(1) | B(2) | B(3) | B(4) | B(5) | B(11) | B(12), FWS_SD_ELEC_AC, 30,
       0.6f, 1, 0, 0, 0, 0, 1, 1, 0, 0),
    EXT(240800056u, "ELEC EXT PWR 1 FAULT", 2, PH_CAUT | B(8), FWS_SD_ELEC_AC, 1, 0),
    EXT(240800057u, "ELEC EXT PWR 2 FAULT", 2, PH_CAUT | B(8), FWS_SD_ELEC_AC, 1, 0),
    EXT(240800058u, "ELEC EXT PWR 3 FAULT", 2, PH_CAUT | B(8), FWS_SD_ELEC_AC, 1, 0),
    EXT(240800059u, "ELEC EXT PWR 4 FAULT", 2, PH_CAUT | B(8), FWS_SD_ELEC_AC, 1, 0),
    EXT(240800061u, "ELEC GEN 1 FAULT", 2, PH_CAUT, FWS_SD_ELEC_AC, 1, 0),
    EXT(240800062u, "ELEC GEN 2 FAULT", 2, PH_CAUT, FWS_SD_ELEC_AC, 1, 0),
    EXT(240800063u, "ELEC GEN 3 FAULT", 2, PH_CAUT, FWS_SD_ELEC_AC, 1, 0),
    EXT(240800064u, "ELEC GEN 4 FAULT", 2, PH_CAUT, FWS_SD_ELEC_AC, 1, 0),
    EXT(240800065u, "ELEC GEN 1 OFF", 2, PH_CAUT, FWS_SD_ELEC_AC, 0, 0),
    EXT(240800066u, "ELEC GEN 2 OFF", 2, PH_CAUT, FWS_SD_ELEC_AC, 0, 0),
    EXT(240800067u, "ELEC GEN 3 OFF", 2, PH_CAUT, FWS_SD_ELEC_AC, 0, 0),
    EXT(240800068u, "ELEC GEN 4 OFF", 2, PH_CAUT, FWS_SD_ELEC_AC, 0, 0),
    EXT(240800072u, "ELEC RAT FAULT", 1, PH_LVL1, FWS_SD_ELEC_AC, 0, 0),
    EXT(240800080u, "ELEC STATIC INV FAULT", 1, PH_LVL1, FWS_SD_ELEC_AC, 0, 0),
    EXT(240800081u, "ELEC TR 1 FAULT", 1, PH_LVL1, FWS_SD_ELEC_DC, 0, 0),
    EXT(240800082u, "ELEC TR 2 FAULT", 1, PH_LVL1, FWS_SD_ELEC_DC, 0, 0),
    EXT(240800083u, "ELEC TR ESS FAULT", 2, PH_CAUT, FWS_SD_ELEC_DC, 0, 0),
    /* ---- ATA 26 (FwsAbnormalSensed:1924-2318) ---- */
    AL(260800001u, "APU FIRE", 3, PH_56, FWS_SD_APU, 3, 0.6f, 1, 0, 1, 0, 1, 1, 0, 0, 0),
    AL(260800005u, "ENG 1 FIRE (IN FLIGHT)", 3, PH_56, FWS_SD_ENG, 9, 0.6f, 1, 0, 1, 0, 1, 1, 0, 0, 0),
    AL(260800006u, "ENG 2 FIRE (IN FLIGHT)", 3, PH_56, FWS_SD_ENG, 7, 0.6f, 1, 0, 1, 0, 1, 1, 0, 0, 0),
    AL(260800007u, "ENG 3 FIRE (IN FLIGHT)", 3, PH_56, FWS_SD_ENG, 7, 0.6f, 1, 0, 1, 0, 1, 1, 0, 0, 0),
    AL(260800008u, "ENG 4 FIRE (IN FLIGHT)", 3, PH_56, FWS_SD_ENG, 7, 0.6f, 1, 0, 1, 0, 1, 1, 0, 0, 0),
    AUR(260800009u, "ENG 1 FIRE (ON GROUND)", 3, PH_56, FWS_SD_ENG, 16, 0),
    AUR(260800010u, "ENG 2 FIRE (ON GROUND)", 3, PH_56, FWS_SD_ENG, 16, 0),
    AUR(260800011u, "ENG 3 FIRE (ON GROUND)", 3, PH_56, FWS_SD_ENG, 16, 0),
    AUR(260800012u, "ENG 4 FIRE (ON GROUND)", 3, PH_56, FWS_SD_ENG, 16, 0),
    /* ---- ATA 27 (FwsAbnormalSensed:2541-3045) ---- */
    AUR(271800003u, "PITCH TRIM NOT IN TO RANGE", 3, PH_TO_CFG, FWS_SD_FCTL, 0, 0),
    AUR(271800004u, "RUDDER TRIM NOT IN TO RANGE", 3, B(5) | B(6) | B(7) | B(8) | B(9) | B(10), FWS_SD_FCTL, 0, 0),
    AUR(271800005u, "SPD BRK NOT RETRACTED", 3, B(5) | B(6) | B(7) | B(8) | B(9) | B(10), FWS_SD_FCTL, 0, 0),
    STD(271800017u, "GND SPLRs NOT ARMED", 2, PH_LG, FWS_SD_NONE, 0, 0),
    STD(271800032u, "PITCH TRIM FMS DISAGREE", 2, PH_TO_SPD, FWS_SD_NONE, 0, 271800003u),
    STD(271800065u, "SPEED BRAKES STILL EXTENDED", 2, PH_LG, FWS_SD_NONE, 0, 0),
    STD(271800069u, "OVERSPEED", 3, B(2) | B(3) | B(4) | B(5) | B(10) | B(11) | B(12), FWS_SD_NONE, 7, 0),
    STD(271800070u, "LOAD ANALYSIS REQUIRED", 1,
        B(1) | B(2) | B(3) | B(4) | B(5) | B(6) | B(7) | B(8) | B(9) | B(10) | B(11), FWS_SD_NONE, 0, 0),
    AUR(272800001u, "SLAT NOT IN TO CONFIG", 3, PH_TO_CFG, FWS_SD_NONE, 0, 0),
    AUR(272800002u, "FLAPS NOT IN TO CONFIG", 3, PH_TO_CFG, FWS_SD_NONE, 0, 0),
    STD(272800003u, "FLAPS LEVER NOT ZERO", 3, PH_ALL_BUT_8, FWS_SD_NONE, 0, 0),
    STD(272800006u, "FLAP SYS 1 FAULT", 2, PH_CAUT, FWS_SD_FCTL, 1, 272800008u),
    STD(272800007u, "FLAP SYS 2 FAULT", 2, PH_CAUT, FWS_SD_FCTL, 1, 272800008u),
    STD(272800008u, "FLAP SYS 1+2 FAULT", 2, PH_HYD, FWS_SD_FCTL, 12, 0),
    STD(272800021u, "SLAT SYS 1 FAULT", 2, PH_CAUT, FWS_SD_FCTL, 1, 272800023u),
    STD(272800022u, "SLAT SYS 2 FAULT", 2, PH_CAUT, FWS_SD_FCTL, 1, 272800023u),
    STD(272800023u, "SLAT SYS 1+2 FAULT", 2, PH_HYD, FWS_SD_FCTL, 11, 0),
    STD(272800028u, "TO FLAPS FMS DISAGREE", 2, PH_TO_SPD, FWS_SD_NONE, 0, 272800002u),
    /* ---- ATA 28 (FwsAbnormalSensed:3056-3203) + [EXT] feed pumps (ata28.ts) ---- */
    STD(281800002u, "ALL FEED TKs LEVEL LO", 2, PH_FEED, FWS_SD_FUEL, 7, 0),
    STD(281800023u, "FEED TK 1 LEVEL LO", 2, PH_FEED, FWS_SD_FUEL, 8, 281800102u, 281800002u),
    STD(281800024u, "FEED TK 2 LEVEL LO", 2, PH_FEED, FWS_SD_FUEL, 8, 281800102u, 281800002u),
    STD(281800025u, "FEED TK 3 LEVEL LO", 2, PH_FEED, FWS_SD_FUEL, 8, 281800103u, 281800002u),
    STD(281800026u, "FEED TK 4 LEVEL LO", 2, PH_FEED, FWS_SD_FUEL, 8, 281800103u, 281800002u),
    EXT(281800027u, "FUEL FEED TK 1 MAIN + STBY PMPs FAULT", 2, PH_CAUT, FWS_SD_FUEL, 0, 0),
    EXT(281800028u, "FUEL FEED TK 2 MAIN + STBY PMPs FAULT", 2, PH_CAUT, FWS_SD_FUEL, 0, 0),
    EXT(281800029u, "FUEL FEED TK 3 MAIN + STBY PMPs FAULT", 2, PH_CAUT, FWS_SD_FUEL, 0, 0),
    EXT(281800030u, "FUEL FEED TK 4 MAIN + STBY PMPs FAULT", 2, PH_CAUT, FWS_SD_FUEL, 0, 0),
    EXT(281800031u, "FUEL FEED TK 1 MAIN PMP FAULT", 1, PH_LVL1, FWS_SD_FUEL, 0, 281800027u),
    EXT(281800032u, "FUEL FEED TK 2 MAIN PMP FAULT", 1, PH_LVL1, FWS_SD_FUEL, 0, 281800028u),
    EXT(281800033u, "FUEL FEED TK 3 MAIN PMP FAULT", 1, PH_LVL1, FWS_SD_FUEL, 0, 281800029u),
    EXT(281800034u, "FUEL FEED TK 4 MAIN PMP FAULT", 1, PH_LVL1, FWS_SD_FUEL, 0, 281800030u),
    EXT(281800035u, "FUEL FEED TK 1 STBY PMP FAULT", 1, PH_LVL1, FWS_SD_FUEL, 0, 281800027u),
    EXT(281800036u, "FUEL FEED TK 2 STBY PMP FAULT", 1, PH_LVL1, FWS_SD_FUEL, 0, 281800028u),
    EXT(281800037u, "FUEL FEED TK 3 STBY PMP FAULT", 1, PH_LVL1, FWS_SD_FUEL, 0, 281800029u),
    EXT(281800038u, "FUEL FEED TK 4 STBY PMP FAULT", 1, PH_LVL1, FWS_SD_FUEL, 0, 281800030u),
    STD(281800076u, "NO ZFW OR ZFWCG DATA", 2,
        B(1) | B(3) | B(4) | B(5) | B(6) | B(7) | B(9) | B(10) | B(11) | B(12), FWS_SD_NONE, 1, 0),
    STD(281800102u, "FEED TKs 1+2 LEVEL LO", 2, PH_FEED, FWS_SD_FUEL, 7, 281800002u),
    STD(281800103u, "FEED TKs 3+4 LEVEL LO", 2, PH_FEED, FWS_SD_FUEL, 7, 281800002u),
    /* ---- ATA 29 (FwsAbnormalSensed:3249-3565) ---- */
    STD(290800001u, "G ELEC PMP A FAULT", 2, PH_EMP, FWS_SD_HYD, 1, 0),
    STD(290800002u, "G ELEC PMP B FAULT", 2, PH_EMP, FWS_SD_HYD, 1, 0),
    STD(290800003u, "Y ELEC PMP A FAULT", 2, PH_EMP, FWS_SD_HYD, 1, 0),
    STD(290800004u, "Y ELEC PMP B FAULT", 2, PH_EMP, FWS_SD_HYD, 1, 0),
    STD(290800005u, "G ENG 1 PMP A PRESS LO", 2, B(1) | PH_HYD | B(12), FWS_SD_HYD, 2, 0),
    STD(290800006u, "G ENG 1 PMP B PRESS LO", 2, B(1) | PH_HYD | B(12), FWS_SD_HYD, 2, 0),
    STD(290800007u, "G ENG 2 PMP A PRESS LO", 2, B(1) | PH_HYD | B(12), FWS_SD_HYD, 2, 0),
    STD(290800008u, "G ENG 2 PMP B PRESS LO", 2, B(1) | PH_HYD | B(12), FWS_SD_HYD, 2, 0),
    STD(290800009u, "Y ENG 3 PMP A PRESS LO", 2, B(1) | PH_HYD | B(12), FWS_SD_HYD, 2, 0),
    STD(290800010u, "Y ENG 3 PMP B PRESS LO", 2, B(1) | PH_HYD | B(12), FWS_SD_HYD, 2, 0),
    STD(290800011u, "Y ENG 4 PMP A PRESS LO", 2, B(1) | PH_HYD | B(12), FWS_SD_HYD, 2, 0),
    STD(290800012u, "Y ENG 4 PMP B PRESS LO", 2, B(1) | PH_HYD | B(12), FWS_SD_HYD, 2, 0),
    STD(290800019u, "G RSVR AIR PRESS LO", 2, PH_HYD, FWS_SD_HYD, 4, 0),
    STD(290800020u, "Y RSVR AIR PRESS LO", 2, PH_HYD, FWS_SD_HYD, 4, 0),
    STD(290800021u, "G RSVR LEVEL LO", 2, PH_HYD, FWS_SD_HYD, 5, 0),
    STD(290800022u, "Y RSVR LEVEL LO", 2, PH_HYD, FWS_SD_HYD, 5, 0),
    STD(290800031u, "G SYS OVERHEAT", 2, PH_HYD, FWS_SD_HYD, 5, 0),
    STD(290800032u, "Y SYS OVERHEAT", 2, PH_HYD, FWS_SD_HYD, 5, 0),
    STD(290800035u, "G SYS LO PRESS", 2, PH_HYD, FWS_SD_HYD, 8, 290800039u),
    STD(290800036u, "Y SYS LO PRESS", 2, PH_HYD, FWS_SD_HYD, 5, 290800039u),
    STD(290800039u, "G + Y SYS LO PRESS", 2, PH_HYD, FWS_SD_HYD, 11, 0),
    /* ---- ATA 32 (FwsAbnormalSensed:3626-3722) + [EXT] (ata31-32-33.ts) ---- */
    AL(320000001u, "AUTO BRK OFF (master caution, non-procedure)", 2, 0, FWS_SD_NONE, 0, 0.0f, 1, 1, 1, 0, 0, 0, 0,
       0, 0),
    EXT(320800001u, "BRAKES A-SKID FAULT ON ALL L/G", 2, B(4) | B(5) | B(6) | B(7), FWS_SD_WHEEL, 2, 0),
    STD(320800008u, "BRAKES A-SKID OFF", 2, B(4) | B(5) | B(6) | B(7), FWS_SD_WHEEL, 2, 0),
    EXT(320800013u, "BRAKES AUTO BRK FAULT", 2, B(4) | B(5) | B(6) | B(7), FWS_SD_WHEEL, 0, 0),
    STD(320800018u, "BRAKES HOT", 2, B(4) | B(5) | B(9) | B(10) | B(11), FWS_SD_WHEEL, 5, 0),
    EXT(320800020u, "BRAKES NORM BRK FAULT", 2, B(4) | B(5) | B(6) | B(7), FWS_SD_WHEEL, 1, 0),
    STD(320800022u, "BRAKES PARK BRK ON", 2, PH_LG, FWS_SD_NONE, 1, 0),
    STD(320800030u, "CONFIG PARK BRK ON", 3,
        B(1) | B(2) | B(4) | B(5) | B(6) | B(7) | B(8) | B(9) | B(10) | B(11) | B(12), FWS_SD_NONE, 0, 0),
    AL(320800037u, "L/G GEAR NOT DOWN (no cancel)", 3, PH_LG, FWS_SD_WHEEL, 0, 0.6f, 0, 0, 0, 0, 0, 0, 0, 0, 0),
    STD(320800038u, "L/G GEAR NOT DOWN (cancel)", 3, PH_LG, FWS_SD_WHEEL, 0, 0),
    EXT(320800039u, "L/G GEAR NOT LOCKED DOWN", 3, PH_AIRBORNE, FWS_SD_WHEEL, 6, 0),
    EXT(320800040u, "L/G GEAR NOT LOCKED UP", 2, PH_AIRBORNE, FWS_SD_WHEEL, 7, 0),
    /* ---- ATA 34 (FwsAbnormalSensed:3725-4376) ---- */
    STD(340800001u, "ADR 1 FAULT", 2, B(4) | B(5) | B(10), FWS_SD_NONE, 4, 340800004u, 340800008u, 340800005u),
    STD(340800002u, "ADR 2 FAULT", 2, B(4) | B(5) | B(10), FWS_SD_NONE, 2, 340800004u, 340800008u, 340800006u),
    STD(340800003u, "ADR 3 FAULT", 2, B(4) | B(5) | B(10), FWS_SD_NONE, 4, 340800005u, 340800006u, 340800008u),
    STD(340800004u, "ADR 1+2 FAULT", 2, B(4) | B(5) | B(10), FWS_SD_NONE, 8, 340800008u),
    STD(340800005u, "ADR 1+3 FAULT", 2, B(4) | B(5) | B(10), FWS_SD_NONE, 8, 340800008u),
    STD(340800006u, "ADR 2+3 FAULT", 2, B(4) | B(5) | B(10), FWS_SD_NONE, 8, 340800008u),
    STD(340800008u, "ADR 1+2+3 FAULT", 3, B(4) | B(5) | B(10), FWS_SD_NONE, 3, 340800010u, 340800071u),
    STD(340800040u, "IR 1 FAULT", 2, B(4) | B(5) | B(6) | B(9) | B(10), FWS_SD_NONE, 2, 340800042u, 340800043u),
    STD(340800041u, "IR 2 FAULT", 2, B(4) | B(5) | B(6) | B(9) | B(10), FWS_SD_NONE, 2, 340800042u, 340800044u),
    STD(340800042u, "IR 1+2 FAULT", 2, B(4) | B(5) | B(6) | B(9) | B(10), FWS_SD_NONE, 5, 0),
    STD(340800043u, "IR 1+3 FAULT", 2, B(4) | B(5) | B(6) | B(9) | B(10), FWS_SD_NONE, 6, 0),
    STD(340800044u, "IR 2+3 FAULT", 2, B(4) | B(5) | B(6) | B(9) | B(10), FWS_SD_NONE, 5, 0),
    STD(340800053u, "RA SYS A FAULT", 1, B(2) | PH_LVL1, FWS_SD_NONE, 0, 340800059u, 340800060u, 340800062u),
    STD(340800054u, "RA SYS B FAULT", 1, B(2) | PH_LVL1, FWS_SD_NONE, 0, 340800059u, 340800061u, 340800062u),
    STD(340800055u, "RA SYS C FAULT", 1, B(2) | PH_LVL1, FWS_SD_NONE, 0, 340800060u, 340800061u, 340800062u),
    STD(340800059u, "RA SYS A+B FAULT", 2, B(3) | B(4) | B(5) | B(6) | B(7) | B(10) | B(11), FWS_SD_NONE, 0,
        340800062u),
    STD(340800060u, "RA SYS A+C FAULT", 2, B(3) | B(4) | B(5) | B(6) | B(7) | B(10) | B(11), FWS_SD_NONE, 0,
        340800062u),
    STD(340800061u, "RA SYS B+C FAULT", 2, B(3) | B(4) | B(5) | B(6) | B(7) | B(10) | B(11), FWS_SD_NONE, 0,
        340800062u),
    STD(340800062u, "RA SYS A+B+C FAULT", 2, B(3) | B(4) | B(5) | B(6) | B(7) | B(10), FWS_SD_NONE, 3, 0),
    STD(340800072u, "IR 3 FAULT", 2, B(4) | B(5) | B(6) | B(9) | B(10), FWS_SD_NONE, 2, 340800043u, 340800044u),
    STD(341800016u, "TCAS 1 FAULT", 2, PH_CAUT | B(11), FWS_SD_NONE, 1, 0),
    STD(341800017u, "TCAS 2 FAULT", 2, PH_CAUT | B(11), FWS_SD_NONE, 1, 0),
    STD(341800018u, "TCAS 1+2 FAULT", 2, PH_CAUT | B(11), FWS_SD_NONE, 0, 0),
    STD(341800019u, "TCAS STBY", 2, PH_ALL_BUT_8, FWS_SD_NONE, 0, 0),
    STD(341800037u, "XPDR STBY", 2, B(1) | B(3) | B(4) | B(5) | B(6) | B(7) | B(10) | B(12), FWS_SD_NONE, 0, 0),
    /* ---- ATA 52 (FwsAbnormalSensed:4387-4576) ---- */
    STD(520800017u, "MAIN 1L NOT CLOSED", 2, PH_DOOR, FWS_SD_DOOR, 6, 0),
    STD(520800018u, "MAIN 1R NOT CLOSED", 2, PH_DOOR, FWS_SD_DOOR, 6, 0),
    STD(520800019u, "MAIN 2L NOT CLOSED", 2, PH_DOOR, FWS_SD_DOOR, 6, 0),
    STD(520800020u, "MAIN 2R NOT CLOSED", 2, PH_DOOR, FWS_SD_DOOR, 6, 0),
    STD(520800021u, "MAIN 3L NOT CLOSED", 2, PH_DOOR, FWS_SD_DOOR, 6, 0),
    STD(520800022u, "MAIN 3R NOT CLOSED", 2, PH_DOOR, FWS_SD_DOOR, 6, 0),
    STD(520800023u, "MAIN 4L NOT CLOSED", 2, PH_DOOR, FWS_SD_DOOR, 6, 0),
    STD(520800024u, "MAIN 4R NOT CLOSED", 2, PH_DOOR, FWS_SD_DOOR, 6, 0),
    STD(520800025u, "MAIN 5L NOT CLOSED", 2, PH_DOOR, FWS_SD_DOOR, 6, 0),
    STD(520800026u, "MAIN 5R NOT CLOSED", 2, PH_DOOR, FWS_SD_DOOR, 6, 0),
    /* ---- ATA 70 (FwsAbnormalSensed:4578-4755) + [EXT] (ata70.ts) ---- */
    EXT(701800009u, "ENG 1 EGT OVER LIMIT", 2, PH_56, FWS_SD_ENG, 0, 0),
    EXT(701800010u, "ENG 2 EGT OVER LIMIT", 2, PH_56, FWS_SD_ENG, 0, 0),
    EXT(701800011u, "ENG 3 EGT OVER LIMIT", 2, PH_56, FWS_SD_ENG, 0, 0),
    EXT(701800012u, "ENG 4 EGT OVER LIMIT", 2, PH_56, FWS_SD_ENG, 0, 0),
    STD(701800029u, "ENG 1 FAIL", 2, PH_56, FWS_SD_ENG, 0, 0),
    STD(701800030u, "ENG 2 FAIL", 2, PH_56, FWS_SD_ENG, 0, 0),
    STD(701800031u, "ENG 3 FAIL", 2, PH_56, FWS_SD_ENG, 0, 0),
    STD(701800032u, "ENG 4 FAIL", 2, PH_56, FWS_SD_ENG, 0, 0),
    EXT(701800073u, "ENG 1 N1/N2 OVER LIMIT", 2, PH_56, FWS_SD_ENG, 0, 0),
    EXT(701800074u, "ENG 2 N1/N2 OVER LIMIT", 2, PH_56, FWS_SD_ENG, 0, 0),
    EXT(701800075u, "ENG 3 N1/N2 OVER LIMIT", 2, PH_56, FWS_SD_ENG, 0, 0),
    EXT(701800076u, "ENG 4 N1/N2 OVER LIMIT", 2, PH_56, FWS_SD_ENG, 0, 0),
    EXT(701800085u, "ENG 1 OIL PRESS LO", 2, PH_56, FWS_SD_ENG, 0, 0),
    EXT(701800086u, "ENG 2 OIL PRESS LO", 2, PH_56, FWS_SD_ENG, 0, 0),
    EXT(701800087u, "ENG 3 OIL PRESS LO", 2, PH_56, FWS_SD_ENG, 0, 0),
    EXT(701800088u, "ENG 4 OIL PRESS LO", 2, PH_56, FWS_SD_ENG, 0, 0),
    STD(701800109u, "ENG 1 SHUT DOWN", 2, PH_56, FWS_SD_NONE, 8, 0),
    STD(701800110u, "ENG 2 SHUT DOWN", 2, PH_56, FWS_SD_NONE, 9, 0),
    STD(701800111u, "ENG 3 SHUT DOWN", 2, PH_56, FWS_SD_NONE, 9, 0),
    STD(701800112u, "ENG 4 SHUT DOWN", 2, PH_56, FWS_SD_NONE, 8, 0),
    EXT(701800117u, "ENG 1 START FAULT", 2, PH_56, FWS_SD_ENG, 0, 0),
    EXT(701800118u, "ENG 2 START FAULT", 2, PH_56, FWS_SD_ENG, 0, 0),
    EXT(701800119u, "ENG 3 START FAULT", 2, PH_56, FWS_SD_ENG, 0, 0),
    EXT(701800120u, "ENG 4 START FAULT", 2, PH_56, FWS_SD_ENG, 0, 0),
    AL(701800151u, "ALL ENGINES FAILURE", 3, B(1) | B(2) | B(3) | B(4) | B(5) | B(6) | B(10) | B(11) | B(12),
       FWS_SD_ENG, 0, 0.6f, 1, 0, 0, 0, 0, 1, 0, 0, 0),
    STD(701800158u, "ENGINE THRUST LOCKED", 2, B(1) | B(2) | B(3) | B(4) | B(5) | B(6) | B(10) | B(11) | B(12),
        FWS_SD_NONE, 1, 0),
    /* ---- secondary failures (FwsAbnormalSensed:4756-4835) ---- */
    STD(999800001u, "*F/CTL", 2, 0, FWS_SD_FCTL, 0, 0),
    STD(999800002u, "*FUEL", 2, 0, FWS_SD_FUEL, 0, 0),
    STD(999800003u, "*WHEEL", 2, 0, FWS_SD_WHEEL, 0, 0),
    STD(999800004u, "*ELEC (AC)", 2, 0, FWS_SD_ELEC_AC, 0, 0),
    STD(999800005u, "*ELEC (DC)", 2, 0, FWS_SD_ELEC_DC, 0, 0),
    STD(999800006u, "*BLEED", 2, 0, FWS_SD_BLEED, 0, 0),
    STD(999800007u, "*HYD", 2, 0, FWS_SD_HYD, 0, 0),
    /* ---- ewdDeferredProcs (FwsAbnormalSensed:4838-4890), simVarIsActive is always true ---- */
    AL(210700001u, "WHEN PACK OVHT OUT (deferred)", 0, 0, FWS_SD_NONE, 2, 0.6f, 1, 0, 0, 0, 0, 0, 0, 1, 0),
    AL(210700002u, "WHEN DIFF PR<2 PSI & FL<100 (deferred)", 0, 0, FWS_SD_NONE, 2, 0.6f, 1, 0, 0, 0, 0, 0, 0, 1, 0),
    AL(221700001u, "LDG ELEVN (deferred)", 0, 0, FWS_SD_NONE, 2, 0.6f, 1, 0, 0, 0, 0, 0, 0, 1, 0),
    AL(270700001u, "RUDDER TRIM RUNAWAY (deferred)", 0, 0, FWS_SD_NONE, 5, 0.6f, 1, 0, 0, 0, 0, 0, 0, 1, 0),
    AL(320700001u, "L/G GRVTY EXTN (deferred)", 0, 0, FWS_SD_NONE, 5, 0.6f, 1, 0, 0, 0, 0, 0, 0, 1, 0),
};

#define FWSS_NUM_ALERTS ((int)(sizeof fwss_alerts / sizeof fwss_alerts[0]))
_Static_assert(sizeof fwss_alerts / sizeof fwss_alerts[0] <= FWS_SIG_MAX_ALERTS, "raise FWS_SIG_MAX_ALERTS");
#undef AL
#undef STD
#undef AUR
#undef EXT

/* ============================================================================================== */
/* INOP SYS / LIMITATIONS dictionaries (FwsInopSys.ts, FwsLimitations.ts), entries the port can   */
/* derive, in FBW Object.entries order (ascending id). FwsInformation.ts has no derivable entry.  */
/* ============================================================================================== */
typedef struct {
    unsigned id;
    unsigned char appr_ldg;     /* phase ApprLdg */
    unsigned char redund;       /* redundancyLoss (inop) / pfd (limitations) */
    unsigned not_active_when[3];
} FwsDictItem;

static const FwsDictItem fwss_inop[] = {
    {240300010u, 0, 0, {240300037u, 0, 0}}, /* GEN 1 */
    {240300011u, 0, 0, {240300037u, 0, 0}}, /* GEN 2 */
    {240300012u, 0, 0, {240300038u, 0, 0}}, /* GEN 3 */
    {240300013u, 0, 0, {240300038u, 0, 0}}, /* GEN 4 */
    {240300037u, 0, 0, {0, 0, 0}},          /* GEN 1+2 */
    {240300038u, 0, 0, {0, 0, 0}},          /* GEN 3+4 */
    {260300002u, 0, 0, {0, 0, 0}},          /* ENG 1 BLEED */
    {260300003u, 0, 0, {0, 0, 0}},
    {260300004u, 0, 0, {0, 0, 0}},
    {260300005u, 0, 0, {0, 0, 0}},
    {290100001u, 0, 0, {290100011u, 0, 0}}, /* PART SPLRs */
    {290100003u, 1, 0, {290100012u, 0, 0}}, /* FLAP SYS 1 */
    {290100004u, 1, 0, {290100012u, 0, 0}}, /* FLAP SYS 2 */
    {290100005u, 1, 0, {290100013u, 0, 0}}, /* SLAT SYS 1 */
    {290100006u, 1, 0, {290100013u, 0, 0}}, /* SLAT SYS 2 */
    {290100011u, 0, 0, {0, 0, 0}},          /* MOST SPLRs */
    {290100012u, 1, 0, {0, 0, 0}},          /* FLAPS */
    {290100013u, 1, 0, {0, 0, 0}},          /* SLATS */
    {290300021u, 0, 0, {0, 0, 0}},          /* G HYD SYS */
    {290300022u, 0, 0, {0, 0, 0}},          /* Y HYD SYS */
    {290300023u, 0, 0, {0, 0, 0}},          /* G ENG 1 PMP A+B */
    {290300024u, 0, 0, {0, 0, 0}},
    {290300025u, 0, 0, {0, 0, 0}},
    {290300026u, 0, 0, {0, 0, 0}},
    {340300004u, 0, 0, {340300007u, 340300009u, 340300010u}}, /* ADR 1 */
    {340300005u, 0, 0, {340300007u, 340300008u, 340300010u}}, /* ADR 2 */
    {340300006u, 0, 0, {340300008u, 340300009u, 340300010u}}, /* ADR 3 */
    {340300007u, 0, 0, {340300010u, 0, 0}},                   /* ADR 1+2 */
    {340300008u, 0, 0, {340300010u, 0, 0}},                   /* ADR 2+3 */
    {340300009u, 0, 0, {340300010u, 0, 0}},                   /* ADR 1+3 */
    {340300010u, 0, 0, {0, 0, 0}},                            /* ADR 1+2+3 */
    {340300011u, 0, 0, {340300029u, 0, 0}},                   /* TCAS 1 */
    {340300012u, 0, 0, {340300029u, 0, 0}},                   /* TCAS 2 */
    {340300022u, 1, 1, {340300025u, 340300026u, 340300028u}}, /* RA SYS A */
    {340300023u, 1, 1, {340300025u, 340300027u, 340300028u}}, /* RA SYS B */
    {340300024u, 1, 1, {340300026u, 340300027u, 340300028u}}, /* RA SYS C */
    {340300025u, 1, 0, {340300028u, 0, 0}},                   /* RA SYS A+B */
    {340300026u, 1, 0, {340300028u, 0, 0}},                   /* RA SYS A+C */
    {340300027u, 1, 0, {340300028u, 0, 0}},                   /* RA SYS B+C */
    {340300028u, 1, 0, {0, 0, 0}},                            /* RA SYS A+B+C */
    {340300029u, 0, 0, {0, 0, 0}},                            /* TCAS 1+2 */
    {340300030u, 0, 0, {340300033u, 340300034u, 340300036u}}, /* IR 1 */
    {340300031u, 0, 0, {340300033u, 340300035u, 340300036u}}, /* IR 2 */
    {340300032u, 0, 0, {340300034u, 340300035u, 340300036u}}, /* IR 3 */
    {340300033u, 0, 0, {340300036u, 0, 0}},                   /* IR 1+2 */
    {340300034u, 0, 0, {340300036u, 0, 0}},                   /* IR 1+3 */
    {340300035u, 0, 0, {340300036u, 0, 0}},                   /* IR 2+3 */
    {340300036u, 0, 0, {0, 0, 0}},                            /* IR 1+2+3 */
    {700300001u, 1, 0, {700300003u, 0, 0}},                   /* ENG 2 REVERSER */
    {700300002u, 1, 0, {700300003u, 0, 0}},                   /* ENG 3 REVERSER */
    {700300003u, 1, 0, {0, 0, 0}},                            /* ENG 2+3 REVERSER */
};
#define FWSS_NUM_INOP ((int)(sizeof fwss_inop / sizeof fwss_inop[0]))

static const FwsDictItem fwss_lims[] = {
    {1u, 0, 1, {0, 0, 0}},         /* LAND ASAP (landAsap), pfd */
    {260400001u, 0, 0, {0, 0, 0}}, /* APU BLEED DO NOT USE (fireButtonEng1) */
    {290400001u, 1, 0, {0, 0, 0}}, /* SLATS SLOW (allSlatSysFault) */
    {290400002u, 1, 0, {0, 0, 0}}, /* FLAPS SLOW (allFlapSysFault) */
};
#define FWSS_NUM_LIMS ((int)(sizeof fwss_lims / sizeof fwss_lims[0]))
#define FWSS_SUP_INOP0 FWSS_NUM_ALERTS
#define FWSS_SUP_LIM0 (FWSS_NUM_ALERTS + FWSS_NUM_INOP)
#define FWSS_NUM_SUP (FWSS_NUM_ALERTS + FWSS_NUM_INOP + FWSS_NUM_LIMS)
_Static_assert(sizeof fwss_alerts / sizeof fwss_alerts[0] + sizeof fwss_inop / sizeof fwss_inop[0] +
                       sizeof fwss_lims / sizeof fwss_lims[0] <= FWS_SIG_MAX_SUPPRESSABLE,
               "raise FWS_SIG_MAX_SUPPRESSABLE");

/* ============================================================================================== */
/* Small helpers                                                                                 */
/* ============================================================================================== */

static int fwss_ra_normal(const FwsInputs *in, int i) { return in->ra_ssm[i] == FWS_SSM_NORMAL; }
static int fwss_ra_ncd(const FwsInputs *in, int i) { return in->ra_ssm[i] == FWS_SSM_NCD; }
static int fwss_ra_fw(const FwsInputs *in, int i) { return in->ra_ssm[i] == FWS_SSM_FAILURE; }
/* Arinc429Register.valueOr(def): value only for NormalOperation */
static double fwss_ra_or(const FwsInputs *in, int i, double def) { return fwss_ra_normal(in, i) ? in->ra_ft[i] : def; }

static int fwss_count4(int a, int b, int c, int d) { return (a != 0) + (b != 0) + (c != 0) + (d != 0); }

static int fwss_ac(const FwsInputs *in, int i) { return !in->elec_valid || in->ac_powered[i]; }
static int fwss_ac_ess(const FwsInputs *in) { return !in->elec_valid || in->ac_ess_powered; }
static int fwss_dc2(const FwsInputs *in) { return !in->elec_valid || in->dc2_powered; }
static int fwss_dc_ess(const FwsInputs *in) { return !in->elec_valid || in->dc_ess_powered; }

static int fwss_phase_in(int phase, unsigned mask) { return phase >= 1 && phase <= 12 && (mask & B(phase)) != 0; }

/* A380 FPPU synchro angle from the surface angle (inverse of A380Hydraulic FLAP/SLAT_FPPU_TO_SURFACE,
 * fbw-a380x/src/wasm/systems/a380_systems/src/hydraulic/mod.rs:1735-1745). */
static double fwss_interp_inv(double deg, const double *surf, const double *fppu, int n)
{
    int i;
    if (deg <= 0.0)
        return 0.0;
    for (i = 1; i < n; i++) {
        if (deg <= surf[i]) {
            double f = (deg - surf[i - 1]) / (surf[i] - surf[i - 1]);
            return fppu[i - 1] + f * (fppu[i] - fppu[i - 1]);
        }
    }
    return fppu[n - 1];
}

static double fwss_flap_fppu(double deg)
{
    static const double surf[] = {0.0, 8.0, 17.0, 22.0, 26.0, 33.0};
    static const double fppu[] = {122.2, 215.68, 259.28, 279.84, 297.59, 338.99};
    return fwss_interp_inv(deg, surf, fppu, 6);
}

static double fwss_slat_fppu(double deg)
{
    static const double surf[] = {0.0, 20.0, 23.0};
    static const double fppu[] = {0.0, 286.48, 327.39};
    return fwss_interp_inv(deg, surf, fppu, 3);
}

int fws_signals_flaps_conf_from_angles(float flap_deg, float slat_deg)
{
    /* SFCC FlapsConf: 0 CONF0, 1 CONF1, 2 1+F, 3 CONF2, 4 2S, 5 CONF3, 6 FULL (slat, flap) */
    static const float cfg[7][2] = {{0, 0}, {20, 0}, {20, 8}, {20, 17}, {23, 17}, {23, 26}, {23, 33}};
    int best = 0, i;
    float best_d = 1e30f;
    for (i = 0; i < 7; i++) {
        float ds = slat_deg - cfg[i][0], df = flap_deg - cfg[i][1];
        float d = ds * ds + df * df;
        if (d < best_d) {
            best_d = d;
            best = i;
        }
    }
    return best;
}

const char *fws_signals_code_str(unsigned id, char buf[16])
{
    snprintf(buf, 16, "%09u", id);
    return buf;
}

/* ============================================================================================== */
/* Inputs defaults                                                                               */
/* ============================================================================================== */
void fws_inputs_defaults(FwsInputs *in)
{
    int i;
    memset(in, 0, sizeof *in);
    for (i = 0; i < 4; i++) {
        in->ac_powered[i] = 1;
        in->gen_pb_on[i] = 1;
        in->gen_potential_normal[i] = 1;
        in->bat_pb_auto[i] = 1;
        in->eng_bleed_pb_auto[i] = 1;
        in->eng_pump_pb_auto[i][0] = in->eng_pump_pb_auto[i][1] = 1;
        in->feed_pump_main_on[i] = in->feed_pump_stby_on[i] = 1;
    }
    in->ac_ess_powered = in->ac_emer_powered = 1;
    in->dc1_powered = in->dc2_powered = in->dc_ess_powered = 1;
    in->dc_ess_inflt_powered = in->dc_eha_powered = 1;
    in->apu_gen_pb_on[0] = in->apu_gen_pb_on[1] = 1;
    in->comm_on[0] = in->comm_on[1] = 1;
    in->galley_auto = 1;
    in->bus_tie_auto = 1;
    in->ac_ess_feed_normal = 1;
    in->eng_mode_selector = 1;   /* NORM */
    in->n1_idle_pct = 19.0f;     /* overwritten by the FADEC idle N1 */
    in->pack_pb_on[0] = in->pack_pb_on[1] = 1;
    for (i = 0; i < 3; i++) {
        in->adr_pb_on[i] = in->ir_pb_on[i] = 1;
        in->ir_mode[i] = 1;
        in->ra_ft[i] = 0.0f;
        in->ra_ssm[i] = FWS_SSM_NORMAL;
    }
    in->air_data_knob = in->att_hdg_knob = 1;
    in->normal_law = 1;
    in->sim_on_ground = in->main_gear_compressed = 1;
    in->gear_lever_down = 1;
    in->lg_grvty_switch_pos = 1; /* OFF (mip.xml GRVTY_LDG START_STATE 1) */
    in->nose_downlocked = in->main_left_downlocked = in->main_right_downlocked = 1;
    in->gear_extended_ratio = 1.0f;
    in->nose_gear_ratio = 1.0f;
    in->antiskid_on = 1;
    in->flaps_conf_index = -1;
    in->fms_to_flaps = -1;
    in->fms_zfw_zfwcg_set = 1;
    in->cargo_fwd_locked = in->cargo_aft_locked = 1;
    in->xpdr_alt_reporting = 1;
    in->tcas_mode = 2;           /* TA/RA */
    in->tat_c = 15.0f;
    in->sd_fail_page = FWS_SD_NONE;
}

/* ============================================================================================== */
/* init                                                                                          */
/* ============================================================================================== */
void fws_signals_init(FwsSignals *s)
{
    int i, j;
    memset(s, 0, sizeof *s);
    fws_inputs_defaults(&s->in);
    s->flight_phase = 1; /* FwsCore flightPhase = Subject.create(FwcFlightPhase.ElecPwr) */
    s->prev_flight_phase = 1;
    /* FwsFlightPhases constructor */
    fwss_conf_init(&s->fp_gnd_memo, 1.0, 1);
    fwss_conf_init(&s->fp_one_eng_running_conf, 30.0, 1);
    fwss_mem_init(&s->fp_speed_above_80kts, 1);
    fwss_mem_init(&s->fp_speed_above_v1, 1);
    fwss_conf_init(&s->fp_mct_memo, 60.0, 0);
    fwss_conf_init(&s->fp_fire_pb_out_conf, 0.2, 1);
    fwss_mtrig_init(&s->fp_fire_pb_out_memo, 2.0, 1, 0);
    fwss_mem_init(&s->fp_fire_pb_clear12, 0);
    fwss_mtrig_init(&s->fp_phase112_memo, 300.0, 1, 0);
    fwss_mtrig_init(&s->fp_phase10_ground_memo, 2.0, 1, 0);
    fwss_mtrig_init(&s->fp_ac80kts_memo, 2.0, 1, 0);
    fwss_mtrig_init(&s->fp_prev_phase11_invert_memo, 3.0, 0, 0);
    fwss_mtrig_init(&s->fp_two_eng_to_power_invert_memo, 1.0, 0, 0);
    fwss_mem_init(&s->fp_phase9_nvm, 1);
    fwss_mtrig_init(&s->fp_ground_immediate_memo, 2.0, 1, 0);
    fwss_mtrig_init(&s->fp_phase6_memo, 15.0, 1, 0);
    fwss_mtrig_init(&s->fp_phase7_memo, 120.0, 1, 0);
    fwss_mtrig_init(&s->fp_phase89_memo, 180.0, 1, 0);
    fwss_conf_init(&s->memo_to_conf01, 120.0, 1);
    fwss_mem_init(&s->memo_to_memo, 0);
    fwss_conf_init(&s->memo_ldg_conf01, 1.0, 1);
    fwss_mem_init(&s->memo_ldg_inhibit, 0);
    fwss_conf_init(&s->memo_ldg_conf02, 10.0, 1);
    fwss_mem_init(&s->memo_ldg_below2000, 1);
    /* FwsCore members */
    fwss_pulse_init(&s->to_config_pulse, 1);
    fwss_pulse_init(&s->rcl_pulse, 1);
    fwss_pulse_init(&s->phase3_pulse, 1);
    fwss_pulse_init(&s->phase_change_pulse, 1);
    fwss_pulse_init(&s->mw_pulse, 1);
    fwss_pulse_init(&s->mc_pulse, 1);
    fwss_mtrig_init(&s->to_config_trigger, 0.5, 1, 0);
    fwss_mtrig_init(&s->to_config_held_1s5, 1.5, 1, 0);
    fwss_mem_init(&s->flight_phase_inhibit_override, 0);
    fwss_conf_init(&s->to_inhibit_timer, 3.0, 1);
    fwss_conf_init(&s->ldg_inhibit_timer, 3.0, 1);
    fwss_conf_init(&s->phase8_conf60, 60.0, 1);
    fwss_conf_init(&s->phase8_conf180, 180.0, 1);
    fwss_conf_init(&s->phase815min, 900.0, 1);
    fwss_conf_init(&s->phase1112_1min, 60.0, 1);
    fwss_conf_init(&s->phase10_90s, 90.0, 1);
    fwss_mem_init(&s->phase12_shutdown_memory, 1);
    fwss_conf_init(&s->shutdown_50min_conf, 3000.0, 1);
    fwss_conf_init(&s->v1_conf, 0.3, 1);
    fwss_mtrig_init(&s->ignore_ra_on_ground_trigger, 10.0, 1, 0);
    for (i = 0; i < 3; i++)
        fwss_mem_init(&s->ra_on_ground_mem[i], 1);
    fwss_conf_init(&s->on_ground_conf, 1.0, 1);
    for (i = 0; i < 4; i++) {
        fwss_conf_init(&s->eng_not_starting_conf[i], 5.0, 0);
        fwss_mem_init(&s->eng_was_running_mem[i], 1);
        fwss_mem_init(&s->eng_fail_mem[i], 1);
        fwss_pulse_init(&s->eng_master_on_pulse[i], 1);
        fwss_conf_init(&s->feed_low_conf[i], 30.0, 1);
        for (j = 0; j < 2; j++)
            fwss_conf_init(&s->eng_pump_off_conf[i][j], 10.0, 1);
    }
    fwss_conf_init(&s->eng12_to_power_conf, 60.0, 0);
    fwss_conf_init(&s->eng34_to_power_conf, 60.0, 0);
    fwss_conf_init(&s->engines_off_on_ground_conf, 7.0, 1);
    /* AP OFF */
    fwss_pulse_init(&s->ap_disengaged_pulse, 0);
    fwss_pulse_init(&s->ap_disc_pressed_pulse, 1);
    fwss_pulse_init(&s->ap_vol_disc_pulse, 1);
    fwss_pulse_init(&s->ap_vol_first_cc_ended_pulse, 0);
    fwss_mtrig_init(&s->ap_disc_in_1p9, 1.9, 1, 0);
    fwss_mtrig_init(&s->ap_disc_twice_in_1p9, 1.9, 1, 0);
    fwss_mtrig_init(&s->ap_first_cc_0p3, 0.3, 1, 0);
    fwss_mtrig_init(&s->ap_vol_end_1p9, 1.9, 1, 0);
    fwss_mtrig_init(&s->ap_vol_first_cc_active, 1.2, 1, 0);
    fwss_mem_init(&s->ap_off_vol_mem, 1);
    fwss_mem_init(&s->ap_off_invol_mem, 0);
    fwss_mem_init(&s->ap_off_unack, 0);
    /* A/THR */
    fwss_pulse_init(&s->athr_disengaged_pulse, 0);
    fwss_pulse_init(&s->athr_disc_pressed_pulse, 1);
    fwss_pulse_init(&s->phase1_pulse, 1);
    fwss_pulse_init(&s->athr_vol_mem_down_pulse, 0);
    fwss_pulse_init(&s->athr_off_vol_cond_pulse, 0);
    fwss_mtrig_init(&s->athr_idle_mtrig, 2.0, 1, 1);
    fwss_mtrig_init(&s->athr_vol_off_mtrig, 1.0, 1, 1);
    fwss_mtrig_init(&s->athr_vol_caution_mtrig, 3.0, 0, 0);
    fwss_mtrig_init(&s->athr_vol_memo_mtrig, 9.0, 0, 0);
    fwss_mem_init(&s->athr_vol_disc_mem, 0);
    fwss_mem_init(&s->athr_vol_caution_mem, 1);
    fwss_mem_init(&s->athr_vol_memo_mem, 1);
    fwss_mem_init(&s->athr_invol_warning_mem, 1);
    fwss_mem_init(&s->athr_invol_caution_mem, 1);
    fwss_mem_init(&s->athr_invol_pfd_memo_mem, 0);
    fwss_conf_init(&s->athr_limited_conf, 5.0, 1);
    fwss_mtrig_init(&s->athr_limited_mtrig, 5.0, 1, 0);
    fwss_conf_init(&s->eng_thrust_locked_conf, 5.0, 1);
    fwss_mtrig_init(&s->eng_thrust_locked_mtrig, 5.0, 1, 0);
    /* autobrake */
    fwss_conf_init(&s->abrk_off_conf, 1.0, 1);
    fwss_pulse_init(&s->abrk_deactivated_pulse, 1);
    fwss_mtrig_init(&s->abrk_athr_disc_trig, 0.5, 1, 1);
    fwss_mtrig_init(&s->abrk_deactivated_node, 9.0, 1, 1);
    fwss_mtrig_init(&s->abrk_off_mc_node, 3.0, 1, 1);
    fwss_mem_init(&s->abrk_off_mem, 0);
    fwss_mem_init(&s->abrk_off_mc_mem, 1);
    /* configuration */
    fwss_mem_init(&s->flap_config_sr, 1);
    fwss_mem_init(&s->slat_config_sr, 1);
    fwss_mem_init(&s->spdbrk_config_sr, 1);
    fwss_mem_init(&s->pitch_config_sr, 1);
    fwss_mem_init(&s->rudder_config_sr, 1);
    fwss_conf_init(&s->trim_disagree_conf, 1.0, 1);
    fwss_pulse_init(&s->to_config_no_speeds_pulse, 1);
    fwss_conf_init(&s->to_config_normal_conf, 0.3, 0);
    fwss_conf_init(&s->spdbrk_5s, 5.0, 1);
    fwss_conf_init(&s->spdbrk_50s, 50.0, 1);
    fwss_conf_init(&s->eng_above_idle_spdbrk, 10.0, 0);
    fwss_conf_init(&s->spdbrk_caution1_conf, 30.0, 1);
    fwss_pulse_init(&s->spdbrk_caution1_pulse, 1);
    fwss_pulse_init(&s->spdbrk_caution2_pulse, 1);
    fwss_mtrig_init(&s->phase10_4s5, 4.5, 0, 0);
    fwss_conf_init(&s->gnd_splr_5s, 5.0, 0);
    fwss_conf_init(&s->spdbrk_5s_delayed, 5.0, 0);
    fwss_mem_init(&s->adr3_overspeed_mem, 0);
    fwss_conf_init(&s->park_brake_2s, 2.0, 1);
    fwss_mem_init(&s->config_park_brk_mem, 1);
    fwss_pulse_init(&s->lg_not_down_pulse1, 1);
    fwss_pulse_init(&s->lg_not_down_pulse2, 1);
    fwss_conf_init(&s->gear_not_locked_down_conf, 30.0, 1);
    fwss_conf_init(&s->gear_not_locked_up_conf, 30.0, 1);
    /* surveillance */
    fwss_conf_init(&s->tcas_stby_conf, 3.0, 1);
    fwss_conf_init(&s->tcas_stby_memo_conf, 3.0, 1);
    fwss_conf_init(&s->tcas1_adr_ir_conf, 3.0, 0);
    fwss_conf_init(&s->tcas2_adr_ir_conf, 3.0, 0);
    fwss_conf_init(&s->tcas1_fault_conf, 3.0, 1);
    fwss_conf_init(&s->tcas2_fault_conf, 3.0, 1);
    /* hydraulics */
    fwss_conf_init(&s->eng12_running_phase_conf, 1.0, 1);
    fwss_conf_init(&s->eng34_running_phase_conf, 1.0, 1);
    s->green_press_sw = s->yellow_press_sw = 1;
    /* ice */
    fwss_conf_init(&s->ice_det1, 40.0, 0);
    fwss_conf_init(&s->ice_det2, 5.0, 1);
    fwss_conf_init(&s->ice_severe, 40.0, 0);
    fwss_conf_init(&s->ice_not_det1, 60.0, 1);
    fwss_conf_init(&s->ice_not_det2, 130.0, 1);
    /* SD (FwsSystemDisplayLogic constructor values) */
    s->sd_crz_timer = 60.0;
    s->sd_fctl_timer = 20.0;
    s->sd_eng_timer = -1.0;
    s->sd_apu_timer = -1.0;
    s->sd_page_when_unselected = FWS_SD_DOOR;
    s->lists.ecam_status_normal = 1;
}

/* ============================================================================================== */
/* Flight phases (FwsFlightPhases.ts:181-424)                                                    */
/* ============================================================================================== */

static double fwss_cas2(const FwsSignals *s)
{
    /* computedAirSpeedToNearest2 (FwsCore:1689): FWS 1 uses ADR 1, ADR 3 when CAPT ON 3 */
    const FwsInputs *in = &s->in;
    int a = in->air_data_knob == 0 ? 2 : 0;
    double cas = in->adr_cas_valid[a] ? in->adr_cas_kt[a] : 0.0;
    return floor(cas / 2.0 + 0.5) * 2.0;
}

static double fwss_mach(const FwsSignals *s)
{
    const FwsInputs *in = &s->in;
    int a = in->air_data_knob == 0 ? 2 : 0;
    return in->adr_mach_valid[a] ? in->adr_mach[a] : 0.0;
}

static void fwss_update_flight_phase(FwsSignals *s, double dt)
{
    const FwsInputs *in = &s->in;
    int ra1_inv = fwss_ra_fw(in, 0) || fwss_ra_ncd(in, 0);
    int ra2_inv = fwss_ra_fw(in, 1) || fwss_ra_ncd(in, 1);
    int rh = ra1_inv ? (ra2_inv ? 2 : 1) : 0;
    int one_eng_running_raw = 0, one_engine_running, no_engine_running, h_fail, ground_immediate, ground;
    double ias, v1;
    int above80, above_v1, h_abv1500, h_abv800, h_abv400;
    int i, n_mcl = 0, n_to = 0, two_eng_mcl, two_eng_to_signal, two_eng_to_power;
    int eng1_fire_pb_memo, reset_fire_pb_clear12;
    int phase10, phase345_cond, phase3, phase4, phase5, set_p11, reset_p11, phase11_nvm, phase211_cond;
    int phase11, phase2, phase112_memo_a, phase112_cond, phase112_memo, phase1, phase12;
    int ground_2s, phase6_cond, phase6, phase7_cond, phase7, phase89_cond, phase89_memo, phase8, phase9;
    int phases[13], active[12], n_active = 0;

    if (in->eng_valid)
        for (i = 0; i < 4; i++)
            if (in->n1_pct[i] > 15.0f)
                one_eng_running_raw = 1;
    one_engine_running = fwss_conf(&s->fp_one_eng_running_conf, one_eng_running_raw, dt);
    no_engine_running = !one_engine_running;
    h_fail = fwss_ra_fw(in, 0) && fwss_ra_fw(in, 1) && fwss_ra_fw(in, 2);
    ground_immediate = in->sim_on_ground != 0;
    ground = fwss_conf(&s->fp_gnd_memo, ground_immediate, dt);

    ias = fwss_cas2(s);
    above80 = fwss_mem(&s->fp_speed_above_80kts, ias > 83.0, ias < 77.0);
    v1 = in->fms_valid ? in->v1_kt : 0.0;
    above_v1 = v1 > 0.0 ? fwss_mem(&s->fp_speed_above_v1, ias > v1 + 3.0, ias < v1 - 3.0) : 0;

    h_abv1500 = fwss_ra_ncd(in, rh) || in->ra_ft[rh] > 1500.0f;
    h_abv800 = fwss_ra_ncd(in, rh) || in->ra_ft[rh] > 800.0f;
    h_abv400 = fwss_ra_ncd(in, rh) || in->ra_ft[rh] > 400.0f;

    for (i = 0; i < 4; i++) {
        double tla = in->tla_valid ? in->tla_deg[i] : 0.0;
        int fto = in->flex_temp_set != 0;
        int mct = tla > 33.3 && tla < 36.7;
        int full = tla > 43.3; /* [DEV 1] FBW eng4TLAFullPwr reads eng3TLA */
        int mcl = tla > 22.9;
        int sup_mct = !(tla < 36.7);
        n_mcl += mcl;
        n_to += (fto && mct) || full || sup_mct;
    }
    two_eng_mcl = n_mcl >= 2;
    two_eng_to_signal = n_to >= 2;
    two_eng_to_power = two_eng_to_signal ||
                       (fwss_conf(&s->fp_mct_memo, two_eng_to_signal, dt) && !h_abv1500 && two_eng_mcl);

    eng1_fire_pb_memo = fwss_mtrig(&s->fp_fire_pb_out_memo,
                                   fwss_conf(&s->fp_fire_pb_out_conf, in->eng_fire_pb_released[0], dt), dt);
    reset_fire_pb_clear12 = eng1_fire_pb_memo && ground;

    phase10 = (fwss_mtrig(&s->fp_phase10_ground_memo, ground_immediate, dt) || ground_immediate) &&
              !two_eng_to_power && above80;
    phase345_cond = ground && two_eng_to_power;
    phase3 = !above80 && one_eng_running_raw && phase345_cond;
    phase4 = above80 && phase345_cond && !above_v1;
    phase5 = above80 && phase345_cond && above_v1;

    set_p11 = phase3 || phase10;
    /* JS short-circuit evaluation order is kept: the monostables are only written when reached */
    reset_p11 = !fwss_mtrig(&s->fp_ac80kts_memo, !above80, dt) &&
                ((ground && fwss_mtrig(&s->fp_prev_phase11_invert_memo, s->fp_prev_phase11, dt)) ||
                 reset_fire_pb_clear12 ||
                 (ground && fwss_mtrig(&s->fp_two_eng_to_power_invert_memo, two_eng_to_power, dt))) &&
                !s->fp_prev_phase11;
    phase11_nvm = fwss_mem(&s->fp_phase9_nvm, set_p11, reset_p11);
    phase211_cond = ground && !two_eng_to_power && !above80;
    phase11 = one_eng_running_raw && phase11_nvm && phase211_cond;
    phase2 = phase211_cond && !phase11_nvm && one_eng_running_raw;

    phase112_memo_a = fwss_mem(&s->fp_fire_pb_clear12, phase11, reset_fire_pb_clear12);
    phase112_cond = !phase11 && no_engine_running && ground_immediate;
    phase112_memo = fwss_mtrig(&s->fp_phase112_memo, phase112_memo_a && phase112_cond, dt);
    phase1 = phase112_cond && !phase112_memo;
    phase12 = phase112_cond && phase112_memo;
    s->fp_prev_phase11 = (unsigned char)phase11;

    ground_2s = fwss_mtrig(&s->fp_ground_immediate_memo, ground_immediate, dt) || ground_immediate;
    phase6_cond = !h_abv400 && two_eng_to_power && !h_fail && !ground_2s;
    phase6 = fwss_mtrig(&s->fp_phase6_memo, phase6_cond, dt) && phase6_cond;
    phase7_cond = !phase6 && !h_abv1500 && two_eng_to_power && !h_fail && !ground_2s;
    phase7 = fwss_mtrig(&s->fp_phase7_memo, phase7_cond, dt) && phase7_cond;
    phase89_cond = !ground_2s && !h_fail && !two_eng_to_power && !h_abv1500 && !h_abv800;
    phase89_memo = fwss_mtrig(&s->fp_phase89_memo, phase89_cond, dt) && phase89_cond;
    phase8 = !phase7 && !ground_2s && !phase89_memo;
    phase9 = phase89_memo && !phase10;

    phases[1] = phase1; phases[2] = phase2; phases[3] = phase3; phases[4] = phase4;
    phases[5] = phase5; phases[6] = phase6; phases[7] = phase7; phases[8] = phase8;
    phases[9] = phase9; phases[10] = phase10; phases[11] = phase11; phases[12] = phase12;
    for (i = 1; i <= 12; i++)
        if (phases[i])
            active[n_active++] = i;
    if (n_active == 1) {
        s->flight_phase = active[0];
    } else if (n_active > 1) {
        int keep = 0;
        for (i = 0; i < n_active; i++)
            if (active[i] == s->flight_phase)
                keep = 1;
        if (!keep)
            s->flight_phase = active[0];
    }
    /* no valid phase: keep the current one (FwsFlightPhases:338-341) */
}

/* FwsFlightPhases._updateTakeoffMemo (:344-366) */
static void fwss_update_to_memo(FwsSignals *s, double dt)
{
    int fp = s->flight_phase;
    /* [DEV 2] FBW never assigns FwsFlightPhases.toConfigTest; the T.O CONFIG 0.5 s pulse is used */
    int set = fp == 2 && s->to_config_test;
    int reset = fp == 12 || fp == 3 || fp == 1 || fp == 8;
    int memo = fwss_mem(&s->memo_to_memo, set, reset);
    int timer = fwss_conf(&s->memo_to_conf01,
                          s->eng_running[0] && s->eng_running[1] && s->eng_running[2] && s->eng_running[3], dt);
    s->to_memo = (unsigned char)(memo || (fp == 2 && timer));
}

/* FwsFlightPhases._updateLandingMemo (:368-414) */
static void fwss_update_ldg_memo(FwsSignals *s, double dt)
{
    const FwsInputs *in = &s->in;
    int fp = s->flight_phase;
    int inv[3], i, set_below = 0, reset_below = 1, memo2, set_inhibit, reset_inhibit, memo1, show_appr, inv_radio;
    /* GEAR TOTAL PCT EXTENDED is read in 'percent' and compared with 0.95 (FBW quirk kept) */
    int gear_downlocked = in->gear_extended_ratio * 100.0f > 0.95f;
    for (i = 0; i < 3; i++) {
        inv[i] = fwss_ra_fw(in, i) || fwss_ra_ncd(in, i);
        if (in->ra_ft[i] < 2000.0f && !inv[i])
            set_below = 1;
        if (!(in->ra_ft[i] > 2200.0f || inv[i]))
            reset_below = 0;
    }
    memo2 = fwss_mem(&s->memo_ldg_below2000, set_below, reset_below);
    set_inhibit = fwss_conf(&s->memo_ldg_conf01, reset_below && !inv[0] && !inv[1] && !inv[2], dt);
    reset_inhibit = !(fp == 9 || fp == 10 || fp == 8);
    memo1 = fwss_mem(&s->memo_ldg_inhibit, set_inhibit, reset_inhibit);
    show_appr = memo1 && memo2 && fp == 8;
    inv_radio = fwss_conf(&s->memo_ldg_conf02, inv[0] && inv[1] && inv[2] && gear_downlocked && fp == 8, dt);
    s->ldg_memo = (unsigned char)(show_appr || inv_radio || fp == 10 || fp == 9);
}

/* FwsFlightPhases._updateAltitudeWarning (:416-517) */
static void fwss_reset_alt(FwsSignals *s)
{
    s->alt_was_below = s->alt_was_above = s->alt_was_in_range = 0;
    s->alt_cchord_short_triggered = 0;
}

static void fwss_update_altitude_warning(FwsSignals *s)
{
    const FwsInputs *in = &s->in;
    double target, delta;
    s->aural_cchord_once = 0;
    if (in->master_warning_pb) {
        fwss_reset_alt(s);
        s->aural_cchord_cont = 0;
        return;
    }
    if (in->sim_on_ground)
        s->aural_cchord_cont = 0;
    target = (in->fg_alt_constraint_ft != 0.0f && in->alt_constraint_is_target) ? in->fg_alt_constraint_ft
                                                                                : in->fcu_alt_ft;
    if (!s->alt_prev_target_valid || s->alt_prev_target != target) {
        s->alt_prev_target = target;
        s->alt_prev_target_valid = 1;
        fwss_reset_alt(s);
        s->alt_was_reach200 = 0;
        s->aural_cchord_cont = 0;
        return;
    }
    if ((s->in.flaps_handle_index >= 1 && in->gear_lever_down) || in->gs_captured || in->nose_gear_ratio > 0.9f ||
        in->tcas_ra_active) {
        fwss_reset_alt(s);
        s->alt_was_reach200 = 0;
        s->aural_cchord_cont = 0;
        return;
    }
    delta = fabs(in->indicated_alt_ft - target);
    if (delta < 200.0) {
        s->alt_was_below = 1;
        s->alt_was_above = 0;
        s->alt_was_reach200 = 1;
    }
    if (delta > 750.0) {
        s->alt_was_above = 1;
        s->alt_was_below = 0;
        s->alt_cchord_short_triggered = 0;
    }
    if (delta >= 200.0 && delta <= 750.0)
        s->alt_was_in_range = 1;
    if (s->alt_was_below && s->alt_was_reach200) {
        s->aural_cchord_cont = delta >= 200.0;
    } else if (s->alt_was_above && delta <= 750.0 && !s->alt_was_reach200 && !s->alt_cchord_short_triggered &&
               !in->ap_engaged) {
        s->alt_cchord_short_triggered = 1;
        s->aural_cchord_cont = 0;
        s->aural_cchord_once = 1;
    } else if (delta > 750.0 && s->alt_was_in_range && !s->alt_was_reach200) {
        s->aural_cchord_cont = 1;
    }
}

/* ============================================================================================== */
/* FwsCore signal computations (FwsCore.ts update(), ~2850-5470)                                 */
/* ============================================================================================== */

static int fwss_phase12_or_1112(int fp) { return fp == 1 || fp == 2 || fp >= 11; }

static void fwss_update_core(FwsSignals *s, double dt)
{
    const FwsInputs *in = &s->in;
    const int fp = s->flight_phase;
    const int phase3 = fp == 3, phase6 = fp == 6, phase8 = fp == 8;
    const int phase112 = fp == 1 || fp == 12;
    const int phase1211 = fp == 1 || fp == 2 || fp == 11;
    const int phase345 = fp == 3 || fp == 4 || fp == 5;
    const int phase23 = fp == 2 || fp == 3;
    int i, j;
    int mw_pressed, mc_pressed;
    double cas2 = fwss_cas2(s), mach = fwss_mach(s);
    int conf_index;
    int ac2 = fwss_ac(in, 1), ac4 = fwss_ac(in, 3);
    int ac_ess = fwss_ac_ess(in), dc2 = fwss_dc2(in), dc_ess = fwss_dc_ess(in);
    int one_eng_above_min_power = 0;
    int all_throttle_idle, tla_idle[4];
    int below50, ra_valid_all, eng_not_running;
    double median_ra = 0.0;

    fwss_pulse(&s->phase3_pulse, phase3);
    s->phase1112_more_than_1min = (unsigned char)fwss_conf(&s->phase1112_1min, fp >= 11, dt);
    fwss_conf(&s->phase815min, fp == 8, dt);
    fwss_mem(&s->phase12_shutdown_memory, fp == 12, !phase112);
    s->checklist_reset_50min = (unsigned char)fwss_conf(&s->shutdown_50min_conf, s->phase12_shutdown_memory.value, dt);

    /* T.O / LDG INHIBIT (FwsCore:2984-2990). [DEV 3] the override is reset on a flight phase change */
    fwss_mem(&s->flight_phase_inhibit_override, s->rcl_pulse.out, s->phase_change_pulse.out);
    s->show_takeoff_inhibit = (unsigned char)fwss_conf(
        &s->to_inhibit_timer, fp >= 3 && fp <= 7 && !s->flight_phase_inhibit_override.value, dt);
    s->show_landing_inhibit = (unsigned char)fwss_conf(
        &s->ldg_inhibit_timer, (fp == 9 || fp == 10) && !s->flight_phase_inhibit_override.value, dt);

    conf_index = in->flaps_conf_index >= 0
                     ? in->flaps_conf_index
                     : (in->flap_slat_valid ? fws_signals_flaps_conf_from_angles(in->flap_angle_deg, in->slat_angle_deg)
                                            : 0);

    /* engines (FwsCore:3055-3080) */
    s->one_eng_running = 0;
    for (i = 0; i < 4; i++) {
        s->eng_running[i] = (unsigned char)(in->eng_valid && in->eng_state[i] == FWS_ENG_ON);
        s->one_eng_running |= s->eng_running[i];
        if (in->eng_valid && in->n1_pct[i] > in->n1_idle_pct + 2.0f)
            one_eng_above_min_power = 1;
        tla_idle[i] = !in->tla_valid || in->tla_deg[i] <= 2.6f; /* [DEV 4] FBW reads TLA 2 for levers 3, 4 */
    }
    all_throttle_idle = tla_idle[0] && tla_idle[1] && tla_idle[2] && tla_idle[3];

    /* APU bleed envelope, ENG BLEED OFF (FwsCore:3124-3147) */
    s->apu_bleed_outside_envelope = (unsigned char)(in->apu_bleed_pb_on &&
                                                    !(s->adr_pressure_alt < 22500.0 && (mach < 0.56 || s->all_engines_failure)));
    for (i = 0; i < 4; i++)
        s->eng_bleed_abnormal_off[i] = (unsigned char)(s->eng_running[i] && !in->eng_bleed_pb_auto[i]);

    mc_pressed = s->mc_pulse.out;
    mw_pressed = s->mw_pulse.out;

    /* hydraulics (FwsCore:3218-3428). Pressure switch hysteresis 2900/3700 psi (A380 hydraulic mod.rs:207-210) */
    if (!in->hyd_valid) {
        s->green_press_sw = s->yellow_press_sw = 1;
    } else {
        if (in->hyd_green_psi >= 3700.0f) s->green_press_sw = 1;
        else if (in->hyd_green_psi <= 2900.0f) s->green_press_sw = 0;
        if (in->hyd_yellow_psi >= 3700.0f) s->yellow_press_sw = 1;
        else if (in->hyd_yellow_psi <= 2900.0f) s->yellow_press_sw = 0;
    }
    {
        int p121112 = fwss_phase12_or_1112(fp);
        int c12 = fwss_conf(&s->eng12_running_phase_conf, s->eng_running[0] || s->eng_running[1] || !p121112, dt);
        int c34 = fwss_conf(&s->eng34_running_phase_conf, s->eng_running[2] || s->eng_running[3] || !p121112, dt);
        int below[4][2];
        s->eng12_34_running_phase = (unsigned char)(c12 && c34);
        s->green_lo = (unsigned char)(!s->green_press_sw && c12);
        s->yellow_lo = (unsigned char)(!s->yellow_press_sw && c34);
        s->green_yellow_lo = (unsigned char)(s->green_lo && s->yellow_lo);
        for (i = 0; i < 4; i++) {
            int sys_lo = i < 2 ? s->green_lo : s->yellow_lo;
            for (j = 0; j < 2; j++) {
                below[i][j] = in->hyd_valid && in->eng_pump_section_low[i][j];
                /* [DEV 5] FBW eng4A reads `!this.eng4APumpAuto` / `this.engine4Running` (objects) */
                fwss_conf(&s->eng_pump_off_conf[i][j], !sys_lo && fp == 2 && !in->eng_pump_pb_auto[i][j], dt);
                s->eng_pump_fault[i][j] = (unsigned char)(fwss_conf_read(&s->eng_pump_off_conf[i][j]) ||
                                                          (s->eng_running[i] && below[i][j] && !s->green_yellow_lo));
            }
        }
        s->three_yellow_pumps_failed =
            (unsigned char)(below[2][0] + below[2][1] + below[3][0] + below[3][1] > 3);
    }

    /* ADIRS (FwsCore:3471-3520) */
    s->adr_faulty[0] = (unsigned char)(!(!ac_ess || phase112) && in->adr_fault[0]);
    s->adr_faulty[1] = (unsigned char)(!(!ac4 || phase112) && in->adr_fault[1]);
    s->adr_faulty[2] = (unsigned char)(!(!ac2 || phase112) && in->adr_fault[2]);
    for (i = 0; i < 3; i++) {
        s->ir_fault[i] = (unsigned char)(!phase112 && in->ir_fault[i]);
        s->ra_failed[i] = (unsigned char)fwss_ra_fw(in, i);
    }
    s->adr_pressure_alt_valid = 0;
    s->adr_pressure_alt = 0.0;
    for (i = 0; i < 3; i++)
        if (in->adr_alt_valid[i]) {
            s->adr_pressure_alt = in->adr_alt_ft[i];
            s->adr_pressure_alt_valid = 1;
            break;
        }

    /* V1 callout (FwsCore:3540-3557) */
    {
        double v1 = in->fms_valid ? in->v1_kt : 0.0;
        int prev = fwss_conf_read(&s->v1_conf), cond = 0;
        for (i = 0; i < 3; i++)
            if (in->adr_cas_valid[i] && in->adr_cas_kt[i] > v1 - 4.0)
                cond = 1;
        fwss_conf(&s->v1_conf, v1 > 0.0 && cond, dt);
        s->aural_v1 = (unsigned char)(fp == 4 && fwss_conf_read(&s->v1_conf) && !prev);
    }

    /* brakes, park brake (FwsCore:3559-3575) */
    s->brakes_hot = (unsigned char)(in->brakes_hot && !fwss_conf_read(&s->phase815min));
    s->lg_park_brk_on = (unsigned char)fwss_conf(&s->park_brake_2s, in->park_brake_set, dt);
    s->config_park_brk =
        (unsigned char)fwss_mem(&s->config_park_brk_mem, phase3 && in->park_brake_set, !in->park_brake_set || phase6);

    /* gear (FwsCore:3585-3593): LGCIU discrete word 1 bits 23/24 main, 25 nose */
    s->main_gear_downlocked = (unsigned char)(!in->gear_valid ||
                                              (in->main_left_downlocked && in->main_right_downlocked));
    s->all_gear_downlocked = (unsigned char)(s->main_gear_downlocked && (!in->gear_valid || in->nose_downlocked));

    /* on ground (FwsCore:3595-3666). LGCIU words are assumed healthy and in agreement with the hardwire */
    {
        int hw1 = dc_ess && in->main_gear_compressed;
        int hw2 = dc2 && in->main_gear_compressed;
        int on_ground_a = hw1 && hw2, ra_og[3], count, ra_invalid;
        fwss_mtrig(&s->ignore_ra_on_ground_trigger, fwss_ra_ncd(in, 0) && fwss_ra_ncd(in, 1) && fwss_ra_ncd(in, 2), dt);
        for (i = 0; i < 3; i++) {
            fwss_mem(&s->ra_on_ground_mem[i], in->ra_ft[i] < 5.0f, !hw1 || !hw2);
            ra_og[i] = fwss_ra_normal(in, i) && (in->ra_ft[i] < 5.0f || s->ra_on_ground_mem[i].value);
        }
        count = hw1 + hw2 + ra_og[0] + ra_og[1] + ra_og[2];
        ra_invalid = fwss_ra_fw(in, 0) || fwss_ra_fw(in, 1) || fwss_ra_fw(in, 2);
        s->on_ground_immediate = (unsigned char)((on_ground_a && fwss_mtrig_read(&s->ignore_ra_on_ground_trigger)) ||
                                                 (count > 2 && !ra_invalid) || (count > 1 && ra_invalid));
        s->aircraft_on_ground = (unsigned char)fwss_conf(&s->on_ground_conf, s->on_ground_immediate, dt);
    }

    /* AP OFF (FwsCore:3667-3744) */
    {
        int ap = in->ap_engaged != 0;
        int disengaged, before, vol, disc_after;
        disengaged = fwss_pulse(&s->ap_disengaged_pulse, ap);
        before = fwss_mtrig_read(&s->ap_disc_in_1p9);
        fwss_mtrig(&s->ap_disc_in_1p9, s->ap_disc_pressed_pulse.out, dt);
        vol = disengaged && fwss_mtrig_read(&s->ap_disc_in_1p9);
        fwss_mtrig(&s->ap_vol_end_1p9, vol, dt);
        fwss_pulse(&s->ap_vol_disc_pulse, vol);
        fwss_mtrig(&s->ap_vol_first_cc_active, s->ap_vol_disc_pulse.out, dt);
        fwss_pulse(&s->ap_vol_first_cc_ended_pulse, fwss_mtrig_read(&s->ap_vol_first_cc_active));
        fwss_mtrig(&s->ap_first_cc_0p3, fwss_mtrig_read(&s->ap_vol_first_cc_active), dt);
        fwss_mtrig(&s->ap_disc_twice_in_1p9,
                   s->ap_disc_pressed_pulse.out && (in->ap_instinctive_disc_count > 1 || before), dt);
        fwss_mem(&s->ap_off_vol_mem, s->ap_vol_disc_pulse.out,
                 ap || (fwss_mtrig_read(&s->ap_disc_twice_in_1p9) && !fwss_mtrig_read(&s->ap_first_cc_0p3)) ||
                     !fwss_mtrig_read(&s->ap_vol_end_1p9));
        disc_after = !disengaged && (s->ap_disc_pressed_pulse.out || mw_pressed);
        fwss_mem(&s->ap_off_unack, disengaged, ap || fwss_mtrig_read(&s->ap_disc_twice_in_1p9) || disc_after);
        fwss_mem(&s->ap_off_invol_mem, !ap && !s->ap_off_vol_mem.value && s->ap_off_unack.value,
                 !s->ap_off_unack.value);
        s->ap_off_involuntary = s->ap_off_invol_mem.value;
        s->ap_off_show_memo = (unsigned char)(s->ap_off_vol_mem.value || s->ap_off_invol_mem.value);
        if (disengaged) {
            s->mw_from_ap_off = 1;
            s->aural_cav_charge = 1;
        }
        if (s->ap_vol_first_cc_ended_pulse.out)
            s->aural_cav_charge = 0;
        if (!s->ap_off_vol_mem.value && !s->ap_off_invol_mem.value) {
            s->mw_from_ap_off = 0;
            s->aural_cav_charge = 0;
        }
        fwss_pulse(&s->ap_disc_pressed_pulse, 0);
    }

    /* A/THR OFF (FwsCore:3770-3856) */
    {
        int athr_disc = s->athr_disc_pressed_pulse.out;
        int engaged = in->athr_status == 2 || in->athr_mode_active;
        int disengaged = fwss_pulse(&s->athr_disengaged_pulse, engaged);
        int ra_v[3], idle_mtrig, all_rev, inhibit, idle_above50, cond, pre, mtrig, mem, mem_down, vol_down;
        int dis_or_mem, pulse_or_pre, vol_disc, reset_w, reset_c, unvol;
        s->athr_engaged_or_armed = (unsigned char)engaged;
        for (i = 0; i < 3; i++)
            ra_v[i] = fwss_ra_normal(in, i);
        ra_valid_all = ra_v[0] && ra_v[1] && ra_v[2];
        if (ra_valid_all) {
            double r1 = in->ra_ft[0], r2 = in->ra_ft[1], r3 = in->ra_ft[2];
            if (r1 < r2)
                median_ra = r2 < r3 ? r2 : (r1 < r3 ? r3 : r1);
            else
                median_ra = r1 < r3 ? r3 : (r2 < r3 ? r3 : r2);
        }
        below50 = s->aircraft_on_ground || (ra_valid_all && median_ra < 50.0) || fwss_ra_or(in, 0, 1e9) < 50.0 ||
                  fwss_ra_or(in, 1, 1e9) < 50.0 || fwss_ra_or(in, 2, 1e9) < 50.0;
        idle_mtrig = fwss_mtrig(&s->athr_idle_mtrig, all_throttle_idle, dt);
        /* [DEV 4] FBW thrustLever2/3Reverse = TLA2 > 0; reverse is TLA < 0 */
        all_rev = in->tla_valid && in->tla_deg[1] < 0.0f && in->tla_deg[2] < 0.0f;
        inhibit = (idle_mtrig || all_rev) && below50;
        idle_above50 = !below50 && all_throttle_idle && in->tla_valid;
        cond = idle_above50 || athr_disc;
        pre = engaged && cond;
        mtrig = fwss_mtrig(&s->athr_vol_off_mtrig, pre, dt);
        mem = fwss_mem(&s->athr_vol_disc_mem, pre, !mtrig || disengaged);
        mem_down = fwss_pulse(&s->athr_vol_mem_down_pulse, mem);
        vol_down = fwss_pulse(&s->athr_off_vol_cond_pulse, !mem_down && !cond && disengaged);
        dis_or_mem = disengaged || vol_down;
        pulse_or_pre = mem_down || cond;
        vol_disc = !engaged && dis_or_mem && pulse_or_pre;
        reset_w = fwss_pulse(&s->phase1_pulse, fp == 1) || engaged;
        reset_c = reset_w || (!dis_or_mem && mc_pressed) || (!dis_or_mem && athr_disc);
        s->vol_athr_off_caution = (unsigned char)fwss_mem(
            &s->athr_vol_caution_mem, vol_disc, !fwss_mtrig(&s->athr_vol_caution_mtrig, vol_disc, dt) || reset_c);
        s->vol_athr_off_memo = (unsigned char)fwss_mem(
            &s->athr_vol_memo_mem, vol_disc, !fwss_mtrig(&s->athr_vol_memo_mtrig, vol_disc, dt) || reset_c);
        unvol = !engaged && !inhibit && !pulse_or_pre && vol_down;
        s->athr_off_invol_warning = (unsigned char)fwss_mem(&s->athr_invol_warning_mem, unvol, reset_w);
        s->athr_off_invol_caution = (unsigned char)fwss_mem(&s->athr_invol_caution_mem, unvol, reset_c);
        fwss_mem(&s->athr_invol_pfd_memo_mem, unvol, reset_w || athr_disc);

        /* A/THR LIMITED (FwsCore:3858-3869) */
        s->athr_limited_clb = (unsigned char)(in->afs_a3 == FWS_A3_LVR_CLB);
        s->athr_limited_mct = (unsigned char)(in->afs_a3 == FWS_A3_LVR_MCT);
        fwss_conf(&s->athr_limited_conf,
                  !below50 && in->athr_status == 2 && (s->athr_limited_clb || s->athr_limited_mct), dt);
        fwss_mtrig(&s->athr_limited_mtrig, fwss_conf_read(&s->athr_limited_conf) && !s->athr_limited_delay, dt);
        s->athr_limited_delay = s->athr_limited_mtrig.out;
        s->athr_limited = (unsigned char)(fwss_conf_read(&s->athr_limited_conf) && s->athr_limited_delay);
        /* ENG THRUST LOCKED (FwsCore:3871-3884) */
        s->eng_thrust_locked = (unsigned char)(in->afs_a3 == FWS_A3_THR_LK);
        {
            int c5 = fwss_conf(&s->eng_thrust_locked_conf, s->eng_thrust_locked && !engaged, dt);
            fwss_mtrig(&s->eng_thrust_locked_mtrig, c5 && !s->eng_thrust_locked_delay, dt);
            s->eng_thrust_locked_delay = s->eng_thrust_locked_mtrig.out;
            s->eng_thrust_locked_alert = (unsigned char)(c5 && s->eng_thrust_locked_delay);
        }
    }

    /* AUTO BRK OFF (FwsCore:3886-3916) */
    {
        int off_conf = fwss_conf(&s->abrk_off_conf, !in->autobrake_active, dt);
        int off_pulse = fwss_pulse(&s->abrk_deactivated_pulse, off_conf);
        int athr_buf = fwss_mtrig(&s->abrk_athr_disc_trig, in->athr_instinctive_disc, dt);
        int il = in->att_hdg_knob == 0, ir = in->att_hdg_knob == 2;
        double gsl = il ? (in->ir_gs_valid[2] ? in->ir_gs_kt[2] : 0.0) : (in->ir_gs_valid[0] ? in->ir_gs_kt[0] : 0.0);
        double gsr = ir ? (in->ir_gs_valid[2] ? in->ir_gs_kt[2] : 0.0) : (in->ir_gs_valid[1] ? in->ir_gs_kt[1] : 0.0);
        int trig = off_pulse && (fp == 10 || fp == 11) && (gsl > 33.0 || gsr > 33.0);
        s->autobrake_off_memo = (unsigned char)fwss_mem(
            &s->abrk_off_mem, trig, !fwss_mtrig(&s->abrk_deactivated_node, trig, dt) || athr_buf);
        s->autobrake_off_mc = (unsigned char)fwss_mem(
            &s->abrk_off_mc_mem, trig, !fwss_mtrig(&s->abrk_off_mc_node, trig, dt) || athr_buf);
    }

    /* takeoff power (FwsCore:3921-3943) */
    {
        int flex = in->thrust_limit_flex != 0, tv = in->tla_valid;
        int to12 = tv && (in->tla_deg[0] >= 45.0f || (in->tla_deg[0] >= 35.0f && flex) || in->tla_deg[1] >= 45.0f ||
                          (in->tla_deg[1] >= 35.0f && flex));
        int to34 = tv && (in->tla_deg[2] >= 45.0f || (in->tla_deg[2] >= 35.0f && flex) || in->tla_deg[3] >= 45.0f ||
                          (in->tla_deg[3] >= 35.0f && flex));
        int ra1500 = fwss_ra_or(in, 0, 0) > 1500.0 || fwss_ra_or(in, 1, 0) > 1500.0 || fwss_ra_or(in, 2, 0) > 1500.0;
        fwss_conf(&s->eng12_to_power_conf, to12, dt);
        fwss_conf(&s->eng34_to_power_conf, to34, dt);
        s->eng12_to_power = (unsigned char)(to12 || (fwss_conf_read(&s->eng12_to_power_conf) && !ra1500));
        s->eng34_to_power = (unsigned char)(to34 || (fwss_conf_read(&s->eng34_to_power_conf) && !ra1500));
    }

    /* T.O speeds (FwsCore:3948-4021) */
    {
        int not_ins = in->fms_valid && in->fms_to_speeds_not_inserted;
        int too_low = in->fms_valid && in->fms_to_speeds_too_low;
        int disagree = in->fms_valid && in->fms_v_disagree;
        int raw = in->to_config_pb != 0;
        if (!phase23)
            s->to_config_checked_in_23 = 0;
        else if (raw)
            s->to_config_checked_in_23 = 1;
        fwss_pulse(&s->to_config_no_speeds_pulse, not_ins && raw);
        if (not_ins && (raw || phase3) && !s->to_speeds_not_inserted_rs)
            s->to_speeds_not_inserted_rs = 1;
        if (!(phase23 && not_ins) && s->to_speeds_not_inserted_rs)
            s->to_speeds_not_inserted_rs = 0;
        s->to_speeds_not_inserted = (unsigned char)(!s->to_config_no_speeds_pulse.out && s->to_speeds_not_inserted_rs &&
                                                    !s->phase3_pulse.out);
        s->to_speeds_too_low = (unsigned char)((s->to_config_checked_in_23 || phase3) && !s->to_config_pulse.out &&
                                               !s->phase3_pulse.out && too_low);
        s->to_v_disagree = (unsigned char)((s->to_config_checked_in_23 || phase3) && !s->to_config_pulse.out &&
                                           !s->phase3_pulse.out && disagree);
    }

    /* overspeed (FwsCore:3966-4005) + load analysis (:5039-5047) */
    {
        int ow = in->adr_overspeed[0] || in->adr_overspeed[1] ||
                 ((in->adr_fault[0] || in->adr_fault[1]) && in->adr_overspeed[2]);
        int os_vmo = cas2 > 340.0 + 4.0 || mach > 0.89 + 0.006; /* Vmo 340, Mmo 0.89 */
        int os_vle = cas2 > 250.0 + 4.0 || mach > 0.55 + 0.006; /* Vle 250, Mle 0.55 */
        s->overspeed_vmo = (unsigned char)(!s->all_gear_downlocked && conf_index == 0 && os_vmo);
        s->overspeed_vle = (unsigned char)(s->all_gear_downlocked && conf_index == 0 && os_vle);
        s->overspeed_vfe[0] = (unsigned char)(conf_index == 1 && cas2 > 263.0 + 4.0);
        s->overspeed_vfe[1] = (unsigned char)(conf_index == 2 && cas2 > 222.0 + 4.0);
        s->overspeed_vfe[2] = (unsigned char)(conf_index == 3 && cas2 > 220.0 + 4.0);
        s->overspeed_vfe[3] = (unsigned char)((conf_index == 4 || conf_index == 5) && cas2 > 196.0 + 4.0);
        s->overspeed_vfe[4] = (unsigned char)(conf_index == 6 && cas2 > 182.0 + 4.0);
        if (ow)
            s->overspeed_occurred = 1;
        if (fp == 1)
            s->overspeed_occurred = 0;
        s->load_analysis_required = s->overspeed_occurred;
    }

    /* air conditioning / pressurization (FwsCore:4110-4490) */
    s->flight_level = (int)floor(s->adr_pressure_alt / 100.0 + 0.5);
    fwss_conf(&s->phase8_conf60, phase8, dt);
    fwss_conf(&s->phase8_conf180, phase8, dt);
    for (i = 0; i < 2; i++)
        s->pack_off[i] = (unsigned char)(!in->pack_pb_on[i] && fwss_conf_read(&s->phase8_conf60));
    s->pack12_fault = (unsigned char)(!in->pack_pb_on[0] && !in->pack_pb_on[1] && fwss_conf_read(&s->phase8_conf180));
    eng_not_running = !s->eng_running[0] && !s->eng_running[1] && !s->eng_running[2] && !s->eng_running[3];
    fwss_conf(&s->engines_off_on_ground_conf, s->aircraft_on_ground && eng_not_running, dt);
    {
        /* CPCS is_excessive_alt (a380_systems air_conditioning/cpiom_b.rs:1322) */
        double cab = in->cabin_alt_ft;
        int ok_dep = !in->departure_elev_valid || cab > in->departure_elev_ft + 1000.0;
        int ok_ldg = !in->landing_elev_valid || cab > in->landing_elev_ft + 1000.0;
        s->excess_cab_alt = (unsigned char)(in->cabin_alt_valid && cab > 9550.0 && ok_dep && ok_ldg);
        s->excess_diff_press = (unsigned char)(in->cabin_alt_valid && in->cabin_diff_psi > 9.65f);
        fwss_conf(&s->phase10_90s, fp >= 10, dt);
        /* inhibitedByDoors (FwsCore:4400-4415): main doors except 1R, TLA 2 at T.O power */
        {
            const unsigned char *d = in->main_door_open;
            int cabin_door_open = d[0] || d[2] || d[3] || d[4] || d[5] || d[6] || d[7] || d[8] || d[9];
            s->inhibited_by_doors = (unsigned char)(cabin_door_open && in->tla_valid &&
                                                    (in->tla_deg[1] >= 45.0f ||
                                                     (in->tla_deg[1] >= 35.0f && in->thrust_limit_flex)));
        }
        s->excess_residual_diff_press =
            (unsigned char)(in->cabin_alt_valid && in->cabin_diff_psi > 0.072f &&
                            ((s->aircraft_on_ground && eng_not_running) || fwss_conf_read(&s->phase10_90s)));
        /* highLandingFieldElevation (FwsCore:4483-4488); cabin altitude only refreshed in 1/2/11/12 */
        {
            double v = 0.0;
            if (phase8)
                v = in->man_cab_alt_mode ? 0.0 : (in->landing_elev_valid ? in->landing_elev_ft : 0.0);
            else if (fwss_phase12_or_1112(fp) && in->cabin_alt_valid)
                v = cab;
            else if (!fwss_phase12_or_1112(fp))
                v = s->high_landing_field ? 8550.0 : 0.0; /* keep the last value outside 1/2/8/11/12 */
            s->high_landing_field = (unsigned char)(v >= 8550.0);
        }
    }

    /* fuel (FwsCore:4552-4572) */
    {
        int n_main = 0, n_stby = 0, n_xf = 0, any_master = 0;
        for (i = 0; i < 4; i++) {
            int stby = in->feed_pump_stby_valid ? in->feed_pump_stby_on[i] : in->feed_pump_main_on[i];
            /* [DEV 6] FBW writes feedTank1LowConfirm for all four tanks */
            s->feed_tank_low[i] =
                (unsigned char)fwss_conf(&s->feed_low_conf[i], in->fuel_valid && in->feed_tank_kg[i] < 1375.0f, dt);
            n_main += in->feed_pump_main_on[i] != 0;
            n_stby += stby != 0;
            n_xf += in->crossfeed_open[i] != 0;
            any_master |= in->eng_master_on[i];
        }
        s->all_crossfeed_open = (unsigned char)(n_xf == 4);
        s->crossfeed_open_memo = (unsigned char)(n_xf >= 2);
        s->all_feed_pumps_on = (unsigned char)(n_main == 4 && n_stby == 4);
        s->all_feed_pumps_off = (unsigned char)(in->fuel_valid && n_main == 0 && n_stby == 0);
        s->all_fuel_pumps_off = (unsigned char)(s->all_feed_pumps_off && !any_master);
    }

    /* flaps / slats / speed brakes / trims (FwsCore:4720-4950) */
    {
        double ffp = fwss_flap_fppu(in->flap_angle_deg), sfp = fwss_slat_fppu(in->slat_angle_deg);
        int v = in->flap_slat_valid, held = s->to_config_test_held;
        int sel1, sel2, sel3, mcdu0, mcdu1, mcdu2, mcdu3, pitch_cfg, rud_cfg;
        s->flaps_inf8 = (unsigned char)(v && ffp < 205.0);
        s->flaps_sup8 = (unsigned char)(v && ffp > 225.0);
        s->flaps_sup17 = (unsigned char)(v && ffp > 270.0);
        s->flaps_sup26 = (unsigned char)(v && ffp > 307.0);
        s->slats_inf20 = (unsigned char)(v && sfp < 276.0);
        s->flaps_conf3_full = (unsigned char)(s->flaps_sup17 || (v && sfp > 317.0));
        s->flaps_not_in_to_pos = (unsigned char)(s->flaps_sup26 || s->flaps_inf8);
        fwss_mem(&s->flap_config_sr, phase345 && s->flaps_not_in_to_pos,
                 !s->flaps_not_in_to_pos || phase6 || fp == 7);
        s->flaps_not_to = (unsigned char)(phase1211 && s->flaps_not_in_to_pos);
        s->flap_config_aural = (unsigned char)((held && s->flaps_not_to) || (phase345 && s->flaps_not_in_to_pos));
        /* [DEV 7] FBW flapConfigWarning reads slatConfigSr */
        s->flap_config_warning = (unsigned char)((held && s->flaps_not_to) || s->flap_config_sr.value);
        s->slats_not_in_to_pos = s->slats_inf20;
        fwss_mem(&s->slat_config_sr, phase345 && s->slats_not_in_to_pos,
                 !s->slats_not_in_to_pos || phase6 || fp == 7);
        s->slats_not_to = (unsigned char)(phase1211 && s->slats_not_in_to_pos);
        s->slat_config_aural = (unsigned char)((held && s->slats_not_to) || (phase345 && s->slats_not_in_to_pos));
        s->slat_config_warning = (unsigned char)((held && s->slats_not_to) || s->slat_config_sr.value);
        fwss_mem(&s->spdbrk_config_sr, phase345 && in->speedbrake_lever_out,
                 !in->speedbrake_lever_out || phase6 || fp == 7);
        s->spdbrk_not_to = (unsigned char)(phase1211 && in->speedbrake_lever_out);
        s->spdbrk_config_aural = (unsigned char)((held && s->spdbrk_not_to) || (phase345 && in->speedbrake_lever_out));
        s->spdbrk_config_warning = (unsigned char)((held && s->spdbrk_not_to) || s->spdbrk_config_sr.value);
        /* T.O flaps FMS disagree */
        sel1 = in->flaps_handle_index == 1;
        sel2 = in->flaps_handle_index == 2;
        sel3 = in->flaps_handle_index == 3;
        mcdu0 = in->fms_valid && in->fms_to_flaps == 0;
        mcdu1 = in->fms_valid && in->fms_to_flaps == 1;
        mcdu2 = in->fms_valid && in->fms_to_flaps == 2;
        mcdu3 = in->fms_valid && in->fms_to_flaps == 3;
        s->flaps_pitch_mcdu_enable =
            (unsigned char)(!s->phase3_pulse.out && !s->to_config_pulse.out && (phase3 || s->to_config_checked_in_23));
        s->flaps_mcdu_disagree = (unsigned char)(((sel1 != mcdu1) || (sel2 != mcdu2) || (sel3 != mcdu3)) &&
                                                 (mcdu0 || mcdu1 || mcdu2 || mcdu3) && s->flaps_pitch_mcdu_enable);
        /* pitch trim (PitchTrimUtils green band -0.18..5.8) */
        pitch_cfg = in->pitch_trim_valid && (in->pitch_trim_deg > 5.8f || in->pitch_trim_deg < -0.18f);
        s->pitch_trim_not_to = (unsigned char)(phase1211 && pitch_cfg);
        {
            int test1211 = pitch_cfg && held && phase1211;
            int in345 = phase345 && pitch_cfg;
            fwss_mem(&s->pitch_config_sr, in345, phase6 || fp == 7 || !pitch_cfg);
            s->pitch_trim_audio = (unsigned char)(test1211 || in345);
            s->pitch_trim_warning = (unsigned char)(test1211 || s->pitch_config_sr.value);
        }
        {
            /* PitchTrimUtils.cgToPitchTrim / pitchTrimInCyanBand */
            double cg = in->gw_cg_pct, stab = in->pitch_trim_deg, trim_cg;
            int cyan;
            if (cg < 29.0) trim_cg = 5.8;
            else if (cg <= 34.5) trim_cg = 5.8 - ((cg - 29.0) * 56.0) / 275.0;
            else if (cg <= 43.0) trim_cg = 4.68 - ((cg - 34.5) * 243.0) / 425.0;
            else trim_cg = -0.18;
            cyan = fabs(trim_cg - stab) < 1.5 && !(stab > 5.8 || stab < -0.18);
            fwss_conf(&s->trim_disagree_conf,
                      in->fms_valid && in->fms_to_pitch_trim_valid && in->gw_cg_valid && in->pitch_trim_valid &&
                          (!cyan || !(fabs(in->fms_to_pitch_trim_cg - cg) < 1.0)),
                      dt);
            s->pitch_trim_mcdu_disagree = (unsigned char)(!s->pitch_trim_warning &&
                                                          fwss_conf_read(&s->trim_disagree_conf) &&
                                                          s->flaps_pitch_mcdu_enable);
        }
        rud_cfg = in->rudder_trim_valid && fabs(in->rudder_trim_deg) > 3.6;
        s->rudder_trim_not_to = (unsigned char)(phase1211 && rud_cfg);
        {
            int test = held && phase1211 && rud_cfg, in345 = phase345 && rud_cfg;
            fwss_mem(&s->rudder_config_sr, in345, phase6 || !rud_cfg);
            s->rudder_trim_audio = (unsigned char)(test || in345);
            s->rudder_trim_warning = (unsigned char)(test || s->rudder_config_sr.value);
        }
        /* flaps lever not zero (FwsCore:4876-4883), SFCC bit 25 not simulated */
        {
            int alt22 = 0;
            for (i = 0; i < 3; i++)
                if (in->adr_alt_valid[i] && in->adr_alt_ft[i] >= 22000.0f)
                    alt22 = 1;
            s->flaps_lever_not_zero = (unsigned char)(alt22 && phase8 && in->flaps_handle_index != 0);
        }
        /* speed brakes still extended (FwsCore:4885-4943). spdbrk_5s is written twice per cycle as in FBW */
        {
            int cmd = in->speedbrake_lever_out != 0, c1, c2;
            fwss_conf(&s->spdbrk_5s, cmd, dt);
            fwss_conf(&s->spdbrk_5s, cmd, dt);
            fwss_conf(&s->spdbrk_50s, cmd, dt);
            fwss_conf(&s->eng_above_idle_spdbrk, fwss_conf_read(&s->spdbrk_50s) && !one_eng_above_min_power, dt);
            fwss_conf(&s->spdbrk_caution1_conf,
                      phase8 && fwss_conf_read(&s->spdbrk_50s) && !fwss_conf_read(&s->eng_above_idle_spdbrk), dt);
            c1 = fwss_conf_read(&s->spdbrk_caution1_conf);
            c2 = fp == 9 && fwss_conf_read(&s->spdbrk_5s);
            fwss_pulse(&s->spdbrk_caution1_pulse, c1);
            fwss_pulse(&s->spdbrk_caution2_pulse, c2);
            /* caution 3 (AP TCAS RA nose up) and FCDC SPD BRK DO NOT USE are not simulated */
            s->speed_brakes_still_extended =
                (unsigned char)(!s->spdbrk_caution1_pulse.out && !s->spdbrk_caution2_pulse.out && (c1 || c2));
        }
        /* GND SPLRs NOT ARMED (FwsCore:4945-4970) */
        {
            int ra500 = fwss_ra_or(in, 0, 1e9) < 500.0 || fwss_ra_or(in, 1, 1e9) < 500.0 ||
                        fwss_ra_or(in, 2, 1e9) < 500.0;
            int lg_down = s->main_gear_downlocked; /* LGCIU word 1 bit 29 */
            fwss_mtrig(&s->phase10_4s5, fp == 10, dt);
            fwss_conf(&s->gnd_splr_5s, in->gnd_splr_armed, dt);
            fwss_conf(&s->spdbrk_5s_delayed, in->speedbrake_lever_out, dt);
            s->gnd_splr_not_armed =
                (unsigned char)(ra500 && lg_down && (fp == 8 || fp == 9) && !s->phase10_4s5.out &&
                                !s->eng12_to_power && !s->eng34_to_power &&
                                !(fwss_conf_read(&s->gnd_splr_5s) || fwss_conf_read(&s->spdbrk_5s_delayed)));
        }
    }

    /* L/G NOT DOWN (FwsCore:4990-5036) */
    {
        int p4567 = fp >= 4 && fp <= 7;
        double ra_min = fwss_ra_or(in, 0, 1e9);
        int below750, alt_inhibit, gear_not_dl, below750_cond, all_fw, all_fw_ncd, appr_cond, reset_pulse;
        int n1_12, n1_34;
        if (fwss_ra_or(in, 1, 1e9) < ra_min) ra_min = fwss_ra_or(in, 1, 1e9);
        if (fwss_ra_or(in, 2, 1e9) < ra_min) ra_min = fwss_ra_or(in, 2, 1e9);
        below750 = ra_min < 750.0;
        alt_inhibit = s->adr_pressure_alt > 18500.0 && fwss_ra_fw(in, 0) && fwss_ra_fw(in, 1) && fwss_ra_fw(in, 2);
        gear_not_dl = !s->main_gear_downlocked; /* LGCIU faults not simulated */
        below750_cond = s->flaps_conf3_full && !s->eng12_to_power && !s->eng34_to_power && below750 && gear_not_dl;
        all_fw = fwss_ra_fw(in, 0) && fwss_ra_fw(in, 1) && fwss_ra_fw(in, 2);
        all_fw_ncd = (fwss_ra_ncd(in, 0) || fwss_ra_fw(in, 0)) && (fwss_ra_ncd(in, 1) || fwss_ra_fw(in, 1)) &&
                     (fwss_ra_ncd(in, 2) || fwss_ra_fw(in, 2));
        appr_cond = ((s->flaps_sup8 && !s->flaps_sup17 && all_fw) || (s->flaps_sup17 && all_fw_ncd)) && phase8 &&
                    gear_not_dl;
        reset_pulse = fwss_pulse(&s->lg_not_down_pulse1, below750_cond) ||
                      fwss_pulse(&s->lg_not_down_pulse2, appr_cond);
        s->lg_not_down_no_cancel = (unsigned char)((below750_cond || appr_cond) && !reset_pulse);
        n1_12 = (in->n1_pct[0] < 75.0f && in->n1_pct[1] < 75.0f) ||
                (in->n1_pct[0] < 97.0f && in->n1_pct[1] < 97.0f && !in->eng_master_on[0] && !in->eng_master_on[1]);
        n1_34 = (in->n1_pct[2] < 75.0f && in->n1_pct[3] < 75.0f) ||
                (in->n1_pct[2] < 97.0f && in->n1_pct[3] < 97.0f && !in->eng_master_on[2] && !in->eng_master_on[3]);
        s->lg_not_down = (unsigned char)(gear_not_dl && !alt_inhibit && !s->eng12_to_power && !s->eng34_to_power &&
                                         n1_12 && n1_34 && below750);
        s->lg_red_arrow = (unsigned char)(!((phase8 && !all_fw) || p4567) && (s->lg_not_down_no_cancel || s->lg_not_down));
    }

    /* [EXT] L/G GEAR NOT LOCKED DOWN / UP: lever position not reached for 30 s */
    s->gear_not_locked_down = (unsigned char)fwss_conf(
        &s->gear_not_locked_down_conf, in->gear_valid && in->gear_lever_down && !s->all_gear_downlocked, dt);
    s->gear_not_locked_up = (unsigned char)fwss_conf(
        &s->gear_not_locked_up_conf, in->gear_valid && !in->gear_lever_down && !in->all_gear_uplocked, dt);

    /* surveillance (FwsCore:5051-5130) */
    {
        int faulty = in->tcas_fault != 0, stby = in->tcas_mode == 0;
        int adr3_left = in->air_data_knob == 0, adr3_right = in->air_data_knob == 2;
        int ir3_left = in->att_hdg_knob == 0, ir3_right = in->att_hdg_knob == 2;
        int adr_alt_bad3 = !in->adr_alt_valid[2], adr_alt_bad2 = !in->adr_alt_valid[1];
        int all_fw = fwss_ra_fw(in, 0) && fwss_ra_fw(in, 1) && fwss_ra_fw(in, 2);
        int l_adr, l_ir, l_ir_align, r_adr, r_ir, r_ir_align, c1, c2;
        fwss_conf(&s->tcas_stby_conf, !faulty && stby, dt);
        fwss_conf(&s->tcas_stby_memo_conf, stby, dt);
        s->tcas_standby = (unsigned char)(fwss_conf_read(&s->tcas_stby_conf) && phase8);
        s->tcas_standby_memo = (unsigned char)fwss_conf_read(&s->tcas_stby_memo_conf);
        l_adr = (in->adr_fault[0] && !adr3_left) || (in->adr_fault[2] && adr_alt_bad3 && adr3_left);
        l_ir = (s->ir_fault[0] && !ir3_left) || (s->ir_fault[2] && ir3_left);
        l_ir_align = ir3_left ? (s->ir_fault[2] || in->ir_in_align[2]) : (s->ir_fault[0] || in->ir_in_align[0]);
        fwss_conf(&s->tcas1_adr_ir_conf, l_adr || l_ir || l_ir_align, dt);
        fwss_conf(&s->tcas1_fault_conf, faulty && !(phase112 && fwss_conf_read(&s->tcas1_adr_ir_conf)), dt);
        c1 = !all_fw && ac_ess && fwss_conf_read(&s->tcas1_fault_conf);
        r_adr = (in->adr_fault[1] && adr_alt_bad2 && !adr3_right) || (in->adr_fault[2] && adr_alt_bad3 && adr3_right);
        r_ir = (s->ir_fault[1] && !ir3_right) || (s->ir_fault[2] && ir3_right);
        r_ir_align = ir3_right ? (s->ir_fault[2] || in->ir_in_align[2]) : (s->ir_fault[1] || in->ir_in_align[1]);
        fwss_conf(&s->tcas2_adr_ir_conf, r_adr || r_ir || r_ir_align, dt);
        fwss_conf(&s->tcas2_fault_conf, faulty && !(phase112 && fwss_conf_read(&s->tcas2_adr_ir_conf)), dt);
        c2 = !all_fw && ac2 && fwss_conf_read(&s->tcas2_fault_conf);
        s->tcas1_fault = (unsigned char)(c1 && !c2);
        s->tcas2_fault = (unsigned char)(c2 && !c1);
        s->tcas12_fault = (unsigned char)(c1 && c2);
    }

    /* stall warning (FwsCore:5134-5145) */
    {
        int cas60 = 0;
        for (i = 0; i < 3; i++)
            if (in->adr_cas_valid[i] && in->adr_cas_kt[i] > 60.0f)
                cas60 = 1;
        s->stall_warning = (unsigned char)(!in->normal_law && cas60 && in->stall_warning_raw && fp >= 6 && fp <= 9 &&
                                           fwss_ra_or(in, 0, 1e9) > 1500.0 && fwss_ra_or(in, 1, 1e9) > 1500.0 &&
                                           fwss_ra_or(in, 2, 1e9) > 1500.0);
    }

    /* fire timers (FwsCore:5226-5286). Date.now() stamps -> clock_s at the rising edge */
    {
        int apu_clock = in->apu_fire_detected && in->apu_fire_pb_released;
        if (apu_clock && !s->apu_fire_clock10)
            s->apu_fire_ts10 = s->clock_s;
        s->apu_fire_clock10 = (unsigned char)apu_clock;
        for (i = 0; i < 4; i++) {
            int c10 = !s->aircraft_on_ground && !in->eng_master_on[i] && in->eng_fire_detected[i] &&
                      !in->eng_agent_discharged[i][0];
            int c30 = !s->aircraft_on_ground && in->eng_fire_pb_released[i] && in->eng_fire_detected[i] &&
                      in->eng_agent_discharged[i][0];
            if (c10 && !s->fire_clock10[i])
                s->fire_ts10[i] = s->clock_s;
            if (c30 && !s->fire_clock30[i])
                s->fire_ts30[i] = s->clock_s;
            s->fire_clock10[i] = (unsigned char)c10;
            s->fire_clock30[i] = (unsigned char)c30;
        }
    }

    /* ice (FwsCore:5294-5326) */
    {
        double ice = in->ice_valid ? in->ice_ratio : 0.0, tat = in->ice_valid ? in->tat_c : 15.0;
        int in_cloud = in->ice_valid && in->in_cloud;
        int all_eng_ai = in->eng_anti_ice_on[0] && in->eng_anti_ice_on[1] && in->eng_anti_ice_on[2] &&
                         in->eng_anti_ice_on[3];
        int any_ai = in->eng_anti_ice_on[0] || in->eng_anti_ice_on[1] || in->eng_anti_ice_on[2] ||
                     in->eng_anti_ice_on[3] || in->wing_anti_ice_on;
        int det1 = fwss_conf(&s->ice_det1, ice >= 0.1 && tat < 10.0 && !s->aircraft_on_ground, dt);
        int nd1;
        s->ice_detected = (unsigned char)fwss_conf(&s->ice_det2, det1 && !all_eng_ai, dt);
        fwss_conf(&s->ice_severe, ice >= 0.5 && tat < 10.0 && !s->aircraft_on_ground, dt);
        nd1 = fwss_conf(&s->ice_not_det1, any_ai, dt);
        s->ice_not_detected = (unsigned char)fwss_conf(&s->ice_not_det2, nd1 && !(ice >= 0.1 || (tat < 10.0 && in_cloud)), dt);
    }

    /* engines: FAIL / SHUTDOWN / OUT (FwsCore:5345-5470) */
    {
        int pre = !fwss_phase12_or_1112(fp), all_fail, n_out = 0;
        for (i = 0; i < 4; i++) {
            int st = in->eng_valid ? in->eng_state[i] : FWS_ENG_OFF;
            int master = in->eng_master_on[i] != 0;
            double n3 = in->eng_valid ? in->n3_pct[i] : 0.0;
            /* [DEV 8] FBW engines 3/4 use `master && !STARTING`; engines 1/2 `running && !STARTING` is used for all */
            fwss_conf(&s->eng_not_starting_conf[i], st != FWS_ENG_STARTING && s->eng_running[i], dt);
            fwss_mem(&s->eng_was_running_mem[i], fwss_conf_read(&s->eng_not_starting_conf[i]),
                     fwss_pulse(&s->eng_master_on_pulse[i], master));
            s->eng_fail[i] = (unsigned char)(!s->all_engines_failure &&
                                             fwss_mem(&s->eng_fail_mem[i],
                                                      master && s->eng_was_running_mem[i].value &&
                                                          !in->eng_fire_pb_released[i] && n3 < 50.0,
                                                      st == FWS_ENG_ON || (n3 > 50.0 && st == FWS_ENG_STARTING)));
            s->eng_start_or_crank[i] =
                (unsigned char)(in->eng_valid && in->n1_pct[i] < in->n1_idle_pct - 1.0f && st == FWS_ENG_STARTING);
            s->eng_primary_abnormal[i] = (unsigned char)(!s->eng_start_or_crank[i] && master && s->eng_fail[i]);
            s->eng_shutdown[i] = (unsigned char)(pre && (in->eng_fire_pb_released[i] || !master));
            s->eng_out[i] = (unsigned char)(s->eng_shutdown[i] || s->eng_fail_mem[i].value);
            n_out += s->eng_out[i];
        }
        all_fail = !s->aircraft_on_ground && eng_not_running && n_out == 4;
        s->all_engines_failure = (unsigned char)all_fail;
    }

    /* T.O CONFIG check (FwsCore:5486-5510), evaluated while T.O CONFIG is pressed with the T.O memo */
    if (s->to_memo && in->to_config_pb) {
        int sys = !in->elec_valid || (in->gen_potential_normal[0] && in->gen_potential_normal[1]);
        int speeds = !(in->fms_valid && in->fms_to_speeds_too_low) && !(in->fms_valid && in->fms_v_disagree) &&
                     !(in->fms_valid && in->fms_to_speeds_not_inserted);
        int doors = !in->main_door_open[0] && !in->main_door_open[3] && in->cargo_aft_locked && in->cargo_fwd_locked;
        int surfaces = s->flaps_not_in_to_pos || s->slats_not_in_to_pos || s->spdbrk_not_to || s->rudder_trim_not_to ||
                       s->pitch_trim_not_to;
        int normal = sys && speeds && !in->brakes_hot && doors && !s->flaps_mcdu_disagree && !surfaces;
        s->to_config_normal = (unsigned char)fwss_conf(&s->to_config_normal_conf, normal, dt);
    }
}

/* ============================================================================================== */
/* SD page when unselected (FwsSystemDisplayLogic.ts:117-260, ECP-driven parts excluded)         */
/* ============================================================================================== */
static void fwss_sd_check_engine(FwsSignals *s, double dt)
{
    const FwsInputs *in = &s->in;
    int sel = in->eng_mode_selector, one_off = 0, i;
    for (i = 0; i < 4; i++)
        if (!in->eng_valid || in->eng_state[i] != FWS_ENG_ON)
            one_off = 1;
    if (sel == 0 || (one_off && sel == 2) || s->sd_eng_timer >= 0.0) {
        if (sel == 0 || (one_off && sel == 2))
            s->sd_eng_timer = 10.0;
        else if (s->sd_eng_timer >= 0.0)
            s->sd_eng_timer -= dt;
        s->sd_page_when_unselected = FWS_SD_ENG;
    }
}

static void fwss_sd_check_apu(FwsSignals *s, double dt)
{
    const FwsInputs *in = &s->in;
    double n = in->apu_n_pct;
    if (in->apu_master_on && in->apu_n_valid && (n <= 95.0 || s->sd_apu_timer >= 0.0)) {
        if (s->sd_apu_timer <= 0.0 && n <= 95.0)
            s->sd_apu_timer = 10.0;
        else if (n > 95.0)
            s->sd_apu_timer -= dt;
        s->sd_page_when_unselected = FWS_SD_APU;
    }
}

static void fwss_update_sd(FwsSignals *s, double dt)
{
    const FwsInputs *in = &s->in;
    switch (s->flight_phase) {
    case 12:
    case 1:
        s->sd_crz_timer = 60.0;
        s->sd_page_when_unselected = FWS_SD_DOOR;
        fwss_sd_check_apu(s, dt);
        fwss_sd_check_engine(s, dt);
        break;
    case 2:
        s->sd_page_when_unselected = FWS_SD_WHEEL;
        if (in->controls_moved) {
            s->sd_page_when_unselected = FWS_SD_FCTL;
            s->sd_fctl_timer = 20.0;
        } else if (s->sd_fctl_timer >= 0.0) {
            s->sd_page_when_unselected = FWS_SD_FCTL;
            s->sd_fctl_timer -= dt;
        }
        fwss_sd_check_apu(s, dt);
        fwss_sd_check_engine(s, dt);
        break;
    case 3: case 4: case 5: case 6: case 7:
        s->sd_page_when_unselected = FWS_SD_ENG;
        break;
    case 8: case 9: case 10: case 11: {
        int gear_ext = in->gear_extended_ratio * 100.0f > 0.95f; /* 'percent' > 0.95 as in FBW */
        double tla_max = 0.0;
        int all_n1 = in->eng_valid, i;
        int splr_flaps;
        for (i = 0; i < 4; i++) {
            /* FBW reads TLA 1, 2, 3, 3 */
            double t = in->tla_valid ? in->tla_deg[i == 3 ? 2 : i] : 0.0;
            if (t > tla_max)
                tla_max = t;
            if (!(in->n1_pct[i] > 15.0f))
                all_n1 = 0;
        }
        splr_flaps = in->flaps_handle_index != 0 || in->speedbrake_lever_out;
        if (gear_ext && s->adr_pressure_alt < 16000.0) {
            s->sd_page_when_unselected = FWS_SD_WHEEL;
            fwss_sd_check_apu(s, dt);
            fwss_sd_check_engine(s, dt);
            break;
        }
        if (splr_flaps || (tla_max >= 35.0 && all_n1)) {
            if (s->sd_crz_timer <= 0.0) {
                s->sd_page_when_unselected = FWS_SD_CRZ;
                fwss_sd_check_apu(s, dt);
                fwss_sd_check_engine(s, dt);
            } else {
                s->sd_crz_timer -= dt;
            }
        } else {
            s->sd_page_when_unselected = FWS_SD_CRZ;
            fwss_sd_check_apu(s, dt);
            fwss_sd_check_engine(s, dt);
        }
        break;
    }
    default:
        s->sd_page_when_unselected = FWS_SD_DOOR;
        break;
    }
    if (in->sd_fail_page != FWS_SD_NONE)
        s->sd_page_when_unselected = in->sd_fail_page;
}

/* ============================================================================================== */
/* Alert evaluation (FwsAbnormalSensed.ts ewdAbnormalSensed / ewdDeferredProcs)                  */
/* ============================================================================================== */

static void fwss_vec(unsigned char *dst, unsigned char *n_out, const unsigned char *src, size_t n)
{
    size_t i;
    if (n > FWS_MAX_ITEMS)
        n = FWS_MAX_ITEMS;
    for (i = 0; i < FWS_MAX_ITEMS; i++)
        dst[i] = i < n ? src[i] : 0;
    if (n_out)
        *n_out = (unsigned char)n;
}

#define SH(...) do { const unsigned char v_[] = {__VA_ARGS__}; fwss_vec(e->items_to_show, NULL, v_, sizeof v_); } while (0)
#define CK(...) do { const unsigned char v_[] = {__VA_ARGS__}; fwss_vec(e->items_checked, &e->n_items, v_, sizeof v_); } while (0)
#define ACTIVE(x) (e->active = (unsigned char)((x) != 0))

/* all items shown, none checked (port extensions) */
static void fwss_items_all(FwsAlertEval *e, int n)
{
    int i;
    e->n_items = (unsigned char)n;
    for (i = 0; i < FWS_MAX_ITEMS; i++) {
        e->items_to_show[i] = (unsigned char)(i < n);
        e->items_checked[i] = 0;
    }
}

static void fwss_timer(FwsAlertEval *e, int idx, int valid, double ts)
{
    e->has_items_timer = 1;
    e->timer_valid[idx] = (unsigned char)(valid != 0);
    e->items_timer_s[idx] = valid ? ts : 0.0;
}

static void fwss_eval(const FwsSignals *s, int index, FwsAlertEval *e)
{
    const FwsInputs *in = &s->in;
    const unsigned id = fwss_alerts[index].id;
    const int fp = s->flight_phase, gnd = s->aircraft_on_ground, fl = s->flight_level;
    const int elec = in->elec_valid;
    const int any_ac = in->ac_powered[0] || in->ac_powered[1] || in->ac_powered[2] || in->ac_powered[3];
    int n, k;

    memset(e, 0, sizeof *e);
    switch (id) {
    /* ---------------- ATA 21 ---------------- */
    case 211800011u: ACTIVE(s->pack_off[0]); break;
    case 211800012u: ACTIVE(s->pack_off[1]); break;
    case 211800021u: {
        int up = !gnd && fl > 100, lo = !gnd && fl <= 100;
        ACTIVE(s->pack12_fault);
        SH(0, 0, 1, 1, 0, 0, 0, up, up, up, up, up, lo, lo, lo);
        CK(0, 0, !in->pack_pb_on[0], !in->pack_pb_on[1], 0, 0, 0, 0, 0, 0, in->ram_air_on, in->cabin_air_extract_on, 0,
           in->ram_air_on, in->cabin_air_extract_on);
        break;
    }
    case 211800051u:
        ACTIVE(s->apu_bleed_outside_envelope);
        SH(0, 0, 0, 0, 1, s->adr_pressure_alt > 22500.0);
        CK(0, 0, 0, 0, !in->apu_bleed_pb_on, 0);
        break;
    case 211800052u: case 211800053u: case 211800054u: case 211800055u:
        ACTIVE(s->eng_bleed_abnormal_off[id - 211800052u]);
        break;
    case 213800001u: {
        int a = fl > 100, b = fl > 100 && fl < 160, c = fl > 160;
        int ofv_open = 0; /* allOutflowValvesOpen: outflow valves not simulated */
        ACTIVE(s->excess_cab_alt);
        SH(a, b, b, c, c, c, c, c, c, c, c, c, 1, 1, 1, a, a, 1, 1, 1, 1, 1, 1);
        CK(0, 0, 0, 0, 0, 0, s->tla_idle_all_cache, in->speedbrake_lever_out, 0, 0, 0, 0, 0, 0,
           in->pax_oxy_masks_deployed, 0, 0, 0, in->ram_air_on, 0,
           in->cabin_air_extract_on || (!in->cabin_air_extract_on && ofv_open), 0,
           !in->cabin_air_extract_on && ofv_open);
        break;
    }
    case 213800002u:
        ACTIVE(s->excess_diff_press);
        SH(1, 1, fl > 100, 1, 1, 1, 1);
        CK(!in->pack_pb_on[0], !in->pack_pb_on[1], 0, 0, 0, in->ram_air_on, in->cabin_air_extract_on);
        break;
    case 213800004u:
        ACTIVE(s->excess_residual_diff_press);
        SH(1, 1, 1, 1, 1);
        CK(!in->pack_pb_on[0], !in->pack_pb_on[1], 0, 0, 0);
        break;
    case 213800009u: ACTIVE(s->inhibited_by_doors); SH(1); CK(1); break;
    /* ---------------- ATA 22 ---------------- */
    case 220000002u: ACTIVE(s->vol_athr_off_caution); break;
    case 220800001u:
        ACTIVE(s->ap_off_involuntary);
        e->aural = s->ap_off_involuntary ? FWS_AURAL_CAVALRY_CHARGE : FWS_AURAL_NONE;
        break;
    case 220800003u:
        ACTIVE(s->athr_limited);
        SH(s->athr_limited_clb, s->athr_limited_mct);
        CK(0, 0);
        break;
    case 220800004u:
        ACTIVE(s->athr_off_invol_warning);
        e->aural = s->athr_off_invol_caution ? FWS_AURAL_SINGLE_CHIME : FWS_AURAL_NONE;
        SH(1);
        CK(!s->eng_thrust_locked);
        break;
    case 221800007u: ACTIVE(s->to_speeds_not_inserted); break;
    case 221800008u: ACTIVE(s->to_speeds_too_low); SH(1); CK(0); break;
    case 221800009u: ACTIVE(s->to_v_disagree); break;
    /* ---------------- ATA 24 [EXT] ---------------- */
    case 240800002u: case 240800004u: case 240800007u: case 240800009u: {
        int b = id == 240800002u ? 0 : id == 240800004u ? 1 : id == 240800007u ? 2 : 3;
        ACTIVE(elec && !in->ac_powered[b] && any_ac);
        fwss_items_all(e, fwss_alerts[index].n_items);
        if (b == 1)
            e->items_checked[1] = !in->comm_on[0];
        if (b == 2) {
            e->items_checked[2] = !in->comm_on[1];
            for (k = 0; k < 4; k++)
                e->items_checked[5 + k] = !in->crossfeed_open[k];
        }
        if (b == 3)
            e->items_checked[2] = in->air_data_knob == 2;
        break;
    }
    case 240800010u: ACTIVE(elec && !in->ac_emer_powered && any_ac); fwss_items_all(e, 4); break;
    case 240800011u: ACTIVE(elec && !in->ac_ess_feed_normal && in->ac_ess_powered); break;
    case 240800012u:
        ACTIVE(elec && !in->ac_ess_powered && any_ac);
        fwss_items_all(e, 11);
        e->items_checked[0] = !in->ac_ess_feed_normal;
        break;
    case 240800013u: ACTIVE(elec && in->bat_fault[3]); fwss_items_all(e, 1); e->items_checked[0] = !in->bat_pb_auto[3]; break;
    case 240800014u: case 240800015u: {
        int g = (int)(id - 240800014u);
        ACTIVE(elec && in->apu_gen_fault[g] && in->apu_avail);
        fwss_items_all(e, 1);
        e->items_checked[0] = !in->apu_gen_pb_on[g];
        break;
    }
    case 240800016u: ACTIVE(elec && in->tr_fault[3] && any_ac); fwss_items_all(e, 2); e->items_checked[1] = !in->bat_pb_auto[3]; break;
    case 240800017u: case 240800018u: {
        int b = (int)(id - 240800017u);
        ACTIVE(elec && in->bat_fault[b]);
        fwss_items_all(e, 1);
        e->items_checked[0] = !in->bat_pb_auto[b];
        break;
    }
    case 240800019u: ACTIVE(elec && !in->bus_tie_auto && any_ac); break;
    case 240800026u: ACTIVE(elec && !in->dc1_powered && any_ac); fwss_items_all(e, 11); break;
    case 240800027u: ACTIVE(elec && !in->dc1_powered && !in->dc2_powered && any_ac); fwss_items_all(e, 22); break;
    case 240800028u: {
        int all_xf = in->crossfeed_open[0] && in->crossfeed_open[1] && in->crossfeed_open[2] && in->crossfeed_open[3];
        ACTIVE(elec && !in->dc1_powered && !in->dc_ess_powered && any_ac);
        fwss_items_all(e, 21);
        e->items_checked[0] = (unsigned char)all_xf;
        break;
    }
    case 240800029u: ACTIVE(elec && !in->dc2_powered && any_ac); fwss_items_all(e, 12); e->items_checked[1] = in->air_data_knob == 2; break;
    case 240800030u: ACTIVE(elec && !in->dc_ess_powered && any_ac); fwss_items_all(e, 20); e->items_checked[1] = in->air_data_knob == 0; break;
    case 240800036u: case 240800037u: case 240800038u: case 240800039u: {
        int g = (int)(id - 240800036u);
        ACTIVE(elec && in->drive_disc[g]);
        fwss_items_all(e, 1);
        e->items_checked[0] = !in->gen_pb_on[g];
        break;
    }
    case 240800055u:
        ACTIVE(elec && !any_ac && !gnd);
        fwss_items_all(e, 30);
        /* 10-12 above FL 200, 24 at or below, 21-23 (repeated branch) hidden: port choice */
        for (k = 10; k <= 12; k++)
            e->items_to_show[k] = fl > 200;
        for (k = 21; k <= 23; k++)
            e->items_to_show[k] = 0;
        e->items_to_show[24] = fl <= 200;
        e->items_checked[0] = in->rat_stow_ratio > 0.0f || in->rat_man_on;
        e->items_checked[4] = !in->bus_tie_auto;
        e->items_checked[7] = !s->athr_engaged_or_armed;
        break;
    case 240800056u: case 240800057u: case 240800058u: case 240800059u: {
        int x = (int)(id - 240800056u);
        ACTIVE(elec && gnd && in->ext_connected && in->ext_pb_on[x] && !in->ext_avail[x]);
        fwss_items_all(e, 1);
        break;
    }
    case 240800061u: case 240800062u: case 240800063u: case 240800064u: {
        int g = (int)(id - 240800061u);
        ACTIVE(elec && in->gen_fault[g] && !in->drive_disc[g] && s->eng_running[g]);
        fwss_items_all(e, 1);
        e->items_checked[0] = !in->gen_pb_on[g];
        break;
    }
    case 240800065u: case 240800066u: case 240800067u: case 240800068u: {
        int g = (int)(id - 240800065u);
        ACTIVE(elec && !in->gen_pb_on[g] && !in->gen_fault[g] && !in->drive_disc[g] && s->eng_running[g]);
        break;
    }
    case 240800072u: ACTIVE(elec && in->rat_fault); break;
    case 240800080u: ACTIVE(elec && in->stat_inv_fault); break;
    case 240800081u: ACTIVE(elec && in->tr_fault[0] && any_ac); break;
    case 240800082u: ACTIVE(elec && in->tr_fault[1] && any_ac); break;
    case 240800083u: ACTIVE(elec && in->tr_fault[2] && any_ac); break;
    /* ---------------- ATA 26 ---------------- */
    case 260800001u:
        ACTIVE(in->apu_fire_detected);
        e->aural = (in->apu_fire_detected && !in->apu_fire_pb_released) ? FWS_AURAL_CRC : FWS_AURAL_NONE;
        SH(!in->fire_test_pb, !in->fire_test_pb, !in->fire_test_pb);
        CK(in->apu_fire_pb_released, in->apu_agent_discharged, !in->apu_master_on);
        fwss_timer(e, 0, 0, 0.0);
        fwss_timer(e, 1, s->apu_fire_clock10, s->apu_fire_ts10);
        fwss_timer(e, 2, 0, 0.0);
        break;
    case 260800005u: case 260800006u: case 260800007u: case 260800008u: {
        int g = (int)(id - 260800005u);
        int pb = in->eng_fire_pb_released[g];
        ACTIVE(!gnd && in->eng_fire_detected[g]);
        e->aural = (in->eng_fire_detected[g] && !pb) ? FWS_AURAL_CRC : FWS_AURAL_NONE;
        if (g == 0) {
            int idle = !in->tla_valid || in->tla_deg[0] <= 2.6f;
            SH(1, 1, 1, in->apu_avail, pb, 1, 1, 1, 1);
            CK(idle && !pb, !in->eng_master_on[0], pb, !in->apu_bleed_valve_open, 0, in->eng_agent_discharged[0][0], 0,
               0, in->eng_agent_discharged[0][1]);
            for (k = 0; k < 8; k++)
                fwss_timer(e, k, 0, 0.0);
            fwss_timer(e, 5, s->fire_clock10[0], s->fire_ts10[0]);
            fwss_timer(e, 7, s->fire_clock30[0], s->fire_ts30[0]);
        } else {
            int idle = !in->tla_valid || in->tla_deg[g] <= 2.6f; /* [DEV 4] */
            SH(1, 1, 1, 1, 1, 1, 1);
            CK(idle && !pb, !in->eng_master_on[g], pb, in->eng_agent_discharged[g][0], 0, 0,
               in->eng_agent_discharged[g][1]);
            for (k = 0; k < 6; k++)
                fwss_timer(e, k, 0, 0.0);
            /* [DEV 9] FBW ENG 2 FIRE reads the ENG 1 agent 1 timestamp */
            fwss_timer(e, 3, s->fire_clock10[g], s->fire_ts10[g]);
            fwss_timer(e, 5, s->fire_clock30[g], s->fire_ts30[g]);
        }
        break;
    }
    case 260800009u: case 260800010u: case 260800011u: case 260800012u: {
        int g = (int)(id - 260800009u);
        int pb = in->eng_fire_pb_released[g], t = !in->fire_test_pb;
        int all_pb = in->eng_fire_pb_released[0] && in->eng_fire_pb_released[1] && in->eng_fire_pb_released[2] &&
                     in->eng_fire_pb_released[3] && in->apu_fire_pb_released;
        int all_bat_off = !(in->bat_pb_auto[0] || in->bat_pb_auto[1] || in->bat_pb_auto[2] || in->bat_pb_auto[3]);
        int all_masters_off = !(in->eng_master_on[0] || in->eng_master_on[1] || in->eng_master_on[2] ||
                                in->eng_master_on[3]);
        ACTIVE(gnd && in->eng_fire_detected[g]);
        e->aural = (in->eng_fire_detected[g] && !pb) ? FWS_AURAL_CRC : FWS_AURAL_NONE;
        SH(t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t);
        CK(s->tla_idle_all_cache && !pb, in->park_brake_set, in->park_brake_set, 0, 0, !in->eng_master_on[g], pb,
           in->eng_agent_discharged[g][0] && in->eng_agent_discharged[g][1], all_masters_off, all_pb, 0, 0,
           !in->evac_command, all_bat_off, 0, 0);
        break;
    }
    /* ---------------- ATA 27 ---------------- */
    case 271800003u:
        ACTIVE(s->pitch_trim_warning);
        e->aural = s->pitch_trim_audio ? FWS_AURAL_CRC : FWS_AURAL_NONE;
        break;
    case 271800004u:
        ACTIVE(s->rudder_trim_warning);
        e->aural = s->rudder_trim_audio ? FWS_AURAL_CRC : FWS_AURAL_NONE;
        break;
    case 271800005u:
        ACTIVE(s->spdbrk_config_warning);
        e->aural = s->spdbrk_config_aural ? FWS_AURAL_CRC : FWS_AURAL_NONE;
        break;
    case 271800017u: ACTIVE(s->gnd_splr_not_armed); break;
    case 271800032u: ACTIVE(s->pitch_trim_mcdu_disagree); break;
    case 271800065u: ACTIVE(s->speed_brakes_still_extended); break;
    case 271800069u:
        ACTIVE(s->overspeed_vmo || s->overspeed_vle || s->overspeed_vfe[0] || s->overspeed_vfe[1] ||
               s->overspeed_vfe[2] || s->overspeed_vfe[3] || s->overspeed_vfe[4]);
        SH(s->overspeed_vmo, s->overspeed_vle, s->overspeed_vfe[0], s->overspeed_vfe[1], s->overspeed_vfe[2],
           s->overspeed_vfe[3], s->overspeed_vfe[4]);
        CK(0, 0, 0, 0, 0, 0, 0);
        break;
    case 271800070u: ACTIVE(s->load_analysis_required); break;
    case 272800001u:
        ACTIVE(s->slat_config_warning);
        e->aural = s->slat_config_aural ? FWS_AURAL_CRC : FWS_AURAL_NONE;
        break;
    case 272800002u:
        ACTIVE(s->flap_config_warning);
        e->aural = s->flap_config_aural ? FWS_AURAL_CRC : FWS_AURAL_NONE;
        break;
    case 272800003u: ACTIVE(s->flaps_lever_not_zero); break;
    case 272800006u: ACTIVE(in->flap_sys_fault[0]); SH(1); CK(0); break;
    case 272800007u: ACTIVE(in->flap_sys_fault[1]); SH(1); CK(0); break;
    case 272800008u: {
        int jam = in->flaps_jammed, retracted = in->flap_slat_valid && in->flap_angle_deg <= 0.5f;
        ACTIVE(in->flap_sys_fault[0] && in->flap_sys_fault[1]);
        SH(1, jam, jam, 1, in->flaps_handle_index < 3, !in->flap_slat_valid, !retracted, !retracted, 1, 1, 1, 1);
        CK(0, 0, 0, 0, 0 /* tawsFlapModeOff */, 0 /* tawsGpwsOff */, 0, 0, 0, 0, 0, 0);
        break;
    }
    case 272800021u: ACTIVE(in->slat_sys_fault[0]); SH(1); CK(0); break;
    case 272800022u: ACTIVE(in->slat_sys_fault[1]); SH(1); CK(0); break;
    case 272800023u: {
        int jam = in->slats_jammed, retracted = in->flap_slat_valid && in->slat_angle_deg <= 0.5f;
        int emer = in->elec_valid && !any_ac;
        ACTIVE(in->slat_sys_fault[0] && in->slat_sys_fault[1]);
        SH(emer, 1, jam, jam, 1, !retracted, !retracted, 1, 1, 1, 1);
        CK(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        break;
    }
    case 272800028u: ACTIVE(s->flaps_mcdu_disagree); break;
    /* ---------------- ATA 28 ---------------- */
    case 281800002u:
        ACTIVE(s->feed_tank_low[0] && s->feed_tank_low[1] && s->feed_tank_low[2] && s->feed_tank_low[3]);
        SH(1, 1, 0, 0, 0, 0, 0);
        CK(s->all_crossfeed_open, s->all_feed_pumps_on, 0, 0, 0, 0, 0);
        break;
    case 281800023u:
        ACTIVE(s->feed_tank_low[0]);
        SH(1, 1, 1, 0, 0, 0, 0, 0);
        CK(0, 0, 0, 0, 0, 0, 0, 0);
        break;
    case 281800024u:
        ACTIVE(s->feed_tank_low[1]);
        SH(1, 1, 1, 0, 0, 0, 0, 0);
        CK(0, in->crossfeed_open[0], in->crossfeed_open[1], 0, 0, 0, 0, 0);
        break;
    case 281800025u: case 281800026u:
        ACTIVE(s->feed_tank_low[id - 281800023u]);
        SH(1, 1, 1, 0, 0, 0, 0, 0);
        CK(0, in->crossfeed_open[2], in->crossfeed_open[3], 0, 0, 0, 0, 0);
        break;
    case 281800027u: case 281800028u: case 281800029u: case 281800030u: {
        int t = (int)(id - 281800027u);
        ACTIVE(in->fuel_valid && in->feed_pump_main_fault[t] && in->feed_pump_stby_fault[t]);
        break;
    }
    case 281800031u: case 281800032u: case 281800033u: case 281800034u: {
        int t = (int)(id - 281800031u);
        ACTIVE(in->fuel_valid && in->feed_pump_main_fault[t] && in->feed_pump_main_on[t]);
        break;
    }
    case 281800035u: case 281800036u: case 281800037u: case 281800038u: {
        int t = (int)(id - 281800035u);
        ACTIVE(in->fuel_valid && in->feed_pump_stby_valid && in->feed_pump_stby_fault[t] && in->feed_pump_stby_on[t]);
        break;
    }
    case 281800076u: ACTIVE(in->fms_valid && !in->fms_zfw_zfwcg_set); SH(1); CK(0); break;
    case 281800102u:
        ACTIVE(s->feed_tank_low[0] && s->feed_tank_low[1]);
        SH(1, 1, 0, 0, 0, 0, 0);
        CK(0, s->all_crossfeed_open, 0, 0, 0, 0, 0);
        break;
    case 281800103u:
        ACTIVE(s->feed_tank_low[2] && s->feed_tank_low[3]);
        SH(1, 1, 0, 0, 0, 0, 0);
        CK(0, s->all_crossfeed_open, 0, 0, 0, 0, 0);
        break;
    /* ---------------- ATA 29 ---------------- */
    case 290800001u: case 290800002u: case 290800003u: case 290800004u: {
        int p = (int)(id - 290800001u);
        ACTIVE(in->hyd_valid && in->elec_pump_fault[p]);
        SH(1);
        CK(in->elec_pump_off[p]);
        break;
    }
    case 290800019u: case 290800020u: {
        int y = id == 290800020u, e1 = y ? 2 : 0, e2 = y ? 3 : 1;
        ACTIVE(in->hyd_valid && (y ? in->yellow_rsv_air_low : in->green_rsv_air_low));
        SH(1, s->eng_running[e1], s->eng_running[e2], fwss_conf_read(&s->engines_off_on_ground_conf));
        CK(0, !in->eng_pump_pb_auto[e1][0] && !in->eng_pump_pb_auto[e1][1],
           !in->eng_pump_pb_auto[e2][0] && !in->eng_pump_pb_auto[e2][1],
           in->elec_pump_off[2 * y] && in->elec_pump_off[2 * y + 1]);
        break;
    }
    case 290800031u: case 290800032u: {
        int y = id == 290800032u, e1 = y ? 2 : 0, e2 = y ? 3 : 1;
        ACTIVE(in->hyd_valid && (y ? in->yellow_rsv_ovht : in->green_rsv_ovht));
        SH(s->eng_running[e1], s->eng_running[e2], 1, 1, fwss_conf_read(&s->engines_off_on_ground_conf));
        CK(!in->eng_pump_pb_auto[e1][0] && !in->eng_pump_pb_auto[e1][1],
           !in->eng_pump_pb_auto[e2][0] && !in->eng_pump_pb_auto[e2][1], in->eng_pump_disc[e1], in->eng_pump_disc[e2],
           in->elec_pump_off[2 * y] && in->elec_pump_off[2 * y + 1]);
        break;
    }
    case 290800005u: case 290800006u: case 290800007u: case 290800008u:
    case 290800009u: case 290800010u: case 290800011u: case 290800012u: {
        int p = (int)(id - 290800005u), g = p / 2, ab = p % 2;
        ACTIVE(s->eng_pump_fault[g][ab]);
        SH(1, gnd && s->three_yellow_pumps_failed);
        CK(!in->eng_pump_pb_auto[g][ab], 0);
        break;
    }
    case 290800021u: case 290800022u: {
        int y = id == 290800022u, e1 = y ? 2 : 0, e2 = y ? 3 : 1;
        ACTIVE(in->hyd_valid && (y ? in->yellow_rsv_level_low : in->green_rsv_level_low));
        SH(s->eng_running[e1], s->eng_running[e2], 1, 1, fwss_conf_read(&s->engines_off_on_ground_conf));
        /* [DEV] FBW checks the YELLOW elec pumps in both procedures; the port checks the system's own */
        CK(!in->eng_pump_pb_auto[e1][0] && !in->eng_pump_pb_auto[e1][1],
           !in->eng_pump_pb_auto[e2][0] && !in->eng_pump_pb_auto[e2][1], in->eng_pump_disc[e1], in->eng_pump_disc[e2],
           in->elec_pump_off[2 * y] && in->elec_pump_off[2 * y + 1]);
        break;
    }
    case 290800035u:
        ACTIVE(s->green_lo);
        SH(s->phase1112_more_than_1min, 1, 1, 1, 0, 0, 1, 1); /* !this.fws.prim3Healthy (object) is false */
        CK(0, 0, 0, 0, 0, 0, 0, 0);
        break;
    case 290800036u:
        ACTIVE(s->yellow_lo);
        SH(s->phase1112_more_than_1min, 1, 0, 0, 1);
        CK(0, 0, 0, 0, 0);
        break;
    case 290800039u: {
        int all_off = 1;
        for (k = 0; k < 4; k++)
            if (in->eng_pump_pb_auto[k][0] || in->eng_pump_pb_auto[k][1])
                all_off = 0;
        ACTIVE(s->green_yellow_lo);
        SH(1, in->flaps_handle_index <= 3, 1, 1, 1, 1, 1, 1, 1, 1, 1);
        CK(all_off, 0 /* tawsFlapModeOff */, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        break;
    }
    /* ---------------- ATA 32 ---------------- */
    case 320000001u: ACTIVE(s->autobrake_off_mc); e->aural = FWS_AURAL_NONE; break;
    case 320800001u: ACTIVE(in->brakes_valid && in->antiskid_on && in->antiskid_fault); fwss_items_all(e, 2); break;
    case 320800008u: ACTIVE(!in->antiskid_on); SH(1, 1); CK(0, 0); break;
    case 320800013u: ACTIVE(in->brakes_valid && in->autobrake_fault); break;
    case 320800020u: ACTIVE(in->brakes_valid && in->norm_brake_fault); fwss_items_all(e, 1); break;
    case 320800018u:
        ACTIVE(s->brakes_hot);
        SH(fp == 1 || fp == 12, gnd, !gnd, !gnd, !gnd);
        CK(0, 0, 0, 0, 0);
        break;
    case 320800022u: ACTIVE(s->lg_park_brk_on); SH(1); CK(!in->park_brake_set); break;
    case 320800030u: ACTIVE(s->config_park_brk); break;
    case 320800037u: ACTIVE(s->lg_not_down_no_cancel); break;
    case 320800038u: ACTIVE(s->lg_not_down && !s->lg_not_down_no_cancel); break;
    case 320800039u:
        ACTIVE(s->gear_not_locked_down);
        fwss_items_all(e, 6);
        e->items_checked[2] = !in->gear_lever_down;
        e->items_checked[3] = in->lg_grvty_switch_pos == 2; /* L/G GRVTY .... DOWN (sensed item) */
        e->items_checked[4] = in->gear_lever_down;
        break;
    case 320800040u:
        ACTIVE(s->gear_not_locked_up);
        fwss_items_all(e, 7);
        e->items_checked[2] = in->gear_lever_down;
        break;
    /* ---------------- ATA 34 ---------------- */
    case 340800001u:
        ACTIVE(s->adr_faulty[0]);
        SH(1, 1, 1, 1);
        CK(in->air_data_knob == 0, !in->adr_pb_on[0], 1, in->ir_mode[0] == 0);
        break;
    case 340800002u:
        ACTIVE(s->adr_faulty[1]);
        SH(1, 1);
        CK(in->air_data_knob == 2, !in->adr_pb_on[1]);
        break;
    case 340800003u:
        ACTIVE(s->adr_faulty[2]);
        SH(1, 1, 1, 1);
        CK(in->air_data_knob == 1, !in->adr_pb_on[2], 1, in->ir_mode[2] == 0);
        break;
    case 340800004u:
        ACTIVE(s->adr_faulty[0] && s->adr_faulty[1]);
        SH(1, 1, 1, 1, 1, 1, 1, 1);
        CK(in->air_data_knob == 0, !in->adr_pb_on[0], !in->adr_pb_on[1], 1, 1, 1, 1, in->ir_mode[0] == 0);
        break;
    case 340800005u:
        ACTIVE(s->adr_faulty[0] && s->adr_faulty[2]);
        SH(1, 1, 1, 1, 1, 1, 1, 1);
        CK(in->air_data_knob == 1, !in->adr_pb_on[0], !in->adr_pb_on[2], 1, 1, 1, 1, in->ir_mode[0] == 0);
        break;
    case 340800006u:
        ACTIVE(s->adr_faulty[1] && s->adr_faulty[2]);
        SH(1, 1, 1, 1, 1, 1, 1, 1);
        CK(in->air_data_knob == 1, !in->adr_pb_on[1], !in->adr_pb_on[2], 1, 1, 1, 1, in->ir_mode[2] == 0);
        break;
    case 340800008u:
        ACTIVE(s->adr_faulty[0] && s->adr_faulty[1] && s->adr_faulty[2]);
        SH(1, 1, 1);
        CK(0, !in->adr_pb_on[0] && !in->adr_pb_on[1] && !in->adr_pb_on[2], 1);
        break;
    case 340800040u: ACTIVE(s->ir_fault[0]); SH(1, 1); CK(in->att_hdg_knob == 0, !in->ir_pb_on[0]); break;
    case 340800041u: ACTIVE(s->ir_fault[1]); SH(1, 1); CK(in->att_hdg_knob == 2, !in->ir_pb_on[1]); break;
    case 340800072u: ACTIVE(s->ir_fault[2]); SH(1, 1); CK(in->att_hdg_knob == 1, !in->ir_pb_on[2]); break;
    case 340800042u:
        ACTIVE(s->ir_fault[0] && s->ir_fault[1]);
        SH(1, 1, 1, 1, 1);
        CK(in->att_hdg_knob == 0, 1, 1, !in->ir_pb_on[0], !in->ir_pb_on[1]);
        break;
    case 340800043u:
        ACTIVE(s->ir_fault[0] && s->ir_fault[2]);
        SH(1, 1, 1, 1, 1, 1);
        CK(in->att_hdg_knob == 1, 1, 1, !in->ir_pb_on[0], !in->ir_pb_on[2], 1);
        break;
    case 340800044u:
        ACTIVE(s->ir_fault[1] && s->ir_fault[2]);
        SH(1, 1, 1, 1, 1);
        CK(in->att_hdg_knob == 1, 1, 1, !in->ir_pb_on[1], !in->ir_pb_on[2]);
        break;
    case 340800053u: ACTIVE(s->ra_failed[0] && fwss_ac(in, 1)); break;
    case 340800054u: ACTIVE(s->ra_failed[1] && fwss_ac(in, 3)); break;
    case 340800055u: ACTIVE(s->ra_failed[2] && fwss_ac_ess(in)); break;
    case 340800059u: ACTIVE(s->ra_failed[0] && s->ra_failed[1] && fwss_ac(in, 1) && fwss_ac(in, 3)); break;
    case 340800060u: ACTIVE(s->ra_failed[0] && s->ra_failed[2] && fwss_ac(in, 1) && fwss_ac_ess(in)); break;
    case 340800061u: ACTIVE(s->ra_failed[1] && s->ra_failed[2] && fwss_ac(in, 3) && fwss_ac_ess(in)); break;
    case 340800062u:
        ACTIVE(s->ra_failed[0] && s->ra_failed[1] && s->ra_failed[2] && fwss_ac(in, 1) && fwss_ac(in, 3) &&
               fwss_ac_ess(in));
        SH(1, 1, 1);
        CK(1, 1, 1);
        break;
    case 341800016u: ACTIVE(s->tcas1_fault); SH(0); CK(0); break;
    case 341800017u: ACTIVE(s->tcas2_fault); SH(0); CK(0); break;
    case 341800018u: ACTIVE(s->tcas12_fault); break;
    case 341800019u: ACTIVE(s->tcas_standby); break;
    case 341800037u: ACTIVE(in->xpdr_stby); break;
    /* ---------------- ATA 52 ---------------- */
    case 520800017u: case 520800018u: case 520800019u: case 520800020u: case 520800021u:
    case 520800022u: case 520800023u: case 520800024u: case 520800025u: case 520800026u: {
        int d = (int)(id - 520800017u), air = !gnd;
        ACTIVE(in->main_door_open[d]);
        SH((fp == 2 || fp == 3) && 0 /* pressSysFault not simulated */, air, air, air, air, air);
        CK(0, 0, 0, 0, 0, 0);
        break;
    }
    /* ---------------- ATA 70 ---------------- */
    case 701800009u: case 701800010u: case 701800011u: case 701800012u: {
        int g = (int)(id - 701800009u);
        ACTIVE(in->eng_valid && in->eng_limits_valid && s->eng_running[g] && in->egt_c[g] > in->egt_red_c);
        break;
    }
    case 701800029u: case 701800030u: case 701800031u: case 701800032u: ACTIVE(s->eng_fail[id - 701800029u]); break;
    case 701800073u: case 701800074u: case 701800075u: case 701800076u: {
        int g = (int)(id - 701800073u);
        ACTIVE(in->eng_valid && in->eng_limits_valid && s->eng_running[g] &&
               (in->n1_pct[g] > in->n1_red_pct || in->n3_pct[g] > in->n3_red_pct));
        break;
    }
    case 701800085u: case 701800086u: case 701800087u: case 701800088u: {
        int g = (int)(id - 701800085u);
        ACTIVE(in->eng_valid && s->eng_running[g] && in->oil_press_low[g]);
        break;
    }
    case 701800109u: case 701800110u: case 701800111u: case 701800112u: {
        int g = (int)(id - 701800109u);
        int fob = in->fuel_valid && in->fuel_on_board_kg >= 55000.0f && in->fuel_on_board_kg <= 95000.0f;
        ACTIVE(s->eng_shutdown[g] && !s->all_engines_failure);
        n = (g == 0 || g == 3) ? 8 : 9;
        {
            unsigned char sh[9] = {!in->tcas_fault, fp == 7, (unsigned char)fob, (unsigned char)fob, 1, 1, 1, 1, 1};
            unsigned char ck[9] = {in->tcas_mode == 1, in->gear_lever_down, 0, in->crossfeed_open[g], 0,
                                   in->pack_flow_lo, 0 /* fwdCargoTempRegulatorOff */, 0, 0};
            fwss_vec(e->items_to_show, NULL, sh, (size_t)n);
            fwss_vec(e->items_checked, &e->n_items, ck, (size_t)n);
        }
        break;
    }
    case 701800117u: case 701800118u: case 701800119u: case 701800120u:
        ACTIVE(in->eng_valid && in->eng_start_fault[id - 701800117u]);
        break;
    case 701800151u: ACTIVE(s->all_engines_failure); break;
    case 701800158u: ACTIVE(s->eng_thrust_locked_alert); SH(1); CK(0); break;
    /* ---------------- secondary failures ---------------- */
    case 999800001u: case 999800003u: ACTIVE(s->green_lo || s->yellow_lo || s->green_yellow_lo); break;
    case 999800002u: case 999800005u: case 999800007u: ACTIVE(0); break; /* MappedSubject.create(or()) = false */
    case 999800004u: case 999800006u:
        ACTIVE(s->eng_shutdown[0] || s->eng_shutdown[1] || s->eng_shutdown[2] || s->eng_shutdown[3]);
        break;
    /* ---------------- deferred procedures ---------------- */
    case 210700001u: ACTIVE(1); SH(1, 1); CK(in->pack_pb_on[0], in->pack_pb_on[1]); break;
    case 210700002u: ACTIVE(1); SH(1, 1); CK(in->ram_air_on, in->cabin_air_extract_on); break;
    case 221700001u: ACTIVE(1); SH(1, 1); CK(in->man_cab_alt_mode, 0); break;
    case 270700001u: ACTIVE(1); SH(1, 1, 1, 1, 1); CK(0, 0, 0, 0, 0); break;
    case 320700001u:
        ACTIVE(1);
        SH(1, 1, 1, 1, 1);
        /* FwsAbnormalSensed 320700001: L/G LEVER UP, L/G_GRVTY_SWITCH_POS === 2, all down-locked, LEVER DOWN */
        CK(0, !in->gear_lever_down, in->lg_grvty_switch_pos == 2, s->all_gear_downlocked, in->gear_lever_down);
        break;
    default:
        break;
    }
}

#undef SH
#undef CK
#undef ACTIVE

/* ============================================================================================== */
/* Deprecated per-alert lists                                                                    */
/* ============================================================================================== */
typedef struct {
    unsigned *out;
    int max, n;
} FwssListOut;

static void fwss_l(FwssListOut *o, unsigned id)
{
    /* '' and null entries are dropped by pushKeyUnique; 0 encodes them here */
    if (o->n < o->max)
        o->out[o->n] = id;
    o->n++;
}

int fws_signals_alert_list(const FwsSignals *s, int index, FwsListKind k, const unsigned char *checked,
                           unsigned n_checked, unsigned *out, int max)
{
    const FwsInputs *in = &s->in;
    FwssListOut o;
    unsigned id;
    if (index < 0 || index >= FWSS_NUM_ALERTS || !out)
        return 0;
    o.out = out;
    o.max = max;
    o.n = 0;
    id = fwss_alerts[index].id;
    switch (id) {
    case 211800011u: if (k == FWS_LIST_INOP_ALL) fwss_l(&o, 210300009u); break;
    case 211800012u: if (k == FWS_LIST_INOP_ALL) fwss_l(&o, 210300010u); break;
    case 211800021u:
        if (k == FWS_LIST_LIM_ALL || k == FWS_LIST_LIM_PFD) { fwss_l(&o, 2u); fwss_l(&o, 210400001u); }
        if (k == FWS_LIST_INOP_ALL) fwss_l(&o, 210300011u);
        break;
    case 211800051u: if (k == FWS_LIST_INOP_ALL) fwss_l(&o, 260300006u); break;
    case 211800052u: case 211800053u: case 211800054u: case 211800055u:
        if (k == FWS_LIST_INOP_ALL) fwss_l(&o, 260300002u + (id - 211800052u));
        break;
    case 213800001u: if (k == FWS_LIST_LIM_ALL || k == FWS_LIST_LIM_PFD) fwss_l(&o, 210400002u); break;
    case 213800002u:
        if (k == FWS_LIST_LIM_ALL || k == FWS_LIST_LIM_PFD) { fwss_l(&o, 210400002u); fwss_l(&o, 210400001u); }
        break;
    case 213800009u: if (k == FWS_LIST_LIM_ALL || k == FWS_LIST_LIM_PFD) fwss_l(&o, 210400001u); break;
    case 320800018u:
        if (k == FWS_LIST_LIM_ALL || k == FWS_LIST_LIM_PFD) fwss_l(&o, s->aircraft_on_ground ? 0u : 260400002u);
        break;
    case 281800002u: if (k == FWS_LIST_LIM_ALL || k == FWS_LIST_LIM_PFD) fwss_l(&o, 2u); break;
    case 290800035u:
        if (k == FWS_LIST_LIM_PFD) fwss_l(&o, 320400001u);
        if (k == FWS_LIST_LIM_ALL) fwss_l(&o, 0u); /* prim3 healthy -> null */
        if (k == FWS_LIST_LIM_APPR_LDG) {
            fwss_l(&o, 320400001u); fwss_l(&o, 290400001u); fwss_l(&o, 290400002u);
            fwss_l(&o, 320400002u); fwss_l(&o, 320400003u);
        }
        if (k == FWS_LIST_INFO) { fwss_l(&o, 220200011u); fwss_l(&o, 270200001u); }
        if (k == FWS_LIST_INOP_ALL) { fwss_l(&o, 290100001u); fwss_l(&o, 320300023u); fwss_l(&o, 290300021u); }
        if (k == FWS_LIST_INOP_APPR_LDG) { fwss_l(&o, 290100003u); fwss_l(&o, 290100006u); fwss_l(&o, 320300020u); }
        break;
    case 290800036u:
        if (k == FWS_LIST_LIM_ALL) fwss_l(&o, 0u);
        if (k == FWS_LIST_INFO) { fwss_l(&o, 800200004u); fwss_l(&o, 800200004u); }
        if (k == FWS_LIST_INOP_ALL) { fwss_l(&o, 290100001u); fwss_l(&o, 320300023u); fwss_l(&o, 290300022u); }
        if (k == FWS_LIST_INOP_APPR_LDG) { fwss_l(&o, 290100004u); fwss_l(&o, 320300024u); }
        break;
    case 290800039u:
        if (k == FWS_LIST_LIM_PFD) fwss_l(&o, 320400001u);
        if (k == FWS_LIST_LIM_ALL) fwss_l(&o, 800400001u);
        if (k == FWS_LIST_LIM_APPR_LDG) {
            fwss_l(&o, 320400001u); fwss_l(&o, 270400001u); fwss_l(&o, 290400001u); fwss_l(&o, 320400002u);
            fwss_l(&o, 220400001u); fwss_l(&o, 800400004u); fwss_l(&o, 320400003u);
        }
        if (k == FWS_LIST_INFO) {
            fwss_l(&o, 340200002u); fwss_l(&o, 220200011u); fwss_l(&o, 320200001u); fwss_l(&o, 800200004u);
            fwss_l(&o, 800200005u);
        }
        if (k == FWS_LIST_INOP_ALL) {
            fwss_l(&o, 290100007u); fwss_l(&o, 290100008u); fwss_l(&o, 290100009u); fwss_l(&o, 290100010u);
            fwss_l(&o, 290100011u); fwss_l(&o, 320300013u); fwss_l(&o, 290300021u); fwss_l(&o, 290300022u);
        }
        if (k == FWS_LIST_INOP_APPR_LDG) {
            fwss_l(&o, 290100012u); fwss_l(&o, 290100006u); fwss_l(&o, 320300008u); fwss_l(&o, 320300014u);
            fwss_l(&o, 320300020u);
        }
        break;
    case 320800008u:
        if (k == FWS_LIST_INOP_ALL) { fwss_l(&o, 320300001u); fwss_l(&o, 320300002u); }
        if (k == FWS_LIST_INFO) {
            fwss_l(&o, 320200002u); fwss_l(&o, 320200003u); fwss_l(&o, s->one_eng_running ? 800200003u : 0u);
            fwss_l(&o, 800200005u);
        }
        break;
    case 340800001u:
        if (k == FWS_LIST_INOP_ALL) {
            fwss_l(&o, 340300004u);
            fwss_l(&o, in->air_data_knob == 0 ? 0u : 340300011u);
            fwss_l(&o, in->air_data_knob == 0 ? 0u : 340300001u);
        }
        break;
    case 340800002u:
        if (k == FWS_LIST_INOP_ALL) {
            fwss_l(&o, 340300005u);
            fwss_l(&o, in->air_data_knob == 2 ? 0u : 340300012u);
            fwss_l(&o, in->air_data_knob == 2 ? 0u : 340300002u);
        }
        break;
    case 340800003u: if (k == FWS_LIST_INOP_ALL) fwss_l(&o, 340300006u); break;
    case 340800004u: case 340800005u: case 340800006u: {
        int a12 = id == 340800004u, a13 = id == 340800005u;
        if (k == FWS_LIST_INOP_ALL) {
            fwss_l(&o, 290100008u); fwss_l(&o, 340300014u); fwss_l(&o, 340300021u); fwss_l(&o, 220300007u);
            fwss_l(&o, 220300024u);
            fwss_l(&o, a12 ? 340300007u : a13 ? 340300009u : 340300008u);
            if (a12) {
                fwss_l(&o, in->air_data_knob == 0 ? 340300012u : 340300029u);
                fwss_l(&o, in->air_data_knob == 0 ? 340300002u : 340300003u);
            } else if (a13) {
                fwss_l(&o, 340300011u); fwss_l(&o, 340300001u);
            } else {
                fwss_l(&o, 340300012u); fwss_l(&o, 340300002u);
            }
        }
        if (k == FWS_LIST_INOP_APPR_LDG) { fwss_l(&o, 220300010u); fwss_l(&o, 220300025u); }
        if (k == FWS_LIST_INFO) { fwss_l(&o, 340200002u); fwss_l(&o, 340200003u); }
        if (k == FWS_LIST_LIM_APPR_LDG) fwss_l(&o, 240400001u);
        break;
    }
    case 340800008u:
        if (k == FWS_LIST_INOP_ALL) {
            fwss_l(&o, 290100008u); fwss_l(&o, 340300014u); fwss_l(&o, 340300021u); fwss_l(&o, 220300007u);
            fwss_l(&o, 220300024u); fwss_l(&o, 340300010u); fwss_l(&o, 340300029u); fwss_l(&o, 340300003u);
        }
        if (k == FWS_LIST_INOP_APPR_LDG) { fwss_l(&o, 220300010u); fwss_l(&o, 220300021u); fwss_l(&o, 220300025u); }
        if (k == FWS_LIST_INFO) { fwss_l(&o, 340200002u); fwss_l(&o, 340200003u); fwss_l(&o, 340200007u); }
        if (k == FWS_LIST_LIM_ALL || k == FWS_LIST_LIM_PFD) {
            fwss_l(&o, 240400002u); fwss_l(&o, 240400003u); fwss_l(&o, 240400004u); fwss_l(&o, 300400001u);
        }
        if (k == FWS_LIST_LIM_APPR_LDG) fwss_l(&o, 240400001u);
        break;
    case 340800040u: case 340800041u: {
        int two = id == 340800041u, own = two ? (in->att_hdg_knob == 2) : (in->att_hdg_knob == 0);
        if (k == FWS_LIST_INOP_ALL) {
            fwss_l(&o, two ? 340300031u : 340300030u);
            fwss_l(&o, own ? 0u : (two ? 340300012u : 340300011u));
            fwss_l(&o, own ? 0u : (two ? 340300038u : 340300037u));
            fwss_l(&o, own ? 0u : (two ? 340300040u : 340300039u));
            fwss_l(&o, own ? 0u : (two ? 340300042u : 340300041u));
        }
        break;
    }
    case 340800072u: if (k == FWS_LIST_INOP_ALL) fwss_l(&o, 340300032u); break;
    case 340800042u: {
        int k0 = in->att_hdg_knob == 0;
        if (k == FWS_LIST_INOP_ALL) {
            fwss_l(&o, 290100008u); fwss_l(&o, 340300033u);
            fwss_l(&o, k0 ? 340300038u : 340300043u); fwss_l(&o, k0 ? 340300012u : 340300029u);
            fwss_l(&o, k0 ? 340300040u : 340300044u); fwss_l(&o, k0 ? 340300042u : 340300045u);
        }
        if (k == FWS_LIST_INFO) fwss_l(&o, 340200002u);
        break;
    }
    case 340800043u:
        if (k == FWS_LIST_INOP_ALL) {
            fwss_l(&o, 290100008u); fwss_l(&o, 340300034u); fwss_l(&o, 220300005u); fwss_l(&o, 220300022u);
            fwss_l(&o, 340300037u); fwss_l(&o, 340300011u); fwss_l(&o, 340300039u); fwss_l(&o, 340300041u);
        }
        if (k == FWS_LIST_INFO) { fwss_l(&o, 340200002u); fwss_l(&o, 340200008u); }
        break;
    case 340800044u:
        if (k == FWS_LIST_INOP_ALL) {
            fwss_l(&o, 290100008u); fwss_l(&o, 340300035u); fwss_l(&o, 220300006u); fwss_l(&o, 220300023u);
            fwss_l(&o, 340300038u); fwss_l(&o, 340300012u); fwss_l(&o, 340300040u); fwss_l(&o, 340300042u);
        }
        if (k == FWS_LIST_INFO) fwss_l(&o, 340200002u);
        break;
    case 340800053u: case 340800054u: case 340800055u:
        if (k == FWS_LIST_REDUND_LOSS) fwss_l(&o, 340300022u + (id - 340800053u));
        if ((k == FWS_LIST_INOP_APPR_LDG || k == FWS_LIST_INFO) && id != 340800054u) fwss_l(&o, 0u);
        break;
    case 340800059u: case 340800060u: case 340800061u:
        if (k == FWS_LIST_INOP_APPR_LDG) fwss_l(&o, 340300025u + (id - 340800059u));
        break;
    case 340800062u:
        if (k == FWS_LIST_INOP_ALL) { fwss_l(&o, 340300029u); fwss_l(&o, 340300003u); fwss_l(&o, 341300003u); }
        if (k == FWS_LIST_INOP_APPR_LDG) {
            fwss_l(&o, 340300028u); fwss_l(&o, 310300001u); fwss_l(&o, 220300010u); fwss_l(&o, 220300021u);
        }
        if (k == FWS_LIST_INFO) { fwss_l(&o, 220200007u); fwss_l(&o, 220200008u); fwss_l(&o, 220200009u); }
        break;
    case 520800017u: case 520800018u: case 520800019u: case 520800020u: case 520800021u:
    case 520800022u: case 520800023u: case 520800024u: case 520800025u: case 520800026u:
        if ((k == FWS_LIST_LIM_ALL || k == FWS_LIST_LIM_PFD) && checked && n_checked > 1 && checked[1])
            fwss_l(&o, 210400001u);
        break;
    default:
        break;
    }
    return o.n < max ? o.n : max;
}

/* ============================================================================================== */
/* Suppressable items (activation time, itemIsActiveConsideringFaultSuppression)                 */
/* ============================================================================================== */

static int fwss_inop_active(const FwsSignals *s, unsigned id)
{
    const FwsInputs *in = &s->in;
    int p121112 = fwss_phase12_or_1112(s->flight_phase);
    int gen_inop[4], i;
    for (i = 0; i < 4; i++)
        gen_inop[i] = !p121112 && !s->eng_running[i];
    switch (id) {
    case 240300010u: return gen_inop[0];
    case 240300011u: return gen_inop[1];
    case 240300012u: return gen_inop[2];
    case 240300013u: return gen_inop[3];
    case 240300037u: return gen_inop[0] && gen_inop[1];
    case 240300038u: return gen_inop[2] && gen_inop[3];
    case 260300002u: return s->eng_out[0];
    case 260300003u: return s->eng_out[1];
    case 260300004u: return s->eng_out[2];
    case 260300005u: return s->eng_out[3]; /* [DEV 10] FBW eng4BleedInop = gen4Inop */
    case 290100001u: return s->green_lo || s->yellow_lo;
    case 290100003u: return in->flap_sys_fault[0];
    case 290100004u: return in->flap_sys_fault[1];
    case 290100005u: return in->slat_sys_fault[0];
    case 290100006u: return in->slat_sys_fault[1];
    case 290100011u: return s->green_lo && s->yellow_lo;
    case 290100012u: return in->flap_sys_fault[0] && in->flap_sys_fault[1];
    case 290100013u: return in->slat_sys_fault[0] && in->slat_sys_fault[1];
    case 290300021u: return s->green_lo;
    case 290300022u: return s->yellow_lo;
    case 290300023u: return gen_inop[0];
    case 290300024u: return gen_inop[1];
    case 290300025u: return gen_inop[2];
    case 290300026u: return gen_inop[3];
    case 340300004u: return s->adr_faulty[0];
    case 340300005u: return s->adr_faulty[1];
    case 340300006u: return s->adr_faulty[2];
    case 340300007u: return s->adr_faulty[0] && s->adr_faulty[1];
    case 340300008u: return s->adr_faulty[1] && s->adr_faulty[2];
    case 340300009u: return s->adr_faulty[0] && s->adr_faulty[2];
    case 340300010u: return s->adr_faulty[0] && s->adr_faulty[1] && s->adr_faulty[2];
    case 340300011u: return s->tcas1_fault;
    case 340300012u: return s->tcas2_fault;
    case 340300029u: return s->tcas12_fault;
    case 340300022u: return s->ra_failed[0];
    case 340300023u: return s->ra_failed[1];
    case 340300024u: return s->ra_failed[2];
    case 340300025u: return s->ra_failed[0] && s->ra_failed[1];
    case 340300026u: return s->ra_failed[0] && s->ra_failed[2];
    case 340300027u: return s->ra_failed[1] && s->ra_failed[2];
    case 340300028u: return s->ra_failed[0] && s->ra_failed[1] && s->ra_failed[2];
    case 340300030u: return s->ir_fault[0];
    case 340300031u: return s->ir_fault[1];
    case 340300032u: return s->ir_fault[2];
    case 340300033u: return s->ir_fault[0] && s->ir_fault[1];
    case 340300034u: return s->ir_fault[0] && s->ir_fault[2];
    case 340300035u: return s->ir_fault[1] && s->ir_fault[2];
    case 340300036u: return s->ir_fault[0] && s->ir_fault[1] && s->ir_fault[2];
    case 700300001u: return s->eng_out[1];
    case 700300002u: return s->eng_out[2];
    case 700300003u: return s->eng_out[1] && s->eng_out[2];
    default: return 0;
    }
}

static int fwss_lim_active(const FwsSignals *s, unsigned id)
{
    const FwsInputs *in = &s->in;
    switch (id) {
    case 1u: /* landAsap (FwsCore:5540-5550); MLG fire / smoke not simulated */
        return !s->aircraft_on_ground &&
               (in->apu_fire_detected || in->eng_fire_detected[0] || in->eng_fire_detected[1] ||
                in->eng_fire_detected[2] || in->eng_fire_detected[3] || s->all_engines_failure);
    case 260400001u: return in->eng_fire_pb_released[0];
    case 290400001u: return in->slat_sys_fault[0] && in->slat_sys_fault[1];
    case 290400002u: return in->flap_sys_fault[0] && in->flap_sys_fault[1];
    default: return 0;
    }
}

static int fwss_sup_index(unsigned id, int alerts_only)
{
    int i;
    for (i = 0; i < FWSS_NUM_ALERTS; i++)
        if (fwss_alerts[i].id == id)
            return i;
    if (alerts_only)
        return -1;
    for (i = 0; i < FWSS_NUM_INOP; i++)
        if (fwss_inop[i].id == id)
            return FWSS_SUP_INOP0 + i;
    for (i = 0; i < FWSS_NUM_LIMS; i++)
        if (fwss_lims[i].id == id)
            return FWSS_SUP_LIM0 + i;
    return -1;
}

static double fwss_sup_confirm(int sup, double def)
{
    if (sup < FWSS_NUM_ALERTS)
        return fwss_alerts[sup].monitor_confirm_s;
    return def;
}

/* FwsCore:5568-5601 itemIsActiveConsideringFaultSuppression. The suppression lists of the
 * dictionaries only reference keys of the same dictionary. */
static int fwss_item_confirmed(const FwsSignals *s, int sup, const unsigned *not_active, int n_not, double def)
{
    double since;
    int i;
    if (sup < 0 || !s->sup_active[sup])
        return 0;
    since = s->sup_since[sup];
    if (s->clock_s < since + fwss_sup_confirm(sup, def))
        return 0;
    for (i = 0; i < n_not; i++) {
        int o;
        if (!not_active[i])
            continue;
        o = fwss_sup_index(not_active[i], 0);
        if (o >= 0 && s->sup_active[o] && s->clock_s >= since + fwss_sup_confirm(o, def))
            return 0;
    }
    return 1;
}

int fws_signals_alert_confirmed(const FwsSignals *s, int index)
{
    if (index < 0 || index >= FWSS_NUM_ALERTS)
        return 0;
    return fwss_item_confirmed(s, index, fwss_alerts[index].not_active_when, FWS_SIG_MAX_SUPPRESS, 0.6);
}

/* ============================================================================================== */
/* STS lists (FwsCore:5784-5806, 5987-6030, 6108-6150)                                           */
/* ============================================================================================== */

static void fwss_push_unique(FwsIdList *l, unsigned id)
{
    unsigned i;
    if (!id)
        return;
    for (i = 0; i < l->n; i++)
        if (l->id[i] == id)
            return;
    if (l->n < FWS_SIG_LIST_MAX)
        l->id[l->n++] = id;
}

static int fwss_lim_priority(unsigned id)
{
    /* messagePriority(EcamLimitations[id]): '1' red, '2' amber, others cyan / undefined */
    if (id == 1u)
        return 1;
    if (id == 2u)
        return 2;
    return 10;
}

static void fwss_sort_lims(FwsIdList *l)
{
    unsigned i, j;
    for (i = 1; i < l->n; i++) { /* stable insertion sort */
        unsigned v = l->id[i];
        for (j = i; j > 0 && fwss_lim_priority(l->id[j - 1]) > fwss_lim_priority(v); j--)
            l->id[j] = l->id[j - 1];
        l->id[j] = v;
    }
}

static void fwss_remove(FwsIdList *l, unsigned id)
{
    unsigned i, j;
    for (i = 0; i < l->n; i++)
        if (l->id[i] == id) {
            for (j = i; j + 1 < l->n; j++)
                l->id[j] = l->id[j + 1];
            l->n--;
            return;
        }
}

static int fwss_has(const FwsIdList *l, unsigned id)
{
    unsigned i;
    for (i = 0; i < l->n; i++)
        if (l->id[i] == id)
            return 1;
    return 0;
}

static void fwss_push_alert_list(const FwsSignals *s, int idx, FwsListKind kind, const unsigned char *checked,
                                 unsigned n_checked, FwsIdList *dst)
{
    unsigned tmp[FWS_SIG_LIST_MAX];
    int n = fws_signals_alert_list(s, idx, kind, checked, n_checked, tmp, FWS_SIG_LIST_MAX), i;
    for (i = 0; i < n; i++)
        fwss_push_unique(dst, tmp[i]);
}

void fws_signals_status_lists(const FwsSignals *s, const FwsActiveAlertRef *alerts, int n_alerts,
                              FwsStatusLists *out)
{
    FwsIdList pfd;
    int i;
    memset(out, 0, sizeof *out);
    memset(&pfd, 0, sizeof pfd);
    if (alerts) {
        for (i = 0; i < n_alerts; i++) {
            int idx = fws_signals_alert_index(alerts[i].id);
            /* pushKeyUnique requires a defined checked state ([] is truthy, undefined is not) */
            if (idx < 0 || fwss_alerts[idx].non_procedure || fwss_alerts[idx].deferred || !alerts[i].checked)
                continue;
            fwss_push_alert_list(s, idx, FWS_LIST_INFO, alerts[i].checked, alerts[i].n_checked, &out->information);
            fwss_push_alert_list(s, idx, FWS_LIST_INOP_ALL, alerts[i].checked, alerts[i].n_checked,
                                 &out->inop_sys_all_phases);
            fwss_push_alert_list(s, idx, FWS_LIST_INOP_APPR_LDG, alerts[i].checked, alerts[i].n_checked,
                                 &out->inop_sys_appr_ldg);
            fwss_push_alert_list(s, idx, FWS_LIST_LIM_ALL, alerts[i].checked, alerts[i].n_checked,
                                 &out->limitations_all_phases);
            fwss_push_alert_list(s, idx, FWS_LIST_LIM_APPR_LDG, alerts[i].checked, alerts[i].n_checked,
                                 &out->limitations_appr_ldg);
            fwss_push_alert_list(s, idx, FWS_LIST_LIM_PFD, alerts[i].checked, alerts[i].n_checked, &pfd);
            /* the LAND ASAP / ANSA push from proc.recommendation (FwsCore:5802) passes no state and
             * therefore never pushes in FBW; kept as is */
        }
    }
    /* INOP SYS dictionary */
    for (i = 0; i < FWSS_NUM_INOP; i++) {
        int sup = FWSS_SUP_INOP0 + i;
        if (!fwss_item_confirmed(s, sup, fwss_inop[i].not_active_when, 3, 1.0))
            continue;
        if (!fwss_inop[i].redund && !fwss_inop[i].appr_ldg)
            fwss_push_unique(&out->inop_sys_all_phases, fwss_inop[i].id);
        else if (!fwss_inop[i].redund && fwss_inop[i].appr_ldg)
            fwss_push_unique(&out->inop_sys_appr_ldg, fwss_inop[i].id);
        else if (fwss_inop[i].redund)
            fwss_push_unique(&out->inop_sys_redundancy_loss, fwss_inop[i].id);
    }
    /* INFO dictionary: no entry derivable in the port (FwsInformation.ts) */
    /* LIMITATIONS dictionary */
    for (i = 0; i < FWSS_NUM_LIMS; i++) {
        int sup = FWSS_SUP_LIM0 + i;
        if (!fwss_item_confirmed(s, sup, fwss_lims[i].not_active_when, 3, 1.0))
            continue;
        if (!fwss_lims[i].appr_ldg)
            fwss_push_unique(&out->limitations_all_phases, fwss_lims[i].id);
        else
            fwss_push_unique(&out->limitations_appr_ldg, fwss_lims[i].id);
        if (fwss_lims[i].redund)
            fwss_push_unique(&pfd, fwss_lims[i].id);
    }
    if (fwss_has(&out->limitations_all_phases, 1u) && fwss_has(&out->limitations_all_phases, 2u))
        fwss_remove(&out->limitations_all_phases, 2u);
    fwss_sort_lims(&out->limitations_all_phases);
    fwss_sort_lims(&out->limitations_appr_ldg);
    fwss_sort_lims(&pfd);
    if (fwss_has(&pfd, 1u) && fwss_has(&pfd, 2u))
        fwss_remove(&pfd, 2u);
    out->limitations_pfd = pfd;
    out->ecam_status_normal = (unsigned char)(!out->information.n && !out->inop_sys_all_phases.n &&
                                              !out->inop_sys_appr_ldg.n && !out->limitations_all_phases.n &&
                                              !out->limitations_appr_ldg.n);
}

/* ============================================================================================== */
/* Memos (FwsMemos.ts; assembly FwsCore:5934-5985)                                               */
/* ============================================================================================== */

/* messagePriority(EcamMemos[code]): magenta 0, red 1, amber 2, green 3 */
static int fwss_memo_priority(unsigned code)
{
    switch (code) {
    case 314000001u: case 314000002u:
        return 0;
    case 220000001u:
        return 1;
    case 220000002u: case 221000002u: case 241000001u: case 242000001u: case 280000013u: case 320000001u:
    case 322000001u: case 340003002u: case 340003004u: case 340003006u: case 340003008u: case 340003102u:
    case 340003104u: case 340003106u: case 340003108u: case 350000002u:
        return 2;
    default:
        return 3;
    }
}

static int fwss_count_running(const FwsSignals *s)
{
    return fwss_count4(s->eng_running[0], s->eng_running[1], s->eng_running[2], s->eng_running[3]);
}

/* FwsCore.adirsMessage1 / adirsMessage2 */
static int fwss_adirs_row(double t, int running, int second)
{
    int m = (int)ceil(t / 60.0);
    if (!second) {
        if (m >= 7) return running ? 1 : 0;
        if (m == 6) return running ? 3 : 2;
        if (m == 5) return running ? 5 : 4;
        if (m == 4) return running ? 7 : 6;
        return 0;
    }
    if (m == 3) return running ? 1 : 0;
    if (m == 2) return running ? 3 : 2;
    if (m == 1) return running ? 5 : 4;
    return 0;
}

static void fwss_update_memos(FwsSignals *s)
{
    const FwsInputs *in = &s->in;
    const int fp = s->flight_phase;
    const int to_ldg = s->to_memo || s->ldg_memo;
    unsigned right[64];
    int nr = 0, i, j;
    int eng_state_running = 0;
    for (i = 0; i < 4; i++) {
        int st = in->eng_valid ? in->eng_state[i] : 0;
        if (st > 0 && st < 4)
            eng_state_running = 1;
    }
#define MEMO(cond, inhib_mask, code)                                  \
    do {                                                              \
        if ((cond) && !fwss_phase_in(fp, (inhib_mask)) && nr < 64)    \
            right[nr++] = (code);                                     \
    } while (0)
    /* ewdMemos in Object.entries order: integer keys ascending, then '0000170', '0000180', '0000290' */
    MEMO(s->high_landing_field, B(3) | B(4) | B(5) | B(6) | B(7) | B(9) | B(10), 210000001u);
    MEMO(s->ap_off_show_memo, 0, 220000001u);
    MEMO(s->vol_athr_off_memo, 0, 220000002u);
    MEMO(in->fms_valid && in->fms_dest_efob_below_min, 0, 221000002u);
    MEMO(in->vhf3_voice, B(3) | B(4) | B(5) | B(7) | B(8), 230000015u);
    {
        int ext = in->elec_valid && (in->ext_contactor_closed[0] || in->ext_contactor_closed[1] ||
                                     in->ext_contactor_closed[2] || in->ext_contactor_closed[3]);
        MEMO(ext, B(3) | B(4) | B(5) | B(6) | B(7) | B(8), fwss_count_running(s) > 1 ? 241000001u : 241000002u);
    }
    MEMO(in->elec_valid && in->rat_stow_ratio > 0.0f, 0,
         (fp == 1 || fp == 2 || fp == 11) ? 242000001u : 242000002u);
    MEMO(in->gnd_splr_armed && !to_ldg, 0, 271000001u);
    MEMO(s->crossfeed_open_memo, B(3) | B(4) | B(5) | B(6) | B(7), 280000001u);
    MEMO(in->defuel_in_progress, B(3) | B(4) | B(5) | B(6) | B(7) | B(8) | B(9) | B(10), 280000003u);
    MEMO(in->refuel_in_progress, B(3) | B(4) | B(5) | B(6) | B(7) | B(8) | B(9) | B(10), 280000009u);
    MEMO(in->refuel_panel_open, B(3) | B(4) | B(5) | B(6) | B(7) | B(8) | B(9) | B(10),
         s->one_eng_running ? 280000011u : 280000010u);
    MEMO(s->crossfeed_open_memo, B(1) | B(2) | B(8) | B(9) | B(10) | B(11) | B(12), 280000013u);
    MEMO(in->elec_pump_active[0], 0, 290000001u);
    MEMO(in->elec_pump_active[1], 0, 290000002u);
    MEMO(in->elec_pump_active[2], 0, 290000003u);
    MEMO(in->elec_pump_active[3], 0, 290000004u);
    MEMO(in->eng_anti_ice_on[0] || in->eng_anti_ice_on[1] || in->eng_anti_ice_on[2] || in->eng_anti_ice_on[3], 0,
         300000001u);
    MEMO(in->wing_anti_ice_on, 0, 300000002u);
    MEMO(s->ice_not_detected && !s->aircraft_on_ground, B(1) | B(2) | B(3) | B(4) | B(8) | B(9) | B(10), 300000003u);
    MEMO(s->show_takeoff_inhibit, 0, 314000001u);
    MEMO(s->show_landing_inhibit, 0, 314000002u);
    MEMO(s->autobrake_off_memo, 0, 320000001u);
    MEMO(in->park_brake_set, B(3) | B(4) | B(5) | B(6) | B(7) | B(8) | B(9) | B(10), 320000002u);
    MEMO(in->nw_steering_disc, B(3) | B(4) | B(5) | B(7) | B(8) | B(9) | B(10),
         fwss_count_running(s) > 1 ? 322000001u : 322000002u);
    MEMO(in->strobe_off && fp == 8, PH_ALL_BUT_8, 333000001u);
    MEMO(in->seat_belts_on && !to_ldg, 0, 335000001u);
    MEMO(in->no_mobile_on, 0, 335000003u);
    MEMO(in->true_north_ref, 0, 340000001u);
    {
        int all_align = in->ir_in_align[0] && in->ir_in_align[1] && in->ir_in_align[2];
        double t = in->ir_align_remaining_s;
        MEMO(t >= 240.0 && all_align, B(3) | B(4) | B(5) | B(6) | B(7) | B(8) | B(9) | B(10),
             340003001u + (unsigned)fwss_adirs_row(t, eng_state_running, 0));
        MEMO(t > 0.0 && t < 240.0 && all_align, B(3) | B(4) | B(5) | B(6) | B(7) | B(8) | B(9) | B(10),
             340003101u + (unsigned)fwss_adirs_row(t, eng_state_running, 1));
    }
    MEMO(in->att_hdg_knob != 1 || in->air_data_knob != 1, 0, 340068001u);
    MEMO(s->tcas_standby_memo, B(1) | B(12), 343000001u);
    MEMO(!in->xpdr_alt_reporting, B(1) | B(12), 343000002u);
    MEMO(in->xpdr_stby, B(1) | B(12), 343000003u);
    /* codesToReturn of OXY PAX SYS ON is evaluated once at construction (on ground false, excess
     * cabin altitude false) and is therefore always '350000002' in FBW */
    MEMO(in->pax_oxy_masks_deployed, 0, 350000002u);
    MEMO(in->eng_mode_selector == 2, 0, 709000001u);
    MEMO(in->apu_avail && !in->apu_bleed_valve_open, 0, 17001u);
    MEMO(in->apu_avail && in->apu_bleed_valve_open, 0, 18001u);
#undef MEMO
    /* mapOrder() with an empty order list reverses the array in V8 / JavaScriptCore, then the stable
     * sort by messagePriority */
    for (i = 0; i < nr / 2; i++) {
        unsigned t = right[i];
        right[i] = right[nr - 1 - i];
        right[nr - 1 - i] = t;
    }
    for (i = 1; i < nr; i++) {
        unsigned v = right[i];
        for (j = i; j > 0 && fwss_memo_priority(right[j - 1]) > fwss_memo_priority(v); j--)
            right[j] = right[j - 1];
        right[j] = v;
    }
    s->n_memo_right = nr < 10 ? nr : 10;
    for (i = 0; i < s->n_memo_right; i++)
        s->memo_right[i] = right[i];
    /* PFD memos: pfdMemoDisplay filter, A/THR OFF on involuntary disconnection, priority sort */
    {
        unsigned pfd[16];
        int np = 0, has_athr = 0;
        for (i = 0; i < nr && np < 15; i++) {
            unsigned c = right[i];
            if (c == 6002u || c == 220000001u || c == 220000002u || c == 300000001u || c == 300000002u ||
                c == 320000001u) {
                pfd[np++] = c;
                if (c == 220000002u)
                    has_athr = 1;
            }
        }
        if (s->athr_invol_pfd_memo_mem.value && !has_athr)
            pfd[np++] = 220000002u;
        for (i = 1; i < np; i++) {
            unsigned v = pfd[i];
            for (j = i; j > 0 && fwss_memo_priority(pfd[j - 1]) > fwss_memo_priority(v); j--)
                pfd[j] = pfd[j - 1];
            pfd[j] = v;
        }
        s->n_pfd_memo = np < 3 ? np : 3;
        for (i = 0; i < s->n_pfd_memo; i++)
            s->pfd_memo[i] = pfd[i];
    }
    /* T.O / LDG memos (left), ordered by codesToReturn index */
    s->n_memo_left = 0;
    if (s->to_memo && !fwss_phase_in(fp, B(1) | B(3) | B(8) | B(12)) && s->n_memo_left + 6 <= 10) {
        int flaps_to = in->flaps_handle_index >= 1 && in->flaps_handle_index <= 3;
        /* L:A32NX_NO_SMOKING_MEMO is forced true by FwsCore.healthInjector */
        s->memo_left[s->n_memo_left++] = 1001u;
        s->memo_left[s->n_memo_left++] = in->seat_belts_on ? 1003u : 1002u;
        s->memo_left[s->n_memo_left++] = in->gnd_splr_armed ? 1007u : 1006u;
        s->memo_left[s->n_memo_left++] = flaps_to ? 1009u : 1008u;
        s->memo_left[s->n_memo_left++] = in->autobrake_mode == 6 ? 1011u : 1010u;
        s->memo_left[s->n_memo_left++] = s->to_config_normal ? 1013u : 1012u;
    }
    if (s->ldg_memo && !fwss_phase_in(fp, B(1) | B(2) | B(3) | B(4) | B(5) | B(6) | B(7) | B(11) | B(12)) &&
        s->n_memo_left + 5 <= 10) {
        int conf3 = in->fms_valid && in->fms_landing_conf3;
        int flaps_ldg = (!conf3 && in->flaps_handle_index == 4) || (conf3 && in->flaps_handle_index == 3);
        s->memo_left[s->n_memo_left++] = 2001u;
        s->memo_left[s->n_memo_left++] = in->seat_belts_on ? 2003u : 2002u;
        s->memo_left[s->n_memo_left++] = s->all_gear_downlocked ? 2007u : 2006u;
        s->memo_left[s->n_memo_left++] = in->gnd_splr_armed ? 2009u : 2008u;
        s->memo_left[s->n_memo_left++] = flaps_ldg ? 2011u : 2010u;
    }
}

/* ============================================================================================== */
/* update                                                                                        */
/* ============================================================================================== */
void fws_signals_update(FwsSignals *s, const FwsInputs *in, double clock_s, float dt_f)
{
    double dt = dt_f > 0.0f ? (double)dt_f : 0.0;
    int i;
    FwsAlertEval e;

    s->in = *in;
    s->clock_s = clock_s;
    s->dt = dt;

    /* buttons (FwsCore:2932-2985). [DEV 11] the T.O CONFIG pulse is written once with the raw level */
    fwss_pulse(&s->mw_pulse, in->master_warning_pb);
    fwss_pulse(&s->mc_pulse, in->master_caution_pb);
    fwss_pulse(&s->rcl_pulse, in->rcl_pb);
    fwss_pulse(&s->to_config_pulse, in->to_config_pb);
    s->to_config_test = (unsigned char)fwss_mtrig(&s->to_config_trigger, s->to_config_pulse.out, dt);
    s->to_config_test_held =
        (unsigned char)(fwss_mtrig(&s->to_config_held_1s5, in->to_config_pb, dt) || in->to_config_pb);
    fwss_pulse(&s->athr_disc_pressed_pulse, in->athr_instinctive_disc);
    fwss_pulse(&s->ap_disc_pressed_pulse, in->ap_instinctive_disc_count > 0);

    /* engine running is needed by the T.O memo before the core block */
    for (i = 0; i < 4; i++)
        s->eng_running[i] = (unsigned char)(in->eng_valid && in->eng_state[i] == FWS_ENG_ON);

    /* flight phases (FwsFlightPhases.update) */
    s->prev_flight_phase = s->flight_phase;
    fwss_update_flight_phase(s, dt);
    fwss_pulse(&s->phase_change_pulse, s->flight_phase != s->prev_flight_phase);
    fwss_update_to_memo(s, dt);
    fwss_update_ldg_memo(s, dt);
    fwss_update_altitude_warning(s);

    /* FwsCore signals */
    {
        int idle = 1;
        for (i = 0; i < 4; i++)
            if (in->tla_valid && in->tla_deg[i] > 2.6f)
                idle = 0;
        s->tla_idle_all_cache = (unsigned char)idle;
    }
    fwss_update_core(s, dt);
    if (s->mw_pulse.out) /* FwsCore:5478-5482 */
        s->mw_from_ap_off = 0;
    s->aural_cav_charge = (unsigned char)(s->aural_cav_charge && (s->ap_off_vol_mem.value || s->ap_off_invol_mem.value));

    /* memos */
    fwss_update_memos(s);

    /* alerts: simVarIsActive + activation times of every suppressable item */
    for (i = 0; i < FWSS_NUM_ALERTS; i++) {
        fwss_eval(s, i, &e);
        s->alert_active[i] = e.active;
        s->alert_aural[i] = e.aural;
        if (e.active && !s->sup_active[i])
            s->sup_since[i] = clock_s;
        s->sup_active[i] = e.active;
    }
    for (i = 0; i < FWSS_NUM_INOP; i++) {
        int sup = FWSS_SUP_INOP0 + i, a = fwss_inop_active(s, fwss_inop[i].id);
        if (a && !s->sup_active[sup])
            s->sup_since[sup] = clock_s;
        s->sup_active[sup] = (unsigned char)a;
    }
    for (i = 0; i < FWSS_NUM_LIMS; i++) {
        int sup = FWSS_SUP_LIM0 + i, a = fwss_lim_active(s, fwss_lims[i].id);
        if (a && !s->sup_active[sup])
            s->sup_since[sup] = clock_s;
        s->sup_active[sup] = (unsigned char)a;
    }

    /* default STS lists: every confirmed, not phase-inhibited procedure with its sensed checked state */
    {
        FwsActiveAlertRef *refs = s->scratch_refs;
        unsigned char (*checked)[FWS_MAX_ITEMS] = s->scratch_checked;
        int n = 0;
        for (i = 0; i < FWSS_NUM_ALERTS; i++) {
            if (fwss_alerts[i].deferred || fwss_alerts[i].non_procedure)
                continue;
            if (!fws_signals_alert_confirmed(s, i) || fwss_phase_in(s->flight_phase, fwss_alerts[i].phase_inhib))
                continue;
            fwss_eval(s, i, &e);
            memcpy(checked[n], e.items_checked, FWS_MAX_ITEMS);
            refs[n].id = fwss_alerts[i].id;
            refs[n].checked = checked[n];
            refs[n].n_checked = e.n_items;
            n++;
        }
        fws_signals_status_lists(s, refs, n, &s->lists);
    }

    /* SD page when unselected */
    fwss_update_sd(s, dt);
    s->initialized = 1;
}

/* ============================================================================================== */
/* accessors                                                                                     */
/* ============================================================================================== */
void fws_signals_outputs(const FwsSignals *s, FwsSignalOutputs *o)
{
    int i;
    memset(o, 0, sizeof *o);
    o->flight_phase = s->flight_phase;
    o->to_memo = s->to_memo;
    o->ldg_memo = s->ldg_memo;
    o->on_ground = s->aircraft_on_ground;
    o->show_takeoff_inhibit = s->show_takeoff_inhibit;
    o->show_landing_inhibit = s->show_landing_inhibit;
    o->to_config_normal = s->to_config_normal;
    o->to_config_test = s->to_config_test;
    for (i = 0; i < 4; i++) {
        o->normal_attention_getter_eng[i] = s->eng_start_or_crank[i];
        o->abnormal_primary_eng_attention_getter[i] = s->eng_primary_abnormal[i];
    }
    o->sd_page_when_unselected = s->sd_page_when_unselected;
    o->master_warning_from_ap_off = s->mw_from_ap_off;
    o->aural_cavalry_charge = s->aural_cav_charge;
    o->aural_cchord_once = s->aural_cchord_once;
    o->aural_cchord_cont = s->aural_cchord_cont;
    o->aural_v1 = s->aural_v1;
    o->stall_warning = s->stall_warning;
    o->land_asap = (unsigned char)fwss_lim_active(s, 1u);
    o->lg_lever_red_arrow = s->lg_red_arrow;
    o->checklist_reset_50min = s->checklist_reset_50min;
    o->autothrust_involuntary_pfd_memo = s->athr_invol_pfd_memo_mem.value;
    o->afs_alpha_floor = (unsigned char)(s->in.afs_a1a2 == 12);
    o->afs_toga_lk = (unsigned char)(s->in.afs_a1a2 == 13);
    o->afs_thr_lk = (unsigned char)(s->in.afs_a3 == FWS_A3_THR_LK);
}

int fws_signals_alert_count(void) { return FWSS_NUM_ALERTS; }

const FwsAlertDesc *fws_signals_alert_desc(int index)
{
    if (index < 0 || index >= FWSS_NUM_ALERTS)
        return NULL;
    return &fwss_alerts[index];
}

int fws_signals_alert_index(unsigned id) { return fwss_sup_index(id, 1); }

int fws_signals_alert_active(const FwsSignals *s, int index)
{
    if (index < 0 || index >= FWSS_NUM_ALERTS)
        return 0;
    return s->alert_active[index];
}

void fws_signals_alert_eval(const FwsSignals *s, int index, FwsAlertEval *out)
{
    if (index < 0 || index >= FWSS_NUM_ALERTS) {
        memset(out, 0, sizeof *out);
        return;
    }
    fwss_eval(s, index, out);
}

static int fwss_copy_ids(const unsigned *src, int n, unsigned *out, int max)
{
    int i;
    if (!out)
        return n;
    for (i = 0; i < n && i < max; i++)
        out[i] = src[i];
    return n < max ? n : max;
}

int fws_signals_memos_left(const FwsSignals *s, unsigned *out, int max)
{
    return fwss_copy_ids(s->memo_left, s->n_memo_left, out, max);
}

int fws_signals_memos_right(const FwsSignals *s, unsigned *out, int max)
{
    return fwss_copy_ids(s->memo_right, s->n_memo_right, out, max);
}

int fws_signals_memos_pfd(const FwsSignals *s, unsigned *out, int max)
{
    return fwss_copy_ids(s->pfd_memo, s->n_pfd_memo, out, max);
}

/* FwsNormalChecklists.sensedItems (FwsNormalChecklists.ts:537-612) */
int fws_signals_normal_sensed(const FwsSignals *s, unsigned proc_id, signed char *out, int max)
{
    const FwsInputs *in = &s->in;
    signed char v[9];
    int n = 0, i;
#define NUL (-1)
#define BOOL(x) ((signed char)((x) != 0))
    switch (proc_id) {
    case 1000001u: n = 4; v[0] = NUL; v[1] = NUL; v[2] = BOOL(in->seat_belts_on); v[3] = NUL; break;
    case 1000002u: n = 3; v[0] = NUL; v[1] = NUL; v[2] = BOOL(in->beacon_on); break;
    case 1000003u:
        /* [DEV 12] FBW compares the signed trim (rudderTrimPosition < 0.35); |trim| is used */
        n = 3; v[0] = NUL; v[1] = NUL; v[2] = BOOL(!in->rudder_trim_valid || fabs(in->rudder_trim_deg) < 0.35);
        break;
    case 1000004u:
        n = 9;
        v[0] = v[1] = v[2] = NUL;
        v[3] = 0;
        v[4] = BOOL(in->seat_belts_on);
        v[5] = BOOL(in->gnd_splr_armed);
        v[6] = BOOL(in->flaps_handle_index >= 1 && in->flaps_handle_index <= 3);
        v[7] = BOOL(in->autobrake_mode == 6);
        v[8] = BOOL(s->to_config_normal);
        break;
    case 1000005u: n = 2; v[0] = v[1] = NUL; break;
    case 1000006u: n = 4; v[0] = v[1] = v[2] = v[3] = NUL; break;
    case 1000007u: case 1000008u: case 1000009u: case 1000011u: n = 1; v[0] = NUL; break;
    case 1000010u: n = 4; v[0] = NUL; v[1] = BOOL(in->seat_belts_on); v[2] = v[3] = NUL; break;
    case 1000012u: {
        int conf3 = in->fms_valid && in->fms_landing_conf3;
        n = 5;
        v[0] = 0;
        v[1] = BOOL(in->seat_belts_on);
        v[2] = BOOL(s->all_gear_downlocked);
        v[3] = BOOL(in->gnd_splr_armed);
        v[4] = BOOL((!conf3 && in->flaps_handle_index == 4) || (conf3 && in->flaps_handle_index == 3));
        break;
    }
    case 1000013u:
        n = 4;
        v[0] = NUL;
        v[1] = BOOL(!in->eng_master_on[0] && !in->eng_master_on[1] && !in->eng_master_on[2] && !in->eng_master_on[3]);
        v[2] = NUL;
        v[3] = BOOL(s->all_fuel_pumps_off);
        break;
    case 1000014u: n = 4; v[0] = BOOL(in->crew_oxy_off); v[1] = BOOL(in->emer_exit_light_off); v[2] = v[3] = NUL; break;
    default:
        return -1;
    }
#undef NUL
#undef BOOL
    for (i = 0; i < n && i < max; i++)
        out[i] = v[i];
    return n;
}

#undef B
#undef PH_CAUT
#undef PH_LVL1
#undef PH_56
#undef PH_TO_CFG
#undef PH_TO_SPD
#undef PH_HYD
#undef PH_EMP
#undef PH_FEED
#undef PH_DOOR
#undef PH_LG
#undef PH_AIRBORNE
#undef PH_ALL_BUT_8
