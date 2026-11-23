"""Contract of the cockpit-feedback pass: FBW-authored controls that had no motion, click target or lamp.

One table drives both sides:
  * tools/cockpit_feedback.py wraps the visible FBW meshes in the panel OBJs with the authored clip,
    appends click targets and lamp quads to the cockpit controls OBJ, and lights the lamp faces that
    live in the emissive decal object;
  * native/avionics/a380x_cockpit_feedback.inc publishes the datarefs and commands
    (a380x_cockpit_feedback_table.h is generated from here: python3 tools/cockpit_feedback_contract.py --emit-c).

Sources (FlyByWire A380X, GPL-3.0): node and clip names from A380_COCKPIT_LOD00.gltf; LVAR meanings from
fbw-common/src/systems/shared/src/publishers/SwitchingPanelPublisher.ts (A32NX_ATT_HDG / AIR_DATA / FMS
_SWITCHING_KNOB, 0 CAPT / NORM / F/O), the MFD SURV page contract (a380x_mfd_page_surv.h) and the ECAM
control panel commands (a380x_sd_adapter.inc).

Kinds:
  press     momentary key: <dataref> travels in on command Begin and back on End (A380XPress). The
            commands already exist (KCCU, ECAM) or are created by the native module (SURV).
  selector  rotary selector with N detents: int <dataref> 0..N-1, commands <dataref>_left / _right.
  switch    toggle with N positions, 0 = bottom: int <dataref>, commands <dataref>_up / _down.
  follow    animation only, driven by an existing dataref; a knob (commands up / down) may add a click,
            `commands` a button click.
  static    lamps only (a fixed light face, no motion).
  toggle    two-position part (guards, circuit breakers, laptop switch, fire pushbuttons, furniture): float
            <dataref> 0..1 eased over `travel` seconds, command <dataref>_toggle; the whole clip is 0 -> 1.
  rotary    knob without detent logic of its own: float <dataref> 0..1 over the whole clip, commands
            <dataref>_up / _down (or hooked `commands` (up, down)) move it by `step`, eased at 1/`travel`
            per second (0: at once); `wrap` turns it endlessly (KCCU wheel).

Press commands are hooked (another module owns them) unless `own`, then this module creates them.
A clip that animates several nodes (`also`, or `all_nodes` for every node it targets) moves each one; a
node below another animated node (seat cushion on the seat, keys on the keyboard tray) composes both.

Extra motions (`extra`, e.g. the RMP volume knobs: push in / pull out, then rotation) are applied inside
the main one, each keyed 0..1 on its own dataref. `knob` gives the body a rotary click target, `push_cap`
a push target over the front of the cap.
"""
from __future__ import annotations

from dataclasses import dataclass, field

PREFIX = "a380x/xp12/"
GREEN_CELL = 16          # solid green FCU annunciator cell of cockpit_3d/fcu_labels*.png
OFF_WHITE_CELL = 48      # "OFF" white (ADIRS / ELEC cell, tools/ovhd_lamps_contract.py)
ON_WHITE_CELL = 0        # "ON" white


@dataclass(frozen=True)
class Lamp:
    face: str            # FBW node of the lamp face (a *_SEQ1 / *_SEQ2 child)
    dataref: str         # 0..1 light level, 0 = hidden
    cell: int | None     # fcu_labels atlas cell for a lit quad in the controls OBJ; None: the face is in
                         # the emissive decal object and is lit in place with its own LIT art
    rule: str = ""       # generic lamp rule (LAMP_RULES), computed by the native module; "" = own logic


# Generic lamp rules (a380x_cockpit_feedback.inc cfb_lamp_rule): lit when the condition holds or ANN LT
# is in TEST, with DC power; 0.3 with ANN LT DIM (FBW: (code || ANN TEST) && powered, DIM 0.1-0.3).
#   efis:<side>:<field>   EFIS option key (a380x_efis.h field), FBW INDICATOR_CODE ..._LIGHT_ON
#   eng_fire:<n>          ENG MASTER FIRE: X-Plane engine fire annunciator
#   eng_fault:<n>         ENG MASTER FAULT: start aborted (a380x_eng_start.inc) or X-Plane engine failure
#   door_open             CKPT DOOR OPEN: cockpit door not locked
#   test                  ANN LT TEST only (no system behind the legend here)
#   pressed:<dataref>     while the key is held (press travel > 0.5)
#   latch_off:<dataref>   latched pushbutton released (FBW_Push_Toggle OFF)
#   latch_on:<dataref>    latched pushbutton pressed in (ON / OVRD)
LAMP_RULES = ("efis", "eng_fire", "eng_fault", "door_open", "test", "pressed", "latch_off", "latch_on")
EFIS_FIELDS = ("cstr", "wpt", "vord", "ndb", "arpt", "vor1", "vor2", "terr", "traf", "ls", "vv",
               "wx_requested", "taxi_requested")
# Atlas cells this pass paints into cockpit_3d/fcu_labels*.png (unused cells; 14 is still free):
# key -> (text, RGBA, cell)
EXTRA_LEGEND_CELLS = {"ovrd_blue": ("OVRD", (80, 175, 255, 255), 15),
                      "fire_red": ("FIRE", (255, 55, 45, 255), 63)}
FAULT_AMBER_CELL, FAULT_WHITE_CELL, OFF_CELL = 47, 12, 48


@dataclass(frozen=True)
class Control:
    id: str              # unique, used in OBJ markers
    kind: str            # press | selector | switch | follow
    node: str            # animated FBW node (its subtree moves)
    clip: str            # FBW animation name
    dataref: str         # animation driver (full name)
    commands: tuple[str, ...] = ()   # press: commands whose phases drive the key
    label: str = ""      # hover label
    positions: int = 2   # selector / switch detents
    default: int = 0
    lamps: tuple[Lamp, ...] = ()
    click: bool = True   # add a click target
    minimum: float = 0.0  # follow: dataref value at the first clip key
    maximum: float = 1.0  # follow: dataref value at the last clip key
    files: tuple[str, ...] = ()      # restrict the panel patch to these OBJ names (default: all)
    group: str = ""
    extra: tuple[tuple, ...] = ()             # (clip, dataref[, min, max]) inside the main motion, keyed 0..1
                                              # (or min..max)
    knob: tuple[str, str] | None = None       # (increase, decrease) commands: rotary click target
    push_cap: str | None = None               # command of a push target over the cap
    also: tuple[str, ...] = ()                # more nodes moved by the same clip(s)
    all_nodes: bool = False                   # every node the clip(s) animate
    own: bool = False                         # press: the module creates the commands
    travel: float = 0.0                       # toggle: seconds end to end; rotary: easing (0 = at once)
    step: float = 0.1                         # rotary step per command
    initial: float = 0.0                      # toggle / rotary start value
    wrap: bool = False                        # rotary turns endlessly
    click_node: str | None = None             # node whose subtree is the click box (default: node)
    manip: str = ""                           # rotary click: "" knob, "up_down" switch up / down
    latch: int = -1                           # press: also a latched int <dataref without _pressed>
                                              # toggled by the command (initial value; -1 = none)
    patch: bool = True                        # wrap the panel geometry (False: lamps / clicks only)


def _ecam():
    # FBW node suffix -> (SD command, page index for the selection light or None, hover label)
    keys = (
        ("APU", "apu", 1, "APU"), ("BLEED", "bleed", 2, "BLEED"), ("COND", "cond", 3, "COND"),
        ("PRESS", "press", 4, "CAB PRESS"), ("DOOR", "door", 5, "DOOR / OXY"), ("ELAC", "elec_ac", 6, "ELEC AC"),
        ("ELDC", "elec_dc", 7, "ELEC DC"), ("FUEL", "fuel", 8, "FUEL"), ("WHEEL", "wheel", 9, "WHEEL"),
        ("HYD", "hyd", 10, "HYD"), ("FCTL", "fctl", 11, "F/CTL"), ("CB", "cb", 12, "C/B"),
        ("STS", "status", 14, "STATUS"), ("VIDEO", "video", 15, "VIDEO"),
        ("ALL", "all", None, "ALL - cycle the system pages while held"), ("MORE", "more", None, "MORE"),
        ("TOCONF", "to_config", None, "T.O CONFIG - test"), ("CLR_LH", "clear_left", None, "CLEAR"),
        ("CLR_RH", "clear_right", None, "CLEAR"), ("RCL", "recall", None, "RCL"),
        ("RCLLAST", "recall_last", None, "RCL LAST"), ("CL", "checklist", None, "C/L - normal checklist"),
        ("ABNPROC", "abnormal_procedures", None, "ABN PROC - abnormal procedures"),
        ("UP", "up", None, "C/L cursor up"), ("DOWN", "down", None, "C/L cursor down"),
        ("CHECK_LH", "check_left", None, "C/L check"), ("CHECK_RH", "check_right", None, "C/L check"),
        ("EMERCANC", "emergency_cancel", None, "EMER CANC"),
    )
    rows = []
    for suffix, command, page, label in keys:
        key = suffix.lower()
        lamps = ()
        if page is not None or suffix.startswith("CLR"):
            lamps = (Lamp(f"PUSH_ECAM_{suffix}_SEQ2", f"{PREFIX}ecam/buttons/{key}/selected", GREEN_CELL),)
        rows.append(Control(f"ecam_{key}", "press", f"PUSH_ECAM_{suffix}", f"PUSH_ECAM_{suffix}",
                            f"{PREFIX}ecam/buttons/{key}/press_ratio", (f"{PREFIX}ecam/sd/{command}",),
                            f"ECAM {label}", lamps=lamps, group="ecam"))
    return rows


ECAM_PAGE_LAMPS = {  # lamp dataref -> SD page index (a380x_sd.h) or -1 for the CLR keys
    f"{PREFIX}ecam/buttons/{key}/selected": page for key, page in (
        ("apu", 1), ("bleed", 2), ("cond", 3), ("press", 4), ("door", 5), ("elac", 6), ("eldc", 7),
        ("fuel", 8), ("wheel", 9), ("hyd", 10), ("fctl", 11), ("cb", 12), ("sts", 14), ("video", 15),
        ("clr_lh", -1), ("clr_rh", -1))
}


def _surv():
    p = PREFIX + "surv/"
    rows = (
        ("xpdr_tcas_sys1", "PUSH_SURV_XPDR_TCAS_SYS1", "XPDR / TCAS SYS 1", "PUSH_SURV_XPDR_TCAS_SYS1_SEQ1", None),
        ("xpdr_tcas_sys2", "PUSH_SURV_XPDR_TCAS_SYS2", "XPDR / TCAS SYS 2", "PUSH_SURV_XPDR_TCAS_SYS2_SEQ1", None),
        ("wxr_taws_sys1", "PUSH_SURV_WXR_TAWS_SYS1", "WXR / TAWS SYS 1 - select / off", "PUSH_SURV_WXR_TAWS_SYS1_SEQ1", None),
        ("wxr_taws_sys2", "PUSH_SURV_WXR_TAWS_SYS2", "WXR / TAWS SYS 2 - select / off", "PUSH_SURV_WXR_TAWS_SYS2_SEQ1", None),
        ("tcas_abv", "PUSH_SURV_TCAS_ABV", "TCAS ABV - above / norm", "PUSH_SURV_TCAS_ABV_SEQ1", None),
        ("tcas_blw", "PUSH_SURV_TCAS_BLW", "TCAS BLW - below / norm", "PUSH_SURV_TCAS_BLW_SEQ1", None),
        ("tcas_ta_only", "PUSH_SURV_TCAS_TAONLY", "TCAS TA ONLY - TA / TA-RA", "PUSH_SURV_TCAS_TAONLY_SEQ1", ON_WHITE_CELL),
        ("gs_mode", "PUSH_SURV_GS_MODE", "G/S MODE - glideslope alert off / on", "PUSH_SURV_GS_MODE_SEQ1", OFF_WHITE_CELL),
    )
    return [Control(f"surv_{key}", "press", node, node, f"{p}{key}_pressed", (f"{p}{key}",), label,
                    lamps=(Lamp(face, f"{p}lamp/{key}", cell),), group="surv")
            for key, node, label, face, cell in rows]


SURV_BUTTONS = ("xpdr_tcas_sys1", "xpdr_tcas_sys2", "wxr_taws_sys1", "wxr_taws_sys2",
                "tcas_abv", "tcas_blw", "tcas_ta_only", "gs_mode")


def _switching():
    p = PREFIX + "switching/"
    return [
        Control("switching_att_hdg", "selector", "KNOB_SWITCHING_1", "KNOB_SWITCHING_1", p + "att_hdg",
                label="SWITCHING ATT HDG - CAPT ON 3 / NORM / F/O ON 3", positions=3, default=1, group="switching"),
        Control("switching_air_data", "selector", "KNOB_SWITCHING_2", "KNOB_SWITCHING_2", p + "air_data",
                label="SWITCHING AIR DATA - CAPT ON 3 / NORM / F/O ON 3", positions=3, default=1, group="switching"),
        Control("switching_fms", "selector", "KNOB_SWITCHING_3", "KNOB_SWITCHING_3", p + "fms",
                label="SWITCHING FMS - BOTH ON 2 / NORM / BOTH ON 1", positions=3, default=1, group="switching"),
    ]


KCCU_NODE_KEYS = (*"ABCDEFGHIJKLMNOPQRSTUVWXYZ", *"0123456789", "DOT", "PLUSMINUS", "ENT", "BACKSPACE", "SP",
                  "SLASH", "ESC", "ESC2", "UP", "DOWN", "LEFT", "RIGHT", "DIR", "PERF", "INIT", "NAVAID",
                  "MAILBOX", "FPLN", "DEST", "SECINDEX", "SURV", "ATCCOM", "ND", "KBD", "CLRINFO", "REWIND",
                  "FORWARD")


def _eng_start():
    # ENG START selector: 0 CRANK (FBW rest pose), 1 NORM, 2 IGN START; the start logic is a380x_eng_start.inc
    return [Control("eng_start_selector", "selector", "ENG_START", "KNOB_ENGINES_MODE", f"{PREFIX}eng_start/selector",
                    label="ENG START - CRANK / NORM / IGN START", positions=3, default=1, group="eng_start")]


def _kccu():
    rows = []
    for letter, side in (("L", "captain"), ("R", "first_officer")):
        for key in KCCU_NODE_KEYS:
            name = key.lower()
            rows.append(Control(f"kccu_{side}_{name}", "press", f"PUSH_KCCU{letter}_{key}",
                                f"PUSH_KCCU{letter}_{key}_ANIM", f"{PREFIX}kccu/{side}/{name}_pressed",
                                (f"{PREFIX}kccu/{side}/{name}",), click=False, group="kccu"))
    return rows


def _eng_master():
    # The lever bodies already follow mixture_ratio (generic export); their FBW child decals (engine
    # number, FIRE / FAULT faces) were baked at the OFF rest pose and floated there while the lever is ON.
    # FIRE (SEQ1) and FAULT (SEQ2) lights on the lever (FBW_Airbus_Engine_Lights) ride with it too.
    p = PREFIX + "eng_master/"
    return [Control(f"eng{n}_master_decals", "follow", f"SWITCH_ENGINES_ENG{n}", f"SWITCH_ENGINES_ENG{n}",
                    f"sim/cockpit2/engine/actuators/mixture_ratio[{n - 1}]", click=False,
                    files=("decal_a380x_cockpit_decal_lovis.obj", "decal_new_decal_noemis.obj"), group="eng_master",
                    lamps=(Lamp(f"PUSH_ENGINES_{n}_SEQ1", f"{p}lamp/fire{n}", EXTRA_LEGEND_CELLS["fire_red"][2],
                                f"eng_fire:{n - 1}"),
                           Lamp(f"PUSH_ENGINES_{n}_SEQ2", f"{p}lamp/fault{n}", FAULT_AMBER_CELL, f"eng_fault:{n - 1}")))
            for n in (1, 2, 3, 4)]


def _glareshield_side():
    rows = []
    for short, side, caption in (("CS", "captain", "CAPT"), ("FO", "first_officer", "F/O")):
        rows.append(Control(f"{side}_console_floor_lt", "switch", f"SWITCH_GLARESHIELD_{short}_CONSOLE_FLOOR_LT",
                            f"SWITCH_GLARESHIELD_{short}_CONSOLE_FLOOR_LT", f"{PREFIX}lighting/{side}_console_floor",
                            label=f"{caption} CONSOLE & FLOOR LT - OFF / DIM / BRT", positions=3, default=0,
                            group="glareshield_side"))
        rows.append(Control(f"{side}_oit_side", "switch", f"SWITCH_GLARESHIELD_{short}_OIT_SIDE",
                            f"SWITCH_GLARESHIELD_{short}_OIT_SIDE", f"{PREFIX}oit/{side}_side",
                            label=f"{caption} OIT SIDE - FLT OPS / NSS AVNCS", positions=2, default=1,
                            group="glareshield_side"))
    return rows


def _overhead():
    # Overhead pushbuttons with a latched position in the lamp module (tools/ovhd_lamps_contract.py
    # pb_state_ids): the same-named command toggles it; here the cap travels and gets a click target.
    from ovhd_lamps_contract import pb_state_ids, row
    rows = []
    for pid in pb_state_ids():
        _pid, caption, node, *_rest = row(pid)
        if node in OVERHEAD_WITHOUT_CLIP:
            continue
        rows.append(Control(f"ovhd_{pid}", "press", node, node, f"{PREFIX}ovhd/pb/{pid}_pressed",
                            (f"{PREFIX}ovhd/pb/{pid}",), f"{caption} - pushbutton", group="overhead"))
    return rows


OVERHEAD_WITHOUT_CLIP = ("PUSH_OVHD_SIGNS",)

# RMP keys: FBW node suffix -> command key (a380x_rmp_adapter.inc rmp_key_names, same order there).
RMP_KEYS = (("0", "0"), ("1", "1"), ("2", "2"), ("3", "3"), ("4", "4"), ("5", "5"), ("6", "6"), ("7", "7"),
            ("8", "8"), ("9", "9"), ("DOT", "dot"), ("CLR", "clr"), ("UP", "up"), ("DOWN", "down"),
            ("ADK1", "adk1"), ("ADK2", "adk2"), ("ADK3", "adk3"), ("LSK1", "lsk1"), ("LSK2", "lsk2"),
            ("LSK3", "lsk3"), ("VHF", "vhf"), ("HF", "hf"), ("TEL", "tel"), ("SQWK", "sqwk"), ("MENU", "menu"),
            ("NAV", "nav"), ("MSG_CLR", "msg_clr"), ("RST", "rst"), ("VHF1", "vhf1_call"), ("VHF2", "vhf2_call"),
            ("VHF3", "vhf3_call"), ("HF1", "hf1_call"), ("HF2", "hf2_call"), ("TEL1", "tel1_call"),
            ("TEL2", "tel2_call"), ("INT", "int_call"), ("CAB", "cab_call"), ("PA", "pa_call"),
            ("VOICE", "voice_call"), ("STBY_NAV", "stby_nav"), ("BLANK", "blank"), ("BLANK2", "blank2"),
            ("DUMMY2", "dummy2"))
RMP_KEY_LABELS = {"adk1": "ADK 1 - select row / swap", "adk2": "ADK 2 - select row / swap",
                  "adk3": "ADK 3 - select row / swap (SQWK: IDENT)", "lsk1": "LSK 1 - select row / enter STBY",
                  "lsk2": "LSK 2 - select row / enter STBY", "lsk3": "LSK 3 - select row / enter STBY",
                  "clr": "CLR (hold: clear entry)", "msg_clr": "MSG CLR", "up": "UP - STBY mode",
                  "down": "DOWN - STBY mode", "vhf1_call": "VHF 1 transmit", "vhf2_call": "VHF 2 transmit",
                  "vhf3_call": "VHF 3 transmit"}
# Reception knobs: FBW knob suffix -> channel (push: receive on / off, turn: volume)
RMP_KNOBS = (("VHF1", "vhf1"), ("VHF2", "vhf2"), ("VHF3", "vhf3"), ("HF1", "hf1"), ("HF2", "hf2"), ("TEL1", "tel1"),
             ("TEL2", "tel2"), ("INT", "int"), ("CAB", "cab"), ("PA", "pa"), ("VOICE", "voice"), ("DUMMY", "dummy"))
RMP_CALL_LAMPS = ("vhf1_call", "vhf2_call", "vhf3_call")


def rmp_commands(n: int) -> tuple[str, ...]:
    p = f"{PREFIX}rmp/{n}/"
    return (*(p + key for _node, key in RMP_KEYS),
            *(p + f"{ch}_knob_{act}" for _node, ch in RMP_KNOBS for act in ("push", "up", "down")),
            p + "brightness_up", p + "brightness_down")


def rmp_datarefs(n: int) -> tuple[str, ...]:
    p = f"{PREFIX}rmp/{n}/"
    return (*(p + f"{ch}_{what}" for _node, ch in RMP_KNOBS for what in ("rx", "volume")),
            p + "brightness", p + "state", p + "page", p + "standby_led", p + "device_active",
            *(p + f"lamp/{key}" for key in RMP_CALL_LAMPS))


def _rmp():
    rows = []
    for n in (1, 2, 3):
        p = f"{PREFIX}rmp/{n}/"
        for suffix, key in RMP_KEYS:
            lamps = ()
            if key in RMP_CALL_LAMPS:
                lamps = (Lamp(f"PUSH_RMP_{n}_{suffix}_SEQ2", f"{p}lamp/{key}", GREEN_CELL),)
            label = RMP_KEY_LABELS.get(key, key.replace("_", " ").upper())
            rows.append(Control(f"rmp{n}_{key}", "press", f"PUSH_RMP_{n}_{suffix}", f"PUSH_RMP_{n}_{suffix}",
                                f"{p}{key}_pressed", (p + key,), f"RMP {n} {label}", lamps=lamps, group="rmp"))
        for suffix, ch in RMP_KNOBS:
            node = f"KNOB_RMP_{n}_{suffix}"
            rows.append(Control(f"rmp{n}_{ch}_knob", "follow", node, f"{node}_PUSH_ANIM", f"{p}{ch}_rx",
                                label=f"RMP {n} {ch.upper()} - push: reception on / off, turn: volume",
                                extra=((f"{node}_ROTATE_ANIM", f"{p}{ch}_volume"),),
                                knob=(f"{p}{ch}_knob_up", f"{p}{ch}_knob_down"), push_cap=f"{p}{ch}_knob_push",
                                also=(f"{node}_SEQ1",), group="rmp"))
        rows.append(Control(f"rmp{n}_brightness", "follow", f"KNOB_RMP_{n}_OFFBRT", f"KNOB_RMP_{n}_OFFBRT",
                            f"{p}brightness", label=f"RMP {n} OFF / BRT",
                            knob=(f"{p}brightness_up", f"{p}brightness_down"), group="rmp"))
        rows.append(Control(f"rmp{n}_nav_sel", "selector", f"KNOB_RMP_{n}_NAV_SEL", f"KNOB_RMP_{n}_NAV_SEL",
                            f"{p}nav_sel", label=f"RMP {n} NAV audio - ADF 1 / ADF 2 / LS / VOR 1 / VOR 2 / MKR",
                            positions=6, default=0, group="rmp"))
        rows.append(Control(f"rmp{n}_int_rad", "switch", f"SWITCH_RMP_{n}_INTRAD", f"SWITCH_RMP_{n}_INTRAD",
                            f"{p}int_rad", label=f"RMP {n} INT / RAD", positions=2, default=0, group="rmp"))
        rows.append(Control(f"rmp{n}_stby_nav_cover", "switch", f"COVER_RMP_{n}_STBY_NAV", f"COVER_RMP_{n}_STBY_NAV",
                            f"{p}stby_nav_cover", label=f"RMP {n} STBY RAD NAV guard - open / close",
                            positions=2, default=0, group="rmp"))
        rows.append(Control(f"rmp{n}_standby_led", "static", f"LIGHT_RMP_{n}_LED_GREEN", "", f"{p}standby_led",
                            lamps=(Lamp(f"LIGHT_RMP_{n}_LED_GREEN", f"{p}standby_led", GREEN_CELL),), click=False,
                            group="rmp"))
    return rows


def _glareshield():
    """Glareshield and sidestick keys of the glareshield module (its commands, hooked; the click targets
    are in config/cockpit_controls.json) and the AUTOLAND lights (test press only)."""
    g = PREFIX + "glareshield/"
    rows = []
    for short, side, caption in (("CS", "left", "CAPT"), ("FO", "right", "F/O")):
        for key, command in (("CHRONO", "chrono"), ("SIDESTICK", "takeover"), ("MASTERCAUT", "master_caution"),
                             ("MASTERWARN", "master_warning"), ("ATCMSG", "atc_ack"), ("PTT", "ptt")):
            node = f"PUSH_GLARESHIELD_{short}_{key}"
            rows.append(Control(f"glareshield_{command}_{side}", "press", node, node, f"{g}{command}_{side}_pressed",
                                (f"{g}{command}_{side}",), click=False, group="glareshield"))
        clip = "PUSH_GLARESHIELD_CS_AUTOLAND" if short == "CS" else "PUSH_AUTOPILOT_MASTERCAUT_R"
        rows.append(Control(f"glareshield_autoland_{side}", "press", f"PUSH_GLARESHIELD_{short}_AUTOLAND", clip,
                            f"{g}autoland_{side}_pressed", (f"{g}autoland_{side}",), f"{caption} AUTOLAND light",
                            own=True, group="glareshield"))
        stick = "LEFT" if short == "CS" else "RIGHT"
        rows.append(Control(f"sidestick_takeover_{side}", "press", f"PUSH_YOKE_{stick}", f"PUSH_YOKE_{stick}",
                            f"{g}stick_takeover_{side}_pressed", (f"{g}takeover_{side}",),
                            f"{caption} SIDESTICK takeover pushbutton", group="glareshield"))
        rows.append(Control(f"{side}_loudspeaker", "follow", f"KNOB_GLARESHIELD_{short}_SPEAKER",
                            f"KNOB_GLARESHIELD_{short}_SPEAKER", f"{g}volume_{side}", click=False, minimum=0,
                            maximum=100, group="glareshield"))
    return rows


def _sidesticks():
    # roll 0 = full left, pitch 0 = full forward (FBW keys 0..20, centre 10); the F/O stick follows the
    # copilot axes, so it stays centred unless a second set of hardware is bound.
    rows = []
    for stick, side, suffix in (("LEFT", "captain", ""), ("RIGHT", "first_officer", "_copilot")):
        rows.append(Control(f"sidestick_{side}", "follow", f"YOKE_{stick}_DUMMY1", "yoke_lever_stick1_l_r",
                            f"sim/joystick/yoke_roll_ratio{suffix}", click=False, minimum=-1, maximum=1,
                            extra=(("yoke_lever_stick_fore1_aft", f"sim/joystick/yoke_pitch_ratio{suffix}", -1, 1),),
                            group="sidestick"))
        rows.append(Control(f"tiller_handle_{side}", "toggle", f"HANDLE_{stick}_YOKE", f"HANDLE_{stick}_YOKE",
                            f"{PREFIX}cockpit/{side}_side_handle", label="Side handle - fold / unfold",
                            travel=0.6, group="sidestick"))
    return rows


EFIS_KEYS = ("PFD", "RECONF", "BLANK")


EFIS_OPTION_KEYS = (("CSTR", "cstr"), ("WPT", "wpt"), ("VORD", "vord"), ("NDB", "ndb"), ("ARPT", "arpt"),
                    ("VOR", "vor1"), ("ADF", "vor2"), ("WX", "wx_requested"), ("TERR", "terr"), ("TRAF", "traf"),
                    ("TAXI", "taxi_requested"))
EFIS_PRESSED = {"vor1": "vor1_pressed", "vor2": "vor2_pressed", "wx_requested": "wx_pressed",
                "taxi_requested": "taxi_pressed"}


def _efis():
    """EFIS option keys: green bar while the option is on (FBW efis-cp.xml INDICATOR_CODE; VOR / ADF are
    the VOR 1 / VOR 2 needle keys). The keys already travel (EFIS module); the lit quad follows that."""
    rows = []
    for short, side in (("CS", "captain"), ("FO", "first_officer")):
        for key, field in EFIS_OPTION_KEYS:
            node = f"PUSH_EFIS_{short}_{key}"
            pressed = EFIS_PRESSED.get(field, field + "_pressed")
            rows.append(Control(f"efis_{side}_{key.lower()}_bar", "follow", node, node,
                                f"{PREFIX}efis/{side}/{pressed}", click=False, patch=False, group="efis",
                                lamps=(Lamp(f"{node}_SEQ2", f"{PREFIX}efis/{side}/lamp/{key.lower()}", GREEN_CELL,
                                            f"efis:{0 if side == 'captain' else 1}:{field}"),)))
    for short, side, caption in (("CS", "captain", "CAPT"), ("FO", "first_officer", "F/O")):
        for key in EFIS_KEYS:
            node = f"PUSH_EFIS_{short}_{key}"
            rows.append(Control(f"efis_{side}_{key.lower()}", "press", node, node,
                                f"{PREFIX}efis/{side}/{key.lower()}_pb_pressed", (f"{PREFIX}efis/{side}/{key.lower()}_pb",),
                                f"{caption} EFIS {key}", own=True, group="efis"))
        rows.append(Control(f"efis_{side}_oit_brightness", "rotary", f"KNOB_EFIS_{short}_OIT", f"KNOB_EFIS_{short}_OIT",
                            f"{PREFIX}oit/{side}_brightness", label=f"{caption} OIT brightness", step=0.05,
                            initial=0.75, group="efis"))
    return rows


def _kccu_extra():
    rows = []
    for letter, side, caption in (("L", "captain", "CAPT"), ("R", "first_officer", "F/O")):
        p = f"{PREFIX}kccu/{side}/"
        for n in range(1, 7):
            node = f"PUSH_KCCU{letter}_BLANK{n}"
            rows.append(Control(f"kccu_{side}_blank{n}", "press", node, node if n == 1 else node + "_ANIM",
                                f"{p}blank{n}_pressed", (f"{p}blank{n}",), f"{caption} KCCU (blank key)", own=True,
                                group="kccu"))
        for what in ("kbd", "ccd"):
            node = f"PUSH_KCCU{letter}_{what.upper()}_ON_OFF"
            rows.append(Control(f"kccu_{side}_{what}_switch", "follow", node, node + "_ANIM", f"{p}{what}_on",
                                click=False, group="kccu"))
    rows.append(Control("kccu_captain_wheel", "rotary", "PUSH_KCCUL_WHEEL", "PUSH_KCCUL_WHEEL_ANIM",
                        f"{PREFIX}kccu/captain/wheel_angle", (f"{PREFIX}kccu/captain/wheel_up",
                                                              f"{PREFIX}kccu/captain/wheel_down"),
                        click=False, step=0.1, wrap=True, group="kccu"))
    return rows


OIT_KEYS = ("0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "ALT_LH", "ALT_RH", "ARROW_DOWN", "ARROW_LEFT",
            "ARROW_RIGHT", "ARROW_UP", "B", "BACKSLASH", "BACKSPACE", "BLANK1", "BLANK2", "BLANK3", "BRACE_CLOSE",
            "BRACE_OPEN", "C", "CAPS", "CAPSLOCK", "COMMA", "CTRL_LH", "CTRL_RH", "D", "DEL", "E", "ELLIPSIS",
            "ENTER", "ESC", "F", "F1", "F10", "F11", "F12", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9",
            "FULLSTOP", "G", "H", "I", "INS", "J", "K", "L", "LH_MOUSE", "M", "MINUS", "N", "O", "OIS_MENU",
            "OIS_XFR", "P", "PLUS", "Q", "R", "RH_MOUSE", "S", "SEMICOLON", "SHIFT_LH", "SHIFT_RH", "SKIP_DOWN",
            "SKIP_UP", "SLASH", "SPACE_LH", "SPACE_RH", "SWUNG_DASH", "T", "TAB", "U", "V", "W", "X", "Y", "Z")
ACD_KEYS = ("OIS_XFR", "OIS_MENU", "ENTER", "ESC", "UP", "DOWN", "LEFT", "RIGHT")


def _oit():
    # OIT keyboard on the captain's tray (FBW models only that one) and the two ACD keypads: key travel.
    p = PREFIX + "oit/"
    rows = [Control(f"oit_kbd_{key.lower()}", "press", f"PUSH_KBD_CPT_{key}", f"PUSH_KBD_CPT_ANIM_{key}",
                    f"{p}captain/kbd/{key.lower()}_pressed", (f"{p}captain/kbd/{key.lower()}",),
                    f"CAPT OIT keyboard {key.replace('_', ' ')}", own=True, all_nodes=True, group="oit")
            for key in OIT_KEYS]
    for short, side, caption in (("CPT", "captain", "CAPT"), ("FO", "first_officer", "F/O")):
        rows += [Control(f"oit_acd_{side}_{key.lower()}", "press", f"PUSH_ACD_{short}_{key}", f"PUSH_ACD_{short}_{key}",
                         f"{p}{side}/acd/{key.lower()}_pressed", (f"{p}{side}/acd/{key.lower()}",),
                         f"{caption} OIT cursor control {key.replace('_', ' ')}", own=True, group="oit")
                 for key in ACD_KEYS]
    return rows


# momentary pushbuttons without a system here yet: (FBW node, clip, key, hover label); a hooked command
# replaces the key's own one
PANEL_KEYS = (
    ("PUSH_OVHD_VENTILATION_SMOKE", "PUSH_OVHD_VENTILATION_SMOKE", "ventilation_smoke", "VENTILATION SMOKE"),
    ("PUSH_OVHD_AP.035", "PUSH_OVHD_AP.035", "ovhd_pb_035", "pushbutton"),
    ("PUSH_OVHD_AP.037", "PUSH_OVHD_AP.037", "ovhd_pb_037", "pushbutton"),
    ("PUSH_OXYGEN_TWRRESET", "PUSH_OXYGEN_TWRRESET", "oxygen_tmr_reset", "OXYGEN TMR RESET"),
    ("PUSH_DOOR_LCKG_SYS", "PUSH_DOOR_LCKG_SYS", "door_locking_sys", "COCKPIT DOOR LOCKING SYS"),
    ("PUSH_OVHD_EMERELECPWR_RAT", "PUSH_OVHD_EMERELECPWR_RAT", "emer_elec_rat", "EMER ELEC PWR"),
    ("PUSH_OVHD_PA.001", "PUSH_OVHD_PA", "ovhd_pa", "PA"),
    ("PUSH_OVHD_OXYGEN_RATMANON", "PUSH_OVHD_OXYGEN_RATMANON", "oxygen_mask_man_on", "MASK MAN ON"),
    ("PUSH_OVHD_CALLS_MECH", "PUSH_OVHD_CALLS_MECH", "calls_mech", "CALLS MECH"),
    ("PUSH_OVHD_CALLS_DECK_UPPER", "PUSH_OVHD_CALLS_DECK_UPPER", "calls_upper_deck", "CALLS UPPER DECK"),
    ("PUSH_OVHD_CALLS_DECK_MAIN", "PUSH_OVHD_CALLS_DECK_MAIN", "calls_main_deck", "CALLS MAIN DECK"),
    ("PUSH_OVHD_CALLS_PURS", "PUSH_OVHD_CALLS_PURS", "calls_purs", "CALLS PURS"),
    ("PUSH_OVHD_CALLS_REST_FWD", "PUSH_OVHD_CALLS_REST_FWD", "calls_rest_fwd", "CALLS REST FWD"),
    ("PUSH_OVHD_CALLS_REST_MAIN", "PUSH_OVHD_CALLS_REST_MAIN", "calls_rest_main", "CALLS REST MAIN"),
    ("PUSH_OVHD_CALLS_ALL", "PUSH_OVHD_CALLS_ALL", "calls_all", "CALLS ALL"),
    ("PUSH_OVHD_EVAC_HORN_OFF", "PUSH_OVHD_EVAC_HORN_OFF", "evac_horn_off", "EVAC HORN SHUT OFF"),
    ("PUSH_GND_HF_DATA_LINK", "PUSH_GND_HF_DATA_LINK", "gnd_hf_datalink", "GND HF DATALINK"),
    ("PUSH_OVHD_MAINT_GND_CONNECTION", "PUSH_OVHD_MAINT_GND_CONNECTION", "maint_gnd_connection", "MAINT GND CONNECTION"),
    ("PUSH_OVHD_CARGOSMOKE_TEST", "PUSH_OVHD_CARGOSMOKE_TEST", "cargo_smoke_test", "CARGO SMOKE TEST"),
    ("PUSH_DFDR_EVENT", "ANIM_PUSH_DFDR_EVENT", "dfdr_event", "DFDR EVENT"),
    ("PUSH_ACMS_TRIGGER", "ANIM_PUSH_ACMS_TRIGGER", "acms_trigger", "ACMS TRIGGER"),
    ("PUSH_KNOB_CPT_WXR_VD_AZIM", "PUSH_KNOB_CPT_WXR_VD_AZIM", "captain_wxr_vd_azim_push", "CAPT WXR VD AZIM - push"),
    ("PUSH_KNOB_FO_WXR_VD_AZIM", "PUSH_KNOB_FO_WXR_VD_AZIM", "first_officer_wxr_vd_azim_push", "F/O WXR VD AZIM - push"),
)
HOOKED_KEYS = {"oxygen_crew": ("PUSH_OVHD_OXYGEN_CREW", "sim/oxy/crew_valve_toggle")}


# FBW_Push_Toggle / FBW_Covered_Push_Toggle keys of PANEL_KEYS: key -> (initial position, lamps)
LATCHED_KEYS = {"door_locking_sys": 1, "gnd_hf_datalink": 0}


def _panel_lamps(key: str, node: str, p: str) -> tuple:
    if key == "oxygen_tmr_reset":     # FAULT (white) / ON while held: FBW A32NX_OXYGEN_TMR_RESET(_FAULT)
        return (Lamp(f"{node}_SEQ1", f"{p}lamp/{key}_fault", FAULT_WHITE_CELL, "test"),
                Lamp(f"{node}_SEQ2", f"{p}lamp/{key}_on", ON_WHITE_CELL, f"pressed:{p}{key}_pressed"))
    if key == "door_locking_sys":     # OFF while released
        return (Lamp(f"{node}_SEQ2", f"{p}lamp/{key}_off", OFF_CELL, f"latch_off:{p}{key}"),)
    if key == "gnd_hf_datalink":      # OVRD (blue) while pressed in
        return (Lamp(f"{node}_SEQ2", f"{p}lamp/{key}_ovrd", EXTRA_LEGEND_CELLS["ovrd_blue"][2], f"latch_on:{p}{key}"),)
    return ()


def _panel_keys():
    p = PREFIX + "cockpit/"
    rows = [Control(f"key_{key}", "press", node, clip, f"{p}{key}_pressed", (f"{p}{key}",), f"{label} - pushbutton",
                    own=True, group="panel_keys", lamps=_panel_lamps(key, node, p), latch=LATCHED_KEYS.get(key, -1))
            for node, clip, key, label in PANEL_KEYS]
    # (CKPT DOOR lock status OPEN / FAULT: the pushbutton is not in the X-Plane objects, so no lamp)
    for key, (node, command) in HOOKED_KEYS.items():   # the CREW OXY SUPPLY click target exists already
        rows.append(Control(f"key_{key}", "press", node, node, f"{p}{key}_pressed", (command,), click=False,
                            group="panel_keys"))
    return rows


FIRE_PUSHBUTTONS = ("ENG1", "ENG2", "ENG3", "ENG4", "APU")


def _fire():
    # ENG / APU FIRE pushbuttons (released = 1, a380x/xp12/engine/fire_pushbutton[0..3] for the ECAM) under
    # their guards.
    rows = []
    for unit in FIRE_PUSHBUTTONS:
        key = unit.lower()
        rows.append(Control(f"fire_{key}_guard", "toggle", f"A380X_OVHD_{unit}_FIRE_GUARD",
                            f"A380X_OVHD_{unit}_FIRE_GUARD", f"{PREFIX}fire/{key}_guard",
                            label=f"{unit} FIRE pushbutton guard - open / close", all_nodes=True, travel=0.3,
                            group="fire"))
        rows.append(Control(f"fire_{key}_pb", "toggle", f"PUSH_OVHD_FIRE_{unit}", f"PUSH_OVHD_FIRE_{unit}",
                            f"{PREFIX}fire/{key}_pb", label=f"{unit} FIRE pushbutton - release / reset",
                            all_nodes=True, travel=0.15, group="fire"))
    return rows


GUARDS = ("LOCK_OVHD_CALLS_EMER", "LOCK_OVHD_CABINPRESS_DITCHING", "LOCK_OVHD_AIRCOND_RAMAIR", "LOCK_OVHD_CARGOSMOKE_FWD",
          "LOCK_OVHD_ENGMANSTART_1", "LOCK_OVHD_ENGMANSTART_2", "LOCK_OVHD_ENGMANSTART_3", "LOCK_OVHD_ENGMANSTART_4",
          "LOCK_OVHD_ENGMANSTARTALTN", "LOCK_OVHD_CARGOSMOKE_AFT", "LOCK_OVHD_FLTCTL_SEC1", "LOCK_OVHD_FLTCTL_SEC2",
          "LOCK_OVHD_FLTCTL_SEC3", "LOCK_OVHD_FLTCTL_PRIM1", "LOCK_OVHD_FLTCTL_PRIM2", "LOCK_OVHD_FLTCTL_PRIM3",
          "LOCK_OVHD_HYD_ENG1_AB", "LOCK_OVHD_HYD_ENG2_AB", "LOCK_OVHD_HYD_ENG3_AB", "LOCK_OVHD_HYD_ENG4_AB",
          "LOCK_OVHD_FUEL_OUTRTK_XFR", "LOCK_OVHD_FUEL_MIDTK_XFR", "LOCK_OVHD_FUEL_INRTK_XFR",
          "LOCK_OVHD_FUEL_TRIMTK_XFR", "LOCK_OVHD_ENG_FADEC1", "LOCK_OVHD_ENG_FADEC2", "LOCK_OVHD_ENG_FADEC3",
          "LOCK_OVHD_ENG_FADEC4", "LOCK_OVHD_HYD_GPUMPA", "LOCK_OVHD_HYD_GPUMPB", "LOCK_OVHD_HYD_YPUMPA",
          "LOCK_OVHD_HYD_YPUMPB", "LOCK_DOOR_LCKG_SYS", "LOCK_OVHD_NSSMASTER", "LOCK_OVHD_EVAC_COMMAND",
          "LOCK_OVHD_OXYGEN_RATMANON", "LOCK_OVHD_CABAIR_EXTRACT", "LOCK_OVHD_JETTISON_ACTIVE", "LOCK_OVHD_JETTISON_ARM",
          "LOCK_OVHD_EMER_OUTR_XFR", "LOCK_GND_HF_DATA_LINK", "LOCK_OVHD_MAINT_CONNECTION")
CIRCUIT_BREAKERS = ("AESU1", "AESU2", "AICU1", "AICU2", "ARPT_NAV", "ATC", "AVS1", "AVS2", "BSCS1", "BSCS2", "CIDS1",
                    "CIDS2", "CIDS3", "CPCS1", "CPCS2", "DSMS", "DTLNKROUTER", "EMPTY1", "EMPTY2", "EMPTY3", "EMPTY4",
                    "EMPTY5", "EMPTY6", "EMPTY7", "EMPTY8", "ENG1_EIPM2", "ENG2_EIPM1", "ENG3_EIPM2", "ENG4_EIPM1",
                    "ESS_TR", "FLAPS1", "FLAPS2", "FMC_A", "FMC_B", "FMC_C", "FQMS1", "FQMS2", "FWS1", "FWS2", "GCU",
                    "LGCIS1", "LGCIS2", "NSS_AVNCS", "NSS_FLT_OPS", "PACK1_CTL", "PACK2_CTL", "PAX_BBAND", "SCS1",
                    "SCS2", "SDF1", "SDF2", "SDF3", "SLAT2", "SLATS1", "TCS1", "TCS2", "TR1", "TR_2A", "VCS1", "VCS2")


def _guards_and_breakers():
    rows = []
    for node in GUARDS:
        key = node.replace("LOCK_OVHD_", "").replace("LOCK_", "").lower()
        rows.append(Control(f"guard_{key}", "toggle", node, node, f"{PREFIX}guard/{key}",
                            label=f"{key.replace('_', ' ').upper()} guard - open / close", travel=0.3, group="guard"))
    for name in CIRCUIT_BREAKERS:
        node = f"CB_{name}"
        rows.append(Control(f"cb_{name.lower()}", "toggle", node, node, f"{PREFIX}cb/{name.lower()}",
                            label=f"C/B {name.replace('_', ' ')} - pull / push", travel=0.1, group="cb"))
    for short, side, caption in (("CS", "captain", "CAPT"), ("FO", "first_officer", "F/O")):
        node = f"SWITCH_LAPTOP_{short}_POWER"
        rows.append(Control(f"laptop_{side}_power", "toggle", node, node, f"{PREFIX}cockpit/{side}_laptop_power",
                            label=f"{caption} laptop POWER", travel=0.1, group="cb"))
    return rows


# overhead knobs and switches without a system behind them here (state and motion only):
# (FBW node / clip, key, kind, label, positions or step, start)
OVERHEAD_KNOBS = (
    ("KNOB_OVHD_AIRCOND_PACKFLOW", "pack_flow", "rotary", "PACK FLOW - LO / NORM / HI", 1 / 3, 1 / 3),
    ("KNOB_OVHD_AIRCOND_COCKPIT", "cockpit_temp", "rotary", "CKPT temperature", 0.05, 0.5),
    ("KNOB_OVHD_AIRCOND_FWDCABIN", "fwd_cabin_temp", "rotary", "FWD CABIN temperature", 0.05, 0.5),
    ("KNOB_OVHD_AIRCOND_BULK", "bulk_temp", "rotary", "BULK CARGO temperature", 0.05, 0.5),
    ("KNOB_OVHD_AIRCOND_FWDCARGO", "fwd_cargo_temp", "rotary", "FWD CARGO temperature", 0.05, 0.5),
    ("KNOB_OVHD_CABINPRESS_LDGELEV", "ldg_elev", "rotary", "LDG ELEV - AUTO / manual", 0.05, 0.0),
    ("SWITCH_OVHD_CABINPRESS_MANVSCTL", "man_vs_ctl", "rotary", "MAN V/S CTL", 1 / 3, 0.0),
    ("KNOB_OVHD_READINGLTL", "reading_light_left", "rotary", "CAPT READING LT", 0.1, 0.0),
    ("KNOB_OVHD_READINGLTR", "reading_light_right", "rotary", "F/O READING LT", 0.1, 0.0),
    ("KNOB_OVHD_AIRCOND_XBLEED", "xbleed", "selector", "X BLEED - SHUT / AUTO / OPEN", 3, 1),
    ("SWITCH_OVHD_EXTLT_RWY", "rwy_turn_off", "switch", "RWY TURN OFF", 2, 0),
    ("SWITCH_OVHD_EXTLT_WING", "wing_lt", "switch", "WING", 2, 0),
    ("SWITCH_OVHD_EXTLT_LOGO", "logo_lt", "switch", "LOGO", 3, 0),
    ("SWITCH_OVHD_INTLT_STORM", "storm_lt", "switch", "STORM", 2, 0),
    ("SWITCH_OVHD_INTLT_EMEREXIT", "emer_exit_lt", "switch", "EMER EXIT LT - OFF / ARM / ON", 3, 1),
    ("SWITCH_OVHD_INTLT_STBYCOMPAS", "stby_compass_lt", "switch", "STBY COMPASS", 3, 0),
    ("SWITCH_OVHD_EVAC_CAPT", "evac_capt", "switch", "EVAC CAPT / CAPT & PURS", 3, 0),
    ("SWITCH_OVHD_ELT_SELECTOR", "elt", "switch", "ELT - TEST / ARM / ON", 3, 1),
    ("LIGHTING_Knob_Panel", "pedestal_panel_lt", "rotary", "Panel lighting", 0.05, 0.5),
    ("LIGHTING_Knob_Glareshield", "pedestal_glareshield_lt", "rotary", "Glareshield lighting", 0.05, 0.5),
    ("LIGHTING_Knob_Pedestal", "pedestal_flood_lt", "rotary", "Pedestal lighting", 0.05, 0.5),
    ("LIGHTING_Knob_Ambient", "pedestal_ambient_lt", "rotary", "Ambient lighting", 0.05, 0.5),
    ("KNOB_CPT_WXR_VD_AZIM", "captain_wxr_vd_azim", "rotary", "CAPT WXR VD AZIM", 0.25, 0.0),
    ("KNOB_FO_WXR_VD_AZIM", "first_officer_wxr_vd_azim", "rotary", "F/O WXR VD AZIM", 0.5, 0.0),
)


def _overhead_knobs():
    p = PREFIX + "cockpit/"
    rows = []
    for node, key, kind, label, size, start in OVERHEAD_KNOBS:
        if kind == "rotary":
            rows.append(Control(f"knob_{key}", kind, node, node, p + key, label=label, step=size, initial=start,
                                group="overhead_knobs"))
        else:
            rows.append(Control(f"knob_{key}", kind, node, node, p + key, label=label, positions=size, default=start,
                                group="overhead_knobs"))
    # the FWD CABIN clip of the FBW model also turns the READ LT knob next to it: its own state here
    rows.append(Control("knob_read_lt", "rotary", "KNOB_OVHD_READ_LT", "KNOB_OVHD_AIRCOND_FWDCABIN", p + "read_lt",
                        label="READ LT", step=0.05, group="overhead_knobs"))
    rows.append(Control("bat_selector", "follow", "KNOB_OVHD_BAT", "KNOB_OVHD_BAT", f"{PREFIX}elec/bat/selector",
                        label="BAT display - ESS / APU / OFF / BAT 1 / BAT 2", minimum=0, maximum=4,
                        knob=(f"{PREFIX}elec/bat_selector_right", f"{PREFIX}elec/bat_selector_left"),
                        group="overhead_knobs"))
    rows.append(Control("parking_brake", "follow", "LEVER_PARKINGBRAKE", "lever_parking_brake",
                        "sim/cockpit2/controls/parking_brake_ratio", ("sim/flight_controls/park_brake_toggle",),
                        "PARK BRK - pull and turn to set / release", also=("LEVER_PARKINGBRAKE_WHITE_MIKE",),
                        group="overhead_knobs"))
    return rows


WIPER_DATAREFS = (f"{PREFIX}wipers/captain_sweep", f"{PREFIX}wipers/first_officer_sweep")


def _wipers():
    # WIPER selector 0 OFF, 1 SLOW, 2 FAST; the blade sweep (0 parked .. 1) is a380x_cockpit_feedback.inc
    rows = []
    for lr, side, caption in (("left", "captain", "CAPT"), ("right", "first_officer", "F/O")):
        rows.append(Control(f"wiper_{side}_selector", "selector", f"HANDLING_Switch_Wiper_{lr}",
                            f"HANDLING_Switch_Wiper_{lr}", f"{PREFIX}wipers/{side}_selector",
                            label=f"{caption} WIPER - OFF / SLOW / FAST", positions=3, default=0, group="wipers"))
        rows.append(Control(f"wiper_{side}_blade", "follow", f"WIPER_BASE_{lr[0].upper()}", f"HANDLING_Wiper_{lr}",
                            f"{PREFIX}wipers/{side}_sweep", click=False, group="wipers"))
    return rows


def _furniture():
    """Seats, armrests, tables, keyboard trays, windows, shades and stowage doors (FBW cabin clips).
    Two-position parts glide over `travel` seconds; seat height / fore-aft and armrest tilt are rotaries."""
    p = PREFIX + "cockpit/"
    rows = []
    for short, side in (("CPT", "captain"), ("FO", "first_officer")):
        rows += [
            Control(f"seat_{side}_height", "rotary", f"SEAT_{short}_MOVE_UP_DOWN_SWITCH", f"SEAT_{short}_MOVE_UP_DOWN_ANIM",
                    f"{p}{side}_seat_height", label=f"{short} SEAT - up / down", step=0.1, travel=0.25,
                    all_nodes=True, manip="up_down", group="furniture"),
            Control(f"seat_{side}_fwd_aft", "rotary", f"SEAT_{short}_MOVE_FWD_AFT_SWITCH", f"SEAT_{short}_MOVE_FWD_AFT_ANIM",
                    f"{p}{side}_seat_fwd_aft", label=f"{short} SEAT - aft / forward", step=0.1, travel=0.25,
                    all_nodes=True, manip="up_down", group="furniture"),
            *([] if short == "FO" else [Control(f"armrest_{side}_stow", "toggle", f"BIGARMREST_{short}_STOW_CLICK", f"BIGARMREST_{short}_STOW_ANIM",
                    f"{p}{side}_armrest_stow", label=f"{short} ARMREST - stow / deploy", travel=1.0, all_nodes=True,
                    group="furniture")]),       # FBW has no F/O stow clip
            Control(f"armrest_{side}_height", "rotary", f"BIGARMREST_{short}_UP_DOWN_CLICK",
                    f"BIGARMREST_{short}_UP_DOWN_ANIM", f"{p}{side}_armrest_height", label=f"{short} ARMREST height",
                    step=0.1, travel=0.25, all_nodes=True, manip="up_down", group="furniture"),
            Control(f"armrest_{side}_tilt", "rotary", f"BIGARMREST_{short}_TILT_CLICK", f"BIGARMREST_{short}_TILT_ANIM",
                    f"{p}{side}_armrest_tilt", label=f"{short} ARMREST tilt", step=0.1, travel=0.25, all_nodes=True,
                    manip="up_down", group="furniture"),
            Control(f"small_armrest_{side}_stow", "toggle", f"SMALLARMREST_{short}_STOW_CLICK",
                    f"SMALLARMREST_{short}_STOW_ANIM", f"{p}{side}_small_armrest_stow",
                    label=f"{short} inboard ARMREST - stow / deploy", travel=1.0, all_nodes=True, group="furniture"),
            Control(f"small_armrest_{side}_fwd", "toggle", f"SMALLARMREST_{short}_FWD_CLICK",
                    f"SMALLARMREST_{short}_FWD_ANIM", f"{p}{side}_small_armrest_fwd",
                    label=f"{short} inboard ARMREST - slide", travel=0.8, all_nodes=True, group="furniture"),
            Control(f"table_{side}", "toggle", f"A380_{short}_TABLE_CLICK", f"A380_{short}_TABLE", f"{p}{side}_table",
                    label=f"{short} TABLE - stow / deploy", travel=2.5, all_nodes=True, group="furniture"),
            Control(f"meal_table_{side}", "toggle", f"A380_{short}_KEYBOARD_CLICK", f"A380_{short}_MEALTABLE",
                    f"{p}{side}_meal_table", label=f"{short} MEAL TABLE - stow / deploy", travel=1.5, all_nodes=True,
                    group="furniture"),
            Control(f"keyboard_{side}", "toggle", f"A380_{short}_MEALTABLE_CLICK", f"A380_{short}_KEYBOARD",
                    f"{p}{side}_keyboard", label=f"{short} KEYBOARD tray - stow / deploy", travel=1.0, all_nodes=True,
                    group="furniture"),
            Control(f"footrest_{side}", "toggle", f"A380_{short}_FOOTREST_CLICK", f"A380_{short}_FOOTREST",
                    f"{p}{side}_footrest", label=f"{short} FOOTREST - stow / deploy", travel=1.5, all_nodes=True,
                    group="furniture"),
            Control(f"sliding_window_{side}", "toggle", f"{short}_SLIDING_WINDOW_CLICK", f"{short}_SLIDING_WINDOW_ANIM",
                    f"{p}{side}_sliding_window", label=f"{short} SLIDING WINDOW - open / close", travel=3.0,
                    all_nodes=True, group="furniture"),
            Control(f"small_shade_{side}", "toggle", "Plane.075" if short == "CPT" else "Plane.154",
                    f"{short}_SMALL_SHADE_ANIM", f"{p}{side}_small_shade", label=f"{short} side SUNSHADE",
                    travel=1.0, all_nodes=True, group="furniture"),
        ]
    for key, clip, node, label in (("fwd_lh", "SUNSHADE_FWD_LH_ANIM", "SUNSHADE_FWD_LH", "FWD LH SUNSHADE"),
                                   ("fwd_rh", "SUNSHADE_FWD_RH_ANIM", "SUNSHADE_FWD_RH", "FWD RH SUNSHADE"),
                                   ("fwd_ctr", "SUNSHADE_FWD_CTR_ANIM", "SUNSHADE_FWD_CTR_BALL", "FWD CTR SUNSHADE")):
        # no click target: the shade's box hangs in front of the overhead and took its clicks (commands only)
        rows.append(Control(f"sunshade_{key}", "toggle", node, clip, f"{p}sunshade_{key}", label=f"{label} - up / down",
                            travel=1.5, all_nodes=True, click=False, group="furniture"))
    for key, clip, node, click, label in (
            ("aft_oxy", "AFT_OXY_ANIM", "Cube.082", "AFT_OXY_CLICK", "AFT OXYGEN stowage"),
            ("aft_oit", "AFT_OIT_ANIM", "Cube.444", "AFT_OIT_CLICK", "AFT OIT stowage"),
            ("cas_lh", "CAS_LH_OPEN_ANIM", "Cube.128", None, "LH stowage"),
            ("cas_rh", "CAS_RH_OPEN_ANIM", "Cube.426", None, "RH stowage"),
            ("cpt_oxy_fwd", "CPT_OXY_FWD_ANIM", "Cube.604", None, "CAPT OXYGEN stowage"),
            ("lg_pin_door", "A380_LGPIN_DOOR", "LG_DOOR_HINGE", None, "Gear pin stowage")):
        rows.append(Control(f"stowage_{key}", "toggle", node, clip, f"{p}{key}_door", label=f"{label} - open / close",
                            travel=0.8, all_nodes=True, click_node=click, group="furniture"))
    return rows


RMP_DEVICES = ("a380x.xp12.rmp.1", "a380x.xp12.rmp.2", "a380x.xp12.rmp.3")
RMP_CHANNELS = (23, 24, 25)       # instrument brightness slots (MFD 14/15, ND 16/17, SD 18, PFD 19/20, ISIS 21/22)
RMP_SCREEN_NODES = ("SCREEN_RMP_1", "SCREEN_RMP_2", "SCREEN_RMP_3")
RMP_W, RMP_H = 768, 466           # device texels (95.1 x 57.7 mm screen)
RMP_COMMANDS = tuple(c for n in (1, 2, 3) for c in rmp_commands(n))
RMP_DATAREFS = tuple(d for n in (1, 2, 3) for d in rmp_datarefs(n))   # EMER EXIT LT OFF: the FBW legend has no pushbutton clip

CONTROLS: tuple[Control, ...] = (*_switching(), *_ecam(), *_surv(), *_kccu(), *_eng_master(), *_glareshield_side(),
                                 *_overhead(), *_rmp(), *_eng_start(), *_glareshield(), *_sidesticks(), *_efis(),
                                 *_kccu_extra(), *_oit(), *_panel_keys(), *_fire(), *_guards_and_breakers(),
                                 *_overhead_knobs(), *_wipers(), *_furniture())


def owned_commands() -> tuple[str, ...]:
    """Commands the native feedback module creates (press commands of other modules are only hooked)."""
    out = [PREFIX + "surv/" + key for key in SURV_BUTTONS]
    for c in CONTROLS:
        if c.kind == "selector":
            out += [c.dataref + "_left", c.dataref + "_right"]
        elif c.kind == "switch":
            out += [c.dataref + "_up", c.dataref + "_down"]
        elif c.kind == "press" and c.own:
            out += list(c.commands)
        elif c.kind == "toggle":
            out.append(c.dataref + "_toggle")
        elif c.kind == "rotary" and not c.commands:
            out += [c.dataref + "_up", c.dataref + "_down"]
    return tuple(dict.fromkeys(out))


def hooked_commands() -> tuple[str, ...]:
    owned = set(owned_commands())
    out = [cmd for c in CONTROLS if c.kind in ("press", "rotary") for cmd in c.commands if cmd not in owned]
    return tuple(dict.fromkeys(out))


def module_datarefs() -> set[str]:
    """Datarefs another native module publishes (the feedback module only binds geometry to them)."""
    return set(RMP_DATAREFS)


FIRE_PUSHBUTTON_DATAREF = PREFIX + "engine/fire_pushbutton"   # int[4], ENG 1..4 released


def published_datarefs() -> tuple[str, ...]:
    out = []
    others = module_datarefs()
    for c in CONTROLS:
        if c.kind not in ("follow", "static"):
            out.append(c.dataref)
        out += [lamp.dataref for lamp in c.lamps]
    out += [c.commands[0] for c in CONTROLS if c.kind == "press" and c.latch >= 0]   # latched position
    out += [*WIPER_DATAREFS, FIRE_PUSHBUTTON_DATAREF]
    return tuple(d for d in dict.fromkeys(out) if d not in others)


COMMANDS = owned_commands()
DATAREFS = published_datarefs()


def rotary_commands(c: Control) -> tuple[str, str]:
    return tuple(c.commands[:2]) if c.commands else (c.dataref + "_up", c.dataref + "_down")


def click_manipulator(c: Control) -> str | None:
    if c.knob:
        return f"ATTR_manip_command_knob\trotate_small\t{c.knob[0]}\t{c.knob[1]}\t{c.label}"
    if not c.click or c.kind == "static" or (c.kind == "follow" and not c.commands):
        return None
    if c.kind in ("press", "follow"):
        return f"ATTR_manip_command\tbutton\t{c.commands[0]}\t{c.label}"
    if c.kind == "toggle":
        return f"ATTR_manip_command\thand\t{c.dataref}_toggle\t{c.label}"
    if c.kind == "rotary":
        up, down = rotary_commands(c)
        if c.manip == "up_down":
            return f"ATTR_manip_command_switch_up_down\tup_down\t{up}\t{down}\t{c.label}"
        return f"ATTR_manip_command_knob\trotate_small\t{up}\t{down}\t{c.label}"
    if c.kind == "selector":
        return f"ATTR_manip_command_knob\trotate_medium\t{c.dataref}_right\t{c.dataref}_left\t{c.label}"
    return f"ATTR_manip_command_switch_up_down\tup_down\t{c.dataref}_up\t{c.dataref}_down\t{c.label}"


def _cf(value: float) -> str:
    """C float literal."""
    return f"{round(float(value), 6)!r}f"


def emit_c_table() -> str:
    """native/avionics/a380x_cockpit_feedback_table.h (generated, checked by native/avionics/build.py)."""
    out = ["/* SPDX-License-Identifier: GPL-3.0-or-later",
           " * GENERATED by tools/cockpit_feedback_contract.py emit_c_table(); do not edit by hand. */",
           "#ifndef A380X_COCKPIT_FEEDBACK_TABLE_H", "#define A380X_COCKPIT_FEEDBACK_TABLE_H", "",
           "typedef struct { const char *dataref; const char *command; } CfbPressRow;",
           "typedef struct { const char *dataref; int positions, initial, switch_kind; } CfbPositionRow;",
           "typedef struct { const char *dataref; int page; } CfbEcamLampRow;",
           "typedef struct { const char *dataref; float travel, initial; } CfbToggleRow;",
           "typedef struct { const char *dataref, *command; int initial; } CfbLatchRow;",
           "typedef struct { const char *dataref; int rule, index; const char *arg; } CfbLampRuleRow;",
           "enum { " + ", ".join(f"CFB_RULE_{r.upper()}" for r in LAMP_RULES) + " };",
           "typedef struct { const char *dataref, *up, *down; float step, initial, travel; int wrap, hooked; } CfbRotaryRow;",
           "",
           "static const CfbPressRow cfb_press_rows[] = {"]
    for c in CONTROLS:
        if c.kind == "press":
            for cmd in c.commands:
                out.append(f'    {{"{c.dataref}", "{cmd}"}},')
    out += ["};", "", "static const CfbPositionRow cfb_position_rows[] = {"]
    for c in CONTROLS:
        if c.kind in ("selector", "switch"):
            out.append(f'    {{"{c.dataref}", {c.positions}, {c.default}, {1 if c.kind == "switch" else 0}}},')
    out += ["};", "", "static const CfbEcamLampRow cfb_ecam_lamps[] = {"]
    out += [f'    {{"{ref}", {page}}},' for ref, page in ECAM_PAGE_LAMPS.items()]
    out += ["};", "", "static const CfbToggleRow cfb_toggle_rows[] = {"]
    out += [f'    {{"{c.dataref}", {_cf(c.travel)}, {_cf(c.initial)}}},' for c in CONTROLS if c.kind == "toggle"]
    out += ["};", "", "static const CfbRotaryRow cfb_rotary_rows[] = {"]
    for c in CONTROLS:
        if c.kind == "rotary":
            up, down = rotary_commands(c)
            out.append(f'    {{"{c.dataref}", "{up}", "{down}", {_cf(c.step)}, {_cf(c.initial)}, {_cf(c.travel)}, '
                       f'{1 if c.wrap else 0}, {1 if c.commands else 0}}},')
    out += ["};", "", "static const CfbLatchRow cfb_latch_rows[] = {"]
    out += [f'    {{"{c.commands[0]}", "{c.commands[0]}", {c.latch}}},' for c in CONTROLS
            if c.kind == "press" and c.latch >= 0]
    out += ["};", "", "static const CfbLampRuleRow cfb_lamp_rules[] = {"]
    for c in CONTROLS:
        for lamp in c.lamps:
            if lamp.rule:
                rule, *args = lamp.rule.split(":")
                index = int(args[0]) if args and args[0].lstrip("-").isdigit() else 0
                arg = args[-1] if args and not args[-1].lstrip("-").isdigit() else ""
                out.append(f'    {{"{lamp.dataref}", CFB_RULE_{rule.upper()}, {index}, "{arg}"}},')
    out += ["};", "", "static const char *const cfb_wiper_selectors[] = {"]
    out += [f'    "{c.dataref}",' for c in CONTROLS if c.group == "wipers" and c.kind == "selector"]
    out += ["};", "", "static const char *const cfb_wiper_sweeps[] = {"]
    out += [f'    "{ref}",' for ref in WIPER_DATAREFS]
    out += ["};", "", "static const char *const cfb_fire_pushbuttons[] = {   /* ENG 1..4 */"]
    out += [f'    "{PREFIX}fire/{unit.lower()}_pb",' for unit in FIRE_PUSHBUTTONS[:4]]
    out += ["};", "", "static const char *const cfb_surv_buttons[] = {"]
    out += [f'    "{key}",' for key in SURV_BUTTONS]
    out += ["};", "", "#endif", ""]
    return "\n".join(out)


if __name__ == "__main__":
    import sys
    if sys.argv[1:] == ["--emit-c"]:
        sys.stdout.write(emit_c_table())
    else:
        print(f"{len(CONTROLS)} controls, {len(COMMANDS)} owned commands, {len(hooked_commands())} hooked commands, "
              f"{len(DATAREFS)} datarefs")
