"""Protocol v8 shared-memory layout, matching a380x_xpkit_shared.h exactly.

v7: every display unit is captured at its kind's scale (a380x_du_scale.h defaults: PFD 1, ND 3/2,
MFD and ECAM SD 7/4) times its 768x1024 layout. v8: plus views 7/8, the two EFB tablets (1440x1000).
"""
import ctypes as C
import re
from pathlib import Path


def _scale(kind, defaults):
    """The default NUM/DEN of a380x_du_scale.h (the header is the single source of truth)."""
    header = Path(__file__).resolve().parents[1] / 'native/avionics/a380x_du_scale.h'
    try:
        text = header.read_text()
        num = int(re.search(r'#define A380X_%s_SCALE_NUM (\d+)' % kind, text).group(1))
        den = int(re.search(r'#define A380X_%s_SCALE_DEN (\d+)' % kind, text).group(1))
        return num, den
    except (OSError, AttributeError):
        return defaults


PFD_SCALE, ND_SCALE, MFD_SCALE = _scale('PFD', (1, 1)), _scale('ND', (3, 2)), _scale('MFD', (7, 4))
_px = lambda v, scale: v * scale[0] // scale[1]
CAP, VERSION = 262144, 8
W, H = _px(768, PFD_SCALE), _px(1024, PFD_SCALE)
ND_W, ND_H = _px(768, ND_SCALE), _px(1024, ND_SCALE)
MFD_W, MFD_H = _px(768, MFD_SCALE), _px(1024, MFD_SCALE)
EFB_W, EFB_H, EFB_CAP, EFB_ACTIONS, EFB_ACTION_LEN = 1440, 1000, 49152, 32, 128
VIEWS = 9


class Rect(C.Structure):
    _fields_ = [(key, C.c_float) for key in ('x', 'y', 'w', 'h')]


class Point(C.Structure):
    _fields_ = [('x', C.c_float), ('y', C.c_float)]


class Command(C.Structure):
    _fields_ = [(key, C.c_uint8) for key in ('kind', 'layer', 'color', 'font', 'anchor', 'closed')] + [
        ('clip', C.c_uint16), ('a', C.c_float * 4), ('width', C.c_float), ('size', C.c_float),
        ('first', C.c_uint32), ('count', C.c_uint32)]


class DrawList(C.Structure):
    _fields_ = [('cmds', Command * 4096), ('points', Point * 16384), ('text', C.c_char * 32768),
                ('clips', Rect * 256), ('cmd_count', C.c_uint32), ('point_count', C.c_uint32),
                ('text_len', C.c_uint32), ('clip_count', C.c_uint32), ('clip_stack', C.c_uint16 * 16),
                ('clip_depth', C.c_int), ('layer', C.c_uint8), ('overflowed', C.c_int)]


class MfdFrame(C.Structure):
    _fields_ = [('sequence', C.c_uint32), ('powered', C.c_uint32), ('uri', C.c_char * 64), ('list', DrawList)]


class SdFrame(C.Structure):
    _fields_ = [('sequence', C.c_uint32), ('size', C.c_uint32), ('json', C.c_char * 65536)]


class EfbFrame(C.Structure):
    _fields_ = [('sequence', C.c_uint32), ('size', C.c_uint32), ('json', C.c_char * EFB_CAP),
                ('action_head', C.c_uint32), ('action_tail', C.c_uint32), ('actions_dropped', C.c_uint32),
                ('action', (C.c_char * EFB_ACTION_LEN) * EFB_ACTIONS)]


class NdProfile(C.Structure):
    _fields_ = [(key, C.c_uint64) for key in ('snapshot_count', 'snapshot_ns', 'apply_count', 'apply_ns',
                'skipped_count', 'upload_count', 'upload_bytes', 'draw_count')]


class Shared(C.Structure):
    _fields_ = [('magic', C.c_uint32), ('version', C.c_uint32),
                ('state_seq', C.c_uint32), ('state_size', C.c_uint32),
                ('state_time', C.c_double), ('state', C.c_char * CAP),
                ('ready', C.c_uint32 * VIEWS), ('frame_seq', C.c_uint32 * VIEWS),
                ('errors', C.c_uint32), ('frame_time', C.c_double * VIEWS),
                ('zoom_delta', C.c_int * 2), ('nd_status', (C.c_uint32 * 8) * 2),
                ('error', C.c_char * 512), ('pixels', (C.c_ubyte * (W * H * 4)) * 2),
                ('nd_pixels', (C.c_ubyte * (ND_W * ND_H * 4)) * 2),
                ('mfd_pixels', (C.c_ubyte * (MFD_W * MFD_H * 4)) * 3), ('mfd', MfdFrame * 2), ('sd', SdFrame),
                ('nd_profile', NdProfile * 2), ('efb_pixels', (C.c_ubyte * (EFB_W * EFB_H * 4)) * 2),
                ('efb', EfbFrame * 2)]


def view_dimensions(view):
    return (W, H) if view < 2 else (ND_W, ND_H) if view < 4 else (MFD_W, MFD_H) if view < 7 else (EFB_W, EFB_H)


def view_buffer(view):
    width, height = view_dimensions(view)
    length = width * height * 4
    field, index = (('pixels', view) if view < 2 else ('nd_pixels', view - 2) if view < 4 else
                    ('mfd_pixels', view - 4) if view < 7 else ('efb_pixels', view - 7))
    return getattr(Shared, field).offset + length * index, length
