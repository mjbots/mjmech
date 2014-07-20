# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

"""Routines to interact with the linux /dev/input subsystem."""

import fcntl
import os
import struct
import sys

import enum # pip install enum34
import trollius as asyncio
from trollius import From, Return

# These helper functions are transliterated from sys/ioctl.h

_IOC_NRBITS = 8
_IOC_TYPEBITS = 8
_IOC_SIZEBITS = 14
_IOC_DIRBITS = 2

_IOC_NRMASK = (1 << _IOC_NRBITS) - 1
_IOC_TYPEMASK = (1 << _IOC_TYPEBITS) - 1
_IOC_SIZEMASK = (1 << _IOC_SIZEBITS) - 1
_IOC_DIRMASK = (1 << _IOC_DIRBITS) - 1

_IOC_NRSHIFT = 0
_IOC_TYPESHIFT = _IOC_NRSHIFT + _IOC_NRBITS
_IOC_SIZESHIFT = _IOC_TYPESHIFT + _IOC_TYPEBITS
_IOC_DIRSHIFT = _IOC_SIZESHIFT + _IOC_SIZEBITS

_IOC_NONE = 0
_IOC_WRITE = 1
_IOC_READ = 2

def _IOC(direction, ioc_type, nr, size):
    return ((direction << _IOC_DIRSHIFT) |
            (ioc_type << _IOC_TYPESHIFT) |
            (nr << _IOC_NRSHIFT) |
            (size << _IOC_SIZESHIFT))

def _IOC_TYPECHECK(t):
    return t.size

def _IO(ioc_type, nr):
    return _IOC(_IOC_NONE, ioc_type, nr, 0)

def _IOR(ioc_type, nr, size):
    return _IOC(_IOC_READ, ioc_type, nr, _IOC_TYPECHECK(size))

def _IOW(ioc_type, nr, size):
    return _IOC(_IOC_WRITE, ioc_type, nr, _IOC_TYPECHECK(size))

def _IOC_DIR(nr):
    return (nr >> _IOC_DIRSHIFT) & _IOC_DIRMASK

def _IOC_TYPE(nr):
    return (nr >> _IOC_TYPESHIFT) & _IOC_TYPEMASK

def _IOC_NR(nr):
    return (nr >> _IOC_NRSHIFT) & _IOC_NRMASK

def _IOC_SIZE(nr):
    return (nr >> _IOC_SIZESHIFT) & _IOC_SIZEMASK

UINT = struct.Struct('I')
INT = struct.Struct('i')
INT2 = struct.Struct('ii')
UINT2 = struct.Struct('II')
INT5 = struct.Struct('iiiii')
SHORT = struct.Struct('h')
USHORT = struct.Struct('H')
SHORT4 = struct.Struct('hhhh')
INPUT_ID = struct.Struct('HHHH')
INPUT_KEYMAP_ENTRY = struct.Struct('BBHI' + 'B' * 32)
INPUT_ABSINFO = struct.Struct('iiiiii')
INPUT_EVENT = struct.Struct('llHHi')

EVIOCGVERSION = _IOR(ord('E'), 0x01, INT)
EVIOCGID = _IOR(ord('E'), 0x02, INPUT_ID)
EVIOCGREP = _IOR(ord('E'), 0x03, UINT2)
EVIOCSREP = _IOW(ord('E'), 0x03, UINT2)

EVIOCGKEYCODE = _IOR(ord('E'), 0x04, UINT2)
EVIOCGKEYCODE_V2 = _IOR(ord('E'), 0x04, INPUT_KEYMAP_ENTRY)
EVIOCSKEYCODE = _IOW(ord('E'), 0x04, UINT2)
EVIOCSKEYCODE_V2 = _IOW(ord('E'), 0x04, INPUT_KEYMAP_ENTRY)

def EVIOCGNAME(evio_len):
    return _IOC(_IOC_READ, ord('E'), 0x06, evio_len)

def EVIOCGPHYS(evio_len):
    return _IOC(_IOC_READ, ord('E'), 0x07, evio_len)

def EVIOCGUNIQ(evio_len):
    return _IOC(_IOC_READ, ord('E'), 0x08, evio_len)

def EVIOCGPROP(evio_len):
    return _IOC(_IOC_READ, ord('E'), 0x09, evio_len)


def EVIOCGMTSLOTS(evio_len):
    return _IOC(_IOC_READ, ord('E'), 0x0a, evio_len)

def EVIOCGKEY(evio_len):
    '''get global key state'''
    return _IOC(_IOC_READ, ord('E'), 0x18, evio_len)

def EVIOCGLED(evio_len):
    '''get all LEDs'''
    return _IOC(_IOC_READ, ord('E'), 0x19, evio_len)

def EVIOCGSND(evio_len):
    '''get all sounds status'''
    return _IOC(_IOC_READ, ord('E'), 0x1a, evio_len)

def EVIOCGSW(evio_len):
    '''get all switch states'''
    return _IOC(_IOC_READ, ord('E'), 0x1b, evio_len)


def EVIOCGBIT(ev, evio_len):
    '''get event bits'''
    return _IOC(_IOC_READ, ord('E'), 0x20 + ev, evio_len)

def EVIOCGABS(ev_abs):
    '''get abs value/limits'''
    return _IOR(ord('E'), 0x40 + ev_abs, INPUT_ABSINFO)

def EVIOCSABS(ev_abs):
    '''set abs value/limits'''
    return _IOW(ord('E'), 0xc0 + ev_abs, INPUT_ABSINFO)


# # send a force effect to a force feedback
# EVIOCSFF = _IOC(_IOC_WRITE, ord('E'), 0x80, FF_EFFECT)

# # Erase a force effect
# EVIOCRMFF = _IOW(ord('E'), 0x81, INT)

# # Report number of effects playable at the same time
# EVIOCGEFFECTS = _IOR(ord('E'), 0x81, INT)

# Grab/Release device
EVIOCGRAB = _IOW(ord('E'), 0x90, INT)

# Revoke device address
EVIOCREVOKE = _IOW(ord('E'), 0x91, INT)

# Set clockid to be used for timestamps
EVIOCSCLOCKID = _IOW(ord('E'), 0xa0, INT)

class INPUT_PROP(enum.Enum):
    '''Device properties and quirks'''
    POINTER = 0x00 # needs a pointer
    DIRECT = 0x01 # direct input devices
    BUTTONPAD = 0x02 # has button(s) under pad
    SEMI_MT = 0x03 # touch rectangle only
    TOPBUTTONPAD = 0x04 # softbuttons at top of pad
    MAX = 0x1f
    CNT = MAX + 1


class EV(enum.Enum):
    '''Event types'''
    SYN = 0x00
    KEY = 0x01
    REL = 0x02
    ABS = 0x03
    MSC = 0x04
    SW = 0x05
    LED = 0x11
    SND = 0x12
    REP = 0x14
    FF = 0x15
    PWR = 0x16
    FF_STATUS = 0x17
    MAX = 0x1f
    CNT = MAX + 1


class SYN(enum.Enum):
    '''Synchronization events'''
    REPORT = 0
    CONFIG = 1
    MT_REPORT = 2
    DROPPED = 3
    MAX = 0xf
    CNT = MAX + 1


class KEY(enum.Enum):
    '''Keys and buttons'''
    KEY_RESERVED = 0
    KEY_ESC = 1
    KEY_1 = 2
    KEY_2 = 3
    KEY_3 = 4
    KEY_4 = 5
    KEY_5 = 6
    KEY_6 = 7
    KEY_7 = 8
    KEY_8 = 9
    KEY_9 = 10
    KEY_0 = 11
    KEY_MINUS = 12
    KEY_EQUAL = 13
    KEY_BACKSPACE = 14
    KEY_TAB = 15
    KEY_Q = 16
    KEY_W = 17
    KEY_E = 18
    KEY_R = 19
    KEY_T = 20
    KEY_Y = 21
    KEY_U = 22
    KEY_I = 23
    KEY_O = 24
    KEY_P = 25
    KEY_LEFTBRACE = 26
    KEY_RIGHTBRACE = 27
    KEY_ENTER = 28
    KEY_LEFTCTRL = 29
    KEY_A = 30
    KEY_S = 31
    KEY_D = 32
    KEY_F = 33
    KEY_G = 34
    KEY_H = 35
    KEY_J = 36
    KEY_K = 37
    KEY_L = 38
    KEY_SEMICOLON = 39
    KEY_APOSTROPHE = 40
    KEY_GRAVE = 41
    KEY_LEFTSHIFT = 42
    KEY_BACKSLASH = 43
    KEY_Z = 44
    KEY_X = 45
    KEY_C = 46
    KEY_V = 47
    KEY_B = 48
    KEY_N = 49
    KEY_M = 50
    KEY_COMMA = 51
    KEY_DOT = 52
    KEY_SLASH = 53
    KEY_RIGHTSHIFT = 54
    KEY_KPASTERISK = 55
    KEY_LEFTALT = 56
    KEY_SPACE = 57
    KEY_CAPSLOCK = 58
    KEY_F1 = 59
    KEY_F2 = 60
    KEY_F3 = 61
    KEY_F4 = 62
    KEY_F5 = 63
    KEY_F6 = 64
    KEY_F7 = 65
    KEY_F8 = 66
    KEY_F9 = 67
    KEY_F10 = 68
    KEY_NUMLOCK = 69
    KEY_SCROLLLOCK = 70
    KEY_KP7 = 71
    KEY_KP8 = 72
    KEY_KP9 = 73
    KEY_KPMINUS = 74
    KEY_KP4 = 75
    KEY_KP5 = 76
    KEY_KP6 = 77
    KEY_KPPLUS = 78
    KEY_KP1 = 79
    KEY_KP2 = 80
    KEY_KP3 = 81
    KEY_KP0 = 82
    KEY_KPDOT = 83
    KEY_ZENKAKUHANKAKU = 85
    KEY_102ND = 86
    KEY_F11 = 87
    KEY_F12 = 88
    KEY_RO = 89
    KEY_KATAKANA = 90
    KEY_HIRAGANA = 91
    KEY_HENKAN = 92
    KEY_KATAKANAHIRAGANA = 93
    KEY_MUHENKAN = 94
    KEY_KPJPCOMMA = 95
    KEY_KPENTER = 96
    KEY_RIGHTCTRL = 97
    KEY_KPSLASH = 98
    KEY_SYSRQ = 99
    KEY_RIGHTALT = 100
    KEY_LINEFEED = 101
    KEY_HOME = 102
    KEY_UP = 103
    KEY_PAGEUP = 104
    KEY_LEFT = 105
    KEY_RIGHT = 106
    KEY_END = 107
    KEY_DOWN = 108
    KEY_PAGEDOWN = 109
    KEY_INSERT = 110
    KEY_DELETE = 111
    KEY_MACRO = 112
    KEY_MUTE = 113
    KEY_VOLUMEDOWN = 114
    KEY_VOLUMEUP = 115
    KEY_POWER = 116
    KEY_KPEQUAL = 117
    KEY_KPPLUSMINUS = 118
    KEY_PAUSE = 119
    KEY_SCALE = 120
    KEY_KPCOMMA = 121
    KEY_HANGEUL = 122
    KEY_HANGUEL = KEY_HANGEUL
    KEY_HANJA = 123
    KEY_YEN = 124
    KEY_LEFTMETA = 125
    KEY_RIGHTMETA = 126
    KEY_COMPOSE = 127
    KEY_STOP = 128
    KEY_AGAIN = 129
    KEY_PROPS = 130
    KEY_UNDO = 131
    KEY_FRONT = 132
    KEY_COPY = 133
    KEY_OPEN = 134
    KEY_PASTE = 135
    KEY_FIND = 136
    KEY_CUT = 137
    KEY_HELP = 138
    KEY_MENU = 139
    KEY_CALC = 140
    KEY_SETUP = 141
    KEY_SLEEP = 142
    KEY_WAKEUP = 143
    KEY_FILE = 144
    KEY_SENDFILE = 145
    KEY_DELETEFILE = 146
    KEY_XFER = 147
    KEY_PROG1 = 148
    KEY_PROG2 = 149
    KEY_WWW = 150
    KEY_MSDOS = 151
    KEY_COFFEE = 152
    KEY_SCREENLOCK = KEY_COFFEE
    KEY_DIRECTION = 153
    KEY_CYCLEWINDOWS = 154
    KEY_MAIL = 155
    KEY_BOOKMARKS = 156
    KEY_COMPUTER = 157
    KEY_BACK = 158
    KEY_FORWARD = 159
    KEY_CLOSECD = 160
    KEY_EJECTCD = 161
    KEY_EJECTCLOSECD = 162
    KEY_NEXTSONG = 163
    KEY_PLAYPAUSE = 164
    KEY_PREVIOUSSONG = 165
    KEY_STOPCD = 166
    KEY_RECORD = 167
    KEY_REWIND = 168
    KEY_PHONE = 169
    KEY_ISO = 170
    KEY_CONFIG = 171
    KEY_HOMEPAGE = 172
    KEY_REFRESH = 173
    KEY_EXIT = 174
    KEY_MOVE = 175
    KEY_EDIT = 176
    KEY_SCROLLUP = 177
    KEY_SCROLLDOWN = 178
    KEY_KPLEFTPAREN = 179
    KEY_KPRIGHTPAREN = 180
    KEY_NEW = 181
    KEY_REDO = 182
    KEY_F13 = 183
    KEY_F14 = 184
    KEY_F15 = 185
    KEY_F16 = 186
    KEY_F17 = 187
    KEY_F18 = 188
    KEY_F19 = 189
    KEY_F20 = 190
    KEY_F21 = 191
    KEY_F22 = 192
    KEY_F23 = 193
    KEY_F24 = 194
    KEY_PLAYCD = 200
    KEY_PAUSECD = 201
    KEY_PROG3 = 202
    KEY_PROG4 = 203
    KEY_DASHBOARD = 204
    KEY_SUSPEND = 205
    KEY_CLOSE = 206
    KEY_PLAY = 207
    KEY_FASTFORWARD = 208
    KEY_BASSBOOST = 209
    KEY_PRINT = 210
    KEY_HP = 211
    KEY_CAMERA = 212
    KEY_SOUND = 213
    KEY_QUESTION = 214
    KEY_EMAIL = 215
    KEY_CHAT = 216
    KEY_SEARCH = 217
    KEY_CONNECT = 218
    KEY_FINANCE = 219
    KEY_SPORT = 220
    KEY_SHOP = 221
    KEY_ALTERASE = 222
    KEY_CANCEL = 223
    KEY_BRIGHTNESSDOWN = 224
    KEY_BRIGHTNESSUP = 225
    KEY_MEDIA = 226
    KEY_SWITCHVIDEOMODE = 227
    KEY_KBDILLUMTOGGLE = 228
    KEY_KBDILLUMDOWN = 229
    KEY_KBDILLUMUP = 230
    KEY_SEND = 231
    KEY_REPLY = 232
    KEY_FORWARDMAIL = 233
    KEY_SAVE = 234
    KEY_DOCUMENTS = 235
    KEY_BATTERY = 236
    KEY_BLUETOOTH = 237
    KEY_WLAN = 238
    KEY_UWB = 239
    KEY_UNKNOWN = 240
    KEY_VIDEO_NEXT = 241
    KEY_VIDEO_PREV = 242
    KEY_BRIGHTNESS_CYCLE = 243
    KEY_BRIGHTNESS_ZERO = 244
    KEY_DISPLAY_OFF = 245
    KEY_WWAN = 246
    KEY_WIMAX = KEY_WWAN
    KEY_RFKILL = 247
    KEY_MICMUTE = 248
    BTN_MISC = 0x100
    BTN_0 = 0x100
    BTN_1 = 0x101
    BTN_2 = 0x102
    BTN_3 = 0x103
    BTN_4 = 0x104
    BTN_5 = 0x105
    BTN_6 = 0x106
    BTN_7 = 0x107
    BTN_8 = 0x108
    BTN_9 = 0x109
    BTN_MOUSE = 0x110
    BTN_LEFT = 0x110
    BTN_RIGHT = 0x111
    BTN_MIDDLE = 0x112
    BTN_SIDE = 0x113
    BTN_EXTRA = 0x114
    BTN_FORWARD = 0x115
    BTN_BACK = 0x116
    BTN_TASK = 0x117
    BTN_JOYSTICK = 0x120
    BTN_TRIGGER = 0x120
    BTN_THUMB = 0x121
    BTN_THUMB2 = 0x122
    BTN_TOP = 0x123
    BTN_TOP2 = 0x124
    BTN_PINKIE = 0x125
    BTN_BASE = 0x126
    BTN_BASE2 = 0x127
    BTN_BASE3 = 0x128
    BTN_BASE4 = 0x129
    BTN_BASE5 = 0x12a
    BTN_BASE6 = 0x12b
    BTN_DEAD = 0x12f
    BTN_GAMEPAD = 0x130
    BTN_SOUTH = 0x130
    BTN_A = BTN_SOUTH
    BTN_EAST = 0x131
    BTN_B = BTN_EAST
    BTN_C = 0x132
    BTN_NORTH = 0x133
    BTN_X = BTN_NORTH
    BTN_WEST = 0x134
    BTN_Y = BTN_WEST
    BTN_Z = 0x135
    BTN_TL = 0x136
    BTN_TR = 0x137
    BTN_TL2 = 0x138
    BTN_TR2 = 0x139
    BTN_SELECT = 0x13a
    BTN_START = 0x13b
    BTN_MODE = 0x13c
    BTN_THUMBL = 0x13d
    BTN_THUMBR = 0x13e
    BTN_DIGI = 0x140
    BTN_TOOL_PEN = 0x140
    BTN_TOOL_RUBBER = 0x141
    BTN_TOOL_BRUSH = 0x142
    BTN_TOOL_PENCIL = 0x143
    BTN_TOOL_AIRBRUSH = 0x144
    BTN_TOOL_FINGER = 0x145
    BTN_TOOL_MOUSE = 0x146
    BTN_TOOL_LENS = 0x147
    BTN_TOOL_QUINTTAP = 0x148
    BTN_TOUCH = 0x14a
    BTN_STYLUS = 0x14b
    BTN_STYLUS2 = 0x14c
    BTN_TOOL_DOUBLETAP = 0x14d
    BTN_TOOL_TRIPLETAP = 0x14e
    BTN_TOOL_QUADTAP = 0x14f
    BTN_WHEEL = 0x150
    BTN_GEAR_DOWN = 0x150
    BTN_GEAR_UP = 0x151
    KEY_OK = 0x160
    KEY_SELECT = 0x161
    KEY_GOTO = 0x162
    KEY_CLEAR = 0x163
    KEY_POWER2 = 0x164
    KEY_OPTION = 0x165
    KEY_INFO = 0x166
    KEY_TIME = 0x167
    KEY_VENDOR = 0x168
    KEY_ARCHIVE = 0x169
    KEY_PROGRAM = 0x16a
    KEY_CHANNEL = 0x16b
    KEY_FAVORITES = 0x16c
    KEY_EPG = 0x16d
    KEY_PVR = 0x16e
    KEY_MHP = 0x16f
    KEY_LANGUAGE = 0x170
    KEY_TITLE = 0x171
    KEY_SUBTITLE = 0x172
    KEY_ANGLE = 0x173
    KEY_ZOOM = 0x174
    KEY_MODE = 0x175
    KEY_KEYBOARD = 0x176
    KEY_SCREEN = 0x177
    KEY_PC = 0x178
    KEY_TV = 0x179
    KEY_TV2 = 0x17a
    KEY_VCR = 0x17b
    KEY_VCR2 = 0x17c
    KEY_SAT = 0x17d
    KEY_SAT2 = 0x17e
    KEY_CD = 0x17f
    KEY_TAPE = 0x180
    KEY_RADIO = 0x181
    KEY_TUNER = 0x182
    KEY_PLAYER = 0x183
    KEY_TEXT = 0x184
    KEY_DVD = 0x185
    KEY_AUX = 0x186
    KEY_MP3 = 0x187
    KEY_AUDIO = 0x188
    KEY_VIDEO = 0x189
    KEY_DIRECTORY = 0x18a
    KEY_LIST = 0x18b
    KEY_MEMO = 0x18c
    KEY_CALENDAR = 0x18d
    KEY_RED = 0x18e
    KEY_GREEN = 0x18f
    KEY_YELLOW = 0x190
    KEY_BLUE = 0x191
    KEY_CHANNELUP = 0x192
    KEY_CHANNELDOWN = 0x193
    KEY_FIRST = 0x194
    KEY_LAST = 0x195
    KEY_AB = 0x196
    KEY_NEXT = 0x197
    KEY_RESTART = 0x198
    KEY_SLOW = 0x199
    KEY_SHUFFLE = 0x19a
    KEY_BREAK = 0x19b
    KEY_PREVIOUS = 0x19c
    KEY_DIGITS = 0x19d
    KEY_TEEN = 0x19e
    KEY_TWEN = 0x19f
    KEY_VIDEOPHONE = 0x1a0
    KEY_GAMES = 0x1a1
    KEY_ZOOMIN = 0x1a2
    KEY_ZOOMOUT = 0x1a3
    KEY_ZOOMRESET = 0x1a4
    KEY_WORDPROCESSOR = 0x1a5
    KEY_EDITOR = 0x1a6
    KEY_SPREADSHEET = 0x1a7
    KEY_GRAPHICSEDITOR = 0x1a8
    KEY_PRESENTATION = 0x1a9
    KEY_DATABASE = 0x1aa
    KEY_NEWS = 0x1ab
    KEY_VOICEMAIL = 0x1ac
    KEY_ADDRESSBOOK = 0x1ad
    KEY_MESSENGER = 0x1ae
    KEY_DISPLAYTOGGLE = 0x1af
    KEY_SPELLCHECK = 0x1b0
    KEY_LOGOFF = 0x1b1
    KEY_DOLLAR = 0x1b2
    KEY_EURO = 0x1b3
    KEY_FRAMEBACK = 0x1b4
    KEY_FRAMEFORWARD = 0x1b5
    KEY_CONTEXT_MENU = 0x1b6
    KEY_MEDIA_REPEAT = 0x1b7
    KEY_10CHANNELSUP = 0x1b8
    KEY_10CHANNELSDOWN = 0x1b9
    KEY_IMAGES = 0x1ba
    KEY_DEL_EOL = 0x1c0
    KEY_DEL_EOS = 0x1c1
    KEY_INS_LINE = 0x1c2
    KEY_DEL_LINE = 0x1c3
    KEY_FN = 0x1d0
    KEY_FN_ESC = 0x1d1
    KEY_FN_F1 = 0x1d2
    KEY_FN_F2 = 0x1d3
    KEY_FN_F3 = 0x1d4
    KEY_FN_F4 = 0x1d5
    KEY_FN_F5 = 0x1d6
    KEY_FN_F6 = 0x1d7
    KEY_FN_F7 = 0x1d8
    KEY_FN_F8 = 0x1d9
    KEY_FN_F9 = 0x1da
    KEY_FN_F10 = 0x1db
    KEY_FN_F11 = 0x1dc
    KEY_FN_F12 = 0x1dd
    KEY_FN_1 = 0x1de
    KEY_FN_2 = 0x1df
    KEY_FN_D = 0x1e0
    KEY_FN_E = 0x1e1
    KEY_FN_F = 0x1e2
    KEY_FN_S = 0x1e3
    KEY_FN_B = 0x1e4
    KEY_BRL_DOT1 = 0x1f1
    KEY_BRL_DOT2 = 0x1f2
    KEY_BRL_DOT3 = 0x1f3
    KEY_BRL_DOT4 = 0x1f4
    KEY_BRL_DOT5 = 0x1f5
    KEY_BRL_DOT6 = 0x1f6
    KEY_BRL_DOT7 = 0x1f7
    KEY_BRL_DOT8 = 0x1f8
    KEY_BRL_DOT9 = 0x1f9
    KEY_BRL_DOT10 = 0x1fa
    KEY_NUMERIC_0 = 0x200
    KEY_NUMERIC_1 = 0x201
    KEY_NUMERIC_2 = 0x202
    KEY_NUMERIC_3 = 0x203
    KEY_NUMERIC_4 = 0x204
    KEY_NUMERIC_5 = 0x205
    KEY_NUMERIC_6 = 0x206
    KEY_NUMERIC_7 = 0x207
    KEY_NUMERIC_8 = 0x208
    KEY_NUMERIC_9 = 0x209
    KEY_NUMERIC_STAR = 0x20a
    KEY_NUMERIC_POUND = 0x20b
    KEY_CAMERA_FOCUS = 0x210
    KEY_WPS_BUTTON = 0x211
    KEY_TOUCHPAD_TOGGLE = 0x212
    KEY_TOUCHPAD_ON = 0x213
    KEY_TOUCHPAD_OFF = 0x214
    KEY_CAMERA_ZOOMIN = 0x215
    KEY_CAMERA_ZOOMOUT = 0x216
    KEY_CAMERA_UP = 0x217
    KEY_CAMERA_DOWN = 0x218
    KEY_CAMERA_LEFT = 0x219
    KEY_CAMERA_RIGHT = 0x21a
    KEY_ATTENDANT_ON = 0x21b
    KEY_ATTENDANT_OFF = 0x21c
    KEY_ATTENDANT_TOGGLE = 0x21d
    KEY_LIGHTS_TOGGLE = 0x21e
    BTN_DPAD_UP = 0x220
    BTN_DPAD_DOWN = 0x221
    BTN_DPAD_LEFT = 0x222
    BTN_DPAD_RIGHT = 0x223
    KEY_ALS_TOGGLE = 0x230
    BTN_TRIGGER_HAPPY = 0x2c0
    BTN_TRIGGER_HAPPY1 = 0x2c0
    BTN_TRIGGER_HAPPY2 = 0x2c1
    BTN_TRIGGER_HAPPY3 = 0x2c2
    BTN_TRIGGER_HAPPY4 = 0x2c3
    BTN_TRIGGER_HAPPY5 = 0x2c4
    BTN_TRIGGER_HAPPY6 = 0x2c5
    BTN_TRIGGER_HAPPY7 = 0x2c6
    BTN_TRIGGER_HAPPY8 = 0x2c7
    BTN_TRIGGER_HAPPY9 = 0x2c8
    BTN_TRIGGER_HAPPY10 = 0x2c9
    BTN_TRIGGER_HAPPY11 = 0x2ca
    BTN_TRIGGER_HAPPY12 = 0x2cb
    BTN_TRIGGER_HAPPY13 = 0x2cc
    BTN_TRIGGER_HAPPY14 = 0x2cd
    BTN_TRIGGER_HAPPY15 = 0x2ce
    BTN_TRIGGER_HAPPY16 = 0x2cf
    BTN_TRIGGER_HAPPY17 = 0x2d0
    BTN_TRIGGER_HAPPY18 = 0x2d1
    BTN_TRIGGER_HAPPY19 = 0x2d2
    BTN_TRIGGER_HAPPY20 = 0x2d3
    BTN_TRIGGER_HAPPY21 = 0x2d4
    BTN_TRIGGER_HAPPY22 = 0x2d5
    BTN_TRIGGER_HAPPY23 = 0x2d6
    BTN_TRIGGER_HAPPY24 = 0x2d7
    BTN_TRIGGER_HAPPY25 = 0x2d8
    BTN_TRIGGER_HAPPY26 = 0x2d9
    BTN_TRIGGER_HAPPY27 = 0x2da
    BTN_TRIGGER_HAPPY28 = 0x2db
    BTN_TRIGGER_HAPPY29 = 0x2dc
    BTN_TRIGGER_HAPPY30 = 0x2dd
    BTN_TRIGGER_HAPPY31 = 0x2de
    BTN_TRIGGER_HAPPY32 = 0x2df
    BTN_TRIGGER_HAPPY33 = 0x2e0
    BTN_TRIGGER_HAPPY34 = 0x2e1
    BTN_TRIGGER_HAPPY35 = 0x2e2
    BTN_TRIGGER_HAPPY36 = 0x2e3
    BTN_TRIGGER_HAPPY37 = 0x2e4
    BTN_TRIGGER_HAPPY38 = 0x2e5
    BTN_TRIGGER_HAPPY39 = 0x2e6
    BTN_TRIGGER_HAPPY40 = 0x2e7
    KEY_MIN_INTERESTING = KEY_MUTE
    KEY_MAX = 0x2ff
    KEY_CNT = (KEY_MAX+1)


class REL(enum.Enum):
    '''Relative axes'''
    X = 0x00
    Y = 0x01
    Z = 0x02
    RX = 0x03
    RY = 0x04
    RZ = 0x05
    HWHEEL = 0x06
    DIAL = 0x07
    WHEEL = 0x08
    MISC = 0x09
    MAX = 0x0f
    CNT = MAX + 1


class ABS(enum.Enum):
    '''Absolute axes'''
    X = 0x00
    Y = 0x01
    Z = 0x02
    RX = 0x03
    RY = 0x04
    RZ = 0x05
    THROTTLE = 0x06
    RUDDER = 0x07
    WHEEL = 0x08
    GAS = 0x09
    BRAKE = 0x0a
    HAT0X = 0x10
    HAT0Y = 0x11
    HAT1X = 0x12
    HAT1Y = 0x13
    HAT2X = 0x14
    HAT2Y = 0x15
    HAT3X = 0x16
    HAT3Y = 0x17
    PRESSURE = 0x18
    DISTANCE = 0x19
    TILT_X = 0x1a
    TILT_Y = 0x1b
    TOOL_WIDTH = 0x1c

    VOLUME = 0x20
    MISC = 0x28

    MT_SLOT = 0x2f # MT slot being modified
    MT_TOUCH_MAJOR = 0x30 # Major axis of touching ellipse
    MT_TOUCH_MINOR = 0x31 # Minor axis (omit if circular)
    MT_WIDTH_MAJOR = 0x32 # Major axis of approaching ellipse
    MT_WIDTH_MINOR = 0x33 # Minor axis (omit if circular)
    MT_ORIENTATION = 0x34 # Ellipse orientation
    MT_POSITION_X = 0x35 # Center X touch position
    MT_POSITION_Y = 0x36 # Center Y touch position
    MT_TOOL_TYPE = 0x37 # Type of touching device
    MT_BLOB_ID = 0x38 # Group a set of packets as a blob
    MT_TRACKING_ID = 0x39 # Unique ID of initiated contact
    MT_PRESSURE = 0x3a # Pressure on contact area
    MT_DISTANCE = 0x3b # Contact hover distance
    MT_TOOL_X = 0x3c # Center X tool position
    MT_TOOL_Y = 0x3d # Center Y tool position

    MAX = 0x3f
    CNT = MAX + 1


class SW(enum.Enum):
    # Switch events
    LID = 0x00 # set = lid shut
    TABLET_MODE = 0x01 # set = tablet mode
    HEADPHONE_INSERT = 0x02 # set = inserted
    RFKILL_ALL = 0x03 # rfkill master switch, type "any" set = radio enabled
    RADIO = RFKILL_ALL
    MICROPHONE_INSERT = 0x04 # set = inserted
    DOCK = 0x05 # set = plugged into dock
    LINEOUT_INSERT = 0x06 # set = inserted
    #...


class ID(enum.Enum):
    BUS = 0
    VENDOR = 1
    PRODUCT = 2
    VERSION = 3


class BUS(enum.Enum):
    PCI = 0x01
    ISAPNP = 0x02
    USB = 0x03
    HIL = 0x04
    BLUETOOTH = 0x05
    VIRTUAL = 0x06

    ISA = 0x10
    I8042 = 0x11
    #...


class MT_TOOL(enum.Enum):
    '''MT_TOOL types'''
    FINGER = 0
    PEN = 1
    MAX = 1


class Event(object):
    def __init__(self, time, ev_type, code, value):
        self.time = time
        self.ev_type = ev_type
        self.code = code
        self.value = value

    def __str__(self):
        if self.ev_type == EV.REL:
            code_str = str(REL(self.code))
        elif self.ev_type == EV.ABS:
            code_str = str(ABS(self.code))
        elif self.ev_type == EV.KEY:
            code_str = str(KEY(self.code))
        else:
            code_str = '0x%x' % self.code
            
        return '<Event time:%f type:%s code:%s value:%d>' % (
            self.time, self.ev_type,
            code_str, self.value)


class AbsInfo(object):
    value = 0
    minimum = 0
    maximum = 0
    fuzz = 0
    flat = 0
    resolution = 0

    def __str__(self):
        return '<AbsInfo value=%d min=%d max=%d>' % (
            self.value, self.minimum, self.maximum)

    def scaled(self):
        if self.value >= 0:
            return float(self.value) / self.maximum
        return -(float(self.value) / self.minimum)

            
class InputDevice(object):
    def __init__(self, filename):
        self._fd = os.open(filename, os.O_RDONLY | os.O_NONBLOCK)
        
        name = fcntl.ioctl(self._fd, EVIOCGNAME(256), chr(0) * 256)
        self.name = name.replace(chr(0), '')

        ver = fcntl.ioctl(self._fd, EVIOCGID, chr(0) * INPUT_ID.size)
        self.id_bus, self.id_vendor, self.id_product, self.id_version = \
            INPUT_ID.unpack(ver)

        def uint_cgbit(val):
            data = fcntl.ioctl(self._fd, EVIOCGBIT(val, 4), chr(0) * 4)
            return UINT.unpack(data)[0]

        self.features = uint_cgbit(0)

        self.features_rel = 0
        if self.features >> EV.REL.value:
            self.features_rel = uint_cgbit(EV.REL.value)

        self.features_abs = 0
        if self.features >> EV.ABS.value:
            self.features_abs = uint_cgbit(EV.ABS.value)

        self.abs = {}
        for axis in ABS:
            if not (self.features_abs & (1 << axis.value)):
                continue

            data = fcntl.ioctl(
                self._fd, EVIOCGABS(axis.value), chr(0) * INPUT_ABSINFO.size)
            info = AbsInfo()
            (info.value, info.minimum, info.maximum,
             info.fuzz, info.flat, info.resolution) = INPUT_ABSINFO.unpack(data)

            self.abs[axis] = info

    def fileno(self):
        return self._fd

    def absinfo(self, axis):
        return self.abs[axis]

    def get_features(self, ev_type):
        if ev_type == EV.ABS:
            return [ABS(bit) for bit in range(ABS.MAX.value + 1)
                    if self.features_abs & (1 << bit)]
        elif ev_type == EV.REL:
            return [REL(bit) for bit in range(REL.MAX.value + 1)
                    if self.features_rel & (1 << bit)]
        return []

    def supported_keys(self):
        size = KEY.KEY_MAX.value / 8 + 1
        data = fcntl.ioctl(self._fd, EVIOCGBIT(EV.KEY.value, size),
                           chr(0) * size)
        val = 0
        for x in reversed(data):
            val <<= 8
            val |= ord(x)

        result = []
        for bit in range(KEY.KEY_MAX.value):
            if val & (1 << bit):
                try:
                    result.append(KEY(bit))
                except ValueError:
                    result.append(bit)

        return result

    @asyncio.coroutine
    def read(self, ev_types=(1 << (EV.MAX.value + 1) - 1)):
        loop = asyncio.get_event_loop()
        event = asyncio.Event()
        loop.add_reader(self._fd, event.set)
        try:
            while True:
                yield From(event.wait())
                event.clear()

                ev = self.raw_read()
                raise Return(ev)
        finally:
            loop.remove_reader(self._fd)

    def raw_read(self):
        tv_sec, tv_usec, ev_type, code, value = INPUT_EVENT.unpack(
            os.read(self._fd, INPUT_EVENT.size))
        ev = Event(tv_sec + tv_usec * 1e-6, EV(ev_type), code, value)

        # If this is an absolute value, update our absolute structure.
        if ev.ev_type == EV.ABS:
            self.abs[ABS(ev.code)].value = value

        return ev
        
    def __str__(self):
        def feature_str(val):
            try:
                return str(EV(val))
            except ValueError:
                return 'UNK%02x' % val

        def flags_str(bitvec, cls):
            return '|'.join([str(cls(bit)) for bit in range(32)
                             if bitvec & (1 << bit)])
            
        feature_flags = flags_str(self.features, EV)
        rel_flags = flags_str(self.features_rel, REL)
        abs_flags = flags_str(self.features_abs, ABS)
        
        return ('<InputDevice "%s" bus=%x vendor=0x%04x '
                'product=0x%04x %s %s %s>' % (
                self.name, self.id_bus, self.id_vendor, self.id_product,
                feature_flags,
                rel_flags, abs_flags))
