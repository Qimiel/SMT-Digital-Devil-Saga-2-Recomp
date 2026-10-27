#!/usr/bin/env python3
"""Translates VU1 microcode images into C++ (static recompilation of VU1 programs).

usage: vu1_translate.py <out_dir> <image.bin>...

Each 16 KB image (as harvested with PS2X_VU1_HARVEST) becomes
<out_dir>/vu1_<hash>.cpp defining `void vu1prog_<hash>(vu1rec::Ctx &)`, and
<out_dir>/vu1_registry.cpp lists them by image hash. The generated code runs a
program from s.pc to its end (E-bit) with the timing model of the runtime's
VU1 interpreter: issue cycles with stalls on VF/VI results not yet written,
and Q, P, MAC and clip results that become visible after their latencies.
See src/vu1rec/vu1rec_rt.h for the helpers it uses.
"""

import pathlib
import struct
import sys

PAIRS = 2048
CODE_MASK = 0x3FFF       # micro memory wraps here (VU0: 4 KB)
UNIT = "vu1"             # --vu0 translates VU0 images (no EFU, no XGKICK)
QADDR = "vu1rec::qaddr"  # data memory address helper (VU0: 4 KB)
FMAC_LATENCY = 4
LANE_BIT = (8, 4, 2, 1)  # x, y, z, w as in the DEST field
LANE_NAME = "xyzw"


def lanes_of(mask):
    return [l for l in range(4) if mask & LANE_BIT[l]]


def lane_mask(lane):
    return LANE_BIT[lane]


def sext(value, bits):
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


class Usage:
    """What an instruction reads and writes, mirroring decode*Usage()."""

    def __init__(self):
        self.vf_reads = []  # (reg, lane mask)
        self.vf_write = None  # (reg, lane mask)
        self.vi_reads = set()
        self.vi_write = None
        self.latency = 0
        self.vf_latency = 0
        self.vi_latency = 0
        self.pipeline = None  # "fmac" "lsu" "fdiv" "efu" "ialu" "branch" "xgkick"
        self.wait_q = False
        self.wait_p = False
        self.delays_branch = False
        self.reserved = False

    def read_vf(self, reg, mask):
        if reg != 0 and mask:
            self.vf_reads.append((reg, mask))

    def write_vf(self, reg, mask):
        self.vf_write = (reg, mask)

    def read_vi(self, reg):
        if reg != 0:
            self.vi_reads.add(reg)

    def write_vi(self, reg):
        if reg != 0:
            self.vi_write = reg


# ---------------------------------------------------------------- fields

def DEST(w): return (w >> 21) & 0xF
def FT(w): return (w >> 16) & 0x1F
def FS(w): return (w >> 11) & 0x1F
def FD(w): return (w >> 6) & 0x1F
def VIT(w): return (w >> 16) & 0xF
def VIS(w): return (w >> 11) & 0xF
def VID(w): return (w >> 6) & 0xF
def IMM11(w): return sext(w & 0x7FF, 11)


def upper_special(u):
    return (u & 3) | ((u >> 4) & 0x7C)


def lower_special(l):
    return (l & 3) | ((l >> 4) & 0x7C)


# ---------------------------------------------------------------- usage

FLAG_WRITING_SPECIAL = set(range(0x00, 0x10)) | set(range(0x18, 0x1D)) | {0x1E} | \
    set(range(0x20, 0x2B)) | {0x2C, 0x2D, 0x2E}


def upper_usage(u):
    usage = Usage()
    usage.pipeline = "fmac"
    usage.latency = FMAC_LATENCY
    op = u & 0x3F
    dest, ft, fs, fd = DEST(u), FT(u), FS(u), FD(u)
    if op <= 0x2F:
        usage.read_vf(fs, dest)
        usage.write_vf(fd, dest)
        if op <= 0x1B:
            usage.read_vf(ft, lane_mask(op & 3))
        elif op >= 0x28:
            usage.read_vf(ft, 0xE if op == 0x2E else dest)
        return usage
    if op >= 0x3C:
        sp = upper_special(u)
        writes_acc = sp <= 0x0F or 0x18 <= sp <= 0x1C or sp == 0x1E or \
            0x20 <= sp <= 0x2A or 0x2C <= sp <= 0x2E
        if writes_acc:
            usage.read_vf(fs, dest)
            if sp <= 0x1B:
                usage.read_vf(ft, lane_mask(sp & 3))
            elif 0x28 <= sp <= 0x2E:
                usage.read_vf(ft, 0xE if sp == 0x2E else dest)
        elif 0x10 <= sp <= 0x17 or sp == 0x1D:
            usage.read_vf(fs, dest)
            usage.write_vf(ft, dest)
        elif sp == 0x1F:
            usage.read_vf(fs, 0xE)
            usage.read_vf(ft, 0x1)
        elif sp not in (0x2F, 0x30):
            usage.reserved = True
        return usage
    usage.reserved = True
    return usage


EFU_LATENCY = {0x70: 11, 0x71: 18, 0x72: 18, 0x77: 18, 0x73: 24, 0x74: 54, 0x75: 54,
               0x7C: 54, 0x76: 12, 0x78: 12, 0x7A: 12, 0x79: 29, 0x7D: 44}


def lower_usage(l):
    usage = Usage()
    if l in (0, 0x8000033C):
        return usage
    op = (l >> 25) & 0x7F
    vft, vfs = FT(l), FS(l)
    vit, vis, vid = VIT(l), VIS(l), VID(l)
    dest = DEST(l)
    if op == 0x00:  # LQ
        usage.pipeline, usage.latency = "lsu", 4
        usage.read_vi(vis)
        usage.write_vf(vft, dest)
    elif op == 0x01:  # SQ
        usage.pipeline, usage.latency = "lsu", 1
        usage.read_vi(vit)
        usage.read_vf(vfs, dest)
    elif op == 0x04:  # ILW
        usage.pipeline, usage.latency = "lsu", 4
        usage.read_vi(vis)
        usage.write_vi(vit)
    elif op == 0x05:  # ISW
        usage.pipeline, usage.latency = "lsu", 1
        usage.read_vi(vis)
        usage.read_vi(vit)
    elif op in (0x08, 0x09):  # IADDIU ISUBIU
        usage.pipeline, usage.latency = "ialu", 1
        usage.delays_branch = True
        usage.read_vi(vis)
        usage.write_vi(vit)
    elif op in (0x10, 0x12, 0x13):  # FCEQ FCAND FCOR
        usage.pipeline, usage.latency = "ialu", 1
        usage.write_vi(1)
    elif op in (0x11, 0x15):  # FCSET FSSET
        usage.pipeline, usage.latency = "fmac", FMAC_LATENCY
    elif op in (0x14, 0x16, 0x17):  # FSEQ FSAND FSOR
        usage.pipeline, usage.latency = "ialu", 1
        usage.write_vi(vit)
    elif op in (0x18, 0x1A, 0x1B):  # FMEQ FMAND FMOR
        usage.pipeline, usage.latency = "ialu", 1
        usage.read_vi(vis)
        usage.write_vi(vit)
    elif op == 0x1C:  # FCGET
        usage.pipeline, usage.latency = "ialu", 1
        usage.write_vi(vit)
    elif op == 0x20:  # B
        usage.pipeline = "branch"
    elif op == 0x21:  # BAL
        usage.pipeline, usage.latency = "branch", 1
        usage.write_vi(vit)
    elif op == 0x24:  # JR
        usage.pipeline = "branch"
        usage.read_vi(vis)
    elif op == 0x25:  # JALR
        usage.pipeline, usage.latency = "branch", 1
        usage.read_vi(vis)
        usage.write_vi(vit)
    elif op in (0x28, 0x29):  # IBEQ IBNE
        usage.pipeline = "branch"
        usage.read_vi(vis)
        usage.read_vi(vit)
    elif op in (0x2C, 0x2D, 0x2E, 0x2F):  # IBLTZ IBGTZ IBLEZ IBGEZ
        usage.pipeline = "branch"
        usage.read_vi(vis)
    elif op == 0x40:
        funct = l & 0x3F
        if funct in (0x30, 0x31, 0x34, 0x35):  # IADD ISUB IAND IOR
            usage.pipeline, usage.latency = "ialu", 1
            usage.delays_branch = True
            usage.read_vi(vis)
            usage.read_vi(vit)
            usage.write_vi(vid)
        elif funct == 0x32:  # IADDI
            usage.pipeline, usage.latency = "ialu", 1
            usage.delays_branch = True
            usage.read_vi(vis)
            usage.write_vi(vit)
        elif funct < 0x3C:
            usage.reserved = True
        else:
            sp = lower_special(l)
            fsf, ftf = (l >> 21) & 3, (l >> 23) & 3
            if sp in (0x30, 0x31):  # MOVE MR32
                usage.pipeline, usage.latency = "fmac", 4
                usage.read_vf(vfs, 0xF if sp == 0x31 else dest)
                usage.write_vf(vft, dest)
            elif sp in (0x34, 0x36):  # LQI LQD
                usage.pipeline, usage.latency, usage.vi_latency = "lsu", 4, 1
                usage.delays_branch = True
                usage.read_vi(vis)
                usage.write_vi(vis)
                usage.write_vf(vft, dest)
            elif sp in (0x35, 0x37):  # SQI SQD
                usage.pipeline, usage.latency = "lsu", 1
                usage.delays_branch = True
                usage.read_vi(vit)
                usage.write_vi(vit)
                usage.read_vf(vfs, dest)
            elif sp == 0x38:  # DIV
                usage.pipeline, usage.latency = "fdiv", 7
                usage.read_vf(vfs, lane_mask(fsf))
                usage.read_vf(vft, lane_mask(ftf))
            elif sp == 0x39:  # SQRT
                usage.pipeline, usage.latency = "fdiv", 7
                usage.read_vf(vft, lane_mask(ftf))
            elif sp == 0x3A:  # RSQRT
                usage.pipeline, usage.latency = "fdiv", 13
                usage.read_vf(vfs, lane_mask(fsf))
                usage.read_vf(vft, lane_mask(ftf))
            elif sp == 0x3B:  # WAITQ
                usage.pipeline = "fdiv"
                usage.wait_q = True
            elif sp == 0x3C:  # MTIR
                usage.pipeline, usage.latency = "ialu", 1
                usage.delays_branch = True
                usage.read_vf(vfs, lane_mask((l >> 21) & 3))
                usage.write_vi(vit)
            elif sp == 0x3D:  # MFIR
                usage.pipeline, usage.latency = "fmac", 4
                usage.read_vi(vis)
                usage.write_vf(vft, dest)
            elif sp == 0x3E:  # ILWR
                usage.pipeline, usage.latency = "lsu", 4
                usage.read_vi(vis)
                usage.write_vi(vit)
            elif sp == 0x3F:  # ISWR
                usage.pipeline, usage.latency = "lsu", 1
                usage.read_vi(vis)
                usage.read_vi(vit)
            elif sp in (0x40, 0x41):  # RNEXT RGET
                usage.pipeline, usage.latency = "fmac", 4
                usage.write_vf(vft, dest)
            elif sp in (0x42, 0x43):  # RINIT RXOR
                usage.pipeline, usage.latency = "ialu", 1
                usage.read_vf(vfs, lane_mask((l >> 21) & 3))
            elif UNIT == "vu0" and (sp in (0x64, 0x6C, 0x7B) or sp in EFU_LATENCY):
                usage.reserved = True  # VU0 has no EFU and no GIF path
            elif sp == 0x64:  # MFP
                usage.pipeline, usage.latency = "fmac", 4
                usage.write_vf(vft, dest)
            elif sp in (0x68, 0x69):  # XTOP XITOP
                usage.pipeline, usage.latency = "ialu", 1
                usage.write_vi(vit)
            elif sp == 0x6C:  # XGKICK
                usage.pipeline, usage.latency = "xgkick", 2
                usage.read_vi(vis)
            elif sp in EFU_LATENCY:
                usage.pipeline, usage.latency = "efu", EFU_LATENCY[sp]
                if 0x70 <= sp <= 0x73:
                    usage.read_vf(vfs, 0xE)
                elif sp == 0x74:
                    usage.read_vf(vfs, 0xC)
                elif sp == 0x75:
                    usage.read_vf(vfs, 0xA)
                elif sp == 0x76:
                    usage.read_vf(vfs, 0xF)
                else:
                    usage.read_vf(vfs, lane_mask((l >> 21) & 3))
            elif sp == 0x7B:  # WAITP
                usage.pipeline = "efu"
                usage.wait_p = True
            else:
                usage.reserved = True
    else:
        usage.reserved = True
    return usage


COND_BRANCHES = {0x28: "==", 0x29: "!=", 0x2C: "<", 0x2D: ">", 0x2E: "<=", 0x2F: ">="}
STATIC_BRANCHES = {0x20, 0x21} | set(COND_BRANCHES)
DYNAMIC_BRANCHES = {0x24, 0x25}


class Pair:
    def __init__(self, pc, lower, upper):
        self.pc = pc
        self.lower = lower
        self.upper = upper
        self.ibit = bool(upper & 0x80000000)
        self.ebit = bool(upper & 0x40000000)
        self.dbit = bool(upper & 0x10000000)
        self.tbit = bool(upper & 0x08000000)
        self.uu = upper_usage(upper)
        self.lu = Usage() if self.ibit else lower_usage(lower)
        self.lop = None if self.ibit else (lower >> 25) & 0x7F

    def is_branch(self):
        return self.lop in STATIC_BRANCHES or self.lop in DYNAMIC_BRANCHES

    def static_target(self):
        return (self.pc + 8 + IMM11(self.lower) * 8) & CODE_MASK


# ---------------------------------------------------------------- emission

def vf(reg, lane):
    return "s.vf[%d][%d]" % (reg, lane)


def upper_code(p, mac_live=True):
    """Returns (compute lines, commit lines) for the upper op; MAC flags are
    produced only when a reader may see them (mac_live)."""
    u = p.upper
    op = u & 0x3F
    dest, ft, fs, fd = DEST(u), FT(u), FS(u), FD(u)
    lanes = lanes_of(dest)
    compute, commit = [], []

    def acc(l):
        return "s.acc[%d]" % l

    def fmac(target, exprs, flags=True):
        # exprs: lane -> expression; result kept in temporaries until commit.
        for l in lanes:
            compute.append("const float u%d = %s;" % (l, exprs(l)))
        if flags and lanes and mac_live:
            compute.append("vu1rec::pushMac(c, cyc, %s);" % " | ".join(
                "vu1rec::macBits(u%d, %d)" % (l, LANE_BIT[l]) for l in lanes))
        if target == "acc":
            for l in lanes:
                commit.append("s.acc[%d] = u%d;" % (l, l))
        elif target != 0:
            for l in lanes:
                commit.append("%s = u%d;" % (vf(target, l), l))

    if op < 0x3C:
        bc = op & 3
        vs = lambda l: vf(fs, l)
        vt = lambda l: vf(ft, l)
        bcv = vf(ft, bc)
        table = {
            0: lambda l: "%s + %s" % (vs(l), bcv),
            1: lambda l: "%s - %s" % (vs(l), bcv),
            2: lambda l: "%s + %s * %s" % (acc(l), vs(l), bcv),
            3: lambda l: "%s - %s * %s" % (acc(l), vs(l), bcv),
            6: lambda l: "%s * %s" % (vs(l), bcv),
        }
        group = op >> 2
        if op <= 0x0F or 0x18 <= op <= 0x1B:
            fmac(fd, table[group])
        elif 0x10 <= op <= 0x13:
            fmac(fd, lambda l: "vu1rec::vmax(%s, %s)" % (vs(l), bcv), flags=False)
        elif 0x14 <= op <= 0x17:
            fmac(fd, lambda l: "vu1rec::vmin(%s, %s)" % (vs(l), bcv), flags=False)
        else:
            simple = {
                0x1C: lambda l: "%s * s.q" % vs(l),
                0x1E: lambda l: "%s * s.i" % vs(l),
                0x20: lambda l: "%s + s.q" % vs(l),
                0x21: lambda l: "%s + %s * s.q" % (acc(l), vs(l)),
                0x22: lambda l: "%s + s.i" % vs(l),
                0x23: lambda l: "%s + %s * s.i" % (acc(l), vs(l)),
                0x24: lambda l: "%s - s.q" % vs(l),
                0x25: lambda l: "%s - %s * s.q" % (acc(l), vs(l)),
                0x26: lambda l: "%s - s.i" % vs(l),
                0x27: lambda l: "%s - %s * s.i" % (acc(l), vs(l)),
                0x28: lambda l: "%s + %s" % (vs(l), vt(l)),
                0x29: lambda l: "%s + %s * %s" % (acc(l), vs(l), vt(l)),
                0x2A: lambda l: "%s * %s" % (vs(l), vt(l)),
                0x2C: lambda l: "%s - %s" % (vs(l), vt(l)),
                0x2D: lambda l: "%s - %s * %s" % (acc(l), vs(l), vt(l)),
            }
            if op in simple:
                fmac(fd, simple[op])
            elif op == 0x1D:
                fmac(fd, lambda l: "vu1rec::vmax(%s, s.i)" % vs(l), flags=False)
            elif op == 0x1F:
                fmac(fd, lambda l: "vu1rec::vmin(%s, s.i)" % vs(l), flags=False)
            elif op == 0x2B:
                fmac(fd, lambda l: "vu1rec::vmax(%s, %s)" % (vs(l), vt(l)), flags=False)
            elif op == 0x2F:
                fmac(fd, lambda l: "vu1rec::vmin(%s, %s)" % (vs(l), vt(l)), flags=False)
            elif op == 0x2E:  # OPMSUB
                cross = {0: (1, 2), 1: (2, 0), 2: (0, 1)}
                fmac(fd, lambda l: "0.0f" if l == 3 else "%s - %s * %s" % (
                    acc(l), vf(fs, cross[l][0]), vf(ft, cross[l][1])))
            else:
                return None
        return compute, commit

    sp = upper_special(u)
    vs = lambda l: vf(fs, l)
    vt = lambda l: vf(ft, l)
    if sp <= 0x0F or 0x18 <= sp <= 0x1B:
        bcv = vf(ft, sp & 3)
        exprs = {
            0: lambda l: "%s + %s" % (vs(l), bcv),
            1: lambda l: "%s - %s" % (vs(l), bcv),
            2: lambda l: "%s + %s * %s" % (acc(l), vs(l), bcv),
            3: lambda l: "%s - %s * %s" % (acc(l), vs(l), bcv),
            6: lambda l: "%s * %s" % (vs(l), bcv),
        }[sp >> 2]
        fmac("acc", exprs)
    elif 0x10 <= sp <= 0x13:  # ITOF
        scale = {0x10: "1.0f", 0x11: "16.0f", 0x12: "4096.0f", 0x13: "32768.0f"}[sp]
        fmac(ft, lambda l: "vu1rec::itof(%s, %s)" % (vs(l), scale), flags=False)
    elif 0x14 <= sp <= 0x17:  # FTOI
        scale = {0x14: "1.0f", 0x15: "16.0f", 0x16: "4096.0f", 0x17: "32768.0f"}[sp]
        fmac(ft, lambda l: "vu1rec::ftoi(%s, %s)" % (vs(l), scale), flags=False)
    elif sp == 0x1D:  # ABS
        fmac(ft, lambda l: "std::fabs(%s)" % vs(l), flags=False)
    elif sp == 0x1F:  # CLIP
        compute.append("vu1rec::clip(c, cyc, s.vf[%d], %s);" % (fs, vf(ft, 3)))
    elif sp in (0x2F, 0x30):  # NOP
        pass
    else:
        simple = {
            0x1C: lambda l: "%s * s.q" % vs(l),
            0x1E: lambda l: "%s * s.i" % vs(l),
            0x20: lambda l: "%s + s.q" % vs(l),
            0x21: lambda l: "%s + %s * s.q" % (acc(l), vs(l)),
            0x22: lambda l: "%s + s.i" % vs(l),
            0x23: lambda l: "%s + %s * s.i" % (acc(l), vs(l)),
            0x24: lambda l: "%s - s.q" % vs(l),
            0x25: lambda l: "%s - %s * s.q" % (acc(l), vs(l)),
            0x26: lambda l: "%s - s.i" % vs(l),
            0x27: lambda l: "%s - %s * s.i" % (acc(l), vs(l)),
            0x28: lambda l: "%s + %s" % (vs(l), vt(l)),
            0x29: lambda l: "%s + %s * %s" % (acc(l), vs(l), vt(l)),
            0x2A: lambda l: "%s * %s" % (vs(l), vt(l)),
            0x2C: lambda l: "%s - %s" % (vs(l), vt(l)),
            0x2D: lambda l: "%s - %s * %s" % (acc(l), vs(l), vt(l)),
        }
        if sp in simple:
            fmac("acc", simple[sp])
        elif sp == 0x2E:  # OPMULA
            cross = {0: (1, 2), 1: (2, 0), 2: (0, 1)}
            fmac("acc", lambda l: "0.0f" if l == 3 else "%s * %s" % (
                vf(fs, cross[l][0]), vf(ft, cross[l][1])))
        else:
            return None
    return compute, commit


def addr(base_reg, imm):
    return "%s(s.vi[%d] + %d)" % (QADDR, base_reg, imm)


def lower_code(p, suppress_vf_write, branch_vi):
    """Returns lower op lines; branch decisions set `br`/`brT`."""
    l = p.lower
    if l in (0, 0x8000033C):
        return []
    op = p.lop
    vft, vfs = FT(l), FS(l)
    vit, vis, vid = VIT(l), VIS(l), VID(l)
    dest = DEST(l)
    lanes = lanes_of(dest)
    out = []

    def set_vi(reg, expr):
        if reg != 0:
            out.append("s.vi[%d] = %s;" % (reg, expr))

    def store_vf(reg, lanes_, source):
        if reg == 0 or suppress_vf_write:
            return
        for ln in lanes_:
            out.append("%s = %s;" % (vf(reg, ln), source(ln)))

    if op == 0x00:  # LQ
        out.append("{ const uint32_t a = %s;" % addr(vis, IMM11(l)))
        store_vf(vft, lanes, lambda ln: "vu1rec::ldf(mem, a + %d)" % (ln * 4))
        out.append("}")
    elif op == 0x01:  # SQ
        out.append("{ const uint32_t a = %s;" % addr(vit, IMM11(l)))
        for ln in lanes:
            out.append("vu1rec::stf(mem, a + %d, %s);" % (ln * 4, vf(vfs, ln)))
        out.append("}")
    elif op == 0x04:  # ILW
        comp = lanes[0] if lanes else 3
        set_vi(vit, "vu1rec::ldi(mem, %s + %d)" % (addr(vis, IMM11(l)), comp * 4))
    elif op == 0x05:  # ISW
        out.append("{ const uint32_t a = %s; const uint32_t v = (uint16_t)s.vi[%d];" % (addr(vis, IMM11(l)), vit))
        for ln in lanes:
            out.append("vu1rec::stu(mem, a + %d, v);" % (ln * 4))
        out.append("}")
    elif op in (0x08, 0x09):  # IADDIU ISUBIU
        imm = (l & 0x7FF) | ((l >> 10) & 0x7800)
        set_vi(vit, "(int16_t)(s.vi[%d] %s %d)" % (vis, "+" if op == 0x08 else "-", imm))
    elif op == 0x10:  # FCEQ
        out.append("vu1rec::syncFlags(c, cyc);")
        set_vi(1, "(s.clip & 0xFFFFFF) == 0x%06Xu ? 1 : 0" % (l & 0xFFFFFF))
    elif op == 0x12:  # FCAND
        out.append("vu1rec::syncFlags(c, cyc);")
        set_vi(1, "(s.clip & 0x%06Xu) != 0 ? 1 : 0" % (l & 0xFFFFFF))
    elif op == 0x13:  # FCOR
        out.append("vu1rec::syncFlags(c, cyc);")
        set_vi(1, "(s.clip | 0x%06Xu) == 0xFFFFFFu ? 1 : 0" % (l & 0xFFFFFF))
    elif op == 0x11:  # FCSET
        out.append("vu1rec::fcset(c, cyc, 0x%06Xu);" % (l & 0xFFFFFF))
    elif op in (0x18, 0x1A, 0x1B):  # FMEQ FMAND FMOR
        expr = {0x18: "(s.mac & 0xFFFF) == (uint32_t)(uint16_t)s.vi[%d] ? 1 : 0",
                0x1A: "(int32_t)(s.mac & (uint32_t)(uint16_t)s.vi[%d])",
                0x1B: "(int32_t)(s.mac | (uint32_t)(uint16_t)s.vi[%d])"}[op] % vis
        out.append("vu1rec::syncFlags(c, cyc);")
        set_vi(vit, expr)
    elif op == 0x1C:  # FCGET
        out.append("vu1rec::syncFlags(c, cyc);")
        set_vi(vit, "(int32_t)(s.clip & 0xFFFu)")
    elif op == 0x20:  # B
        out.append("br = 1; brT = 0x%04X;" % p.static_target())
    elif op == 0x21:  # BAL
        set_vi(vit, "%d" % ((p.pc + 16) // 8))
        out.append("br = 1; brT = 0x%04X;" % p.static_target())
    elif op in (0x24, 0x25):  # JR JALR
        out.append("{ const uint32_t t = ((uint32_t)(uint16_t)s.vi[%d] * 8u) & 0x%04Xu;" % (vis, CODE_MASK))
        if op == 0x25:
            set_vi(vit, "%d" % ((p.pc + 16) // 8))
        out.append("br = 1; brT = t; }")
    elif op in COND_BRANCHES:
        cmp = COND_BRANCHES[op]
        a = "(int16_t)%s" % branch_vi(vis)
        b = "(int16_t)%s" % branch_vi(vit) if op in (0x28, 0x29) else "0"
        out.append("if (%s %s %s) { br = 1; brT = 0x%04X; }" % (a, cmp, b, p.static_target()))
    elif op == 0x40:
        funct = l & 0x3F
        if funct == 0x30:
            set_vi(vid, "(int16_t)(s.vi[%d] + s.vi[%d])" % (vis, vit))
        elif funct == 0x31:
            set_vi(vid, "(int16_t)(s.vi[%d] - s.vi[%d])" % (vis, vit))
        elif funct == 0x32:
            set_vi(vit, "(int16_t)(s.vi[%d] + %d)" % (vis, sext((l >> 6) & 0x1F, 5)))
        elif funct == 0x34:
            set_vi(vid, "s.vi[%d] & s.vi[%d]" % (vis, vit))
        elif funct == 0x35:
            set_vi(vid, "s.vi[%d] | s.vi[%d]" % (vis, vit))
        else:
            sp = lower_special(l)
            fsf, ftf = (l >> 21) & 3, (l >> 23) & 3
            if sp == 0x30:  # MOVE
                if vft != 0 and not suppress_vf_write:
                    out.append("{ float t[4]; std::memcpy(t, s.vf[%d], 16);" % vfs)
                    for ln in lanes:
                        out.append("%s = t[%d];" % (vf(vft, ln), ln))
                    out.append("}")
            elif sp == 0x31:  # MR32
                if vft != 0 and not suppress_vf_write:
                    out.append("{ const float t[4] = {%s, %s, %s, %s};" % (
                        vf(vfs, 1), vf(vfs, 2), vf(vfs, 3), vf(vfs, 0)))
                    for ln in lanes:
                        out.append("%s = t[%d];" % (vf(vft, ln), ln))
                    out.append("}")
            elif sp in (0x34, 0x36):  # LQI LQD
                if sp == 0x36 and vis != 0:
                    out.append("s.vi[%d] = (int16_t)(s.vi[%d] - 1);" % (vis, vis))
                out.append("{ const uint32_t a = %s((uint16_t)s.vi[%d]);" % (QADDR, vis))
                store_vf(vft, lanes, lambda ln: "vu1rec::ldf(mem, a + %d)" % (ln * 4))
                out.append("}")
                if sp == 0x34 and vis != 0:
                    out.append("s.vi[%d] = (int16_t)(s.vi[%d] + 1);" % (vis, vis))
            elif sp in (0x35, 0x37):  # SQI SQD
                if sp == 0x37 and vit != 0:
                    out.append("s.vi[%d] = (int16_t)(s.vi[%d] - 1);" % (vit, vit))
                out.append("{ const uint32_t a = %s((uint16_t)s.vi[%d]);" % (QADDR, vit))
                for ln in lanes:
                    out.append("vu1rec::stf(mem, a + %d, %s);" % (ln * 4, vf(vfs, ln)))
                out.append("}")
                if sp == 0x35 and vit != 0:
                    out.append("s.vi[%d] = (int16_t)(s.vi[%d] + 1);" % (vit, vit))
            elif sp == 0x38:
                out.append("vu1rec::fdiv(c, cyc, vu1rec::vdiv(%s, %s), 7);" % (vf(vfs, fsf), vf(vft, ftf)))
            elif sp == 0x39:
                out.append("vu1rec::fdiv(c, cyc, vu1rec::vsqrt(%s), 7);" % vf(vft, ftf))
            elif sp == 0x3A:
                out.append("vu1rec::fdiv(c, cyc, vu1rec::vrsqrt(%s, %s), 13);" % (vf(vfs, fsf), vf(vft, ftf)))
            elif sp == 0x3B:  # WAITQ: handled by the stall
                pass
            elif sp == 0x3C:  # MTIR
                set_vi(vit, "vu1rec::lo16(%s)" % vf(vfs, (l >> 21) & 3))
            elif sp == 0x3D:  # MFIR
                if vft != 0 and not suppress_vf_write:
                    out.append("{ const float t = vu1rec::i2f((int32_t)(int16_t)s.vi[%d]);" % vis)
                    for ln in lanes:
                        out.append("%s = t;" % vf(vft, ln))
                    out.append("}")
            elif sp == 0x3E:  # ILWR
                comp = lanes[0] if lanes else 3
                set_vi(vit, "vu1rec::ldi(mem, %s((uint16_t)s.vi[%d]) + %d)" % (QADDR, vis, comp * 4))
            elif sp == 0x3F:  # ISWR
                out.append("{ const uint32_t a = %s((uint16_t)s.vi[%d]); const uint32_t v = (uint16_t)s.vi[%d];" % (QADDR, vis, vit))
                for ln in lanes:
                    out.append("vu1rec::stu(mem, a + %d, v);" % (ln * 4))
                out.append("}")
            elif sp in (0x40, 0x41, 0x42, 0x43):
                out.append("vu1rec::rop(s, 0x%02X, %d, %d, %d, %d);" % (sp, vft if not suppress_vf_write else 0, vfs, dest, (l >> 21) & 3))
            elif sp == 0x64:  # MFP
                if vft != 0 and not suppress_vf_write:
                    for ln in lanes:
                        out.append("%s = s.p;" % vf(vft, ln))
            elif sp == 0x68:
                set_vi(vit, "(int32_t)(s.top & 0x3FFu)")
            elif sp == 0x69:
                set_vi(vit, "(int32_t)(s.itop & 0x3FFu)")
            elif sp == 0x6C:
                out.append("vu1rec::xgkick(c, cyc, (uint16_t)s.vi[%d]);" % vis)
            elif sp in EFU_LATENCY:
                out.append("vu1rec::efu(c, cyc, 0x%02X, s.vf[%d], %d, %d);" % (sp, vfs, (l >> 21) & 3, EFU_LATENCY[sp]))
            elif sp == 0x7B:  # WAITP: handled by the stall
                pass
            else:
                return None
    else:
        return None
    return out


MAC_READERS = {0x18, 0x1A, 0x1B}  # FMEQ FMAND FMOR
Q_READERS = {0x1C, 0x20, 0x21, 0x24, 0x25}  # MULq ADDq MADDq SUBq MSUBq (and the ACC forms)


def reads_q(p):
    """Whether the pair's upper instruction takes Q as an operand."""
    u = p.upper
    op = u & 0x3F
    if op <= 0x2F:
        return op in Q_READERS
    if op >= 0x3C:
        return upper_special(u) in Q_READERS
    return False


def needs_commit(p):
    """Q and P results only have to reach s.q / s.p before something reads
    them: at a Q or P reader, before another FDIV or EFU result is queued
    (WAITQ/WAITP included), and at the end of the program (finish())."""
    if reads_q(p):
        return True
    if p.ibit:
        return False
    if p.lu.pipeline in ("fdiv", "efu") or p.lu.wait_q or p.lu.wait_p:
        return True
    return p.lop == 0x40 and lower_special(p.lower) == 0x64  # MFP


def writes_mac(p):
    """Whether the pair's upper instruction updates the MAC flags."""
    u = p.upper
    op = u & 0x3F
    if DEST(u) == 0:
        return False
    if op < 0x3C:
        return op <= 0x0F or 0x18 <= op <= 0x1C or op == 0x1E or 0x20 <= op <= 0x2A and op != 0x2B \
            or op in (0x2C, 0x2D, 0x2E)
    return upper_special(u) in FLAG_WRITING_SPECIAL


def live_mac_writers(pairs):
    """MAC writers whose flags some reader may see.

    A reader at cycle C sees the latest writer issued at or before C - 4.
    Writers issue at least a cycle apart, so with four writers between a
    writer and a reader, one of them is visible instead. A writer is live if
    some path reaches a reader (or the program's end or a computed jump,
    where the next program may read s.mac) through at most three writers.
    """
    count = len(pairs)
    successors = [[] for _ in range(count)]
    sink = [False] * count  # flow leaves the known code after this pair
    for i, p in enumerate(pairs):
        nxt = (i + 1) % count
        prev = pairs[i - 1] if i > 0 else None
        if prev is not None and prev.ebit:
            sink[i] = True
            continue
        if prev is not None and prev.is_branch():
            if prev.lop in DYNAMIC_BRANCHES:
                sink[i] = True
                successors[i].append(nxt)  # not taken never happens, kept conservative
            else:
                successors[i].append(prev.static_target() // 8)
                if prev.lop in COND_BRANCHES:
                    successors[i].append(nxt)
            continue
        successors[i].append(nxt)

    def reader(p):
        return not p.ibit and p.lop in MAC_READERS

    INF = 99
    # after[i]: fewest writers between pair i and a reader along a path.
    after = [INF] * count
    changed = True
    while changed:
        changed = False
        for i in range(count):
            best = 0 if sink[i] else INF
            for s in successors[i]:
                q = pairs[s]
                if reader(q):
                    value = 0
                else:
                    value = min(INF, after[s] + (1 if writes_mac(q) else 0))
                best = min(best, value)
            if best < after[i]:
                after[i] = best
                changed = True
    return {p.pc for i, p in enumerate(pairs) if writes_mac(p) and after[i] <= 3}


def pair_effects(p):
    """(VF lane reads, VI reads, VF lane writes {key: latency}, VI write or None)."""
    vf_reads = []
    for reg, mask in p.uu.vf_reads + p.lu.vf_reads:
        for ln in lanes_of(mask):
            if ("vf", reg, ln) not in vf_reads:
                vf_reads.append(("vf", reg, ln))
    vi_reads = [("vi", reg) for reg in sorted(p.lu.vi_reads)]
    writes = {}
    upper_reg = p.uu.vf_write[0] if p.uu.vf_write else 0
    suppressed = bool(upper_reg) and p.lu.vf_write is not None and p.lu.vf_write[0] == upper_reg
    if p.lu.vf_write and p.lu.vf_write[0] != 0 and not suppressed:
        reg, mask = p.lu.vf_write
        for ln in lanes_of(mask):
            writes[("vf", reg, ln)] = p.lu.vf_latency or p.lu.latency
    if p.uu.vf_write and upper_reg != 0:
        for ln in lanes_of(p.uu.vf_write[1]):
            writes[("vf", upper_reg, ln)] = FMAC_LATENCY
    if p.lu.vi_write:
        writes[("vi", p.lu.vi_write)] = p.lu.vi_latency or p.lu.latency
    return vf_reads + vi_reads, writes


def region_leaders(pairs, entries):
    """Pairs where execution may arrive other than from the pair before:
    program entries, branch targets, and the pairs after delay slots and
    E-bit delay slots (fall-through, return points, MSCNT continuation)."""
    leaders = {0} | set(entries)
    for i, p in enumerate(pairs):
        if p.is_branch():
            if p.lop in STATIC_BRANCHES:
                leaders.add(p.static_target())
            leaders.add((p.pc + 16) & CODE_MASK)
        if p.ebit:
            leaders.add((p.pc + 16) & CODE_MASK)
    return leaders


def plan_scoreboard(pairs, leaders):
    """Decides which operand checks and ready-cycle stores need the runtime
    scoreboard. Within a straight-line region, a result produced at least its
    latency before a read is known to be ready; a ready cycle is stored only
    if a nearby read needs it or the value may be read outside the region.
    (A computed jump into the middle of a region skips its earlier pairs:
    such targets are expected to be region leaders, e.g. jump tables of
    branches.)"""
    effects = [pair_effects(p) for p in pairs]
    starts = []
    start = 0
    for i, p in enumerate(pairs):
        if p.pc in leaders:
            start = i
        starts.append(start)
    region_end = [0] * len(pairs)
    end = len(pairs)
    for i in range(len(pairs) - 1, -1, -1):
        region_end[i] = end
        if pairs[i].pc in leaders:
            end = i

    waits = []
    for j in range(len(pairs)):
        needed = []
        for key in effects[j][0]:
            writer = None
            for i in range(j - 1, starts[j] - 1, -1):
                if key in effects[i][1]:
                    writer = i
                    break
            if writer is None or j - writer < effects[writer][1][key]:
                needed.append(key)
        waits.append(needed)

    stores = []
    for i in range(len(pairs)):
        kept = []
        for key, latency in effects[i][1].items():
            keep = True
            for j in range(i + 1, region_end[i]):
                if key in effects[j][0] and j - i < latency:
                    break  # a close read needs the stored cycle
                if key in effects[j][1]:
                    keep = False  # overwritten before anything needs it
                    break
            kept.append((key, latency) if keep else None)
        stores.append([k for k in kept if k])
    return waits, stores


def ready_ref(key):
    if key[0] == "vf":
        return "c.vfReady[%d][%d]" % (key[1], key[2])
    return "c.viReady[%d]" % key[1]


def translate(image, name, entries=()):
    pairs = [Pair(pc * 8, *struct.unpack_from("<II", image, pc * 8)) for pc in range(PAIRS)]
    live_mac = live_mac_writers(pairs)
    leaders = region_leaders(pairs, entries)
    waits, stores = plan_scoreboard(pairs, leaders)
    lines = []
    emit = lines.append
    warnings = []

    emit("// Generated by tools/vu1_translate.py from %s microcode image %s. Do not edit." % (UNIT.upper(), name))
    emit('#include "vu1rec/vu1rec_rt.h"')
    emit("")
    emit("void %sprog_%s(vu1rec::Ctx &c)" % (UNIT, name))
    emit("{")
    emit("    VU1State &s = *c.s;")
    # Data memory never overlaps the registers: without this every store to
    # it would force the registers to be reloaded.
    emit("    uint8_t *__restrict const mem = c.mem;")
    emit("    uint64_t cyc = c.cyc;")
    emit("    uint32_t pc = s.pc & 0x%04Xu;" % (CODE_MASK & ~7))
    emit("    uint32_t brT = 0;")
    emit("    int br = 0;")
    emit("    int ending = 0;")
    emit("    int32_t bak = 0;")
    emit("    uint32_t bakPc = ~0u;")
    emit("    (void)bak; (void)bakPc;")
    emit("dispatch:")
    emit("    switch (pc)")
    emit("    {")
    for p in pairs:
        emit("    case 0x%04X: goto L%04X;" % (p.pc, p.pc))
    emit("    default: break;")
    emit("    }")
    emit("    vu1rec::trap(c, cyc, pc);")
    emit("    return;")

    for index, p in enumerate(pairs):
        prev = pairs[index - 1] if index > 0 else None
        nxt = pairs[index + 1] if index + 1 < PAIRS else None
        emit("L%04X:" % p.pc)
        uc = upper_code(p, p.pc in live_mac)
        if uc is None or p.uu.reserved or p.lu.reserved:
            emit("    vu1rec::trap(c, cyc, 0x%04X); return;" % p.pc)
            continue
        if p.dbit or p.tbit:
            warnings.append("D/T bit at %04x ignored" % p.pc)
        body = []
        # Stall until the operands are ready (the interpreter's
        # calculatePairReadyCycle), then commit the Q/P results now due.
        for key in waits[index]:
            body.append("vu1rec::wait(cyc, %s);" % ready_ref(key))
        if p.lu.pipeline == "fdiv" and not p.lu.wait_q:
            body.append("if (c.qPending) vu1rec::wait(cyc, c.qReady);")
        if p.lu.wait_q:
            body.append("if (c.qPending) vu1rec::wait(cyc, c.qReady);")
        if p.lu.pipeline == "efu":
            body.append("vu1rec::wait(cyc, c.efuFree);")
        if p.lu.wait_p:
            body.append("vu1rec::waitP(c, cyc);")
        if p.lu.pipeline == "xgkick":
            body.append("vu1rec::wait(cyc, c.kickFree);")
        if needs_commit(p):
            body.append("if (cyc >= c.nextDue) vu1rec::commit(c, cyc);")

        # Conditional branch reading a VI the previous pair just wrote sees
        # the value from before that write (readBranchVi).
        quirk_regs = set()
        if p.lop in COND_BRANCHES and prev is not None and prev.lu.delays_branch and prev.lu.vi_write:
            if prev.lu.vi_write in p.lu.vi_reads:
                quirk_regs.add(prev.lu.vi_write)

        def branch_vi(reg):
            if reg == 0:
                return "0"
            if reg in quirk_regs:
                return "(bakPc == 0x%04Xu ? bak : s.vi[%d])" % (p.pc, reg)
            return "s.vi[%d]" % reg

        # Record the value a following conditional branch may need.
        if nxt is not None and p.lu.delays_branch and p.lu.vi_write and nxt.lop in COND_BRANCHES \
                and p.lu.vi_write in nxt.lu.vi_reads:
            body.append("bak = s.vi[%d]; bakPc = 0x%04Xu;" % (p.lu.vi_write, nxt.pc))

        compute, commit = uc
        upper_writes = p.uu.vf_write[0] if p.uu.vf_write else 0
        suppress = bool(upper_writes) and p.lu.vf_write is not None and p.lu.vf_write[0] == upper_writes
        body.extend(compute)
        if p.ibit:
            lower = ["s.i = vu1rec::bitsf(0x%08Xu);" % p.lower]
        else:
            lower = lower_code(p, suppress, branch_vi)
            if lower is None:
                emit("    vu1rec::trap(c, 0x%04X); return;" % p.pc)
                continue
        if p.ibit:
            body.extend(commit)
            body.extend(lower)
        else:
            body.extend(lower)
            body.extend(commit)
        if quirk_regs:
            body.append("bakPc = ~0u;")

        # Results become readable after their latency (markPairWrites).
        for key, latency in stores[index]:
            body.append("%s = cyc + %d;" % (ready_ref(key), latency))
        body.append("++cyc;")

        # End of the program: the pair after an E-bit.
        if prev is not None and prev.ebit:
            if prev.is_branch():
                warnings.append("E-bit on branch at %04x" % prev.pc)
            body.append("if (ending) { s.pc = br ? brT : 0x%04Xu; vu1rec::finish(c, cyc); return; }" % ((p.pc + 8) & CODE_MASK))
        if p.ebit:
            body.append("ending = 1;")
        # Delay slot of a branch: take it now.
        if prev is not None and prev.is_branch():
            if p.is_branch():
                warnings.append("branch in delay slot at %04x" % p.pc)
            if prev.lop in STATIC_BRANCHES:
                body.append("if (br) { br = 0; bakPc = ~0u; goto L%04X; }" % prev.static_target())
            else:
                body.append("if (br) { br = 0; bakPc = ~0u; pc = brT; goto dispatch; }")
        # Each pair in its own block: its temporaries never cross a label.
        emit("    {")
        for line in body:
            emit("        " + line)
        emit("    }")
    emit("    pc = 0;")
    emit("    goto dispatch;")
    emit("}")
    writers = sum(1 for p in pairs if writes_mac(p))
    warnings.append("info: %d of %d MAC-writing pairs keep their flags" % (len(live_mac), writers))
    total_reads = sum(len(pair_effects(p)[0]) for p in pairs)
    total_writes = sum(len(pair_effects(p)[1]) for p in pairs)
    warnings.append("info: %d of %d operand checks and %d of %d ready stores kept" % (
        sum(len(w) for w in waits), total_reads, sum(len(s) for s in stores), total_writes))
    return "\n".join(lines) + "\n", warnings


def main():
    global PAIRS, CODE_MASK, UNIT, QADDR
    args = sys.argv[1:]
    if args and args[0] == "--vu0":
        args = args[1:]
        PAIRS, CODE_MASK, UNIT, QADDR = 512, 0xFFF, "vu0", "vu1rec::qaddr0"
    out_dir = pathlib.Path(args[0])
    out_dir.mkdir(parents=True, exist_ok=True)
    names = []
    for path in args[1:]:
        image = pathlib.Path(path).read_bytes()
        if len(image) != PAIRS * 8:
            sys.exit("%s: expected a %d KB %s code image" % (path, PAIRS * 8 // 1024, UNIT.upper()))
        name = pathlib.Path(path).stem
        # Entry points seen by the harvester (entries.txt next to the image).
        entries = set()
        listing = pathlib.Path(path).with_name("entries.txt")
        if listing.exists():
            for line in listing.read_text().split("\n"):
                parts = line.split()
                if len(parts) == 2 and parts[0] == name:
                    entries.add(int(parts[1], 16))
        source, warnings = translate(image, name, entries)
        (out_dir / ("%s_%s.cpp" % (UNIT, name))).write_text(source)
        for w in warnings:
            print("%s: %s" % (name, w))
        names.append(name)
    registry = ["// Generated by tools/vu1_translate.py. Do not edit.",
                '#include "vu1rec/vu1rec_rt.h"', ""]
    for name in names:
        registry.append("void %sprog_%s(vu1rec::Ctx &c);" % (UNIT, name))
    table = "kPrograms" if UNIT == "vu1" else "kVu0Programs"
    registry += ["", "namespace vu1rec", "{", "    const ProgramEntry %s[] = {" % table]
    for name in names:
        registry.append("        {0x%sull, &%sprog_%s}," % (name, UNIT, name))
    registry += ["        {0, nullptr},", "    };", "}", ""]
    (out_dir / ("%s_registry.cpp" % UNIT)).write_text("\n".join(registry))
    print("translated %d image(s)" % len(names))


main()
