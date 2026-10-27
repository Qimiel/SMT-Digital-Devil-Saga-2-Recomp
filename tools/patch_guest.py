"""Produce the buildable guest sources from the raw PS2Recomp output.

Input is pc_port/recomp/generated, written by the vendored recompiler from
dds2_recomp.toml (the first-pass translation in recovered/code/recompiled is
kept unchanged as evidence). This script writes a patched copy (default:
pc_port/recomp/guest) and only rewrites files whose patched content differs, so
incremental builds stay cheap.

Patches applied:
  * Tail calls. PS2Recomp emits `j func` as a direct C++ call
    `sub_X_0xADDR(rdram, ctx, runtime); return;`, which bypasses the runtime
    dispatch table and therefore any game override bound to that address.
    They are rewritten to dispatch through the table:
    `runtime->lookupFunction(0xADDR)(rdram, ctx, runtime); return;`.
"""
import argparse
import os
import re
import shutil
import sys

TAIL_CALL = re.compile(
    r'^(?P<indent>[ \t]*)(?P<name>\w+_0x(?P<addr>[0-9a-f]+))\(rdram, ctx, runtime\); return;(?P<eol>\r?)$',
    re.M)


# VU0 macro-mode Q results (fixed in the vendored recompiler too, so this only
# matches output generated before the fix): VRSQRT ignored its numerator,
# division by zero gave 0 instead of +-FLT_MAX, and SQRT clamped negative
# operands to 0 instead of taking their magnitude.
VRSQRT = re.compile(
    r'(?P<head>// 0x[0-9a-f]+: 0x(?P<word>[0-9a-f]{8})\s+vrsqrt[^\n]*\n[^\n]*\n[ \t]*)'
    r'\{ float ft = _mm_cvtss_f32\(_mm_shuffle_ps\(ctx->vu0_vf\[(?P<ft>\d+)\], ctx->vu0_vf\[\d+\], '
    r'_MM_SHUFFLE\(0,0,0,(?P<ftf>\d)\)\)\); ctx->vu0_q = \(ft > 0\.0f\) \? \(1\.0f / sqrtf\(ft\)\) : 0\.0f; \}')
VDIV_ZERO = 'ctx->vu0_q = (ft != 0.0f) ? (fs / ft) : 0.0f; }'
VDIV_FIXED = ('ctx->vu0_q = (ft != 0.0f) ? (fs / ft) : ((std::signbit(fs) != std::signbit(ft)) '
              '? -3.402823466e38f : 3.402823466e38f); }')
VSQRT_CLAMP = 'ctx->vu0_q = sqrtf(std::max(0.0f, ft)); }'
VSQRT_FIXED = 'ctx->vu0_q = sqrtf(std::fabs(ft)); }'


# EE FPU (also fixed in the vendored recompiler): division by zero gave an
# infinity (the EE saturates to +-FLT_MAX), RSQRT.S computed 1/sqrt(fs)
# instead of fs/sqrt(ft), SQRT.S rooted fs instead of ft, and CVT.W.S did not
# saturate.
FPU_DIV = re.compile(
    r'if \(ctx->f\[(?P<ft>\d+)\] == 0\.0f\) \{ ctx->fcr31 \|= 0x100000; /\* DZ flag \*/ '
    r'ctx->f\[(?P<fd>\d+)\] = copysignf\(INFINITY, ctx->f\[(?P<fs>\d+)\] \* 0\.0f\); \}')
FPU_RSQRT = re.compile(
    r'(?P<head>// 0x[0-9a-f]+: 0x(?P<word>[0-9a-f]{8})\s+rsqrt\.s[^\n]*\n[^\n]*\n[ \t]*)'
    r'ctx->f\[(?P<fd>\d+)\] = 1\.0f / sqrtf\(ctx->f\[(?P<fs>\d+)\]\);')
# SQRT.S took its operand from the fs field, but the EE encodes it in ft (fs
# is 0), and roots the magnitude of a negative operand instead of giving NaN.
FPU_SQRT = re.compile(
    r'(?P<head>// 0x[0-9a-f]+: 0x(?P<word>[0-9a-f]{8})\s[^\n]*\n(?:[ \t]*ctx->pc = [^\n]*\n)?[ \t]*)'
    r'ctx->f\[(?P<fd>\d+)\] = FPU_SQRT_S\(ctx->f\[(?P<fs>\d+)\]\);')
# CVT.W.S: out-of-range values saturate on the EE (x86 gives INT_MIN for both
# signs).
FPU_CVT_W = re.compile(
    r'\{ int32_t tmp = FPU_CVT_W_S\(ctx->f\[(?P<fs>\d+)\]\); std::memcpy\(&ctx->f\[(?P<fd>\d+)\], &tmp, sizeof\(tmp\)\); \}')


def patch_fpu(text):
    count = 0

    def div(m):
        nonlocal count
        count += 1
        ft, fd, fs = m.group('ft'), m.group('fd'), m.group('fs')
        return (f'if (ctx->f[{ft}] == 0.0f) {{ ctx->fcr31 |= 0x100000; /* DZ flag */ '
                f'ctx->f[{fd}] = (std::signbit(ctx->f[{fs}]) != std::signbit(ctx->f[{ft}])) '
                f'? -3.402823466e38f : 3.402823466e38f; }}')

    def rsqrt(m):
        nonlocal count
        count += 1
        ft = (int(m.group('word'), 16) >> 16) & 0x1F
        fd, fs = m.group('fd'), m.group('fs')
        return (f"{m.group('head')}{{ float root = sqrtf(std::fabs(ctx->f[{ft}])); ctx->f[{fd}] = (root != 0.0f) "
                f"? ctx->f[{fs}] / root : (std::signbit(ctx->f[{fs}]) ? -3.402823466e38f : 3.402823466e38f); }}")

    def sqrt(m):
        nonlocal count
        count += 1
        ft = (int(m.group('word'), 16) >> 16) & 0x1F
        return f"{m.group('head')}ctx->f[{m.group('fd')}] = sqrtf(std::fabs(ctx->f[{ft}]));"

    def cvt_w(m):
        nonlocal count
        count += 1
        return (f"{{ const float v = ctx->f[{m.group('fs')}]; const int32_t tmp = v >= 2147483648.0f ? 0x7FFFFFFF : "
                f"(v <= -2147483648.0f ? (-2147483647 - 1) : static_cast<int32_t>(v)); "
                f"std::memcpy(&ctx->f[{m.group('fd')}], &tmp, sizeof(tmp)); }}")

    text = FPU_DIV.sub(div, text)
    text = FPU_RSQRT.sub(rsqrt, text)
    text = FPU_SQRT.sub(sqrt, text)
    text = FPU_CVT_W.sub(cvt_w, text)
    return text, count


# VU0 macro-mode memory access (fixed in the vendored recompiler too): VLQI,
# VLQD, VSQI, VSQD, VILWR and VISWR addressed EE memory at vi*16 (vi*4 for
# the integer forms) instead of VU0 data memory, VSQI/VSQD swapped their
# data and address registers, and VILWR/VISWR ignored the field.
VU0_MEM = re.compile(
    r'(?P<head>// 0x[0-9a-f]+: 0x(?P<word>[0-9a-f]{8})\s+(?P<op>vlqi|vlqd|vsqi|vsqd|vilwr|viswr)\b[^\n]*\n[^\n]*\n[ \t]*)'
    r'(?P<body>\{[^\n]*(?:READ(?:32|128)|WRITE(?:32|128))\(addr[^\n]*\})')


def vu0_qword(vi):
    return f'(runtime->memory().getVU0Data() + ((ctx->vi[{vi}] & 0xFFu) << 4))'


def vu0_memory_op(op, word):
    dest = (word >> 21) & 0xF
    rt = (word >> 16) & 0x1F  # ft / it
    rd = (word >> 11) & 0x1F  # fs / is
    blend = ((dest >> 3) & 1) | (((dest >> 2) & 1) << 1) | (((dest >> 1) & 1) << 2) | ((dest & 1) << 3)
    if op == 'vilwr':
        if rt == 0:
            return '// VILWR to vi0 (no effect)'
        offset = 0 if dest & 8 else 4 if dest & 4 else 8 if dest & 2 else 12
        return (f'{{ uint32_t word; std::memcpy(&word, {vu0_qword(rd)} + {offset}u, sizeof(word)); '
                f'ctx->vi[{rt}] = static_cast<uint16_t>(word); }}')
    if op == 'viswr':
        stores = ''.join(f' std::memcpy(q + {lane * 4}u, &word, sizeof(word));'
                         for lane in range(4) if blend & (1 << lane))
        return f'{{ uint8_t *q = {vu0_qword(rd)}; const uint32_t word = ctx->vi[{rt}];{stores} }}'
    store = op.startswith('vsq')
    data_reg, addr_reg = (rd, rt) if store else (rt, rd)
    step = '+' if op.endswith('i') else '-'
    step_code = '' if addr_reg == 0 else \
        f' ctx->vi[{addr_reg}] = static_cast<uint16_t>(ctx->vi[{addr_reg}] {step} 1);'
    pre, post = ('', step_code) if step == '+' else (step_code, '')
    if store:
        return (f'{{{pre} float *q = reinterpret_cast<float *>({vu0_qword(addr_reg)}); '
                f'_mm_storeu_ps(q, _mm_blend_ps(_mm_loadu_ps(q), ctx->vu0_vf[{data_reg}], {blend}));{post} }}')
    if data_reg == 0:
        return f'{{{pre}{post} }}'
    return (f'{{{pre} const __m128 v = _mm_loadu_ps(reinterpret_cast<const float *>({vu0_qword(addr_reg)})); '
            f'ctx->vu0_vf[{data_reg}] = _mm_blend_ps(ctx->vu0_vf[{data_reg}], v, {blend});{post} }}')


def patch_vu0_memory(text):
    count = 0

    def repl(m):
        nonlocal count
        count += 1
        return m.group('head') + vu0_memory_op(m.group('op'), int(m.group('word'), 16))

    return VU0_MEM.sub(repl, text), count


def patch_vu0_q(text):
    count = 0

    def repl(m):
        nonlocal count
        count += 1
        word = int(m.group('word'), 16)
        fs, fsf = (word >> 11) & 0x1F, (word >> 21) & 3
        ft, ftf = m.group('ft'), m.group('ftf')
        return (f"{m.group('head')}{{ float fs = _mm_cvtss_f32(_mm_shuffle_ps(ctx->vu0_vf[{fs}], ctx->vu0_vf[{fs}], "
                f"_MM_SHUFFLE(0,0,0,{fsf}))); float ft = _mm_cvtss_f32(_mm_shuffle_ps(ctx->vu0_vf[{ft}], "
                f"ctx->vu0_vf[{ft}], _MM_SHUFFLE(0,0,0,{ftf}))); float root = sqrtf(std::fabs(ft)); "
                f"ctx->vu0_q = (root != 0.0f) ? (fs / root) : (std::signbit(fs) ? -3.402823466e38f : "
                f"3.402823466e38f); }}")

    text = VRSQRT.sub(repl, text)
    count += text.count(VDIV_ZERO) + text.count(VSQRT_CLAMP)
    text = text.replace(VDIV_ZERO, VDIV_FIXED).replace(VSQRT_CLAMP, VSQRT_FIXED)
    return text, count


def patch_source(text):
    count = 0

    def repl(m):
        nonlocal count
        count += 1
        return (f"{m.group('indent')}runtime->lookupFunction(0x{m.group('addr')}u)"
                f"(rdram, ctx, runtime); return; // tail call {m.group('name')}{m.group('eol')}")

    return TAIL_CALL.sub(repl, text), count


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    workspace = os.path.normpath(os.path.join(here, '..', '..', '..'))
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', default=os.path.join(here, '..', 'generated'))
    ap.add_argument('--dst', default=os.path.join(here, '..', 'guest'))
    args = ap.parse_args()
    os.makedirs(args.dst, exist_ok=True)

    written = unchanged = tail_calls = vu0_fixes = fpu_fixes = 0
    names = sorted(os.listdir(args.src))
    for name in names:
        src_path = os.path.join(args.src, name)
        if not os.path.isfile(src_path) or not name.endswith(('.cpp', '.h')):
            continue
        with open(src_path, 'r', encoding='utf-8', newline='') as f:
            text = f.read()
        if name.endswith('.cpp') and name != 'register_functions.cpp':
            text, n = patch_source(text)
            tail_calls += n
            text, n = patch_vu0_q(text)
            vu0_fixes += n
            text, n = patch_fpu(text)
            fpu_fixes += n
            text, n = patch_vu0_memory(text)
            vu0_fixes += n
        dst_path = os.path.join(args.dst, name)
        if os.path.exists(dst_path):
            with open(dst_path, 'r', encoding='utf-8', newline='') as f:
                if f.read() == text:
                    unchanged += 1
                    continue
        with open(dst_path, 'w', encoding='utf-8', newline='') as f:
            f.write(text)
        written += 1

    stale = set(os.listdir(args.dst)) - set(names)
    for name in stale:
        os.remove(os.path.join(args.dst, name))
    print(f'guest sources: {written} written, {unchanged} unchanged, {len(stale)} removed; '
          f'{tail_calls} tail calls routed through the dispatch table; {vu0_fixes} VU0 Q results and '
          f'{fpu_fixes} FPU divisions/RSQRTs/SQRTs/CVT.Ws fixed')


if __name__ == '__main__':
    sys.exit(main())
