"""Find EE code addresses that are reached only through pointers.

PS2Recomp emits an entry point for every function start in the unit map and for
every internal label it can see statically (branch targets, resolved jump
tables, call return addresses). Code that is only ever reached through a
function pointer -- callbacks passed as arguments, thread entry points, C++
vtables, unresolved jump tables -- gets no entry, and the runtime reports a
missing function the first time the game jumps there.

Sources of pointers:
  * data: every aligned word in the ELF's data sections whose value is a
    word-aligned .text address;
  * code: lui/addiu, lui/daddiu and lui/ori pairs that materialize a .text
    address in a register (tracked per register, linearly, inside each unit).

Output: a CSV of targets with no registered entry, classified as
  function  - looks like a function start (previous instruction pair is a
              `jr $ra` + delay slot or alignment nops, or the target opens a
              stack frame), and
  label     - anything else (typically a jump-table case or mid-function label).
"""
import argparse
import csv
import os
import re
import struct
import sys
from bisect import bisect_right

TEXT_START, TEXT_END = 0x00100000, 0x00370B7C
DATA_SECTIONS = ('.ctors', '.dtors', '.data', '.rodata', '.lit4', '.sdata', '.vudata')


class Elf:
    def __init__(self, path):
        self.data = open(path, 'rb').read()
        d = self.data
        shoff = struct.unpack_from('<I', d, 0x20)[0]
        shentsize, shnum, shstrndx = struct.unpack_from('<HHH', d, 0x2E)
        raw = [struct.unpack_from('<10I', d, shoff + i * shentsize) for i in range(shnum)]
        stroff = raw[shstrndx][4]
        self.sections = {}
        for s in raw:
            name_off, typ, flags, addr, off, size = s[:6]
            end = d.index(b'\0', stroff + name_off)
            name = d[stroff + name_off:end].decode()
            self.sections[name] = (addr, off, size, typ)

    def words(self, name):
        addr, off, size, typ = self.sections[name]
        if typ == 8:  # NOBITS
            return
        for i in range(0, size - 3, 4):
            yield addr + i, struct.unpack_from('<I', self.data, off + i)[0]

    def text_word(self, va):
        addr, off, size, _ = self.sections['.text']
        if not (addr <= va < addr + size):
            return None
        return struct.unpack_from('<I', self.data, off + va - addr)[0]


def load_units(path):
    units = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            units.append((int(r['Start'], 16), int(r['End'], 16), r['Name']))
    units.sort()
    return units


def load_registered(path):
    pat = re.compile(r'// 0x([0-9a-f]+)\s*$')
    regs = set()
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            m = pat.search(line)
            if m:
                regs.add(int(m.group(1), 16))
    return regs


def load_resolved_jump_cases(generated_dir):
    """Case targets of jump tables PS2Recomp already resolved into local switches."""
    case_re = re.compile(r'case 0x([0-9A-F]+)u: goto label_')
    resolved = set()
    for name in os.listdir(generated_dir):
        if not name.endswith('.cpp') or name == 'register_functions.cpp':
            continue
        with open(os.path.join(generated_dir, name), encoding='utf-8', errors='replace') as f:
            text = f.read()
        for block in text.split('switch (jumpTarget) {')[1:]:
            end = block.find('default: break;')
            resolved.update(int(m.group(1), 16) for m in case_re.finditer(block[:end]))
    return resolved


def code_pointer_candidates(elf, units):
    """lui/addiu|daddiu|ori pairs building .text addresses."""
    found = {}
    for start, end, name in units:
        hi = {}
        for va in range(start, end, 4):
            w = elf.text_word(va)
            if w is None:
                break
            op = w >> 26
            rs = (w >> 21) & 31
            rt = (w >> 16) & 31
            imm = w & 0xFFFF
            simm = imm - 0x10000 if imm & 0x8000 else imm
            if op == 0x0F:  # lui
                hi[rt] = imm << 16
                continue
            value = None
            if op in (0x09, 0x19) and rs in hi:  # addiu/daddiu
                value = (hi[rs] + simm) & 0xFFFFFFFF
            elif op == 0x0D and rs in hi:  # ori
                value = hi[rs] | imm
            if value is not None and TEXT_START <= value < TEXT_END and value % 4 == 0:
                found.setdefault(value, va)
            # Any write to rt by an I-type ALU/load op clobbers a tracked hi part.
            if op in (0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x18, 0x19,
                      0x20, 0x21, 0x23, 0x24, 0x25, 0x27, 0x37, 0x1A, 0x1B, 0x1E):
                if op not in (0x09, 0x19, 0x0D) or rt != rs:
                    hi.pop(rt, None)
                else:
                    hi.pop(rt, None)
            elif op == 0:  # SPECIAL: rd = (w >> 11) & 31
                hi.pop((w >> 11) & 31, None)
    return found


def classify(elf, target):
    prev2 = elf.text_word(target - 8)
    prev1 = elf.text_word(target - 4)
    first = elf.text_word(target)
    if prev2 == 0x03E00008:  # jr $ra ; <delay>
        return 'function'
    if prev1 == 0 and prev2 in (0, None):
        # alignment padding; walk back over nops to a jr $ra
        va = target - 4
        while va >= TEXT_START and elf.text_word(va) == 0:
            va -= 4
        if elf.text_word(va - 4) == 0x03E00008 or elf.text_word(va) == 0x03E00008:
            return 'function'
    if first is not None and (first >> 16) == 0x27BD and (first & 0x8000):  # addiu sp,sp,-N
        return 'function'
    return 'label'


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ws = os.path.normpath(os.path.join(here, '..', '..', '..'))
    ap = argparse.ArgumentParser()
    ap.add_argument('--elf', default=os.path.join(ws, 'SLUS_211.52'))
    ap.add_argument('--units', default=os.path.join(ws, 'recovered', 'code', 'ee', 'recomp_units.csv'))
    ap.add_argument('--registered', default=os.path.join(ws, 'recovered', 'code', 'recompiled', 'register_functions.cpp'))
    ap.add_argument('--out', default=os.path.join(here, '..', 'research', 'pointer_targets.csv'))
    ap.add_argument('--entries-out', default=os.path.join(here, '..', 'research', 'extra_entry_points.txt'))
    args = ap.parse_args()

    elf = Elf(args.elf)
    units = load_units(args.units)
    registered = load_registered(args.registered)
    resolved = load_resolved_jump_cases(os.path.dirname(os.path.abspath(args.registered)))
    starts = [u[0] for u in units]

    sources = {}
    for name in DATA_SECTIONS:
        if name not in elf.sections:
            continue
        for va, value in elf.words(name):
            if TEXT_START <= value < TEXT_END and value % 4 == 0:
                sources.setdefault(value, ('data', va))
    for value, va in code_pointer_candidates(elf, units).items():
        sources.setdefault(value, ('code', va))

    rows = []
    for target in sorted(sources):
        if target in registered:
            continue
        kind, ref = sources[target]
        idx = bisect_right(starts, target) - 1
        owner = units[idx][2] if idx >= 0 else ''
        rows.append(dict(target=f'0x{target:08X}', class_=classify(elf, target), source=kind,
                         referenced_at=f'0x{ref:08X}', containing_unit=owner,
                         resolved_jump_case=int(target in resolved)))

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['target', 'class_', 'source', 'referenced_at', 'containing_unit',
                                          'resolved_jump_case'])
        w.writeheader()
        w.writerows(rows)
    entries = [r for r in rows if not r['resolved_jump_case']]
    with open(args.entries_out, 'w', newline='\n') as f:
        f.write('# Generated by find_code_pointers.py: pointer-referenced EE code addresses\n'
                '# without a recompiled entry point (resolved jump-table cases excluded).\n')
        for r in entries:
            f.write(f"{r['target']}  # {r['class_']} via {r['source']} @{r['referenced_at']}\n")
    n_fn = sum(r['class_'] == 'function' for r in rows)
    print(f'{len(sources)} pointer targets, {len(rows)} without an entry: '
          f'{n_fn} function-like, {len(rows) - n_fn} label-like; '
          f'{len(entries)} not resolved jump-table cases -> {args.entries_out}')


if __name__ == '__main__':
    sys.exit(main())
