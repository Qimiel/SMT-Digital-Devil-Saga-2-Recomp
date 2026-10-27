"""Summarise profile.txt written by dds2.exe with DDS2_PROFILE=1.

usage: profile_report.py <dds2.exe> <profile.txt> [top_n]

Symbols come from `nm` (on PATH via w64devkit); the guest translation units
are compiled with -g0 but keep their function symbols.
"""
import bisect
import collections
import subprocess
import sys

PREFERRED_BASE = 0x140000000


def load_symbols(exe):
    out = subprocess.run(['nm', '-C', '--defined-only', exe], capture_output=True, text=True, check=True).stdout
    symbols = []
    for line in out.splitlines():
        parts = line.split(' ', 2)
        if len(parts) == 3 and parts[1] in 'tTwW':
            symbols.append((int(parts[0], 16), parts[2]))
    symbols.sort()
    return [a for a, _ in symbols], [n for _, n in symbols]


def main():
    exe, profile = sys.argv[1], sys.argv[2]
    top_n = int(sys.argv[3]) if len(sys.argv) > 3 else 30
    addrs, names = load_symbols(exe)
    per_thread = collections.defaultdict(collections.Counter)
    callers = collections.defaultdict(collections.Counter)
    base = PREFERRED_BASE

    def name_of(rip):
        rip = rip - base + PREFERRED_BASE
        i = bisect.bisect_right(addrs, rip) - 1
        return names[i] if 0 <= i < len(names) and addrs[0] <= rip else f'?{rip:x}'

    with open(profile) as f:
        for line in f:
            parts = line.split()
            if parts[0] == 'base':
                base = int(parts[1], 16)
                continue
            if parts[0] == 'caller':
                # Return address seen while in a routine outside the executable.
                callers[int(parts[1])][name_of(int(parts[2], 16))] += int(parts[3])
                continue
            tid, rip, count = int(parts[0]), int(parts[1], 16), int(parts[2])
            if len(parts) > 3:
                # Outside the executable: the profiler already named it (module!export).
                per_thread[tid]['[' + parts[3] + ']'] += count
                continue
            rip = rip - base + PREFERRED_BASE
            i = bisect.bisect_right(addrs, rip) - 1
            name = names[i] if 0 <= i < len(names) and addrs[0] <= rip else f'?{rip:x}'
            per_thread[tid][name] += count
    for tid, counter in sorted(per_thread.items(), key=lambda kv: -sum(kv[1].values())):
        total = sum(counter.values())
        if total < 50:
            continue
        print(f'== thread {tid}: {total} samples')
        for name, count in counter.most_common(top_n):
            print(f'{100.0 * count / total:6.2f}%  {name[:150]}')
        if callers.get(tid):
            print('  callers of routines outside the executable:')
            for name, count in callers[tid].most_common(8):
                print(f'  {100.0 * count / total:6.2f}%  {name[:140]}')
        print()


if __name__ == '__main__':
    main()
