"""Score full-travel crossings from a CSV written by Forge's log_read.

Usage: python score_crossings.py run.csv [--band 0.05] [--min-step 5] [--window 1.4]

The CSV needs (any prefix; matched by the end of the column name):
  TargetInches, positionInches (or PositionInches), profileErrorInches,
  leftStatorAmps, rightStatorAmps; optional accelInPerSec2, jerkInPerSec3, cruiseInPerSec.
A crossing starts when the target jumps by at least --min-step inches. Its time is from that
jump until the carriage is within --band of the target and stays there until the next jump.
Only --window seconds after the jump count (default 1.4; the routines hold 1.5 s), so a
park after the last crossing does not count. Prints one row per crossing and the worst per (cruise, accel, jerk) setting.
Read log_read's CSV at its full rate (max_rows 5000 over a short span): settling times are
only as fine as the rows.
"""
import csv, math, sys, collections

def col(header, *ends):
    for e in ends:
        for h in header:
            if h.split('/')[-1] == e:
                return h
    return None

def num(x):
    try:
        v = float(x)
        return v if not math.isnan(v) else None
    except (TypeError, ValueError):
        return None

def main(path, band=0.05, min_step=5.0, window=1.4):
    rows = list(csv.DictReader(open(path)))
    h = rows[0].keys()
    C = {k: col(h, *v) for k, v in {
        'tgt': ('TargetInches', 'targetInches'), 'pos': ('positionInches', 'PositionInches'),
        'err': ('profileErrorInches', 'ProfileErrorInches'), 'L': ('leftStatorAmps',),
        'R': ('rightStatorAmps',), 'acc': ('accelInPerSec2',), 'jerk': ('jerkInPerSec3',),
        'cruise': ('cruiseInPerSec',)}.items()}
    if not (C['tgt'] and C['pos']):
        sys.exit('need TargetInches and positionInches columns')
    last, D = {}, []
    for r in rows:  # hold every signal at its last value
        for k, c in C.items():
            if c and num(r[c]) is not None:
                last[k] = num(r[c])
        D.append((float(r['t']), dict(last)))
    starts = [i for i in range(1, len(D))
              if 'tgt' in D[i][1] and 'tgt' in D[i-1][1]
              and abs(D[i][1]['tgt'] - D[i-1][1]['tgt']) >= min_step]
    ends = starts[1:] + [len(D)]
    out = []
    for s, e in zip(starts, ends):
        t0, tgt = D[s][0], D[s][1]['tgt']
        while e > s + 1 and D[e-1][0] - t0 > window:  # judge only the hold that follows
            e -= 1
        settled = None
        for i in range(s, e):
            if abs(D[i][1].get('pos', 1e9) - tgt) > band:
                settled = None
            elif settled is None:
                settled = D[i][0]
        seg = [d for _, d in D[s:e]]
        up = tgt > D[s-1][1]['tgt']
        over = max(((d['pos'] - tgt) if up else (tgt - d['pos'])) for d in seg if 'pos' in d)
        out.append(dict(t=t0, dir='up' if up else 'down',
                        time=(settled - t0) if settled else float('nan'),
                        worst_err=max(abs(d.get('err', 0)) for d in seg),
                        overshoot=max(0.0, over),
                        peakL=max(abs(d.get('L', 0)) for d in seg),
                        peakR=max(abs(d.get('R', 0)) for d in seg),
                        setting=(D[s][1].get('cruise'), D[s][1].get('acc'), D[s][1].get('jerk'))))
    print('t        dir   time   worst_err overshoot peakL peakR  (cruise, accel, jerk)')
    for o in out:
        print('%8.2f %-5s %5.3f  %7.3f  %7.3f  %5.1f %5.1f  %s' % (
            o['t'], o['dir'], o['time'], o['worst_err'], o['overshoot'], o['peakL'], o['peakR'], o['setting']))
    g = collections.defaultdict(list)
    for o in out:
        g[o['setting']].append(o)
    print('\nworst per setting:')
    for k, v in g.items():
        print('%s n=%d time %.3f s, err %.3f in, overshoot %.3f in, peak L/R %.0f/%.0f A' % (
            k, len(v), max(x['time'] for x in v), max(x['worst_err'] for x in v),
            max(x['overshoot'] for x in v), max(x['peakL'] for x in v), max(x['peakR'] for x in v)))

if __name__ == '__main__':
    a = sys.argv[1:]
    band = float(a[a.index('--band') + 1]) if '--band' in a else 0.05
    step = float(a[a.index('--min-step') + 1]) if '--min-step' in a else 5.0
    win = float(a[a.index('--window') + 1]) if '--window' in a else 1.4
    main(a[0], band, step, win)
