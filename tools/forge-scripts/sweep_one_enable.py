# Template: a parameter sweep in ONE enable (run 8f6d0aae, EL48, 18 Nov 2026, the jerk sweep).
# To sweep another value, change JERKS and the TU + "JerkInPerSec3" / ST + "TalonJerkInPerSec3" pair.
import forge_lab as f
# Jerk sweep, take 4, in ONE enable. Same settings as EL45 (kA 0.0016, 90 in/s, 700 in/s2, kP 4);
# only the jerk limit changes. Before each set it checks the Talon really holds that jerk.
JERKS = [0, 9999, 7000, 5000, 3500]      # in/s^3; 0 = no jerk limit; 9999 is the Talon's maximum
STARTS = 3                               # full-travel starts per jerk (4 moves each)
ST = '/Elevator/Status/'
TU = '/Elevator/Tuning/'
ROUTINE = '/Elevator/Control/Routine'
RESET_PEAKS = '/Elevator/Control/ResetPeaks'
# every sets.csv row has the same columns, skipped or not
SET_COLS = ['peak_stator_A', 'peak_volts_frac', 'peak_pdh_A', 'max_err_in', 'energy_J_per_start']

f.log_topics(['/Elevator/', '/Power/'])
# Wait until the robot program is up and publishing
f.wait_until(lambda: f.nt_get(ST + 'PositionInches') is not None, 60)
# Stop if the carriage leaves its travel, is more than 0.9 in off the planned path,
# or either motor goes over 39 A stator (the Talon limit is 40 A)
f.limits([f.Limit(ST + 'PositionInches', above=10.3, below=-2.5),
          f.Limit(ST + 'ProfileErrorInches', above=0.9, below=-0.9),
          f.Limit(ST + 'RightStatorAmps', above=39), f.Limit(ST + 'LeftStatorAmps', above=39)])
# On any ending: park, then put the jerk back to 0 (it is written once parked)
f.on_exit(lambda: f.nt_set(ROUTINE, 'park'))
f.on_exit(lambda: f.nt_set(TU + 'JerkInPerSec3', 0.0))

subs = {k: f.nt_subscribe(ST + k) for k in
        ['PositionInches', 'ProfileErrorInches', 'RightStatorAmps', 'LeftStatorAmps',
         'RightSupplyAmps', 'LeftSupplyAmps', 'BatteryVolts']}
last = {}

def vt(s):
    # a sample as (value, server_time), whichever shape it comes in
    if isinstance(s, dict):
        return s.get('value'), s.get('server_time', s.get('time'))
    if isinstance(s, (tuple, list)):
        return s[0], s[-1]
    return getattr(s, 'value', None), getattr(s, 'server_time', getattr(s, 'time', None))

def drain(jerk, n, acc):
    # one CSV row per position sample; acc gathers worst path error and energy (battery V x supply A)
    rows = []
    for k, sub in subs.items():
        for s in sub.read():
            v, t = vt(s)
            if v is not None and t is not None:
                rows.append((t, k, v))
    for t, k, v in sorted(rows, key=lambda r: r[0]):
        if n > 0 and acc.get('t') is not None and 'BatteryVolts' in last:
            amps = abs(last.get('RightSupplyAmps', 0)) + abs(last.get('LeftSupplyAmps', 0))
            acc['joules'] += last['BatteryVolts'] * amps * (t - acc['t'])
        acc['t'] = t
        last[k] = v
        if k == 'ProfileErrorInches' and n > 0:
            acc['err'] = max(acc['err'], abs(v))
        if k == 'PositionInches':
            f.record('samples', jerk=jerk, n=n, server_time=t, **{name: last.get(name) for name in subs})

def running(name):
    return name in str(f.nt_get(ST + 'Routine') or '')

def run_routine(name):
    f.nt_set(ROUTINE, '')
    f.sleep(0.3)
    f.nt_set(ROUTINE, name)

def park():
    run_routine('park')
    f.wait_until(lambda: f.nt_get(ST + 'Parked') is True, 10)

f.log('Enable in TELEOP. It ranges, then runs %d jerk values, %d starts each, and ends by itself.'
      % (len(JERKS), STARTS))
f.wait_enabled(300)
f.sleep(1.0)
if running('AutoRange'):
    f.mark('auto-range')
    f.wait_until(lambda: not running('AutoRange'), 30)

for i, jerk in enumerate(JERKS):
    # 1. Park (motors off), write the jerk, and wait for the Talon's read-back to show it
    park()
    f.nt_set(TU + 'JerkInPerSec3', float(jerk))
    try:
        # a read-back of 0.0 is a real value, so test for None, not for falsy
        f.wait_until(lambda: f.nt_get(ST + 'TalonJerkInPerSec3') is not None
                     and abs(f.nt_get(ST + 'TalonJerkInPerSec3') - jerk) < 0.5, 5)
        f.sleep(0.2)
        landed = True
    except f.TimedOut:
        landed = False
    ok, status = f.nt_get(ST + 'ConfigOk'), f.nt_get(ST + 'ConfigStatus')
    talon = (f.nt_get(ST + 'TalonJerkInPerSec3'), f.nt_get(ST + 'TalonAccelInPerSec2'),
             f.nt_get(ST + 'TalonCruiseInPerSec'), f.nt_get(ST + 'TalonkA'), f.nt_get(ST + 'TalonkP'))
    f.log('jerk %s: in the Talon %s (jerk, accel, cruise, kA, kP), write ok %s, %s' % (jerk, talon, ok, status))
    if not landed or ok is not True:
        f.log('the Talon does not hold jerk %s: set skipped' % jerk)
        f.record('sets', jerk=jerk, talon_jerk=talon[0], config_ok=ok, config_status=status, skipped=True,
                 **{c: None for c in SET_COLS})
        continue
    # 2. Fresh peaks for this set, then the starts
    f.nt_set(RESET_PEAKS, float(i + 1))
    f.sleep(0.2)
    acc = {'t': None, 'err': 0.0, 'joules': 0.0}
    for n in range(1, STARTS + 1):
        park()
        drain(jerk, 0, acc)
        acc['t'] = None
        f.mark('J%s S%d' % (jerk, n))
        timed = f.nt_get(ST + 'ProfilesTimed') or 0
        run_routine('fulltravel')
        f.sleep(0.5)
        while running('FullTravel'):
            drain(jerk, n, acc)
            # each move the robot timed: its profile's seconds and inches
            now_timed = f.nt_get(ST + 'ProfilesTimed') or 0
            if now_timed != timed:
                timed = now_timed
                f.record('profiles', jerk=jerk, n=n, server_time=f.nt_get_with_time(ST + 'LastProfileSeconds')[2],
                         seconds=f.nt_get(ST + 'LastProfileSeconds'), inches=f.nt_get(ST + 'LastProfileInches'))
            f.sleep(0.02)
        drain(jerk, n, acc)
    # 3. This set's peaks, from the robot (every 20 ms loop since the reset)
    row = dict(jerk=jerk, talon_jerk=talon[0], config_ok=ok, config_status=status, skipped=False,
               peak_stator_A=f.nt_get(ST + 'PeakStatorAmps'),
               peak_volts_frac=f.nt_get(ST + 'PeakVoltsFractionOfBattery'),
               peak_pdh_A=f.nt_get('/Power/Status/PeakPdhTotalAmps'),
               max_err_in=round(acc['err'], 3), energy_J_per_start=round(acc['joules'] / STARTS, 1))
    f.record('sets', **row)
    f.log('set done: %s' % row)

f.mark('park')
park()
f.log('DONE: all sets run, parked, jerk back to 0 on exit.')