import forge_lab as f
# From trigger_test.py. Kept as the template for stand tests (16 Nov 2026): wait for the topic, limits, on_exit park +
# defaults, settings while disabled, wait_enabled, routines by name with park between, marks,
# records, then TEST DONE and wait for the PERSON's disable (Forge's disable fails in scripts).

# Ramp-per-start test (1c) on one enable. Caps 6 V and 40 A. ONE motor alone, open loop.
# Each start: the routine ranges if needed, switches drive (configs written, as soloright does), goes to the
# bottom with Motion Magic, then ramps VoltageOut up at 0.5 V/s (to 5 V or the top guard), brakes, ramps down,
# brakes. Right and left alternate, 6 starts each. Scored afterwards from the robot log: volts needed per
# in/s (kV), the volts at which it starts to move, per start. A jump between starts = the motor changed state.
# At the end the stand parks with motors off and the script waits for YOUR disable.
ORDER = (('R', 'rampsolo'), ('L', 'rampsolo_left')) * 6
PEAK_VOLTS, AMPS = 6.0, 40
START_SECONDS_MAX = 60.0     # one start is well under this
DEFAULTS = (('kV', 0.0643), ('kP', 8), ('kD', 0.10), ('CruiseInPerSec', 10),
            ('AccelInPerSec2', 60), ('JerkInPerSec3', 0), ('PeakVolts', 3), ('StatorAmps', 20))
ST = '/Elevator/Status/'
TU = '/Elevator/Tuning/'
ROUTINE = '/Elevator/Control/Routine'

f.wait_until(lambda: f.nt_get(ST + 'PositionInches') is not None, 60)
f.log_topics(['/Elevator/Status/', '/Elevator/Tuning/'])
# Stop if the carriage leaves its travel, or is more than 0.5 in off the planned path
f.limits([f.Limit(ST + 'PositionInches', above=10.3, below=-2.5),
          f.Limit(ST + 'ProfileErrorInches', above=0.5, below=-0.5)])

def restore_defaults():
    # park first (that also puts the right back in the lead); defaults are written once parked
    f.nt_set(ROUTINE, 'park')
    for k, v in DEFAULTS:
        f.nt_set(TU + k, v)
f.on_exit(restore_defaults)

def running(name):
    return name in str(f.nt_get(ST + 'Routine') or '')

def run_routine(name):
    f.nt_set(ROUTINE, '')
    f.sleep(0.3)
    f.nt_set(ROUTINE, name)

def park():
    run_routine('park')
    f.wait_until(lambda: f.nt_get(ST + 'Parked') is True, 10)

def run_ramp(part, n, name):
    # one start: run the routine until it ends by itself, recording position (the robot log decides)
    f.mark('%s%d %s' % (part, n, name))
    run_routine(name)
    f.sleep(1.0)
    t0, last = f.now().monotonic, None
    while running('Ramp') and f.now().monotonic - t0 < START_SECONDS_MAX:
        pos, _, st = f.nt_get_with_time(ST + 'PositionInches')
        if st != last:
            last = st
            f.record('samples', part=part, n=n, routine=name, server_time=st,
                     seconds=f.now().monotonic - t0, pos=pos)
        f.sleep(0.005)
    f.log('%s%d %s took %.1f s' % (part, n, name, f.now().monotonic - t0))
    park()

# Caps and the first speed, written while disabled
f.nt_set(TU + 'PeakVolts', PEAK_VOLTS)
f.nt_set(TU + 'StatorAmps', AMPS)
f.nt_set(TU + 'JerkInPerSec3', 0)
f.nt_set(TU + 'CruiseInPerSec', 30)
f.nt_set(TU + 'AccelInPerSec2', 200)

f.log('Enable in TELEOP. Everything runs by itself. At TEST DONE, please disable.')
f.wait_enabled(120)

# 0. Auto-range runs by itself after the new program starts; wait for it
f.mark('start')
f.sleep(1.0)
if running('AutoRange'):
    f.wait_until(lambda: not running('AutoRange'), 30)
f.log('travel %.3f in' % (f.nt_get(ST + 'TravelInches') or 0))

# Twelve starts, right and left alternating
park()
for n, (part, name) in enumerate(ORDER, start=1):
    run_ramp(part, n, name)

# The end: park with the defaults back, 20 A included (on_exit does it too)
f.mark('park')
restore_defaults()
f.wait_until(lambda: f.nt_get(ST + 'Parked') is True, 10)
f.mark('parked')
f.log('TEST DONE. Parked at the bottom, motors off, right leading, limit back to 20 A. PLEASE DISABLE NOW.')
# Wait for the person's disable; the stand stays parked with motors off meanwhile
try:
    f.wait_disabled(60)
    f.log('Disabled by hand. Thank you.')
except f.TimedOut:
    f.log('Still enabled after 60 s: parked, motors off. Please disable at the Driver Station.')
