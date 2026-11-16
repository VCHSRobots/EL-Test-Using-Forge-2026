import forge_lab as f
# Kept as the template for stand tests (16 Nov 2026): wait for the topic, limits, on_exit park +
# defaults, settings while disabled, wait_enabled, routines by name with park between, marks,
# records, then TEST DONE and wait for the PERSON's disable (Forge's disable fails in scripts).

# Trigger test (1b) on one enable. Caps 6 V and 40 A. Full travel at 30 in/s, 200 in/s^2, ONE motor alone.
# fulltravel_noconfig: a routine start only switches the control requests (no config writes).
# fulltravel_reconfig: a routine start also rewrites every config (as soloright does).
# Right motor 4+4 alternating, then left motor 4+4 alternating. Scored afterwards from the robot
# log (apparent back-EMF at cruise per start): whichever kind of start jumps is the trigger.
# At the end the stand parks with motors off and the script waits for YOUR disable.
ORDER = (('N', 'fulltravel_noconfig'), ('R', 'fulltravel_reconfig')) * 4 \
      + (('N', 'fulltravel_noconfig_left'), ('R', 'fulltravel_reconfig_left')) * 4
SOLO_MOTION = (30, 200)      # cruise in/s, accel in/s^2
PEAK_VOLTS, AMPS = 6.0, 40
FULL_SECONDS = 8.0           # the routine is 5 targets x 1.5 s
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

def set_motion(cruise, accel):
    # parked, motors off: the robot writes new tuning to the Talons only while parked
    park()
    f.nt_set(TU + 'CruiseInPerSec', cruise)
    f.nt_set(TU + 'AccelInPerSec2', accel)
    f.sleep(0.5)

def run_full(part, n, name):
    # one full-travel run in the named drive; record what the robot publishes (the robot log decides)
    f.mark('%s%d %s' % (part, n, name))
    run_routine(name)
    t0, last, worst = f.now().monotonic, None, 0.0
    while f.now().monotonic - t0 < FULL_SECONDS:
        pos, _, st = f.nt_get_with_time(ST + 'PositionInches')
        if st != last:
            last = st
            err = f.nt_get(ST + 'ProfileErrorInches') or 0.0
            worst = max(worst, abs(err))
            f.record('samples', part=part, n=n, routine=name, server_time=st,
                     seconds=f.now().monotonic - t0, pos=pos,
                     target=f.nt_get(ST + 'TargetInches'), err=err)
        f.sleep(0.005)
    f.log('%s%d %s: worst path error seen %.3f in (10 Hz)' % (part, n, name, worst))
    park()

# Caps and the first speed, written while disabled
f.nt_set(TU + 'PeakVolts', PEAK_VOLTS)
f.nt_set(TU + 'StatorAmps', AMPS)
f.nt_set(TU + 'JerkInPerSec3', 0)
f.nt_set(TU + 'CruiseInPerSec', SOLO_MOTION[0])
f.nt_set(TU + 'AccelInPerSec2', SOLO_MOTION[1])

f.log('Enable in TELEOP. Everything runs by itself. At TEST DONE, please disable.')
f.wait_enabled(120)

# 0. Auto-range runs by itself after the new program starts; wait for it
f.mark('start')
f.sleep(1.0)
if running('AutoRange'):
    f.wait_until(lambda: not running('AutoRange'), 30)
f.log('travel %.3f in' % (f.nt_get(ST + 'TravelInches') or 0))

# One start each, alternating no-config and reconfig starts
set_motion(*SOLO_MOTION)
for n, (part, name) in enumerate(ORDER, start=1):
    run_full(part, n, name)

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
