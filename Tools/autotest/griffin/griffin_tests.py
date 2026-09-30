#!/usr/bin/env python3
"""
Griffin Pro landing-controller SITL test suite.

  griffin_tests.py wind      hover hold, wind 0 until t0+15 s, then a step (head / 45 deg / cross)
  griffin_tests.py baseline  same wind steps with scripting disabled (ArduPilot VTOL loiter)
  griffin_tests.py moving    no wind, the script triggers with the aircraft moving forward / sideways
  griffin_tests.py approach  full landing approach: fixed-wing cruise, script triggers 400 m out and flies K/U/Y
  griffin_tests.py all       everything above with the default matrices

See README.md for the setup of each test, the metrics and the PASS criteria.
"""
import argparse
import csv
import datetime
import json
import math
import os
import queue
import shutil
import sys
import threading
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import griffin_analysis as ga  # noqa: E402
from griffin_sim import (ASSETS, LAND_LAT, LAND_LON, MODE_AUTO, MODE_QLOITER, Sitl,  # noqa: E402
                         ne_from_land, offset_latlon)

MOTORS = {'fastmot': 168, 'lagmot': 0}   # SIM_VIB_MOT_MAX: 168 overwrites the motor rpm state (no spin-up lag)
WIND_DIRS = {'head': 0, 'x45': 45, 'cross': 90}   # SIM_WIND_DIR = direction the wind comes FROM, aircraft faces N
T_STEP = 15.0            # s after t0
BASELINE_T_END = 36.0    # s after t0
HANDOFF_MARGIN_MS = 6000

# hover start: take off 399 m due S of the VTOL_LAND WP, force VTOL, loiter -> the script triggers from hover
HOVER_LAT, HOVER_LON = offset_latlon(-399.0, 0.0)
HOVER_MISSION = [(16, 0, HOVER_LAT, HOVER_LON, 223.94, 0), (84, 3, HOVER_LAT, HOVER_LON, 50, 0),
                 (3000, 3, 0, 0, 0, 3), (19, 3, HOVER_LAT, HOVER_LON, 50, 120), (85, 3, LAND_LAT, LAND_LON, 0, 0)]
# forward-flight approach mission from flight log 255 (VTOL takeoff, 3 WPs at 50 m, VTOL_LAND)
APPROACH_HOME = (40.0594879, -88.5513138)
APPROACH_MISSION = [(16, 0, 40.0594878, -88.5513136, 223.94, 0), (84, 3, 40.0594898, -88.5513114, 10, 0),
                    (16, 3, 40.0556726, -88.5579586, 50, 0), (16, 3, 40.0542437, -88.5566497, 50, 0),
                    (16, 3, 40.0580869, -88.5525727, 50, 0), (85, 3, LAND_LAT, LAND_LON, 50, 0)]
MOVE_TRIGGER_N = -399.5  # trigger point P, m N of the VTOL_LAND WP (on the approach axis)


# ============================================================== scenario runners (one SITL each)
def _watch_text(s, r, ev):
    """Common STATUSTEXT bookkeeping. Returns True when the text was a hand-off."""
    t = r.text
    if 'transition time' in t and ev.get('t0_ms') is None and ev.get('scripted', True):
        ev['t0_ms'] = float(t.split('=')[1])
        s.log('script t0 at boot ms', ev['t0_ms'])
    if 'Hit ground' in t:
        ev.setdefault('hit_ground', []).append((s.last_boot_ms, t))
    if 'Landing Approach Completed' in t and ev.get('handoff_ms') is None:
        ev['handoff_ms'] = s.last_boot_ms
        s.log('hand-off at boot ms', s.last_boot_ms)
    if any(k in t for k in ('Transition', 'transition failed', 'Crash', 'Thrust Loss')):
        s.log(t)


def run_wind(s, job):
    """Hover hold, wind step at t0+15 s. Scripted: t0 = script start. Baseline: t0 = LOITER item reached."""
    ev = dict(scripted=job['scripted'])
    s.wait_gps()
    s.upload_mission(HOVER_MISSION)
    s.arm()
    s.set_mode(MODE_AUTO)
    s.message_interval(30, 20)   # ATTITUDE for boot time
    t_wall = time.time()
    while time.time() - t_wall < job['timeout']:
        r = s.recv(['STATUSTEXT', 'ATTITUDE', 'MISSION_CURRENT', 'HEARTBEAT'])
        if r is None:
            continue
        ty = r.get_type()
        if ty == 'STATUSTEXT':
            _watch_text(s, r, ev)
        elif ty == 'MISSION_CURRENT' and not job['scripted'] and r.seq == 3 and ev.get('t0_ms') is None \
                and s.last_boot_ms > 0:
            ev['t0_ms'] = float(s.last_boot_ms)
            s.log('LOITER reached, t0 at boot ms', ev['t0_ms'])
        t0 = ev.get('t0_ms')
        if t0 is not None and ev.get('step_ms') is None and s.last_boot_ms - t0 >= T_STEP * 1000:
            s.pset('SIM_WIND_SPD', job['wind_speed'])
            s.pset('SIM_WIND_DIR', job['wind_dir'])
            ev['step_ms'] = s.last_boot_ms
            s.log('wind step %g m/s from %g deg at +%.2f s' % (job['wind_speed'], job['wind_dir'],
                                                                (s.last_boot_ms - t0) / 1000))
        if job['scripted'] and ev.get('handoff_ms') and s.last_boot_ms - ev['handoff_ms'] > HANDOFF_MARGIN_MS:
            break
        if not job['scripted'] and t0 is not None and s.last_boot_ms - t0 >= BASELINE_T_END * 1000:
            break
    else:
        ev['timeout'] = True
        s.log('TIMEOUT')
    return ev


def run_moving(s, job):
    """No wind. QLOITER run-up with stick loops (heading 0, hold the line, full stick along the run), switch to AUTO
    when the trigger point P is crossed -> the script triggers on its next loop with the aircraft at speed.
    carry: wind following the ground speed during the run-up (aircraft sees still air), zeroed at the switch."""
    ev = dict(scripted=True)
    v, dr = job['speed'], job['direction']
    sn, se = (MOVE_TRIGGER_N - job['runup'], 0.0) if dr == 'fwd' else (MOVE_TRIGGER_N, -job['runup'])
    start = offset_latlon(sn, se)
    items = [(16, 0, start[0], start[1], 223.94, 0), (84, 3, start[0], start[1], 50, 0), (3000, 3, 0, 0, 0, 3),
             (19, 3, start[0], start[1], 50, 120), (85, 3, LAND_LAT, LAND_LON, 0, 0)]
    s.wait_gps()
    s.upload_mission(items)
    s.arm()
    s.set_mode(MODE_AUTO)
    s.message_interval(33, 50)   # GLOBAL_POSITION_INT
    s.message_interval(30, 50)   # ATTITUDE
    clamp = lambda x, lim: max(-lim, min(lim, x))  # noqa: E731
    dz = lambda x: 0 if abs(x) < 35 else x          # noqa: E731
    stage, yaw, settle_ms, wind_now = 'takeoff', None, None, 0.0
    t_wall = time.time()
    while time.time() - t_wall < job['timeout']:
        r = s.recv(['STATUSTEXT', 'MISSION_CURRENT', 'GLOBAL_POSITION_INT', 'ATTITUDE'])
        if r is None:
            continue
        ty = r.get_type()
        if ty == 'ATTITUDE':
            yaw = math.degrees(r.yaw)
        elif ty == 'STATUSTEXT':
            _watch_text(s, r, ev)
        elif ty == 'MISSION_CURRENT' and stage == 'takeoff' and r.seq == 3:
            s.set_mode(MODE_QLOITER)
            stage = 'settle'
        elif ty == 'GLOBAL_POSITION_INT' and stage in ('settle', 'runup') and yaw is not None:
            n, e = ne_from_land(r.lat / 1e7, r.lon / 1e7)
            vn, ve = r.vx / 100, r.vy / 100
            settle_ms = settle_ms or r.time_boot_ms
            ry = 1500 + dz(clamp(-25 * ((yaw + 180) % 360 - 180), 300))
            if dr == 'fwd':
                rr = 1500 + clamp(-(40 * (e - se) + 150 * ve), 386)
                rp = 1100 if stage == 'runup' else 1500 + clamp(40 * (n - sn) + 150 * vn, 400)
            else:
                rp = 1500 + clamp(40 * (n - sn) + 150 * vn, 400)
                rr = 1886 if stage == 'runup' else 1500 + clamp(-(40 * (e - se) + 150 * ve), 386)
            s.rc_override(rr, rp, 1500, ry)
            if job['carry'] and stage == 'runup':
                wt = min(v, max(0.0, vn if dr == 'fwd' else ve))
                if abs(wt - wind_now) > 0.25:
                    wind_now = wt
                    s.pset('SIM_WIND_DIR', 180.0 if dr == 'fwd' else 270.0)
                    s.pset('SIM_WIND_SPD', wind_now)
            if stage == 'settle' and r.time_boot_ms - settle_ms > 8000:
                stage = 'runup'
            elif stage == 'runup' and ((dr == 'fwd' and n >= MOVE_TRIGGER_N) or (dr == 'side' and e >= 0)):
                s.set_mode(MODE_AUTO)
                if job['carry']:
                    s.pset('SIM_WIND_SPD', 0.0)
                s.rc_override(0, 0, 0, 0)
                ev['switch'] = dict(boot_ms=r.time_boot_ms, n=n, e=e, vn=vn, ve=ve, vd=r.vz / 100,
                                    alt=r.relative_alt / 1000, yaw=yaw, carry_wind=wind_now)
                stage = 'auto'
                s.log('P crossed -> AUTO: vN %.2f vE %.2f yaw %.1f' % (vn, ve, yaw))
        if ev.get('handoff_ms') and s.last_boot_ms - ev['handoff_ms'] > HANDOFF_MARGIN_MS:
            break
    else:
        ev['timeout'] = True
        s.log('TIMEOUT in stage', stage)
    return ev


def run_approach(s, job):
    """Mission from log 255: VTOL takeoff, fixed-wing to the WPs at 50 m, script triggers 400 m from VTOL_LAND."""
    ev = dict(scripted=True)
    s.wait_gps()
    s.upload_mission(APPROACH_MISSION)
    s.arm()
    s.set_mode(MODE_AUTO)
    s.message_interval(30, 20)
    t_wall = time.time()
    while time.time() - t_wall < job['timeout']:
        r = s.recv(['STATUSTEXT', 'ATTITUDE'])
        if r is not None and r.get_type() == 'STATUSTEXT':
            _watch_text(s, r, ev)
        if ev.get('handoff_ms') and s.last_boot_ms - ev['handoff_ms'] > HANDOFF_MARGIN_MS:
            break
    else:
        ev['timeout'] = True
        s.log('TIMEOUT')
    return ev


RUNNERS = {'wind': run_wind, 'baseline': run_wind, 'moving': run_moving, 'approach': run_approach}


# ============================================================== job matrices
def _floats(txt):
    return [float(x) for x in txt.split(',') if x]


def build_jobs(suite, a):
    motors = a.motor.split(',')
    jobs = []
    if suite in ('wind', 'baseline'):
        for mot in motors:
            for d in a.dirs.split(','):
                for sp in _floats(a.speeds):
                    jobs.append(dict(suite=suite, tag='%s_%s_%s_%g' % (suite, mot, d, sp), motor=mot,
                                     scripted=suite == 'wind', wind_dir=WIND_DIRS[d], wind_speed=sp,
                                     arrays=a.arrays or os.path.join(ASSETS, 'arrays', 'hover_hold'),
                                     home=(HOVER_LAT, HOVER_LON), extra={} if suite == 'wind' else {'SCR_ENABLE': 0}))
    elif suite == 'moving':
        for mot in motors:
            for d in a.move_dirs.split(','):
                for sp in _floats(a.speeds):
                    carry = d == 'side' and not a.no_carry
                    runup = max(80.0, 12 * sp)
                    sn, se = (MOVE_TRIGGER_N - runup, 0.0) if d == 'fwd' else (MOVE_TRIGGER_N, -runup)
                    jobs.append(dict(suite=suite, tag='moving_%s_%s_%g' % (mot, d, sp), motor=mot, scripted=True,
                                     direction=d, speed=sp, carry=carry, runup=runup,
                                     arrays=a.arrays or os.path.join(ASSETS, 'arrays', 'hover_hold'),
                                     home=offset_latlon(sn, se), extra={'Q_ENABLE': 2, 'Q_LOIT_SPEED_MS': sp}))
    elif suite == 'approach':
        for mot in motors:
            for w in a.approach_wind.split(';'):
                spd, dr = _floats(w)
                jobs.append(dict(suite=suite, tag='approach_%s_w%g_%g' % (mot, spd, dr), motor=mot, scripted=True,
                                 arrays=a.arrays or os.path.join(ASSETS, 'arrays', 'approach_31s'),
                                 home=APPROACH_HOME, extra={'SIM_WIND_SPD': spd, 'SIM_WIND_DIR': dr}))
    for j in jobs:
        j['timeout'] = a.timeout
    return jobs


# ============================================================== execution + analysis
def execute(job, inst, a, out):
    run_dir = os.path.join(out, 'runs', job['tag'])
    params = {'SIM_VIB_MOT_MAX': MOTORS[job['motor']], 'SIM_WIND_SPD': 0, 'SIM_WIND_DIR': 0, 'SIM_WIND_TURB': 0}
    params.update(job['extra'])
    res = dict(tag=job['tag'], suite=job['suite'], motor=job['motor'])
    try:
        with Sitl(run_dir, inst, job['home'], params=params, script=a.script if job['scripted'] else None,
                  arrays=job['arrays'], binary=a.binary, speedup=a.speedup, tag=job['tag']) as s:
            ev = RUNNERS[job['suite']](s, job)
            log = s.bin_log()
        res['events'] = ev
        if log:
            os.makedirs(os.path.join(out, 'logs'), exist_ok=True)
            dst = os.path.join(out, 'logs', job['tag'] + '.BIN')
            shutil.copy(log, dst)
            res['log'] = dst
            res.update(analyse(job, ev, dst, out))
    except Exception as e:
        res['error'] = '%s: %s' % (type(e).__name__, e)
        traceback.print_exc()
    if not a.keep_runs:
        shutil.rmtree(run_dir, ignore_errors=True)
    res['pass'], res['why'] = verdict(job, res)
    print('[%s] %s %s' % (job['tag'], 'PASS' if res['pass'] else 'FAIL', res['why']), flush=True)
    return res


def analyse(job, ev, log, out):
    os.makedirs(os.path.join(out, 'plots'), exist_ok=True)
    if job['scripted']:
        m, series = ga.analyze_scripted(log, t_step=T_STEP if job['suite'] == 'wind' else None)
    else:
        m, series = ga.analyze_baseline(log, ev['t0_ms'] / 1000, (HOVER_LAT, HOVER_LON), t_step=T_STEP,
                                        t_end=BASELINE_T_END)
    if series:
        title = job['tag']
        if 'switch' in ev:
            sw = ev['switch']
            title += '   trigger: vN %.2f vE %.2f m/s, yaw %.1f deg' % (sw['vn'], sw['ve'], sw['yaw'])
        if m.get('crashed'):
            title += '   CRASHED'
        ga.plot(series, title, os.path.join(out, 'plots', job['tag'] + '.png'))
    return {'metrics': m}


def verdict(job, res):
    if 'error' in res:
        return False, res['error']
    ev, m = res.get('events', {}), res.get('metrics', {})
    if ev.get('timeout'):
        return False, 'timed out'
    if m.get('crashed'):
        return False, 'ground impact at t=%s s' % m.get('crash_t')
    if job['scripted']:
        if not m.get('script_ran'):
            return False, 'script never took over'
        if not m.get('handoff'):
            return False, 'no hand-off message'
        return True, 'held to hand-off'
    return True, 'held to t0+%g s' % BASELINE_T_END


def run_suite(suite, a, out):
    jobs = build_jobs(suite, a)
    print('== %s: %d runs, %d in parallel' % (suite, len(jobs), a.parallel), flush=True)
    q = queue.Queue()
    for j in jobs:
        q.put(j)
    results, lock = [], threading.Lock()

    def worker(inst):
        while True:
            try:
                j = q.get_nowait()
            except queue.Empty:
                return
            r = execute(j, inst, a, out)
            with lock:
                results.append(r)

    ts = [threading.Thread(target=worker, args=(a.instance_base + i,)) for i in range(a.parallel)]
    [t.start() for t in ts]
    [t.join() for t in ts]
    order = {j['tag']: i for i, j in enumerate(jobs)}
    return sorted(results, key=lambda r: order[r['tag']])


def write_summary(results, out):
    rows = []
    for r in results:
        row = dict(tag=r['tag'], suite=r['suite'], motor=r['motor'], result='PASS' if r['pass'] else 'FAIL',
                   why=r['why'])
        sw = r.get('events', {}).get('switch')
        if sw:
            row.update(trigger_vn=sw['vn'], trigger_ve=sw['ve'], trigger_yaw=sw['yaw'])
        row.update(r.get('metrics', {}))
        rows.append(row)
    keys = []
    for row in rows:
        keys += [k for k in row if k not in keys]
    with open(os.path.join(out, 'summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        for row in rows:
            w.writerow({k: ('%.3f' % v if isinstance(v, float) else v) for k, v in row.items()})
    json.dump(results, open(os.path.join(out, 'results.json'), 'w'), indent=1, default=str)
    cols = [('max_abs_xerr', 'xerr'), ('max_abs_xte', 'xte'), ('max_abs_herr', 'herr'), ('max_abs_roll', 'roll'),
            ('max_abs_pitch', 'pitch'), ('max_abs_yaw_err', 'yaw'), ('max_abs_cross', 'cross'),
            ('max_abs_yaw_dev', 'yawdev'), ('dthr_sat_pct', 'sat%')]
    print('\n%-34s %-5s ' % ('run', 'res') + ' '.join('%7s' % c[1] for c in cols))
    for row in rows:
        print('%-34s %-5s ' % (row['tag'], row['result']) +
              ' '.join('%7s' % ('%.1f' % row[k] if isinstance(row.get(k), float) else '-') for k, _ in cols))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('suite', choices=['wind', 'baseline', 'moving', 'approach', 'all'])
    p.add_argument('--motor', default='fastmot,lagmot', help='fastmot (SIM_VIB_MOT_MAX 168) and/or lagmot (0)')
    p.add_argument('--dirs', default='head,x45,cross', help='wind directions for wind/baseline')
    p.add_argument('--move-dirs', default='fwd,side', help='directions for moving')
    p.add_argument('--speeds', default='2.5,5,7.5,10', help='wind step / run-up speeds [m/s]')
    p.add_argument('--no-carry', action='store_true', help='moving/side: still-air run-up (only works to ~5 m/s)')
    p.add_argument('--approach-wind', default='0,0', help='approach winds "spd,dir;spd,dir"')
    p.add_argument('--script', default=os.path.join(ASSETS, 'controller_spd.v2.lua'))
    p.add_argument('--arrays', default=None, help='K/U/Y directory (with meta.json) instead of the suite default')
    p.add_argument('--binary', default=None, help='arduplane SITL binary (default: <repo>/build/sitl/bin/arduplane)')
    p.add_argument('--parallel', type=int, default=6)
    p.add_argument('--speedup', type=int, default=10)
    p.add_argument('--instance-base', type=int, default=1,
                   help='first SITL instance (-I); use distinct ranges for concurrent invocations')
    p.add_argument('--timeout', type=float, default=600, help='wall-clock seconds per run')
    p.add_argument('--out', default=None)
    p.add_argument('--keep-runs', action='store_true', help='keep the per-run SITL directories')
    a = p.parse_args()
    a.script = os.path.abspath(a.script)
    out = a.out or os.path.join(HERE, 'results', '%s_%s' % (a.suite, datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))
    os.makedirs(out, exist_ok=True)
    json.dump(vars(a), open(os.path.join(out, 'args.json'), 'w'), indent=1)
    suites = ['wind', 'baseline', 'moving', 'approach'] if a.suite == 'all' else [a.suite]
    results = []
    for s in suites:
        results += run_suite(s, a, out)
    write_summary(results, out)
    n_fail = sum(not r['pass'] for r in results)
    print('\n%d/%d PASS, results in %s' % (len(results) - n_fail, len(results), out))
    sys.exit(1 if n_fail else 0)


if __name__ == '__main__':
    main()
