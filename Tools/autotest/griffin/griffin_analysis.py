"""
Log analysis for the Griffin Pro SITL tests.

Scripted runs are measured against the controller's own reference (DESR) and state (FBL), plus LXTE
(cross-track), LROL (roll at script rate, roll differential thrust), LYAW (yaw error) and ATT.
No-script baseline runs are measured from POS/ATT against the loiter point.
t = 0 is the script start (first LXTE sample) or, for baselines, the given t0.
Note: ATT/POS are only logged at ~2-3 Hz in these SITL logs; LROL/LXTE/FBL run at script rate.
"""
import math

import numpy as np
from pymavlink import mavutil

from griffin_sim import LAND_LAT, M_PER_DEG_LAT

DTHR_LIMIT = 199  # us, saturation threshold for the roll differential thrust (AG.dthr_max = 200)


def _wrap(a):
    return (a + 180) % 360 - 180


def _read(path, types):
    m = mavutil.mavlink_connection(path)
    data = {t: [] for t in types}
    msgs = []
    while True:
        r = m.recv_match(type=list(types) + ['MSG'])
        if r is None:
            break
        ty = r.get_type()
        if ty == 'MSG':
            msgs.append((r.TimeUS / 1e6, r.Message))
        else:
            data[ty].append(r.to_dict())
    return data, msgs


def _arr(rows, *keys):
    return np.array([[r['TimeUS'] / 1e6] + [r[k] for k in keys] for r in rows]) if rows else np.zeros((0, len(keys) + 1))


def _win(a, lo, hi, col):
    s = a[(a[:, 0] >= lo) & (a[:, 0] < hi), col]
    return s if len(s) else np.array([np.nan])


def _absmax(x):
    return float(np.nanmax(np.abs(x)))


def analyze_scripted(path, t_step=None):
    """Metrics for a run where the Lua controller flew. t_step (s after script start) splits pre/post windows."""
    d, msgs = _read(path, ('LXTE', 'FBL', 'DESR', 'LROL', 'LYAW', 'ATT', 'POS', 'ARRS'))
    if not d['LXTE']:
        return {'script_ran': False}, None
    t0 = d['LXTE'][0]['TimeUS'] / 1e6
    xte = _arr(d['LXTE'], 'xte')
    fbl = _arr(d['FBL'], 'y1', 'y2', 'y3', 'y4')
    des = _arr(d['DESR'], 'y1', 'y2', 'y3', 'y4')
    lrol = _arr(d['LROL'], 'roll', 'dthrf', 'dthrr')
    lyaw = _arr(d['LYAW'], 'yerr')
    att = _arr(d['ATT'], 'Roll', 'Pitch', 'Yaw')
    arrs = _arr(d['ARRS'], 'u1_cmd', 'u2_cmd', 'u3_cmd', 'u4_cmd')
    for a in (xte, fbl, des, lrol, lyaw, att, arrs):
        if len(a):
            a[:, 0] -= t0
    T = xte[-1, 0]
    handoff = any('Landing Approach Completed' in s for _, s in msgs)
    hits = [(t - t0, s) for t, s in msgs if 'Hit ground' in s]
    crash = [h for h in hits if 0 <= h[0] <= T + 1]
    n = min(len(fbl), len(des))
    # x err: + = past the reference; h err: + = high (y2 = -h); pitch err deg
    err = np.c_[fbl[:n, 0], fbl[:n, 1] - des[:n, 1], -(fbl[:n, 2] - des[:n, 2]),
                np.degrees(fbl[:n, 3] - des[:n, 3]), fbl[:n, 4] - des[:n, 4]]
    roll = np.c_[lrol[:, 0], np.degrees(lrol[:, 1])]
    yaw = np.c_[lyaw[:, 0], np.degrees(lyaw[:, 1])]
    sat = lrol[(lrol[:, 0] >= 0) & (lrol[:, 0] <= T)]
    ts = t_step if t_step is not None else 0.0
    r = dict(script_ran=True, handoff=handoff, crashed=bool(crash),
             crash_t=round(crash[0][0], 1) if crash else None, script_s=round(T, 1),
             max_abs_xerr=_absmax(_win(err, ts, T, 1)), end_xerr=float(np.nanmean(_win(err, T - 3, T, 1))),
             max_abs_herr=_absmax(_win(err, ts, T, 2)), end_herr=float(np.nanmean(_win(err, T - 3, T, 2))),
             max_abs_pitch_err=_absmax(_win(err, ts, T, 3)),
             max_abs_xte=_absmax(_win(xte, ts, T, 1)), end_xte=float(np.nanmean(_win(xte, T - 3, T, 1))),
             max_abs_roll=_absmax(_win(roll, ts, T, 1)), end_roll=float(np.nanmean(_win(roll, T - 3, T, 1))),
             max_abs_pitch=_absmax(_win(att, ts, T, 2)),
             max_abs_yaw_err=_absmax(_win(yaw, ts, T, 1)), end_yaw_err=float(np.nanmean(_win(yaw, T - 3, T, 1))),
             dthr_sat_pct=float(100 * np.mean(np.abs(sat[:, 2:4]).max(1) >= DTHR_LIMIT)) if len(sat) else np.nan,
             handoff_xerr=float(np.nanmean(_win(err, T - 0.5, T, 1))),
             handoff_herr=float(np.nanmean(_win(err, T - 0.5, T, 2))),
             handoff_vx_err=float(np.nanmean(_win(err, T - 0.5, T, 4))))
    if t_step is not None:
        r.update(pre_max_abs_xerr=_absmax(_win(err, 5, t_step, 1)), pre_max_abs_xte=_absmax(_win(xte, 5, t_step, 1)),
                 pre_max_abs_roll=_absmax(_win(roll, 5, t_step, 1)))
    series = dict(kind='scripted', T=T, t_step=t_step, crash_t=r['crash_t'],
                  panels=[('x err [m]\n(+ = past ref)', [(err[:, 0], err[:, 1], 'x err')]),
                          ('xte [m]\n(+ = right)', [(xte[:, 0], xte[:, 1], 'xte')]),
                          ('h err [m]\n(+ = high)', [(err[:, 0], err[:, 2], 'h err')]),
                          ('deg', [(roll[:, 0], roll[:, 1], 'roll (LROL)'), (att[:, 0], att[:, 2], 'pitch (ATT)'),
                                   (err[:, 0], err[:, 3], 'pitch err'), (yaw[:, 0], yaw[:, 1], 'yaw err (LYAW)')]),
                          ('rpm cmd', [(arrs[:, 0], arrs[:, 1], 'front'), (arrs[:, 0], arrs[:, 2], 'rear')]),
                          ('tilt / elev [rad]', [(arrs[:, 0], arrs[:, 3], 'tilt'), (arrs[:, 0], arrs[:, 4], 'elevator')]),
                          ('roll dthr [us]', [(lrol[:, 0], lrol[:, 2], 'front'), (lrol[:, 0], lrol[:, 3], 'rear')])])
    return r, series


def analyze_baseline(path, t0_boot_s, hold_latlon, t_step=None, t_end=36.0):
    """Metrics for a no-script run from POS/ATT, relative to the loiter point hold_latlon at 50 m."""
    d, msgs = _read(path, ('POS', 'ATT'))
    kE = M_PER_DEG_LAT * math.cos(math.radians(LAND_LAT))
    pos = np.array([[r['TimeUS'] / 1e6 - t0_boot_s, (r['Lat'] - hold_latlon[0]) * M_PER_DEG_LAT,
                     (r['Lng'] - hold_latlon[1]) * kE, r['RelHomeAlt'] - 50] for r in d['POS']])
    att = _arr(d['ATT'], 'Roll', 'Pitch', 'Yaw')
    att[:, 0] -= t0_boot_s
    pre = att[(att[:, 0] >= 0) & (att[:, 0] < (t_step or 10)), 3]
    h0 = math.degrees(math.atan2(np.sin(np.radians(pre)).mean(), np.cos(np.radians(pre)).mean()))
    att[:, 3] = _wrap(att[:, 3] - h0)
    hits = [(t - t0_boot_s, s) for t, s in msgs if 'Hit ground' in s]
    crash = [h for h in hits if 0 <= h[0] <= t_end]
    ts = t_step or 0.0
    T = min(t_end, pos[-1, 0])
    r = dict(script_ran=False, crashed=bool(crash), crash_t=round(crash[0][0], 1) if crash else None,
             max_abs_along=_absmax(_win(pos, ts, T, 1)), max_abs_cross=_absmax(_win(pos, ts, T, 2)),
             end_cross=float(np.nanmean(_win(pos, T - 3, T, 2))), max_abs_herr=_absmax(_win(pos, ts, T, 3)),
             max_abs_roll=_absmax(_win(att, ts, T, 1)), max_abs_pitch=_absmax(_win(att, ts, T, 2)),
             max_abs_yaw_dev=_absmax(_win(att, ts, T, 3)), end_yaw_dev=float(np.nanmean(_win(att, T - 3, T, 3))))
    if t_step is not None:
        r['pre_max_abs_horiz'] = _absmax(np.hypot(_win(pos, 5, t_step, 1), _win(pos, 5, t_step, 2)))
    series = dict(kind='baseline', T=T, t_step=t_step, crash_t=r['crash_t'],
                  panels=[('along err [m]\n(+ = north)', [(pos[:, 0], pos[:, 1], 'along')]),
                          ('cross err [m]\n(+ = east)', [(pos[:, 0], pos[:, 2], 'cross')]),
                          ('h err [m]', [(pos[:, 0], pos[:, 3], 'h err')]),
                          ('deg', [(att[:, 0], att[:, 1], 'roll'), (att[:, 0], att[:, 2], 'pitch'),
                                   (att[:, 0], att[:, 3], 'yaw - hdg0')])])
    return r, series


def plot(series, title, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    panels = series['panels']
    T = series['T']
    f, ax = plt.subplots(len(panels), 1, figsize=(10, 1.9 * len(panels) + 1), sharex=True)
    for a, (label, lines) in zip(ax, panels):
        for t, y, name in lines:
            k = (t >= -2) & (t <= T + 6)
            a.plot(t[k], y[k], lw=0.8, label=name)
        a.set_ylabel(label)
        a.grid(alpha=0.3)
        if len(lines) > 1:
            a.legend(loc='upper right', fontsize=7)
        if series.get('t_step') is not None:
            a.axvline(series['t_step'], color='r', ls='--', lw=1)
        a.axvline(T, color='k', ls=':', lw=1)
        if series.get('crash_t') is not None:
            a.axvline(series['crash_t'], color='r', lw=1.5)
    ax[-1].set_xlim(-2, T + 6)
    ax[-1].set_xlabel('time [s]  (red dashed: disturbance, black dotted: hand-off/end, red solid: ground impact)')
    ax[0].set_title(title)
    f.tight_layout()
    f.savefig(path, dpi=90)
    plt.close(f)
