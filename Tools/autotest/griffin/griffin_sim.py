"""
SITL harness for the Griffin Pro tilt-quadplane landing-controller tests.

Runs build/sitl/bin/arduplane directly (not sim_vehicle.py) with the quadplane-tilt frame, the
GriffinPro.json model and the Lua controller + K/U/Y arrays from assets/. One Sitl object = one
SITL instance in its own run directory; several can run in parallel with different instance numbers.
"""
import glob
import json
import os
import re
import shutil
import subprocess
import time

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, 'assets')
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))

# VTOL_LAND waypoint used by every test (from flight log 255)
LAND_LAT, LAND_LON = 40.0594829, -88.5513282
HOME_ALT = 224
M_PER_DEG_LAT = 111320.0

MODE_AUTO = 10
MODE_QLOITER = 19


def default_binary():
    return os.path.join(REPO, 'build', 'sitl', 'bin', 'arduplane')


def offset_latlon(north_m, east_m, lat=LAND_LAT, lon=LAND_LON):
    import math
    return (lat + north_m / M_PER_DEG_LAT,
            lon + east_m / (M_PER_DEG_LAT * math.cos(math.radians(lat))))


def ne_from_land(lat, lon):
    import math
    return ((lat - LAND_LAT) * M_PER_DEG_LAT,
            (lon - LAND_LON) * M_PER_DEG_LAT * math.cos(math.radians(LAND_LAT)))


def patch_script(src_text, t_total, n_steps):
    """Set sim_time_total / sim_time_steps in the controller to match the array set (keeps CRLF)."""
    out, n1 = re.subn(r'(local sim_time_total\s*=\s*)[0-9.]+', r'\g<1>%g' % t_total, src_text, count=1)
    out, n2 = re.subn(r'(local sim_time_steps\s*=\s*)[0-9]+', r'\g<1>%d' % n_steps, out, count=1)
    if n1 != 1 or n2 != 1:
        raise ValueError('controller script has no "local sim_time_total=" / "local sim_time_steps=" lines')
    return out


def array_set_info(arrays_dir):
    meta = json.load(open(os.path.join(arrays_dir, 'meta.json')))
    with open(os.path.join(arrays_dir, 'Y.csv')) as f:
        n_steps = len(f.readline().strip().split(','))
    return meta['t_total'], n_steps


class Sitl:
    """One SITL instance. Use as a context manager; the process is stopped on exit."""

    def __init__(self, run_dir, inst, home, params=None, script=None, arrays=None,
                 binary=None, speedup=10, tag=None):
        self.run_dir = run_dir
        self.inst = inst
        self.home = home                  # (lat, lon)
        self.params = dict(params or {})  # written to an extra --defaults file (last, so it wins)
        self.script = script              # path to the controller .lua, or None for no script
        self.arrays = arrays              # directory with K/U/Y.csv + meta.json
        self.binary = binary or default_binary()
        self.speedup = speedup
        self.tag = tag or os.path.basename(run_dir)
        self.proc = None
        self.mav = None
        self.last_boot_ms = 0

    # ---------------------------------------------------------------- setup / teardown
    def _prepare(self):
        shutil.rmtree(self.run_dir, ignore_errors=True)
        os.makedirs(os.path.join(self.run_dir, 'scripts'))
        # relative model path: SITL resolves absolute JSON paths relative to its cwd
        shutil.copy(os.path.join(ASSETS, 'GriffinPro.json'), self.run_dir)
        if self.script:
            t_total, n_steps = array_set_info(self.arrays)
            src = open(self.script, newline='').read()
            with open(os.path.join(self.run_dir, 'scripts', 'controller.lua'), 'w', newline='') as f:
                f.write(patch_script(src, t_total, n_steps))
            for c in 'KUY':
                shutil.copy(os.path.join(self.arrays, c + '.csv'), os.path.join(self.run_dir, 'scripts'))
        with open(os.path.join(self.run_dir, 'test.parm'), 'w') as f:
            for k, v in self.params.items():
                f.write('%s %s\n' % (k, v))

    def __enter__(self):
        if not os.path.exists(self.binary):
            raise FileNotFoundError('%s not found - build with ./waf configure --board sitl && ./waf plane '
                                    'or pass --binary' % self.binary)
        self._prepare()
        dp = os.path.join(REPO, 'Tools', 'autotest', 'default_params')
        defaults = ','.join([os.path.join(dp, 'quadplane.parm'), os.path.join(dp, 'quadplane-tilt.parm'),
                             os.path.join(ASSETS, 'GriffinPro.parm'), os.path.join(ASSETS, 'griffin_sitl.parm'),
                             os.path.join(self.run_dir, 'test.parm')])
        self.sitl_out = open(os.path.join(self.run_dir, 'sitl.out'), 'w')
        self.proc = subprocess.Popen(
            [self.binary, '--model', 'quadplane-tilt:GriffinPro.json', '--defaults', defaults,
             '--speedup', str(self.speedup), '--wipe', '-I%d' % self.inst,
             '--home', '%.7f,%.7f,%d,0' % (self.home[0], self.home[1], HOME_ALT)],
            cwd=self.run_dir, stdout=self.sitl_out, stderr=subprocess.STDOUT)
        time.sleep(3)
        self.mav = mavutil.mavlink_connection('tcp:127.0.0.1:%d' % (5760 + 10 * self.inst),
                                              source_system=255, retries=30)
        self.mav.wait_heartbeat(timeout=60)
        return self

    def __exit__(self, *exc):
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.sitl_out.close()
        return False

    def log(self, *x):
        print('[%s]' % self.tag, *x, flush=True)

    def bin_log(self):
        bins = sorted(glob.glob(os.path.join(self.run_dir, 'logs', '*.BIN')), key=os.path.getmtime)
        return bins[-1] if bins else None

    # ---------------------------------------------------------------- MAVLink helpers
    def pset(self, name, value):
        m = self.mav
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def recv(self, types, timeout=1):
        r = self.mav.recv_match(type=types, blocking=True, timeout=timeout)
        if r is not None and hasattr(r, 'time_boot_ms'):
            self.last_boot_ms = r.time_boot_ms
        return r

    def message_interval(self, msg_id, hz):
        m = self.mav
        m.mav.command_long_send(m.target_system, m.target_component, 511, 0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    def wait_gps(self, timeout=120):
        t0 = time.time()
        while time.time() - t0 < timeout:
            r = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=5)
            if r and 'is using GPS' in r.text:
                time.sleep(2)
                return
        raise RuntimeError('EKF never started using GPS')

    def upload_mission(self, items):
        """items: (cmd, frame, lat, lon, alt, param1). Dedupe repeated MISSION_REQUESTs (ACK 13 otherwise)."""
        m = self.mav
        for _ in range(5):
            m.mav.mission_count_send(m.target_system, m.target_component, len(items))
            ok, sent = True, {}
            while True:
                r = m.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
                                 blocking=True, timeout=10)
                if r is None:
                    ok = False
                    break
                if r.get_type() == 'MISSION_ACK':
                    ok = r.type == 0
                    break
                if r.seq in sent and time.time() - sent[r.seq] < 1.5:
                    continue
                sent[r.seq] = time.time()
                c, fr, la, lo, al, p1 = items[r.seq]
                m.mav.mission_item_int_send(m.target_system, m.target_component, r.seq, fr, c, 0, 1,
                                            p1, 0, 0, 0, int(la * 1e7), int(lo * 1e7), al)
            if ok:
                return
        raise RuntimeError('mission upload failed')

    def arm(self, timeout=180):
        """Force-arm (21196). Must happen BEFORE switching to AUTO or the takeoff item is consumed."""
        m = self.mav
        t0 = time.time()
        while time.time() - t0 < timeout:
            m.mav.command_long_send(m.target_system, m.target_component, 400, 0, 1, 21196, 0, 0, 0, 0, 0)
            r = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if m.motors_armed() or (r and r.command == 400 and r.result == 0):
                return
            time.sleep(2)
        raise RuntimeError('arm timeout')

    def set_mode(self, mode):
        self.mav.set_mode(mode)

    def rc_override(self, ch1=0, ch2=0, ch3=0, ch4=0):
        m = self.mav
        m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                        int(ch1), int(ch2), int(ch3), int(ch4), 0, 0, 0, 0)
