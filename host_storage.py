# -*- coding: utf-8 -*-
"""
host_storage.py - the controller's own copies of things.

PresetLibrary  ~/.manta/presets.json  slot -> name, period, motors, cycles.
               The Pico stores only period + motors; names and the cycle
               count live here. The library is the master copy that gets
               pushed to the Pico when the sync check says they differ.
LogStore       ~/manta_logs/*.bin     sequence logs received from the Pico,
               with CSV export.
"""
import json
import os
import time

import pico_protocol as P


class PresetLibrary:
    def __init__(self, path=os.path.expanduser('~/.manta/presets.json')):
        self.path = path
        self.presets = {}
        self.load()

    def load(self):
        try:
            with open(self.path) as f:
                raw = json.load(f)
            self.presets = {int(k): v for k, v in raw.items()}
        except (OSError, ValueError):
            self.presets = {}

    def save(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump({str(k): v for k, v in sorted(self.presets.items())}, f, indent=1)
        os.replace(tmp, self.path)

    def put(self, slot, name, period, motors, cycles):
        self.presets[slot] = {'name': name, 'period': float(period),
                              'motors': [[int(d), float(l), float(p)] for d, l, p in motors],
                              'cycles': int(cycles)}
        self.save()

    def delete(self, slot):
        self.presets.pop(slot, None)
        self.save()

    def clear(self):
        self.presets = {}
        self.save()

    def crc(self):
        return P.presets_crc({k: (v['period'], v['motors']) for k, v in self.presets.items()})

    def free_slot(self):
        for s in range(P.MAX_PRESET_ID + 1):
            if s not in self.presets:
                return s
        return None


class LogStore:
    def __init__(self, folder=os.path.expanduser('~/manta_logs')):
        self.folder = folder
        os.makedirs(folder, exist_ok=True)

    def save(self, raw, preset_name=''):
        hdr, _ = P.decode_log(raw)
        pid = 'temp' if hdr['preset'] == P.TEMP_PRESET_ID else 'p%03d' % hdr['preset']
        name = time.strftime('seq_%Y%m%d_%H%M%S_') + pid + '.bin'
        with open(os.path.join(self.folder, name), 'wb') as f:
            f.write(raw)
        return name

    def list(self):
        """Newest first: [(file name, header dict or None)]."""
        out = []
        for name in sorted(os.listdir(self.folder), reverse=True):
            if not name.endswith('.bin'):
                continue
            try:
                hdr, _ = self.load(name)
            except (OSError, ValueError):
                hdr = None
            out.append((name, hdr))
        return out

    def load(self, name):
        with open(os.path.join(self.folder, name), 'rb') as f:
            return P.decode_log(f.read())

    def export_csv(self, name):
        hdr, samples = self.load(name)
        cpr = hdr['counts_per_rev'] or 1
        path = os.path.join(self.folder, name[:-4] + '.csv')
        with open(path, 'w') as f:
            f.write('# preset %s, cycles %d, period %.3f s, interval %d ms, %s\n'
                    % (hdr['preset'], hdr['cycles'], hdr['period'], hdr['interval_ms'],
                       P.flags_text(hdr['flags'])))
            f.write('t_ms,' + ','.join('m%d_counts' % i for i in range(1, 5)) + ','
                    + ','.join('m%d_deg' % i for i in range(1, 5)) + '\n')
            base = samples[0][1:] if samples else (0, 0, 0, 0)
            for s in samples:
                degs = ['%.2f' % ((c - b) * 360.0 / cpr) for c, b in zip(s[1:], base)]
                f.write('%d,%d,%d,%d,%d,' % s + ','.join(degs) + '\n')
        return path

    def delete(self, name):
        for ext in ('.bin', '.csv'):
            try:
                os.remove(os.path.join(self.folder, name[:-4] + ext))
            except OSError:
                pass
