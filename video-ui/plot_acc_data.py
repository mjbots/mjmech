#!/usr/bin/env python
import csv
import sys
import numpy
import os
import time
import json
from matplotlib import ticker, pyplot

from vui_helpers import FCMD

class DataSet(object):
    def __init__(self):
        self.titles = []
        # Set of numpy named arrays. each array must have
        # 'ts' field which is a unix timestamp
        self.data = dict()

    def parse_acc_data(self, fname):
        print 'Parsing accelerometer %r' % fname
        self.titles.append(
            os.path.basename(fname).replace('.txt', ''))

        raw = {'0': [], '1': []}
        with open(fname, 'r') as fh:
            for row in csv.reader(fh):
                dt = float(row[0])
                x, y, z = [float(v) for v in row[2:5]]
                raw[row[1]].append((dt, x, y, z))

        for ch in ['0', '1']:
            self._append_data(
                'acc' + ch, raw[ch],
                [('ts', 'f8'), ('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

    def _append_data(self, dataset, data, dtype):
        old = self.data.get(dataset, numpy.array([], dtype=dtype))
        self.data[dataset] = numpy.hstack((old, numpy.array(data, dtype=dtype)))
        self.data[dataset].sort(order='ts')


    def parse_jsonlog_data(self, src):
        print 'Parsing JSON log %r' % src,
        last_cd = None
        cd_data = list()
        ss_data = list()
        events = { n: [] for n in ('fire_cmd', 'turret_cmd',
                                   'fire_sent', 'firing_failed',
                                   'udp_loss') }
        for line_txt in open(src, 'r'):
            line = json.loads(line_txt)
            if line['_type'] == 'control-dict':
                ts = line['cli_time']
                fire_cmd = 0
                if last_cd is not None:
                    if last_cd.get('turret') != line.get('turret'):
                        events['turret_cmd'].append((ts, ))
                    if last_cd.get('fire_cmd') != line.get('fire_cmd'):
                        events['fire_cmd'].append((ts, ))

                if line.get('fire_cmd') and \
                   line['fire_cmd_deadline'] > ts:
                    fire_cmd = line['fire_cmd'][0]

                cd_data.append((
                    ts, line['agitator_on'],
                    int(fire_cmd == FCMD.inpos3),
                    int(fire_cmd not in [0, FCMD.inpos3])))
                last_cd = line
            elif line['_type'] == 'srv-state':
                ts = line['srv_time']
                servo_status = line.get('servo_status', {})
                firing = ('moving' in servo_status.get("99", ''))
                turret_moving = 0
                status_x = servo_status.get("12", "")
                status_y = servo_status.get("13", "")
                ss_data.append((ts, firing,
                                int('moving' in status_x),
                                int('moving' in status_y),
                                int('inposition' not in status_x),
                                int('inposition' not in status_y),
                            ))
            elif line['_type'] == 'srv-log':
                ts = line['srv_time']
                message = line['message']
                if message.startswith('Bang bang'):
                    events['fire_sent'].append((ts, ))
                elif message.startswith('Seq number jump'):
                    events['udp_loss'].append((ts, ))
                elif message.startswith('Ignoring old fire_cmd '):
                    events['firing_failed'].append((ts, ))

        print '%d control, %d server' % (len(cd_data), len(ss_data))

        self._append_data(
            'control', cd_data, [
                ('ts', 'f8'), ('agitator_on', 'u1'),
                ('firing_ip3', 'u1'), ('firing_other', 'u1')])
        self._append_data(
            'server', ss_data, [
                ('ts', 'f8'), ('firing', 'i1'),
                ('moving_x', 'i1'), ('moving_y', 'i1'),
                ('ninpos_x', 'i1'), ('ninpos_y', 'i1'),
                ])
        for key, val in events.items():
            self._append_data(key, val, [('ts', 'f8')])


    def plot_xy_data(self, ax, spec, style, y_offs=0, label=None, **kwargs):
        dataset, field = spec.split('.')
        entries = self.data[dataset]
        if label is None:
            label = spec
        ax.plot(entries['ts'], entries[field] + y_offs, style,
                label=label, **kwargs)

    def plot_events(self, ax, spec, y_offs=0, label=None, **kwargs):
        if '.' in spec:
            dataset, field = spec.split('.')
            entries = self.data[dataset]
            # use non-zero values as events
            field_vals = entries[field]
            ts = entries['ts'][numpy.where(field_vals)]
        else:
            ts = self.data[spec]['ts']
        ax.plot(ts, numpy.zeros(ts.shape) + y_offs, 'x',
                label=(spec if label is None else label),
                color='b',
                markersize=10, **kwargs)



class LocalTimeFormatter(ticker.Formatter):
    def __call__(self, x, pos=0):
        result = time.strftime("%H:%M:%S", time.localtime(x))
        frac = (x % 1)
        if frac: result += ( "%.2f" % frac)[1:]
        return result

def main():
    ds = DataSet()
    for src in sys.argv[1:]:
        if src.endswith('.txt'):
            ds.parse_acc_data(src)
        elif src.endswith('.jsonlist'):
            ds.parse_jsonlog_data(src)
        elif src.endswith('.mkv') or src.endswith('.log'):
            pass
        else:
            print >>sys.stderr, 'Ignoring unknown file %r' % src
    print 'Parsing complete'

    fig = pyplot.figure()
    fig.subplots_adjust(top=0.95, bottom=0.01, left=0.1, right=0.99)
    ax = None
    subplot_max = 2
    if 'control' in ds.data:
        subplot_max += 1
    subplot_num = 0

    subplot_num += 1
    ax = fig.add_subplot(subplot_max, 1, subplot_num, sharex=ax)
    ds.plot_xy_data(ax, 'acc0.x', '.-')
    ds.plot_xy_data(ax, 'acc0.y', '.-')
    ds.plot_xy_data(ax, 'acc0.z', '.-')
    pyplot.legend()
    pyplot.grid(True)

    if 'control' in ds.data:
        subplot_num += 1
        ax = fig.add_subplot(subplot_max, 1, subplot_num, sharex=ax)
        y_offsets = list()
        y_names = list()
        y_offs = 0
        for spec in ('control.agitator_on',
                     'control.firing_ip3', '+!fire_cmd',
                     'control.firing_other', '+!fire_cmd',
                     'server.firing', '+!fire_sent',
                     '!firing_failed',
                     'server.ninpos_x', '+!turret_cmd',
                     'server.moving_x',
                     'server.ninpos_y', '+!turret_cmd',
                     'server.moving_y',
                     '!udp_loss',
                 ):
            merge = spec.startswith('+')
            if merge:
                spec = spec[1:]
            else:
                y_offs -= 2

            if spec.startswith('!'):
                spec = spec[1:]
                ds.plot_events(ax, spec, y_offs=(y_offs), label='')
            else:
                ds.plot_xy_data(ax, spec, '.-', y_offs=y_offs, label='')

            if merge:
                y_names[-1] += '\n+' + spec
            else:
                y_offsets.append(y_offs)
                y_names.append(spec)

        pyplot.yticks(y_offsets, y_names)
        pyplot.grid(True)

    subplot_num += 1
    ax = fig.add_subplot(subplot_max, 1, subplot_num, sharex=ax)
    ds.plot_xy_data(ax, 'acc1.x', '.-')
    ds.plot_xy_data(ax, 'acc1.y', '.-')
    ds.plot_xy_data(ax, 'acc1.z', '.-')
    pyplot.grid(True)
    pyplot.legend()

    assert subplot_max == subplot_num, (subplot_num, subplot_max)

    ax.xaxis.set_major_formatter(LocalTimeFormatter())
    fig.canvas.set_window_title(', '.join(ds.titles))

    pyplot.show()

if __name__ == '__main__':
    main()
