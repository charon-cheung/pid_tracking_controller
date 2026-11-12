#!/usr/bin/env python3
"""Offline PID gate evidence and nominal motion plots. See USAGE.md."""
import argparse
import csv
from datetime import datetime
import hashlib
import html
import io
import json
import math
from pathlib import Path
import re
import subprocess
from zoneinfo import ZoneInfo

FIELDS = re.compile(r'(\w+)=(\([^)]*\)|\S+)')
STAMP = re.compile(r'\[(\d{10}\.\d+)\]')
PHASES = {0: 'initial', 1: 'hold', 2: 'braking', 3: 'delay', 4: 'transition'}


def vector(value):
    numbers = [float(v) for v in value.strip('()').split(',')]
    if len(numbers) != 3 or not all(math.isfinite(v) for v in numbers):
        raise ValueError('expected three finite velocity components')
    return numbers


def moving(v):
    return math.hypot(*v[:2]) > 1e-6 or abs(v[2]) > 1e-6


def source_name(raw):
    prefix = raw.split('PID_', 1)[0]
    names = re.findall(r'\[([^]]+)\]', prefix)
    names = [name for name in names if name not in ('INFO', 'WARN', 'ERROR', 'DEBUG', 'FATAL')
             and not re.fullmatch(r'\d+(?:\.\d+)?', name)]
    if not names:
        raise ValueError('unrecognized log source')
    return '|'.join(names)


def parse_log(text, issues=None):
    """Pair exact cycles, then infer retained probes from OBSERVED gate transitions.

    Logs are throttled: inference is never certified as the actual retained value.
    A truncated log, pairing failure, epoch/revision or ROS-time reset loses evidence.
    """
    issues = issues if issues is not None else []
    pending, records = {}, []
    probe, inhibited, previous = None, None, None
    for line_no, raw in enumerate(text.splitlines(), 1):
        if 'PID_MOTION ' not in raw and 'PID_GATE ' not in raw:
            continue
        fields = dict(FIELDS.findall(raw))
        if 'revision' not in fields and (fields.get('model') == 'nominal' or
                fields.get('source') == 'odometry' or 'PID_GATE started ' in raw):
            pending.clear()
            probe = None
            inhibited = None
            previous = None
            continue
        stamp = STAMP.search(raw)
        try:
            required = {'revision', 'now', 'candidate'}
            if 'PID_MOTION ' in raw:
                required.update(('epoch', 'alignment', 'measured', 'effective_delay'))
            else:
                required.update(('action', 'candidate_state'))
            if not stamp or not required <= fields.keys():
                raise ValueError('incomplete record')
            source = source_name(raw)
            now = float(fields['now'])
            candidate = vector(fields['candidate'])
            if not math.isfinite(now):
                raise ValueError('invalid control time')
            if 'PID_MOTION ' in raw:
                vector(fields['measured'])
                if not math.isfinite(float(fields['effective_delay'])):
                    raise ValueError('invalid delay')
        except ValueError as error:
            issues.append(dict(line=line_no, reason=str(error)))
            pending.clear()
            probe = None
            inhibited = None
            previous = None
            continue
        key = (source, fields['revision'], fields['now'])
        if 'PID_MOTION ' in raw:
            pending[key] = (fields, line_no)
            if len(pending) > 8:
                pending.pop(next(iter(pending)))
            continue
        motion, motion_line = pending.pop(key, ({}, None))
        row = dict(fields, line=line_no, wall=float(stamp[1]), motion=motion,
                   motion_line=motion_line, probe=None, source=source)
        row['pair_valid'] = bool(motion and motion.get('candidate') == fields['candidate'])
        identity = (key[0], fields['revision'], motion.get('epoch'))
        if (not row['pair_valid'] or previous is None or identity != previous[0]
                or now <= previous[1]):
            probe = None
            inhibited = None
        previous = (identity, now)
        action = fields.get('action')
        if fields.get('candidate_state') != 'clear':
            if inhibited is False and moving(candidate):
                probe = dict(target=candidate, line=line_no, wall=row['wall'])
        elif inhibited is True and probe is None and moving(candidate):
            # Can't distinguish an absent probe from lost evidence: do not guess.
            pass
        row['probe'] = probe.copy() if probe else None
        if 'resume_policy' in fields:
            # New logs identify the current-cycle target; old logs still need inference.
            probe = None
            row['probe'] = None
            if (fields['resume_policy'] == 'current_candidate' and row['pair_valid']
                    and fields.get('resume_reason') != 'not_required' and moving(candidate)):
                row['probe'] = dict(target=candidate, line=line_no, wall=row['wall'],
                                    source='current_candidate')
        if action in ('released', 'pass', 'path_clear_bypass'):
            probe = None
            inhibited = False
        else:
            inhibited = True
        records.append(row)
    return records


def validate_config(config):
    for key in ('acceleration', 'deceleration', 'braking_deceleration'):
        v = config[key]
        if len(v) != 3 or not all(math.isfinite(x) and x > 0 for x in v):
            raise ValueError(f'{key}: need three positive finite numbers')
    if not math.isfinite(config['duration']) or config['duration'] <= 0:
        raise ValueError('duration must be positive and finite')


def replay(binary, measured, target, delay, config, resume, step):
    values = (measured + target + config['acceleration'] + config['deceleration'] +
              config['braking_deceleration'] + [delay, config['duration'], step, int(resume)])
    result = subprocess.run([str(binary)], input=' '.join(map(str, values)), text=True,
                            capture_output=True, timeout=30, check=True)
    return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(io.StringIO(result.stdout))]


def clock(stamp, zone):
    return datetime.fromtimestamp(stamp, zone).isoformat(timespec='milliseconds')


def select(records, args, zone):
    chosen = {r['line']: r for r in records if r['line'] in args.line}
    for line in args.line:
        if line not in chosen:
            raise ValueError(f'line {line} is not a PID_GATE record')
    for value in args.at:
        if 'T' in value or ' ' in value:
            date = datetime.fromisoformat(value)
            if date.tzinfo is None:
                date = date.replace(tzinfo=zone)
            target = date.timestamp()
        else:
            days = {datetime.fromtimestamp(r['wall'], zone).date() for r in records}
            if len(days) != 1:
                raise ValueError('multi-day log: use full ISO datetime')
            target = datetime.fromisoformat(f'{next(iter(days))}T{value}').replace(tzinfo=zone).timestamp()
        row = min(records, key=lambda r: abs(r['wall'] - target))
        if abs(row['wall'] - target) > args.tolerance:
            raise ValueError(f'no gate record within {args.tolerance}s of {value}')
        chosen[row['line']] = row
    if not args.at and not args.line:
        for row in records:
            if row.get('resume_state') == 'collision' and row.get('candidate_state') == 'clear':
                chosen[row['line']] = row
                if len(chosen) == args.limit:
                    break
    if not chosen:
        raise ValueError('no matching cases; use --list, --line or --at')
    return sorted(chosen.values(), key=lambda r: r['line'])


def plot_case(row, records, models, output, zone, window, footprint):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    xy, speed, timeline, angular = axes.flat
    colors = {'candidate': '#1875bb', 'resume': '#ca4335'}
    for name, points in models.items():
        color = colors[name]
        xy.plot([p['x'] for p in points], [p['y'] for p in points], color=color, label=name)
        xy.scatter(points[-1]['x'], points[-1]['y'], color=color, marker='x')
        speed.plot([p['time'] for p in points], [math.hypot(p['vx'], p['vy']) for p in points],
                   color=color, label=name)
        angular.plot([p['time'] for p in points], [p['omega'] for p in points], color=color, label=name)
        ends = [p for i, p in enumerate(points) if i == len(points)-1 or
                points[i+1]['phase'] != p['phase']]
        for p in ends:
            if p['phase']:
                speed.annotate(PHASES[int(p['phase'])], (p['time'], math.hypot(p['vx'], p['vy'])),
                               fontsize=7, color=color, rotation=25)
        if footprint:
            for index in sorted({round(i*(len(points)-1)/5) for i in range(6)}):
                p = points[index]; c, s = math.cos(p['yaw']), math.sin(p['yaw'])
                vertices = [(p['x']+c*x-s*y, p['y']+s*x+c*y) for x, y in footprint]
                xy.add_patch(Polygon(vertices, fill=False, edgecolor=color, alpha=.25))
    xy.scatter(0, 0, color='black', s=25)
    xy.set(xlabel='initial body x (m)', ylabel='initial body y (m)',
           title='Nominal motion to stop; X is NOT a collision point')
    xy.set_aspect('equal', adjustable='datalim')
    speed.set(xlabel='model time (s)', ylabel='body planar speed (m/s)', title='Response phases')
    angular.set(xlabel='model time (s)', ylabel='body omega (rad/s)', title='Angular response')
    nearby = [r for r in records if r['source'] == row['source']
              and abs(r['wall']-row['wall']) <= window]
    risk_colors = {'CLEARANCE_LOW': '#d8a700', 'COLLISION_AHEAD': '#db3333', 'CLEAR': '#309850'}
    labels = ['risk', 'candidate', 'resume', 'action']
    for r in nearby:
        delta = r['wall']-row['wall']
        for y, field in enumerate(('path_risk', 'candidate_state', 'resume_state', 'action')):
            value = r.get(field, '?')
            color = risk_colors.get(value, '#309850' if value in ('clear', 'pass', 'released') else
                                    '#db3333' if value == 'collision' else '#777777')
            timeline.scatter(delta, y, color=color, s=24)
        if len(nearby) <= 25:
            timeline.annotate(r.get('action', '?'), (delta, 3), rotation=70, fontsize=7)
    timeline.axvline(0, color='black', linestyle='--', alpha=.4)
    timeline.set(yticks=range(4), yticklabels=labels, xlabel='wall time relative to selected record (s)',
                 title='Logged events only; gaps are unobserved')
    timeline.set_ylim(-.5, 4.7)
    from matplotlib.lines import Line2D
    timeline.legend(handles=[Line2D([], [], marker='o', linestyle='', color=color, label=label)
                    for color, label in [('#309850', 'clear / pass'), ('#db3333', 'collision / red'),
                                         ('#d8a700', 'yellow'), ('#777777', 'other / unavailable')]],
                    fontsize=7, loc='upper left', ncol=2)
    for ax in axes.flat:
        ax.grid(alpha=.2)
    if models:
        xy.legend()
        speed.legend()
        angular.legend()
    fig.suptitle(f"{clock(row['wall'], zone)} | line {row['line']} | {row.get('path_risk')} / {row.get('action')}")
    fig.text(.5, .015, 'No historical costmap / exact footprint loaded. '
             'No collision re-evaluation. Explicit nominal assumptions (see report).', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .04, 1, .96))
    fig.savefig(output, dpi=130)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--source', help='select source in a mixed-node log (shown by --list)')
    parser.add_argument('--list', action='store_true', help='list gate records without plotting')
    parser.add_argument('--line', type=int, action='append', default=[])
    parser.add_argument('--at', action='append', default=[], help='local HH:MM:SS[.sss] or ISO datetime')
    parser.add_argument('--timezone', default='Asia/Shanghai')
    parser.add_argument('--tolerance', type=float, default=1.)
    parser.add_argument('--window', type=float, default=5.)
    parser.add_argument('--limit', type=int, default=6, help='default selected case count')
    parser.add_argument('--model-config', type=Path, help='explicit nominal assumptions JSON')
    parser.add_argument('--replay', type=Path, help='built pid_motion_replay executable')
    parser.add_argument('--step', type=float, default=.001)
    parser.add_argument('--footprint', type=Path, help='optional body [x,y] polygon JSON; illustration only')
    args = parser.parse_args()
    try:
        if (not all(math.isfinite(v) for v in (args.step, args.tolerance, args.window)) or
                not .0001 <= args.step <= .1 or args.tolerance < 0 or args.window <= 0 or args.limit < 1):
            raise ValueError('invalid step, tolerance, window or limit')
        zone = ZoneInfo(args.timezone)
        raw = args.log.read_bytes()  # immutable bytes used for both parsing and provenance
        text = raw.decode('utf-8', errors='replace')
        issues = []
        records = parse_log(text, issues)
        if not records:
            raise ValueError('no supported PID_GATE records')
        if args.list:
            for r in records:
                print(r['line'], r['source'], clock(r['wall'], zone), r.get('path_risk'), r.get('action'),
                      'candidate='+r.get('candidate_state', '?'), 'resume='+r.get('resume_state', '?'))
            if issues:
                print('Skipped malformed records:', json.dumps(issues))
            return
        sources = {r['source'] for r in records}
        if args.source:
            records = [r for r in records if r['source'] == args.source]
            if not records:
                raise ValueError('source not found; use --list')
        elif len(sources) > 1:
            raise ValueError('mixed log: choose --source from ' + ', '.join(sorted(sources)))
        if not args.output:
            raise ValueError('--output required (use a new directory)')
        selected = select(records, args, zone)
        config = json.loads(args.model_config.read_text()) if args.model_config else None
        if config:
            validate_config(config)
            if not args.replay or not args.replay.is_file():
                raise ValueError('--replay executable required with --model-config')
        footprint = json.loads(args.footprint.read_text()) if args.footprint else None
        if footprint is not None and (len(footprint) < 3 or any(len(p) != 2 or
                not all(math.isfinite(x) for x in p) for p in footprint)):
            raise ValueError('footprint must have at least three finite [x,y] vertices')
        args.output.mkdir(parents=True, exist_ok=False)
        (args.output/'source.log').write_bytes(raw)
        report = dict(source=str(args.log.resolve()), sha256=hashlib.sha256(raw).hexdigest(),
                      logged_gate_records=len(records), timezone=args.timezone, skipped_records=issues,
                      model_config=config, footprint_illustration=footprint,
                      replay_sha256=hashlib.sha256(args.replay.read_bytes()).hexdigest() if config else None,
                      tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), cases=[])
        sections = ['<h1>恢复运动离线分析</h1><p>名义运动重建，不是碰撞复现。'
                    '旧日志的恢复目标依赖节流日志推断；新日志按 resume_policy 识别本周期目标。'
                    '此工具未加载同期地图和精确车体轮廓，不进行碰撞复核。</p>']
        if issues:
            sections.append('<p>跳过损坏记录：'+html.escape(str(issues))+'</p>')
        for row in selected:
            case = dict(row, time=clock(row['wall'], zone), models={}, warnings=[])
            models = {}
            if not row['pair_valid']:
                case['warnings'].append('缺少同周期 PID_MOTION，禁止运动重建')
            elif row['motion'].get('alignment') != 'ready':
                case['warnings'].append('运动输入未对齐，禁止运动重建')
            elif row.get('sweep_requested') != '1' or row.get('candidate_reason') in (
                    'inputs_unavailable', 'gate_budget_exceeded'):
                case['warnings'].append('该周期没有有效候选运动检查，禁止运动重建')
            elif config:
                measured = vector(row['motion']['measured'])
                delay = float(row['motion']['effective_delay'])
                if not math.isfinite(delay) or delay < 0:
                    raise ValueError('invalid effective_delay')
                for name, target in [('candidate', vector(row['candidate'])),
                                     ('resume', row['probe']['target'] if row['probe'] else None)]:
                    if target is None or (name == 'resume' and row.get('resume_reason') == 'not_required'):
                        continue
                    model_config = config
                    if name == 'resume' and row.get('resume_policy') == 'current_candidate':
                        model_config = dict(config, duration=float(row['resume_hold']))
                        validate_config(model_config)
                    points = replay(args.replay.resolve(), measured, target, delay, model_config,
                                    name == 'resume', args.step)
                    models[name] = points
                    case['models'][name] = dict(target=target, duration=points[-1]['time'],
                        center_travel_m=points[-1]['distance'], end_xy=[points[-1]['x'], points[-1]['y']])
                    with (args.output/f"line_{row['line']}_{name}.csv").open('w') as stream:
                        writer = csv.DictWriter(stream, fieldnames=points[0].keys())
                        writer.writeheader()
                        writer.writerows(points)
            if not config:
                case['warnings'].append('未提供模型参数，仅展示线上日志状态')
            if not row['probe']:
                case['warnings'].append('恢复目标来源无法追溯，不猜测恢复曲线')
            case['warnings'].append('phase / hit_t 属于 candidate，不是 resume 的碰撞时刻')
            name = f"line_{row['line']}.png"
            plot_case(row, records, models, args.output/name, zone, args.window, footprint)
            report['cases'].append(case)
            table = '<table border="1" cellpadding="8"><tr><th>比较项</th><th>当前候选</th><th>恢复目标</th></tr>'
            for label, key in [('目标速度 [vx,vy,omega]', 'target'),
                               ('完整模型时长 / s', 'duration'),
                               ('完整模型中心路程 / m', 'center_travel_m')]:
                cells = [case['models'].get(k, {}).get(key, '无有效重建数据')
                         for k in ('candidate', 'resume')]
                table += '<tr><td>'+label+'</td>'+''.join(
                    '<td>'+html.escape(str(v))+'</td>' for v in cells)+'</tr>'
            table += '</table>'
            origin = '未知'
            if row['probe']:
                basis = '本周期目标' if row['probe'].get('source') == 'current_candidate' else '推断'
                origin = f"日志行 {row['probe']['line']}（{basis}）"
            sections.append(
                f"<h2>{case['time']} · 行 {row['line']}</h2><p>恢复目标来源：{origin}</p>"+
                table+f"<img src='{name}' width='100%'>"+
                '<p>'+html.escape('；'.join(case['warnings']))+'</p>'+
                '<details><summary>完整日志字段和分析数据</summary><pre>'+
                html.escape(json.dumps(case, ensure_ascii=False, indent=2))+'</pre></details>')
        (args.output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>离线分析</title>'
            '<style>body{max-width:1200px;margin:30px auto;font-family:sans-serif}pre{white-space:pre-wrap}</style>'
            + '\n'.join(sections))
        print(args.output/'index.html')
    except (ValueError, OSError, KeyError, TypeError, subprocess.SubprocessError) as error:
        parser.exit(2, f'error: {error}\n')


if __name__ == '__main__':
    main()
