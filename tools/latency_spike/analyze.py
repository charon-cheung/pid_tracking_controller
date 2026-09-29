#!/usr/bin/env python3
"""Disposable, offline cmd/odom lag analysis. Source ROS 2 Jazzy before running.

Example:
  python3 analyze.py BAG --output /tmp/latency-result
  python3 analyze.py BAG --output /tmp/reversal --window reversal:10:16

Window times are seconds since the first selected message's bag receive time.
Positive lag means odom follows cmd. This measures waveform lag, not pure
transport delay or independently observed physical motion. No ROS node starts.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

AXES = ('vx', 'vy', 'omega')


def percentiles(values):
    return dict(zip(('p50', 'p95', 'max'), map(float, np.percentile(values, [50, 95, 100]))))


def load_bag(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    expected = (('geometry_msgs/msg/TwistStamped', 'geometry_msgs/msg/Twist'),
                ('nav_msgs/msg/Odometry',))
    for topic, kinds in zip(topics, expected):
        if types.get(topic) not in kinds:
            raise ValueError(f'{topic}: expected one of {kinds}, found {types.get(topic)}')
    classes = {t: get_message(types[t]) for t in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    records = {t: [] for t in topics}
    frames = {t: set() for t in topics}
    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        if types[topic] == 'geometry_msgs/msg/Twist':
            stamp, twist, frame = None, msg, ''
        else:
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            twist = msg.twist if topic == topics[0] else msg.twist.twist
            frame = msg.header.frame_id if topic == topics[0] else msg.child_frame_id
        frames[topic].add(frame)
        records[topic].append((receive_ns, stamp, twist.linear.x, twist.linear.y, twist.angular.z))
    if any(len(records[t]) < 10 for t in topics):
        raise ValueError('Need at least 10 messages on each topic.')
    return records, frames


def prepare(rows, origin, time_base):
    # Subtract integer nanoseconds before float conversion to retain precision.
    receive = np.array([(r[0] - origin) * 1e-9 for r in rows])
    header = np.array([(r[1] - origin) * 1e-9 if r[1] is not None else np.nan for r in rows])
    times = header if time_base == 'header' else receive
    velocity = np.array([r[2:] for r in rows])
    if not np.isfinite(velocity).all():
        raise ValueError('Non-finite velocity samples; inspect the source bag.')
    valid_stamps = all(r[1] is not None and r[1] > 0 for r in rows)
    if time_base == 'header' and not valid_stamps:
        raise ValueError('Header time mode requires valid nonzero stamps; use receive mode for unstamped Twist.')
    if np.any(np.diff(times) < 0):
        raise ValueError('Selected clock moved backwards / messages reordered; split or inspect bag.')
    # Last received sample wins when selected timestamps repeat. Report repeats.
    keep = np.r_[np.diff(times) != 0, True]
    stats = {'messages': len(rows), 'duplicate_selected_timestamps': int((~keep).sum()),
             'missing_header_stamps': sum(r[1] is None for r in rows),
             'zero_header_stamps': sum(r[1] == 0 for r in rows),
             'header_nonincreasing_intervals': int((np.diff(header) <= 0).sum())}
    if valid_stamps:
        age = receive - header
        stats['receive_minus_header_s_not_transport_latency'] = percentiles(age)
        stats['negative_header_age_samples'] = int((age < -0.001).sum())
    times, velocity = times[keep], velocity[keep]
    if len(times) < 10:
        raise ValueError('Too few distinct timestamps.')
    stats['selected_message_interval_s'] = percentiles(np.diff(times))
    return times, velocity, stats


def compare_window(ct, cv, ot, ov, start, end, max_lag, min_excitation):
    ci = (ct >= start) & (ct <= end)
    oi = (ot >= start) & (ot <= end)
    if ci.sum() < 10 or oi.sum() < 10:
        return {'status': 'insufficient_samples'}
    cperiod, operiod = np.median(np.diff(ct[ci])), np.median(np.diff(ot[oi]))
    dt = max(cperiod, operiod)
    if max(np.max(np.diff(ct[ci])) / cperiod, np.max(np.diff(ot[oi])) / operiod) > 3:
        return {'status': 'sampling_gap_over_3_periods'}
    steps = int(np.floor(max_lag / dt))
    if steps < 1:
        return {'status': 'lag_range_below_sample_period', 'grid_s': float(dt)}
    lags = np.arange(-steps, steps + 1) * dt
    # Identical overlap for every candidate; never extrapolate either stream.
    lo = max(start, ct[0], ot[0]) + max_lag
    hi = min(end, ct[-1], ot[-1]) - max_lag
    grid = np.arange(lo, hi, dt)
    if len(grid) < 10:
        return {'status': 'window_too_short_for_lag_range', 'grid_s': float(dt)}
    # Tolerate roundoff when grid points coincide with recorded sample times.
    command = cv[np.searchsorted(ct, grid + dt * 1e-9, side='right') - 1]
    feedback = np.interp(grid, ot, ov)
    result = {'grid_s': float(dt), 'samples': len(grid),
              'rmse_before': float(np.sqrt(np.mean((feedback - command)**2))),
              'mean_error_before': float(np.mean(feedback - command))}
    if min(np.ptp(command), np.ptp(feedback)) < min_excitation:
        return dict(result, status='insufficient_excitation')
    candidates = [np.interp(grid + lag, ot, ov) for lag in lags]
    scores = np.array([np.corrcoef(command, v)[0, 1] if np.std(v) > 1e-9 else -1
                       for v in candidates])
    best = int(np.argmax(scores))
    result.update(candidate_lag_s=float(lags[best]), correlation=float(scores[best]))
    tied = lags[scores >= scores[best] - 1e-4]
    if scores[best] < 0.6:
        return dict(result, status='weak_correlation')
    if np.ptp(tied) > 2 * dt:
        return dict(result, status='ambiguous_lag')
    if best in (0, len(lags) - 1):
        return dict(result, status='lag_at_search_boundary')
    error = candidates[best] - command
    return dict(result, status='estimated', rmse_aligned=float(np.sqrt(np.mean(error**2))),
                peak_error_aligned=float(np.max(np.abs(error))), mean_error_aligned=float(np.mean(error)))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', required=True, type=Path, help='New directory; never overwritten')
    parser.add_argument('--cmd-topic', default='/cmd_vel_nav')
    parser.add_argument('--odom-topic', default='/dev/odometry/raw/dist')
    parser.add_argument('--time-base', choices=('receive', 'header'), default='receive',
                        help='Header requires verified common clock/stamp semantics; default: bag receive')
    parser.add_argument('--window', action='append', default=[], metavar='LABEL:START:END',
                        help='Repeat for acceleration/deceleration/reversal/steady intervals')
    parser.add_argument('--window-seconds', type=float, default=5.0, help='Automatic windows if none supplied')
    parser.add_argument('--max-lag', type=float, default=0.5, help='Search both +/- this many seconds')
    parser.add_argument('--min-excitation', type=float, default=0.03, help='Minimum velocity range, m/s or rad/s')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new directory.')
    if not all(np.isfinite(v) and v > 0 for v in (args.window_seconds, args.max_lag, args.min_excitation)):
        parser.error('Window duration, max lag and minimum excitation must be positive finite values.')
    topics = (args.cmd_topic, args.odom_topic)
    records, frames = load_bag(args.bag, topics)
    known_frames = [frames[t] - {''} for t in topics]
    if all(known_frames) and (known_frames[0] != known_frames[1] or len(known_frames[0]) != 1):
        raise ValueError(f'Command and odom twist frames differ/change: {frames}; transform before comparison.')
    origin = min(rows[0][0] for rows in records.values())
    ct, cv, cs = prepare(records[topics[0]], origin, args.time_base)
    ot, ov, os = prepare(records[topics[1]], origin, args.time_base)
    start, end = max(ct[0], ot[0]), min(ct[-1], ot[-1])
    if end <= start:
        raise ValueError('No overlapping time range; check clocks and stamp semantics.')
    windows = []
    for spec in args.window:
        label, left, right = spec.rsplit(':', 2)
        left, right = float(left), float(right)
        if not np.isfinite([left, right]).all() or not start <= left < right <= end:
            raise ValueError(f'Window outside overlapping range [{start:.3f}, {end:.3f}]: {spec}')
        windows.append((label, left, right))
    if not windows:
        windows = [(f'window_{i}', float(t), min(float(t + args.window_seconds), end))
                   for i, t in enumerate(np.arange(start, end, args.window_seconds))]
    results = []
    for label, left, right in windows:
        for axis, name in enumerate(AXES):
            result = compare_window(ct, cv[:, axis], ot, ov[:, axis], left, right,
                                    args.max_lag, args.min_excitation)
            results.append(dict(window=label, axis=name, start_s=left, end_s=right, **result))
    summary = {'bag': str(args.bag), 'time_base': args.time_base, 'origin_bag_receive_ns': origin,
               'topics': {topics[0]: cs, topics[1]: os}, 'frames': {k: sorted(v) for k, v in frames.items()},
               'windows': results, 'notes': [
                   'With --sim recording, bag receive time uses /clock; lags are simulation seconds, not wall seconds.',
                   'Positive lag means odom follows cmd; this is waveform lag, not a measured causal stage delay.',
                   'Receive time is rosbag recorder time, not driver or controller callback time.',
                   'Header age includes stamp semantics/clock offset; pose stamps do not date velocity samples.',
                   'Grid follows slower median sample period; interpolation does not provide millisecond accuracy.',
                   'Window lag percentiles describe different windows, not event-level latency jitter.',
                   'Confirm twist coordinate frames, clock synchronization and recorder overhead before interpreting results.']}
    if any('' in frames[t] for t in topics):
        summary['notes'].append('Missing frame IDs: coordinate agreement cannot be verified from this bag.')
    summary['accepted_window_lag_s'] = {}
    for name in AXES:
        delays = [r['candidate_lag_s'] for r in results if r['axis'] == name and r['status'] == 'estimated']
        summary['accepted_window_lag_s'][name] = dict(count=len(delays), **(percentiles(delays) if delays else {}))
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    with (args.output / 'windows.csv').open('w') as f:
        fields = list(dict.fromkeys(key for result in results for key in result))
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)
    with (args.output / 'samples.csv').open('w') as f:
        writer = csv.writer(f)
        writer.writerow(('topic', 'receive_ns', 'header_ns', *AXES))
        for topic in topics:
            writer.writerows((topic, *row) for row in records[topic])
    print(json.dumps(summary['accepted_window_lag_s'], indent=2))
    print(f'Results: {args.output}; inspect windows.csv statuses and summary.json notes before use.')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError) as error:
        raise SystemExit(f'Analysis failed: {error}') from error
