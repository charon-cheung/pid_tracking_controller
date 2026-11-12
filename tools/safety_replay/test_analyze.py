import math
import os
from pathlib import Path
import subprocess
import unittest

from analyze import parse_log, replay, validate_config


def cycle(now, candidate_state='clear', action='pass', epoch=3, revision=1):
    common = f'revision={revision} now={now} candidate=(0.4,-0.5,0)'
    return (f'[node] [1794471445.{now:03d}] PID_MOTION {common} epoch={epoch} alignment=ready measured=(0,0,0) effective_delay=0.5\n'
            f'[node] [1794471445.{now:03d}] PID_GATE {common} candidate_state={candidate_state} '
            f'action={action} resume_state=collision\n')


class EvidenceTests(unittest.TestCase):
    def test_traces_first_rejection_not_latest_candidate(self):
        rows = parse_log(cycle(1)+cycle(2, 'collision', 'braking')+cycle(3, action='hold'))
        self.assertEqual(rows[-1]['probe']['line'], 4)
        self.assertEqual(rows[-1]['probe']['target'], [.4, -.5, 0])

    def test_truncated_log_does_not_invent_probe(self):
        self.assertIsNone(parse_log(cycle(2, 'collision', 'hold'))[-1]['probe'])

    def test_reset_and_release_invalidate_probe(self):
        prefix = cycle(1)+cycle(2, 'collision', 'braking')
        for suffix in (cycle(3, action='hold', epoch=4), cycle(3, action='hold', revision=2),
                       cycle(1, action='hold'), cycle(3, action='released')+cycle(4, action='hold')):
            with self.subTest(suffix=suffix):
                self.assertIsNone(parse_log(prefix+suffix)[-1]['probe'])

    def test_missing_motion_is_not_paired_with_another_cycle(self):
        rows = parse_log(cycle(1)+cycle(2, 'collision', 'braking')+
                         cycle(3, action='hold').splitlines()[1])
        self.assertFalse(rows[-1]['pair_valid'])
        self.assertIsNone(rows[-1]['probe'])

    def test_truncated_tail_preserves_good_cases(self):
        issues = []
        good = cycle(1)+cycle(2, 'collision', 'braking')+cycle(3, action='hold')
        tail = cycle(4).splitlines()[1].replace('candidate=(0.4,-0.5,0)', 'candidate=(0.4,')
        rows = parse_log(good+tail, issues)
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[-1]['probe']['line'], 4)
        self.assertEqual(issues[0]['line'], 7)

    def test_standard_logger_identity_prevents_cross_node_pairing(self):
        motion, gate = cycle(1).splitlines()
        motion = motion.replace('[node] ', '[INFO] ').replace(' PID_', ' [one]: PID_')
        gate = gate.replace('[node] ', '[INFO] ').replace(' PID_', ' [two]: PID_')
        row = parse_log(motion+'\n'+gate)[0]
        self.assertFalse(row['pair_valid'])
        self.assertEqual(row['source'], 'two')

    def test_startup_message_resets_evidence_without_corruption_warning(self):
        issues = []
        startup = '[node] [1794471446.000] PID_GATE started stage=2-C\n'
        rows = parse_log(cycle(1)+cycle(2, 'collision', 'braking')+startup+
                         cycle(3, action='hold'), issues)
        self.assertEqual(issues, [])
        self.assertIsNone(rows[-1]['probe'])

    def test_current_policy_uses_changed_candidate_even_in_truncated_logs(self):
        current = cycle(3, action='hold').replace('(0.4,-0.5,0)', '(0,0.05,0)')
        current = current.replace('resume_state=collision',
                                  'resume_state=clear resume_policy=current_candidate resume_hold=0.2')
        for prefix in ('', cycle(1)+cycle(2, 'collision', 'braking')):
            row = parse_log(prefix+current)[-1]
            self.assertEqual(row['probe']['target'], [0, .05, 0])
            self.assertEqual(row['probe']['line'], row['line'])
            self.assertEqual(row['probe']['source'], 'current_candidate')
        self.assertIsNone(parse_log(current.splitlines()[1])[-1]['probe'])

    def test_unknown_policy_does_not_inherit_old_probe(self):
        current = cycle(3, action='hold').replace('resume_state=collision',
                                                'resume_state=clear resume_policy=unknown')
        row = parse_log(cycle(1)+cycle(2, 'collision', 'braking')+current)[-1]
        self.assertIsNone(row['probe'])

    def test_invalid_rates_rejected(self):
        config = dict(acceleration=[1, 1, 1], deceleration=[1, 1, 1],
                      braking_deceleration=[1, 1, 0], duration=.1)
        with self.assertRaises(ValueError):
            validate_config(config)


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = Path(os.environ['PID_MOTION_REPLAY'])
        cls.config = dict(acceleration=[1, 1, 1], deceleration=[1, 1, 1],
                          braking_deceleration=[1, 1, 1], duration=.1)

    def test_straight_acceleration_and_brake_distance(self):
        points = replay(self.binary, [0, 0, 0], [1, 0, 0], .5, self.config, False, .001)
        self.assertAlmostEqual(points[-1]['x'], .01, places=8)
        self.assertAlmostEqual(points[-1]['time'], .7, places=8)
        self.assertEqual(points[-1]['vx'], 0)

    def test_resume_reaches_target_then_brakes(self):
        points = replay(self.binary, [0, 0, 0], [1, 0, 0], .5, self.config, True, .001)
        self.assertAlmostEqual(max(p['vx'] for p in points), 1)
        self.assertAlmostEqual(points[-1]['x'], 1.1, places=8)

    def test_rotating_body_velocity_and_refinement(self):
        args = (self.binary, [.2, -.1, .3], [-.3, .4, -.2], .5, self.config, True)
        a, b = replay(*args, .001), replay(*args, .0005)
        self.assertLess(math.hypot(a[-1]['x']-b[-1]['x'], a[-1]['y']-b[-1]['y']), 1e-6)
        self.assertAlmostEqual(a[-1]['omega'], 0)

    def test_invalid_model_fails_without_a_curve(self):
        with self.assertRaises(subprocess.CalledProcessError):
            replay(self.binary, [0, 0, 0], [1, 0, 0], -1, self.config, False, .001)


if __name__ == '__main__':
    unittest.main()
