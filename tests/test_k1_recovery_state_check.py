"""Do not confuse motor-bridge zero placeholders with healthy feedback."""

from types import SimpleNamespace
import unittest

from booster_deploy.utils.recovery_state_check import summarize_samples


def sample():
    def motor(temperature):
        return SimpleNamespace(q=0., dq=0., tau_est=0., mode=0, lost=0,
                               reserve=[0, 0], temperature=temperature)
    return SimpleNamespace(motor_state_serial=[motor(0) for _ in range(22)],
                           motor_state_parallel=[motor(38) for _ in range(22)])


class RecoveryStateCheckTests(unittest.TestCase):
    def test_no_sample_is_unknown(self):
        self.assertEqual(summarize_samples([])['sample_count'], 0)

    def test_serial_zero_temperature_alone_is_not_motor_fault(self):
        self.assertEqual(summarize_samples([sample(), sample()])['suspect_arm_indices'], [])

    def test_reproduces_three_failed_feedback_slots(self):
        samples = [sample(), sample()]
        for s in samples:
            for i in (2, 5, 9):
                s.motor_state_parallel[i].temperature = 0
        self.assertEqual(summarize_samples(samples)['suspect_arm_indices'], [2, 5, 9])

    def test_nonzero_feedback_anywhere_in_window_is_preserved(self):
        samples = [sample(), sample()]
        for s in samples:
            s.motor_state_parallel[2].temperature = 0
        samples[0].motor_state_serial[2].q = 0.2
        samples[0].motor_state_parallel[2].q = 0.2
        report = summarize_samples(samples)
        self.assertEqual(report['suspect_arm_indices'], [])
        self.assertEqual(report['groups']['motor_state_serial'][2]['q_range'], [0., 0.2])


if __name__ == '__main__':
    unittest.main()
