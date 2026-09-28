"""Read-only summaries of K1 feedback; these are not motor health proofs."""


def summarize_samples(samples):
    if not samples:
        return {'sample_count': 0, 'suspect_arm_indices': [],
                'diagnosis': 'No low_state received; state is unknown.'}
    report = {'sample_count': len(samples), 'groups': {},
              'suspect_arm_indices': [],
              'diagnosis': 'Zero fault fields do not prove healthy motors. '
                           'Check firmware faults independently.'}
    for group in ('motor_state_serial', 'motor_state_parallel'):
        latest = getattr(samples[-1], group)
        result = []
        for i, motor in enumerate(latest):
            row = {'index': i, 'temperature': int(motor.temperature),
                   'mode': int(motor.mode), 'lost': int(motor.lost),
                   'reserve': [int(v) for v in motor.reserve]}
            complete = all(len(getattr(s, group)) == len(latest) for s in samples)
            row['consistent_joint_count'] = complete
            for field in ('q', 'dq', 'tau_est'):
                values = [float(getattr(getattr(s, group)[i], field))
                          for s in samples if len(getattr(s, group)) > i]
                row[field] = float(getattr(motor, field))
                row[field + '_range'] = [min(values), max(values)]
            row['all_zero_feedback'] = complete and all(
                row[field + '_range'] == [0., 0.]
                for field in ('q', 'dq', 'tau_est'))
            result.append(row)
        report['groups'][group] = result

    # K1 arms have identical indices in serial and parallel feedback. Never
    # apply this comparison to the coupled ankle motors or another robot.
    serial = report['groups']['motor_state_serial']
    parallel = report['groups']['motor_state_parallel']
    if len(serial) == len(parallel) == 22:
        report['suspect_arm_indices'] = [
            i for i in range(2, 10)
            if serial[i]['all_zero_feedback']
            and parallel[i]['all_zero_feedback']
            and all(s.motor_state_parallel[i].temperature == 0 for s in samples)
        ]
    else:
        report['diagnosis'] = 'Expected 22 serial and parallel K1 joints; cannot assess arms.'
    return report
