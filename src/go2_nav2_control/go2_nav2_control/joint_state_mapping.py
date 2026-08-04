JOINT_NAMES = (
    'FL_hip_joint',
    'FL_thigh_joint',
    'FL_calf_joint',
    'FR_hip_joint',
    'FR_thigh_joint',
    'FR_calf_joint',
    'RL_hip_joint',
    'RL_thigh_joint',
    'RL_calf_joint',
    'RR_hip_joint',
    'RR_thigh_joint',
    'RR_calf_joint',
)

MOTOR_INDICES = (3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8)


def extract_joint_values(motor_states):
    if len(motor_states) < 12:
        raise ValueError(
            'LowState motor_state must contain at least 12 motors'
        )

    selected = [motor_states[index] for index in MOTOR_INDICES]
    return (
        [float(motor.q) for motor in selected],
        [float(motor.dq) for motor in selected],
        [float(motor.tau_est) for motor in selected],
    )
