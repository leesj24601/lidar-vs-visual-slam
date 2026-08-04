import json
import math
from dataclasses import dataclass


MOVE_API_ID = 1008
STOP_MOVE_API_ID = 1003


@dataclass(frozen=True)
class SportCommand:
    api_id: int
    parameter: str


class SportCommandPolicy:
    def __init__(
        self,
        min_linear_x,
        max_linear_x,
        max_linear_y,
        max_angular_z,
        cmd_vel_timeout,
        enabled,
    ):
        limits = (
            min_linear_x,
            max_linear_x,
            max_linear_y,
            max_angular_z,
        )
        if not all(math.isfinite(value) for value in limits):
            raise ValueError('Velocity limits must be finite')
        if min_linear_x > 0.0 or max_linear_x < 0.0:
            raise ValueError('Linear x limits must contain zero')
        if max_linear_y < 0.0 or max_angular_z < 0.0:
            raise ValueError(
                'Symmetric velocity limits must be non-negative'
            )
        if not math.isfinite(cmd_vel_timeout) or cmd_vel_timeout <= 0.0:
            raise ValueError('cmd_vel_timeout must be finite and positive')

        self._min_linear_x = min_linear_x
        self._max_linear_x = max_linear_x
        self._max_linear_y = max_linear_y
        self._max_angular_z = max_angular_z
        self._cmd_vel_timeout = cmd_vel_timeout
        self.enabled = enabled
        self._active = False
        self._has_commanded_motion = False
        self._last_command_time = None

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def _stop_command():
        return SportCommand(api_id=STOP_MOVE_API_ID, parameter='')

    def accept_velocity(self, vx, vy, vyaw, now_sec):
        if not self.enabled:
            return None

        values = (vx, vy, vyaw, now_sec)
        if not all(math.isfinite(value) for value in values):
            self._active = False
            return self._stop_command()

        payload = {
            'x': self._clamp(
                vx,
                self._min_linear_x,
                self._max_linear_x,
            ),
            'y': self._clamp(
                vy,
                -self._max_linear_y,
                self._max_linear_y,
            ),
            'z': self._clamp(
                vyaw,
                -self._max_angular_z,
                self._max_angular_z,
            ),
        }
        if all(value == 0.0 for value in payload.values()):
            self._active = False
            return self._stop_command()

        self._active = True
        self._has_commanded_motion = True
        self._last_command_time = now_sec
        return SportCommand(
            api_id=MOVE_API_ID,
            parameter=json.dumps(payload, separators=(',', ':')),
        )

    def check_timeout(self, now_sec):
        if not self.enabled or not self._active:
            return None
        if not math.isfinite(now_sec):
            self._active = False
            return self._stop_command()
        if now_sec - self._last_command_time < self._cmd_vel_timeout:
            return None

        self._active = False
        return self._stop_command()

    def shutdown_command(self):
        if not self.enabled or not self._has_commanded_motion:
            return None
        self._active = False
        return self._stop_command()
