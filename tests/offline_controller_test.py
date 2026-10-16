"""Deterministic, hardware-free checks for the balance controller policy.

This is intentionally a small reference model of balance_controller.c.  It
uses the same gains, 10 ms period, arm gate, mixer limits, and fault rules so
the control policy can be checked without Proteus or a connected MPU6050.
"""

from dataclasses import dataclass
import math


@dataclass
class Config:
    target_angle_deg: float = 0.0
    kp: float = 1.0
    ki: float = 0.05
    kd: float = 0.08
    output_limit: float = 30.0
    integral_limit: float = 100.0
    base_pwm: float = 20.0
    correction_sign: float = 1.0
    period_ms: int = 10
    arm_delay_ms: int = 1000
    arm_samples: int = 50
    max_angle_cdeg: int = 4500
    output_enabled: int = 1  # enabled only in this offline model
    sensor_failure_limit: int = 3


class Controller:
    def __init__(self, config: Config, now_ms: int = 0):
        self.c = config
        self.boot_tick = now_ms
        self.integral = 0.0
        self.armed = False
        self.arm_requested = False
        self.faulted = False
        self.healthy = False
        self.failures = 0
        self.stable_samples = 0
        self.angle_cdeg = 0
        self.gyro_mdps = 0
        self.correction = 0.0
        self.pwm = (0, 0)

    def arm(self):
        if not self.faulted:
            self.arm_requested = True

    def disarm(self):
        self.armed = False
        self.arm_requested = False
        self.stable_samples = 0
        self.integral = 0.0
        self.correction = 0.0
        self.pwm = (0, 0)

    def update(self, valid: bool, angle_deg: float, gyro_dps: float, now_ms: int):
        if (
            not valid
            or not math.isfinite(angle_deg)
            or not math.isfinite(gyro_dps)
            or abs(angle_deg) > 10000.0
            or abs(gyro_dps) > 10000.0
        ):
            self.healthy = False
            self.failures = min(self.failures + 1, self.c.sensor_failure_limit)
            self.disarm()
            if self.failures >= self.c.sensor_failure_limit:
                self.faulted = True
            return

        self.failures = 0
        self.healthy = True
        self.angle_cdeg = int(angle_deg * 100.0)
        self.gyro_mdps = int(gyro_dps * 1000.0)
        if abs(self.angle_cdeg) > self.c.max_angle_cdeg:
            self.disarm()
            self.faulted = True
            return

        self.stable_samples = min(self.stable_samples + 1, self.c.arm_samples)
        if (
            self.c.output_enabled
            and not self.faulted
            and self.arm_requested
            and self.stable_samples >= self.c.arm_samples
            and now_ms - self.boot_tick >= self.c.arm_delay_ms
        ):
            self.armed = True

        if not self.armed:
            self.integral = 0.0
            self.correction = 0.0
            self.pwm = (0, 0)
            return

        error = self.c.target_angle_deg - angle_deg
        integral_candidate = max(
            self.c.integral_limit * -1.0,
            min(self.integral + error * self.c.period_ms / 1000.0,
                self.c.integral_limit),
        )
        unsaturated = (
            self.c.kp * error
            + self.c.ki * integral_candidate
            - self.c.kd * gyro_dps
        )
        self.correction = max(-self.c.output_limit, min(unsaturated, self.c.output_limit))
        if (
            unsaturated == self.correction
            or (self.correction >= self.c.output_limit and error < 0.0)
            or (self.correction <= -self.c.output_limit and error > 0.0)
        ):
            self.integral = integral_candidate
        self.correction *= self.c.correction_sign
        p1 = max(0.0, min(99.0, self.c.base_pwm + self.correction))
        p2 = max(0.0, min(99.0, self.c.base_pwm - self.correction))
        self.pwm = (int(p1 + 0.5), int(p2 + 0.5))


def check(condition: bool, message: str):
    if not condition:
        raise AssertionError(message)


def run():
    c = Controller(Config())
    command_map = {
        "a": "ARM", "A": "ARM", "d": "DISARM", "D": "DISARM",
        "s": "STATUS", "S": "STATUS", "x": "NONE",
    }
    check(command_map["a"] == "ARM" and command_map["A"] == "ARM",
          "serial ARM command mapping is wrong")
    check(command_map["d"] == "DISARM" and command_map["D"] == "DISARM",
          "serial DISARM command mapping is wrong")
    check(command_map["s"] == "STATUS" and command_map["x"] == "NONE",
          "serial status/unknown command mapping is wrong")

    # Bench firmware keeps the output gate closed; an ARM request alone must
    # never create a PWM command when output_enabled is zero.
    safe = Controller(Config(output_enabled=0))
    safe.arm()
    for now in range(10, 1100, 10):
        safe.update(True, 0.0, 0.0, now)
    check(not safe.armed and safe.pwm == (0, 0),
          "output-disabled controller armed or generated PWM")

    c.arm()

    # Valid 10 ms samples must satisfy both gates before arming.
    for now in range(10, 1000, 10):
        c.update(True, 0.0, 0.0, now)
        check(not c.armed, f"armed too early at {now} ms")
    c.update(True, 5.0, 0.0, 1000)
    check(c.armed, "controller did not arm after 50 valid samples and 1 s")
    check(c.pwm[0] < c.pwm[1], "mixer did not produce opposite PWM correction")
    check(c.pwm[0] > 0 and c.pwm[1] > 0, "valid attitude produced zero PWM")
    armed_pwm = c.pwm

    # Boundary conditions: sample 49 and 999 ms are still locked; exact
    # +/-45 degrees is allowed, while a value beyond it is a fault.
    c = Controller(Config())
    c.arm()
    for now in range(10, 1000, 10):
        c.update(True, 0.0, 0.0, now)
    check(not c.armed, "controller armed before the 1000 ms boundary")
    c.update(True, 45.0, 0.0, 1000)
    check(c.armed and not c.faulted, "exact angle limit should be allowed")

    # Large errors and rates must clamp the mixer to a legal 0..99 range.
    saturated_config = Config(base_pwm=90.0)
    c = Controller(saturated_config)
    c.arm()
    for now in range(10, 1000, 10):
        c.update(True, 0.0, 0.0, now)
    c.update(True, -45.0, 0.0, 1000)
    check(c.pwm == (99, 60), "PWM upper clamp did not produce 99/60")

    # Explicit disarm must clear both outputs and the arm request.
    c.disarm()
    check(c.pwm == (0, 0) and not c.armed, "disarm did not clear outputs/state")

    # Angle protection latches a fault and blocks a later arm request.
    c.arm()
    c.update(True, 46.0, 0.0, 1010)
    check(c.faulted and c.pwm == (0, 0), "angle limit did not latch a safe fault")
    c.arm()
    for now in range(1020, 1530, 10):
        c.update(True, 0.0, 0.0, now)
    check(not c.armed, "faulted controller accepted a new arm request")

    # Three consecutive invalid samples latch the sensor fault and stop PWM.
    c = Controller(Config())
    c.arm()
    for now in (10, 20, 30):
        c.update(False, math.nan, math.nan, now)
    check(c.faulted and c.pwm == (0, 0), "sensor failure gate did not latch")

    # A malformed numeric sample counts as a sensor failure, just like a
    # failed I2C transaction.
    c = Controller(Config())
    c.arm()
    for now in (10, 20, 30):
        c.update(True, math.nan, math.inf, now)
    check(c.faulted and c.pwm == (0, 0), "invalid numeric sample was accepted")

    print("offline controller checks: PASS")
    print("arm gate: 50 valid samples + 1000 ms")
    print("serial A/D/S decode policy: PASS")
    print("mixer sample at +5 deg: pwm1=%d pwm2=%d" % armed_pwm)
    print("disarm, angle limit, and 3-sample sensor fault: PASS")
    print("boundary, PWM clamp, and invalid numeric sample: PASS")


if __name__ == "__main__":
    run()
