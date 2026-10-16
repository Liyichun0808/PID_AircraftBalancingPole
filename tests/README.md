# Offline controller check

Run from the repository root:

```text
python tests/offline_controller_test.py
```

The script is a deterministic reference-model check for the policy in
`MyCode/balance_controller.c`. It exercises the startup arm gate, the two-motor
PID mixer, explicit disarm, angle-limit fault latch, and three-sample sensor
failure latch. The model enables output only for the test; the STM32 firmware
configuration remains `output_enabled = 0` until the real motor polarity,
sensor axis, and gains have been checked.
