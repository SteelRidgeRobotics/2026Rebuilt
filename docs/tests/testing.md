# Testing

File: [`tests/pyfrc_test.py`](../../tests/pyfrc_test.py)

## Current setup

The project imports **`pyfrc.tests`** wholesale (`from pyfrc.tests import *`). That suite exercises basic RobotPy scaffolding—useful smoke tests but not a substitute for **hardware integration** tests.

## Suggested team practices

- Run pytest / RobotPy test entry points **before PRs** once mentors wire them in CI.
- Add **focused tests** for pure helpers (for example aiming interpolation) without needing the RoboRIO.
- Use **simulation** and **log replay** for integration issues that unit tests cannot catch.

## Extending

When students add algorithms (pure functions), colocate `test_foo.py` under `tests/` and keep IO-heavy code thin so it remains mockable.
