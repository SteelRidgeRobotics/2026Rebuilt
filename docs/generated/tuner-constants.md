# Generated Tuner X constants

Directories: [`generated/`](../../generated/), [`generated/larry/`](../../generated/larry/)

## What this is

The swerve drivetrain (Phoenix 6 **Tuner X Swerve Generator**) emits Python modules with:

- Module **offsets** and gear ratios
- **SwerveDrivetrain** factory methods (`create_drivetrain()`)
- Speed limits (`speed_at_12_volts`, …)

This repository keeps **two** generations:

- **`generated/tuner_constants.py`** — primary competition robot (“Dwayne”).
- **`generated/larry/tuner_constants.py`** — practice bot **Larry**.

`subsystems/swerve/__init__.py` and `robot_container.py` select imports using **`currentRobot`** from `robot_config.py`.

## When to regenerate

Regenerate when you:

- Change wheel diameter, module mount angles, or CAN/CAD offsets
- Replace swerve modules or move them on the chassis
- Update Phoenix firmware/generator output format for the season

Follow CTRE’s documentation for exporting the **RobotPy** project and **replace** the files here rather than hand-merging.

## Student safety rules

- Do **not** “guess” numeric fixes in generated files—rotations and offsets are coupled.
- Always **test straight-line and rotation** after flashing new constants before enabling aggressive autos.

## See also

- [../subsystems/swerve.md](../subsystems/swerve.md)
