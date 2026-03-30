# `util.py` — shared helpers

File: [`util.py`](../../util.py)

## Logging and diagnostics

- **`install_loop_overrun_stderr_filter()`** — Suppresses noisy WPILib timing messages on stderr so students see real failures first.
- **`install_safe_power_distribution_logging()`** — Avoids PyKit CAN errors when no PDH is present or IDs mismatch; pairs with `Constants.CanIDs.POWER_DISTRIBUTION_MODULE_ID`.

## Game metadata

- **`get_game_phase()`** — Returns a human-readable phase label and timer slice for dashboard use (works with FMS game data and alliance color).
- **`hub_status(...)`** — Helper for phase strings involving the hub.

## Geometry helpers

- **`make_turret_pose_supplier(...)`** — Produces pose suppliers that offset robot center to **turret center** using `Constants.TURRET_OFFSET` for consistent distance-to-goal calculations.

## Phoenix helpers

- **`try_until_ok(...)`** — Retries Phoenix calls that may briefly fail during bring-up.

When adding cross-cutting helpers, prefer **pure functions** and **small surfaces** so subsystems stay easy to read.
