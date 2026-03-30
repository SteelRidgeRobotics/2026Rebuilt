# Making changes safely

Use this as a **pre-merge and pre-comp** checklist. Adapt to your team’s GitHub rules.

## Before you code

- Pull latest from the branch your team treats as **integration**.
- Confirm which **robot** you target (Larry vs. comp) and update `robot_config.py` if hardware layout changed.
- Skim [../core/constants-and-config.md](../core/constants-and-config.md) for CAN ID collisions.

## While you code

- Prefer **small diffs**; avoid formatting entire unrelated files.
- Gate new hardware behind **`has_subsystem()`** if only one chassis has it yet.
- Update **docs** when you change operator bindings or auto names.

## Before merging

- Run **tests** and **simulator** at least once for command wiring changes.
- Walk through **bindings** with a mentor (two controllers confuse people fast).
- Verify **PathPlanner** names still match `NamedCommands`.

## Before competition deploy

- Tag or note the **commit hash** you intend to load.
- Deploy **early** at the venue; debug with time to spare.
- Keep a **rollback** USB or known-good branch ready.

## Communication

Tell drive team and mechatronics when anything important changes — software cannot fix a swapped motor controller alone.
