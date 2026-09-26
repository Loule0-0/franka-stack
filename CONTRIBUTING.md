# Contributing to Franka Stack

Contributions are welcome, especially reproducible hardware reports, new Franka
profiles, teleoperation adapters, policy backends, and documentation fixes.

Before opening a pull request:

1. Keep hardware behavior fail-closed and do not weaken limits, metadata, or
   timeout checks to make a test pass.
2. Add tests for contract or runtime changes and document which validation gate
   was exercised on physical hardware.
3. Run `uv sync --frozen --dev` and `uv run pytest -q` in the parent checkout.
4. Run `bash -n scripts/franka/*.sh deploy/panda-polymetis/*.sh` on Linux or WSL
   when shell scripts change.
5. Do not commit credentials, private network details, datasets, checkpoints,
   or unredacted camera recordings.

Changes to generic OpenPI behavior belong upstream whenever possible. Changes
needed only by the reference policy backend should target the `franka-stack`
branch of [`openpi-franka`](https://github.com/Loule0-0/openpi-franka), then
update the pinned submodule commit in this repository.
