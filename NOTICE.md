# Third-party notice

Franka Stack keeps third-party source history outside the parent repository.

- OpenPI is included as the pinned `third_party/openpi` Git submodule from
  [`Loule0-0/openpi-franka`](https://github.com/Loule0-0/openpi-franka), a fork
  of [`Physical-Intelligence/openpi`](https://github.com/Physical-Intelligence/openpi).
  Its history, authorship, Apache-2.0 license, and Gemma license remain in that
  repository.
- GELLO, Polymetis, libfranka, franka_ros2, LeRobot, and other dependencies are
  installed or referenced at pinned versions where the relevant profile
  requires them. Their own licenses and notices continue to apply.

The separation is architectural and attribution-preserving: the parent history
tracks Franka Stack work, while upstream projects retain their original
contributors and commit histories.
