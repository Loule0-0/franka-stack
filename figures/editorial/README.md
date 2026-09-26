# Editorial figures

These figures are the human-facing overview layer for Franka Stack. The
Mermaid source in [`../franka-end-to-end.mmd`](../franka-end-to-end.mmd) remains
the compact, editable topology reference.

| File | Purpose | Prompt summary |
| --- | --- | --- |
| `franka-stack-pipeline.png` | Repository hero and complete physical-robot path | Warm editorial engineering plate: demonstrate → curate → pluggable policy backend → guarded deployment → Franka robot |
| `pi05-policy-contract.png` | Current π0.5 reference backend contract | Five observation streams → π0.5 → 20-step chunk → seven joint deltas plus absolute gripper |
| `deployment-gates.png` | Real-time and safety boundary | GPU loopback → SSH → operator client → real-time boundary → RT NUC → Franka robot, with four explicit gates |

The visual direction uses warm ivory paper, charcoal hairlines, oxblood
junctions, restrained olive/blue accents, serif labels, and generous
whitespace. It was generated with the built-in image generation tool using
user-provided images as style references only; all diagrams and content are
original to this repository.
