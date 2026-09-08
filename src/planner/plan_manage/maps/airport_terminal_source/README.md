# Open-RMF Airport Terminal source

`airport_terminal.building.yaml` and `airport_terminal.png` are copied from
[`open-rmf/rmf_demos`](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_maps/maps/airport_terminal)
at commit `4f8850fd3ff9c214252e4428d2ed03a646e6c839`.

The upstream project is distributed under Apache License 2.0. The generated
`../airport_terminal_core.pcd` is a geometry-only derivative containing the L1
structural walls. It excludes visual textures, people, furniture, doors, lifts,
and the RMF navigation graph.

The source measures approximately `282.1 x 83.4 m`. The converter centres it at
the world origin; the generated bounds are approximately `x=[-141.06, 141.06]`
and `y=[-41.68, 41.68] m`.
