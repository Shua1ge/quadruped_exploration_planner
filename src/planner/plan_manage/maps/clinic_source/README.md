# Open-RMF Clinic source

`clinic.building.yaml` and `clinic.png` are copied from
[`open-rmf/rmf_demos`](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_maps/maps/clinic)
at commit `4f8850fd3ff9c214252e4428d2ed03a646e6c839`.

The upstream project is distributed under Apache License 2.0. The generated
`../clinic_L1_core.pcd` is a geometry-only derivative containing L1 structural
walls. It excludes L2, visual textures, people, furniture, doors, lifts, and
the RMF navigation graph.

Clinic has two levels. L1 spans approximately `52.80 x 46.45 m` at elevation
`0 m`; L2 spans approximately `52.83 x 40.67 m` at elevation `10 m`. Only L1 is
used by the current single-level Explorer.

Clinic uses `0.20 m` wall sampling rather than the generator's `0.15 m`
default. Its dense internal partitions otherwise create unnecessary duplicate
LiDAR rays; `0.20 m` still matches the Explorer grid resolution and remains
well inside SCAN's inflated body-clearance margin.
