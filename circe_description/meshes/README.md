# circe_description/meshes

Intentionally empty.

The chassis, tracks, and wheels are loaded from `roverrobotics_description/meshes/`
(`flipper_chassis.dae`, `flipper_track.dae`, `pro_tire.dae`) rather than duplicated
here — that package is already in the workspace, so copying 24 MB of `.dae` would buy
nothing. See the `roverrobotics_description` `exec_depend` in `package.xml`.

The RPLidar A1 and the sensor mast are modelled with primitives (box/cylinder) because
no A1 mesh ships upstream and the mast is a CIRCE part. The D456 uses
`realsense2_description/meshes/d455.stl`.

Put CIRCE-specific geometry here when there is any — a real sensor-mast mesh, or the
cable dispenser from `IMPLEMENTATION_PLAN.md` Phase 4.
