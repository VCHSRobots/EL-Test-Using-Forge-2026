# The robot's model for AdvantageScope

`Robot_Forge/` is a custom robot for AdvantageScope's 3D field: the robot's body and one moving part, the elevator's carriage. The program publishes the carriage's place as `/Forge/Elevator/Components` (an array of `Pose3d`, robot-relative, one per component in index order) and the elevator as a `Mechanism2d` at `/Forge/Elevator/Mechanism`.

## Files

- `config.json`: the robot's name ("Forge"), how to turn and shift the body model so the robot's origin is at the centre of its frame on the floor (`rotations`, `position` in metres), and one entry per component. Forge maintains it; tell Forge when the CAD changes.
- `model.glb`: the robot's body without the carriage. Committed (8.5 MB, repacked from CAD Assistant's output; see `docs/sim-and-forge-lessons.md`). Moved here from `VCHSRobots/elevator` (now `VCHSRobots/bench-lamp-led`) on 15 Nov 2026.
- `model_0.glb`: the carriage alone, exported in the same assembly coordinates as `model.glb`.

`zeroedPosition` moves the carriage model so its origin is at the robot's origin; the program then moves it to its live place. It is the negative of `CARRIAGE_ORIGIN_AT_BOTTOM` in `src/main/java/frc/robot/subsystems/ElevatorView.java`, with the carriage exported at the bottom of travel; Forge keeps the two in step.

## From Onshape to glb (a student)

1. In Onshape, put the elevator at the bottom of its travel. Export the main assembly as STEP (not Onshape's own glTF, which is far too large). Export the carriage's parts as a second STEP, from the same assembly so the coordinates match.
2. Open each STEP in CAD Assistant (free, from OpenCascade). In the export settings tick "Merge faces within the same part". Save the body as `model.glb` and the carriage as `model_0.glb` (binary glTF).
3. Put both files in `Robot_Forge/`, commit them, and tell Forge the robot's dimensions so it can set `rotations` and `position`.

AdvantageScope simplifies the geometry itself; a mesh whose name contains `NOSIMPLIFY` is kept exact.

## Showing it

In AdvantageScope (v26), choose Help, then Use Custom Assets Folder, and pick this `assets` folder. Connect to the simulator (File, Connect to Simulator) or open a log; in a 3D Field tab choose the robot "Forge", drag the robot's pose onto it, and drag `/Forge/Elevator/Components` onto it as its components. A Mechanism tab shows `/Forge/Elevator/Mechanism`.
