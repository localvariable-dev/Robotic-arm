# CoppeliaSim scene

Run `scripts/build_scene.lua` from CoppeliaSim's Lua editor in an empty/new scene. This scene-builder script uses the simulator's primitive-shape, joint, parenting, color, and pose APIs to create a real articulated model (not a prerecorded animation). Then save the scene as `scene.ttt`.

The scene file is intentionally not committed yet: CoppeliaSim is required to generate its native binary scene format, and no simulator execution was available in this code authoring session. Once generated and locally checked, add `simulation/scene.ttt` to version control.

Expected names are listed in the root configuration. If your CoppeliaSim edition changes an API, use the simulator console error line to adjust the script. Check joint axes/motion and gripper clearance before running the full demo.
