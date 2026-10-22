"""Run the six-block pick-and-place task against an already-open CoppeliaSim scene."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.simulator_adapter.coppelia import CoppeliaAdapter, SimulatorConnectionError
from src.task_manager.pick_place import PickPlaceTask


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/simulation_config.json")
    parser.add_argument("--scene", default="simulation/scene.ttt", help="Expected scene path (open this scene in CoppeliaSim first)")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    args = parser.parse_args()
    cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
    host = args.host or cfg["simulation"]["host"]
    port = args.port or cfg["simulation"]["port"]
    print(f"Expected scene: {args.scene}")
    if not Path(args.scene).exists():
        print("ERROR: scene file not found. Create/save it using simulation/scripts/build_scene.lua in CoppeliaSim.")
        return 2
    try:
        adapter = CoppeliaAdapter(
            host, port, cfg["simulation"]["motion_timeout_s"],
            base_world=cfg["frame"]["base_world"], tool_offset=cfg["frame"]["tool_offset"],
        )
        adapter.verify_scene(cfg["objects"]["blocks"])
        adapter.start()
        task = PickPlaceTask(
            adapter, cfg["objects"]["blocks"], cfg["source_positions"], cfg["destination_positions"],
            cfg["robot"]["approach_height_m"], cfg["robot"]["safe_transfer_height_m"], cfg["robot"]["place_height_m"],
        )
        result = task.run()
        print(json.dumps({"success": result.success, "placed": result.placed, "failed_block": result.failed_block,
                          "error": result.error, "state_history": result.state_history}, indent=2))
        adapter.stop()
        return 0 if result.success and len(result.placed) == len(cfg["objects"]["blocks"]) else 1
    except SimulatorConnectionError as exc:
        print(f"ERROR: {exc}")
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
