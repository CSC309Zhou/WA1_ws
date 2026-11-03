#!/usr/bin/env python3
"""Open the WA1 MuJoCo scene at its configured home pose."""

from pathlib import Path

import mujoco


SCENE_PATH = Path(__file__).resolve().parent / "WA1_400L" / "scene.xml"


def load_home() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_id < 0:
        raise RuntimeError("WA1 scene has no home keyframe; run main.py WA1_400L --convert")
    mujoco.mj_resetDataKeyframe(model, data, home_id)
    mujoco.mj_forward(model, data)
    return model, data


if __name__ == "__main__":
    import mujoco.viewer

    mujoco.viewer.launch(*load_home())
