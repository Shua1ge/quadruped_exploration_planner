import pathlib
import sys

import pytest


SCRIPT_DIR = pathlib.Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))
from explorer_core.sdf_world_geometry import load_sdf_world  # noqa: E402


COLLADA = """<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="0.01" name="centimeter"/><up_axis>Y_UP</up_axis></asset>
  <library_geometries><geometry id="wall"><mesh>
    <source id="positions"><float_array id="positions-array" count="9">0 0 0 100 0 0 0 0 100</float_array>
      <technique_common><accessor source="#positions-array" count="3" stride="3"><param name="X"/><param name="Y"/><param name="Z"/></accessor></technique_common>
    </source>
    <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
    <triangles count="1"><input semantic="VERTEX" source="#vertices" offset="0"/><p>0 1 2</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="scene"><node><instance_geometry url="#wall"/></node></visual_scene></library_visual_scenes>
</COLLADA>"""


def test_sdf_loader_applies_collada_units_poses_boxes_and_recentering(tmp_path):
    dae = tmp_path / "wall.dae"
    dae.write_text(COLLADA)
    world = tmp_path / "world.sdf"
    world.write_text(f"""<sdf version="1.9"><world name="test">
      <model name="mesh"><pose>10 20 0 0 0 0</pose><link name="link">
        <collision name="wall"><geometry><mesh><uri>{dae}</uri></mesh></geometry></collision>
      </link></model>
      <model name="cap"><pose>12 20 1 0 0 0</pose><link name="link">
        <collision name="cap"><geometry><box><size>2 2 2</size></box></geometry></collision>
      </link></model>
    </world></sdf>""")

    cloud = load_sdf_world(world, spacing=0.5, mode="surface", recenter=True)

    assert cloud.triangle_count == 1
    assert cloud.box_count == 1
    assert cloud.source_bounds[0] == pytest.approx((10.0, 19.0, 0.0))
    assert cloud.source_bounds[1] == pytest.approx((13.0, 21.0, 2.0))
    assert cloud.translation == pytest.approx((-11.5, -20.0, 0.0))
    assert cloud.points
