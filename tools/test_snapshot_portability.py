"""Resource closure and dry-run tests; no Gazebo, arming or motion commands."""
import importlib.util
import json
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[1]

class PortableSnapshotTest(unittest.TestCase):
    def test_newmine_meshes_resolve(self):
        root = ET.parse(REPO/'resources/newmine/NewMine.sdf')
        for uri in root.findall('.//uri'):
            self.assertTrue(uri.text.startswith('model://'), uri.text)
            self.assertTrue((REPO/'resources/newmine/models'/uri.text[8:]).is_file())

    def test_newmine_images_are_local_and_resolve(self):
        for path in (REPO/'resources/newmine/models').rglob('*.DAE'):
            for node in ET.parse(path).iter():
                if node.tag.endswith('init_from') and node.text:
                    self.assertNotIn('://', node.text)
                    self.assertTrue((path.parent/node.text).is_file(), str(path)+': '+node.text)

    def test_default_dry_run_has_explicit_policy_and_resources(self):
        result = subprocess.run(['/usr/bin/python3', str(REPO/'run_newmine_rl.py'),
                                 '--dry-run', '--no-sparse-routing',
                                 '--workspace', '/tmp/dyn workspace with spaces'],
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('sparse_routing_enabled:=false', result.stdout)
        self.assertIn('policy_model_path:=', result.stdout)
        self.assertIn('gazebo_resource_path:=', result.stdout)
        self.assertNotIn('/home/t1an/ros2_ws/mine_tunnel_world', result.stdout)

    def test_bad_world_is_rejected_before_launch(self):
        result = subprocess.run(['/usr/bin/python3', str(REPO/'run_newmine_rl.py'),
                                 '--dry-run', '--world', '/nonexistent/dyn.sdf'],
                                text=True, capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('world file not found', result.stderr)

    def test_newmine_manifest(self):
        import hashlib
        base = REPO/'resources/newmine'
        manifest = json.loads((base/'MANIFEST.json').read_text())
        for name, expected in manifest['files_sha256'].items():
            self.assertEqual(hashlib.sha256((base/name).read_bytes()).hexdigest(), expected, name)

    def test_policy_digest(self):
        import hashlib
        manifest = json.loads((REPO/'models/go2_rl_lab_policy.json').read_text())
        self.assertEqual(hashlib.sha256((REPO/'models/go2_rl_lab_policy.pt').read_bytes()).hexdigest(), manifest['sha256'])

    def test_world_template_preserves_axis_and_physics(self):
        world = ET.parse(REPO/'resources/worlds/campus_fortress.sdf')
        self.assertEqual(world.findtext('.//gravity'), '0 0 -9.81')
        self.assertTrue(world.findtext('.//visual/geometry/mesh/uri').endswith('campus_zup.obj'))
        self.assertEqual(world.findtext('.//collision/geometry/mesh/uri'), 'model://campus_runtime/campus_collision.stl')

if __name__ == '__main__':
    unittest.main()
