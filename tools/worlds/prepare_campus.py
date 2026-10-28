#!/usr/bin/env python3
"""Prepare the official Campus for Fortress without changing world geometry."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import urllib.request
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
SOURCE_URL = 'https://drive.usercontent.google.com/download?id=1V4MWCD3qYLr3RFLLATbqAh_lXKcCU39C&export=download&confirm=t'


def run(command, log):
    with log.open('w') as stream:
        subprocess.run(command, check=True, stdout=stream, stderr=subprocess.STDOUT)


def validate(reference, visual):
    """Compare every loaded triangle, independent of face/corner ordering."""
    assert reference.shape == visual.shape, 'triangle count changed'
    tree = cKDTree(reference.mean(axis=1, dtype='f8'))
    permutations = list(itertools.permutations(range(3)))
    maximum = 0.0
    for start in range(0, len(visual), 50000):
        batch = visual[start:start+50000]
        distances, index = tree.query(batch.mean(axis=1, dtype='f8'), workers=2)
        if distances.max() > .001:
            raise RuntimeError('visual/collision coordinates disagree by >1mm')
        matched = reference[index]
        errors = np.full(len(batch), np.inf)
        for order in permutations:
            errors = np.minimum(errors, np.linalg.norm(batch-matched[:, order, :], axis=2).max(axis=1))
        for i in np.where(errors > .001)[0]:
            _, nearby = tree.query(batch[i].mean(axis=0, dtype='f8'), k=8)
            for j in nearby:
                errors[i] = min(errors[i], min(float(np.linalg.norm(batch[i]-reference[j, list(order), :], axis=1).max()) for order in permutations))
        maximum = max(maximum, float(errors.max()))
        if maximum > .001:
            raise RuntimeError('visual/collision triangle mismatch')
    return maximum


def write_stl(triangles, target):
    record = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    with target.open('wb') as stream:
        stream.write(b'Campus Z-up triangles from Gazebo MeshManager'.ljust(80, b'\0'))
        stream.write(np.uint32(len(triangles)).astype('<u4').tobytes())
        for start in range(0, len(triangles), 100000):
            tri = triangles[start:start+100000]
            rows = np.zeros(len(tri), dtype=record)
            rows['vertices'] = tri
            normal = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
            length = np.linalg.norm(normal, axis=1)
            np.divide(normal, length[:, None], out=normal, where=length[:, None] > 0)
            rows['normal'] = normal
            stream.write(rows.tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--source', type=Path, help='extracted campus directory with meshes/campus.dae')
    group.add_argument('--archive', type=Path, help='official environments ZIP (filename may be .download)')
    group.add_argument('--download', action='store_true', help='download the public official environments archive')
    parser.add_argument('--output', type=Path, default=REPO/'assets/runtime_worlds/campus')
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.download:
        args.archive = output/'environments.zip'
        with urllib.request.urlopen(SOURCE_URL, timeout=120) as response, args.archive.open('wb') as stream:
            shutil.copyfileobj(response, stream)
    if args.archive:
        with zipfile.ZipFile(args.archive) as archive:
            # Extract only Campus and reject path traversal in downloaded input.
            for entry in archive.infolist():
                path = Path(entry.filename)
                if path.parts and path.parts[0] == 'campus':
                    if path.is_absolute() or '..' in path.parts:
                        raise ValueError('unsafe archive member')
                    archive.extract(entry, output/'source')
        args.source = output/'source/campus'
    source = args.source.expanduser().resolve()
    dae = source/'meshes/campus.dae'
    if not dae.is_file():
        parser.error(f'missing {dae}')
    up = ET.parse(dae).getroot()
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    if up.findtext('c:asset/c:up_axis', namespaces=ns) != 'Z_UP':
        parser.error('this recipe requires the official Z_UP Campus')
    build = output/'build'
    build.mkdir(exist_ok=True)
    for name, packages in [('export_runtime', ['assimp']), ('load_mesh', ['ignition-common4-graphics'])]:
        flags = subprocess.check_output(['pkg-config', '--cflags', '--libs', *packages], text=True).split()
        subprocess.run(['c++', '-O2', '-std=c++17', str(HERE/f'{name}.cpp'), '-o', str(build/name), *flags], check=True)
    model = output/'models/campus_runtime'
    visual = model/'visual'
    visual.mkdir(parents=True, exist_ok=True)
    run([str(build/'export_runtime'), str(dae), str(visual/'campus_zup.obj')], output/'export.log')
    textures = visual/'textures'
    textures.mkdir(exist_ok=True)
    mtl = visual/'campus_zup.mtl'
    material_lines = []
    for line in mtl.read_text().splitlines():
        if line.startswith('map_Kd '):
            name = Path(line[7:]).name
            src = source/'meshes'/name
            if not src.is_file():
                raise FileNotFoundError(src)
            with Image.open(src) as image:
                image.thumbnail((2048, 2048), getattr(Image, "Resampling", Image).LANCZOS)
                image.save(textures/name)
            line = 'map_Kd textures/'+name
        material_lines.append(line)
    mtl.write_text('\n'.join(material_lines)+'\n')
    reference_file = output/'reference_triangles.f32'
    visual_file = output/'visual_triangles.f32'
    run([str(build/'load_mesh'), str(dae), str(reference_file)], output/'reference_loader.log')
    run([str(build/'load_mesh'), str(visual/'campus_zup.obj'), str(visual_file)], output/'visual_loader.log')
    reference = np.memmap(reference_file, dtype='<f4', mode='r').reshape(-1, 3, 3)
    rendered = np.memmap(visual_file, dtype='<f4', mode='r').reshape(-1, 3, 3)
    error = validate(reference, rendered)
    write_stl(reference, model/'campus_collision.stl')
    # Both the collision and visual use the same Z_UP coordinates and pose.
    shutil.copy2(REPO/'resources/worlds/campus_fortress.sdf', output/'campus_fortress_runtime.sdf')
    (model/'model.config').write_text('<model><name>campus_runtime</name><version>1</version><sdf version="1.8">model.sdf</sdf></model>\n')
    world = ET.parse(output/'campus_fortress_runtime.sdf')
    model_tree = ET.Element('sdf', version='1.8')
    model_tree.append(world.find('.//model[@name="campus"]'))
    ET.ElementTree(model_tree).write(model/'model.sdf', encoding='utf-8', xml_declaration=True)
    result = {'source_url': SOURCE_URL, 'source_dae_sha256': hashlib.sha256(dae.read_bytes()).hexdigest(), 'triangles': len(reference), 'max_triangle_vertex_error_m': error, 'up_axis': 'Z_UP', 'world': str(output/'campus_fortress_runtime.sdf'), 'resource_path': str(output/'models')}
    (output/'preparation_manifest.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
