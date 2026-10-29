import csv
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

spec = importlib.util.spec_from_file_location('record_actual_route', Path(__file__).parents[1] / 'record_actual_route.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def odom(t, x=0., y=0., z=.45, frame='world'):
    ns = round(t*1e9)
    return NS(header=NS(stamp=NS(sec=ns//10**9, nanosec=ns%10**9), frame_id=frame),
              child_frame_id='base_link',
              pose=NS(pose=NS(position=NS(x=x,y=y,z=z), orientation=NS(x=0.,y=0.,z=0.,w=1.))),
              twist=NS(twist=NS(linear=NS(x=.5,y=0.,z=0.))))


def test_measured_xyz_timestamp_and_stop_are_preserved(tmp_path):
    out=tmp_path/'route.csv'; w=m.RouteWriter(out)
    for msg in [odom(12.,z=-2.),odom(12.2,x=3.,y=4.,z=-1.),odom(12.4,x=3.,y=4.,z=-1.)]:
        assert w.append(msg,wall_time=100.)
    w.close()
    rows=list(csv.DictReader(out.open()))
    assert len(rows)==3
    assert rows[0]['stamp_ns']=='12000000000' and float(rows[0]['z'])==-2.
    assert float(rows[-1]['distance_xy_m'])==5.
    assert float(rows[-1]['distance_3d_m'])==pytest.approx(26**.5)
    assert float(rows[-1]['elapsed_s'])==pytest.approx(.4)


def test_pause_duplicate_and_invalid_do_not_change_reference(tmp_path):
    w=m.RouteWriter(tmp_path/'r.csv')
    assert w.append(odom(1.))
    assert not w.append(odom(1.,x=100.))
    assert not w.append(odom(1.2,x=float('nan')))
    assert not w.append(odom(1.2,frame=''))
    assert w.append(odom(1.4,x=1.))
    assert w.count==2 and w.distance_xy==1.
    w.close()


def test_clock_reset_frame_change_and_input_gap_segment_without_bridge(tmp_path):
    out=tmp_path/'r.csv';w=m.RouteWriter(out)
    for msg in [odom(10.),odom(10.2,x=1.),odom(1.,x=100.),
                odom(1.2,x=200.,frame='odom'),odom(9.,x=400.,frame='odom')]:
        assert w.append(msg)
    w.close()
    rows=list(csv.DictReader(out.open()))
    assert [int(r['segment']) for r in rows]==[0,0,1,2,3]
    assert w.distance_xy==1.


def test_existing_recording_and_metadata_are_not_overwritten(tmp_path):
    p=tmp_path/'r.csv';w=m.RouteWriter(p);w.close()
    original=p.read_bytes()
    with pytest.raises(FileExistsError):m.RouteWriter(p)
    assert p.read_bytes()==original
    q=tmp_path/'other.csv';q.with_suffix('.json').write_text('owned')
    with pytest.raises(FileExistsError):m.RouteWriter(q)
    assert not q.exists() and q.with_suffix('.json').read_text()=='owned'
