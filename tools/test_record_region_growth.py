import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import unittest

from record_region_growth import BagSink, RegionRecording, TOPICS


class FakeSink:
    def __init__(self, path):
        self.rows=[]
        self.closed=False
    def append(self, topic, message, receipt_wall_ns):
        self.rows.append((topic,message,receipt_wall_ns))
    def close(self):
        self.closed=True


def header(ns=1):
    return NS(stamp=NS(sec=ns//10**9,nanosec=ns%10**9),frame_id='world')


def odom(ns=1):
    return NS(header=header(ns),pose=NS(pose=NS(position=NS(x=1.,y=2.,z=.4))),
              twist=NS(twist=NS(linear=NS(x=.3,y=0.,z=0.))))


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.out=Path(self.temp.name)/'capture'
        self.session=RegionRecording(self.out,148,sink_factory=FakeSink)
    def tearDown(self):
        self.session.close()
        self.temp.cleanup()

    def test_no_pretend_full_map_or_measured_provenance(self):
        self.session.flush()
        m=json.loads((self.out/'session.json').read_text())
        self.assertFalse(m['raw_map_history_complete'])
        self.assertFalse(m['measured_free_provenance_available'])
        self.assertFalse(m['source_snapshot_schema_compatible'])

    def test_receipt_and_header_time_are_both_preserved(self):
        msg=odom(123)
        before=copy.deepcopy(msg)
        self.session.append('/quad_0/body_pose',msg,wall_ns=9000,monotonic_ns=2000)
        self.session.flush()
        event=json.loads((self.out/'events.jsonl').read_text())
        self.assertEqual(123,event['message_header_stamp_ns'])
        self.assertEqual(9000,event['receipt_wall_ns'])
        self.assertEqual(2000,event['receipt_monotonic_ns'])
        self.assertEqual(msg,before)

    def test_odom_sampling_and_duplicates_are_explicit(self):
        self.assertTrue(self.session.append('/quad_0/body_pose',odom(100),monotonic_ns=0))
        self.assertFalse(self.session.append('/quad_0/body_pose',odom(101),monotonic_ns=1000))
        self.assertFalse(self.session.append('/quad_0/body_pose',odom(100),monotonic_ns=300_000_000))
        self.assertEqual(3,self.session.received['/quad_0/body_pose'])
        self.assertEqual(1,self.session.written['/quad_0/body_pose'])
        self.assertEqual(1,self.session.skipped['/quad_0/body_pose:rate'])
        self.assertEqual(1,self.session.skipped['/quad_0/body_pose:duplicate_stamp'])

    def test_clock_reset_is_seen_even_on_rate_limited_message(self):
        self.session.append('/clock',NS(clock=NS(sec=10,nanosec=0)),monotonic_ns=0)
        self.session.append('/clock',NS(clock=NS(sec=1,nanosec=0)),monotonic_ns=1000)
        self.assertEqual(1,self.session.clock_resets)
        self.assertEqual(1_000_000_000,self.session.clock_ns)
        self.assertEqual(1,self.session.written['/clock'])

    def test_records_actual_published_region_identity(self):
        m=NS(ns='information_regions',id=17,action=0,points=[1,2],
             color=NS(r=.1,g=.2,b=.3,a=.8))
        self.session.append('/explorer/region_overlay',NS(markers=[m]))
        self.assertEqual([17],self.session.latest_region_ids)
        self.session.append('/explorer/region_overlay',NS(markers=[]))
        self.assertEqual([],self.session.latest_region_ids)

    def test_published_task_does_not_claim_installed_motion(self):
        msg=NS(task_id=9,task_version=2,request_id=10,lifecycle=1,region_id=17,
               has_region=True,goal=NS(x=2.,y=3.),target_cell_count=12,target_cells=[],coarse_route=[])
        self.session.append('/observation_task',msg)
        self.assertEqual(9,self.session.latest_published_task['task_id'])
        self.assertNotIn('installed',self.session.latest_published_task)
        self.assertNotIn('active_task',self.session.manifest)

    def test_updates_before_full_map_are_not_silently_complete(self):
        update=NS(header=header(3),x=1,y=2,width=3,height=4,data=[0]*12)
        self.session.append('/map_updates',update)
        self.session.flush()
        e=json.loads((self.out/'events.jsonl').read_text())
        self.assertFalse(e['payload']['full_map_received'])

    def test_existing_recording_is_not_overwritten(self):
        with self.assertRaises(FileExistsError):
            RegionRecording(self.out,148,sink_factory=FakeSink)

    def test_close_writes_final_manifest(self):
        self.session.append('/planning/status',NS(data='REFERENCE_PATH_ACCEPTED request_id=10'))
        self.session.close()
        self.session.close()
        m=json.loads((self.out/'session.json').read_text())
        self.assertEqual('closed',m['status'])
        self.assertEqual(1,m['written']['/planning/status'])
        self.assertTrue(self.session.sink.closed)


@unittest.skipUnless(importlib.util.find_spec('rosbag2_py'), 'source ROS Humble for CDR/bag integration test')
class BagIntegrationTests(unittest.TestCase):
    def test_real_cdr_bag_roundtrip_without_dds_or_robot(self):
        import rosbag2_py
        from nav_msgs.msg import OccupancyGrid
        from map_msgs.msg import OccupancyGridUpdate
        from rclpy.serialization import deserialize_message
        with tempfile.TemporaryDirectory() as temp:
            session=RegionRecording(Path(temp)/'capture',148)
            msg=OccupancyGrid()
            msg.header.frame_id='world'
            msg.header.stamp.sec=7
            msg.info.width=3;msg.info.height=2;msg.info.resolution=.2
            msg.info.origin.orientation.w=1.
            msg.data=[0,-1,100,0,0,-1]
            session.append('/explorer/map',msg,wall_ns=1000,monotonic_ns=1000)
            update=OccupancyGridUpdate()
            update.header.frame_id='world';update.header.stamp.sec=8
            update.x=1;update.y=0;update.width=1;update.height=1;update.data=[0]
            session.append('/map_updates',update,wall_ns=2000,monotonic_ns=2000)
            session.close()
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(session.output/'bag'),storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('',''))
            self.assertEqual(len(TOPICS),len(reader.get_all_topics_and_types()))
            topic,data,stamp=reader.read_next()
            decoded=deserialize_message(data,OccupancyGrid)
            self.assertEqual('/explorer/map',topic)
            self.assertEqual(1000,stamp)
            self.assertEqual(7,decoded.header.stamp.sec)
            self.assertEqual(list(msg.data),list(decoded.data))
            topic,data,stamp=reader.read_next()
            decoded=deserialize_message(data,OccupancyGridUpdate)
            self.assertEqual('/map_updates',topic)
            self.assertEqual(2000,stamp)
            self.assertEqual([0],list(decoded.data))
            self.assertFalse(reader.has_next())


if __name__=='__main__':
    unittest.main()
