import unittest
from analyze_local_motion import handoffs,motion,quantiles


def event(name,attempt=1,status=-1,**kwargs):
    return dict(topic='/planning/handoff_metrics',receipt_wall_ns=100,
        payload=dict(event=name,request_id=10,trajectory_id=11,attempt_id=attempt,
            ack_status=status,prepare_wall_ms=100.,cross_task=False,**kwargs))

class AnalysisTests(unittest.TestCase):
    def test_ack_before_attempt_and_duplicates_are_counted_once(self):
        events=[event('ACK',status=4,install_total_wall_ms=130.,reason='ACTIVE'),event('SUBMITTED'),event('ATTEMPT'),event('ACK',status=4)]
        report,rows,_=handoffs(events)
        self.assertEqual(report['installed'],1);self.assertEqual(report['installation_rate'],1.)
        self.assertEqual(report['installed_end_to_end_wall_ms']['p95'],130.)
        self.assertEqual(report['selection_and_recording_events']['duplicate_ACK'],1)

    def test_window_boundary_ack_does_not_inflate_installation_rate(self):
        r,rows,_=handoffs([event('ACK',status=4,install_total_wall_ms=130.)])
        self.assertEqual(r['installed'],0)
        self.assertEqual(r['activation_without_submission'],1)
        self.assertIsNone(r['installation_rate'])

    def test_timeout_late_ack_cannot_grant_activation(self):
        report,rows,_=handoffs([event('SUBMITTED'),event('TIMEOUT'),event('LATE_ACK',status=4)])
        self.assertEqual(report['installed'],0);self.assertEqual(report['late_ack'],1)
        self.assertIsNone(report['installed_end_to_end_wall_ms']['p95'])

    def test_installed_distribution_excludes_fast_rejections(self):
        events=[event('SUBMITTED'),event('ACK',status=4,install_total_wall_ms=500.)]
        events += [event('SUBMITTED',attempt=i) for i in range(2,12)]
        events += [event('ACK',attempt=i,status=2,reason='ENTRY') for i in range(2,12)]
        r,_,_=handoffs(events);self.assertEqual(r['installed_end_to_end_wall_ms']['p95'],500.)
        self.assertAlmostEqual(r['installation_rate'],1/11)
        self.assertEqual(r['ack_rejections']['ENTRY'],10)

    def test_old_record_missing_metrics_is_null_not_zero_latency(self):
        r,_,_=handoffs([]);self.assertFalse(r['available']);self.assertIsNone(r['installation_rate'])
        self.assertIsNone(r['installed_end_to_end_wall_ms']['p95'])

    def test_physical_stop_not_controller_hold_and_gaps_not_filled(self):
        samples=[dict(t=i*.2,wall=i*.2,x=0.,y=0.) for i in range(31)]
        r,s=motion(samples);self.assertEqual(r['stops_ge_3s'],1);self.assertTrue(s[0]['censored'])
        samples += [dict(t=30+i*.2,wall=30+i*.2,x=i*.2,y=0.) for i in range(10)]
        r,s=motion(samples);self.assertEqual(r['invalid_or_discontinuous_samples'],1)
        self.assertLess(r['valid_duration_sim_s'],8.)
        self.assertEqual(r['stops_ge_3s'],1)

    def test_moving_and_out_and_back_are_not_stationary(self):
        samples=[dict(t=i*.2,wall=i*.2,x=(i%10)*.2,y=0.) for i in range(80)]
        r,_=motion(samples);self.assertEqual(r['stops_ge_3s'],0)

if __name__=='__main__':unittest.main()
