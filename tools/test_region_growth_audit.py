"""Run with unittest; no ROS or production node construction required."""
from pathlib import Path
import copy
import json
import os
import unittest

import region_growth_audit as audit

ROOT = Path(os.environ.get('SCAN_PLANNER_SOURCE_ROOT', Path(__file__).resolve().parents[1]))


class GrowthAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.inputs = {c['name']: c for c in audit.demo_cases()}
        cls.results = {name: audit.run_case(value, ROOT) for name, value in cls.inputs.items()}

    def last(self, name):
        return self.results[name]['frames'][-1]

    def test_advance_disagrees_with_actual_tracker_for_evidence_reason(self):
        frame = self.last('open_advance_8m')
        self.assertEqual([1], [r['id'] for r in frame['baseline_regions']])
        row, = frame['clusters']
        self.assertEqual('CONTINUITY_SUPPORTED', row['status'])
        self.assertEqual(0, row['suggested_parent_id'])
        self.assertTrue(row['baseline_identity_disagreement'])

    def test_all_growth_witnesses_are_free_and_measured_inside(self):
        count = 0
        for name, result in self.results.items():
            for index, frame in enumerate(result['frames']):
                data = self.inputs[name]['frames'][index]['occupancy']
                measured = {tuple(c) for c in frame['delta']['measured_unknown_to_free_cells']}
                for row in frame['clusters']:
                    for witness in row['witnesses'].values():
                        path = witness['path']
                        self.assertTrue(all(data[y][x] == audit.FREE for x,y in path))
                        self.assertTrue(all(abs(a[0]-b[0])+abs(a[1]-b[1]) == 1
                                            for a,b in zip(path,path[1:])))
                        if witness['kind'] == 'MEASURED_FREE_GROWTH':
                            count += 1
                            self.assertTrue(all(tuple(c) in measured for c in path[1:-1]))
        self.assertGreater(count, 0)

    def test_pillar_does_not_invent_a_baseline_failure(self):
        frame = self.last('pillar_splits_frontier')
        self.assertEqual([0], [r['id'] for r in frame['baseline_regions']])
        self.assertEqual(2, len(frame['clusters']))
        self.assertTrue(all(r['suggested_parent_id'] == 0 for r in frame['clusters']))
        self.assertTrue(all(not r['baseline_identity_disagreement'] for r in frame['clusters']))
        self.assertTrue(any(c['status'] == 'SHORT_CONNECTION_SUPPORTED'
                            for c in frame['continuity_checks']))

    def test_t_continuations_are_not_blindly_merged(self):
        frame = self.last('t_junction_multiple_continuations')
        self.assertEqual(2, len(frame['clusters']))
        self.assertTrue(all(r['candidate_parent_ids'] == [0] for r in frame['clusters']))
        self.assertTrue(all(r['status'] == 'REVIEW_MULTIPLE_CONTINUATIONS'
                            and r['suggested_parent_id'] is None for r in frame['clusters']))
        self.assertEqual(1, len(frame['active_child_candidates']))

    def test_multiple_sources_are_preserved(self):
        row, = self.last('two_parents_meet')['clusters']
        self.assertEqual([0,1], row['candidate_parent_ids'])
        self.assertEqual('AMBIGUOUS_MULTIPLE_PARENTS', row['status'])
        self.assertIsNone(row['suggested_parent_id'])
        self.assertEqual({0,1}, set(row['witnesses']))

    def test_unmeasured_geometry_never_becomes_growth(self):
        frame = self.last('unmeasured_free_is_not_growth')
        self.assertEqual([], frame['delta']['measured_unknown_to_free_cells'])
        self.assertTrue(all(r['status'] == 'NO_PARENT_EVIDENCE' for r in frame['clusters']))

    def test_absent_provenance_is_explicit(self):
        case = copy.deepcopy(self.inputs['open_advance_8m'])
        del case['frames'][-1]['measured_free_cells']
        frame = audit.run_case(case, ROOT)['frames'][-1]
        self.assertFalse(frame['delta']['evidence_supplied'])
        self.assertEqual('NO_PARENT_EVIDENCE', frame['clusters'][0]['status'])

    def test_growth_capacity_is_not_geometry_failure(self):
        frame = self.last('growth_budget_exhausted')
        self.assertFalse(frame['growth_complete'])
        self.assertEqual(20, frame['growth_budget_used'])
        self.assertTrue(all(r['status'] == 'DEFERRED_GROWTH_BUDGET' for r in frame['clusters']))

    def test_continuity_capacity_is_not_split_proof(self):
        frame = self.last('continuity_budget_exhausted')
        self.assertEqual(0, frame['continuity_budget_used'])
        self.assertTrue(all(r['status'] == 'DEFERRED_CONTINUITY_BUDGET' for r in frame['clusters']))

    def test_cold_start_has_an_explicit_creation_suggestion(self):
        frame = self.last('cold_start')
        self.assertTrue(all(r['status'] == 'NEW_WORK_CANDIDATE' for r in frame['clusters']))

    def test_input_and_source_task_fields_are_untouched(self):
        case = copy.deepcopy(self.inputs['open_advance_8m'])
        case['frames'][0]['task_context'] = dict(task_id=43, task_version=2,
                                                frozen_targets=[[15,3]])
        original = copy.deepcopy(case)
        audit.run_case(case, ROOT)
        self.assertEqual(original, case)

    def test_budget_does_not_change_baseline(self):
        full = self.last('open_advance_8m')['baseline_regions']
        limited = self.last('growth_budget_exhausted')['baseline_regions']
        self.assertEqual(full, limited)

    def test_epoch_reset_does_not_inherit_previous_world(self):
        case = copy.deepcopy(self.inputs['open_advance_8m'])
        case['frames'][-1].update(epoch=1, revision=1)
        frame = audit.run_case(case, ROOT)['frames'][-1]
        self.assertTrue(frame['epoch_reset'])
        self.assertTrue(all(r['status'] == 'NEW_WORK_CANDIDATE'
                            and not r['candidate_parent_ids'] for r in frame['clusters']))

    def test_revision_replay_is_not_a_second_observation(self):
        case = copy.deepcopy(self.inputs['open_advance_8m'])
        case['frames'][-1]['revision'] = 1
        with self.assertRaisesRegex(ValueError, 'increase'):
            audit.run_case(case, ROOT)

    def test_invalid_free_evidence_is_rejected(self):
        case = copy.deepcopy(self.inputs['open_advance_8m'])
        case['frames'][-1]['measured_free_cells'].append([0,0])
        with self.assertRaisesRegex(ValueError, 'currently be FREE'):
            audit.run_case(case, ROOT)

    def test_input_order_does_not_choose_a_parent(self):
        case = copy.deepcopy(self.inputs['two_parents_meet'])
        for frame in case['frames']:
            frame['measured_free_cells'].reverse()
        frame = audit.run_case(case, ROOT)['frames'][-1]
        self.assertEqual([0,1], frame['clusters'][0]['candidate_parent_ids'])
        self.assertEqual('AMBIGUOUS_MULTIPLE_PARENTS', frame['clusters'][0]['status'])

    def test_unchanged_scan_does_not_manufacture_growth(self):
        case = copy.deepcopy(self.inputs['cold_start'])
        frame = copy.deepcopy(case['frames'][0])
        frame['revision'] = 2
        case['frames'].append(frame)
        result = audit.run_case(case, ROOT)['frames'][-1]
        self.assertEqual([], result['delta']['measured_unknown_to_free_cells'])
        self.assertEqual(0, result['delta']['changed_cells'])
        self.assertEqual(0, result['clusters'][0]['suggested_parent_id'])
        self.assertEqual('OVERLAP', result['clusters'][0]['witnesses'][0]['kind'])

    def test_old_source_turned_occupied_cannot_support_growth(self):
        case = copy.deepcopy(self.inputs['open_advance_8m'])
        for y in range(1,23):
            case['frames'][-1]['occupancy'][y][14] = audit.OCCUPIED
        frame = audit.run_case(case, ROOT)['frames'][-1]
        self.assertTrue(frame['clusters'])
        self.assertTrue(all(not r['candidate_parent_ids'] for r in frame['clusters']))
        self.assertTrue(all(r['status'] == 'NO_PARENT_EVIDENCE' for r in frame['clusters']))

    def test_json_report_is_serializable(self):
        encoded = json.dumps(self.results, sort_keys=True)
        self.assertIn('MEASURED_FREE_GROWTH', encoded)


if __name__ == '__main__':
    unittest.main()
