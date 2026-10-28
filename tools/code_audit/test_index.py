#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Regression checks for the generated audit index.

Run after:  python3 tools/code_audit/generate.py
Then:       python3 tools/code_audit/test_index.py

Checks are ground-truth facts verified by hand against the current source
(point-in-time); they guard the extractor against silent regressions.
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
D = json.load(open(os.path.join(HERE, 'audit_data.json'), encoding='utf-8'))
FAILS = []


def check(name, cond, detail=''):
    print(('PASS  ' if cond else 'FAIL  ') + name + (('  [' + detail + ']') if detail else ''))
    if not cond:
        FAILS.append(name + ' ' + detail)


def funcs_named(name, file_part=None):
    return [f for f in D['funcs'] if f['name'] == name
            and (not file_part or file_part in f['file'])]


def field_named(name):
    return [v for v in D['fields'] if v['name'] == name]


def topic(name):
    return D['topics'].get(name, {})


# ---- 1. file tree / functions ------------------------------------------------
fsm = [f for f in D['funcs'] if f['file'].endswith('scan_replan_fsm.cpp')]
check('fsm functions extracted (>=45)', len(fsm) >= 45, f'{len(fsm)}')
for fn in ['init', 'pathCallback', 'callReboundReplan', 'getLocalTarget',
           'getReferencePathLocalTarget']:
    check(f'fsm function {fn} present', bool(funcs_named(fn, 'scan_replan_fsm.cpp')))
check('adapter function observationTaskRejected present',
      bool(funcs_named('observationTaskRejected', 'observation_task_adapter.cpp')))
check('controller constructor detected',
      bool(funcs_named('ClosedLoopController', 'closed_loop_controller.cpp')))
check('all funcs have sane line ranges',
      all(f['line'] >= 1 and (f['end'] or f['line']) >= f['line'] for f in D['funcs']))
pc = funcs_named('pathCallback', 'scan_replan_fsm.cpp')
check('no self-edge from signature line', bool(pc) and not any(
    e['a'] == pc[0]['id'] and e['b'] == pc[0]['id'] and e['line'] == pc[0]['line']
    for e in D['calls']))
check('member .load() not misresolved as a call edge', bool(pc) and not any(
    e['a'] == pc[0]['id'] and
    next((f['name'] for f in D['funcs'] if f['id'] == e['b']), '') == 'load'
    for e in D['calls']))
check('cross-object call kept (refreshPlanningOdomFromSafety from pathCallback)',
      bool(pc) and any(
          e['a'] == pc[0]['id'] and
          next((f['name'] for f in D['funcs'] if f['id'] == e['b']), '') ==
          'refreshPlanningOdomFromSafety' for e in D['calls']))
check('member receiver type inferred (planner_manager_ -> planGlobalTraj)',
      bool(pc) and any(
          e['a'] == pc[0]['id'] and e['conf'] == 'exact' and
          next((f['name'] for f in D['funcs'] if f['id'] == e['b']), '') ==
          'planGlobalTraj' for e in D['calls']))

# ---- 2. call graph -----------------------------------------------------------
ids = {f['id'] for f in D['funcs']}
check('call count > 1200', len(D['calls']) > 1200, str(len(D['calls'])))
check('no dangling call endpoints',
      all(e['a'] in ids and e['b'] in ids for e in D['calls']))
check('unresolved calls are recorded, not dropped', D['meta']['n_unresolved'] > 0,
      str(D['meta']['n_unresolved']))
conf = {e['conf'] for e in D['calls']}
check('call edges are graded', conf.issubset({'exact', 'name', 'ambig'}) and
      {'exact', 'name'} <= conf, str(sorted(conf)))

# ---- 3. recovery chain: state fields ----------------------------------------
v = field_named('task_lateral_repair_used_')
check('task_lateral_repair_used_ indexed', bool(v))
if v:
    c = v[0]['counts']
    check('task_lateral_repair_used_ has >=5 set sites', c['set'] >= 5, str(c))
    check('task_lateral_repair_used_ has no clear site (retained)', c['clear'] == 0, str(c))
    set_at = {(s['file'].split('/')[-1], s['line']) for s in v[0]['sites'] if s['kind'] == 'set'}
    for want in [('scan_replan_fsm.cpp', 3169), ('scan_replan_fsm.cpp', 3291)]:
        check(f'set site {want}', want in set_at, str(sorted(set_at)[:6]))
check('local_recovery_state_ indexed (nested-scope regression)',
      bool(field_named('local_recovery_state_')))
check('departure_recovery_memory_ indexed', bool(field_named('departure_recovery_memory_')))
bad_ctx = []
for v0 in D['fields']:
    for s0 in v0['sites']:
        if 'ctx' in s0 and not __import__('re').search(
                r'(?<![A-Za-z0-9_])' + __import__('re').escape(v0['name']) +
                r'(?![A-Za-z0-9_])', s0['ctx']):
            bad_ctx.append((v0['name'], s0['file'].split('/')[-1], s0['line']))
check('field sites respect identifier boundaries', not bad_ctx, str(bad_ctx[:3]))

# ---- 4. reason code / log sites --------------------------------------------
sites = {(s['file'].split('/')[-1], s['line']) for s in
         D['reasons'].get('LATERAL_RECOVERY_ALREADY_USED', [])}
check('LATERAL_RECOVERY_ALREADY_USED at scan_replan_fsm.cpp:3288',
      ('scan_replan_fsm.cpp', 3288) in sites, str(sorted(sites)))
stop = [l for l in D['logs'] if 'LATERAL_RECOVERY_STOP' in l['msg']]
check('LATERAL_RECOVERY_STOP log site exists', bool(stop))
if stop:
    f = next((x for x in D['funcs'] if x['id'] == stop[0]['func']), {})
    check('STOP log mapped to latchLateralRecoveryStop',
          f.get('name') == 'latchLateralRecoveryStop',
          f"{stop[0]['file']}:{stop[0]['line']} -> {f.get('name')}")

# ---- 5. message wiring: ObservationFeedback --------------------------------
fb = topic('planning/observation_feedback')
check('ObservationFeedback publisher in scan_replan_fsm.cpp',
      any(p['line'] == 311 and p['file'].endswith('scan_replan_fsm.cpp')
          for p in fb.get('pubs', [])), str(fb.get('pubs')))
check('ObservationFeedback consumer is observation_feedback_callback',
      any(s['cb'] == 'observation_feedback_callback'
          and s['file'].endswith('frontier_explorer.py') for s in fb.get('subs', [])),
      str(fb.get('subs')))
check('ObservationFeedback publish sites >= 3 (cross-file resolution)',
      len(fb.get('publish_sites', [])) >= 3, str(len(fb.get('publish_sites', []))))

# ---- 6. non-recovery cross-check: LocalMapPatch ----------------------------
patch = topic('grid_map/local_map_patch')
check('LocalMapPatch has producer and consumer',
      bool(patch.get('pubs')) and bool(patch.get('subs')), str(patch.get('subs')))
check('LocalMapPatch consumer callback is patch_callback',
      any(s['cb'] == 'patch_callback' for s in patch.get('subs', [])), str(patch.get('subs')))
check('LocalMapPatch publish site mapped to publisher function',
      any((next((f['name'] for f in D['funcs'] if f['id'] == ps.get('func')), '') ==
           'publishLocalMapPatch') for ps in patch.get('publish_sites', [])),
      str([(p['file'].split('/')[-1], p['line'], p.get('func'))
           for p in patch.get('publish_sites', [])]))

# ---- 7. subscription callback parsing ---------------------------------------
cloud = topic('cloud')
check('std::bind callback resolved to method name (not "bind")',
      any(s['cb'] == 'cloudCallback' for s in cloud.get('subs', [])), str(cloud.get('subs')))

# ---- 8. messages + output file ---------------------------------------------
check('15 msg definitions indexed', D['meta']['n_msgs'] == 15, str(D['meta']['n_msgs']))
check('every msg has fields', all(m['fields'] for m in D['msgs']))
html_path = os.path.join(ROOT, 'docs', 'CONTRACT_STACK_INTERACTIVE_ZH.html')
html = open(html_path, encoding='utf-8').read()
check('html exists and is single-file offline',
      len(html) > 2_000_000 and html.count('<script>') == 1 and 'src="http' not in html,
      f'{len(html)//1024} KB')
check('html embeds snapshot metadata',
      'workspace' in D['meta'] and D['meta']['head'] != '?')
check('total LOC excludes binary assets', D['meta']['total_loc'] < 2_000_000,
      str(D['meta']['total_loc']))

print()
if FAILS:
    print(f'{len(FAILS)} check(s) failed')
    sys.exit(1)
print('all checks passed')
