#!/usr/bin/env python3
"""Read-only motion/handoff report. Uses recorded events.jsonl and actual_route.csv.

Physical stops are XY one-second speed <0.04m/s for >=3s; not FSM HOLD.
Clock gaps/resets split traces. Latencies are wall milliseconds, motion is sim seconds.
Missing telemetry stays missing. No dependencies beyond Python standard library.
"""
import argparse
import bisect
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path


def quantiles(values):
    a=sorted(float(x) for x in values if x is not None and math.isfinite(float(x)))
    def q(p):
        z=(len(a)-1)*p;i=int(z)
        return a[i]+(a[min(i+1,len(a)-1)]-a[i])*(z-i) if a else None
    return dict(samples=len(a),p50=q(.5),p95=q(.95),maximum=max(a) if a else None)


def handoffs(events):
    groups=defaultdict(dict);extra=Counter();states=[]
    for e in events:
        p=e.get('payload',{});topic=e.get('topic')
        if topic=='/planning/execution_state' and isinstance(p,dict):
            states.append((e['receipt_wall_ns']/1e9,p))
        if topic!='/planning/handoff_metrics' or not isinstance(p,dict):continue
        name=p.get('event');key=(p.get('request_id'),p.get('attempt_id'),p.get('trajectory_id'))
        if not key[1]:extra[str(p.get('reason','unknown'))]+=1;continue
        # Recording directories should contain one planner process lifetime.
        prior=groups[key].get(name)
        if prior is None:groups[key][name]=p
        else:extra['duplicate_'+str(name)]+=1
    rows=[]
    for key,g in groups.items():
        a=g.get('ATTEMPT',{});s=g.get('SUBMITTED');ack=g.get('ACK')
        # ATTEMPT may publish after ACK. Group by exact attempt, not receipt order.
        p={**(ack or {}),**(s or {}),**a};r=dict(p)
        r['submitted']=s is not None
        r['activation_observed']=bool(ack and ack.get('ack_status') in (1,4))
        r['installed']=r['submitted'] and r['activation_observed']
        r['timed_out']='TIMEOUT' in g;r['late_ack']='LATE_ACK' in g
        r['ack_reason']=ack.get('reason') if ack else None
        r['ack_status']=ack.get('ack_status') if ack else None
        r['ack_wall_ms']=ack.get('ack_wall_ms') if ack else None
        r['install_total_wall_ms']=ack.get('install_total_wall_ms') if r['installed'] else None
        rows.append(r)
    installed=[r for r in rows if r['installed']];submitted=[r for r in rows if r['submitted']]
    report=dict(available=bool(rows),attempts=len(rows),submitted=len(submitted),installed=len(installed),
        installation_rate=len(installed)/len(submitted) if submitted else None,
        activation_without_submission=sum(r['activation_observed'] and not r['submitted'] for r in rows),
        ack_rejections=dict(Counter(r['ack_reason'] for r in rows if r['ack_status']==2)),
        timed_out=sum(r['timed_out'] for r in rows),late_ack=sum(r['late_ack'] for r in rows),
        unresolved_submissions=sum(r['submitted'] and r['ack_status'] is None and not r['timed_out'] for r in rows),
        submitted_prepare_ms=quantiles(r.get('prepare_wall_ms') for r in submitted),
        installed_prepare_ms=quantiles(r.get('prepare_wall_ms') for r in installed),
        installed_end_to_end_wall_ms=quantiles(r.get('install_total_wall_ms') for r in installed),
        generation_ms=quantiles(r.get('generation_ms') for r in rows if (r.get('generation_ms') or 0)>0),
        generation_not_reached=sum(r.get('generation_ms')==0 for r in rows),
        preparation_failures=dict(Counter(r.get('reason','UNKNOWN') for r in rows if not r['submitted'])),
        rejected_or_failed_prepare_ms=quantiles(r.get('prepare_wall_ms') for r in rows if not r['installed']),
        incomplete_attempts=sum(not r['submitted'] and 'ATTEMPT' not in g for r,g in zip(rows,groups.values())),
        selection_and_recording_events=dict(extra))
    for name,cross in [('same_task',False),('cross_task',True)]:
        subset=[r for r in submitted if r.get('cross_task')==cross]
        success=[r for r in subset if r['installed']]
        report[name]=dict(submitted=len(subset),installed=len(success),installation_rate=len(success)/len(subset) if subset else None,
            installed_end_to_end_wall_ms=quantiles(r.get('install_total_wall_ms') for r in success))
    return report,rows,sorted(states,key=lambda v:v[0])


def motion(samples,states=(),minimum_stop=3.,gap=.75):
    blocks=[];block=[];invalid=0
    for p in samples:
        if not all(math.isfinite(p[k]) for k in ('t','wall','x','y')):invalid+=1;continue
        if block and (p.get('segment',0)!=block[-1].get('segment',0) or
                      not 0<p['t']-block[-1]['t']<=gap):
            blocks.append(block);block=[];invalid+=1
        block.append(p)
    if block:blocks.append(block)
    stops=[];distance=duration=0.;stamps=[x[0] for x in states]
    for b in blocks:
        if len(b)<2:continue
        duration+=b[-1]['t']-b[0]['t'];arc=[0.]
        for a,p in zip(b,b[1:]):
            d=math.hypot(p['x']-a['x'],p['y']-a['y']);arc.append(arc[-1]+d);distance+=d
        times=[p['t'] for p in b];start=None
        for i,p in enumerate(b):
            j=bisect.bisect_right(times,p['t']-1.,0,i+1)-1
            # Arc motion prevents out-and-back being mistaken for stationary.
            still=j>=0 and (arc[i]-arc[j])/(p['t']-b[j]['t'])<.04
            if still and start is None:start=i
            if start is not None and (not still or i==len(b)-1):
                end=i if still else i-1;a=b[start];z=b[end];elapsed=z['t']-a['t']
                if elapsed>=minimum_stop:
                    evidence=Counter();k=bisect.bisect_left(stamps,a['wall'])
                    while k<len(states) and states[k][0]<=z['wall']:
                        evidence[states[k][1].get('reason','UNKNOWN')]+=1;k+=1
                    stops.append(dict(start_sim_s=a['t'],end_sim_s=z['t'],duration_sim_s=elapsed,
                        start_wall_s=a['wall'],end_wall_s=z['wall'],
                        censored=(a['t']-b[0]['t']<=1.25 or end==len(b)-1),
                        observed_reasons=dict(evidence)))
                start=None
    return dict(distance_xy_m=distance,valid_duration_sim_s=duration,
        actual_average_xy_mps=distance/duration if duration else None,
        stops_ge_3s=len(stops),stops_per_100m=len(stops)*100/distance if distance else None,
        stop_duration_sim_s=sum(s['duration_sim_s'] for s in stops),
        censored_stops=sum(s['censored'] for s in stops),invalid_or_discontinuous_samples=invalid,
        definition='1s XY arc speed <0.04m/s, sustained >=3s; gaps >0.75s split; reason is observed evidence, not inferred cause'),stops


def read_route(path,start=0.,end=math.inf):
    with Path(path).open() as f:
        return [dict(t=float(r['elapsed_s']),wall=float(r['wall_time_s']),x=float(r['x']),y=float(r['y']),segment=r.get('segment',0))
            for r in csv.DictReader(f) if start<=float(r['elapsed_s'])<=end]


def write_csv(path,rows):
    keys=sorted(set().union(*(r.keys() for r in rows))) if rows else ['no_samples']
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader()
        for r in rows:w.writerow({k:json.dumps(v) if isinstance(v,(dict,list)) else v for k,v in r.items()})


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--record',type=Path,required=True,help='region_record directory')
    ap.add_argument('--route',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--start',type=float,default=0.,help='route elapsed seconds, excludes get-up if desired')
    ap.add_argument('--end',type=float,default=math.inf)
    args=ap.parse_args();samples=read_route(args.route,args.start,args.end)
    events=[]
    if samples:
        lo=min(s['wall'] for s in samples)*1e9;hi=max(s['wall'] for s in samples)*1e9
        with (args.record/'events.jsonl').open() as f:
            for line in f:
                e=json.loads(line)
                if lo<=e['receipt_wall_ns']<=hi and e.get('topic') in ('/planning/handoff_metrics','/planning/execution_state'):events.append(e)
    h,attempts,states=handoffs(events);m,stops=motion(samples,states)
    args.output.mkdir(parents=True,exist_ok=True)
    result=dict(schema_version=1,record=str(args.record),route=str(args.route),motion=m,handoff=h,
        limitation='Only attempts/events within selected receipt-wall range. Missing old-run handoff telemetry is not reconstructed or zero-filled.')
    (args.output/'summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    write_csv(args.output/'attempts.csv',attempts);write_csv(args.output/'stops.csv',stops)
    print(json.dumps(result,indent=2,ensure_ascii=False))
if __name__=='__main__':main()
