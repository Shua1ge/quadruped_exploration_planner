#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render an offline 3D view of the Campus coverage route (Plotly, self-contained).

    python3 tools/coverage_route/visualize_3d.py

Reads walkable.npz / route_coverage.npz / route_on_graph.csv and samples the
collision mesh for context. Writes docs/dyn/coverage/route/route_3d.html.
"""

import os
import sys

import numpy as np
import trimesh
import plotly.graph_objects as go

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
BASE = os.path.join(ROOT, 'docs/dyn/coverage')
OUT = os.path.join(BASE, 'route', 'route_3d.html')
STL = os.path.join(ROOT, 'assets/runtime_worlds/campus/models/campus_runtime/campus_collision.stl')


def main():
    w = np.load(os.path.join(BASE, 'walkable.npz'))
    gz = w['ground_z']
    walk = w['walkable']
    res = float(w['res'])
    origin = w['origin']
    h, wd = walk.shape
    cov = np.load(os.path.join(BASE, 'route', 'route_coverage.npz'))['cov']
    route = np.loadtxt(os.path.join(BASE, 'route', 'route_on_graph.csv'),
                       delimiter=',', skiprows=1)
    ys, xs = np.where(walk)
    x = origin[0] + (xs + 0.5) * res
    y = origin[1] + (ys + 0.5) * res
    z = gz[ys, xs]
    covered = cov[ys, xs]
    n_cov = int(covered.sum())
    n_all = int(walk.sum())
    pct = 100 * n_cov / n_all

    # downsample covered ground for a light WebGL scene
    step = max(1, int(np.ceil(n_cov / 90000)))
    idx = np.where(covered)[0][::step]
    cov_trace = go.Scatter3d(
        x=x[idx], y=y[idx], z=z[idx] + 0.05,
        mode='markers', name=f'已覆盖 {pct:.1f}%',
        marker=dict(size=1.4, color='#2ecc71', opacity=0.75),
        hoverinfo='skip')
    unc = ~covered
    unc_trace = go.Scatter3d(
        x=x[unc], y=y[unc], z=z[unc] + 0.10,
        mode='markers', name=f'未覆盖 {100-pct:.1f}%',
        marker=dict(size=2.0, color='#e74c3c', opacity=0.9),
        hoverinfo='skip')

    # Campus mesh context (sampled); colour by height
    mesh = trimesh.load(STL, process=False)
    pts, _ = trimesh.sample.sample_surface(mesh, 55000)
    mesh_trace = go.Scatter3d(
        x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
        mode='markers', name='Campus 网格',
        marker=dict(size=0.8, color=pts[:, 2], colorscale='Greys',
                    opacity=0.18, showscale=False),
        hoverinfo='skip')

    route_trace = go.Scatter3d(
        x=route[:, 0], y=route[:, 1], z=route[:, 2] + 0.12,
        mode='lines', name='路线 4.70 km',
        line=dict(color='#00e5ff', width=7), hoverinfo='skip')

    g0 = float(gz[int(np.clip((0 - origin[1]) / res, 0, h - 1)),
                  int(np.clip((0 - origin[0]) / res, 0, wd - 1))])
    spawn_trace = go.Scatter3d(
        x=[0], y=[0], z=[g0 + 0.4], mode='markers+text', name='spawn',
        marker=dict(size=6, color='#f1c40f', symbol='diamond'),
        text=['spawn'], textposition='top center',
        textfont=dict(color='#f1c40f', size=11), hoverinfo='skip')

    fig = go.Figure([mesh_trace, cov_trace, unc_trace, route_trace, spawn_trace])
    fig.update_layout(
        title=dict(
            text=(f'Campus 观测路线（离线 oracle） · 路线 4.70 km · '
                  f'严格覆盖 {pct:.1f}% · 0.3 m 容差 98.6%'),
            font=dict(color='#e8eefc', size=15), x=0.02),
        paper_bgcolor='#0b0f17', plot_bgcolor='#0b0f17',
        font=dict(color='#c9d4e6'),
        legend=dict(bgcolor='rgba(16,22,34,0.85)', bordercolor='#2a3547',
                    borderwidth=1, x=0.01, y=0.99),
        scene=dict(
            bgcolor='#0b0f17',
            aspectmode='data',
            xaxis=dict(title='x [m]', gridcolor='#1d2635', zerolinecolor='#2a3547',
                       backgroundcolor='#0b0f17', showbackground=True),
            yaxis=dict(title='y [m]', gridcolor='#1d2635', zerolinecolor='#2a3547',
                       backgroundcolor='#0b0f17', showbackground=True),
            zaxis=dict(title='z [m]', gridcolor='#1d2635', zerolinecolor='#2a3547',
                       backgroundcolor='#0b0f17', showbackground=True),
            camera=dict(eye=dict(x=0.85, y=0.75, z=0.55), center=dict(x=0, y=0, z=-0.05))),
        margin=dict(l=0, r=0, t=45, b=0),
        annotations=[dict(text='绿色=已覆盖 · 红色=未覆盖 · 青色=路线 · 可拖拽旋转/滚轮缩放',
                          xref='paper', yref='paper', x=0.01, y=0.02,
                          showarrow=False, font=dict(color='#8fa3bf', size=11))])
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.write_html(OUT, include_plotlyjs='inline', full_html=True,
                   config={'displayModeBar': True, 'scrollZoom': True},
                   post_script=open(os.path.join(HERE, 'actual_route_viewer.js'),
                                    encoding='utf-8').read())
    print('wrote', OUT, f'({os.path.getsize(OUT)//1024//1024} MB), '
          f'covered={n_cov}/{n_all} ({pct:.1f}%), route points={len(route)}')


if __name__ == '__main__':
    main()
