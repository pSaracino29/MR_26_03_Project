#!/usr/bin/env python3
"""
fan_plot.py

Script di POST-PROCESSING (non è un nodo ROS, va lanciato a simulazione
conclusa) che legge il file aruco_observations.csv generato da
aruco_mapper.py e produce:

  - fan_plot.png: per ogni marker ArUco, tutte le osservazioni grezze
    raccolte durante la simulazione, collegate con una linea sottile alla
    posizione media stimata (da qui il nome "fan plot"), più un'ellisse
    a 2 deviazioni standard che rappresenta la dispersione delle stime.
  - marker_summary.csv: tabella riassuntiva con media, deviazione
    standard e numero di osservazioni per ogni marker.

Uso:
    ros2 run ddr_assignments fan_plot
    oppure, direttamente:
    python3 fan_plot.py --input ~/ros_workspace/aruco_observations.csv
"""

import argparse
import csv
import os
from collections import defaultdict

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


def load_observations(csv_path):
    observations = defaultdict(list)
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            marker_id = int(row['marker_id'])
            x = float(row['x'])
            y = float(row['y'])
            observations[marker_id].append((x, y))
    return observations


def compute_stats(observations):
    stats = {}
    for marker_id, pts in observations.items():
        arr = np.array(pts)
        mean = arr.mean(axis=0)
        std = arr.std(axis=0)
        stats[marker_id] = {
            'mean_x': mean[0], 'mean_y': mean[1],
            'std_x': std[0], 'std_y': std[1],
            'n_obs': len(pts),
        }
    return stats


def plot_fan(observations, stats, output_path):
    fig, ax = plt.subplots(figsize=(10, 10))

    cmap = plt.get_cmap('tab10')
    colors = {m_id: cmap(i % 10) for i, m_id in enumerate(sorted(observations.keys()))}

    for marker_id in sorted(observations.keys()):
        pts = np.array(observations[marker_id])
        mean_x = stats[marker_id]['mean_x']
        mean_y = stats[marker_id]['mean_y']
        color = colors[marker_id]

        # "Ventaglio": una linea sottile dalla media a ogni osservazione grezza
        for (ox, oy) in pts:
            ax.plot([mean_x, ox], [mean_y, oy],
                    color=color, alpha=0.25, linewidth=0.8, zorder=1)

        # Osservazioni grezze
        ax.scatter(pts[:, 0], pts[:, 1], s=12, color=color, alpha=0.5, zorder=2)

        # Ellisse di confidenza a 2 deviazioni standard (95% circa, sotto
        # assunzione di rumore gaussiano su x e y indipendenti)
        std_x = stats[marker_id]['std_x']
        std_y = stats[marker_id]['std_y']
        if std_x > 0 and std_y > 0:
            ellipse = Ellipse(
                (mean_x, mean_y), width=4 * std_x, height=4 * std_y,
                edgecolor=color, facecolor='none', linewidth=1.5,
                linestyle='--', zorder=3)
            ax.add_patch(ellipse)

        # Media, ben visibile
        ax.scatter([mean_x], [mean_y], marker='*', s=250,
                   color=color, edgecolor='black', linewidth=1, zorder=4)
        ax.annotate(
            f"ID {marker_id}\nn={stats[marker_id]['n_obs']}\n"
            f"\u03c3x={std_x:.3f}m \u03c3y={std_y:.3f}m",
            (mean_x, mean_y), textcoords="offset points", xytext=(10, 10),
            fontsize=8, weight='bold', color=color)

    ax.set_title("Fan plot — osservazioni ArUco grezze, media e varianza")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.4)

    fig.savefig(output_path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f"Fan plot salvato in: {output_path}")


def save_summary_csv(stats, output_path):
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['marker_id', 'n_obs', 'mean_x', 'mean_y', 'std_x', 'std_y'])
        for marker_id in sorted(stats.keys()):
            s = stats[marker_id]
            writer.writerow([
                marker_id, s['n_obs'],
                f"{s['mean_x']:.4f}", f"{s['mean_y']:.4f}",
                f"{s['std_x']:.4f}", f"{s['std_y']:.4f}"
            ])
    print(f"Riepilogo statistico salvato in: {output_path}")


def main(args=None):
    parser = argparse.ArgumentParser(description="Genera un fan plot dai marker ArUco osservati.")
    parser.add_argument(
        '--input', type=str,
        default=os.path.expanduser('~/ros_workspace/results/aruco_observations.csv'),
        help="Path al CSV generato da aruco_mapper.py")
    parser.add_argument(
        '--output-dir', type=str,
        default=os.path.expanduser('~/ros_workspace/results'),
        help="Cartella dove salvare fan_plot.png e marker_summary.csv")
    parsed = parser.parse_args(args)

    if not os.path.exists(parsed.input):
        print(f"ERRORE: file non trovato: {parsed.input}")
        print("Assicurati di aver lanciato prima aruco_mapper.py durante una simulazione.")
        return

    observations = load_observations(parsed.input)
    if not observations:
        print("Nessuna osservazione trovata nel CSV.")
        return

    stats = compute_stats(observations)

    os.makedirs(parsed.output_dir, exist_ok=True)
    plot_fan(observations, stats, os.path.join(parsed.output_dir, 'fan_plot.png'))
    save_summary_csv(stats, os.path.join(parsed.output_dir, 'marker_summary.csv'))


if __name__ == '__main__':
    main()