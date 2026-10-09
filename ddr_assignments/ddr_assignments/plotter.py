#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import csv
import os


class ExplorationPlotter(Node):
    def __init__(self):
        super().__init__('exploration_plotter')

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.time_data = []
        self.area_data = []

        self.start_time = None
        self.last_log_time = 0.0

        run_dir = os.environ.get('EXPLORATION_RUN_DIR', os.path.expanduser('~/ros_workspace/results'))
        os.makedirs(run_dir, exist_ok=True)
        self.csv_path = os.path.join(run_dir, 'exploration_progress.csv')

        write_header = not os.path.exists(self.csv_path)
        self.csv_file = open(self.csv_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if write_header:
            self.csv_writer.writerow(['time_s', 'explored_area_m2'])
            self.csv_file.flush()

        self.get_logger().info("Nodo plotter avviato. In attesa della mappa...")
        self.get_logger().info(f"Log progressione CSV: {self.csv_path}")
        self.get_logger().info("Premi Ctrl+C alla fine dell'esplorazione per generare il grafico.")

    def map_callback(self, msg):
        if self.start_time is None:
            self.start_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

        current_time = (msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)) - self.start_time

        grid = np.array(msg.data)
        explored_cells = np.sum(grid != -1)
        area_m2 = explored_cells * (msg.info.resolution ** 2)

        self.time_data.append(current_time)
        self.area_data.append(area_m2)

        self.csv_writer.writerow([f"{current_time:.3f}", f"{area_m2:.4f}"])
        self.csv_file.flush()

        if current_time - self.last_log_time >= 5.0:
            self.get_logger().info(f"T={current_time:.1f}s | Area esplorata: {area_m2:.2f} m²")
            self.last_log_time = current_time

    def generate_plot(self):
        if not self.time_data:
            self.get_logger().warn("Nessun dato raccolto. Grafico non generato.")
            return

        self.get_logger().info("Generazione del grafico in corso...")

        plt.figure(figsize=(10, 6))
        plt.plot(self.time_data, self.area_data, '-', color='b', linewidth=2)
        plt.title('Autonomous Exploration Progress', fontsize=14, fontweight='bold')
        plt.xlabel('Simulation Time (s)', fontsize=12)
        plt.ylabel('Explored Area (m²)', fontsize=12)
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.fill_between(self.time_data, self.area_data, color='b', alpha=0.1)
        
        run_dir = os.environ.get('EXPLORATION_RUN_DIR', os.path.expanduser('~/ros_workspace/results'))
        filename = os.path.join(run_dir, 'exploration_plot.png')

        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()

        self.get_logger().info(f"Grafico salvato con successo: {filename}")

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.generate_plot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()