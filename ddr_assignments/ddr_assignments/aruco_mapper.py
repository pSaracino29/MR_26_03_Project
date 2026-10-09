#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import csv

from geometry_msgs.msg import PoseStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
import tf2_ros
import tf2_geometry_msgs
from rclpy.executors import ExternalShutdownException
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np


class ArucoMapperNode(Node):
    def __init__(self):
        super().__init__('aruco_mapper')

        self.markers_pub = self.create_publisher(MarkerArray, '/detected_markers', 10)

        # Buffer e Listener per leggere le trasformazioni globali
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Dizionario per elencare i dati con le osservazioni
        self.marker_observations = {}

        # --- Setup CSV: una riga per OGNI osservazione, non solo la prima ---
        # workspace_dir = os.path.expanduser("~/ros_workspace/results")
        # os.makedirs(workspace_dir, exist_ok=True)
        # self.csv_path = os.path.join(workspace_dir, "aruco_observations.csv")

        run_dir = os.environ.get('EXPLORATION_RUN_DIR', os.path.expanduser('~/ros_workspace/results'))
        os.makedirs(run_dir, exist_ok=True)
        self.csv_path = os.path.join(run_dir, 'aruco_observations.csv')

        # header scritto solo se il file non esiste già, per non duplicarlo
        # tra run diverse
        write_header = not os.path.exists(self.csv_path)
        self.csv_file = open(self.csv_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if write_header:
            self.csv_writer.writerow([
                'stamp_sec', 'stamp_nanosec', 'marker_id','x', 'y', 'z'])
            self.csv_file.flush()

        self.subscription = self.create_subscription(
            ArucoMarkers, '/aruco_markers', self.marker_callback, 10)

        self.get_logger().info("Nodo Aruco Mapper avviato. In attesa di marker...")
        self.get_logger().info(f"Log osservazioni CSV: {self.csv_path}")

    def marker_callback(self, msg):
        for i, marker_id in enumerate(msg.marker_ids):

            pose_cam = PoseStamped()
            pose_cam.header = msg.header
            pose_cam.pose = msg.poses[i]

            try:
                # Trasformiamo OGNI osservazione, non solo la prima volta che
                # vediamo un ID: servono tutte per calcolare media e varianza
                pose_map = self.tf_buffer.transform(pose_cam, 'map')
                p = pose_map.pose.position

                # --- Log su CSV, riga per riga, subito (robusto a crash) ---
                self.csv_writer.writerow([
                    msg.header.stamp.sec, msg.header.stamp.nanosec, int(marker_id), p.x, p.y, p.z])
                self.csv_file.flush()

                # --- Accumula l'osservazione in memoria ---
                if marker_id not in self.marker_observations:
                    self.marker_observations[marker_id] = []
                    self.get_logger().info(f"+++ Nuovo marker ID {marker_id} rilevato! +++")
                self.marker_observations[marker_id].append((p.x, p.y, p.z))

                # Ripubblica su /detected_markers usando la MEDIA di tutte
                # le osservazioni raccolte finora, non l'ultima singola stima:
                # è una fusione semplice che riduce il rumore di detection
                self.publish_markers()

            except tf2_ros.TransformException as ex:
                self.get_logger().debug(
                    f"In attesa del frame /map per mappare l'ID {marker_id}: {ex}")

    def publish_markers(self):
        marker_array = MarkerArray()
        for m_id, observations in self.marker_observations.items():
            pts = np.array(observations)
            mean_pos = pts.mean(axis=0)

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = int(m_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(mean_pos[0])
            marker.pose.position.y = float(mean_pos[1])
            marker.pose.position.z = float(mean_pos[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        self.markers_pub.publish(marker_array)

    def save_markers_to_file(self):
        if not self.marker_observations:
            self.get_logger().info("Nessun marker rilevato. File non creato.")
            return

        # workspace_dir = os.path.expanduser("~/ros_workspace/results")
        # os.makedirs(workspace_dir, exist_ok=True)
        # filename = os.path.join(workspace_dir, "aruco_mappati.txt")

        run_dir = os.environ.get('EXPLORATION_RUN_DIR', os.path.expanduser('~/ros_workspace/results'))
        os.makedirs(run_dir, exist_ok=True)
        filename = os.path.join(run_dir, 'aruco_mappati.txt')

        with open(filename, 'w') as f:
            f.write("LISTA MARKER ARUCO RILEVATI (media di tutte le osservazioni)\n")
            f.write("Sistema di riferimento: /map\n")
            f.write("=" * 60 + "\n\n")

            for m_id, observations in sorted(self.marker_observations.items()):
                pts = np.array(observations)
                mean = pts.mean(axis=0)
                std = pts.std(axis=0)

                f.write(f"ID MARKER: {m_id}\n")
                f.write(f"  Osservazioni totali: {len(observations)}\n")
                f.write(f"  Posizione media [x, y, z]: "
                        f"[{mean[0]:.3f}, {mean[1]:.3f}, {mean[2]:.3f}] metri\n")
                f.write(f"  Deviazione standard [x, y, z]: "
                        f"[{std[0]:.3f}, {std[1]:.3f}, {std[2]:.3f}] metri\n")
                f.write("-" * 60 + "\n")

        self.get_logger().info(
            f"Salvataggio completato! Trovati {len(self.marker_observations)} marker.")
        self.get_logger().info(f"File generato in: {filename}")

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interruzione da terminale (Ctrl+C). Scrittura in corso...")
    except ExternalShutdownException:
        node.get_logger().info("Spegnimento dal sistema di Launch. Scrittura in corso...")
    finally:
        node.save_markers_to_file()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()