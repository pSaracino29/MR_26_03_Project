#!/usr/bin/env python3

import csv
import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ros_gz_interfaces.msg import Contacts


class SimulationSupervisor(Node):

    def __init__(self):
        super().__init__('sim_supervisor')

        self.declare_parameter('max_runs', 20)
        self.declare_parameter('robot_name', 'ddr')
        self.declare_parameter('ignore_names', ['ground_plane'])
        # secondi (tempo reale) dopo l'avvio del launch in cui i contatti vengono ignorati
        self.declare_parameter('grace_period_s', 5.0)

        self.max_runs = self.get_parameter('max_runs').value
        self.robot_name = self.get_parameter('robot_name').value
        self.ignore_names = list(self.get_parameter('ignore_names').value)
        self.grace_period = self.get_parameter('grace_period_s').value

        self.base_dir = Path(os.path.expanduser('~/ros_workspace/results'))
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.summary_path = self.base_dir / 'summary.csv'

        self.sim_process = None
        self.current_run_dir = None
        self.run_start_time = 0.0

        self.lock = threading.Lock()
        self.restarting = False      # True mentre la run corrente e' in chiusura

        self.runs_done = 0           # run completate in questa sessione
        self.results = []            # lista di bool (success) di questa sessione

        self.run_number = self.get_next_run_number()

        self.create_subscription(
            Bool, '/exploration_complete',
            self.exploration_complete_callback, 10)
        self.create_subscription(
            Contacts, '/chassisContact',
            self.chassis_contact_callback, 10)

        self.get_logger().info(
            f"Simulation Supervisor avviato (max_runs={self.max_runs}).")
        self.start_simulation()

    # ==========================================================
    # GESTIONE RUN
    # ==========================================================

    def get_next_run_number(self):
        numbers = []
        for directory in self.base_dir.glob('run_*'):
            if not directory.is_dir():
                continue
            try:
                numbers.append(int(directory.name.split('_')[1]))
            except (IndexError, ValueError):
                pass
        return max(numbers) + 1 if numbers else 1

    def create_run_directory(self):
        run_dir = self.base_dir / f'run_{self.run_number:03d}'
        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / 'map').mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Directory della run: {run_dir}")
        return run_dir

    # ==========================================================
    # AVVIO SIMULAZIONE
    # ==========================================================

    def start_simulation(self):
        run_dir = self.create_run_directory()
        self.current_run_dir = run_dir
        run_id = f'{self.run_number:03d}'

        env = os.environ.copy()
        env['EXPLORATION_RUN_DIR'] = str(run_dir)
        env['EXPLORATION_RUN_ID'] = run_id

        self.get_logger().info(
            f"Avvio RUN {run_id} ({self.runs_done + 1}/{self.max_runs})")

        try:
            self.run_start_time = time.monotonic()
            self.sim_process = subprocess.Popen(
                ['ros2', 'launch', 'ddr_assignments', 'assignments.launch.py'],
                env=env,
                start_new_session=True
            )
            self.get_logger().info(
                f"assignments.launch.py avviato (PID={self.sim_process.pid})")
        except Exception as e:
            self.get_logger().error(f"Errore nell'avvio della simulazione: {e}")

    # ==========================================================
    # FINE RUN (successo o collisione)
    # ==========================================================

    def finish_run(self, success, reason, collided_with=''):
        """Punto unico di chiusura run. Thread-safe: solo la prima chiamata vince."""
        with self.lock:
            if self.restarting:
                return
            self.restarting = True

        duration = time.monotonic() - self.run_start_time
        self.results.append(success)
        self.write_result(success, reason, collided_with, duration)

        threading.Thread(target=self.restart_simulation, daemon=True).start()

    def exploration_complete_callback(self, msg):
        if not msg.data:
            return
        self.get_logger().info("Ricevuto /exploration_complete = True")
        self.finish_run(True, 'exploration_complete')

    def chassis_contact_callback(self, msg):
        if self.restarting or not msg.contacts:
            return

        # ignora i contatti nella fase di avvio/assestamento
        if time.monotonic() - self.run_start_time < self.grace_period:
            return

        for c in msg.contacts:
            other = self.other_entity(c)
            if other is None:
                continue
            if any(k in other for k in self.ignore_names):
                continue

            self.get_logger().error(f"COLLISIONE con '{other}'. Chiudo la run.")
            self.finish_run(False, 'collision', other)
            return

    def other_entity(self, contact):
        """Nome dell'entita' che NON appartiene al robot."""
        for e in (contact.collision1, contact.collision2):
            if not e.name.startswith(self.robot_name + '::'):
                return e.name
        return None

    # ==========================================================
    # SALVATAGGIO RISULTATO
    # ==========================================================

    def write_result(self, success, reason, collided_with, duration):
        result = {
            'run_id': f'{self.run_number:03d}',
            'success_simulation': success,
            'reason': reason,
            'collided_with': collided_with,
            'duration_s': round(duration, 1),
        }

        try:
            with open(self.current_run_dir / 'result.json', 'w') as f:
                json.dump(result, f, indent=2)

            write_header = not self.summary_path.exists()
            with open(self.summary_path, 'a', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(result.keys()))
                if write_header:
                    w.writeheader()
                w.writerow(result)

            self.get_logger().info(f"Risultato run salvato: {result}")
        except Exception as e:
            self.get_logger().error(f"Errore nel salvataggio del risultato: {e}")

    # ==========================================================
    # CHIUSURA SIMULAZIONE
    # ==========================================================

    def stop_simulation(self):
        if self.sim_process is None:
            return

        if self.sim_process.poll() is not None:
            self.get_logger().info("La simulazione è già terminata.")
            self.sim_process = None
            return

        try:
            self.get_logger().info("Invio SIGINT alla simulazione...")
            os.killpg(os.getpgid(self.sim_process.pid), signal.SIGINT)
        except ProcessLookupError:
            self.sim_process = None
            return

        timeout = 20.0
        start = time.time()
        while self.sim_process.poll() is None:
            if time.time() - start > timeout:
                self.get_logger().warn(
                    "La simulazione non si è chiusa entro il timeout.")
                try:
                    os.killpg(os.getpgid(self.sim_process.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass
                break
            time.sleep(0.2)

        self.get_logger().info("Simulazione terminata.")
        self.sim_process = None

    # ==========================================================
    # RESTART / FINE CAMPAGNA
    # ==========================================================

    def restart_simulation(self):
        try:
            self.stop_simulation()
            self.runs_done += 1

            if self.runs_done >= self.max_runs:
                self.print_final_summary()
                rclpy.shutdown()   # fa uscire lo spin() nel main
                return

            time.sleep(10.0)
            self.run_number += 1
            self.get_logger().info(f"Riavvio con RUN {self.run_number:03d}")
            self.start_simulation()

        except Exception as e:
            self.get_logger().error(f"Errore durante il restart: {e}")

        finally:
            self.restarting = False

    def print_final_summary(self):
        n = len(self.results)
        ok = sum(self.results)
        rate = 100.0 * ok / n if n else 0.0
        self.get_logger().info(
            f"=== CAMPAGNA COMPLETATA: {ok}/{n} run riuscite "
            f"(success rate {rate:.1f}%) === Dettagli in {self.summary_path}")

    # ==========================================================
    # SHUTDOWN DEL SUPERVISOR
    # ==========================================================

    def destroy_node(self):
        self.stop_simulation()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimulationSupervisor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()