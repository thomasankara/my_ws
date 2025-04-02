#!/usr/bin/env python3

import os
import time
import threading
import subprocess
import signal
import sys
from datetime import datetime
from tkinter import (
    Tk, Frame, Label, Checkbutton, Button, Entry, StringVar, BooleanVar
)
import shutil  # pour l'espace disque

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Types de messages ROS
from sensor_msgs.msg import PointCloud2, Image, Joy
from tf2_msgs.msg import TFMessage

# --------------------------------------------------------------------
# Liste complète des topics d'intérêt
# --------------------------------------------------------------------
TOPICS_OF_INTEREST = {
    "/lidar_points": PointCloud2,
    "/lidar_points_crop": PointCloud2,
    "/livox/lidar_front": PointCloud2,
    "/livox/lidar_back": PointCloud2,
    "/tf": TFMessage,
    "/tf_static": TFMessage,
    "/cam_1_light": Image,
    "/joy_touchscreen": Joy
}

# Dossier où placer les rosbags
ROSBAG_FOLDER = "/home/thomasraynal/no_ros_project/my_ws/data/data_hive_data_collector"
if not os.path.exists(ROSBAG_FOLDER):
    os.makedirs(ROSBAG_FOLDER)

# --------------------------------------------------------------------
# Node ROS pour surveiller la fréquence des topics + toggle Joy
# --------------------------------------------------------------------
class TopicMonitorNode(Node):
    def __init__(self, topics_dict):
        super().__init__("topic_monitor_node")

        self.topics_info = {}
        for topic_name in topics_dict.keys():
            self.topics_info[topic_name] = {
                "present": False,
                "freq": 0.0,
                "last_msg_time": None,
                "msg_count": 0,
                "timestamps": [],
            }

        # Pour le toggle d’enregistrement via /joy_touchscreen (bouton #1)
        self.last_joy_button1 = 0
        self.record_toggle_count = 0

        # Création des subscriptions
        for topic_name, msg_type in topics_dict.items():
            try:
                self.create_subscription(
                    msg_type,
                    topic_name,
                    lambda msg, t=topic_name: self.topic_callback(msg, t),
                    10
                )
                self.topics_info[topic_name]["present"] = True
            except Exception as e:
                self.get_logger().warn(f"Impossible de s'abonner à {topic_name}: {e}")
                self.topics_info[topic_name]["present"] = False

    def topic_callback(self, msg, topic_name):
        now = time.time()
        info = self.topics_info[topic_name]

        # Calcul de la fréquence
        info["msg_count"] += 1
        info["timestamps"].append(now)
        if len(info["timestamps"]) > 100:
            info["timestamps"].pop(0)

        if len(info["timestamps"]) > 1:
            t_first = info["timestamps"][0]
            t_last = info["timestamps"][-1]
            nb_intervals = len(info["timestamps"]) - 1
            elapsed = t_last - t_first
            if elapsed > 0:
                info["freq"] = nb_intervals / elapsed
            else:
                info["freq"] = 0.0
        else:
            info["freq"] = 0.0

        info["last_msg_time"] = now

        # Gestion du bouton #1 pour le toggle
        if topic_name == "/joy_touchscreen":
            current_button1 = 0
            if len(msg.buttons) > 1:
                current_button1 = msg.buttons[1]
            # Front montant (0->1)
            if current_button1 == 1 and self.last_joy_button1 == 0:
                self.record_toggle_count += 1
            self.last_joy_button1 = current_button1

    def get_topics_info(self):
        return self.topics_info

    def get_record_toggle_count(self):
        return self.record_toggle_count

# --------------------------------------------------------------------
# Interface Tkinter
# --------------------------------------------------------------------
class RosbagRecorderGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("ROS2 Bag Recorder Manager")

        # Node ROS + exécuteur
        self.ros_node = None
        self.executor = None
        self.running = True

        # Variables d'UI
        self.topic_vars = {}
        self.topic_labels_freq = {}
        self.topic_labels_last = {}

        # Enregistrement rosbag
        self.recording_process = None
        self.recording_start_time = None
        self.current_bag_folder = None

        # Crop Node
        self.crop_node_process = None

        # Cam Node
        self.cam_node_process = None

        # Compteur pour le toggle via le Joy
        self.last_record_toggle_count = 0

        # Construction de l'UI
        self._build_ui()

        # Initialisation du Node ROS
        self._init_ros_node()

        # Lancement de la boucle de rafraîchissement
        self.refresh_ui()

    def _build_ui(self):
        # Nom de la capture
        Label(self.master, text="Nom de la capture :").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.bag_name_var = StringVar(value="ma_capture")
        bag_name_entry = Entry(self.master, textvariable=self.bag_name_var)
        bag_name_entry.grid(row=0, column=1, padx=5, pady=5, sticky="w")

        # Frame pour lister les topics
        topics_frame = Frame(self.master)
        topics_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")

        Label(topics_frame, text="Topic").grid(row=0, column=0, padx=5, pady=5)
        Label(topics_frame, text="Inclure").grid(row=0, column=1, padx=5, pady=5)
        Label(topics_frame, text="Fréquence (Hz)").grid(row=0, column=2, padx=5, pady=5)
        Label(topics_frame, text="Dernier msg (s)").grid(row=0, column=3, padx=5, pady=5)

        row_index = 1
        for t in TOPICS_OF_INTEREST.keys():
            Label(topics_frame, text=t).grid(row=row_index, column=0, padx=5, pady=2, sticky="w")
            var = BooleanVar(value=True)
            Checkbutton(topics_frame, variable=var).grid(row=row_index, column=1, padx=5, pady=2)
            self.topic_vars[t] = var

            lbl_freq = Label(topics_frame, text="N/A")
            lbl_freq.grid(row=row_index, column=2, padx=5, pady=2)
            self.topic_labels_freq[t] = lbl_freq

            lbl_last = Label(topics_frame, text="N/A")
            lbl_last.grid(row=row_index, column=3, padx=5, pady=2)
            self.topic_labels_last[t] = lbl_last

            row_index += 1

        # Boutons Start/Stop RECORD
        self.start_button = Button(self.master, text="Démarrer l'enregistrement", command=self.start_recording)
        self.start_button.grid(row=2, column=0, padx=10, pady=10, sticky="e")

        self.stop_button = Button(self.master, text="Arrêter l'enregistrement", command=self.stop_recording, state="disabled")
        self.stop_button.grid(row=2, column=1, padx=10, pady=10, sticky="w")

        # Label de statut
        self.recording_info_label = Label(self.master, text="Statut : Inactif")
        self.recording_info_label.grid(row=3, column=0, columnspan=2, padx=10, pady=5, sticky="w")

        # Boutons Crop Node
        self.start_crop_button = Button(self.master, text="Lancer le Crop Node", command=self.start_crop_node)
        self.start_crop_button.grid(row=4, column=0, padx=10, pady=10, sticky="e")

        self.stop_crop_button = Button(self.master, text="Arrêter le Crop Node", command=self.stop_crop_node, state="disabled")
        self.stop_crop_button.grid(row=4, column=1, padx=10, pady=10, sticky="w")

        self.crop_info_label = Label(self.master, text="Crop Node : Inactif")
        self.crop_info_label.grid(row=5, column=0, columnspan=2, padx=10, pady=5, sticky="w")

        # Boutons Cam Node
        self.start_cam_button = Button(self.master, text="Lancer la Cam Node", command=self.start_cam_node)
        self.start_cam_button.grid(row=6, column=0, padx=10, pady=10, sticky="e")

        self.stop_cam_button = Button(self.master, text="Arrêter la Cam Node", command=self.stop_cam_node, state="disabled")
        self.stop_cam_button.grid(row=6, column=1, padx=10, pady=10, sticky="w")

        self.cam_info_label = Label(self.master, text="Cam Node : Inactif")
        self.cam_info_label.grid(row=7, column=0, columnspan=2, padx=10, pady=5, sticky="w")

        # Espace disque
        self.disk_space_label = Label(self.master, text="Espace disque libre : ???")
        self.disk_space_label.grid(row=8, column=0, columnspan=2, padx=10, pady=5, sticky="w")

    def _init_ros_node(self):
        rclpy.init(args=None)
        self.ros_node = TopicMonitorNode(TOPICS_OF_INTEREST)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.ros_node)

        def spin_ros():
            while self.running:
                self.executor.spin_once(timeout_sec=0.1)

        threading.Thread(target=spin_ros, daemon=True).start()

    # ----------------------------------------------------------------
    # Outils pour Crop/Cam nodes (kill en group)
    # ----------------------------------------------------------------
    def _launch_subprocess_group(self, cmd_str, label):
        print(f"[DEBUG] Launching {label} with: {cmd_str}")
        return subprocess.Popen(
            ["/bin/bash", "-c", cmd_str],
            shell=False,
            executable="/bin/bash",
            preexec_fn=os.setpgrp  # nouveau groupe de process
        )

    def _kill_subprocess_group(self, popen_obj, label):
        if popen_obj is None:
            return
        import signal
        try:
            pgid = os.getpgid(popen_obj.pid)
            os.killpg(pgid, signal.SIGTERM)
        except Exception as e:
            print(f"Erreur kill_subprocess_group (SIGTERM) {label}: {e}")

        for _ in range(10):
            ret = popen_obj.poll()
            if ret is not None:
                break
            time.sleep(0.2)
        else:
            try:
                os.killpg(pgid, signal.SIGKILL)
                popen_obj.wait(timeout=1)
            except Exception as e:
                print(f"Impossible de kill -9 le sous-processus {label}: {e}")

    # ----------------------------------------------------------------
    # Crop Node
    # ----------------------------------------------------------------
    def start_crop_node(self):
        if self.crop_node_process is not None:
            return
        cmd_str = (
            "source /opt/ros/humble/setup.bash && "
            "source ~/hive_ws/install/setup.bash && "
            "ros2 run hive_data_collector lidar_crop_node"
        )
        try:
            self.crop_node_process = self._launch_subprocess_group(cmd_str, "CROP NODE")
            self.crop_info_label.config(text="Crop Node : En cours", fg="green")
            self.start_crop_button.config(state="disabled")
            self.stop_crop_button.config(state="normal")
        except Exception as e:
            self.crop_info_label.config(text=f"Erreur lancement Crop Node : {e}", fg="red")

    def stop_crop_node(self):
        if self.crop_node_process:
            self._kill_subprocess_group(self.crop_node_process, "CROP NODE")
            self.crop_node_process = None
        self.crop_info_label.config(text="Crop Node : Inactif", fg="black")
        self.start_crop_button.config(state="normal")
        self.stop_crop_button.config(state="disabled")

    # ----------------------------------------------------------------
    # Cam Node
    # ----------------------------------------------------------------
    def start_cam_node(self):
        if self.cam_node_process is not None:
            return
        cmd_str = (
            "source /opt/ros/humble/setup.bash && "
            "source ~/hive_ws/install/setup.bash && "
            "ros2 run v4l2_camera v4l2_camera_node "
            "--ros-args -p video_device:=/dev/cam_1 -p output_topic:=/cam_1"
        )
        try:
            self.cam_node_process = self._launch_subprocess_group(cmd_str, "CAM NODE")
            self.cam_info_label.config(text="Cam Node : En cours", fg="green")
            self.start_cam_button.config(state="disabled")
            self.stop_cam_button.config(state="normal")
        except Exception as e:
            self.cam_info_label.config(text=f"Erreur lancement Cam Node : {e}", fg="red")

    def stop_cam_node(self):
        if self.cam_node_process:
            self._kill_subprocess_group(self.cam_node_process, "CAM NODE")
            self.cam_node_process = None
        self.cam_info_label.config(text="Cam Node : Inactif", fg="black")
        self.start_cam_button.config(state="normal")
        self.stop_cam_button.config(state="disabled")

    # ----------------------------------------------------------------
    # Enregistrement rosbag (ancienne méthode)
    # ----------------------------------------------------------------
    def start_recording(self):
        if self.recording_process is not None:
            return  # déjà en cours ?

        selected_topics = [t for t, var in self.topic_vars.items() if var.get()]
        if not selected_topics:
            self.recording_info_label.config(text="Statut : Aucune sélection de topics !")
            return

        bag_name = self.bag_name_var.get().strip() or "ma_capture"
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_name = f"{bag_name}_{timestamp_str}"
        output_path = os.path.join(ROSBAG_FOLDER, folder_name)

        # On construit la commande EXACTEMENT comme dans l'ancien code
        cmd = [
            "ros2", "bag", "record",
            "-o", output_path
        ]
        cmd.extend(selected_topics)

        print(f"[DEBUG] Launch rosbag recording: {cmd}")

        try:
            # On lance la commande sans fancy kill group
            self.recording_process = subprocess.Popen(cmd)
        except Exception as e:
            self.recording_info_label.config(text=f"Erreur lors du start_recording: {e}")
            return

        self.current_bag_folder = output_path
        self.recording_start_time = time.time()

        self.recording_info_label.config(
            fg="red",
            text=f"Statut : Enregistrement en cours dans {output_path} ..."
        )
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")

    def stop_recording(self):
        if self.recording_process is not None:
            # On fait comme l'ancien code : terminate() + wait()
            self.recording_process.terminate()
            self.recording_process.wait()
            self.recording_process = None

        self.recording_info_label.config(fg="black", text="Statut : Enregistrement arrêté")
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.current_bag_folder = None
        self.recording_start_time = None

    def _toggle_recording_from_joy(self):
        if self.recording_process is None:
            self.start_recording()
        else:
            self.stop_recording()

    # ----------------------------------------------------------------
    # Mise à jour périodique UI
    # ----------------------------------------------------------------
    def refresh_ui(self):
        now = time.time()

        # Maj fréquences
        if self.ros_node:
            info = self.ros_node.get_topics_info()
            for topic_name, tinfo in info.items():
                freq_str = f"{tinfo['freq']:.1f}"
                self.topic_labels_freq[topic_name].config(text=freq_str)
                if tinfo["last_msg_time"] is not None:
                    delta = now - tinfo["last_msg_time"]
                    self.topic_labels_last[topic_name].config(text=f"{delta:.1f}s")
                else:
                    self.topic_labels_last[topic_name].config(text="N/A")

        # Afficher taille si enregistrement en cours
        if self.recording_process is not None and self.current_bag_folder is not None:
            duration = time.time() - self.recording_start_time
            size_str = self._get_bag_folder_size_str()
            self.recording_info_label.config(
                text=(f"Statut : Enregistrement en cours - {self.current_bag_folder}\n"
                      f"Durée : {duration:.1f}s - Taille : {size_str}"),
                fg="red"
            )

        # Espace disque
        self._update_disk_space()

        # Crop Node terminé ?
        if self.crop_node_process and self.crop_node_process.poll() is not None:
            self.crop_node_process = None
            self.crop_info_label.config(text="Crop Node : Inactif", fg="black")
            self.start_crop_button.config(state="normal")
            self.stop_crop_button.config(state="disabled")

        # Cam Node terminé ?
        if self.cam_node_process and self.cam_node_process.poll() is not None:
            self.cam_node_process = None
            self.cam_info_label.config(text="Cam Node : Inactif", fg="black")
            self.start_cam_button.config(state="normal")
            self.stop_cam_button.config(state="disabled")

        # Gestion du toggle Joy
        if self.ros_node:
            new_count = self.ros_node.get_record_toggle_count()
            if new_count > self.last_record_toggle_count:
                toggles = new_count - self.last_record_toggle_count
                for _ in range(toggles):
                    self._toggle_recording_from_joy()
                self.last_record_toggle_count = new_count

        self.master.after(1000, self.refresh_ui)

    def _get_bag_folder_size_str(self):
        if not self.current_bag_folder or not os.path.exists(self.current_bag_folder):
            return "0 o"
        total_size = 0
        for root, dirs, files in os.walk(self.current_bag_folder):
            for f in files:
                fp = os.path.join(root, f)
                total_size += os.path.getsize(fp)
        return self._format_size(total_size)

    def _update_disk_space(self):
        total, used, free = shutil.disk_usage("/")
        free_str = self._format_size(free)
        total_str = self._format_size(total)
        self.disk_space_label.config(
            text=f"Espace disque libre : {free_str} / {total_str}"
        )

    def _format_size(self, size_in_bytes):
        if size_in_bytes < 1024:
            return f"{size_in_bytes} o"
        elif size_in_bytes < 1024**2:
            return f"{size_in_bytes/1024:.1f} Kio"
        elif size_in_bytes < 1024**3:
            return f"{size_in_bytes/(1024**2):.1f} Mio"
        else:
            return f"{size_in_bytes/(1024**3):.1f} Gio"

    # ----------------------------------------------------------------
    # Fermeture
    # ----------------------------------------------------------------
    def on_close(self):
        self.running = False

        # Stop recording si en cours
        if self.recording_process:
            self.recording_process.terminate()
            self.recording_process.wait()
            self.recording_process = None

        # Stop Crop
        if self.crop_node_process:
            self._kill_subprocess_group(self.crop_node_process, "CROP NODE")
            self.crop_node_process = None

        # Stop Cam
        if self.cam_node_process:
            self._kill_subprocess_group(self.cam_node_process, "CAM NODE")
            self.cam_node_process = None

        if self.executor:
            self.executor.shutdown()
        rclpy.shutdown()
        self.master.destroy()

# --------------------------------------------------------------------
# main
# --------------------------------------------------------------------
def main():
    def signal_handler(sig, frame):
        app.on_close()
        sys.exit(0)

    root = Tk()
    app = RosbagRecorderGUI(root)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
