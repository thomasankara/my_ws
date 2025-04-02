#!/usr/bin/env python3

import os
import time
import threading
import subprocess
from datetime import datetime
from tkinter import (
    Tk, Frame, Label, Checkbutton, Button, Entry, StringVar, BooleanVar
)
import shutil  # pour l'espace disque

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Types de messages ROS (adaptez selon vos besoins exacts)
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
# Node ROS pour surveiller la fréquence des topics
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

        info["msg_count"] += 1
        info["timestamps"].append(now)
        # on limite à 100 timestamps
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

    def get_topics_info(self):
        return self.topics_info

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

        # Enregistrement
        self.recording_process = None
        self.recording_start_time = None
        self.current_bag_folder = None

        # Construction de l'UI
        self._build_ui()

        # Initialisation du Node ROS
        self._init_ros_node()

        # Lancement de la boucle de rafraîchissement
        self.refresh_ui()

    def _build_ui(self):
        Label(self.master, text="Nom de la capture :").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.bag_name_var = StringVar()
        self.bag_name_var.set("ma_capture")
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

            var = BooleanVar(value=True)  # coché par défaut
            Checkbutton(topics_frame, variable=var).grid(row=row_index, column=1, padx=5, pady=2)
            self.topic_vars[t] = var

            lbl_freq = Label(topics_frame, text="N/A")
            lbl_freq.grid(row=row_index, column=2, padx=5, pady=2)
            self.topic_labels_freq[t] = lbl_freq

            lbl_last = Label(topics_frame, text="N/A")
            lbl_last.grid(row=row_index, column=3, padx=5, pady=2)
            self.topic_labels_last[t] = lbl_last

            row_index += 1

        # Boutons Start/Stop
        self.start_button = Button(self.master, text="Démarrer l'enregistrement", command=self.start_recording)
        self.start_button.grid(row=2, column=0, padx=10, pady=10, sticky="e")

        self.stop_button = Button(self.master, text="Arrêter l'enregistrement", command=self.stop_recording, state="disabled")
        self.stop_button.grid(row=2, column=1, padx=10, pady=10, sticky="w")

        # Label de statut
        self.recording_info_label = Label(self.master, text="Statut : Inactif")
        self.recording_info_label.grid(row=3, column=0, columnspan=2, padx=10, pady=5, sticky="w")

        # Espace disque
        self.disk_space_label = Label(self.master, text="Espace disque libre : ???")
        self.disk_space_label.grid(row=4, column=0, columnspan=2, padx=10, pady=5, sticky="w")

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
    # Gestion de l'enregistrement rosbag
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

        cmd = [
            "ros2", "bag", "record",
            "-o", output_path
        ]
        cmd.extend(selected_topics)

        try:
            self.recording_process = subprocess.Popen(cmd)
        except Exception as e:
            self.recording_info_label.config(text=f"Erreur lors du start_recording: {e}")
            return

        self.current_bag_folder = output_path
        self.recording_start_time = time.time()

        self.recording_info_label.config(fg="red", text=f"Statut : Enregistrement en cours dans {output_path} ...")
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")

    def stop_recording(self):
        if self.recording_process is not None:
            self.recording_process.terminate()
            self.recording_process.wait()
            self.recording_process = None

        self.recording_info_label.config(fg="black", text="Statut : Enregistrement arrêté")
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.current_bag_folder = None
        self.recording_start_time = None

    # ----------------------------------------------------------------
    # Mise à jour périodique UI
    # ----------------------------------------------------------------
    def refresh_ui(self):
        # Met à jour fréquences / dernier message
        if self.ros_node is not None:
            now = time.time()
            for topic_name, info in self.ros_node.get_topics_info().items():
                freq_str = f"{info['freq']:.1f}"
                self.topic_labels_freq[topic_name].config(text=freq_str)

                if info["last_msg_time"] is not None:
                    last_delta = now - info["last_msg_time"]
                    self.topic_labels_last[topic_name].config(text=f"{last_delta:.1f}s")
                else:
                    self.topic_labels_last[topic_name].config(text="N/A")

        # Si enregistrement en cours, on met à jour la durée et la taille
        if self.recording_process is not None and self.current_bag_folder is not None:
            duration = time.time() - self.recording_start_time
            size_str = self._get_bag_folder_size_str()
            self.recording_info_label.config(
                text=(f"Statut : Enregistrement en cours - {self.current_bag_folder}\n"
                      f"Durée : {duration:.1f}s - Taille : {size_str}"),
                fg="red"
            )

        # Espace disque restant
        self._update_disk_space()

        # rafraîchit toutes les 1 secondes
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
        if self.recording_process is not None:
            self.recording_process.terminate()
            self.recording_process.wait()
        if self.executor is not None:
            self.executor.shutdown()
        rclpy.shutdown()
        self.master.destroy()

# --------------------------------------------------------------------
# main
# --------------------------------------------------------------------
def main():
    root = Tk()
    app = RosbagRecorderGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
