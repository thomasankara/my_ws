#!/usr/bin/env python3

import os
import time
import threading
import datetime
import tkinter as tk
from tkinter import ttk, messagebox, StringVar
import numpy as np
import cv2
import shutil

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge

# -- Chemins par défaut (adapter si besoin) --
THIS_WS = os.path.expanduser("~/my_ws")
DATA_FOLDER = os.path.join(THIS_WS, "data")
BIN_FOLDER = os.path.join(DATA_FOLDER, "bin_folder")
PNG_FOLDER = os.path.join(DATA_FOLDER, "png_folder")
INDEX_FILE_PATH = os.path.join(DATA_FOLDER, "frames_index.txt")


class RosbagPreKittiNode(Node):
    def __init__(self):
        super().__init__("rosbag_pre_kitti")

        # === Fenêtre Tkinter ===
        self.root = tk.Tk()
        self.root.title("Rosbag Pre-KITTI")

        # === Variables d'affichage ===
        self.lidar_ts_var = StringVar(self.root, value="N/A")
        self.image_ts_var = StringVar(self.root, value="N/A")

        # === Données reçues ===
        self.last_lidar_points = None
        self.last_image_cv = None

        # Topics par défaut
        self.lidar_topic = "/lidar_points_crop"
        self.image_topic = "/cam_1_light"

        # Mémoires de souscriptions
        self._sub_handles = []

        # Pont ROS -> OpenCV
        self.bridge = CvBridge()

        # === Interface Tkinter ===
        self._build_ui()

        # Création des dossiers de sortie
        os.makedirs(BIN_FOLDER, exist_ok=True)
        os.makedirs(PNG_FOLDER, exist_ok=True)

        # Souscriptions initiales
        self.subscribe_topics()

    def _build_ui(self):
        """Construit l'interface Tkinter"""
        # Ligne 0 : LIDAR
        ttk.Label(self.root, text="Topic LIDAR:").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.lidar_var = StringVar(value=self.lidar_topic)
        ttk.Entry(self.root, textvariable=self.lidar_var, width=30).grid(row=0, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(self.root, text="Dernier TS:").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.lidar_ts_label = ttk.Label(self.root, textvariable=self.lidar_ts_var)
        self.lidar_ts_label.grid(row=0, column=3, padx=5, pady=5, sticky="w")

        # >>> AJOUT : Label alerte "No intensity" en rouge
        # On utilise un tk.Label (pas ttk) pour gérer facilement la couleur fg="red".
        self.lidar_intensity_alert = tk.Label(self.root, text="", fg="red")
        self.lidar_intensity_alert.grid(row=0, column=4, padx=5, pady=5, sticky="w")

        # Ligne 1 : IMAGE
        ttk.Label(self.root, text="Topic Image:").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        self.image_var = StringVar(value=self.image_topic)
        ttk.Entry(self.root, textvariable=self.image_var, width=30).grid(row=1, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(self.root, text="Dernier TS:").grid(row=1, column=2, padx=5, pady=5, sticky="e")
        self.image_ts_label = ttk.Label(self.root, textvariable=self.image_ts_var)
        self.image_ts_label.grid(row=1, column=3, padx=5, pady=5, sticky="w")

        # Ligne 2 : Appliquer Topics
        topics_button = ttk.Button(self.root, text="Appliquer Topics", command=self.set_topics)
        topics_button.grid(row=2, column=0, columnspan=4, padx=5, pady=5)

        # LIGNE 3 : Sauver / Réinitialiser
        save_button = ttk.Button(self.root, text="Sauver la frame courante", command=self.save_frame)
        save_button.grid(row=3, column=0, padx=5, pady=5, sticky="e")

        reset_button = ttk.Button(self.root, text="Réinitialiser (plage)", command=self.reset_data)
        reset_button.grid(row=3, column=1, padx=5, pady=5, sticky="w")

        # Ligne 4 : Générer Index
        index_button = ttk.Button(self.root, text="Générer Index", command=self.generate_index_file)
        index_button.grid(row=4, column=0, columnspan=4, padx=5, pady=5)

        self.root.rowconfigure(5, weight=1)
        self.root.columnconfigure(3, weight=1)

    def subscribe_topics(self):
        """Crée les souscriptions aux topics LIDAR et Image."""
        for sub in self._sub_handles:
            self.destroy_subscription(sub)
        self._sub_handles = []

        self.lidar_topic = self.lidar_var.get().strip()
        self.image_topic = self.image_var.get().strip()

        local_topics = {
            self.lidar_topic: PointCloud2,
            self.image_topic: Image
        }

        for t_name, msg_type in local_topics.items():
            new_sub = self.create_subscription(
                msg_type,
                t_name,
                lambda msg, t=t_name: self.generic_callback(msg, t),
                10
            )
            self._sub_handles.append(new_sub)

        self.get_logger().info(f"Topics mis à jour : LIDAR={self.lidar_topic}, IMAGE={self.image_topic}")

    def generic_callback(self, msg, topic_name):
        if topic_name == self.lidar_topic and isinstance(msg, PointCloud2):
            self.handle_lidar(msg)
        elif topic_name == self.image_topic and isinstance(msg, Image):
            self.handle_image(msg)

    def handle_lidar(self, msg: PointCloud2):
        fields = [f.name for f in msg.fields]
        points_list = []

        # Alerte en rouge si le champ "intensity" n'est pas présent
        if 'intensity' not in fields:
            self.lidar_intensity_alert.config(text="No intensity")
        else:
            self.lidar_intensity_alert.config(text="")  # On retire l'alerte

        # Lecture du nuage
        if 'intensity' in fields:
            for p in point_cloud2.read_points(
                msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
            ):
                points_list.append([p[0], p[1], p[2], p[3]])
        else:
            for p in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            ):
                points_list.append([p[0], p[1], p[2], 0.0])

        if points_list:
            self.last_lidar_points = np.array(points_list, dtype=np.float32)
        else:
            self.last_lidar_points = None

        now = datetime.datetime.now()
        self.lidar_ts_var.set(now.strftime("%Y-%m-%d %H:%M:%S.%f"))

    def handle_image(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.last_image_cv = cv_img
        except Exception as e:
            self.get_logger().error(f"Erreur conversion Image: {e}")
            self.last_image_cv = None

        now = datetime.datetime.now()
        self.image_ts_var.set(now.strftime("%Y-%m-%d %H:%M:%S.%f"))

    def set_topics(self):
        self.subscribe_topics()
        messagebox.showinfo("Infos", "Les topics ont été mis à jour.")

    def save_frame(self):
        if self.last_lidar_points is None:
            messagebox.showerror("Erreur", "Pas de nuage de points disponible.")
            return
        if self.last_image_cv is None:
            messagebox.showerror("Erreur", "Pas d'image disponible.")
            return

        next_index = self.find_next_index()
        index_str = f"{next_index:06d}"

        bin_path = os.path.join(BIN_FOLDER, f"{index_str}.bin")
        png_path = os.path.join(PNG_FOLDER, f"{index_str}.png")

        self.last_lidar_points.tofile(bin_path)
        cv2.imwrite(png_path, self.last_image_cv)

        # On ne montre plus la pop-up 
        # messagebox.showinfo("Infos", f"Frame sauvegardée :\n{bin_path}\n{png_path}")

    def find_next_index(self):
        existing_indices = []
        for fname in os.listdir(BIN_FOLDER):
            if fname.endswith(".bin"):
                base = os.path.splitext(fname)[0]
                try:
                    idx = int(base)
                    existing_indices.append(idx)
                except ValueError:
                    pass
        return (max(existing_indices) + 1) if existing_indices else 1

    def reset_data(self):
        reset_win = tk.Toplevel(self.root)
        reset_win.title("Réinitialiser (plage)")

        tk.Label(reset_win, text="De l'index :").grid(row=0, column=0, padx=5, pady=5)
        start_var = StringVar(value="0")
        tk.Entry(reset_win, textvariable=start_var, width=6).grid(row=0, column=1, padx=5, pady=5)

        tk.Label(reset_win, text="à l'index :").grid(row=1, column=0, padx=5, pady=5)
        end_var = StringVar(value="200")
        tk.Entry(reset_win, textvariable=end_var, width=6).grid(row=1, column=1, padx=5, pady=5)

        def do_reset():
            try:
                start_idx = int(start_var.get())
                end_idx = int(end_var.get())
            except ValueError:
                messagebox.showerror("Erreur", "Les indices doivent être des entiers.")
                return

            resp = messagebox.askyesno("Confirmation", f"Supprimer les frames {start_idx} à {end_idx} ?")
            if not resp:
                return

            deleted_count = 0
            for idx in range(start_idx, end_idx + 1):
                idx_str = f"{idx:06d}"
                bin_file = os.path.join(BIN_FOLDER, f"{idx_str}.bin")
                png_file = os.path.join(PNG_FOLDER, f"{idx_str}.png")

                for path_file in [bin_file, png_file]:
                    if os.path.exists(path_file):
                        os.remove(path_file)
                        deleted_count += 1

            messagebox.showinfo("Infos", f"Fichiers supprimés : {deleted_count}")
            reset_win.destroy()

        tk.Button(reset_win, text="Valider", command=do_reset).grid(row=2, column=0, columnspan=2, padx=5, pady=5)

    def generate_index_file(self):
        all_indices = []
        for fname in os.listdir(BIN_FOLDER):
            if fname.endswith(".bin"):
                base = os.path.splitext(fname)[0]
                try:
                    idx = int(base)
                    all_indices.append(idx)
                except ValueError:
                    pass
        all_indices.sort()

        with open(INDEX_FILE_PATH, "w") as f:
            for i in all_indices:
                f.write(f"{i:06d}\n")

        messagebox.showinfo("Infos", f"Index généré dans :\n{INDEX_FILE_PATH}")

def main(args=None):
    rclpy.init(args=args)
    node = RosbagPreKittiNode()

    # Exécuteur multi-thread (fil d'exécution distinct pour ros2 spin)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    def spin_thread():
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)

    t = threading.Thread(target=spin_thread, daemon=True)
    t.start()

    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
