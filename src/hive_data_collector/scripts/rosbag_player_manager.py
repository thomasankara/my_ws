#!/usr/bin/env python3

import os
import time
import shutil
import subprocess
from datetime import datetime
import tkinter as tk
from tkinter import ttk, messagebox, StringVar, Toplevel
import re

# Dossier où sont stockés vos rosbags
ROSBAG_FOLDER = "/home/thomasraynal/no_ros_project/my_ws/data/data_hive_data_collector"

class RosbagPlayerGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("ROS2 Bag Player Manager")

        # -- Contiendra toutes les infos sur les bags (nom, date, poids, nb_topics, etc.)
        self.bag_data = []

        # ----- Zone de frame pour le Treeview et la scrollbar -----
        frame = ttk.Frame(master)
        frame.grid(row=0, column=0, columnspan=4, padx=10, pady=5, sticky="nsew")

        # On configure la grille pour que le Treeview soit extensible
        self.master.rowconfigure(0, weight=1)
        self.master.columnconfigure(0, weight=1)

        # -- Treeview à plusieurs colonnes --
        self.bag_tree = ttk.Treeview(frame, columns=("col_name", "col_date", "col_size", "col_topics"), show='headings', height=15)
        self.bag_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Définition des en-têtes
        self.bag_tree.heading("col_name", text="Nom")
        self.bag_tree.heading("col_date", text="Date")
        self.bag_tree.heading("col_size", text="Poids (Mo)")
        self.bag_tree.heading("col_topics", text="Nb Topics")

        # Définition des largeurs de colonnes
        self.bag_tree.column("col_name", width=200)
        self.bag_tree.column("col_date", width=120)
        self.bag_tree.column("col_size", width=80, anchor=tk.E)
        self.bag_tree.column("col_topics", width=80, anchor=tk.E)

        # -- On ajoute une scrollbar verticale si besoin --
        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=self.bag_tree.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.bag_tree.configure(yscrollcommand=scrollbar.set)

        # -- On rend chaque colonne triable en cliquant sur les en-têtes --
        self.bag_tree.heading("col_name", text="Nom (cliquez pour trier)", command=lambda: self.sort_by("col_name"))
        self.bag_tree.heading("col_date", text="Date (cliquez pour trier)", command=lambda: self.sort_by("col_date"))
        self.bag_tree.heading("col_size", text="Poids (Mo) (cliquez pour trier)", command=lambda: self.sort_by("col_size", numeric=True))
        self.bag_tree.heading("col_topics", text="Nb Topics (cliquez pour trier)", command=lambda: self.sort_by("col_topics", numeric=True))

        # ----- Boutons -----
        self.refresh_button = ttk.Button(master, text="Rafraîchir la liste", command=self.refresh_bags)
        self.refresh_button.grid(row=1, column=0, padx=5, pady=5, sticky="w")

        self.info_button = ttk.Button(master, text="Infos sur le bag", command=self.show_bag_info)
        self.info_button.grid(row=1, column=1, padx=5, pady=5, sticky="w")

        self.delete_button = ttk.Button(master, text="Supprimer", command=self.delete_bag)
        self.delete_button.grid(row=1, column=2, padx=5, pady=5, sticky="e")

        # ----- Labels + champs : Taux et offset -----
        ttk.Label(master, text="Taux (rate) :").grid(row=2, column=0, padx=5, pady=5, sticky="e")
        self.rate_var = StringVar(value="1.0")
        self.rate_entry = ttk.Entry(master, textvariable=self.rate_var, width=8)
        self.rate_entry.grid(row=2, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(master, text="Start offset (s) :").grid(row=3, column=0, padx=5, pady=5, sticky="e")
        self.offset_var = StringVar(value="0")
        self.offset_entry = ttk.Entry(master, textvariable=self.offset_var, width=8)
        self.offset_entry.grid(row=3, column=1, padx=5, pady=5, sticky="w")

        self.play_button = ttk.Button(master, text="Lancer la lecture", command=self.play_bag)
        self.play_button.grid(row=4, column=0, padx=5, pady=5, sticky="e")

        self.stop_button = ttk.Button(master, text="Arrêter la lecture", command=self.stop_bag, state="disabled")
        self.stop_button.grid(row=4, column=1, padx=5, pady=5, sticky="w")

        # -- Label d'info sur la lecture en cours --
        self.playback_info_label = ttk.Label(master, text="Aucun bag en lecture")
        self.playback_info_label.grid(row=5, column=0, columnspan=4, padx=5, pady=5, sticky="w")

        # Gestion du processus de lecture
        self.play_process = None
        self.play_start_time = None
        self.current_bag_path = None
        self.current_rate = 1.0
        self.current_offset = 0.0

        # Au démarrage, on rafraîchit la liste et on lance la boucle d'UI
        self.refresh_bags()
        self.refresh_ui()

    def refresh_bags(self):
        """
        Scanne le dossier ROSBAG_FOLDER pour lister les sous-dossiers,
        calcule pour chacun :
          - son nom
          - la date extraite du pattern (si existant)
          - son poids total en Mo
          - le nombre de topics
        Puis les insère dans le Treeview.
        """
        self.bag_tree.delete(*self.bag_tree.get_children())
        self.bag_data.clear()

        if not os.path.exists(ROSBAG_FOLDER):
            return

        for name in os.listdir(ROSBAG_FOLDER):
            full_path = os.path.join(ROSBAG_FOLDER, name)
            if os.path.isdir(full_path):
                meta_path = os.path.join(full_path, "metadata.yaml")
                if not os.path.exists(meta_path):
                    continue  # On ignore si pas de metadata.yaml

                # -- Tente d'extraire la date depuis le pattern "_YYYYMMDD_HHMMSS" --
                date_extracted = self.extract_date_from_name(name)

                # -- Calcule le poids du dossier en Mo --
                size_mb = self.get_folder_size_mb(full_path)

                # -- Récupère le nombre de topics (via ros2 bag info, ou en parsant metadata.yaml) --
                nb_topics = self.get_bag_topic_count(full_path)

                self.bag_data.append({
                    "name": name,
                    "date": date_extracted,
                    "size": size_mb,
                    "topics": nb_topics,
                    "path": full_path
                })

        # On insère maintenant dans le Treeview
        for item in self.bag_data:
            # On formate la date si elle est connue, sinon on met un vide
            date_str = item["date"].strftime("%Y-%m-%d %H:%M:%S") if item["date"] else ""
            self.bag_tree.insert(
                "", tk.END,
                values=(
                    item["name"],
                    date_str,
                    f"{item['size']:.1f}",  # 1 décimale, par ex
                    item["topics"]
                )
            )

    def extract_date_from_name(self, bag_name):
        """
        Tente de trouver un pattern du type '_YYYYMMDD_HHMMSS' dans le nom.
        Exemple : 'gros_20250402_185131' => 2025-04-02 18:51:31
        Retourne un objet datetime ou None si pas trouvé.
        """
        pattern = r".*_(\d{8}_\d{6})$"
        match = re.match(pattern, bag_name)
        if match:
            date_str = match.group(1)  # ex: "20250402_185131"
            try:
                return datetime.strptime(date_str, "%Y%m%d_%H%M%S")
            except ValueError:
                return None
        return None

    def get_folder_size_mb(self, folder_path):
        """
        Calcule le poids total (en Mo) d'un dossier et de son contenu.
        """
        total_size = 0
        for dirpath, dirnames, filenames in os.walk(folder_path):
            for f in filenames:
                fp = os.path.join(dirpath, f)
                if os.path.isfile(fp):
                    total_size += os.path.getsize(fp)
        return total_size / (1024 * 1024.0)

    def get_bag_topic_count(self, bag_path):
        """
        Récupère le nombre de topics d'un rosbag (via 'ros2 bag info' par exemple).
        Pour éviter de parser du YAML, on appelle ros2 bag info et on compte le nombre
        de lignes 'Topic: ...'.
        """
        try:
            out = subprocess.check_output(["ros2", "bag", "info", bag_path], text=True)
            # Compter le nombre de fois que 'Topic:' apparaît en début de ligne
            count = 0
            for line in out.splitlines():
                if line.strip().startswith("Topic:"):
                    count += 1
            return count
        except Exception:
            return 0

    def sort_by(self, column_name, numeric=False):
        """
        Trie self.bag_data suivant la colonne demandée, puis ré-affiche le Treeview.
        'column_name' est parmi 'col_name', 'col_date', 'col_size', 'col_topics'.
        """
        # On mappe le nom de colonne du treeview à la clé dans self.bag_data
        column_map = {
            "col_name": "name",
            "col_date": "date",
            "col_size": "size",
            "col_topics": "topics"
        }
        key_name = column_map[column_name]

        # On inverse le tri si on reclique sur la même colonne
        # (On peut par exemple se souvenir du dernier tri...)
        # Pour faire simple, on va juste trier par ordre croissant, 
        # à vous d'améliorer pour un "toggle" croissant/décroissant si besoin.
        if numeric:
            # Pour les colonnes numériques (size, topics), on trie par float/int
            self.bag_data.sort(key=lambda x: x[key_name])
        else:
            # Pour le nom, on trie par string
            # Pour la date, si None on met en fin
            if key_name == "date":
                self.bag_data.sort(key=lambda x: (x[key_name] is None, x[key_name]))
            else:
                self.bag_data.sort(key=lambda x: x[key_name])

        # -- On réactualise l'affichage dans le Treeview --
        self.bag_tree.delete(*self.bag_tree.get_children())
        for item in self.bag_data:
            date_str = item["date"].strftime("%Y-%m-%d %H:%M:%S") if item["date"] else ""
            self.bag_tree.insert(
                "", tk.END,
                values=(
                    item["name"],
                    date_str,
                    f"{item['size']:.1f}",
                    item["topics"]
                )
            )

    def show_bag_info(self):
        """Affiche dans une fenêtre pop-up les infos du bag sélectionné."""
        sel = self.bag_tree.selection()
        if not sel:
            messagebox.showinfo("Info bag", "Veuillez sélectionner un bag.")
            return

        # Récupération de l'item sélectionné
        item_values = self.bag_tree.item(sel[0], 'values')
        # La première colonne, c'est le nom
        bag_name = item_values[0]
        # Retrouver le bag_data correspondant
        bag_dict = next((b for b in self.bag_data if b["name"] == bag_name), None)
        if not bag_dict:
            messagebox.showwarning("Info bag", "Impossible de retrouver les infos du bag.")
            return

        full_path = bag_dict["path"]
        try:
            out = subprocess.check_output(["ros2", "bag", "info", full_path], text=True)
        except Exception as e:
            messagebox.showerror("Erreur", f"Impossible de récupérer les infos : {e}")
            return

        info_win = Toplevel(self.master)
        info_win.title(f"Infos sur {bag_name}")
        ttk.Label(info_win, text=out, justify="left", anchor="nw").pack(padx=10, pady=10, fill="both", expand=True)

    def play_bag(self):
        """Lance la lecture du bag sélectionné avec le rate et l'offset indiqués."""
        sel = self.bag_tree.selection()
        if not sel:
            messagebox.showinfo("Lecture bag", "Veuillez sélectionner un bag.")
            return

        item_values = self.bag_tree.item(sel[0], 'values')
        bag_name = item_values[0]
        bag_dict = next((b for b in self.bag_data if b["name"] == bag_name), None)
        if not bag_dict:
            messagebox.showerror("Erreur", f"Bag introuvable : {bag_name}")
            return

        full_path = bag_dict["path"]
        if not os.path.isdir(full_path):
            messagebox.showerror("Erreur", f"Le dossier n'existe pas : {full_path}")
            return

        # Si un bag est déjà en cours de lecture, on stoppe
        self.stop_bag()

        # Récupère le rate souhaité
        try:
            self.current_rate = float(self.rate_var.get())
        except ValueError:
            self.current_rate = 1.0
            self.rate_var.set("1.0")

        # Récupère l'offset souhaité
        try:
            self.current_offset = float(self.offset_var.get())
        except ValueError:
            self.current_offset = 0.0
            self.offset_var.set("0")

        # Construction de la commande
        cmd = ["ros2", "bag", "play", full_path, "--rate", str(self.current_rate), "--clock"]
        if self.current_offset > 0.0:
            cmd.extend(["--start-offset", str(self.current_offset)])

        try:
            self.play_process = subprocess.Popen(cmd)
            self.play_start_time = time.time()
            self.current_bag_path = full_path
            self.play_button.config(state="disabled")
            self.stop_button.config(state="normal")
        except Exception as e:
            messagebox.showerror("Erreur", f"Impossible de lancer le bag : {e}")

    def stop_bag(self):
        """Arrête la lecture en cours."""
        if self.play_process is not None:
            self.play_process.terminate()
            self.play_process.wait()
            self.play_process = None
            self.play_start_time = None
            self.current_bag_path = None

        self.play_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.playback_info_label.config(text="Aucun bag en lecture")

    def delete_bag(self):
        """Supprime le bag sélectionné (dossier complet) après confirmation."""
        sel = self.bag_tree.selection()
        if not sel:
            messagebox.showinfo("Suppression bag", "Veuillez sélectionner un bag.")
            return

        item_values = self.bag_tree.item(sel[0], 'values')
        bag_name = item_values[0]
        bag_dict = next((b for b in self.bag_data if b["name"] == bag_name), None)
        if not bag_dict:
            messagebox.showerror("Erreur", f"Bag introuvable : {bag_name}")
            return

        full_path = bag_dict["path"]

        # Demande de confirmation
        resp = messagebox.askyesno("Confirmer", f"Supprimer le bag {bag_name} ?")
        if not resp:
            return

        # Stopper la lecture s'il s'agit du bag en cours
        if self.current_bag_path == full_path:
            self.stop_bag()

        try:
            shutil.rmtree(full_path)
        except Exception as e:
            messagebox.showerror("Erreur", f"Impossible de supprimer : {e}")
            return

        messagebox.showinfo("Info", f"Bag {bag_name} supprimé.")
        self.refresh_bags()

    def refresh_ui(self):
        """Mise à jour régulière de l'affichage (durée de lecture, etc.)."""
        if self.play_process is not None and self.play_start_time is not None:
            elapsed = time.time() - self.play_start_time
            bag_str = os.path.basename(self.current_bag_path) if self.current_bag_path else "?"
            self.playback_info_label.config(
                text=(
                    f"Lecture en cours : {bag_str}\n"
                    f"Taux : {self.current_rate}x | Offset : {self.current_offset}s | Temps écoulé : {elapsed:.1f}s"
                )
            )
        # Relance dans 1 seconde
        self.master.after(1000, self.refresh_ui)

def main():
    if not os.path.exists(ROSBAG_FOLDER):
        print(f"Le dossier de rosbags n'existe pas : {ROSBAG_FOLDER}")
        return

    root = tk.Tk()
    app = RosbagPlayerGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
