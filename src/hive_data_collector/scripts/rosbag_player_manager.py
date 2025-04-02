#!/usr/bin/env python3

import os
import time
import threading
import subprocess
from datetime import datetime
from tkinter import (
    Tk, Frame, Label, Button, Listbox, Entry, StringVar, END, SINGLE, Toplevel, messagebox
)

# Dossier où sont stockés vos rosbags
ROSBAG_FOLDER = "/home/thomasraynal/no_ros_project/my_ws/data/data_hive_data_collector"

class RosbagPlayerGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("ROS2 Bag Player Manager")

        # Liste des bag folders
        self.bag_listbox = Listbox(master, selectmode=SINGLE, width=60)
        self.bag_listbox.grid(row=0, column=0, columnspan=3, padx=10, pady=5, sticky="nsew")

        # Bouton de rafraîchissement
        self.refresh_button = Button(master, text="Rafraîchir la liste", command=self.refresh_bags)
        self.refresh_button.grid(row=1, column=0, padx=5, pady=5, sticky="w")

        # Bouton pour afficher infos
        self.info_button = Button(master, text="Infos sur le bag", command=self.show_bag_info)
        self.info_button.grid(row=1, column=1, padx=5, pady=5, sticky="w")

        # Bouton pour supprimer
        self.delete_button = Button(master, text="Supprimer", command=self.delete_bag)
        self.delete_button.grid(row=1, column=2, padx=5, pady=5, sticky="e")

        # Label + champ : Taux (rate)
        Label(master, text="Taux (rate) :").grid(row=2, column=0, padx=5, pady=5, sticky="e")
        self.rate_var = StringVar(value="1.0")
        self.rate_entry = Entry(master, textvariable=self.rate_var, width=8)
        self.rate_entry.grid(row=2, column=1, padx=5, pady=5, sticky="w")

        # Label + champ : Start offset
        Label(master, text="Start offset (s) :").grid(row=3, column=0, padx=5, pady=5, sticky="e")
        self.offset_var = StringVar(value="0")
        self.offset_entry = Entry(master, textvariable=self.offset_var, width=8)
        self.offset_entry.grid(row=3, column=1, padx=5, pady=5, sticky="w")

        # Boutons play / stop
        self.play_button = Button(master, text="Lancer la lecture", command=self.play_bag)
        self.play_button.grid(row=4, column=0, padx=5, pady=5, sticky="e")

        self.stop_button = Button(master, text="Arrêter la lecture", command=self.stop_bag, state="disabled")
        self.stop_button.grid(row=4, column=1, padx=5, pady=5, sticky="w")

        # Label d'info sur la lecture en cours
        self.playback_info_label = Label(master, text="Aucun bag en lecture")
        self.playback_info_label.grid(row=5, column=0, columnspan=3, padx=5, pady=5, sticky="w")

        # Gestion du processus de lecture
        self.play_process = None
        self.play_start_time = None
        self.current_bag_path = None
        self.current_rate = 1.0
        self.current_offset = 0.0

        # Redimensionnement
        master.rowconfigure(0, weight=1)
        master.columnconfigure(0, weight=1)

        # Lancement du rafraîchissement
        self.refresh_bags()
        self.refresh_ui()

    def refresh_bags(self):
        """Scanne le dossier ROSBAG_FOLDER pour lister les sous-dossiers qui contiennent un rosbag."""
        self.bag_listbox.delete(0, END)

        if not os.path.exists(ROSBAG_FOLDER):
            return

        # Pour chaque sous-dossier, on suppose qu'il y a un bag
        for name in os.listdir(ROSBAG_FOLDER):
            full_path = os.path.join(ROSBAG_FOLDER, name)
            if os.path.isdir(full_path):
                # On vérifie s'il y a un metadata.yaml, par ex
                meta_path = os.path.join(full_path, "metadata.yaml")
                if os.path.exists(meta_path):
                    self.bag_listbox.insert(END, name)
                # sinon on ignore

    def show_bag_info(self):
        """Affiche dans une fenêtre pop-up les infos du bag sélectionné."""
        sel = self.bag_listbox.curselection()
        if not sel:
            messagebox.showinfo("Info bag", "Veuillez sélectionner un bag.")
            return

        bag_name = self.bag_listbox.get(sel[0])
        full_path = os.path.join(ROSBAG_FOLDER, bag_name)
        try:
            out = subprocess.check_output(["ros2", "bag", "info", full_path], text=True)
        except Exception as e:
            messagebox.showerror("Erreur", f"Impossible de récupérer les infos : {e}")
            return

        info_win = Toplevel(self.master)
        info_win.title(f"Infos sur {bag_name}")
        Label(info_win, text=out, justify="left", anchor="nw").pack(padx=10, pady=10, fill="both", expand=True)

    def play_bag(self):
        """Lance la lecture du bag sélectionné avec le rate et l'offset indiqués."""
        sel = self.bag_listbox.curselection()
        if not sel:
            messagebox.showinfo("Lecture bag", "Veuillez sélectionner un bag.")
            return

        bag_name = self.bag_listbox.get(sel[0])
        full_path = os.path.join(ROSBAG_FOLDER, bag_name)
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
        cmd = ["ros2", "bag", "play", full_path, "--rate", str(self.current_rate)]
        # Si offset > 0, on ajoute l'option "--start-offset offset_value"
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
        sel = self.bag_listbox.curselection()
        if not sel:
            messagebox.showinfo("Suppression bag", "Veuillez sélectionner un bag.")
            return

        bag_name = self.bag_listbox.get(sel[0])
        full_path = os.path.join(ROSBAG_FOLDER, bag_name)

        # Demande de confirmation
        resp = messagebox.askyesno("Confirmer", f"Supprimer le bag {bag_name} ?")
        if not resp:
            return

        # Stopper la lecture s'il s'agit du bag en cours
        if self.current_bag_path == full_path:
            self.stop_bag()

        try:
            import shutil
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
        else:
            # Pas de lecture en cours
            pass

        # Relance dans 1 sec
        self.master.after(1000, self.refresh_ui)

def main():
    # Vérifier l'existence du dossier
    if not os.path.exists(ROSBAG_FOLDER):
        print(f"Le dossier de rosbags n'existe pas : {ROSBAG_FOLDER}")
        return

    root = Tk()
    app = RosbagPlayerGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
