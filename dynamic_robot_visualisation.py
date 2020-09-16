#urdf parser und manipulator
import urdfpy as up
import pyrender

#thread zur parallelisierung von visualisierung und manuellen eingaben
import threading

#mathematische hilfsmittel
import numpy as np
import math

#dynamische benutzeroberfläche als schaltfläche für die Joint-Werte
import tkinter as tk

#Netzwerkkommunikation um live werte zu erhalten
import opcua

#sleep für disconnect
from time import sleep


class Robot_Visualisation(threading.Thread):

    """
    Klasse läuft über thread um nebenläufig tkinter zum laufen zu
    bringen, als auch live werte von opcua zu übernehmen
    """

    # Globale Variablen
    meshes = []

    joint1_value = 0.00
    joint2_value = 0.00
    joint3_value = 0.00
    joint4_value = 0.00
    joint5_value = 0.00
    joint6_value = 0.00

    #globale variablen für OPC-UA
    opcua_bool = False
    client = None


    def run(self):
        """
        Lädt die Urdf Datei und printet die Links // main schleife für die Visualisierung

        :return: //
        """

        #URDF Datei laden:
        robot_urdf_file = "cr35ia.urdf"
        self.robot = up.URDF.load(robot_urdf_file)#laden....


        #printen der Links: -->robot.links container!!
        for link in self.robot.links:
            #mit link.name // Attributen zugriff
            print("Link: {}".format(link.name))


        # Ausgeben aller Joints allgemein:
        for joint in self.robot.joints:
            # mit joint.name //als Attributen zugriff
            print("Joint: {}".format(joint.name))


        #Ausgeben der Joint-Link-Verbindungen des Roboters ausgehend von dem URDF file
        # -->robot.joints container
        for joint in self.robot.joints:
            #mit joint.name/parent/child  // Attributen Zugriff
            print("{} connects {} to {}".format(joint.name, joint.parent, joint.child))


        self.show_visualisation(cfg={
            "joint_1": self.joint1_value,
            "joint_2": self.joint2_value,
            "joint_3": self.joint3_value,
            "joint_4": self.joint4_value,
            "joint_5": self.joint5_value,
            "joint_6": self.joint6_value,
        })

        #rendern des Viewers
        viewer = pyrender.Viewer(self.scene, use_raymond_lighting=True, run_in_thread=True)


        #zum aktualisieren der Schleife // dauerschleife ohne abbruchkriterium
        while True:

            #opc werte aktualisieren, falls opc = True
            if self.opcua_bool:
                self.update_values_opc()

            #blockieren des Renders vor der aktualisierung, sonst kollision // Kapitel kryptografie
            viewer.render_lock.acquire()

            # Aktualisieren der Visualiserung mit den aktuellen Joint-werten
            self.update_visualisation(cfg={
                "joint_1": self.joint1_value,
                "joint_2": self.joint2_value,
                "joint_3": self.joint3_value,
                "joint_4": self.joint4_value,
                "joint_5": self.joint5_value,
                "joint_6": self.joint6_value
            })

            #nach aktualisierung render freigeben // Kapitel Kryptografie
            viewer.render_lock.release()


    def show_visualisation(self, cfg=None, use_collision=False):
        """
        Initialisieren der Visualisierung mit initalen Joint-Werten
        wurde aus urdfpy kopiert und sinnvoll angepasst

        :param cfg: configuration
        :param use_collision: collision mesh or visual
        :return: scene
        """

        #für collision == True wird die collisoion visualisierung genommen
        if use_collision:
            fk = self.robot.collision_trimesh_fk(cfg=cfg)

        else:
            fk = self.robot.visual_trimesh_fk(cfg=cfg)

        #visualisierung:
        self.scene = pyrender.Scene()

        #hier werden die aktuellen meshes ausgelesen und visualisiert
        for tm in fk:
            pose = fk[tm]
            mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
            nm = pyrender.Node(mesh=mesh, matrix=np.eye(4))#node wird gegettet
            self.meshes.append(nm)
            #visualisierung:
            self.scene.add_node(nm)

    def update_visualisation(self, cfg=None, use_collision=False):
        """
        Aktualisieren der visualisierung mit Joint-Werten
        Diese Funktion wurde aus urdfpy kopiert und sinnvoll angepasst

        :param cfg: configuration
        :param use_collision: collision mesh or viusal
        :return: //
        """

        #collision mesh für collision == TRUE:
        if use_collision:
            fk = self.robot.collision_trimesh_fk(cfg=cfg)
        else:
            fk = self.robot.visual_trimesh_fk(cfg=cfg)

        #zählvariable
        i = 0

        #um die meshes zu bekommen
        for tm in fk:
            pose = fk[tm]
            #set_pose um setzt die aktuelle Position für ein Mesh
            self.scene.set_pose(self.meshes[i], pose=pose)
            i = i + 1



    def update_values_opc(self):
        """
        Aktualisiert die Joint-Werte mit OPC-UA Werten // Live-Werten

        :return: //
        """


        if self.opcua_bool:
            """
            Wenn OPC-UA aktiv ist sollen die werte übernommen werden, ansonsten manuelle Werte
            """

            #joints mit netzwerk kommunikation ansprechen
            joint1_node = self.client.get_node('ns=3;s="cr35ia"."Angles"."axis1angle"')
            joint2_node = self.client.get_node('ns=3;s="cr35ia"."Angles"."axis2angle"')
            joint3_node = self.client.get_node('ns=3;s="cr35ia"."Angles"."axis3angle"')
            joint4_node = self.client.get_node('ns=3;s="cr35ia"."Angles"."axis4angle"')
            joint5_node = self.client.get_node('ns=3;s="cr35ia"."Angles"."axis5angle"')
            joint6_node = self.client.get_node('ns=3;s="cr35ia"."Angles"."axis6angle"')


            #Live Werte auslesen mit var.client.get_node.get_value und an die globalen Variablen übergeben
            #degree live werte in radian transformieren mit math.radians(degree)
            self.joint1_value = math.radians(float(joint1_node.get_value()))
            self.joint2_value = math.radians(float(joint2_node.get_value()))
            self.joint3_value = math.radians(float(joint3_node.get_value()))
            self.joint4_value = math.radians(float(joint4_node.get_value()))
            self.joint5_value = math.radians(float(joint5_node.get_value()))
            self.joint6_value = math.radians(float(joint6_node.get_value()))




    def update_values_tk(self):
        """
        Aktualisiert die Joint-Werte mit den Scale-Werten der tkinter Oberfläche

        :return://
        """


        if not self.opcua_bool:
            """
            Wenn OPC-UA deaktiv ist sollen die Schieberegler werte übernommen werden
            """

            #werte von tkinter abfragen // durch 100 teilen, da schieberegler werte als 100fache angegeben wurden
            self.joint1_value = self.joint1_scale.get() / 100
            self.joint2_value = self.joint2_scale.get() / 100
            self.joint3_value = self.joint3_scale.get() / 100
            self.joint4_value = self.joint4_scale.get() / 100
            self.joint5_value = self.joint5_scale.get() / 100
            self.joint6_value = self.joint6_scale.get() / 100

            #Fortesetzen der Schleife:
            self.root.after(1, self.update_values_tk)


    def opcua_on_off(self):
        """
        Aktiviert und deaktiviert opcua mithilfe des buttons von tkinter

        :return: //
        """

        if self.opcua_bool:
            """
            Wenn OPC-UA aktiv ist, dann Verbindung trennen, opcua_bool False setzen und Button umbennen
            """

            # opcua False setzen
            self.opcua_bool = False
            sleep(3)
            #verbindung trennen
            self.client.disconnect()
            self.update_values_tk()



            #Text ändern; auf variable über dict zugreifen:
            self.opcua_button["text"] = "OPC-UA aktivieren"

        else:
            """
            Wenn OPC-UA deaktiv ist, dann verbinden, opcua_bool false setzen und Button umbennen
            """

            #verbindung aufnehmen:
            self.client = opcua.Client("opc.tcp://192.168.0.4:4840/")
            self.client.connect()

            #opcua True setzen:
            self.opcua_bool = True

            #Text ändern; auf die variable über dict zugreifen:
            self.opcua_button["text"] = "OPC-UA deaktivieren"





    def create_tk_window(self):
        """
        Erstellen einer benutzeroberfläch zur dynamischen Schaltfläche der joint Werte

        :return: //
        """

        self.root = tk.Tk()

        self.root.geometry('400x700')
        self.root.title("Fanuc-r2000iB")


        #Rahmen, Frame...
        root_frame = tk.Frame(self.root, bg="white")
        root_frame.pack(fill="both", expand=True)

        #Button zum an und aus schalten der Live Werte (OPC-UA)
        label_button = tk.LabelFrame(root_frame, text="OPC_UA", bg="orange", font=(30))
        label_button.pack(fill="both", expand=True, padx=10, pady=10)
        self.opcua_button = tk.Button(label_button, text="OPC-UA aktivieren", width=20, command=self.opcua_on_off,
                                      bg="black", fg="white", font=(20))
        self.opcua_button.pack(expand=True, pady=5)

        #Schieberegler für die manuellen Joint Werte
        #(from, to mit 100 multiplizieren; keine komma werte!)

        label_joint1 = tk.LabelFrame(root_frame, text="JOINT 1", bg="orange", font=(30))
        label_joint1.pack(fill="both", expand=True, padx=10, pady=10)
        self.joint1_scale = tk.Scale(label_joint1, length=290, from_=-296, to=296, orient='horizontal', bg="black",
                                     fg="white", font=(20))
        self.joint1_scale.pack(expand=True)

        label_joint2 = tk.LabelFrame(root_frame, text="JOINT 2", bg="orange", font=(30))
        label_joint2.pack(fill="both", expand=True, padx=10, pady=10)
        self.joint2_scale = tk.Scale(label_joint2, length=290, from_=-78, to=209, orient='horizontal', bg="black",
                                     fg="white", font=(20))
        self.joint2_scale.pack(expand=True)

        label_joint3 = tk.LabelFrame(root_frame, text="JOINT 3", bg="orange", font=(30))
        label_joint3.pack(fill="both", expand=True, padx=10, pady=10)
        self.joint3_scale = tk.Scale(label_joint3, length=290, from_=-235, to=214, orient='horizontal', bg="black",
                                     fg="white", font=(20))
        self.joint3_scale.pack(expand=True)

        label_joint4 = tk.LabelFrame(root_frame, text="JOINT 4", bg="orange", font=(30))
        label_joint4.pack(fill="both", expand=True, padx=10, pady=10)
        self.joint4_scale = tk.Scale(label_joint4, length=290, from_=-314, to=314, orient='horizontal', bg="black",
                                     fg="white", font=(20))
        self.joint4_scale.pack(expand=True)

        label_joint5 = tk.LabelFrame(root_frame, text="JOINT 5", bg="orange", font=(30))
        label_joint5.pack(fill="both", expand=True, padx=10, pady=10)
        self.joint5_scale = tk.Scale(label_joint5, length=290, from_=-314, to=314, orient='horizontal', bg="black",
                                     fg="white", font=(20))
        self.joint5_scale.pack(expand=True)

        label_joint6 = tk.LabelFrame(root_frame, text="JOINT 6", bg="orange", font=(30))
        label_joint6.pack(fill="both", expand=True, padx=10, pady=10)
        self.joint6_scale = tk.Scale(label_joint6, length=290, from_=-314, to=314, orient='horizontal', bg="black",
                                     fg="white", font=20)
        self.joint6_scale.pack(expand=True)




        #Update Schleife starten, welche die Schieberegler werte auf die joint werte überträgt

        self.root.after(1, self.update_values_tk)

        #hauptschleife
        self.root.mainloop()




if __name__ == '__main__':

    my_robot = Robot_Visualisation()
    my_robot.start()

    my_robot.create_tk_window()





