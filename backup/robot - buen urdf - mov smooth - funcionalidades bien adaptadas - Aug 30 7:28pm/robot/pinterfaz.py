#!/usr/bin/env python3

#pinterfaz.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import customtkinter as ctk
from PIL import Image  # lo dejamos por si más adelante cargas imágenes
import numpy as np
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
from std_msgs.msg import String 

# ====== CONFIG ======
TOPIC_NAME = '/cmd_deg'            # tópico donde publicamos los grados
JOINT_COUNT = 6
GRIPPER_TOPIC = '/cmd_gripper'  # tópico para el gripper

# Estilo global UI
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Longitudes de eslabón (ajusta a tus medidas reales)
DH_PARAMS = [
    # [theta_offset (rad), d (m), a (m), alpha (rad)]  for links 1..4 (link2 has theta always 0)
    [ -np.pi/2,   -0.03,  -0.01,     0.0    ],   # link 1
    [   0.0,    0.00,   0.00,   -np.pi/2],   # link 2 (θ always 0)
    [ -np.pi,  0.03, -0.105,    0.0    ],   # link 3
    [ 0.0,    0.025, -0.16,     0.0    ],   # link 4
]
# ====================

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# ------------------ DH FUNCTIONS (denavit) ------------------
def denavit(theta, d, a, alpha):
    cth, sth = np.cos(theta), np.sin(theta)
    cal, sal = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ cth,    -sth * cal,   sth * sal,   a * cth ],
        [ sth,     cth * cal,  -cth * sal,   a * sth ],
        [ 0.0,        sal,         cal,        d   ],
        [ 0.0,        0.0,         0.0,      1.0  ]
    ], dtype=float)

def fk_from_dh(q_deg_triplet, dh_params=DH_PARAMS):
    """
    q_deg_triplet: list/tuple of 3 angles in degrees that correspond to joints:
        - slider1 -> joint1
        - slider2 -> joint3
        - slider3 -> joint4
    That reconstructs complete q_deg = [q1, 0.0, q3, q4] (link2 theta=0).
    Returns: T_total (4x4), translation p (3,)
    IMPORTANT: this function uses the **angles as provided by the GUI** (no shift applied).
    """
    # Build q_deg list for DH order: [joint1, joint2(always 0), joint3, joint4]
    q_deg = [float(q_deg_triplet[0]), 0.0, float(q_deg_triplet[1]), float(q_deg_triplet[2])]

    T_total = np.eye(4)
    for i, (theta_off, d, a, alpha) in enumerate(dh_params, start=1):
        theta = np.deg2rad(q_deg[i-1]) + theta_off
        T_i = denavit(theta, d, a, alpha)
        T_total = T_total @ T_i
    p = T_total[:3, 3]
    return T_total, p
# -----------------------------------------------------------


class Ros2Bridge(Node):
    """Nodo mínimo para publicar Float32MultiArray en /cmd_deg."""
    def __init__(self, topic_name=TOPIC_NAME, gripper_topic=GRIPPER_TOPIC, select_topic='/cmd_arm_select'):
        super().__init__('robot_gui_node')

        print("🚀 Versión ACTUALIZADA de pinterfaz cargada correctamente")
        
        self.publisher_ = self.create_publisher(Float32MultiArray, topic_name, 10)
        self.gripper_publisher_ = self.create_publisher(Float32MultiArray, gripper_topic, 10)
        self.arm_select_publisher_ = self.create_publisher(String, select_topic, 10)  # CORREGIDO: String, no Float32MultiArray
        #self.smooth_publisher_ = self.create_publisher(Float32MultiArray, '/cmd_deg_smooth', 10)  # Para movimientos suaves

        # parametros
        self.declare_parameter('use_smooth_motion', False)
        self.use_smooth_motion = self.get_parameter('use_smooth_motion').value
        
        self.get_logger().info(f'Ros2Bridge listo para publicar en {topic_name} y {gripper_topic} y {select_topic} y {"/cmd_deg_smooth"}')
        self.get_logger().info(f'Movimientos suaves: {"habilitados" if self.use_smooth_motion else "deshabilitados"}')

    def publish_degrees(self, angles_deg):
        if len(angles_deg) != JOINT_COUNT:
            self.get_logger().warn(f'publish_degrees: se esperaban {JOINT_COUNT} valores, llegaron {len(angles_deg)}')
            return
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles_deg]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado en {TOPIC_NAME}: {msg.data}')

    def publish_gripper(self, value):
        """Publica un valor float en /cmd_gripper."""
        msg = Float32MultiArray()
        msg.data = [float(value)]
        self.gripper_publisher_.publish(msg)
        self.get_logger().info(f'Publicado en {GRIPPER_TOPIC}: {msg.data}')

    def publish_arm_selection(self, arm_code):
        """Publica 'A' o 'B' en /cmd_arm_select."""
        msg = String()
        msg.data = arm_code
        self.arm_select_publisher_.publish(msg)
        self.get_logger().info(f'Publicado en /cmd_arm_select: {msg.data}')


class VentanaPrincipal(ctk.CTk):
    def __init__(self, ros: Ros2Bridge):
        super().__init__()
        self.ros = ros

        self.title("INTERFAZ CENTAURO")
        self.geometry("634x563")
        self.resizable(False, False)

        self.grid_rowconfigure((0, 1, 2, 3, 4), weight=1)
        self.grid_columnconfigure(0, weight=1)

        self.boton1 = ctk.CTkButton(self,
                                   text="UPPER BODY",
                                   command=self.cambio_upper_body,
                                   fg_color="#737373",
                                   text_color="white",
                                   corner_radius=10,
                                   font=("Arial", 20),
                                   width=225, height=75,
                                   hover_color="#838181")
        self.boton1.grid(row=0, column=0, pady=10)

        self.boton2 = ctk.CTkButton(self,
                                   text="LOWER BODY",
                                   command=self.cambio_lower_body,
                                   fg_color="#737373",
                                   text_color="white",
                                   corner_radius=10,
                                   font=("Arial", 20),
                                   width=225, height=75,
                                   hover_color="#838181")
        self.boton2.grid(row=2, column=0, pady=10)

        self.boton3 = ctk.CTkButton(self,
                                   text="GEARS",
                                   command=self.cambio_gears,
                                   fg_color="#737373",
                                   text_color="white",
                                   corner_radius=10,
                                   font=("Arial", 20),
                                   width=225, height=75,
                                   hover_color="#838181")
        self.boton3.grid(row=4, column=0, pady=10)

        self.linea = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.linea.grid(row=1, column=0, sticky="ew", padx=10, pady=10)

        self.linea2 = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.linea2.grid(row=3, column=0, sticky="ew", padx=10, pady=10)

        # Al cerrar la ventana principal cerramos ROS
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def cambio_upper_body(self):
        self.destroy()
        segunda = UpperBody(self.ros)
        segunda.mainloop()

    def cambio_lower_body(self):
        self.destroy()
        tercera = UpperBody(self.ros)  # placeholder (puedes crear clase distinta si quieres)
        tercera.title("LOWER BODY")
        tercera.mainloop()

    def cambio_gears(self):
        self.destroy()
        cuarta = UpperBody(self.ros)  # placeholder
        cuarta.title("GEARS")
        cuarta.mainloop()

    def on_close(self):
        try:
            # Apagamos nodo ROS antes de cerrar
            try:
                self.ros.get_logger().info("Cerrando nodo ROS desde ventana principal")
                self.ros.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()
        finally:
            self.destroy()


class UpperBody(ctk.CTk):
    def __init__(self, ros: Ros2Bridge):
        super().__init__()
        self.ros = ros

        self.title("UPPER BODY")
        self.geometry("650x700")
        self.resizable(False, False)

        # === Frame superior ========================
        top_frame = ctk.CTkFrame(self, fg_color="transparent", height=50)
        top_frame.pack(fill="x", padx=10, pady=10)

        self.all_joints = [90.0] * JOINT_COUNT

        # CORREGIDO: Variable de instancia definida correctamente
        self.arm_choice = ctk.IntVar(value=1)  # CORREGIDO: era arm_choise
        
        rb1 = ctk.CTkRadioButton(top_frame, text="Brazo Izquierdo",
                         variable=self.arm_choice, value=2,
                         command=self.on_arm_selection,
                         text_color="white", font=("Arial", 20))
        rb2 = ctk.CTkRadioButton(top_frame, text="Brazo Derecho",
                         variable=self.arm_choice, value=1,
                         command=self.on_arm_selection,
                         text_color="white", font=("Arial", 20))
        rb1.grid(row=0, column=0, sticky="w", pady=2)
        rb2.grid(row=1, column=0, sticky="w", pady=2)

        self.boton_records = ctk.CTkButton(top_frame, text="RECORDS", fg_color="#737373", text_color="white",
                                           corner_radius=7, font=("Arial", 20), width=120, height=20,
                                           hover_color="#838181")
        self.boton_records.place(relx=1, x=-10, rely=0.5, anchor="e")

        # Divisoria
        self.divisoria = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.divisoria.pack(fill="x", padx=10, pady=20)

        # === Sliders ===============================
                # === Sliders ===============================
        sliders_frame = ctk.CTkFrame(self, fg_color="transparent")
        sliders_frame.pack(fill="x", padx=10, pady=10)

        sliders_frame.grid_columnconfigure(0, weight=1)
        sliders_frame.grid_columnconfigure(1, weight=0)
        sliders_frame.grid_columnconfigure(2, weight=0)
        sliders_frame.grid_columnconfigure(3, weight=0)
        sliders_frame.grid_columnconfigure(4, weight=0)
        sliders_frame.grid_columnconfigure(5, weight=1)

        self.slider_vars = []

        # Configuración de rangos específicos
        slider_configs = [
            {"min": 0,   "max": 125},   # J1
            {"min": 55,  "max": 180},   # J2
            {"min": 20,  "max": 180}    # J3
        ]

        for i, cfg in enumerate(slider_configs):
            lbl = ctk.CTkLabel(sliders_frame, text=f"J{i+1}", 
                               text_color="#737373", font=("Arial", 15))
            lbl.grid(row=i, column=1, padx=5, pady=5, sticky="w")

            # Valor inicial centrado en el rango
            init_val = round(((cfg["min"] + cfg["max"]) / 2),0)
            slider_value = ctk.DoubleVar(value=init_val)
            self.slider_vars.append(slider_value)

            # Label dinámico con valor actual
            value_lbl = ctk.CTkLabel(sliders_frame, textvariable=slider_value, 
                                     text_color="white", font=("Arial", 15))
            value_lbl.grid(row=i, column=2, padx=5, pady=5, sticky="n")

            # Slider con sus límites personalizados
            slider = ctk.CTkSlider(sliders_frame, 
                                   from_=cfg["min"], to=cfg["max"], 
                                   number_of_steps=int(cfg["max"] - cfg["min"]),
                                   variable=slider_value,
                                   command=lambda val, var=slider_value: var.set(int(float(val))))
            slider.grid(row=i, column=3, padx=5, pady=5, sticky="ew")

            # Mostrar rango real al lado
            range_lbl = ctk.CTkLabel(sliders_frame, 
                                     text=f"{cfg['min']} - {cfg['max']}", 
                                     text_color="#737373", font=("Arial", 10))
            range_lbl.grid(row=i, column=4, padx=5, pady=5, sticky="e")



        # === Botones Confirmar / Gripper ===========
        debajosliders_frame = ctk.CTkFrame(self, fg_color="transparent")
        debajosliders_frame.pack(fill="x", padx=10, pady=10)

        debajosliders_frame.grid_columnconfigure(0, weight=1)
        debajosliders_frame.grid_columnconfigure(1, weight=0)
        debajosliders_frame.grid_columnconfigure(2, weight=0)
        debajosliders_frame.grid_columnconfigure(3, weight=0)
        debajosliders_frame.grid_columnconfigure(4, weight=0)
        debajosliders_frame.grid_columnconfigure(5, weight=1)

        Confirmar_btn = ctk.CTkButton(debajosliders_frame, text="CONFIRMAR",
                                      command=self.confirmar,
                                      fg_color="#737373", text_color="white",
                                      corner_radius=10, font=("Arial", 20),
                                      width=50, height=75, hover_color="#838181")
        Confirmar_btn.grid(row=0, column=1, padx=10)

        gripper_btn = ctk.CTkButton(debajosliders_frame, text="GRIPPER",
                                    #command=self.gripper,
                                    fg_color="#737373", text_color="white",
                                    corner_radius=10, font=("Arial", 20),
                                    width=50, height=75, hover_color="#838181")
        gripper_btn.grid(row=0, column=3, padx=10)

        # Vincular eventos de presionar y soltar
        gripper_btn.bind("<ButtonPress-1>", lambda e: self.start_gripper(1))
        gripper_btn.bind("<ButtonRelease-1>", lambda e: self.stop_gripper())

        # Variables de control
        self.gripper_running = False
        self._after_id = None  # ID del after para poder cancelarlo

        abrir_btn = ctk.CTkButton(debajosliders_frame, text="ABRIR GR",
                                    #command=self.gripper,
                                    fg_color="#737373", text_color="white",
                                    corner_radius=10, font=("Arial", 20),
                                    width=50, height=75, hover_color="#838181")
        abrir_btn.grid(row=0, column=4, padx=10)

        # Vincular eventos de presionar y soltar
        abrir_btn.bind("<ButtonPress-1>", lambda e: self.start_gripper(-1))
        abrir_btn.bind("<ButtonRelease-1>", lambda e: self.stop_gripper())

        # Divisor
        self.divisoria2 = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.divisoria2.pack(fill="x", padx=10, pady=20)

        # ------------------ AQUI CREAMOS LA PARTE GRÁFICA DE COORDENADAS X,Y,Z ------------------
        coord_frame = ctk.CTkFrame(self, fg_color="transparent")
        coord_frame.pack(fill="both", padx=10, pady=5)
        coord_frame.grid_columnconfigure(0, weight=1)
        coord_frame.grid_columnconfigure(1, weight=0)
        coord_frame.grid_columnconfigure(2, weight=0)
        coord_frame.grid_columnconfigure(3, weight=0)
        coord_frame.grid_columnconfigure(4, weight=1)

        # Store references to update the information later
        self.info_labels = []
        coord = ["X", "Y", "Z"]

        for i in range(3):
            # Label de la coordenada
            lbl = ctk.CTkLabel(coord_frame, text=f"{coord[i]}", text_color="#737373", font=("Arial", 15))
            lbl.grid(row=0, column=i+1, padx=5, pady=5)

            # Etiqueta que mostrará el valor (se actualiza con update_coords)
            info_label = ctk.CTkLabel(coord_frame, text="0.0", text_color="white",
                                     font=("Arial", 14), fg_color="#2B2B2B", corner_radius=5,
                                     width=90, height=30)
            info_label.grid(row=1, column=i+1, padx=5, pady=5)
            self.info_labels.append(info_label)
        # -----------------------------------------------------------------------------------------

        self.divisoria3 = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.divisoria3.pack(fill="x", padx=10, pady=20)

        # === Frame inferior ====================
        bottom_frame = ctk.CTkFrame(self, fg_color="transparent")
        bottom_frame.pack(fill="both", padx=10, pady=10)

        bottom_frame.grid_columnconfigure(0, weight=1)
        bottom_frame.grid_columnconfigure(1, weight=0)
        bottom_frame.grid_columnconfigure(2, weight=0)
        bottom_frame.grid_columnconfigure(3, weight=0)
        bottom_frame.grid_columnconfigure(4, weight=1)

        back_btn = ctk.CTkButton(bottom_frame, text="BACK", command=self.volver_menu,
                                 fg_color="#737373", text_color="white",
                                 corner_radius=20, font=("Arial", 20),
                                 width=80, height=70, hover_color="#838181")
        back_btn.grid(row=0, column=1, padx=10, pady=20)

        stop_btn = ctk.CTkButton(bottom_frame, text="STOP", command=self.stop,
                                 fg_color="#737373", text_color="white",
                                 corner_radius=90, font=("Arial", 20),
                                 width=80, height=100, hover_color="#838181")
        stop_btn.grid(row=0, column=2, padx=10, pady=20)

        home_btn = ctk.CTkButton(bottom_frame, text="HOME", command=self.home,
                                 fg_color="#737373", text_color="white",
                                 corner_radius=20, font=("Arial", 20),
                                 width=80, height=70, hover_color="#838181")
        home_btn.grid(row=0, column=3, padx=10, pady=20)

        # No cerramos todo ROS al cerrar solo esta ventana (volver al menú)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ======= Callbacks UI =======

    def read_sliders(self):
        """Return list of 3 slider values (float) in the order [s1,s2,s3]."""
        return [float(v.get()) for v in self.slider_vars]
    
    def confirmar(self):
        sliders = self.read_sliders()  # three values from GUI (these are the angles DH expects)
        arm = int(self.arm_choice.get())
        start = 0 if arm == 1 else 3

        # Update internal all_joints state
        for i, val in enumerate(sliders):
            self.all_joints[start + i] = val

        # Publish 6 values (L1..L3, R1..R3)
        self.ros.publish_degrees(self.all_joints)

        # Compute DH FK using the sliders as [joint1, joint3, joint4]
        # Mapping: slider1 -> joint1, slider2 -> joint3, slider3 -> joint4
        T_total, p = fk_from_dh(sliders, DH_PARAMS)  # uses the GUI angles directly
        # Update coordinate display (p is in meters per DH_PARAMS)
        self.update_coords(p[0], p[1], p[2])
        # ---------------------------------------------------------------------

    def stop(self):
        # Definir posición neutra
        neutral = [90, 90, 0,   90, 90, 0]

        # Actualizar estado interno
        self.all_joints = neutral.copy()

        # Publicar al tópico
        self.ros.publish_degrees(self.all_joints)

        # --- Actualizar sliders del brazo activo ---
        arm = int(self.arm_choice.get())
        start = 0 if arm == 1 else 3
        for i in range(3):
            self.slider_vars[i].set(self.all_joints[start + i])

        # --- Calcular y mostrar coordenadas ---
        sliders = self.read_sliders()  # lee los 3 sliders ya actualizados
        T_total, p = fk_from_dh(sliders, DH_PARAMS)
        self.update_coords(p[0], p[1], p[2])


    def start_gripper(self, direction):
        self.gripper_running = True
        self._send_gripper_loop(direction)

    def stop_gripper(self):
        self.gripper_running = False
        if self._after_id:
            self.after_cancel(self._after_id)

    def _send_gripper_loop(self, direction):
        if self.gripper_running:
            self.ros.publish_gripper(direction)  # 1 = cerrar, -1 = abrir
            self._after_id = self.after(100, lambda: self._send_gripper_loop(direction))

    def on_arm_selection(self):
        val = self.arm_choice.get()
        if val == 1:
            self.ros.publish_arm_selection('A')
        elif val == 2:
            self.ros.publish_arm_selection('B')

    def home(self):
        # Placeholder para orden de "home"
        pass

    def volver_menu(self):
        self.destroy()
        v = VentanaPrincipal(self.ros)
        v.mainloop()

    def on_close(self):
        # Cerramos solo la ventana; el nodo ROS sigue vivo si retornas al menú
        try:
            # usar el logger del nodo ROS
            try:
                self.ros.get_logger().info("Cerrando ventana UpperBody")
            except Exception:
                pass
        finally:
            self.destroy()

    # ------------------ Método para actualizar coordenadas ------------------
    def update_coords(self, x, y, z):
        """
        Actualiza las etiquetas X,Y,Z en la interfaz.
        Se usa self.after(0,...) para ser thread-safe si se llama desde otro hilo.
        """
        def _apply():
            vals = [x, y, z]
            for lbl, v in zip(self.info_labels, vals):
                try:
                    lbl.configure(text=f"{float(v):.3f}")  # 3 decimales
                except Exception:
                    lbl.configure(text=str(v))
        try:
            self.after(0, _apply)
        except Exception:
            _apply()
    # -------------------------------------------------------------------------

def main():
    rclpy.init()
    ros = Ros2Bridge(topic_name=TOPIC_NAME)
    app = VentanaPrincipal(ros)
    app.mainloop()
    # Al cerrar la ventana principal:
    if rclpy.ok():
        try:
            ros.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()