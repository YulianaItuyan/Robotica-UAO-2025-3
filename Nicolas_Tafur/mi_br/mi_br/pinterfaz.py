#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import customtkinter as ctk
from PIL import Image
import numpy as np

# ====== CONFIG ======
TOPIC_NAME = '/cmd_deg'            # tópico donde publicamos los grados
JOINT_COUNT = 6                    # 6 joints: [L1,L2,L3, R1,R2,R3]
GRIPPER_TOPIC = '/cmd_gripper'     # tópico para el gripper
# Default link geometry (used by DH FK). Ajusta si cada brazo tiene distinta geometría.
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
    def __init__(self, topic_name=TOPIC_NAME):
        super().__init__('mi_br_gui_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, topic_name, 10)
        # AGREGADO: Publisher para gripper
        self.gripper_publisher_ = self.create_publisher(Float32MultiArray, GRIPPER_TOPIC, 10)
        self.get_logger().info(f'Ros2Bridge listo para publicar en {topic_name} y {GRIPPER_TOPIC}')

    def publish_degrees(self, angles_deg):
        if len(angles_deg) != JOINT_COUNT:
            self.get_logger().warn(f'publish_degrees: se esperaban {JOINT_COUNT} valores, llegaron {len(angles_deg)}')
            return
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles_deg]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado en {TOPIC_NAME}: {msg.data}')

    # AGREGADO: Método para publicar gripper
    def publish_gripper(self, value):
        """Publica un valor float en /cmd_gripper."""
        msg = Float32MultiArray()
        msg.data = [float(value)]
        self.gripper_publisher_.publish(msg)
        self.get_logger().info(f'Publicado en {GRIPPER_TOPIC}: {msg.data}')


class VentanaPrincipal(ctk.CTk):
    def __init__(self, ros: Ros2Bridge):
        super().__init__()
        self.ros = ros
        self.title("INTERFAZ CENTAURO")
        self.geometry("634x563")
        self.resizable(False, False)
        self.grid_rowconfigure((0, 1, 2, 3, 4), weight=1)
        self.grid_columnconfigure(0, weight=1)

        self.boton1 = ctk.CTkButton(self, text="UPPER BODY", command=self.cambio_upper_body, fg_color="#737373",
                                   text_color="white", corner_radius=10, font=("Arial", 20), width=225, height=75,
                                   hover_color="#838181")
        self.boton1.grid(row=0, column=0, pady=10)

        self.boton2 = ctk.CTkButton(self, text="LOWER BODY", command=self.cambio_lower_body, fg_color="#737373",
                                   text_color="white", corner_radius=10, font=("Arial", 20), width=225, height=75,
                                   hover_color="#838181")
        self.boton2.grid(row=2, column=0, pady=10)

        self.boton3 = ctk.CTkButton(self, text="GEARS", command=self.cambio_gears, fg_color="#737373",
                                   text_color="white", corner_radius=10, font=("Arial", 20), width=225, height=75,
                                   hover_color="#838181")
        self.boton3.grid(row=4, column=0, pady=10)

        self.linea = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.linea.grid(row=1, column=0, sticky="ew", padx=10, pady=10)
        self.linea2 = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.linea2.grid(row=3, column=0, sticky="ew", padx=10, pady=10)

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def cambio_upper_body(self):
        self.destroy()
        segunda = UpperBody(self.ros)
        segunda.mainloop()

    def cambio_lower_body(self):
        self.destroy()
        tercera = UpperBody(self.ros)
        tercera.title("LOWER BODY")
        tercera.mainloop()

    def cambio_gears(self):
        self.destroy()
        cuarta = UpperBody(self.ros)
        cuarta.title("GEARS")
        cuarta.mainloop()

    def on_close(self):
        try:
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
        self.geometry("750x900")  # Aumenté la altura para la matriz
        self.resizable(False, False)

        # State: all 6 joints (L1..L3, R1..R3) initialized to 90° (neutral)
        # self.all_joints = [90.0] * JOINT_COUNT
        
        
        self.HOME_POSITIONS = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]  # Tus valores reales
        self.all_joints = self.HOME_POSITIONS.copy()
        
        

        # AGREGADO: Variables de control del gripper
        self.gripper_running = False
        self._after_id = None  # ID del after para poder cancelarlo

        # ---------- TOP FRAME (radio buttons) -----------
        top_frame = ctk.CTkFrame(self, fg_color="transparent", height=60)
        top_frame.pack(fill="x", padx=10, pady=10)

        # Radio buttons to choose arm: 1=Left, 2=Right
        self.arm_choice = ctk.IntVar(value=1)
        rb1 = ctk.CTkRadioButton(top_frame, text="Brazo Derecho", variable=self.arm_choice, value=1,
                                 text_color="white", font=("Arial", 18), command=self.on_arm_change)
        rb2 = ctk.CTkRadioButton(top_frame, text="Brazo Izquierdo",  variable=self.arm_choice, value=2,
                                 text_color="white", font=("Arial", 18), command=self.on_arm_change)
        rb1.grid(row=0, column=0, sticky="w", pady=2)
        rb2.grid(row=1, column=0, sticky="w", pady=2)

        # Records button (placeholder)
        self.boton_records = ctk.CTkButton(top_frame, text="RECORDS", fg_color="#737373", text_color="white",
                                           corner_radius=7, font=("Arial", 16), width=120, height=24,
                                           hover_color="#838181")
        self.boton_records.place(relx=1, x=-10, rely=0.5, anchor="e")

        # ---------- SLIDERS FRAME ----------
        sliders_frame = ctk.CTkFrame(self, fg_color="transparent")
        sliders_frame.pack(fill="x", padx=10, pady=10)

        # ---- HERE: Configure slider limits and inversion per joint ----
        # Format per slider: (min_val, max_val, steps, invert_flag)
        self.LEFT_SLIDER_CONFIGS = [
            (20.0, 96.0, 76, False),  # Left slider J1 -> joint1
            (65.0, 180.0, 115, False),  # Left slider J2 -> joint3
            (0.0, 110.0, 110, False),  # Left slider J3 -> joint4
        ]
        # If right arm has different limits, set them here:
        self.RIGHT_SLIDER_CONFIGS = [
            (20.0, 96.0, 76, False),  # Right slider J1 -> joint1 (right)
            (65.0, 180.0, 115, False),  # Right slider J2 -> joint3 (right)
            (0.0, 110.0, 110, False),  # Right slider J3 -> joint4 (right)
        ]
        # ----------------------------------------------------------------

        sliders_frame.grid_columnconfigure(0, weight=1)
        sliders_frame.grid_columnconfigure(1, weight=0)
        sliders_frame.grid_columnconfigure(2, weight=0)
        sliders_frame.grid_columnconfigure(3, weight=0)
        sliders_frame.grid_columnconfigure(4, weight=0)
        sliders_frame.grid_columnconfigure(5, weight=0)
        sliders_frame.grid_columnconfigure(6, weight=1)

        self.slider_vars = []
        self.slider_widgets = []

        # create sliders (we'll apply configs in on_arm_change to allow different limits per arm)
        for i in range(3):
            lbl = ctk.CTkLabel(sliders_frame, text=f"J{i+1}", text_color="#737373", font=("Arial", 15))
            lbl.grid(row=i, column=1, padx=5, pady=5, sticky="w")

            slider_value = ctk.DoubleVar(value=90.0)
            self.slider_vars.append(slider_value)

            value_lbl = ctk.CTkLabel(sliders_frame, textvariable=slider_value, text_color="white", font=("Arial", 15))
            value_lbl.grid(row=i, column=2, padx=5, pady=5, sticky="n")

            # placeholder slider; real from_/to set in apply_slider_configs
            slider = ctk.CTkSlider(sliders_frame, from_=0, to=180, number_of_steps=180,
                                   variable=slider_value,
                                   command=lambda val, var=slider_value: var.set(round(float(val), 3)))
            slider.grid(row=i, column=3, padx=5, pady=5, sticky="ew")

            range_lbl = ctk.CTkLabel(sliders_frame, text="0-180", text_color="#737373", font=("Arial", 10))
            range_lbl.grid(row=i, column=4, padx=5, pady=5, sticky="e")

            self.slider_widgets.append(slider)

        # Apply initial slider configs for the default selected arm (Left)
        self.apply_slider_configs(self.LEFT_SLIDER_CONFIGS)

        # Populate sliders with current arm values (from self.all_joints)
        self.sync_sliders_with_selected_arm()

        # ---------- Confirm / Gripper ----------
        debajosliders_frame = ctk.CTkFrame(self, fg_color="transparent")
        debajosliders_frame.pack(fill="x", padx=10, pady=10)
        debajosliders_frame.grid_columnconfigure(0, weight=1)
        debajosliders_frame.grid_columnconfigure(1, weight=0)
        debajosliders_frame.grid_columnconfigure(2, weight=0)
        debajosliders_frame.grid_columnconfigure(3, weight=0)
        debajosliders_frame.grid_columnconfigure(4, weight=0)
        debajosliders_frame.grid_columnconfigure(5, weight=1)

        Confirmar_btn = ctk.CTkButton(debajosliders_frame, text="CONFIRMAR", command=self.confirmar,
                                      fg_color="#737373", text_color="white",
                                      corner_radius=10, font=("Arial", 20), width=50, height=75,
                                      hover_color="#838181")
        Confirmar_btn.grid(row=0, column=1, padx=10)

        # MODIFICADO: Gripper con control por presión (cerrar)
        gripper_btn = ctk.CTkButton(debajosliders_frame, text="CERRAR GR",
                                    fg_color="#737373", text_color="white", corner_radius=10,
                                    font=("Arial", 20), width=50, height=75,
                                    hover_color="#838181")
        gripper_btn.grid(row=0, column=2, padx=10)

        # AGREGADO: Vincular eventos de presionar y soltar para cerrar gripper
        gripper_btn.bind("<ButtonPress-1>", lambda e: self.start_gripper(1))
        gripper_btn.bind("<ButtonRelease-1>", lambda e: self.stop_gripper())

        # AGREGADO: Botón para abrir gripper
        abrir_btn = ctk.CTkButton(debajosliders_frame, text="ABRIR GR",
                                    fg_color="#737373", text_color="white", corner_radius=10,
                                    font=("Arial", 20), width=50, height=75,
                                    hover_color="#838181")
        abrir_btn.grid(row=0, column=3, padx=10)

        # AGREGADO: Vincular eventos de presionar y soltar para abrir gripper
        abrir_btn.bind("<ButtonPress-1>", lambda e: self.start_gripper(-1))
        abrir_btn.bind("<ButtonRelease-1>", lambda e: self.stop_gripper())

        # ---------- COORDINATES DISPLAY (X,Y,Z) ----------
        coord_frame = ctk.CTkFrame(self, fg_color="transparent")
        coord_frame.pack(fill="both", padx=10, pady=5)
        coord_frame.grid_columnconfigure(0, weight=1)
        coord_frame.grid_columnconfigure(1, weight=0)
        coord_frame.grid_columnconfigure(2, weight=0)
        coord_frame.grid_columnconfigure(3, weight=0)
        coord_frame.grid_columnconfigure(4, weight=1)

        self.info_labels = []
        coord = ["X (m)", "Y (m)", "Z (m)"]
        for i in range(3):
            lbl = ctk.CTkLabel(coord_frame, text=f"{coord[i]}", text_color="#737373", font=("Arial", 15))
            lbl.grid(row=0, column=i+1, padx=5, pady=5)
            info_label = ctk.CTkLabel(coord_frame, text="0.000", text_color="white",
                                     font=("Arial", 14), fg_color="#2B2B2B", corner_radius=5,
                                     width=100, height=30)
            info_label.grid(row=1, column=i+1, padx=5, pady=5)
            self.info_labels.append(info_label)

        # ---------- NUEVA SECCIÓN: MATRIZ DE TRANSFORMACIÓN HOMOGÉNEA 4x4 ----------
        # Título de la sección
        matrix_title = ctk.CTkLabel(self, text="Matriz de Transformación Homogénea T", 
                                   text_color="white", font=("Arial", 16, "bold"))
        matrix_title.pack(pady=(20, 5))

        matrix_frame = ctk.CTkFrame(self, fg_color="transparent")
        matrix_frame.pack(fill="both", padx=10, pady=5)

        matrix_frame.grid_columnconfigure(0, weight=1)
        for j in range(4):
            matrix_frame.grid_columnconfigure(j+1, weight=0)
        matrix_frame.grid_columnconfigure(5, weight=1)

        # Crear etiquetas 4x4 para los valores de T
        self.T_labels = []
        for i in range(4):
            row_labels = []
            for j in range(4):
                val = ctk.CTkLabel(matrix_frame, text="0.000",
                                   text_color="white", font=("Arial", 12),
                                   fg_color="#2B2B2B", corner_radius=6,
                                   width=80, height=28)
                val.grid(row=i, column=j+1, padx=2, pady=2)
                row_labels.append(val)
            self.T_labels.append(row_labels)

        # ---------- bottom buttons ----------
        self.divisoria3 = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.divisoria3.pack(fill="x", padx=10, pady=10)

        bottom_frame = ctk.CTkFrame(self, fg_color="transparent")
        bottom_frame.pack(fill="both", padx=10, pady=10)
        bottom_frame.grid_columnconfigure(0, weight=1)
        bottom_frame.grid_columnconfigure(1, weight=0)
        bottom_frame.grid_columnconfigure(2, weight=0)
        bottom_frame.grid_columnconfigure(3, weight=0)
        bottom_frame.grid_columnconfigure(4, weight=1)

        back_btn = ctk.CTkButton(bottom_frame, text="BACK", command=self.volver_menu,
                                 fg_color="#737373", text_color="white",
                                 corner_radius=20, font=("Arial", 20), width=80, height=70,
                                 hover_color="#838181")
        back_btn.grid(row=0, column=1, padx=10, pady=20)

        stop_btn = ctk.CTkButton(bottom_frame, text="STOP", command=self.stop,
                                 fg_color="#737373", text_color="white",
                                 corner_radius=90, font=("Arial", 20),
                                 width=80, height=100,
                                 hover_color="#838181")
        stop_btn.grid(row=0, column=2, padx=10, pady=20)

        home_btn = ctk.CTkButton(bottom_frame, text="HOME", command=self.home,
                                 fg_color="#737373", text_color="white",
                                 corner_radius=20, font=("Arial", 20), width=80, height=70,
                                 hover_color="#838181")
        home_btn.grid(row=0, column=3, padx=10, pady=20)

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- Slider helpers ----------
    def apply_slider_configs(self, configs):
        """Apply (min,max,steps,invert) to the 3 slider widgets and update range labels."""
        for i, cfg in enumerate(configs):
            min_val, max_val, steps, invert = cfg
            from_arg = max_val if invert else min_val
            to_arg = min_val if invert else max_val
            # update slider widget
            self.slider_widgets[i].configure(from_=from_arg, to=to_arg, number_of_steps=int(steps))
            # update adjacent range label (if present)
            parent = self.slider_widgets[i].master
            # find the label in same row column=4
            for w in parent.grid_slaves(row=i, column=4):
                try:
                    w.configure(text=f"{min_val:.0f}-{max_val:.0f}")
                except Exception:
                    pass

    def sync_sliders_with_selected_arm(self):
        """Copy values from self.all_joints to sliders for the currently selected arm."""
        arm = int(self.arm_choice.get())
        start = 0 if arm == 1 else 3
        for i in range(3):
            self.slider_vars[i].set(self.all_joints[start + i])
            # update the visual slider position too
            try:
                self.slider_widgets[i].set(self.all_joints[start + i])
            except Exception:
                pass

    def read_sliders(self):
        """Return list of 3 slider values (float) in the order [s1,s2,s3]."""
        return [float(v.get()) for v in self.slider_vars]

    def on_arm_change(self):
        """Called when user switches the radio button (left/right). Apply configs and sync values."""
        if int(self.arm_choice.get()) == 1:
            self.apply_slider_configs(self.LEFT_SLIDER_CONFIGS)
        else:
            self.apply_slider_configs(self.RIGHT_SLIDER_CONFIGS)
        self.sync_sliders_with_selected_arm()

    # ---------- Callbacks UI ----------
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
        
        # NUEVO: Update homogeneous transformation matrix display
        self.update_T_matrix(T_total)

    def stop(self):
        neutral = [90.0] * JOINT_COUNT
        self.all_joints = neutral.copy()
        self.ros.publish_degrees(self.all_joints)
        # update sliders and coords for current arm
        self.sync_sliders_with_selected_arm()
        sliders = self.read_sliders()
        T_total, p = fk_from_dh(sliders, DH_PARAMS)
        self.update_coords(p[0], p[1], p[2])
        # NUEVO: Update matrix display for neutral position
        self.update_T_matrix(T_total)

    # AGREGADO: Métodos para control del gripper por presión
    def start_gripper(self, direction):
        """Inicia el gripper en la dirección especificada (1=cerrar, -1=abrir)."""
        self.gripper_running = True
        self._send_gripper_loop(direction)

    def stop_gripper(self):
        """Detiene el gripper."""
        self.gripper_running = False
        if self._after_id:
            self.after_cancel(self._after_id)

    def _send_gripper_loop(self, direction):
        """Loop que envía comandos del gripper mientras esté presionado."""
        if self.gripper_running:
            self.ros.publish_gripper(direction)  # 1 = cerrar, -1 = abrir
            self._after_id = self.after(100, lambda: self._send_gripper_loop(direction))

    def gripper(self):
        # Método original mantenido por compatibilidad (ahora vacío)
        pass

    def home(self):
        pass

    def volver_menu(self):
        self.destroy()
        v = VentanaPrincipal(self.ros)
        v.mainloop()

    def on_close(self):
        try:
            try:
                self.ros.get_logger().info("Cerrando ventana UpperBody")
            except Exception:
                pass
        finally:
            self.destroy()

    # ---------- Update coords display (thread-safe) ----------
    def update_coords(self, x, y, z):
        def _apply():
            vals = [x, y, z]
            for lbl, v in zip(self.info_labels, vals):
                try:
                    lbl.configure(text=f"{float(v):.4f}")
                except Exception:
                    lbl.configure(text=str(v))
        try:
            self.after(0, _apply)
        except Exception:
            _apply()

    # ---------- NUEVO: Update transformation matrix display ----------
    def update_T_matrix(self, T):
        """
        Actualiza las 16 celdas con la matriz de transformación 4x4.
        Acepta cualquier array-like convertible a (4,4).
        """
        def _apply():
            try:
                T_array = np.array(T, dtype=float).reshape(4, 4)
                for i in range(4):
                    for j in range(4):
                        try:
                            self.T_labels[i][j].configure(text=f"{T_array[i, j]:.3f}")
                        except Exception:
                            self.T_labels[i][j].configure(text=str(T_array[i, j]))
            except Exception as e:
                print(f"[UI] Error actualizando matriz T: {e}")
                # Si hay error, mostrar matriz identidad
                for i in range(4):
                    for j in range(4):
                        val = 1.0 if i == j else 0.0
                        try:
                            self.T_labels[i][j].configure(text=f"{val:.3f}")
                        except Exception:
                            self.T_labels[i][j].configure(text=str(val))
        
        try:
            self.after(0, _apply)
        except Exception:
            _apply()


# ------------------ main ------------------
def main():
    rclpy.init()
    ros = Ros2Bridge(topic_name=TOPIC_NAME)
    app = VentanaPrincipal(ros)
    app.mainloop()
    if rclpy.ok():
        try:
            ros.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

if __name__ == "__main__":
    main()
