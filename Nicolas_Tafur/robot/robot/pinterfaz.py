#!/usr/bin/env python3
#codigo1
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



ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

def dh_standard(theta, d, a, alpha):
    """
    Matriz de transformación homogénea 4x4 usando parámetros DH estándar.
    """
    
    alpha = np.deg2rad(alpha)
    theta = np.deg2rad(theta)

    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)

    T = np.array([
        [ ct, -ca*st,  st*sa, a*ct ],
        [ st,  ct*ca, -sa*ct, a*st ],
        [  0,     sa,     ca,    d ],
        [  0,      0,      0,    1 ]
    ], dtype=float)

    return T
    

# ------------------ DH FUNCTIONS (denavit) ------------------

def fk_from_dh(q,arm):
    """

    Calcula la cinemática directa con d y a fijos.
    q: lista de ángulos variables [theta1, theta3, theta4] en grados.
    """
    if arm == 2:
        # Brazo izquierdo
        dh_table = [
        (q[0],   -0.03,  -0.01, 0 ),   # link 1 -> θ1 variable
        (90,     0.0, 0.0,  90),   # link 2 -> θ2 fijo = 90°
        (q[1],   0.03, 0.105, 0),  # link 3 -> θ3 variable
        (q[2],  -0.025, 0.16,  0)  # link 4 -> θ4 variable ]
         ]
    else: 
        # Brazo derecho
        dh_table = [
        (q[0],  -3, -1, 0 ),   # link 1 -> θ1 variable
        (90,     0.0, 0.0,  90),   # link 2 -> θ2 fijo = 90°
        (q[1],   3, 10, 0),  # link 3 -> θ3 variable
        (q[2],  -2.0, 160, 0)  # link 4 -> θ4 variabl
        ]
    
    # --- Producto de transformaciones ---
    T = np.eye(4)
    for (theta, d, a, alpha) in dh_table:
        T = T @ dh_standard(theta, d, a, alpha, )

    return T

   

    # --- Parámetros fijos (a, alpha, d) ---
    # Estructura: (a, alpha, d, theta)
    

    # --- Producto de transformaciones ---
    T = np.eye(4)
    for (theta, d, a, alpha) in dh_table:
        T = T @ dh_standard(theta, d, a, alpha, degrees=degrees)

    return T

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

        # LOWER BODY - DESHABILITADO
        self.boton2 = ctk.CTkButton(self,
                                   text="LOWER BODY",
                                   command=self.boton_deshabilitado,  # Función que no hace nada
                                   fg_color="#404040",  # Color más oscuro para indicar deshabilitado
                                   text_color="#808080",  # Texto gris
                                   corner_radius=10,
                                   font=("Arial", 20),
                                   width=225, height=75,
                                   hover_color="#404040",  # Sin cambio en hover
                                   state="disabled")  # Estado deshabilitado
        self.boton2.grid(row=2, column=0, pady=10)

        # GEARS - DESHABILITADO
        self.boton3 = ctk.CTkButton(self,
                                   text="GEARS",
                                   command=self.boton_deshabilitado,  # Función que no hace nada
                                   fg_color="#404040",  # Color más oscuro para indicar deshabilitado
                                   text_color="#808080",  # Texto gris
                                   corner_radius=10,
                                   font=("Arial", 20),
                                   width=225, height=75,
                                   hover_color="#404040",  # Sin cambio en hover
                                   state="disabled")  # Estado deshabilitado
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

    def boton_deshabilitado(self):
        """Función placeholder para botones deshabilitados - no hace nada"""
        print("Esta opción no está disponible aún")
        pass

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
        self.geometry("650x800")
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

        # === Sliders con botones +/- ===============================
        sliders_frame = ctk.CTkFrame(self, fg_color="transparent")
        sliders_frame.pack(fill="x", padx=10, pady=10)

        # Ajusta columnas: deja un espacio de expansión al final
        sliders_frame.grid_columnconfigure(0, weight=1)
        sliders_frame.grid_columnconfigure(1, weight=0)  # etiqueta J#
        sliders_frame.grid_columnconfigure(2, weight=0)  # valor actual
        sliders_frame.grid_columnconfigure(3, weight=1)  # slider
        sliders_frame.grid_columnconfigure(4, weight=0)  # rango
        sliders_frame.grid_columnconfigure(5, weight=0)  # botón -
        sliders_frame.grid_columnconfigure(6, weight=0)  # botón +
        sliders_frame.grid_columnconfigure(7, weight=1)  # espacio

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

            # Botón "−" (decrementa 1)
            btn_minus = ctk.CTkButton(
                sliders_frame, text="−",
                width=28, height=28,
                fg_color="#5e5e5e", hover_color="#6b6b6b",
                text_color="white",
                command=lambda idx=i, c=cfg: self._nudge_slider(idx, -1, c["min"], c["max"])
            )
            btn_minus.grid(row=i, column=5, padx=(8,4), pady=5)

            # Botón "+" (incrementa 1)
            btn_plus = ctk.CTkButton(
                sliders_frame, text="+",
                width=28, height=28,
                fg_color="#5e5e5e", hover_color="#6b6b6b",
                text_color="white",
                command=lambda idx=i, c=cfg: self._nudge_slider(idx, +1, c["min"], c["max"])
            )
            btn_plus.grid(row=i, column=6, padx=(4,8), pady=5)

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

        # ------------------ MATRIZ DE TRANSFORMACIÓN HOMOGÉNEA 4x4 ------------------
        matrix_frame = ctk.CTkFrame(self, fg_color="transparent")
        matrix_frame.pack(fill="both", padx=10, pady=2)

        matrix_frame.grid_columnconfigure(0, weight=1)
        matrix_frame.grid_columnconfigure(5, weight=1)

        # cabeceras de columnas
        header = ["", "", "", "", ""]
        for j, h in enumerate(header):
            c = ctk.CTkLabel(matrix_frame, text=h, text_color="#737373", font=("Arial", 14))
            c.grid(row=1, column=j, padx=6, pady=2)

        # etiquetas 4x4 para los valores de T
        self.T_labels = []
        for i in range(4):
            row_labels = []
            # cabecera de fila
            r = ctk.CTkLabel(matrix_frame, text=f"", text_color="#737373", font=("Arial", 14))
            r.grid(row=2+i, column=0, padx=6, pady=2, sticky="e")
            for j in range(4):
                val = ctk.CTkLabel(matrix_frame, text="0.000",
                                   text_color="white", font=("Arial", 14),
                                   fg_color="#2B2B2B", corner_radius=6,
                                   width=90, height=28)
                val.grid(row=2+i, column=1+j, padx=4, pady=4)
                row_labels.append(val)
            self.T_labels.append(row_labels)
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
        # Aplicar correcciones específicas similar al código 2
        theta1 = sliders[0] -90
        theta3 = sliders[1] -180
        theta4 = sliders[2]     # theta4 correction
        
        
        try:
            T = fk_from_dh([theta1, theta3, theta4],arm) # usa ángulos corregidos
            # Update transformation matrix display
            self.update_T(T)
        except Exception as e:
            print(f"[FK-DH] Error calculando FK: {e}")

    def stop(self):
        
        pass

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
        neutral = [90, 90, 20,   90, 90, 20]

        # Actualizar estado interno
        self.all_joints = neutral.copy()

        # Publicar al tópico
        self.ros.publish_degrees(self.all_joints)

        # --- Actualizar sliders del brazo activo ---
        arm = int(self.arm_choice.get())
        start = 0 if arm == 1 else 3
        for i in range(3):
            self.slider_vars[i].set(self.all_joints[start + i])

        # --- Calcular y mostrar matriz de transformación ---
        sliders = self.read_sliders()  # lee los 3 sliders ya actualizados

        theta1 = sliders[0] -90
        theta3 = sliders[1] -180
        theta4 = sliders[2]     # theta4 correction
        
        
        try:
            T = fk_from_dh([theta1, theta3, theta4],arm) # usa ángulos corregidos
            # Update transformation matrix display
            self.update_T(T)
        except Exception as e:
            print(f"[FK-DH] Error calculando FK: {e}")
        

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

    def _nudge_slider(self, idx, delta, minv, maxv):
        """Incrementa/decrementa el slider idx en 'delta', respetando su rango."""
        val = float(self.slider_vars[idx].get())
        new_val = max(minv, min(maxv, val + delta))
        self.slider_vars[idx].set(int(round(new_val)))

    def update_T(self, T):
        """
        Actualiza las 16 celdas con la matriz de transformación 4x4.
        Se usa self.after(0,...) para ser thread-safe si se llama desde otro hilo.
        """
        try:
            T = np.array(T, dtype=float).reshape(4, 4)
        except Exception as e:
            print(f"[UI] Matriz T inválida: {e}")
            return

        def _apply():
            for i in range(4):
                for j in range(4):
                    try:
                        self.T_labels[i][j].configure(text=f"{T[i, j]:.3f}")
                    except Exception:
                        self.T_labels[i][j].configure(text=str(T[i, j]))
        try:
            self.after(0, _apply)
        except Exception:
            _apply()

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