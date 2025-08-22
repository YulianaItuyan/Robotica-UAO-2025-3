#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import customtkinter as ctk
from PIL import Image  # lo dejamos por si más adelante cargas imágenes
import numpy as np

# ====== CONFIG ======
TOPIC_NAME = '/cmd_deg'            # tópico donde publicamos los grados
JOINT_COUNT = 3
# Longitudes de eslabón (ajusta a tus medidas reales)
LINK_LENGTHS = [0.35, 0.2, 0.2]  # en metros
# ====================

# Estilo global UI
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


# ------------------ FUNCIONES DE CINEMÁTICA DIRECTA ------------------
def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s, 0.0],
                  [s,  c, 0.0],
                  [0.0, 0.0, 1.0]])
    return R

def rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[ c, 0.0,  s],
                  [0.0, 1.0, 0.0],
                  [-s, 0.0,  c]])
    return R

def to_hom(R, p):
    T = np.eye(4)  #Crea una matriz identidad 4x4
    T[:3,:3] = R    # Asigna la rotación a la parte superior izquierda
    T[:3, 3] = p    # Asigna la posición al vector de traslación
    # T es ahora una matriz homogénea 4x4
    return T

def Tx(L):
    T = np.eye(4)
    T[2,3] = L
    return T

def forward_kinematics(q_deg, L):
    """
    q_deg: [q1_deg, q2_deg, q3_deg] in degrees (already shifted if needed)
    L:     [L1, L2, L3] link lengths
    Model: T = Rz(q1) * Tx(L1) * Ry(q2) * Tx(L2) * Ry(q3) * Tx(L3)
    Returns: p (3,), R (3,3), T (4,4)
    """
    q = np.deg2rad(np.array(q_deg, dtype=float))
    L1, L2, L3 = L

    T01 = to_hom(rot_z(q[0]), np.zeros(3)) @ Tx(L1)
    T12 = to_hom(rot_y(q[1]), np.zeros(3)) @ Tx(L2)
    T23 = to_hom(rot_y(q[2]), np.zeros(3)) @ Tx(L3)

    T03 = T01 @ T12 @ T23
    p = T03[:3, 3]
    R = T03[:3, :3]
    return p, R, T03
# --------------------------------------------------------------------


class Ros2Bridge(Node):
    """Nodo mínimo para publicar Float32MultiArray en /cmd_deg."""
    def __init__(self, topic_name=TOPIC_NAME):
        super().__init__('mi_br_gui_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, topic_name, 10)
        self.get_logger().info(f'Ros2Bridge listo para publicar en {topic_name}')

    def publish_degrees(self, angles_deg):
        """Publica una lista de floats (grados) en /cmd_deg."""
        if len(angles_deg) != JOINT_COUNT:
            self.get_logger().warn(f'publish_degrees: se esperaban {JOINT_COUNT} valores, llegaron {len(angles_deg)}')
            return
        msg = Float32MultiArray()
        # Aseguramos float
        msg.data = [float(a) for a in angles_deg]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado en {TOPIC_NAME}: {msg.data}')


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

        # Radio buttons (opcional)
        self.arm_choise = ctk.IntVar(value=0)
        rb1 = ctk.CTkRadioButton(top_frame, text="Brazo Izquierdo", variable=self.arm_choise, value=1,
                                 text_color="white", font=("Arial", 20))
        rb2 = ctk.CTkRadioButton(top_frame, text="Brazo Derecho", variable=self.arm_choise, value=2,
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
        sliders_frame = ctk.CTkFrame(self, fg_color="transparent")
        sliders_frame.pack(fill="x", padx=10, pady=10)

        sliders_frame.grid_columnconfigure(0, weight=1)
        sliders_frame.grid_columnconfigure(1, weight=0)
        sliders_frame.grid_columnconfigure(2, weight=0)
        sliders_frame.grid_columnconfigure(3, weight=0)
        sliders_frame.grid_columnconfigure(4, weight=0)
        sliders_frame.grid_columnconfigure(5, weight=0)
        sliders_frame.grid_columnconfigure(6, weight=1)

        self.slider_vars = []  # guardamos las variables para leer en "confirmar"

        for i in range(3):
            lbl = ctk.CTkLabel(sliders_frame, text=f"J{i+1}", text_color="#737373", font=("Arial", 15))
            lbl.grid(row=i, column=1, padx=5, pady=5, sticky="w")

            # Valor por defecto colocado en 90 -> esto representa posición "neutra" (0° real en URDF)
            slider_value = ctk.DoubleVar(value=90)
            self.slider_vars.append(slider_value)

            value_lbl = ctk.CTkLabel(sliders_frame, textvariable=slider_value, text_color="white", font=("Arial", 15))
            value_lbl.grid(row=i, column=2, padx=5, pady=5, sticky="n")

            slider = ctk.CTkSlider(sliders_frame, from_=0, to=180, number_of_steps=180,
                                   variable=slider_value,
                                   command=lambda val, var=slider_value: var.set(int(float(val))))
            slider.grid(row=i, column=3, padx=5, pady=5, sticky="ew")

            range_lbl = ctk.CTkLabel(sliders_frame, text="0-180", text_color="#737373", font=("Arial", 10))
            range_lbl.grid(row=i, column=4, padx=5, pady=5, sticky="e")

        # === Botones Confirmar / Gripper ===========
        debajosliders_frame = ctk.CTkFrame(self, fg_color="transparent")
        debajosliders_frame.pack(fill="x", padx=10, pady=10)

        debajosliders_frame.grid_columnconfigure(0, weight=1)
        debajosliders_frame.grid_columnconfigure(1, weight=0)
        debajosliders_frame.grid_columnconfigure(2, weight=0)
        debajosliders_frame.grid_columnconfigure(3, weight=0)
        debajosliders_frame.grid_columnconfigure(4, weight=1)

        Confirmar_btn = ctk.CTkButton(debajosliders_frame, text="CONFIRMAR",
                                      command=self.confirmar,
                                      fg_color="#737373", text_color="white",
                                      corner_radius=10, font=("Arial", 20),
                                      width=50, height=75, hover_color="#838181")
        Confirmar_btn.grid(row=0, column=1, padx=10)

        gripper_btn = ctk.CTkButton(debajosliders_frame, text="GRIPPER",
                                    command=self.gripper,
                                    fg_color="#737373", text_color="white",
                                    corner_radius=10, font=("Arial", 20),
                                    width=50, height=75, hover_color="#838181")
        gripper_btn.grid(row=0, column=3, padx=10)

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
    def confirmar(self):
        # Obtenemos los grados directamente (0..180). Tu nodo 'joint_commander_deg'
        # ya espera grados en /cmd_deg y hace el shift (-90) internamente.
        angles_deg = [float(v.get()) for v in self.slider_vars]
        self.ros.publish_degrees(angles_deg)

        # ---------------------------------------------------------------------
        # Calcular la FK y actualizar las coordenadas en la interfaz
        # Aplicamos el mismo shift que usa joint_commander_deg (90° GUI -> 0° real)
        shifted = [a - 90.0 for a in angles_deg]
        # Usamos las longitudes definidas en LINK_LENGTHS
        p, R, T = forward_kinematics(shifted, LINK_LENGTHS)
        # p es (x,y,z) en las unidades de LINK_LENGTHS (m)
        self.update_coords(p[0], p[1], p[2])
        # ---------------------------------------------------------------------

    def stop(self):
        # Publicar posición neutra (90°, que el otro nodo convertirá a 0° reales)
        neutral = [90.0 for _ in range(JOINT_COUNT)]
        self.ros.publish_degrees(neutral)

        # Actualizar la FK para la posición neutra también
        shifted = [a - 90.0 for a in neutral]
        p, R, T = forward_kinematics(shifted, LINK_LENGTHS)
        self.update_coords(p[0], p[1], p[2])

    def gripper(self):
        
        # Placeholder para manejo del gripper (otro tópico si lo necesitas)
        pass

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

