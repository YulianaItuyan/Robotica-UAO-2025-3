# mi_br/mi_br/mi_br_interfaz.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import customtkinter as ctk
from PIL import Image  # lo dejamos por si más adelante cargas imágenes

# ====== CONFIG ======
TOPIC_NAME = '/cmd_deg'            # tópico donde publicamos los grados
JOINT_COUNT = 3
# ====================

# Estilo global UI
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


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
        self.geometry("650x600")
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

        # === Botones inferiores ====================
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

    def stop(self):
        # Publicar posición neutra (90°, que el otro nodo convertirá a 0° reales)
        neutral = [90.0 for _ in range(JOINT_COUNT)]
        self.ros.publish_degrees(neutral)

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
            self.get_logger().info("Cerrando ventana UpperBody")
        finally:
            self.destroy()


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

