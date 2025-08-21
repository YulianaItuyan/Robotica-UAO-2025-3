from PIL import Image
import customtkinter as ctk

# Config global (una sola vez)
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


class VentanaPrincipal(ctk.CTk):
    def __init__(self,parent):
        self.parent = parent  # Guardamos la referencia al padre
        super().__init__()
        self.title("INTERFAZ CENTAURO")
        self.geometry("634x563")
        self.resizable(False,False)

        self.grid_rowconfigure((0,1,2,3,4), weight=1)
        self.grid_columnconfigure(0, weight=1)

        # Boton para el UPPER BODY 

        self.boton1 = ctk.CTkButton(self, 
                              text="UPPER BODY", 
                             
                              fg_color="#737373",
                              text_color="white",
                              corner_radius=10,
                              font=("Arial", 20),
                              command=self.cambio_upper_body, 
                              width=225,
                              height=75,
                              hover_color="#838181")
        self.boton1.grid(row=0, column=0, pady=10)


        # Boton para el LOWER BODY

        self.boton2 = ctk.CTkButton(self, 
                              text="LOWER BODY",
                               # Cambiar a la función correspondiente
                              fg_color="#737373",
                              text_color="white",
                              corner_radius=10,
                              font=("Arial", 20),
                              command=self.cambio_lower_body,  # Cambiar a la función correspondiente
                              width=225,
                              height=75,
                              hover_color="#838181")
        self.boton2.grid(row=2, column=0, pady=10)

        # Boton para los GEARS

        self.boton3 = ctk.CTkButton(self, 
                              text="GEARS",
                                # Cambiar a la función correspondiente
                              fg_color="#737373",
                              text_color="white",
                              corner_radius=10,
                              font=("Arial", 20),
                              width=225,
                              height=75,
                              hover_color="#838181")
        self.boton3.grid(row=4, column=0, pady=10)

        # Lineas horizontales -----------------------------
        self.linea = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.linea.grid(row=1, column=0, sticky="ew", padx=10, pady=10)

        self.linea2 = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.linea2.grid(row=3, column=0, sticky="ew", padx=10, pady=10)

    
    def cambio_upper_body(self):
        self.withdraw()  # Hide VentanaPrincipal
        upper_body_window = UpperBody(self)
        upper_body_window.protocol("WM_DELETE_WINDOW", lambda: self.on_upper_body_close(upper_body_window))

    def on_upper_body_close(self, upper_body_window):
        upper_body_window.destroy()
        self.deiconify()


    def cambio_lower_body(self):
        self.withdraw()  # Hide VentanaPrincipal
        lower_body_window = LowerBody(self)
        lower_body_window.protocol("WM_DELETE_WINDOW", lambda: self.on_lower_body_close(lower_body_window))

    def on_lower_body_close(self, lower_body_window):
        lower_body_window.destroy()
        self.deiconify()
            


class UpperBody(ctk.CTkToplevel):  # Change from CTk to CTkToplevel
    def __init__(self, parent):  # Add parent parameter
        self.parent = parent
        super().__init__(parent)
        self.title("UPPER BODY")
        self.geometry("750x700")
        self.resizable(False,False)

        # === Frame superior ========================
        top_frame = ctk.CTkFrame(self, fg_color="transparent", height=50)
        #top_frame.place(x=0, y=0, relwidth=1)
        top_frame.pack(fill="x", padx=10, pady=10)

        # Radio buttons para seleccionar el modo ----
        self.arm_choise = ctk.IntVar(value=0)  # Valor por defecto
        rb1 = ctk.CTkRadioButton(top_frame, 
                                 text="Brazo Izquierdo", 
                                 variable=self.arm_choise, 
                                 value=1,
                                 text_color="white",
                                 font=("Arial", 20))
        rb2 = ctk.CTkRadioButton(top_frame, 
                                 text="Brazo Derecho", 
                                 variable=self.arm_choise, 
                                 value=2,
                                 text_color="white",
                                 font=("Arial", 20))
        rb1.grid(row=0, column=0, sticky="w", pady=2)
        rb2.grid(row=1, column=0, sticky="w", pady=2)



        # Boton records -------------------
        self.boton_records = ctk.CTkButton(top_frame,
                                            text="RECORDS",
                                            fg_color="#737373",
                                            text_color="white",
                                            corner_radius=7,
                                            font=("Arial", 20),
                                            width=120,
                                            height=20,
                                            hover_color="#838181")
        self.boton_records.place(relx=1, x=-10, rely=0.5, anchor="e")
        #self.boton_records.grid(row=0, column=3, rowspan=2, padx=10, sticky="e")

        # === Divisoria horizontal =================
        self.divisoria = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.divisoria.pack(fill="x", padx=10, pady=20)

        # === Frame sliders =========================
        sliders_frame = ctk.CTkFrame(self, fg_color="transparent")
        sliders_frame.pack(fill="x", padx=10, pady=10)


        sliders_frame.grid_columnconfigure(0, weight=1)  # columna vacía izquierda
        sliders_frame.grid_columnconfigure(1, weight=0)  # label Jx
        sliders_frame.grid_columnconfigure(2, weight=0)  # valor encima del slider (si lo pones)
        sliders_frame.grid_columnconfigure(3, weight=0)  # slider
        sliders_frame.grid_columnconfigure(4, weight=0)  # rango
        sliders_frame.grid_columnconfigure(5, weight=0)  # OK
        sliders_frame.grid_columnconfigure(6, weight=1)  # columna vacía derecha


        self.slider_vars = []   # lista para guardar todas las variables

        for i in range(3):
            lbl = ctk.CTkLabel(sliders_frame, 
                            text=f"J{i+1}", 
                            text_color="#737373",
                            font=("Arial",15))
            lbl.grid(row=i, column=1, padx=5, pady=5, sticky="w")

            # - var para el valor del slider --------
            slider_value = ctk.DoubleVar(value=0)
            self.slider_vars.append(slider_value)   # guardar la referencia

            # - Label para mostrar el valor ---------
            value_lbl = ctk.CTkLabel(sliders_frame, 
                                    textvariable=slider_value, 
                                    text_color="white", font=("Arial", 15))
            value_lbl.grid(row=i, column=2, padx=5, pady=5, sticky="n")

            slider = ctk.CTkSlider(sliders_frame, from_=0, to=180, number_of_steps=180,
                                variable=slider_value,
                                command=lambda val, var=slider_value: var.set(int(float(val))))
            slider.grid(row=i, column=3, padx=5, pady=5, sticky="ew")

            range_lbl = ctk.CTkLabel(sliders_frame, 
                                    text="0-180", 
                                    text_color="#737373",
                                    font=("Arial",10))
            range_lbl.grid(row=i, column=4, padx=5, pady=5, sticky="e")


            # ok_btn = ctk.CTkButton(sliders_frame, 
            #                                 text="OK", 
            #                                 command=lambda j=i: self.enviar_valores(j),
            #                                 fg_color="#737373",
            #                                 text_color="white",
            #                                 corner_radius=10,
            #                                 font=("Arial", 12),
            #                                 width=50,
            #                                 height=30,
            #                                 hover_color="#838181")
            # ok_btn.grid(row=i, column=5, padx=5, pady=5, sticky="e")

        debajosliders_frame = ctk.CTkFrame(self, fg_color="transparent")
        debajosliders_frame.pack(fill="x", padx=10, pady=10)


        debajosliders_frame.grid_columnconfigure(0, weight=1)  # espacio izquierdo
        debajosliders_frame.grid_columnconfigure(1, weight=0)  # CONFIRMAR
        debajosliders_frame.grid_columnconfigure(2, weight=0)  # espacio entre botones
        debajosliders_frame.grid_columnconfigure(3, weight=0)  # GRIPPER
        debajosliders_frame.grid_columnconfigure(4, weight=1)  # espacio derecho


        Confirmar_btn = ctk.CTkButton(debajosliders_frame,
                                       text="CONFIRMAR",
                                       #command=self.confirmar,
                                       fg_color="#737373",
                                       text_color="white",
                                       corner_radius=10,
                                       font=("Arial", 20),
                                       command=self.mostrar_valores,
                                       width=50,
                                       height=75,
                                       hover_color="#838181")
        Confirmar_btn.grid(row=0, column=1, padx=10)

        gripper_btn = ctk.CTkButton(debajosliders_frame,
                                       text="GRIPPER",
                                       #command=self.gripper,
                                       fg_color="#737373",
                                       text_color="white",
                                       corner_radius=10,
                                       font=("Arial", 20),
                                       width=50,
                                       height=75,
                                       hover_color="#838181")
        gripper_btn.grid(row=0, column=3, padx=10)

        # === Divisoria horizontal entre sliders ===
        self.divisoria2 = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.divisoria2.pack(fill="x", padx=10, pady=20)

        # === Frame coord =========================

        coord_frame = ctk.CTkFrame(self, fg_color="transparent")
        coord_frame.pack(fill="both", padx=10, pady=10)

        coord_frame.grid_columnconfigure(0, weight=1)  # espacio izquierdo
        coord_frame.grid_columnconfigure(1, weight=0)  # J1 info
        coord_frame.grid_columnconfigure(2, weight=0)  # J2 info  
        coord_frame.grid_columnconfigure(3, weight=0)  # J3 info
        coord_frame.grid_columnconfigure(4, weight=1)  # espacio derecho

        # Store references to update the information later
        self.info_labels = []
        coord = ["X", "Y", "Z"]

        for i in range(3):
            # Joint label (J1, J2, J3)
            lbl = ctk.CTkLabel(coord_frame,
                            text= f"{coord[i]:}",
                            text_color="#737373",
                            font=("Arial", 15))
            lbl.grid(row=0, column=i+1, padx=5, pady=5)
            
            # Information display space
            info_label = ctk.CTkLabel(coord_frame,
                                    text="0.0",  # Default value
                                    text_color="white",
                                    font=("Arial", 14),
                                    fg_color="#2B2B2B",  # Background color
                                    corner_radius=5,
                                    width=80,
                                    height=30)
            info_label.grid(row=1, column=i+1, padx=5, pady=5)
            
            # Store reference for later updates
            self.info_labels.append(info_label)

            
        

        self.divisoria3 = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.divisoria3.pack(fill="x", padx=10, pady=20)

        

        # === Frame inferior ======================
        bottom_frame = ctk.CTkFrame(self, fg_color="transparent")
        bottom_frame.pack(fill="both", padx=10, pady=10)

        bottom_frame.grid_columnconfigure(0, weight=1)  # espacio izquierdo
        bottom_frame.grid_columnconfigure(1, weight=0)  # CONFIRMAR
        bottom_frame.grid_columnconfigure(2, weight=0)  # espacio entre botones
        bottom_frame.grid_columnconfigure(3, weight=0)  # GRIPPER
        bottom_frame.grid_columnconfigure(4, weight=1)  # espacio derecho

        
        stop_btn = ctk.CTkButton(bottom_frame,
                                       text="STOP",
                                       #command=self.stop,
                                       fg_color="#737373",
                                       text_color="white",
                                       corner_radius=90,
                                       font=("Arial", 20),
                                       width=80,
                                       height=100,
                                       hover_color="#838181")
        stop_btn.grid(row=0, column=2, padx=10, pady=20)

        back_btn = ctk.CTkButton(bottom_frame,
                                       text="BACK",
                                       #command=self.stop,
                                       fg_color="#737373",
                                       text_color="white",
                                       corner_radius=20,
                                       font=("Arial", 20),
                                       command=self.close_and_return,
                                       width=80,
                                       height=70,
                                       hover_color="#838181")
        back_btn.grid(row=0, column=1, padx=10, pady=20)

        home_btn = ctk.CTkButton(bottom_frame,
                                       text="HOME",
                                       #command=self.stop,
                                       fg_color="#737373",
                                       text_color="white",
                                       corner_radius=20,
                                       font=("Arial", 20),
                                       width=80,
                                       height=70,
                                       hover_color="#838181")
        home_btn.grid(row=0, column=3, padx=10, pady=20)

    def mostrar_valores(self):
        print([var.get() for var in self.slider_vars])

    def close_and_return(self):
        """Cerrar UpperBody y volver a VentanaPrincipal"""
        self.destroy()           # Cerrar ventana actual
        self.parent.deiconify()


class LowerBody(ctk.CTkToplevel):  
    def __init__(self,parent):
        self.parent = parent  
        super().__init__(parent)  # Pass parent to CTkToplevel
        self.title("LOWER BODY")
        self.geometry("750x700")
        self.resizable(False, False)

        # === Frame superior ========================
        top_frame = ctk.CTkFrame(self, fg_color="transparent", height=50)
        #top_frame.place(x=0, y=0, relwidth=1)
        top_frame.pack(fill="x", padx=10, pady=10)
        top_frame.grid_columnconfigure(0, weight=1)  # izquierda
        top_frame.grid_columnconfigure(1, weight=1)  # espacio
        top_frame.grid_columnconfigure(2, weight=1)  # derecha


        # Radio buttons para seleccionar el modo ----
        self.arm_choise = ctk.IntVar(value=0)  # Valor por defecto
        rb1 = ctk.CTkRadioButton(top_frame, 
                                 text="Frontal Izquierda", 
                                 variable=self.arm_choise, 
                                 value=1,
                                 text_color="white",
                                 font=("Arial", 20))
        rb2 = ctk.CTkRadioButton(top_frame, 
                                 text="Frontal Derecha", 
                                 variable=self.arm_choise, 
                                 value=2,
                                 text_color="white",
                                 font=("Arial", 20))
        rb3 = ctk.CTkRadioButton(top_frame, 
                                 text="Trasera Derecha", 
                                 variable=self.arm_choise, 
                                 value=3,
                                 text_color="white",
                                 font=("Arial", 20))
        rb4 = ctk.CTkRadioButton(top_frame, 
                                 text="Trasera Izquierda", 
                                 variable=self.arm_choise, 
                                 value=4,
                                 text_color="white",
                                 font=("Arial", 20))
        
        rb1.grid(row=0, column=0, sticky="w", pady=2)
        rb2.grid(row=1, column=0, sticky="w", pady=2)
        rb3.grid(row=0, column=1, sticky="w", pady=2)
        rb4.grid(row=1, column=1, sticky="w", pady=2)
        



        # Boton records -------------------
        self.boton_records = ctk.CTkButton(top_frame,
                                            text="RECORDS",
                                            fg_color="#737373",
                                            text_color="white",
                                            corner_radius=7,
                                            font=("Arial", 20),
                                            width=120,
                                            height=20,
                                            hover_color="#838181")
        self.boton_records.place(relx=1, x=-10, rely=0.5, anchor="e")
        #self.boton_records.grid(row=0, column=3, rowspan=2, padx=10, sticky="e")

        # === Divisoria horizontal =================
        self.divisoria = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.divisoria.pack(fill="x", padx=10, pady=10)

        # === Frame sliders =========================
        sliders_frame = ctk.CTkFrame(self, fg_color="transparent", width=0, height=100)
        sliders_frame.pack(fill="x", padx=10, pady=10)
        


        sliders_frame.grid_columnconfigure(0, weight=1)  # columna vacía izquierda
        sliders_frame.grid_columnconfigure(1, weight=0)  # label Jx
        sliders_frame.grid_columnconfigure(2, weight=0)  # valor encima del slider (si lo pones)
        sliders_frame.grid_columnconfigure(3, weight=0)  # slider
        sliders_frame.grid_columnconfigure(4, weight=0)  # rango
        sliders_frame.grid_columnconfigure(5, weight=1)  # 
        sliders_frame.grid_columnconfigure(6, weight=0)  # columna vacía derecha
        sliders_frame.grid_columnconfigure(7, weight=1)
        sliders_frame.grid_columnconfigure(8, weight=0)  # columna vacía derecha

        self.slider_vars = []   # lista para guardar todas las variables

        for i in range(3):
            lbl = ctk.CTkLabel(sliders_frame, 
                            text=f"J{i+1}", 
                            text_color="#737373",
                            font=("Arial",15))
            lbl.grid(row=i, column=1, padx=5, pady=5, sticky="w")

            # - var para el valor del slider --------
            slider_value = ctk.DoubleVar(value=0)
            self.slider_vars.append(slider_value)   # guardar la referencia

            # - Label para mostrar el valor ---------
            value_lbl = ctk.CTkLabel(sliders_frame, 
                                    textvariable=slider_value, 
                                    text_color="white", font=("Arial", 15))
            value_lbl.grid(row=i, column=2, padx=5, pady=5, sticky="n")

            slider = ctk.CTkSlider(sliders_frame, from_=0, to=180, number_of_steps=180,
                                variable=slider_value,
                                command=lambda val, var=slider_value: var.set(int(float(val))))
            slider.grid(row=i, column=3, padx=5, pady=5, sticky="ew")

            range_lbl = ctk.CTkLabel(sliders_frame, 
                                    text="0-180", 
                                    text_color="#737373",
                                    font=("Arial",10))
            range_lbl.grid(row=i, column=4, padx=5, pady=5, sticky="e")

        
        # Botón confirmar debajo de los sliders
        Confirmar_btn = ctk.CTkButton(sliders_frame,
            text="CONFIRMAR",
            fg_color="#737373",
            text_color="white",
            corner_radius=10,
            font=("Arial", 20),
            command= self.mostrar_valores ,  # Aquí puedes poner la función que quieras
            width=120,
            height=50,
            hover_color="#838181")
        Confirmar_btn.grid(row=3, column=1, columnspan=4, pady=20)

        #Aca van las entradas para las coordenadas X, Y, Z de cinematica inversa

        self.info_entries = []  
        coord = ["X", "Y", "Z"]

        def validate_number(value):
            if value == "": return True
            try:
                num = float(value)
                return 0 <= num <= 999 and (len(value.split('.')[1]) <= 2 if '.' in value else True)
            except (ValueError, IndexError):
                return False

        
        for i in range(3):
            # Joint label (J1, J2, J3)
            lbl = ctk.CTkLabel(sliders_frame,
                            text=f"{coord[i]:}",
                            text_color="#737373",
                            font=("Arial", 15))
            lbl.grid(row=i , column=6, padx=5, pady=5)
            
            # Information entry space (instead of label)
            info_entry = ctk.CTkEntry(sliders_frame,
                                 
                                    text_color="white",
                                    font=("Arial", 14),
                                    fg_color="#2B2B2B",  # Background color
                                    border_color="#404040",  # Border color
                                    corner_radius=5,
                                    width=80,
                                    height=30,
                                    justify="center")  # Center the text
            
            info_entry.configure(validate="key", 
                    validatecommand=(info_entry.register(validate_number), '%P'))
            
            info_entry.grid(row=i, column=7, padx=5, pady=5)
            
            
            # Store reference for later updates
            self.info_entries.append(info_entry)

        # Botón confirmar debajo de las coordenadas para cinematica inversa
        Confirmar_btn2 = ctk.CTkButton(sliders_frame,
            text="CONFIRMAR",
            fg_color="#737373",
            text_color="white",
            corner_radius=10,
            font=("Arial", 20),
            command= self.get_all_info,  # Aquí puedes poner la función que quieras
            width=120,
            height=50,
            hover_color="#838181")
        Confirmar_btn2.grid(row=3, column=6, columnspan=4, pady=20)

    

        # === Divisoria horizontal entre sliders ===
        self.divisoria2 = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.divisoria2.pack(fill="x", padx=10, pady=10)

        # === Frame coord =========================

        coord_frame = ctk.CTkFrame(self, fg_color="transparent")
        coord_frame.pack(fill="both", padx=10, pady=10)

        coord_frame.grid_columnconfigure(0, weight=1)  # espacio izquierdo
        coord_frame.grid_columnconfigure(1, weight=0)  
        coord_frame.grid_columnconfigure(2, weight=0)   
        coord_frame.grid_columnconfigure(3, weight=0)  
        coord_frame.grid_columnconfigure(4, weight=1)  # espacio derecho

        # Store references to update the information later
        self.info_labels = []
        coord = ["X", "Y", "Z"]

        for i in range(3):
            # Joint label (J1, J2, J3)
            lbl = ctk.CTkLabel(coord_frame,
                            text= f"{coord[i]:}",
                            text_color="#737373",
                            font=("Arial", 15))
            lbl.grid(row=0, column=i+1, padx=5, pady=5)
            
            # Information display space
            info_label = ctk.CTkLabel(coord_frame,
                                    text="0.0",  # Default value
                                    text_color="white",
                                    font=("Arial", 14),
                                    fg_color="#2B2B2B",  # Background color
                                    corner_radius=5,
                                    width=80,
                                    height=30)
            info_label.grid(row=1, column=i+1, padx=5, pady=5)
            
            # Store reference for later updates
            self.info_labels.append(info_label)



        self.divisoria3 = ctk.CTkFrame(self, height=2, corner_radius=0, fg_color="#3B3B3B")
        self.divisoria3.pack(fill="x", padx=10, pady=20)

        # === Frame inferior ======================
        bottom_frame = ctk.CTkFrame(self, fg_color="transparent")
        bottom_frame.pack(fill="both", padx=10, pady=20)

        bottom_frame.grid_columnconfigure(0, weight=1)  # espacio izquierdo
        bottom_frame.grid_columnconfigure(1, weight=0)  
        bottom_frame.grid_columnconfigure(2, weight=0)  # espacio entre botones
        bottom_frame.grid_columnconfigure(3, weight=0)  
        bottom_frame.grid_columnconfigure(4, weight=1)  # espacio derecho

        
        stop_btn = ctk.CTkButton(bottom_frame,
                                       text="STOP",
                                       #command=self.stop,
                                       fg_color="#737373",
                                       text_color="white",
                                       corner_radius=90,
                                       font=("Arial", 20),
                                       width=80,
                                       height=100,
                                       hover_color="#838181")
        stop_btn.grid(row=0, column=2, padx=10, pady=20)

        back_btn = ctk.CTkButton(bottom_frame,
                                       text="BACK",
                                       #command=self.stop,
                                       fg_color="#737373",
                                       text_color="white",
                                       corner_radius=20,
                                       font=("Arial", 20),
                                       command=self.close_and_return,
                                       width=80,
                                       height=70,
                                       hover_color="#838181")
        back_btn.grid(row=0, column=1, padx=10, pady=20)

        home_btn = ctk.CTkButton(bottom_frame,
                                       text="HOME",
                                       #command=self.stop,
                                       fg_color="#737373",
                                       text_color="white",
                                       corner_radius=20,
                                       font=("Arial", 20),
                                       width=80,
                                       height=70,
                                       hover_color="#838181")
        home_btn.grid(row=0, column=3, padx=10, pady=20)

    def mostrar_valores(self):
        print([var.get() for var in self.slider_vars])

    def get_all_info(self):
        print([float(entry.get()) for entry in self.info_entries])

    def close_and_return(self):

        self.destroy()           # Cerrar ventana actual
        self.parent.deiconify()










if __name__ == "__main__":
    app = VentanaPrincipal(None)
    app.mainloop()