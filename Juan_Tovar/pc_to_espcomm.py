import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import socket
import struct
import threading
import time

# Usamos como protocolo TCP, ya que se encarga de la retransmisión de datos en caso de fallar y permite tener una cola,
# de modo que si se envía una secuencia se garantiza el orden de llegada de los paquetes

class DegreesSubscriber(Node):
    def __init__(self):
        super().__init__('degree_subscriber')
        
        # TCP connection parameters
        self.declare_parameter('esp32_ip', '192.168.1.100')
        self.declare_parameter('esp32_port', 8080)
        self.esp32_ip = self.get_parameter('esp32_ip').get_parameter_value().string_value
        self.esp32_port = self.get_parameter('esp32_port').get_parameter_value().integer_value
        self.get_logger().info(f'Parámetros configurados:')
        self.get_logger().info(f'ESP32 IP: {self.esp32_ip}')
        self.get_logger().info(f'ESP32 Port: {self.esp32_port}')
        self.socket = None
        self.connection_lock = threading.Lock()
           
        # Manejo de la reconexión
        self.reconnect_delay = 1.0   
        self.max_reconnect_delay = 30.0  
        self.reconnect_attempts = 0
        self.last_reconnect_time = 0
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/cmd_deg',
            self.listener_callback,
            10)
        
        #Verificación de conexión al inicializar
        self.get_logger().info('Iniciando conexión a ESP')
        if not self.conectar_a_esp32():
            self.get_logger().error('No se pudo conectar a ESP')
            self.get_logger().error('Verifique que ESP32 esté encendido y en la red')
            raise RuntimeError(f'No se pudo conectar a ESP en {self.esp32_ip}:{self.esp32_port}')
        
        #Revisión de conexión periodica
        self.timer = self.create_timer(180.0, self.verificar_conexión)
        
        # Función para checkear la conexión periodicamente
    def set_keepalive(self, sock, after_idle_sec=60, interval_sec=7, max_fails=3):
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1) # Activa la función del socket que permite el checkeo
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, after_idle_sec) # Tiempo a esperar antes del checkeo 
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, interval_sec) # Cada cuanto checkea
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, max_fails) # Maximo numero de fallas de checkeo
            self.get_logger().info(f'Linux keepalive: idle={after_idle_sec}s, interval={interval_sec}s, max_fails={max_fails}')
        
    def conectar_a_esp32(self):
        try:
            with self.connection_lock:
                if self.socket:
                    self.socket.close()
                
                self.get_logger().info(f'Conectando a ESP32 en {self.esp32_ip}:{self.esp32_port}...')
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #Se define el tipo de socket a usar, estos son los predeterminados y supuestamente los que mejor funcionan xd
                self.socket.settimeout(5.0) # Tiempo de espera de conexión
                self.socket.connect((self.esp32_ip, self.esp32_port))
                # Configuración del socket
                self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Esta función deshabilita una función de TCP que busca enviar múltiples paquetes en uno solo                
                self.set_keepalive(self.socket)
                self.get_logger().info(f'Conectado con ESP32 en {self.esp32_ip}:{self.esp32_port}')
                return True
                
        except Exception as e:
            self.get_logger().error(f'Fallo al conectarse con ESP32: {e}')
            self.socket = None
            
            #Intento de reconexiones
            self.reconnect_attempts += 1
            self.reconnect_delay = min(self.reconnect_delay * 1.5, self.max_reconnect_delay)
            self.last_reconnect_time = time.time()
            return False
        
    def diagnostico_conexion(self):
        #Revisamos el estado del socket
        if not self.socket:
            return False
            
        try:
            error = self.socket.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR) #Usamos SO_ERROR para que nos indica si hay fallos en el socket y cual
            if error:
                self.get_logger().warn(f'Error en socket: {error}')
                return False
            return True
        except Exception as e:
            self.get_logger().warn(f'Diagnostico de conexión fallido: {e}')
            return False
           
    def enviar_grados_a_esp32(self, float_list):
        if not self.socket:
            self.get_logger().warn('No hay conexión a ESP32')
            return False
            
        try:
            with self.connection_lock:
                # Es necesario pasar los datos a binario para su transmision
                num_floats = len(float_list)
                if num_floats > 255:
                    self.get_logger().error('Maximo de floats es 255')
                    return False
                
                # Transformamos los datos de float a bytes y añadimos un byte sin signo que indique cuantos bytes debe recibir la ESP
                format_string = f'B{num_floats}f'
                packed_data = struct.pack(format_string, num_floats, *float_list)
                self.get_logger().info("Packed data (hex):", packed_data.hex())
                self.get_logger().info("Length:", len(packed_data))
                
                # Ponemos un timeout
                original_timeout = self.socket.gettimeout()
                self.socket.settimeout(2.0)
                
                try:
                    self.socket.sendall(packed_data)
                    return True
                finally:
                    # Restauramos el timeout
                    self.socket.settimeout(original_timeout)
                
        except socket.timeout:
            self.get_logger().error('Timeout red congestionada')
            self.socket = None
            return False
        except socket.error as e:
            self.get_logger().error(f'Socket error: {e}')
            self.socket = None
            return False
        except Exception as e:
            self.get_logger().error(f'Error enviando los datos: {e}')
            return False
    
    def should_attempt_reconnect(self):
        #Verificar tiempo para reconexión
        if not self.last_reconnect_time:
            return True
        return (time.time() - self.last_reconnect_time) >= self.reconnect_delay
    
    def verificar_conexión(self):
        if not self.socket or not self.diagnostico_conexion():
            if self.should_attempt_reconnect():
                self.get_logger().info(f'Conexión perdida, reintentando... (intento {self.reconnect_attempts + 1})')
                if self.conectar_a_esp32():
                    self.get_logger().info('Reconexión exitosa')
                else:
                    self.get_logger().warn(f'Reconexión falló, siguiente intento en {self.reconnect_delay:.1f}s')
            else:
                remaining_time = self.reconnect_delay - (time.time() - self.last_reconnect_time)
                self.get_logger().debug(f'Esperando {remaining_time:.1f}s antes del próximo intento de reconexión')
        else:
            # Reseteo de reconexión a buen diagnostico
            if self.reconnect_attempts > 0:
                self.reconnect_attempts = 0
                self.reconnect_delay = 1.0
            self.get_logger().debug('Conexión activa y saludable')
            
    def listener_callback(self, msg):
        self.get_logger().info('Grados recibidos: "%s"' % msg.data)
        
        # Enviar los floats
        success = self.enviar_grados_a_esp32(list(msg.data))
        if success:
            self.get_logger().info('Datos enviados correctamente')
        else:
            self.get_logger().warn('Fallo en el envío')
            # Intentar reconexión
            if self.should_attempt_reconnect():
                self.get_logger().info('Intentando reconexión inmediata')
                if self.conectar_a_esp32():
                    self.get_logger().info('Reconexión exitosa tras fallo de envío')
            
    def destroy_node(self):
        if self.socket:
            self.socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    degree_subscriber = DegreesSubscriber()
    
    try:
        rclpy.spin(degree_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        degree_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()