import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import paho.mqtt.client as mqtt
import json
import math
import time

class MQTTBridgeROS2(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_ros2')

        # Parámetros
        self.declare_parameter('mqtt_broker', 'test.mosquitto.org')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', 'robot/servo_cmd')
        self.declare_parameter('ros_topic', '/cmd_deg')
        self.declare_parameter('send_only_on_change', True)
        self.declare_parameter('reconnect_secs', 5.0)

        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = int(self.get_parameter('mqtt_port').value)
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        self.ros_topic = self.get_parameter('ros_topic').value
        self.send_only_on_change = bool(self.get_parameter('send_only_on_change').value)
        self.reconnect_secs = float(self.get_parameter('reconnect_secs').value)

        self.last_frame = None
        self.mqtt_connected = False

        # Configurar cliente MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        
        # Intentar conectar a MQTT
        self._connect_mqtt()

        # Suscripción ROS2
        self.ros_sub = self.create_subscription(
            Float32MultiArray, 
            self.ros_topic, 
            self.on_ros_cmd, 
            10
        )

        # Timer de reconexión MQTT
        self.reconnect_timer = self.create_timer(self.reconnect_secs, self._reconnect_if_needed)

        self.get_logger().info(f'MQTT Bridge iniciado:')
        self.get_logger().info(f'  - ROS Topic: {self.ros_topic}')
        self.get_logger().info(f'  - MQTT Broker: {self.mqtt_broker}:{self.mqtt_port}')
        self.get_logger().info(f'  - MQTT Topic: {self.mqtt_topic}')

    def _connect_mqtt(self):
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f'Conectando a MQTT broker: {self.mqtt_broker}:{self.mqtt_port}')
        except Exception as e:
            self.get_logger().error(f'Error conectando a MQTT: {e}')

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.mqtt_connected = True
            self.get_logger().info('✅ Conectado a MQTT broker')
        else:
            self.mqtt_connected = False
            self.get_logger().error(f' Error conectando a MQTT, código: {rc}')

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.mqtt_connected = False
        self.get_logger().warn(' Desconectado del MQTT broker')

    def _reconnect_if_needed(self):
        if not self.mqtt_connected:
            self.get_logger().info('Reintentando conexión MQTT...')
            self._connect_mqtt()

    @staticmethod
    def _sanitize_angle(x):
        """Limpia y valida ángulos entre 0-180°"""
        if x is None or (isinstance(x, float) and math.isnan(x)):
            return 90
        try:
            xi = int(round(float(x)))
        except Exception:
            xi = 90
        return max(0, min(180, xi))

    def on_ros_cmd(self, msg: Float32MultiArray):
        """Callback cuando llegan comandos desde ROS2"""
        if len(msg.data) != 3:
            self.get_logger().warn(f'Se esperaban 3 valores, llegaron {len(msg.data)}')
            return

        # Limpiar y validar ángulos
        a, b, c = [self._sanitize_angle(v) for v in msg.data]
        
        # Crear payload JSON
        payload = {
            "servo1": a,
            "servo2": b,
            "servo3": c,
            "timestamp": time.time()
        }
        
        payload_str = json.dumps(payload)

        # Evitar envío redundante
        if self.send_only_on_change and payload_str == self.last_frame:
            return

        self.last_frame = payload_str

        # Enviar por MQTT
        if not self.mqtt_connected:
            self.get_logger().warn('MQTT no conectado, reintentando...')
            self._connect_mqtt()
            return

        try:
            result = self.mqtt_client.publish(self.mqtt_topic, payload_str, qos=1)
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().info(f'📡 Enviado MQTT: S1={a}°, S2={b}°, S3={c}°')
            else:
                self.get_logger().error(f'Error publicando MQTT, código: {result.rc}')
        except Exception as e:
            self.get_logger().error(f'Excepción enviando MQTT: {e}')

    def destroy_node(self):
        """Cleanup al cerrar el nodo"""
        try:
            if self.mqtt_connected:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
                self.get_logger().info('MQTT desconectado')
        except Exception as e:
            self.get_logger().error(f'Error cerrando MQTT: {e}')
        super().destroy_node()

def main():
    rclpy.init()
    node = MQTTBridgeROS2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
