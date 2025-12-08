import time
from database_manager import DatabaseManager
import paho.mqtt.client as mqtt


#Se conecta a la base de datos, donde previamente creamos un usuario con
todos los permisos
db_manager = DatabaseManager('10.48.233.73', 'raspberry_2', 'weccat','tacoverde')


def on_message(client, userdata, msg):
  try:
    print("Se encontro un mensaje")
    msg = msg.payload.decode()
    #El mensje de mqtt se va a dividir cuando se encuentre un _
    #El mensaje se mando en formato sensorNombre_valor1_valor2
    msg_split = msg.split("_")
    sensor = msg_split[0].lower()
  if sensor == "adc":
    valor = msg_split[1]
    voltaje = msg_split[2]
    db_manager.insert_adc(valor, voltaje)
    print(f"Insertado en ADC: Valor= {valor}, Voltaje= {voltaje}")
  elif sensor == "acelerometro":
    valor_x = msg_split[1]
    valor_y = msg_split[2]
    valor_z = msg_split[3]
    db_manager.insert_acelerometro(valor_x, valor_y, valor_z)
    print(f"Insertado en Acelerómetro: X= {valor_x}, Y= {valor_y}, Z= {valor_z}")
  elif sensor == "bmp":
    temperatura = msg_split[1]
    presion = msg_split[2]
    altitud = msg_split[3]
    db_manager.insert_presion(temperatura, presion, altitud)
    print(f"Insertado en Presión: Temp ={temperatura}, Presión= {presion}, Altitud= {altitud}")
  elif sensor == "ultrasonico":
    distancia = msg_split[1]
    db_manager.insert_ultrasonico(distancia)
    print(f"Insertado en Ultrasonico: Distancia ={distancia} cm")
  else:
    print(f"No tienes : {sensor}")
except Exception as e:
  print(f"Error al procesar el mensaje: {e}")


unacked_publish = set()
mqtt_client = mqtt.Client()
mqtt_client.on_message = on_message
# Se guardan los nombres de los topicos a donde se va a mandar los datos
suscripcion = ["weccat/adc", "weccat/bmp", "weccat/ultrasonido",
"weccat/acelerometro"]
try:
  #Se conecta al broker
  mqtt_client.connect("broker.hivemq.com", 1883)
  # Se suscribe a los topicos de Hive
  for sensor in suscripcion:
    mqtt_client.subscribe(sensor)
    print(f"Suscrito a: {sensor}")
except Exception as e:
  print(f"Error al conectar con el broker: {e}")
  exit()
mqtt_client.loop_start()
try:
  print("Esperando mensajes")
  while True:
    time.sleep(1)
except:
  print(f"Ocurrio un error")
finally:
  mqtt_client.loop_stop()
  mqtt_client.disconnect()
