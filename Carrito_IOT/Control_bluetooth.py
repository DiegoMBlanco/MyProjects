import serial
import time
import board
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import adafruit_bmp280
import busio
import adafruit_adxl34x
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
#---------------------- Configuracion de hardware ---------------------- #
GPIO.setmode(GPIO.BCM)
Pines
ultrasonido
TRIG = 23
ECHO = 24
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
#Pines motores
MotorIa = 5
MotorIr = 6
MotorIp = 26
MotorRa = 16
MotorRr = 20
MotorRp = 21
GPIO.setup(MotorIa, GPIO.OUT)
GPIO.setup(MotorIr, GPIO.OUT)
GPIO.setup(MotorIp, GPIO.OUT)
GPIO.setup(MotorRa, GPIO.OUT)
GPIO.setup(MotorRr, GPIO.OUT)
GPIO.setup(MotorRp, GPIO.OUT)
#Configuracion de sensores
i2c = board.I2C()
bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)
bmp280.sea_level_pressure = 1013.25
i2c_a = busio.I2C(board.SCL, board.SDA)
mide = adafruit_adxl34x.ADXL345(i2c_a)
i2c_adc = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c_adc)
adc_channel = AnalogIn(ads, ADS.P0)
#---------------------- Configuracion
Bluetooth - --------------------- #
joystick_x = 0
joystick_y = 0
button_a = False
button_b = False
button_c = False
button_d = False
def procesar_datos(linea): #asigna el valor correspondiente a cada variable enviada por el arduino
  global joystick_x, joystick_y, button_a, button_b, button_c, button_d
  try:
    datos = linea.split(",")
    for dato in datos:
      clave, valor = dato.split(":")
      if clave.strip() == "X":
        joystick_x = int(valor.strip())
      elif clave.strip() == "Y":
        joystick_y = int(valor.strip())
      elif clave.strip() == "A":
        button_a = valor.strip() == "1"
      elif clave.strip() == "B":
        button_b = valor.strip() == "1"
      elif clave.strip() == "C":
        button_c = valor.strip() == "1"
      elif clave.strip() == "D":
        button_d = valor.strip() == "1"
  except Exception as e:
    print(f"Error procesando datos: {e}")
#---------------------- Control de motores - --------------------- #
def determinar_direccion(x, y): #Determina que pines se encienden para poder controlar los motores usando los valores recibidos del joystick.
  dead_zone_min = 450
  dead_zone_max = 550
  if dead_zone_min < x < dead_zone_max and dead_zone_min < y <
    dead_zone_max:
    print("Detener")
    GPIO.output(MotorIp, GPIO.LOW)
    GPIO.output(MotorRp, GPIO.LOW)
  elif y > dead_zone_max:
    print("Adelante")
    GPIO.output(MotorIa, GPIO.HIGH)
    GPIO.output(MotorIr, GPIO.LOW)
    GPIO.output(MotorIp, GPIO.HIGH)
    GPIO.output(MotorRa, GPIO.HIGH)
    GPIO.output(MotorRr, GPIO.LOW)
    GPIO.output(MotorRp, GPIO.HIGH)
  elif y < dead_zone_min:
    print("Atras")
    GPIO.output(MotorIa, GPIO.LOW)
    GPIO.output(MotorIr, GPIO.HIGH)
    GPIO.output(MotorIp, GPIO.HIGH)
    GPIO.output(MotorRa, GPIO.LOW)
    GPIO.output(MotorRr, GPIO.HIGH)
    GPIO.output(MotorRp, GPIO.HIGH)
  elif x > dead_zone_max:
    print("Derecha")
    GPIO.output(MotorRa, GPIO.HIGH)
    GPIO.output(MotorRr, GPIO.LOW)
    GPIO.output(MotorRp, GPIO.HIGH)
  elif x < dead_zone_min:
    print("Izquierda")
    GPIO.output(MotorIa, GPIO.HIGH)
    GPIO.output(MotorIr, GPIO.LOW)
    GPIO.output(MotorIp, GPIO.HIGH)

mqtt_client = mqtt.Client()
mqtt_client.connect("broker.hivemq.com", 1883)
mqtt_client.loop_start()


def publicar_datos(): #Publica los datos de los sensores
  try:
    # ADC
    adc_value = adc_channel.value
    adc_voltage = adc_channel.voltage
    mqtt_client.publish("weccat/adc", f"adc_{adc_value}_{adc_voltage}",
    qos=2)
    # Acelerometro
    axxel = ("%f_%f_%f" % mide.acceleration)
    mqtt_client.publish("weccat/acelerometro", f"acelerometro_{axxel}",
    qos=2)
    # Sensor BMP280
    temp = bmp280.temperature
    pres = bmp280.pressure
    alti = bmp280.altitude
    mqtt_client.publish("weccat/bmp", f"bmp_{temp}_{pres}_{alti}", qos=2)
    # Sensor ultrasonico
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    while GPIO.input(ECHO) == 0:
      pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
      pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    mqtt_client.publish("weccat/ultrasonido", f"ultrasonico_{distance}",
    qos=2)
  except Exception as e:
    print(f"Error publicando datos: {e}")

try:
  #Inicia la comunicacion serial del puerto rfcomm0 con el modulo HC06
  bluetooth = serial.Serial('/dev/rfcomm0', 9600, timeout=1)
  print("Bluetooth conectado. Esperando datos...")
  last_mqtt_time = time.time()
  while True:
    # Bluetooth
    data = bluetooth.readline()
    if data:
      decoded_data = data.decode('ascii', errors='ignore').strip()
      procesar_datos(decoded_data)
      determinar_direccion(joystick_x, joystick_y)
    # Publicar datos a MQTT cada 10 segundos
    if time.time() - last_mqtt_time >= 10:
      publicar_datos()
      last_mqtt_time = time.time()
except KeyboardInterrupt:
  print("Programa interrumpido.")
finally:
  GPIO.cleanup()
  mqtt_client.disconnect()
  mqtt_client.loop_stop()
  bluetooth.close()
  print("Recursos liberados.")
