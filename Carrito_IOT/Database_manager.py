import mysql.connector
from mysql.connector import Error
from datetime import datetime
#Variable donde se guarda la fecha actual
fecha = datetime.now()
class DatabaseManager:
  def __init__(self, host, database, user, password):
    self.host = host
    self.database = database
    self.user = user
    self.password = password
    self.connection = None
    self.cursor = None
    self.connect()

  
  def connect(self):
    try:
      self.connection = mysql.connector.connect(
      host='10.48.233.73',
      database='raspberry_2',
      user='weccat',
      password='tacoverde'
      )
      if self.connection.is_connected():
        self.cursor = self.connection.cursor()
    except Error as e:
      print(e)

  
  def disconnect(self):
    if self.connection.is_connected():
      self.cursor.close()
      self.connection.close()
  #Funcion para imprimir los datos de algun sensor
  def list_sensor(self,sensor):
    query = f"SELECT * FROM {sensor}"
    self.cursor.execute(query)
    result = self.cursor.fetchall()
    for row in result:
      print(row[1])
  #Funciones para insertar los valores en las tablas de SQL
  def insert_adc(self, value,voltaje):
    query = """INSERT INTO adc (fecha,valor_res_luz,voltage) VALUES
    (%s,%s,%s)"""
    values = (fecha,value,voltaje,)
    self.cursor.execute(query, values)
    self.connection.commit()
  def insert_acelerometro(self, valor_acel_x,valor_acel_y,valor_acel_z):
    query = """INSERT INTO acelerometro (
    fecha,valor_acel_x,valor_acel_y,valor_acel_z) VALUES (%s,%s,%s,%s)"""
    values = ( fecha,valor_acel_x,valor_acel_y,valor_acel_z)
    self.cursor.execute(query, values,)
    self.connection.commit()
  def insert_presion(self, valor_temperatura,valor_presion,valor_altitud):
    query = """INSERT INTO presion
    (fecha,valor_temperatura,valor_presion,valor_altitud) VALUES
    (%s,%s,%s,%s)"""
    values = (fecha,valor_temperatura,valor_presion,valor_altitud,)
    self.cursor.execute(query, values)
    self.connection.commit()
  def insert_ultrasonico(self, valor_distancia_cm):
    query = """INSERT INTO ultrasonico ( fecha,valor_distancia_cm)
    VALUES (%s,%s)"""
  values = ( fecha,valor_distancia_cm,)
  self.cursor.execute(query, values)
  self.connection.commit()
