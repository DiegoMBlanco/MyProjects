#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, Bool, String
from geometry_msgs.msg import Twist


class LineFollowerPID(Node):

    def __init__(self):
        super().__init__('line_follower_pid')

        # Suscripciones esenciales para línea y color
        self.subscription    = self.create_subscription(Int32,   '/line_error',        self.error_callback,        10)
        self.color_sub       = self.create_subscription(Float32, '/color',              self.color_callback,        10)
        self.finish_sub      = self.create_subscription(Bool,    '/finish_line',        self.finish_callback,       10)
        self.inter_sub       = self.create_subscription(Bool,    '/intersection_line',  self.intersection_callback, 10)
        self.obstacle_sub_sim       = self.create_subscription(Bool,    '/obstacle_detected_sim',  self.obstacle_callback_sim, 10)
        self.obstacle_sub_real       = self.create_subscription(Bool,    '/obstacle_detected_real',  self.obstacle_callback_real, 10)
        self.yolo_sub        = self.create_subscription(String,  '/yolo/command',       self.yolo_callback,         10)
        
        self.cmd_pub         = self.create_publisher(Twist, '/cmd_vel', 10)

        # Configuración del PID
        self.kp = 0.10
        self.ki = 0.0
        self.kd = 0.10
        self.error      = 0.0
        self.prev_error = 0.0
        self.integral   = 0.0
        self.dt         = 0.05

        # Parámetros de velocidad dinámicos
        self.base_speed    = 0.11
        self.deadband      = 30
        self.max_linear    = 0.04
        self.max_angular   = 0.50
        self.linear_accel  = 0.02
        self.angular_accel = 0.10
        self.slow_factor   = 0.55

        self.current_linear  = 0.0
        self.current_angular = 0.0

        # Estados de parada/Meta
        self.last_valid_color = 3.0  # Arranca en ROJO (3.0) por seguridad
        self.finished = False

        self.intersection_verification = False
        self.obstacle_verification_sim = False
        self.obstacle_verification_real = False


        # --- Variables añadidas para YOLO y Maniobras ---
        self.ADVANCE_TIME_VERDE    = 5.0
        self.ADVANCE_TIME_AMARILLO = 1.5
        self.PAUSE_TIME   = 1.0
        self.ROTATE_TIME  = 3.0
        self.TURN_LINEAR  = 0.08
        self.TURN_ANGULAR = 0.45
        self.INSTANT_WAIT = 2.0

        self.action_command  = "none"
        self.action_phase    = "none"
        self.phase_end       = None
        self.cooldown_until  = None
        self.turn_direction  = 0

        # Timer del bucle de control principal
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Line follower básico iniciado — Control por PID y Color activo")# --- Variables añadidas para YOLO y Maniobras ---
        self.ADVANCE_TIME_VERDE    = 4.0
        self.ADVANCE_TIME_AMARILLO = 1.5
        self.PAUSE_TIME   = 1.0
        self.ROTATE_TIME  = 3.0
        self.TURN_LINEAR  = 0.08
        self.TURN_ANGULAR = 0.45
        self.INSTANT_WAIT = 2.0
        self.POST_ADVANCE_TIME = 1.5

        self.action_command  = "none"
        self.action_phase    = "none"
        self.phase_end       = None
        self.cooldown_until  = None
        self.turn_direction  = 0

    def yolo_callback(self, msg):
        cmd = msg.data
        if cmd == "none":
            return
        now = self.now_sec()
        if self.cooldown_until is not None and now < self.cooldown_until:
            return

        if cmd == "turn_right":
            self.action_command = "turn_right"
            self.turn_direction = -1
            self.action_phase   = "none"
            self.cooldown_until = now + self.ADVANCE_TIME_VERDE + self.PAUSE_TIME + self.ROTATE_TIME + 5.0 # +5.0
            self.get_logger().info("YOLO: TURN RIGHT — esperando intersección")

        elif cmd == "turn_left":
            self.action_command = "turn_left"
            self.turn_direction = +1
            self.action_phase   = "none"
            self.cooldown_until = now + self.ADVANCE_TIME_VERDE + self.PAUSE_TIME + self.ROTATE_TIME + 5.0 # +5.0
            self.get_logger().info("YOLO: TURN LEFT — esperando intersección")

        elif cmd == "stop":
            self.action_command = "stop"
            self.action_phase   = "active"
            self.phase_end      = now + 999.0
            self.cooldown_until = now + 5.0
            self.get_logger().info("YOLO: STOP — esperando verde")

        elif cmd == "roadwork_ahead":
            self.action_command = "slow"
            self.action_phase   = "wait"
            self.phase_end      = now + self.INSTANT_WAIT
            self.cooldown_until = now + self.INSTANT_WAIT + 5.0
            self.get_logger().info("YOLO: ROADWORK — espera 2s")

        elif cmd == "give_way":
            self.action_command = "give_way"
            self.action_phase   = "wait"
            self.phase_end      = now + self.INSTANT_WAIT
            self.cooldown_until = now + self.INSTANT_WAIT + 4.0
            self.get_logger().info("YOLO: GIVE WAY — espera 2s")

        elif cmd == "straight":
            self.action_command = "straight"
            self.action_phase   = "none"
            self.cooldown_until = now + self.ADVANCE_TIME_VERDE + 3.0
            self.get_logger().info("YOLO: STRAIGHT — esperando intersección")

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def error_callback(self, msg):
        self.error = float(msg.data)

    def color_callback(self, msg):
        color = float(msg.data)
        if color != 0.0:
            self.last_valid_color = color
    
    def obstacle_callback_sim(self, msg):
        self.obstacle_verification_sim = bool(msg.data)
        if self.obstacle_verification_sim:
            self.get_logger().info("OBSTÁCULO SIMULADO — Deteniendo robot")

    def obstacle_callback_real(self, msg):
        self.obstacle_verification_real = bool(msg.data)
        if self.obstacle_verification_real:
            self.get_logger().info("OBSTÁCULO REAL — Deteniendo robot")


    def intersection_callback(self, msg):
        if msg.data == True:
            self.intersection_verification = True

            if self.action_command in ("turn_right", "turn_left", "straight") and self.action_phase == "none":
                advance = self.ADVANCE_TIME_AMARILLO if self.last_valid_color == 1.0 else self.ADVANCE_TIME_VERDE
                self.action_phase = "advance"
                self.phase_end    = self.now_sec() + advance
                self.get_logger().info(f"INTERSECCIÓN — Ejecutando orden de YOLO: {self.action_command.upper()}")
        else:
            self.intersection_verification = False

    def finish_callback(self, msg):
        if msg.data and not self.finished:
            self.finished = True
            self.get_logger().info("META — línea de llegada detectada, deteniendo robot")

    def saturate(self, v, lim):
        return max(-lim, min(lim, v))

    def ramp(self, target, current, step):
        if target > current:
            return min(current + step, target)
        elif target < current:
            return max(current - step, target)
        return current

    def control_loop(self):
        # Si el robot llegó a la meta, enviar velocidad 0 de inmediato y salir
        if self.finished:
            self.cmd_pub.publish(Twist())
            return
        
        # 1. FILTRO DE SEGURIDAD ABSOLUTA (Garantiza detenerse pase lo que pase)
        if self.obstacle_verification_sim or self.obstacle_verification_real:
            # Forzamos la publicación manual de ceros absolutos
            freno_emergencia = Twist()
            self.cmd_pub.publish(freno_emergencia)
            
            # Reseteamos las rampas internas para que al reanudar no de un tirón
            self.current_linear = 0.0
            self.current_angular = 0.0
            
            self.get_logger().warn("BUCLE INTERRUMPIDO: Obstáculo en el camino. Esperando liberación...")
            return # <--- AQUÍ CORTAMOS EL SCRIPT. Nada de lo de abajo se ejecutará.

        # 1. Cálculo del algoritmo PID para el seguimiento de línea
        proportional = self.error
        self.integral   += self.error * self.dt
        derivative       = (self.error - self.prev_error) / self.dt
        angular_pid      = self.kp * proportional + self.ki * self.integral + self.kd * derivative
        self.prev_error  = self.error
        angular_pid      = self.saturate(angular_pid, self.max_angular)

        # 2. Generar consignas objetivo de velocidad basándose en el error de la línea
        if abs(self.error) < self.deadband:
            target_linear  = self.base_speed
            target_angular = 0.0
        else:
            # Si el error es muy grande (> 150), reduce velocidad lineal para no salirse de la curva
            target_linear  = 0.05 if abs(self.error) > 150 else self.base_speed
            target_angular = -angular_pid

        # 3. Aplicar rampas de aceleración suaves
        self.current_linear  = self.ramp(target_linear,  self.current_linear,  self.linear_accel)
        self.current_angular = self.ramp(target_angular, self.current_angular, self.angular_accel)
        self.current_linear  = self.saturate(self.current_linear,  self.max_linear)
        self.current_angular = self.saturate(self.current_angular, self.max_angular)

        # 4. Máquina de estados basada puramente en el COLOR detectado
        estado = "PID-VERDE"

        if self.last_valid_color == 3.0:
            self.current_linear  = 0.0
            self.current_angular = 0.0
            estado = "ROJO-STOP"
        elif self.last_valid_color == 1.0:
            self.current_linear  *= self.slow_factor
            self.current_angular *= self.slow_factor
            estado = "AMARILLO-LENTO"

        # --- LÓGICA DE YOLOS ACTIVOS ---
        now = self.now_sec()
        if self.action_command != "none" and self.action_phase != "none":
            
            if self.action_command == "stop":
                self.current_linear  = 0.0
                self.current_angular = 0.0
                estado = "YOLO-STOP"
                if self.last_valid_color == 2.0: # Si cambia a verde se libera
                    self.action_command = "none"
                    self.action_phase   = "none"

            elif self.action_command in ("turn_right", "turn_left"):
                if self.action_phase == "advance":
                    self.current_linear  = self.TURN_LINEAR
                    self.current_angular = 0.0
                    estado = f"YOLO-{self.action_command.upper()}-AVANZANDO"
                    if now >= self.phase_end:
                        self.action_phase = "pause"
                        self.phase_end    = now + self.PAUSE_TIME
                elif self.action_phase == "pause":
                    self.current_linear  = 0.0
                    self.current_angular = 0.0
                    estado = "YOLO-PAUSA"
                    if now >= self.phase_end:
                        self.action_phase = "rotate"
                        self.phase_end    = now + self.ROTATE_TIME
                elif self.action_phase == "rotate":
                    self.current_linear  = 0.0
                    self.current_angular = self.TURN_ANGULAR * self.turn_direction
                    estado = f"YOLO-{self.action_command.upper()}-ROTANDO"
                    if now >= self.phase_end:
                        # --- MODIFICADO: En vez de terminar, vamos a la nueva fase ---
                        self.action_phase = "post_advance"
                        self.phase_end    = now + self.POST_ADVANCE_TIME
                        self.get_logger().info(f"{self.action_command.upper()} → POST_ADVANCE para salir de intersección")

                elif self.action_phase == "post_advance":
                    # --- NUEVA FASE: Avanza recto ignorando por completo las líneas del piso ---
                    self.current_linear  = self.TURN_LINEAR  # O puedes usar self.base_speed si quieres que vaya más rápido
                    self.current_angular = 0.0
                    estado = "YOLO-POST-AVANCE"
                    if now >= self.phase_end:
                        # Ahora sí, la maniobra terminó por completo
                        self.action_command = "none"
                        self.action_phase   = "none"
                        self.get_logger().info("GIRO Y EVASIÓN completados → Regresando a PID")

            elif self.action_command == "straight":
                if self.action_phase == "advance":
                    self.current_linear  = self.TURN_LINEAR
                    self.current_angular = 0.0
                    estado = "YOLO-RECTO-AVANZANDO"
                    if now >= self.phase_end:
                        self.action_command = "none"
                        self.action_phase   = "none"

            elif self.action_command in ("slow", "give_way"):
                if self.action_phase == "wait" and now >= self.phase_end:
                    self.action_phase = "active"
                    self.phase_end    = now + (4.0 if self.action_command == "slow" else 3.0)
                elif self.action_phase == "active":
                    factor = 0.65 if self.action_command == "slow" else 0.45
                    self.current_linear  *= factor
                    self.current_angular *= 0.5
                    estado = f"YOLO-{self.action_command.upper()}-ACTIVO"
                    if now >= self.phase_end:
                        self.action_command = "none"
                        self.action_phase   = "none"
        
        if self.intersection_verification == True and self.action_phase == "none":
            self.current_angular = 0.0
            self.current_linear = 0.0
        
        #if ((self.obstacle_verification_sim) or (self.obstacle_verification_real)):
           # self.current_angular = 0.0
            #self.current_linear = 0.0

        # 5. Publicar los comandos de velocidad finales al robot
        cmd = Twist()
        cmd.linear.x  = self.current_linear
        cmd.angular.z = self.current_angular
        self.cmd_pub.publish(cmd)

        # Log limpio en la consola
        self.get_logger().info(
            f'Error Linea: {self.error:.0f} | Estado: {estado} | '
            f'Lin: {self.current_linear:.2f} | Ang: {self.current_angular:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()