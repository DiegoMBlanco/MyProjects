#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class WeldingCellManager(Node):

    def __init__(self):
        super().__init__('welding_cell_manager')

        self.trajectory_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10) # Moves each robot
        
        self.command_sub = self.create_subscription(String, '/welding_cell/command',self.command_callback,10) # Receives commands

        self.line_timer = self.create_timer(self.cycle_time,self.advance_line)

        self.station_order = ['r6', 'r3', 'r5', 'r2', 'r4', 'r1']
        self.robots = ['r1', 'r2', 'r3', 'r4', 'r5', 'r6']
        self.joints_per_robot = [
            'joint_NAUO2',
            'joint_NAUO4',
            'joint_NAUO3',
            'joint_NAUO5',
            'joint_NAUO6',
            'joint_NAUO7'
        ]

        # r1-r3
        self.HOME_R1_R3 = [3.8, 0.8, -1.5, 0.0, 0.0, 0.0]
        self.IDLE_R1_R3 = [1.6, 0.9, -1.0, 0.0, 0.0, 0.0]
        self.WELDING_UP_R1_R3 = [1.6, 0.0, 0.0, 1.0, 1.0, 0.0]
        self.WELDING_DOWN_R1_R3 = [1.6, -0.5, -0.3, 0.0, -1.0, 0.0]

        # r4-r6
        self.HOME_R4_R6 = [0.0, 0.8, -1.5, 0.0, 0.0, 0.0]
        self.IDLE_R4_R6 = [-1.6, 0.9, -1.0, 0.0, 0.0, 0.0]
        self.WELDING_UP_R4_R6 = [-1.6, 0.0, 0.0, 1.0, 1.0, 0.0]
        self.WELDING_DOWN_R4_R6 = [-1.6, -0.5, -0.3, 0.0, -1.0, 0.0]

      
        self.cycle_time = 8.0 # For one object to advance to the next station

        self.welding_time = 6.0

        self.motion_time = 1.0 # For each trajectory

        self.cycle_number = 0

        self.next_piece_id = 1

        self.pieces = {}

        self.states = { # Robot's actual state
            robot: 'idle'
            for robot in self.robots
        }

        self.robot_piece = { # Robot's object for welding
            robot: None
            for robot in self.robots
        }

        self.maintenance = { # Maintenance structure
            robot: False
            for robot in self.robots
        }

        self.piece_welding_mode = {} # Up/Down

        self.welding_timers = {}
        
        self.get_logger().info('===============================')
        self.get_logger().info(' Welding Cell Manager initiated')
        self.get_logger().info('===============================')

        self.publish_all_states() # Start in IDLE pose

        self.spawn_piece() # Creates first piece

    def advance_line(self):

        self.cycle_number += 1

        self.get_logger().info(f'========== CYCLE {self.cycle_number} ==========')

        pieces_to_move = list(self.pieces.items())

        new_positions = {}

        for piece_id, station_index in pieces_to_move:

            next_station = station_index + 1

            if next_station >= len(self.station_order): # Piece has reached the end 

                current_robot = self.station_order[station_index]

                if self.robot_piece[current_robot] == piece_id: # Cleans piece ID
                    self.robot_piece[current_robot] = None

                self.get_logger().info(
                    f'Pieza {piece_id} terminó la línea y salió.'
                )

                continue

            new_positions[piece_id] = next_station

        self.pieces = new_positions


        if self.cycle_number % 2 == 1: # Every 2 cycles a new piece is created
            self.spawn_piece()

        self.update_robot_assignments()

        self.publish_all_states()

        self.print_line_status()

    def spawn_piece(self):

        piece_id = self.next_piece_id
        self.next_piece_id += 1

        self.pieces[piece_id] = 0

        if piece_id % 2 == 1: # UP/DOWN one & one
            self.piece_welding_mode[piece_id] = 'welding_up'
        else:
            self.piece_welding_mode[piece_id] = 'welding_down'

        self.get_logger().info(
            f'>>> One Piece {piece_id} enters production line on r6 ' # Gear 5???
            f'({self.piece_welding_mode[piece_id]})'
        )

    def update_robot_assignments(self):

        for robot in self.robots:
            self.robot_piece[robot] = None

        for piece_id, station_index in self.pieces.items():

            robot = self.station_order[station_index]

            if self.maintenance[robot]:
                self.get_logger().warn(
                    f'Piece {piece_id} arrived to {robot}, '
                    f'but it is on maintenance'
                )
                continue # If robot is maintenance, it cannot weld any piece

            self.robot_piece[robot] = piece_id

        for robot in self.robots:

            if self.maintenance[robot]: # Returns home pose por maintenance
                self.states[robot] = 'home'
                continue

            piece_id = self.robot_piece[robot]

            if piece_id is None:
                self.states[robot] = 'idle' # Executes idle position if there is no piece in front
            else:
                self.states[robot] = ( # Executes welding position
                    self.piece_welding_mode[piece_id]
                )

                self.start_welding_timer(robot,piece_id)

    def start_welding_timer(self, robot, piece_id):

        if robot in self.welding_timers: # Creates timer
            return

        timer = self.create_timer(self.welding_time, lambda r=robot, p=piece_id: self.finish_welding(r, p))

        self.welding_timers[robot] = timer

    def finish_welding(self, robot, piece_id):
   
        timer = self.welding_timers.pop(robot, None) # Erases timer

        if timer is not None:
            timer.cancel()

        if self.robot_piece[robot] == piece_id: # If same piece:

            if self.maintenance[robot]:
                self.states[robot] = 'home'
            else:
                self.states[robot] = 'idle'

            self.get_logger().info(
                f'{robot} finished welding piece {piece_id} -> IDLE'
            )

            self.publish_all_states()

    def command_callback(self, msg): # Receives commands from dashboard
        command = msg.data.strip().lower()


        if command.startswith('mantenimiento_'):

            robot = command.replace('mantenimiento_','')

            if robot not in self.robots:
                self.get_logger().warn(f'UNK Robot: {robot}') 
                return

            if self.maintenance[robot]:
                self.get_logger().warn(f'{robot} already on maintenance.')
                return

            self.maintenance[robot] = True
            self.states[robot] = 'home'

            timer = self.welding_timers.pop(robot, None) # Erases timer

            if timer is not None:
                timer.cancel()

            self.get_logger().info(
                f'{robot} -> HOME / MANTENIMIENTO'
            )

            self.publish_all_states()

            return

        if command.startswith('activar_'):

            robot = command.replace('activar_','')

            if robot not in self.robots:
                self.get_logger().warn(f'UNK Robot: {robot}')
                return

            if not self.maintenance[robot]:
                self.get_logger().warn(f'{robot} already active.')
                return

            self.maintenance[robot] = False
            self.states[robot] = 'idle'

            self.get_logger().info(f'{robot} -> ACTIVADO / IDLE')

            self.update_robot_assignments() # Resumes normal operation

            self.publish_all_states()

            return

        self.get_logger().warn(f'UNK command: {command}')

    def get_robot_position(self, robot):

        robot_number = int(robot[1:])
        state = self.states[robot]

        if robot_number <= 3: # Left robots
            if state == 'home':
                return self.HOME_R1_R3
            elif state == 'welding_up':
                return self.WELDING_UP_R1_R3
            elif state == 'welding_down':
                return self.WELDING_DOWN_R1_R3
            else:
                return self.IDLE_R1_R3
        else: # Right robots
            if state == 'home':
                return self.HOME_R4_R6
            elif state == 'welding_up':
                return self.WELDING_UP_R4_R6
            elif state == 'welding_down':
                return self.WELDING_DOWN_R4_R6
            else:
                return self.IDLE_R4_R6


    def publish_all_states(self):

        msg = JointTrajectory()

        for robot in self.robots: # Same controller 

            for joint in self.joints_per_robot:
                msg.joint_names.append(f'{robot}/{joint}')

        point = JointTrajectoryPoint()

        for robot in self.robots:

            point.positions.extend(self.get_robot_position(robot))

        point.time_from_start.sec = int(
            self.motion_time
        )

        point.time_from_start.nanosec = int(
            (self.motion_time % 1) * 1e9
        )

        msg.points.append(point)

        self.trajectory_pub.publish(msg) # Publishes trajectories

    def print_line_status(self):

        status = []
        for robot in self.station_order:
            piece_id = self.robot_piece[robot]
            if self.maintenance[robot]:
                status.append(f'{robot}: MAINTENANCE')
            elif piece_id is None:
                status.append(f'{robot}: IDLE')
            else:
                status.append(f'{robot}: PIEZA {piece_id} 'f'{self.states[robot].upper()}')
        self.get_logger().info(' | '.join(status))


def main(args=None):
    rclpy.init(args=args)
    node = WeldingCellManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()