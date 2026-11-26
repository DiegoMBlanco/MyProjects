import os
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import math

# --- CONFIGURACIÓN ---
# Ruta a la CARPETA del bag (donde está el archivo .db3 o .mcap y el metadata.yaml)
BAG_PATH = 'circulo_MPC_lineal_0.db3' 

def get_rosbag_options(path, storage_id='sqlite3'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    return storage_options, converter_options

def get_yaw(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

def run_analysis():
    reader = rosbag2_py.SequentialReader()
    storage_options, converter_options = get_rosbag_options(BAG_PATH)
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error abriendo el bag: {e}")
        return

    # Diccionarios para almacenar datos (Ampliados para Ground Truth)
    data = {
        'time': [], 'x': [], 'y': [], 'v': [], 'w': [],
        'cte': [], 'epsi': [], 'path_x': [], 'path_y': [],
        'solve_time': [],
        'gt_x': [], 'gt_y': [], 'cte_gt': [], 'epsi_gt': []  # <--- NUEVOS DATOS
    }

    # Mapa de tipos de mensaje actualizados según tus nuevos tópicos
    topic_types = {
        '/r1/odom': 'nav_msgs/msg/Odometry',
        '/r1/ground_truth': 'geometry_msgs/msg/PoseStamped',  # <--- NUEVO (PoseStamped)
        '/drawn_plan': 'nav_msgs/msg/Path',
        '/r1/cmd_vel': 'geometry_msgs/msg/Twist',
        '/CTE': 'std_msgs/msg/Float32',
        '/e_psi': 'std_msgs/msg/Float32',
        '/CTE_gt': 'std_msgs/msg/Float32',                    # <--- NUEVO
        '/e_psi_gt': 'std_msgs/msg/Float32',                  # <--- NUEVO
        '/solve_time_ms': 'std_msgs/msg/Float32'
    }

    print("Leyendo mensajes...")
    while reader.has_next():
        (topic, data_raw, t_nanos) = reader.read_next()
        if topic not in topic_types:
            continue

        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data_raw, msg_type)
        t_sec = t_nanos / 1e9

        if topic == '/r1/odom':
            data['time'].append(t_sec)
            data['x'].append(msg.pose.pose.position.x)
            data['y'].append(msg.pose.pose.position.y)
        elif topic == '/r1/ground_truth':
            # Nota: Si en tu nodo mantienes la inversión de signo física (-msg.pose.position.x), 
            # aplícala aquí también multiplicando por -1. De lo contrario, déjalo directo:
            data['gt_x'].append(msg.pose.position.x) 
            data['gt_y'].append(msg.pose.position.y)
        elif topic == '/r1/cmd_vel':
            data['v'].append(msg.linear.x)
            data['w'].append(msg.angular.z)
        elif topic == '/CTE':
            data['cte'].append(msg.data)
        elif topic == '/e_psi':
            data['epsi'].append(msg.data)
        elif topic == '/CTE_gt':
            data['cte_gt'].append(msg.data)
        elif topic == '/e_psi_gt':
            data['epsi_gt'].append(msg.data)
        elif topic == '/drawn_plan':
            data['path_x'] = [p.pose.position.x for p in msg.poses]
            data['path_y'] = [p.pose.position.y for p in msg.poses]
        elif topic == '/solve_time_ms':
            data['solve_time'].append(msg.data)

    # --- CÁLCULO DE ZERO CROSSINGS ---
    def get_zero_crossings(signal):
        sig = np.array(signal)
        return np.where(np.diff(np.sign(sig)))[0]

    zc_cte = get_zero_crossings(data['cte']) if data['cte'] else []
    zc_cte_gt = get_zero_crossings(data['cte_gt']) if data['cte_gt'] else []

   # --- GENERACIÓN DE GRÁFICAS (INTEGRANDO GROUND TRUTH) ---
    print("Generando gráficas...")
    plt.style.use('ggplot') 
    fig = plt.figure(figsize=(16, 16))
    fig.suptitle(f'Métricas de Desempeño MPC - Bag: {os.path.basename(BAG_PATH)}', fontsize=16)

    # 1. Trayectoria (XY) - Ocupa la columna izquierda completa
    ax1 = plt.subplot(4, 2, (1, 3)) 
    if data['path_x']:
        ax1.plot(data['path_x'], data['path_y'], 'r--', label='Trayectoria Deseada', alpha=0.8)
    ax1.plot(data['x'], data['y'], 'b-', label='Trayectoria Seguida (Odom)', linewidth=1.5)
    if data['gt_x']:
        ax1.plot(data['gt_x'], data['gt_y'], 'g-.', label='Trayectoria Real (Ground Truth)', linewidth=1.5)
    ax1.set_title('Comparativa de Trayectoria')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.legend()
    ax1.grid(True)

    # 2. CTE + Zero Crossings (Odometría vs Ground Truth)
    ax2 = plt.subplot(4, 2, 2)
    ax2.axhline(y=0, color='black', linestyle='-', linewidth=0.8)
    
    if len(data['cte']) > 0:
        cte_np = np.array(data['cte'])
        ax2.plot(cte_np, 'b-', label='CTE Est. (Odom)', alpha=0.5)
        if len(zc_cte) > 0:
            ax2.scatter(zc_cte, cte_np[zc_cte], color='blue', marker='x', s=30, label=f'Z-Cross Odom ({len(zc_cte)})')
            
    if len(data['cte_gt']) > 0:
        cte_gt_np = np.array(data['cte_gt'])
        ax2.plot(cte_gt_np, 'g-', label='CTE Real (GT)', alpha=0.8)
        if len(zc_cte_gt) > 0:
            ax2.scatter(zc_cte_gt, cte_gt_np[zc_cte_gt], color='red', marker='o', s=20, label=f'Z-Cross GT ({len(zc_cte_gt)})')
            
    ax2.set_ylabel('Metros')
    ax2.set_title('Error Lateral (CTE)')
    ax2.legend()
    ax2.grid(True)

    # 3. Heading Error (e_psi) - Comparativa Odom vs GT
    ax3 = plt.subplot(4, 2, 4)
    if len(data['epsi']) > 0:
        ax3.plot(data['epsi'], 'b-', label='e_psi Est. (Odom)', alpha=0.5)
    if len(data['epsi_gt']) > 0:
        ax3.plot(data['epsi_gt'], 'g-', label='e_psi Real (GT)', alpha=0.8)
    ax3.set_ylabel('Radianes')
    ax3.set_title('Error de Orientación (e_psi)')
    ax3.legend()
    ax3.grid(True)

    # 4. Velocidad Lineal - Abajo Izquierda
    ax4 = plt.subplot(4, 2, 5)
    if data['v']:
        ax4.plot(data['v'], 'k-', label='v (m/s)')
    ax4.set_title('Velocidad Lineal (v)')
    ax4.set_xlabel('Muestras')
    ax4.set_ylabel('m/s')
    ax4.grid(True)

    # 5. Control de Dirección (Angular) - Abajo Derecha
    ax5 = plt.subplot(4, 2, 6)
    if data['w']:
        ax5.plot(data['w'], 'purple', label='w (rad/s)')
    ax5.set_title('Control Angular (w)')
    ax5.set_xlabel('Muestras')
    ax5.set_ylabel('rad/s')
    ax5.grid(True)

    # 6. Costo computacional
    ax6 = plt.subplot(4, 2, (7, 8))
    if data['solve_time']:
        st = np.array(data['solve_time'])
        ax6.plot(st, 'teal', label='Solve time (ms)', alpha=0.7)
        ax6.axhline(y=np.mean(st), color='red', linestyle='--',
                    label=f'Media: {np.mean(st):.2f} ms')
        ax6.axhline(y=np.max(st), color='orange', linestyle=':',
                    label=f'Máx: {np.max(st):.2f} ms')
        ax6.set_ylabel('ms')
    ax6.set_title('Costo Computacional OSQP (solve time)')
    ax6.set_xlabel('Muestras')
    ax6.legend()
    ax6.grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

if __name__ == '__main__':
    run_analysis()