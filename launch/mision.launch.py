"""Lanza el escenario (TurtleBot3 Burger en turtlebot3_dqn_stage4) + la misión.

    ros2 launch tb3_cobertura mision.launch.py            # Gazebo + misión
    ros2 launch tb3_cobertura mision.launch.py rviz:=true # + RViz con mapa y celdas
    ros2 launch tb3_cobertura mision.launch.py obstaculos_moviles:=true   # cilindros móviles

Para grabar el video (todo listo y en pausa hasta darle play):

    ros2 launch tb3_cobertura mision.launch.py rviz:=true pausado:=true terminal_matriz:=true

Este launch hace lo mismo que turtlebot3_dqn_stage4.launch.py del paquete
oficial (servidor y cliente de Gazebo, robot_state_publisher, spawn del robot y
puente ROS-Gazebo), con dos agregados:

1. Presentación. Con vista_superior:=true (por defecto) la interfaz de Gazebo
   abre con la cámara desde arriba (x a la derecha, y hacia arriba, igual que
   RViz) y con el panel "Visualize Lidar" ya agregado. Se parte de la
   configuración de interfaz de Gazebo del usuario (o la de fábrica) y solo se
   cambian esos dos puntos (el panel dibuja los rayos, pero en Gazebo
   Harmonic los ubica en el origen del mundo; ver README). Con pausado:=true la simulación arranca detenida;
   la misión espera, porque corre sobre el reloj de Gazebo. Con
   terminal_matriz:=true se abre una terminal que muestra la matriz de
   cobertura actualizándose en el lugar.

2. Plugins de los cilindros. El paquete de Jazzy instala los plugins que mueven
   los cilindros (libobstacle1.so, libobstacle2.so) en lib/turtlebot3_gazebo/,
   carpeta que Gazebo no revisa. Con obstaculos_moviles:=true se agrega esa ruta
   a GZ_SIM_SYSTEM_PLUGIN_PATH.
"""
import glob
import os
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# robot_state_publisher.launch.py del paquete oficial lee esta variable
os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')

NOMBRE_ROBOT = 'burger'
ARCHIVO_MATRIZ = '/tmp/tb3_matriz.txt'
# cámara desde arriba, a 7.5 m: x hacia la derecha, y hacia arriba (como RViz)
CAMARA_SUPERIOR = '0 0 7.5 0 1.5700 1.5708'


def _es(context, nombre):
    return LaunchConfiguration(nombre).perform(context).lower() == 'true'


def _config_gui():
    """Copia la configuración de interfaz de Gazebo con cámara superior y panel del LiDAR."""
    candidatos = [os.path.expanduser('~/.gz/sim/8/gui.config')]
    candidatos += sorted(glob.glob('/opt/ros/*/opt/gz_sim_vendor/share/gz/gz-sim*/gui/gui.config'))
    base = next((c for c in candidatos if os.path.isfile(c)), None)
    if base is None:
        return None
    txt = re.sub(r'<\?xml[^>]*\?>', '', open(base).read())
    raiz = ET.fromstring('<raiz>' + txt + '</raiz>')
    for pl in raiz.findall('plugin'):
        if pl.get('filename') == 'MinimalScene':
            cam = pl.find('camera_pose')
            if cam is None:
                cam = ET.SubElement(pl, 'camera_pose')
            cam.text = CAMARA_SUPERIOR
    if not any(p.get('filename') == 'VisualizeLidar' for p in raiz.findall('plugin')):
        pl = ET.SubElement(raiz, 'plugin', {'filename': 'VisualizeLidar', 'name': 'Visualize Lidar'})
        gui = ET.SubElement(pl, 'gz-gui')
        ET.SubElement(gui, 'title').text = 'Visualize Lidar'
    salida = os.path.join(tempfile.gettempdir(), 'tb3_cobertura_gui.config')
    with open(salida, 'w') as f:
        f.write('<?xml version="1.0"?>\n')
        for elem in raiz:
            f.write(ET.tostring(elem, encoding='unicode'))
    return salida


def _gazebo(context, ros_gz_sim, mundo):
    """Servidor (en pausa o corriendo) y cliente (con o sin vista preparada)."""
    arranque = '' if _es(context, 'pausado') else '-r '
    servidor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': arranque + '-s -v2 ' + mundo}.items())
    args_cliente = '-g -v2 '
    acciones = [servidor]
    if _es(context, 'vista_superior'):
        cfg = _config_gui()
        if cfg:
            args_cliente += '--gui-config ' + cfg + ' '
        else:
            acciones.append(LogInfo(msg='No se encontró la configuración de la interfaz de Gazebo; '
                                        'se usa la vista por defecto.'))
    acciones.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': args_cliente}.items()))
    return acciones


def _terminal_matriz(context):
    """Abre una terminal con la matriz de cobertura actualizándose en el lugar."""
    if not _es(context, 'terminal_matriz'):
        return []
    with open(ARCHIVO_MATRIZ, 'w') as f:
        f.write('Esperando el inicio de la misión...\n')
    orden = f'watch -t -n 0.5 cat {ARCHIVO_MATRIZ}'
    if shutil.which('xfce4-terminal'):
        cmd = ['xfce4-terminal', '--title=Matriz de cobertura', '--geometry=62x36', '-x', 'bash', '-c', orden]
    elif shutil.which('xterm'):
        cmd = ['xterm', '-T', 'Matriz de cobertura', '-geometry', '62x36', '-fa', 'Monospace', '-fs', '11',
               '-e', 'bash', '-c', orden]
    elif shutil.which('lxterminal'):
        cmd = ['lxterminal', '-t', 'Matriz de cobertura', '-e', orden]
    elif shutil.which('x-terminal-emulator'):
        cmd = ['x-terminal-emulator', '-e', 'bash', '-c', orden]
    else:
        return [LogInfo(msg=f'No hay terminal gráfica instalada; en otra terminal ejecutar: {orden}')]
    return [ExecuteProcess(cmd=cmd, output='log')]


def _robot(context, tb3):
    """Spawn del robot desde una copia del modelo oficial con el tópico del LiDAR como '/scan'.

    El sensor declara <topic>scan</topic>. Para Gazebo es el mismo tópico que
    '/scan', pero el panel "Visualize Lidar" busca el sensor comparando el texto
    del tópico elegido ('/scan') con el declarado ('scan'); como no coinciden,
    no lo encuentra y no puede ubicar los rayos en el robot. Solo se cambia
    ese texto: el resto del modelo (incluido el marco base_scan) queda igual.
    """
    oficial = os.path.join(tb3, 'models', 'turtlebot3_burger', 'model.sdf')
    archivo = oficial
    if _es(context, 'vista_superior'):
        with open(oficial) as f:
            sdf = f.read()
        sdf = sdf.replace('<topic>scan</topic>', '<topic>/scan</topic>', 1)
        archivo = os.path.join(tempfile.gettempdir(), 'tb3_cobertura_burger.sdf')
        with open(archivo, 'w') as f:
            f.write(sdf)
    return [Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', NOMBRE_ROBOT, '-file', archivo,
                            '-x', '0.0', '-y', '0.0', '-z', '0.01'])]


def generate_launch_description():
    pkg = get_package_share_directory('tb3_cobertura')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    params = os.path.join(pkg, 'config', 'parametros.yaml')
    mundo = os.path.join(tb3, 'worlds', 'turtlebot3_dqn_stage4.world')
    plugins_tb3 = os.path.join(get_package_prefix('turtlebot3_gazebo'), 'lib', 'turtlebot3_gazebo')

    # ---------------- escenario (equivalente a turtlebot3_dqn_stage4.launch.py)
    estado_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())
    puente = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['--ros-args', '-p',
                   'config_file:=' + os.path.join(tb3, 'params', 'turtlebot3_burger_bridge.yaml')])

    # ---------------- misión
    # el juez conoce el escenario (lee el mismo .world que Gazebo); el robot no
    con_cilindros = ParameterValue(
        PythonExpression(["'", LaunchConfiguration('obstaculos_moviles'), "' != 'true'"]),
        value_type=bool)
    monitor = Node(package='tb3_cobertura', executable='monitor',
                   name='monitor_cobertura', output='screen',
                   parameters=[params, {
                       'use_sim_time': True,
                       'archivo_mundo': mundo,
                       'rutas_modelos': [os.path.join(tb3, 'models')],
                       'incluir_cilindros': con_cilindros,
                       'archivo_matriz': ARCHIVO_MATRIZ}])
    # sin obstáculos móviles en el escenario, no tiene sentido evadirlos:
    # el mapa y la capa de seguridad siguen protegiendo al robot
    moviles = ParameterValue(LaunchConfiguration('obstaculos_moviles'), value_type=bool)
    navegador = Node(package='tb3_cobertura', executable='navegador',
                     name='navegador_cobertura', output='screen',
                     parameters=[params, {'use_sim_time': True,
                                          'evitar_moviles': moviles}])
    # SOLO validación: la pose verdadera de Gazebo para medir la cobertura real.
    # La navegación no la usa (no se suscribe a este tópico).
    verdad = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  name='puente_verdad', output='log',
                  arguments=['/world/dqn/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
                  remappings=[('/world/dqn/dynamic_pose/info', '/verdad/poses')],
                  parameters=[{'use_sim_time': True}],
                  condition=IfCondition(LaunchConfiguration('validar')))
    visor = Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                 arguments=['-d', os.path.join(pkg, 'rviz', 'cobertura.rviz')],
                 parameters=[{'use_sim_time': True}],
                 condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('retardo', default_value='8.0',
                              description='segundos (reales) antes de arrancar la misión'),
        DeclareLaunchArgument('validar', default_value='true',
                              description='puentear la pose verdadera de Gazebo (solo para medir)'),
        DeclareLaunchArgument('obstaculos_moviles', default_value='false',
                              description='cargar los plugins que mueven los cilindros'),
        DeclareLaunchArgument('pausado', default_value='false',
                              description='arrancar Gazebo en pausa (se inicia con el botón play)'),
        DeclareLaunchArgument('vista_superior', default_value='true',
                              description='abrir Gazebo con cámara superior y panel Visualize Lidar'),
        DeclareLaunchArgument('terminal_matriz', default_value='false',
                              description='abrir una terminal con la matriz de cobertura en vivo'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugins_tb3,
                                  condition=IfCondition(LaunchConfiguration('obstaculos_moviles'))),
        OpaqueFunction(function=_gazebo, args=[ros_gz_sim, mundo]),
        estado_robot,
        OpaqueFunction(function=_robot, args=[tb3]),
        puente,
        visor,
        verdad,
        # se espera a que Gazebo, el robot y el puente estén arriba
        TimerAction(period=LaunchConfiguration('retardo'), actions=[monitor, navegador]),
        TimerAction(period=LaunchConfiguration('retardo'),
                    actions=[OpaqueFunction(function=_terminal_matriz)]),
    ])
