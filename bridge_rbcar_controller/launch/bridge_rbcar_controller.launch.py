from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obtener la ruta del archivo YAML
    parametros_file = os.path.join(
        get_package_share_directory('bridge_rbcar_controller'), 'config', 'bridge_rbcar_controller.yaml')

    return LaunchDescription([
        # Lanzar el nodo con los parámetros especificados en el archivo YAML
        Node(
            package='bridge_rbcar_controller',  # Reemplaza 'nombre_del_paquete' con el nombre real del paquete
            executable='bridge_rbcar_controller',  # Nombre del ejecutable generado a partir de 'bridge_rbcar_controller.cpp'
            name='bridge_rbcar_controller_node',
            output='screen',
            parameters=[parametros_file],
        ),
    ])
