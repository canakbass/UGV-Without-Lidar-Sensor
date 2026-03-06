from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1. Serial Bridge (Mock Mode = True)
        Node(
            package='ika_bridge',
            executable='serial_bridge',
            name='serial_bridge',
            parameters=[{'mock_mode': True}]
        ),
        
        # 2. Vision Node (Mock Mode = True)
        Node(
            package='ika_vision',
            executable='detector',
            name='vision_node',
            parameters=[{'mock_mode': True}]
        ),
        
        # 3. Decision Node (State Machine)
        Node(
            package='ika_decision',
            executable='state_machine',
            name='state_machine'
        ),
        
        # 4. Acceleration Node
        Node(
            package='ika_control',
            executable='acceleration',
            name='acceleration_logic'
        ),
        
        # 5. Web Bridge (Optional - mostly specific logic)
        Node(
            package='ika_web_bridge',
            executable='web_bridge',
            name='web_bridge'
        ),
        
        # 6. Rosbridge Server (For Web UI Connection)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket'
        ),
    ])
