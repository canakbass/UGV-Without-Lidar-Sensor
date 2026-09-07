from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    TEKNOFEST 2026 UGV - Full Autonomy System Launch
    Launches State Machine, Vision Detectors, Controllers, Bridges, and Telemetry.
    """
    return LaunchDescription([
        # 1. State Machine (Autonomous Competition Stage Decision Engine)
        Node(
            package='ika_decision',
            executable='state_machine',
            name='state_machine',
            output='screen'
        ),
        # 2. Vision Detector (Sign Recognition & Lane Centering)
        Node(
            package='ika_vision',
            executable='detector',
            name='vision_detector',
            output='screen',
            parameters=[{'camera_source': 'ros_topic'}]
        ),
        # 3. Obstacle Detector (Slalom Cones & Dynamic Sliding Wall)
        Node(
            package='ika_vision',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen'
        ),
        # 4. Motion Controller (PID, Steep Incline, Descents, Skid-Steer Assist)
        Node(
            package='ika_control',
            executable='controller',
            name='controller',
            output='screen'
        ),
        # 5. Navigation Node (Ultrasonic Proximity Array Steering)
        Node(
            package='ika_control',
            executable='navigation',
            name='navigation',
            output='screen'
        ),
        # 6. Serial Bridge (ESP32 Hardware & Simulation Physics)
        Node(
            package='ika_bridge',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{'mock_mode': False}]
        ),
        # 7. Web Telemetry Bridge (WebSocket Ground Control Station)
        Node(
            package='ika_web_bridge',
            executable='web_bridge',
            name='web_bridge',
            output='screen'
        ),
    ])
