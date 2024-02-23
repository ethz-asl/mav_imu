
# THIS DOES NOT WORK YET !!!!!!

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'imu',
            default_value='bmi088',
            description='The IMU model (adis16448 or bmi088).'
        ),
        DeclareLaunchArgument(
            'spi_path',
            default_value='/dev/spidev0.0',
            description='The SPI directory, where the first digit is the SPI port and the second digit is the chip select number. Defaults to BMI088'
        ),
        DeclareLaunchArgument(
            'frequency',
            default_value='200',
            description='The IMU sample frequency.'
        ),
        DeclareLaunchArgument(
            'launch-prefix',
            default_value='',
            description='Start nodes with launch-prefix.'
        ),
        DeclareLaunchArgument(
            'output',
            default_value='screen',
            description='Logging directory.'
        ),
        Node(
            package='mav_imu',
            executable='mav_imu_node',
            output=LaunchConfiguration('output'),
            prefix=LaunchConfiguration('launch-prefix'),
            parameters=[
                {'imu': LaunchConfiguration('imu')},
                {'spi_path': LaunchConfiguration('spi_path')},
                {'frequency': LaunchConfiguration('frequency')}
            ]
        )
    ])