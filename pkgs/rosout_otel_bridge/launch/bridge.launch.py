"""Launch the rosout -> OTLP bridge with overridable OTLP endpoint and service name."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the rosout_otel_bridge node."""
    otlp_endpoint = LaunchConfiguration("otlp_endpoint")
    service_name = LaunchConfiguration("service_name")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "otlp_endpoint",
                default_value="http://localhost:4318",
                description="OTLP/HTTP base endpoint of the collector.",
            ),
            DeclareLaunchArgument(
                "service_name",
                default_value="ros2",
                description="service.name attached to exported logs.",
            ),
            Node(
                package="rosout_otel_bridge",
                executable="bridge",
                name="rosout_otel_bridge",
                output="screen",
                parameters=[
                    {"otlp_endpoint": otlp_endpoint, "service_name": service_name}
                ],
            ),
        ]
    )
