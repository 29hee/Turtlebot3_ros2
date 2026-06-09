"""ROS2 node that forwards /rosout log records to the ObservabilityStack via OTLP.

It subscribes to ``/rosout`` (``rcl_interfaces/msg/Log``) and emits each record as
an OpenTelemetry log over OTLP/HTTP to the collector. Only logs produced through the
ROS logging API flow through ``/rosout`` — raw ``print``/stdout, Gazebo internals,
and launch-system output do not.

Two feedback loops are deliberately avoided:

1. The ROS loop: the bridge never logs through the ROS logging API (it would land
   back on ``/rosout`` and be re-exported), and it skips records emitted by its own
   node name.
2. The Python-logging loop: the OpenTelemetry handler is attached to a dedicated,
   non-propagating logger, so the exporter's own internal logging on the root logger
   is never captured and re-exported.
"""

import logging
import os

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from opentelemetry.exporter.otlp.proto.http._log_exporter import OTLPLogExporter
from opentelemetry.sdk._logs import LoggerProvider, LoggingHandler
from opentelemetry.sdk._logs.export import BatchLogRecordProcessor
from opentelemetry.sdk.resources import Resource

from rosout_otel_bridge.translate import build_attributes, ros_level_to_python

#: QoS profile matching the rcl ``/rosout`` publisher (reliable, volatile,
#: keep-last depth 1000). A mismatch here silently delivers zero messages.
ROSOUT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1000,
)


class RosoutOtelBridge(Node):
    """Subscribes to /rosout and exports each record as an OpenTelemetry log."""

    def __init__(self):
        super().__init__("rosout_otel_bridge")
        endpoint_base = self.declare_parameter(
            "otlp_endpoint",
            os.environ.get("OTEL_EXPORTER_OTLP_ENDPOINT", "http://localhost:4318"),
        ).value
        service_name = self.declare_parameter(
            "service_name", os.environ.get("OTEL_SERVICE_NAME", "ros2")
        ).value
        self._include_self = self.declare_parameter("include_self", False).value
        logs_endpoint = endpoint_base.rstrip("/") + "/v1/logs"
        resource = Resource.create(
            {"service.name": service_name, "service.namespace": "turtlebot3"}
        )
        self._provider = LoggerProvider(resource=resource)
        self._provider.add_log_record_processor(
            BatchLogRecordProcessor(OTLPLogExporter(endpoint=logs_endpoint))
        )
        self._otel_logger = logging.getLogger("rosout_otel_bridge.export")
        self._otel_logger.setLevel(logging.DEBUG)
        self._otel_logger.addHandler(
            LoggingHandler(level=logging.NOTSET, logger_provider=self._provider)
        )
        self._otel_logger.propagate = False
        self.create_subscription(Log, "/rosout", self._on_log, ROSOUT_QOS)
        print(
            f"[rosout_otel_bridge] exporting /rosout -> {logs_endpoint} "
            f"as service.name={service_name}",
            flush=True,
        )

    def _on_log(self, msg):
        """Forward one /rosout record as an OpenTelemetry log."""
        if not self._include_self and msg.name == self.get_name():
            return
        self._otel_logger.log(
            ros_level_to_python(msg.level),
            msg.msg,
            extra=build_attributes(msg.name, msg.file, msg.function, msg.line),
        )

    def shutdown(self):
        """Flush and shut down the OpenTelemetry exporter."""
        self._provider.shutdown()


def main(args=None):
    """Entry point: spin the bridge until interrupted, flushing on exit."""
    rclpy.init(args=args)
    node = RosoutOtelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
