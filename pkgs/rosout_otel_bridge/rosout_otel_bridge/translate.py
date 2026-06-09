"""Pure translation helpers from rcl_interfaces/msg/Log to OpenTelemetry inputs.

This module intentionally has no ``rclpy`` dependency so it can be unit-tested on
any host. It maps a ROS log severity to a Python ``logging`` level and extracts the
attributes that should ride along on the emitted OpenTelemetry log record.
"""

import logging

#: ROS severity level (rcl_interfaces/msg/Log) -> Python logging level.
#: The numeric values already align (WARN=30, FATAL=50), but the mapping is made
#: explicit so an unexpected value degrades to INFO rather than passing through.
ROS_LEVEL_TO_PYTHON = {
    10: logging.DEBUG,
    20: logging.INFO,
    30: logging.WARNING,
    40: logging.ERROR,
    50: logging.CRITICAL,
}


def ros_level_to_python(level):
    """Translate a ROS log severity to a Python logging level.

    :param level: The ``rcl_interfaces/msg/Log`` severity (10/20/30/40/50).
    :returns: The matching Python logging level, or ``logging.INFO`` if unknown.
    """
    return ROS_LEVEL_TO_PYTHON.get(int(level), logging.INFO)


def build_attributes(name, file, function, line):
    """Build the OpenTelemetry attribute mapping for one ROS log record.

    Empty source-location fields are omitted so they do not clutter the record.

    :param name: The originating ROS node / logger name.
    :param file: Source file the log call came from.
    :param function: Source function the log call came from.
    :param line: Source line number the log call came from.
    :returns: A dict of OpenTelemetry log-record attributes.
    """
    attributes = {"ros.node": name}
    if file:
        attributes["code.filepath"] = file
    if function:
        attributes["code.function"] = function
    if line:
        attributes["code.lineno"] = int(line)
    return attributes
