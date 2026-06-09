"""Unit tests for the rclpy-free translation helpers."""

import logging

from rosout_otel_bridge.translate import build_attributes, ros_level_to_python


def test_known_levels_map_to_python():
    assert ros_level_to_python(10) == logging.DEBUG
    assert ros_level_to_python(20) == logging.INFO
    assert ros_level_to_python(30) == logging.WARNING
    assert ros_level_to_python(40) == logging.ERROR
    assert ros_level_to_python(50) == logging.CRITICAL


def test_unknown_level_defaults_to_info():
    assert ros_level_to_python(0) == logging.INFO
    assert ros_level_to_python(99) == logging.INFO


def test_attributes_include_node_name():
    attributes = build_attributes("nav2", "planner.cpp", "compute", 42)
    assert attributes["ros.node"] == "nav2"
    assert attributes["code.filepath"] == "planner.cpp"
    assert attributes["code.function"] == "compute"
    assert attributes["code.lineno"] == 42


def test_empty_source_location_is_omitted():
    attributes = build_attributes("nav2", "", "", 0)
    assert attributes == {"ros.node": "nav2"}
