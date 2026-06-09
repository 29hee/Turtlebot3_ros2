"""Package setup for rosout_otel_bridge."""

import os
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "rosout_otel_bridge"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", PACKAGE_NAME)]),
        (os.path.join("share", PACKAGE_NAME), ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Austin",
    maintainer_email="hi@haklee.me",
    description="Bridge /rosout ROS2 logs into the ObservabilityStack over OTLP.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = rosout_otel_bridge.bridge_node:main",
        ],
    },
)
