import os
from glob import glob
from setuptools import setup

package_name = "joy_manual_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="taewook",
    maintainer_email="utsi09@g.skku.edu",
    description="Bridge joy controller command to Autoware manual control topics",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joy_to_manual_bridge = joy_manual_bridge.joy_to_manual_bridge:main",
            "joy_remap = joy_manual_bridge.joy_remap:main",
        ],
    },
)