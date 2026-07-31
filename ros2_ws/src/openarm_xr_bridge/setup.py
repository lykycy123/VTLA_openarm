from setuptools import setup, find_packages

package_name = "openarm_xr_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/xr_bridge.launch.py"]),
        ("share/" + package_name + "/scripts", ["scripts/run_xr_bridge.sh"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenArm",
    maintainer_email="openarm@enactic.ai",
    description="Bridge XRoboToolkit VR data to ROS 2 topics for OpenArm teleoperation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "xr_bridge_node = openarm_xr_bridge.xr_bridge_node:main",
        ],
    },
)
