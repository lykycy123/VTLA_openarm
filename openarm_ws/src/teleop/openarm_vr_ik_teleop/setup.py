from setuptools import setup, find_packages

package_name = "openarm_vr_ik_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/ik_teleop.launch.py"]),
        ("share/" + package_name + "/scripts", [
            "scripts/run_check_urdf_placo.sh",
            "scripts/run_ik_teleop.sh",
        ]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="OpenArm",
    maintainer_email="openarm@enactic.ai",
    description="VR teleoperation with Placo IK for OpenArm bimanual",
    license="Apache-2.0",
        entry_points={
        "console_scripts": [
            "check_urdf_placo = openarm_vr_ik_teleop.check_urdf_placo:main",
            "ik_teleop_node = openarm_vr_ik_teleop.ik_teleop_node:main",
            "print_frames = openarm_vr_ik_teleop.print_frames:main",
        ],
    },
)
