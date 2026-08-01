from glob import glob
from setuptools import find_packages, setup


package_name = "rbm_base_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="arx",
    maintainer_email="arx@todo.todo",
    description="Bridge VR joystick input to mobile-base velocity commands.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "base_controller = rbm_base_control.base_controller:main",
        ],
    },
)
