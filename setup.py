# ifp2107 — Ian Pichs, Columbia University, ELEN 6760 Spring 2026
from setuptools import setup

package_name = "lqr_pid_controller"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/lqr_pid_controller.launch.py"]),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="Ian Pichs (ifp2107)",
    maintainer_email="ian.pichs@columbia.edu",
    description="ifp2107 — LQR lateral + PID longitudinal controller for DUT25 skidpad. ELEN 6760 Spring 2026.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller_node = lqr_pid_controller.controller_node:main",
        ],
    },
)
