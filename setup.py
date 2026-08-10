from setuptools import setup


package_name = "scratch_hci_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/scratch_hci_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="iszwm3",
    maintainer_email="hkmp5a5.ki@gmail.com",
    description="Bridge Scratch HCI HTTP commands to OperaSim ROS 2 topics.",
    license="Apache-2.0",
    url="https://github.com/KeiIszw/ros2_ws",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scratch_hci_bridge = scratch_hci_bridge.node:main",
        ],
    },
)
