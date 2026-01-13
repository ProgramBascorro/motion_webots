from setuptools import setup

package_name = "op3_joy_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/op3_joy_teleop.launch.py"]),
        ("share/" + package_name + "/config", ["config/op3_joy_teleop.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="farhan",
    maintainer_email="farhan@todo.todo",
    description="Gamepad teleop bridge for OP3 walking module",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "op3_joy_teleop = op3_joy_teleop.teleop_node:main",
        ],
    },
)
