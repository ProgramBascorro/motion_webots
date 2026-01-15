from setuptools import setup

package_name = "op3_ball_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/ball_localizer.launch.py"]),
        ("share/" + package_name + "/config", ["config/ball_localizer.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="farhan",
    maintainer_email="farhan@todo.todo",
    description="Ball localization and approach node for OP3.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ball_localizer = op3_ball_localization.ball_localizer_node:main",
        ],
    },
)
