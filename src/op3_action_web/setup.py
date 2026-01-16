from setuptools import setup

package_name = "op3_action_web"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/action_web.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="farhan",
    maintainer_email="farhan@todo.todo",
    description="Web UI bridge for editing OP3 action pages.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "apply_node = op3_action_web.apply_node:main",
            "asset_server = op3_action_web.asset_server:main",
        ],
    },
)
