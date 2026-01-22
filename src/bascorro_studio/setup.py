from setuptools import setup

package_name = "bascorro_studio"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/bascorro_studio.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="farhan",
    maintainer_email="farhan@todo.todo",
    description="Studio UI and telemetry bridge for OP3 workflows.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "apply_node = bascorro_studio.apply_node:main",
            "asset_server = bascorro_studio.asset_server:main",
            "studio_agent = bascorro_studio.studio_agent:main",
        ],
    },
)
