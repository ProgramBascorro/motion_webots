from setuptools import setup

package_name = "soccer_object_detection"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, f"{package_name}.camera"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="farhan",
    maintainer_email="farhan@todo.todo",
    description="Minimal camera utilities for soccer localization.",
    license="BSD",
    tests_require=["pytest"],
)
