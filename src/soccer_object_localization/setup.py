from glob import glob
from setuptools import find_packages, setup

package_name = "soccer_object_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="farhan",
    maintainer_email="farhan@todo.todo",
    description="Field line detector adapted from soccerbot.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector_fieldline = soccer_object_localization.detector_fieldline_ros:main",
        ],
    },
)
