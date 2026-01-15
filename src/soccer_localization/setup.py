from setuptools import find_packages, setup

package_name = "soccer_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/localization.launch.py"]),
        ("share/" + package_name + "/rviz", ["rviz/op3_localization.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="farhan",
    maintainer_email="farhan@todo.todo",
    description="Field line UKF localization adapted from soccerbot.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "field_lines_ukf = soccer_localization.field_lines_ukf_ros:main",
            "field_markers = soccer_localization.field_markers:main",
        ],
    },
)
