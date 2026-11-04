from setuptools import setup

package_name = "tello_control_station"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["assets/commands.png", "assets/go1_commands.png"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maxime",
    maintainer_email="maxime@todo.todo",
    description="A control station for tello",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "control_station = tello_control_station.control_station_node:main"
        ],
    },
)
