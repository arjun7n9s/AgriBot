from setuptools import find_packages, setup

package_name = "agribot_autonomy"

setup(
    name=package_name,
    version="0.4.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/autonomy.yaml"]),
        ("share/" + package_name + "/launch", ["launch/autonomy.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="arjun7n9s",
    maintainer_email="arjun7n9s@gmail.com",
    description="Row following, fruit detection and the harvest mission for AgriBOT.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drive_node = agribot_autonomy.drive_node:main",
            "row_follower = agribot_autonomy.row_follower_node:main",
            "fruit_detector = agribot_autonomy.fruit_detector_node:main",
            "harvest_mission = agribot_autonomy.harvest_mission_node:main",
            "harvest_client = agribot_autonomy.harvest_client:main",
        ],
    },
)
