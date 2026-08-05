from glob import glob
from setuptools import find_packages, setup


package_name = "piper_x_face_follow"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml") + glob("config/*.rviz"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ethan",
    maintainer_email="ethan@example.com",
    description="Depth-aware ergonomic hand following for a PIPER-X keyboard tray with J6 locked.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "face_center_detector = piper_x_face_follow.face_center_detector:main",
            "face_follow_controller = piper_x_face_follow.face_follow_controller:main",
            "cartesian_hand_controller = piper_x_face_follow.cartesian_hand_controller:main",
            "ergonomic_target = piper_x_face_follow.ergonomic_target_node:main",
            "hand_follow_data_recorder = piper_x_face_follow.hand_follow_data_recorder:main",
        ],
    },
    scripts=["scripts/mediapipe_body_hands"],
)
