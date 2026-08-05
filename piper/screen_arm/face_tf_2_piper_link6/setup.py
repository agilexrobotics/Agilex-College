from glob import glob
import os

from setuptools import find_packages, setup


package_name = "face_tf_2_piper_link6"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lansijian",
    maintainer_email="2033374848@qq.com",
    description="Convert a face TF into a fixed-orientation Piper arm_pose TF.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "face_tf_2_piper_link6 = "
            "face_tf_2_piper_link6.face_tf_2_piper_link6_node:main",
        ],
    },
)
