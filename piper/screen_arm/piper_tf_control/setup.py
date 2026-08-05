from glob import glob
import os

from setuptools import find_packages, setup


package_name = "piper_tf_control"

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
    description="Read an arm_pose TF and publish smoothed Piper PosCmd commands.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "piper_tf_control = piper_tf_control.piper_tf_control_node:main",
            "piper_tf_z_test = piper_tf_control.piper_tf_z_test_node:main",
        ],
    },
)
