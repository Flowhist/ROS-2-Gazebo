from setuptools import find_packages, setup
import os
from glob import glob

package_name = "robo_ctrl"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 安装launch文件
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # 安装配置文件
        (os.path.join("share", package_name, "config"), glob("config/*.lua")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
        # 安装模型文件
        (os.path.join("share", package_name, "models"), glob("models/*.sdf")),
        (
            os.path.join("share", package_name, "models/vehicle_robot"),
            glob("models/vehicle_robot/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="flowhist",
    maintainer_email="2944709230@qq.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
)
