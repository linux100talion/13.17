import os
from glob import glob

from setuptools import setup

package_name = "nav_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andriy Kutsevol",
    maintainer_email="andriykutsevol@gmail.com",
    description="Навигационные нейросети (NN1/NN2) + OpenHD-стример с оверлеем детекций.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "openhd_streamer = nav_pkg.openhd_streamer:main",
            "nn1_anchor = nav_pkg.nn1_anchor:main",
            "nn2_scene = nav_pkg.nn2_scene:main",
        ],
    },
)
