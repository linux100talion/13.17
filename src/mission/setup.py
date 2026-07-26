from setuptools import find_packages, setup

package_name = "mission_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andriy Kutsevol",
    maintainer_email="andriykutsevol@gmail.com",
    description="Миссии (FSM фаз) поверх control_pkg. Срез 1: bootstrap gz-hold+shuttle.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "bootstrap_arch2 = mission_pkg.nodes.bootstrap_node:main",
        ],
    },
)
