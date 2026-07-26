from setuptools import find_packages, setup

package_name = "control_pkg"

setup(
    name=package_name,
    version="0.0.1",
    # control_pkg + подпакеты domain/ (чистый), application/, infrastructure/ (ROS).
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andriy Kutsevol",
    maintainer_email="andriykutsevol@gmail.com",
    description="Ядро управления (hexagonal/DDD): стратегии + ControlStack + порты/ROS-адаптеры.",
    license="Proprietary",
    # Библиотечный пакет — нод-точек входа нет (потребитель: mission_pkg). Bare
    # control_node (пилот-ассист) добавит срез 2.
    entry_points={"console_scripts": []},
)
