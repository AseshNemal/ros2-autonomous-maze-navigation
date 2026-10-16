import os
from glob import glob
from setuptools import find_packages, setup

package_name = "dqn_agent"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="AseshNemal",
    maintainer_email="aseshnemal@gmail.com",
    description="Lightweight NumPy DQN training and evaluation for TurtleBot3 Gazebo",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "train_dqn = dqn_agent.train:main",
        "evaluate_dqn = dqn_agent.evaluate:main",
        "compare_navigation = dqn_agent.compare:main",
    ]},
)
