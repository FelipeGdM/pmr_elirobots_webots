import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ec63_webots"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "protos"), glob("protos/*.proto")),
        (os.path.join("share", package_name, "resource"), glob("resource/*")),
        (
            os.path.join("share", package_name, "protos", "meshes", "ec63"),
            glob("protos/meshes/ec63/*.STL"),
        ),
        # ('launch/' + package_name, ['*.py']),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="felipe",
    maintainer_email="felipe@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    # tests_require=['pytest'],
    entry_points={
        "console_scripts": ["ec63_controller = ec63_webots.ec63_controller:main"],
    },
)
