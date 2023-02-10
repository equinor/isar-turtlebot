from setuptools import find_packages, setup

setup(
    name="isar_turtlebot",
    description="Integration and Supervisory control of Autonomous Robots - Turtlebot3 implementation",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    version="0.0.1",
    author="Equinor ASA",
    author_email="fg_robots_dev@equinor.com",
    url="https://github.com/equinor/isar-turtlebot",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    package_data={
        "isar_turtlebot": [
            "settings/maps/turtleworld.json",
            "settings/maps/klab_turtlebot_old.json",
            "settings/maps/klab_turtlebot.json",
            "config/settings.env",
        ]
    },
    classifiers=[
        "Environment :: Other Environment",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Programming Language :: Python",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Physics",
        "Topic :: Software Development :: Libraries",
    ],
    include_package_data=True,
    setup_requires=["wheel"],
    install_requires=[
        "alitra",
        "isar",
        "numpy",
        "paho-mqtt",
        "Pillow",
        "pydantic",
        "roslibpy",
        "scipy",
        "pynput",
    ],
    extras_require={"dev": ["pytest", "black", "mypy", "flake8", "pre-commit"]},
    python_requires=">=3.10",
    tests_require=["pytest"],
)
