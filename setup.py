from setuptools import setup, find_packages

setup(
    name="homebot",
    version="0.1.0",
    description="HomeBot 家用机器人控制软件",
    packages=find_packages(where="software/src"),
    package_dir={"": "software/src"},
    install_requires=[
        "pyzmq>=25.0.0",
        "opencv-python>=4.8.0",
        "pyserial>=3.5",
        "flask>=3.0.0",
        "flask-socketio>=5.3.0",
        "ultralytics>=8.3.0",
        "numpy>=1.24.0",
        "filterpy>=1.4.5",
    ],
    include_package_data=True,
)
