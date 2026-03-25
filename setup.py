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
        "sherpa-onnx>=1.9.0",
        "sounddevice>=0.4.6",
        "openai>=1.0.0",
        "fastmcp>=2.14.0",
        "websockets>=12.0",
        "volcengine-python-sdk",
    ],
    include_package_data=True,
)
