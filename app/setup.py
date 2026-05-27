#!/usr/bin/env python3
import os
import ssl
from setuptools import setup

if not os.environ.get("PYTHONHTTPSVERIFY", "") and getattr(ssl, "_create_unverified_context", None):
    ssl._create_default_https_context = ssl._create_unverified_context

setup(
    name="wsExt1",
    version="0.1.0",
    description="WaterSampler python backend",
    license="MIT",
    install_requires=[
        "pymavlink==2.4.49",
        "pyserial==3.5",
        "uvicorn==0.39.0",
        "numpy==2.0.2",
        "fastapi==0.135.3",
        "requests==2.32.5",
        "smbus2==0.6.1"
    ],
)
