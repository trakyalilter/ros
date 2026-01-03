"""
Setup configuration for Navigation Viewer package.
"""

from setuptools import setup, find_packages

setup(
    name="nav_viewer",
    version="1.0.0",
    description="PyQt5 ROS2 Navigation Viewer with Hexagonal Architecture",
    author="Navigation Team",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "PyQt5>=5.15.0",
        "numpy>=1.20.0",
    ],
    entry_points={
        "console_scripts": [
            "nav-viewer=nav_viewer.main:main",
        ],
    },
    classifiers=[
        "Development Status :: 4 - Beta",
        "Environment :: X11 Applications :: Qt",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Topic :: Scientific/Engineering :: Visualization",
    ],
)
