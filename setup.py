#!/usr/bin/env python3
"""
Setup script for gsv8-ethercat package
"""

from setuptools import setup, find_packages
from pathlib import Path

# Read README
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text(encoding="utf-8")

setup(
    name="gsv8-ethercat",
    version="0.1.0",
    author="Konstantinos Katsampiris Salgado",
    author_email="katsampiris.konst@gmail.com",
    description="EtherCAT interface for ME-Messsysteme GSV-8 force/torque sensor amplifier",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/gsv8-ethercat",
    project_urls={
        "Bug Reports": "https://github.com/yourusername/gsv8-ethercat/issues",
        "Source": "https://github.com/yourusername/gsv8-ethercat",
        "Documentation": "https://github.com/yourusername/gsv8-ethercat#readme",
    },
    packages=find_packages(exclude=["tests", "tests.*", "examples", "docs"]),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering",
        "Topic :: System :: Hardware :: Hardware Drivers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Operating System :: Microsoft :: Windows",
        "Operating System :: POSIX :: Linux",
    ],
    python_requires=">=3.7",
    install_requires=[
        "pysoem>=1.0.0",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0",
            "black>=22.0",
            "pylint>=2.0",
            "twine>=4.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "gsv8-diagnostics=gsv8_ethercat.cli:run_diagnostics_cli",
            "gsv8-calibrate=gsv8_ethercat.cli:run_calibration_cli",
            "gsv8-monitor=gsv8_ethercat.cli:run_monitor_cli",
        ],
    },
    keywords="ethercat, force sensor, torque sensor, GSV-8, ME-Messsysteme, industrial automation",
    include_package_data=True,
    zip_safe=False,
)
