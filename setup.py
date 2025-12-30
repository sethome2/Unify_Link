"""
Setup script for Unify Link Python package.
This is optional if using pyproject.toml with scikit-build-core,
but included for compatibility with older Python versions.
"""

from skbuild import setup
from setuptools import find_packages

setup(
    name="unify-link",
    version="1.0.0",
    description="Unified communication protocol library for embedded systems",
    long_description=open("PYTHON_README.md").read(),
    long_description_content_type="text/markdown",
    author="sethome",
    author_email="your.email@example.com",
    url="*",
    project_urls={
        "Documentation": "*",
        "Source": "*",
        "Tracker": "*",
    },
    license="MIT",
    python_requires=">=3.6",
    packages=find_packages("python"),
    package_dir={"": "python"},
    cmake_args=[
        "-DUNIFY_LINK_BUILD_PYTHON=ON",
        "-DUNIFY_LINK_BUILD_TESTS=OFF",
        "-DUNIFY_LINK_BUILD_EXAMPLES=OFF",
    ],
    zip_safe=False,
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: C++",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Topic :: Software Development :: Libraries",
        "Topic :: System :: Hardware :: Hardware Drivers",
    ],
    keywords="communication protocol embedded motor encoder",
)
