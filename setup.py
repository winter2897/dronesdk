import setuptools
import os

version = "1.0.0"

with open(os.path.join(os.path.dirname(__file__), "README.md")) as f:
    LongDescription = f.read()

setuptools.setup(
    name="dronesdk",
    zip_safe=True,
    version=version,
    description="Developer Tools for Drones.",
    long_description_content_type="text/markdown",
    long_description=LongDescription,
    url="https://github.com/winter2897/dronesdk.git",
    author="winter2897",
    install_requires=[
        "pymavlink>=2.2.20",
        "monotonic>=1.3",
    ],
    author_email="haiquantran2897@gmail.com",
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Environment :: Console",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Topic :: Scientific/Engineering",
    ],
    license="gpl-3.0",
    packages=setuptools.find_packages(),
)
