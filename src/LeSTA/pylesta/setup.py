"""
Modified by: Haoran Wang
Revision date: 2026-08-12
"""

# setup.py

from setuptools import setup, find_namespace_packages

setup(
    name="pylesta",
    version="0.2.0",
    author="Haoran Wang",
    author_email="haoranwang@todo.todo",
    url="",
    packages=find_namespace_packages(),
    install_requires=[],  # requirements.txt
    description="Training a risk-aware self-supervised traversability model with navigation experiences of mobile robots",
    python_requires='>=3.6',
    include_package_data=True,
)
