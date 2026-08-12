"""
Modified by: Haoran Wang
Revision date: 2026-08-12
"""


import os
import yaml as pyyaml


def load(yaml_path):
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"The file {yaml_path} does not exist.")
    return pyyaml.safe_load(open(yaml_path, 'r'))
