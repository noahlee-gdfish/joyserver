#!/usr/bin/python

import os
import sys
import configparser

def get_config(key):
    properties = configparser.ConfigParser()
    if getattr(sys, 'frozen', False):
        config_file = os.path.join(os.path.dirname(os.path.abspath(sys.executable)), "config/config.ini")
    else:
        config_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), "config/config.ini")

    properties.read(config_file, encoding='utf-8')
    config = properties[key]

    return config
