"""Robots package: the edge-computing decision logic that runs on each AMR.

Every module here is written so it could run unmodified on a Raspberry Pi /
Jetson Nano: no dependency on pygame or Flask, only on config.py and
plain-Python data (communication/ handles the network transport separately).
"""
