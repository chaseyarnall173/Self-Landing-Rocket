"""Summarise every RASP file in sim/data/motors.

    python sim/scripts/motor_table.py
"""
import _path
from rocketsim.designs import DATA_DIR
from rocketsim.motors import Motor

for f in sorted(DATA_DIR.glob("*.eng")):
    print(Motor.from_eng(f).summary())
