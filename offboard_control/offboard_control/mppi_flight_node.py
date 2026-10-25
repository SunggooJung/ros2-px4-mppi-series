"""Explicit bounded SITL flight; separate from the unarmed 10-1 interface."""
import argparse
from pathlib import Path
from mppi_series.ros_flight import run


def main():
    parser=argparse.ArgumentParser(description='Isolated PX4 SITL MPPI flight')
    parser.add_argument('--output',type=Path,required=True)
    args,ros_args=parser.parse_known_args()
    return run(args.output,ros_args)
