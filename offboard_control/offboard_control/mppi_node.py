"""Chapter 4 package structure extended with the canonical chapter 9 controller."""
import argparse
from pathlib import Path
from mppi_series.ros_interface import run

def main():
    parser=argparse.ArgumentParser(description='Unarmed, bounded SITL MPPI interface validation')
    parser.add_argument('--output',type=Path,required=True)
    args,ros_args=parser.parse_known_args()
    return run(args.output,ros_args)
