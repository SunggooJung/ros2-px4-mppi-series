import unittest
import numpy as np
from .ros_flight import flight_gate


class FlightGuards(unittest.TestCase):
    def test_armed_hover_is_valid_but_ground_stale_and_overrun_rejected(self):
        state=np.zeros(12);state[2]=2.
        args=(state,True,True,'OFFBOARD',.02,.03)
        self.assertIsNone(flight_gate(*args))
        for change in [('armed',False),('mode','AUTO.LAND'),('state_age',.3),('compute_s',.06)]:
            values=dict(state=state,connected=True,armed=True,mode='OFFBOARD',state_age=.02,header_age=.03,compute_s=0)
            values[change[0]]=change[1]
            with self.subTest(change=change):self.assertIsNotNone(flight_gate(**values))
        for index,value in [(2,0.),(2,5.),(0,5.),(6,.7),(3,float('nan'))]:
            s=state.copy();s[index]=value
            with self.subTest(index=index,value=value):self.assertIsNotNone(flight_gate(s,True,True,'OFFBOARD',.02,.03))
