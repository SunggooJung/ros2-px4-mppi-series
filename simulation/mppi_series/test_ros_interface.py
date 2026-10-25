"""Prevent frame mistakes and publication after stale/invalid calculations."""
import math
from types import SimpleNamespace as S
import unittest
import numpy as np
from .ros_interface import odometry_state,quaternion_rotation,gate
from .controller import attitude_target_fields

def vector(x=0.,y=0.,z=0.):return S(x=x,y=y,z=z)
def odom():
    return S(header=S(frame_id='map'),child_frame_id='base_link',
             pose=S(pose=S(position=vector(1,2,3),orientation=S(x=0,y=0,z=math.sqrt(.5),w=math.sqrt(.5)))),
             twist=S(twist=S(linear=vector(1,0,0),angular=vector(.1,.2,.3))))

class ContractTests(unittest.TestCase):
    def test_body_velocity_rotates_to_world_without_rotating_body_rates(self):
        state=odometry_state(odom())
        np.testing.assert_allclose(state[:6],[1,2,3,0,1,0],atol=1e-14)
        np.testing.assert_allclose(state[9:],[.1,.2,.3])
        self.assertAlmostEqual(state[8],math.pi/2)

    def test_unknown_frames_and_invalid_state_rejected(self):
        msg=odom();msg.child_frame_id='odom'
        with self.assertRaises(ValueError):odometry_state(msg)
        msg=odom();msg.twist.twist.linear.x=float('nan')
        with self.assertRaises(ValueError):odometry_state(msg)
        with self.assertRaises(ValueError):quaternion_rotation([0,0,0,0])

    def test_flu_rates_are_not_preconverted(self):
        fields=attitude_target_fields([.6,.1,.2,.3])
        self.assertEqual(fields['type_mask'],128)
        self.assertEqual(fields['body_rate_flu_rad_s'],[.1,.2,.3])

    def test_stale_disconnected_armed_or_overrun_cannot_publish(self):
        valid=dict(connected=True,armed=False,state_age_s=.01,header_age_s=.01,compute_s=.02)
        self.assertIsNone(gate(**valid))
        for change,expected in [({'state_age_s':.3},'stale_state'),({'header_age_s':1},'stale_state'),
                                ({'connected':False},'disconnected'),({'armed':True},'unexpected_armed_vehicle'),
                                ({'compute_s':.051},'compute_overrun')]:
            self.assertEqual(gate(**(valid|change)),expected)

if __name__=='__main__':unittest.main()
