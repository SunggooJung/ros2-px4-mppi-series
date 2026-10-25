"""Model and command-contract risks for chapter 9-1, not flight tests."""
import unittest
import numpy as np
from .controller import Config,MPPIController,dynamics,dynamics_batch,LOW,HIGH,attitude_target_fields


class ControllerTests(unittest.TestCase):
    def test_hover_equilibrium(self):
        cfg=Config();state=np.zeros(12);state[2]=3.
        original=state.copy()
        for _ in range(100):state=dynamics(state,cfg.hover,cfg)
        np.testing.assert_allclose(state,original,atol=1e-12)

    def test_scalar_batch_match(self):
        rng=np.random.default_rng(23);state=rng.normal(0,.15,(128,12));inputs=rng.uniform(LOW,HIGH,(128,4))
        scalar=np.array([dynamics(s,u) for s,u in zip(state,inputs)])
        np.testing.assert_allclose(dynamics_batch(state,inputs),scalar,atol=1e-12,rtol=0)

    def test_euler_rate_is_not_body_rate(self):
        s=np.zeros(12);s[7]=np.pi/4;s[11]=1.;cfg=Config()
        nxt=dynamics(s,cfg.hover,cfg)
        self.assertAlmostEqual(nxt[6],cfg.dt)
        self.assertAlmostEqual(nxt[8],np.sqrt(2)*cfg.dt)

    def test_rate_response_has_explicit_lag(self):
        s=np.zeros(12);u=Config().hover;u[1]=1.
        nxt=dynamics(s,u)
        self.assertAlmostEqual(nxt[9],1-np.exp(-.05/.08))
        self.assertLess(nxt[9],u[1])

    def test_no_hover_at_zero_or_low_thrust(self):
        s=np.zeros(12);u=Config().hover;u[0]=LOW[0]
        self.assertLess(dynamics(s,u)[5],0)
        u[0]=0
        with self.assertRaises(ValueError):dynamics(s,u)

    def test_warm_start_uses_hover_tail(self):
        c=MPPIController();s=np.zeros(12);s[2]=3.
        u=c.step(s,[1,0,3])
        np.testing.assert_allclose(u,c.last['updated_sequence'][0])
        np.testing.assert_allclose(c.nominal[:-1],c.last['updated_sequence'][1:])
        np.testing.assert_array_equal(c.nominal[-1],c.cfg.hover)

    def test_output_bounds_and_finite_weights(self):
        c=MPPIController();s=np.zeros(12);s[2]=3.
        for goal in [[0,0,3],[1,0,3],[-1,1,3],[0,0,2],[2,-2,3]]:
            u=c.step(s,goal)
            self.assertTrue(np.isfinite(u).all());self.assertTrue(np.all(u>=LOW)&np.all(u<=HIGH))
            self.assertAlmostEqual(c.last['weights'].sum(),1.)
            self.assertTrue(np.isfinite(c.last['weights']).all())

    def test_invalid_costs_excluded_and_all_invalid_rejected(self):
        w=MPPIController.weights([1.,np.nan,np.inf,2.],1.)
        self.assertEqual(w[1],0.);self.assertEqual(w[2],0.)
        self.assertAlmostEqual(w[3]/w[0],np.exp(-1))
        with self.assertRaises(ValueError):MPPIController.weights([np.nan,np.inf],1.)

    def test_temperature_changes_weight_concentration(self):
        low=MPPIController.weights([0,1,2],.1);high=MPPIController.weights([0,1,2],10.)
        self.assertGreater(low[0],high[0])

    def test_same_seed_reproduces_commands(self):
        a,b=MPPIController(Config(seed=2)),MPPIController(Config(seed=2));s=np.zeros(12);s[2]=3.
        for _ in range(4):np.testing.assert_array_equal(a.step(s,[1,0,3]),b.step(s,[1,0,3]))

    def test_singular_and_nonfinite_states_rejected(self):
        for s in [np.full(12,np.nan),np.r_[np.zeros(7),1.3,np.zeros(4)],np.zeros(6)]:
            with self.assertRaises(ValueError):MPPIController().step(s,[0,0,3])
        with self.assertRaises(ValueError):MPPIController().step(np.zeros(12),[np.inf,0,3])

    def test_parameter_validation(self):
        for changes in [dict(temperature=0),dict(rate_tau=0),dict(thrust_hover=np.nan),dict(samples=1),dict(horizon=0),dict(dt=.2)]:
            with self.assertRaises(ValueError):Config(**changes)

    def test_attitude_target_contract(self):
        fields=attitude_target_fields([.6,.1,-.2,.3])
        self.assertEqual(fields['type_mask'],128)
        self.assertEqual(fields['body_rate_flu_rad_s'],[.1,-.2,.3])
        self.assertEqual(fields['thrust_01'],.6)
        for u in [[19.62,0,0,0],[.6,2,0,0]]:
            with self.assertRaises(ValueError):attitude_target_fields(u)


if __name__=='__main__':unittest.main()
