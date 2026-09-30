import unittest
import numpy as np
from scripts.g1.analyze_results import analyze_trace, reset_valid_mask


def make_trace(n=100,dt=.02):
    t=np.arange(n)*dt
    target=np.zeros((n,3,3));target[:,1,0]=.1*np.sin(t*4)
    return dict(target=target,actual=target.copy(),q=np.zeros((n,29)),action=np.zeros((n,29)),
                done=np.zeros(n,dtype=bool),dt=np.array(dt),enabled=np.ones(n,dtype=bool))


class AnalysisTests(unittest.TestCase):
    def test_reset_jump_excluded_from_error_and_derivative(self):
        d=make_trace();d['done'][50]=True;d['actual'][50]=100;d['q'][50:]=100
        result,valid=analyze_trace(d)
        np.testing.assert_array_equal(np.flatnonzero(~valid),[49,50,51])
        self.assertEqual(result['reset_rows'],1)
        self.assertEqual(result['mean_point_tracking_m']['max'],0.)
        self.assertEqual(result['joint_speed_rad_s']['max'],0.)

    def test_nonfinite_values_flagged_not_hidden(self):
        d=make_trace();d['action'][30,0]=np.nan;d['q'][40,2]=np.inf
        result,valid=analyze_trace(d)
        self.assertFalse(result['finite']['action']);self.assertFalse(result['finite']['q'])
        self.assertEqual(result['nonfinite_rows']['action'],1)
        self.assertFalse(valid[30]);self.assertFalse(valid[40])

    def test_manual_reset_boundary_is_excluded(self):
        d=make_trace();d['manual_reset']=np.zeros(100,dtype=bool)
        d['manual_reset'][50]=True;d['actual'][50]=100;d['q'][50:]=100
        result,valid=analyze_trace(d)
        np.testing.assert_array_equal(np.flatnonzero(~valid),[49,50,51])
        self.assertEqual(result['reset_rows'],1)
        self.assertEqual(result['mean_point_tracking_m']['max'],0.)
        self.assertEqual(result['joint_speed_rad_s']['max'],0.)

    def test_tracking_gain_and_lag_diagnostic(self):
        d=make_trace(500);d['actual'][:,1,0]=.6*d['target'][:,1,0]
        result,_=analyze_trace(d)
        response=result['response']['left_wrist']['x']
        self.assertAlmostEqual(response['least_squares_gain'],.6,places=8)
        self.assertAlmostEqual(response['correlation'],1,places=8)
        self.assertEqual(response['best_positive_lag_seconds'],0.)
        self.assertIsNone(result['response']['head']['x']['correlation'])

    def test_empty_valid_window_is_explicit_and_shape_invalid_rejected(self):
        d=make_trace(10);d['done'][:]=True
        result,valid=analyze_trace(d)
        self.assertFalse(valid.any());self.assertEqual(result['valid_rows'],0)
        self.assertIsNone(result['mean_point_tracking_m']['mean'])
        d['q']=d['q'][:,:5]
        with self.assertRaises(ValueError):analyze_trace(d)
        with self.assertRaises(ValueError):reset_valid_mask([False],-1)

if __name__=='__main__':unittest.main()
