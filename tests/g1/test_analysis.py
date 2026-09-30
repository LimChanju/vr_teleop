import json
import unittest
import numpy as np
from scripts.g1.analyze_results import analyze_trace, reset_valid_mask


def make_trace(n=100,dt=.02):
    t=np.arange(n)*dt
    target=np.zeros((n,3,3));target[:,1,0]=.1*np.sin(t*4)
    return dict(target=target,actual=target.copy(),q=np.zeros((n,29)),action=np.zeros((n,29)),
                done=np.zeros(n,dtype=bool),dt=np.array(dt),enabled=np.ones(n,dtype=bool))


def with_locomotion(d):
    n=len(d['done'])
    d.update(root_position_w=np.zeros((n,3)),root_quaternion_wxyz=np.tile([1.,0.,0.,0.],(n,1)),
             root_linear_velocity_b=np.zeros((n,3)),foot_contact=np.zeros((n,2),dtype=bool),
             foot_linear_velocity_w=np.zeros((n,2,3)))
    return d


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

    def test_old_traces_keep_existing_report_without_optional_diagnostics(self):
        result,_=analyze_trace(make_trace())
        self.assertNotIn('locomotion',result)

    def test_root_segments_do_not_bridge_reset_jumps_and_unwrap_yaw(self):
        d=with_locomotion(make_trace(8,dt=.1));d['done'][3]=True
        d['root_position_w'][:,0]=[0,.1,.2,100,200,200.1,200.2,200.3]
        yaw=np.deg2rad([170,175,-179,0,-170,-165,-160,-155])
        d['root_quaternion_wxyz'][:,0]=np.cos(yaw/2)
        d['root_quaternion_wxyz'][:,3]=np.sin(yaw/2)
        d['root_linear_velocity_b'][:,0]=1.
        result,_=analyze_trace(d,reset_guard=0);report=result['locomotion']
        segments=report['root_motion_segments']
        self.assertEqual([(s['start_frame'],s['end_frame'])for s in segments],[(0,2),(4,7)])
        self.assertAlmostEqual(segments[0]['planar_net_displacement_m'],.2)
        self.assertAlmostEqual(segments[1]['planar_path_length_m'],.3)
        self.assertAlmostEqual(report['root_planar_path_length_m'],.5)
        self.assertAlmostEqual(segments[0]['yaw_drift_rad'],np.deg2rad(11))
        self.assertAlmostEqual(segments[1]['yaw_drift_rad'],np.deg2rad(15))
        self.assertAlmostEqual(report['root_planar_speed_from_position_mps']['max'],1.)
        self.assertAlmostEqual(segments[0]['root_speed_mps']['mean'],1.)

    def test_foot_transitions_and_contact_velocity_exclude_reset_boundary(self):
        d=with_locomotion(make_trace(8));d['done'][3]=True
        d['foot_contact'][:,0]=[1,0,1,1,0,1,0,0]
        d['foot_contact'][:,1]=[0,1,0,1,1,0,0,1]
        d['foot_linear_velocity_w'][:,0,0]=[.1,.2,.3,99,.5,.6,.7,.8]
        d['foot_linear_velocity_w'][:,1,1]=.4
        result,_=analyze_trace(d,reset_guard=0);feet=result['locomotion']['feet']
        for side in ['left','right']:
            self.assertEqual(feet[side]['liftoff_count'],2)
            self.assertEqual(feet[side]['touchdown_count'],2)
            self.assertAlmostEqual(feet[side]['contact_fraction'],3/7)
        speed=feet['left']['foot_link_horizontal_velocity_during_contact_mps']
        self.assertEqual(speed['count'],3)
        self.assertAlmostEqual(speed['mean'],1/3)
        self.assertAlmostEqual(speed['max'],.6)
        self.assertIn('not exact contact-point slip',result['locomotion']['foot_velocity_interpretation'])

    def test_invalid_optional_data_is_flagged_and_splits_segments(self):
        d=with_locomotion(make_trace(10,dt=.1));d['root_position_w'][:,0]=np.arange(10)*.1
        d['root_position_w'][3,0]=np.nan
        d['foot_contact']=d['foot_contact'].astype(float);d['foot_contact'][5,0]=np.nan
        d['root_quaternion_wxyz'][7]=0.
        result,base_valid=analyze_trace(d);report=result['locomotion']
        self.assertTrue(base_valid.all())  # Optional diagnostics do not erase old tracking statistics.
        self.assertFalse(report['data_valid'])
        self.assertEqual(report['invalid_rows']['root_position_w'],1)
        self.assertEqual(report['nonfinite_rows']['foot_contact'],1)
        self.assertEqual(report['invalid_rows']['root_quaternion_wxyz'],1)
        self.assertEqual(report['nonfinite_rows']['root_quaternion_wxyz'],0)
        self.assertEqual([(s['start_frame'],s['end_frame'])for s in report['root_motion_segments']],
                         [(0,2),(4,4),(6,6),(8,9)])
        self.assertAlmostEqual(report['root_planar_path_length_m'],.3)
        json.dumps(result,allow_nan=False)

    def test_invalid_contact_values_and_wrong_shapes_cannot_appear_valid(self):
        d=with_locomotion(make_trace(10));d['foot_contact']=np.full((10,2),2)
        result,_=analyze_trace(d);report=result['locomotion']
        self.assertFalse(report['data_valid']);self.assertEqual(report['valid_rows'],0)
        self.assertIsNone(report['feet']['left']['contact_fraction'])
        self.assertEqual(report['root_motion_segments'],[])
        d['root_position_w']=np.zeros((10,2))
        with self.assertRaises(ValueError):analyze_trace(d)

    def test_manual_reset_uses_same_guard_for_optional_metrics(self):
        d=with_locomotion(make_trace(10));d['manual_reset']=np.zeros(10,bool);d['manual_reset'][5]=True
        result,_=analyze_trace(d)
        segments=result['locomotion']['root_motion_segments']
        self.assertEqual([(s['start_frame'],s['end_frame'])for s in segments],[(0,3),(7,9)])

    def test_partial_optional_velocity_data_is_supported(self):
        d=make_trace(10);d['root_linear_velocity_b']=np.tile([3.,4.,12.],(10,1))
        result,_=analyze_trace(d);report=result['locomotion']
        self.assertTrue(report['data_valid'])
        self.assertAlmostEqual(report['root_body_xy_speed_mps']['mean'],5.)
        self.assertAlmostEqual(report['root_speed_mps']['mean'],13.)
        self.assertNotIn('feet',report);self.assertNotIn('root_motion_segments',report)

if __name__=='__main__':unittest.main()
