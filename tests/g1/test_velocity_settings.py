"""CPU-only CLI validation and saved velocity configuration restoration."""
import copy
from types import SimpleNamespace
import unittest

from scripts.g1.run import restore_velocity_settings, validate_linear_options, validate_standing_yaw_options, validate_yaw_options


def configuration():
    return SimpleNamespace(moving_linear_velocity_reward_scale=4., moving_linear_velocity_error_variance=.04,
        moving_yaw_velocity_reward_scale=2., moving_yaw_velocity_error_variance=.1,
        standing_yaw_velocity_reward_scale=1., standing_yaw_velocity_error_variance=.25,
        command_max=(.3,.15,.4), standing_fraction=.3, command_resampling_s=5.,
        command_sampling="mixed", moving_xy_threshold=.08,
        feet_air_time_reward_scale=.5, both_feet_air_penalty_scale=.15)


def metadata():
    return dict(environment_variant="g1_commanded_velocity_v1", training_velocity_limits=[.2,.1,.3],
        training_standing_fraction=.4, velocity_command_resampling_s=4., feet_air_time_reward_scale=.7,
        both_feet_air_penalty_scale=.2, velocity_reward_contract=dict(version="moving_velocity_replacement_v2",
            linear_velocity=dict(scale=3., squared_error_denominator_m2_s2=.03),
            yaw_velocity=dict(scale=4., squared_error_denominator_rad2_s2=.04)))


class VelocitySettingsTests(unittest.TestCase):
    def test_fresh_and_nonvelocity_metadata_keep_defaults(self):
        for saved in (None, {}, {"environment_variant":"sparse_tracking_v2"}):
            cfg=configuration();before=vars(cfg).copy()
            restore_velocity_settings(cfg,saved)
            self.assertEqual(vars(cfg),before)

    def test_saved_nested_contract_and_auxiliary_scales_restore_exactly(self):
        cfg=configuration();saved=metadata();before=copy.deepcopy(saved)
        restore_velocity_settings(cfg,saved)
        self.assertEqual(vars(cfg),dict(moving_linear_velocity_reward_scale=3.,moving_linear_velocity_error_variance=.03,
            moving_yaw_velocity_reward_scale=4.,moving_yaw_velocity_error_variance=.04,
            standing_yaw_velocity_reward_scale=1.,standing_yaw_velocity_error_variance=.25,
            command_max=(.2,.1,.3),standing_fraction=.4,command_resampling_s=4.,
            command_sampling="mixed",moving_xy_threshold=.08,
            feet_air_time_reward_scale=.7,both_feet_air_penalty_scale=.2))
        self.assertEqual(saved,before)

    def test_legacy_without_replacement_contract_restores_original_base_terms(self):
        cfg=configuration();restore_velocity_settings(cfg,{"environment_variant":"g1_commanded_velocity_v1"})
        self.assertEqual((cfg.moving_linear_velocity_reward_scale,cfg.moving_linear_velocity_error_variance),(2.,.25))
        self.assertEqual((cfg.moving_yaw_velocity_reward_scale,cfg.moving_yaw_velocity_error_variance),(1.,.25))
        self.assertEqual(cfg.command_max,(.3,.15,.4))

    def test_invalid_saved_contract_is_rejected_without_partial_mutation(self):
        cases=[]
        saved=metadata();saved['velocity_reward_contract']['version']='unknown';cases.append(saved)
        saved=metadata();del saved['velocity_reward_contract']['yaw_velocity']['scale'];cases.append(saved)
        for value in (0,-.1,float('nan'),float('inf')):
            saved=metadata();saved['velocity_reward_contract']['yaw_velocity']['squared_error_denominator_rad2_s2']=value;cases.append(saved)
        for key,value in [('training_standing_fraction',1.1),('velocity_command_resampling_s',0),
                          ('training_velocity_limits',[.3,float('inf'),.4]),('feet_air_time_reward_scale',-1)]:
            saved=metadata();saved[key]=value;cases.append(saved)
        for saved in cases:
            cfg=configuration();before=vars(cfg).copy()
            with self.assertRaises(ValueError):restore_velocity_settings(cfg,saved)
            self.assertEqual(vars(cfg),before)

    def test_yaw_options_require_training_velocity_variant(self):
        validate_yaw_options('evaluate',False)
        validate_yaw_options('train',True,.2,4.)
        for mode,variant in [('evaluate',True),('teleop',True),('export',True),('train',False)]:
            with self.assertRaises(ValueError):validate_yaw_options(mode,variant,.2,4.)

    def test_yaw_options_reject_nonfinite_out_of_bounds_and_underflow(self):
        for value in (0,-.2,2.1,float('nan'),float('inf'),1e-300):
            with self.assertRaises(ValueError):validate_yaw_options('train',True,sigma=value)
        for value in (0,-2,20.1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):validate_yaw_options('train',True,weight=value)

    def test_saved_defaults_remain_unchanged_for_existing_v2_checkpoint(self):
        cfg=configuration();before=vars(cfg).copy();saved=metadata()
        saved.update(training_velocity_limits=list(cfg.command_max),training_standing_fraction=cfg.standing_fraction,
            velocity_command_resampling_s=cfg.command_resampling_s,feet_air_time_reward_scale=cfg.feet_air_time_reward_scale,
            both_feet_air_penalty_scale=cfg.both_feet_air_penalty_scale)
        saved['velocity_reward_contract']['linear_velocity']=dict(scale=4.,squared_error_denominator_m2_s2=.04)
        saved['velocity_reward_contract']['yaw_velocity']=dict(scale=2.,squared_error_denominator_rad2_s2=.1)
        restore_velocity_settings(cfg,saved)
        self.assertEqual(vars(cfg),before)

    def test_saved_standing_yaw_restores_and_legacy_resets_to_base(self):
        cfg=configuration();saved=metadata();before=copy.deepcopy(saved)
        saved['velocity_reward_contract']['standing_yaw_velocity']={
            'scale':3.,'squared_error_denominator_rad2_s2':.05}
        restore_velocity_settings(cfg,saved)
        self.assertEqual((cfg.standing_yaw_velocity_reward_scale,cfg.standing_yaw_velocity_error_variance),(3.,.05))
        self.assertEqual(cfg.moving_yaw_velocity_reward_scale,4.)
        for legacy in (before,{'environment_variant':'g1_commanded_velocity_v1'}):
            cfg.standing_yaw_velocity_reward_scale=3.;cfg.standing_yaw_velocity_error_variance=.05
            restore_velocity_settings(cfg,legacy)
            self.assertEqual((cfg.standing_yaw_velocity_reward_scale,cfg.standing_yaw_velocity_error_variance),(1.,.25))

    def test_invalid_standing_metadata_is_transactional(self):
        for bad in (None,{}, {'scale':3.}, {'squared_error_denominator_rad2_s2':.05}):
            saved=metadata();saved['velocity_reward_contract']['standing_yaw_velocity']=bad
            cfg=configuration();before=vars(cfg).copy()
            with self.assertRaises(ValueError):restore_velocity_settings(cfg,saved)
            self.assertEqual(vars(cfg),before)
        for key in ('scale','squared_error_denominator_rad2_s2'):
            for bad in (0,-1,True,float('nan'),float('inf')):
                saved=metadata();term={'scale':3.,'squared_error_denominator_rad2_s2':.05};term[key]=bad
                saved['velocity_reward_contract']['standing_yaw_velocity']=term
                cfg=configuration();before=vars(cfg).copy()
                with self.assertRaises(ValueError):restore_velocity_settings(cfg,saved)
                self.assertEqual(vars(cfg),before)

    def test_standing_overrides_are_finite_positive_training_only(self):
        validate_standing_yaw_options('evaluate',False)
        validate_standing_yaw_options('train',True,sigma=.2236068,weight=3.)
        for mode,variant in [('evaluate',True),('teleop',True),('export',True),('train',False)]:
            with self.assertRaises(ValueError):validate_standing_yaw_options(mode,variant,.2236068,3.)
        for value in (True,0,-.2,2.1,float('nan'),float('inf'),1e-300):
            with self.assertRaises(ValueError):validate_standing_yaw_options('train',True,sigma=value)
        for value in (True,0,-2,20.1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):validate_standing_yaw_options('train',True,weight=value)

    def test_linear_overrides_are_finite_positive_training_only(self):
        validate_linear_options('evaluate',False)
        validate_linear_options('train',True,sigma=.1,weight=6.)
        validate_linear_options('train',True,sigma=2.,weight=20.)
        for mode,variant in [('evaluate',True),('teleop',True),('export',True),('train',False)]:
            with self.assertRaises(ValueError):validate_linear_options(mode,variant,.1,6.)
        for value in (True,0,-.2,2.1,float('nan'),float('inf'),1e-300):
            with self.assertRaises(ValueError):validate_linear_options('train',True,sigma=value)
        for value in (True,0,-2,20.1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):validate_linear_options('train',True,weight=value)

    def test_linear_override_contract_restores_with_other_reward_terms_unchanged(self):
        saved=metadata();cfg=configuration()
        saved['velocity_reward_contract']['linear_velocity']={
            'scale':6.,'squared_error_denominator_m2_s2':.1**2}
        restore_velocity_settings(cfg,saved)
        self.assertEqual(cfg.moving_linear_velocity_reward_scale,6.)
        self.assertEqual(cfg.moving_linear_velocity_error_variance,.1**2)
        self.assertEqual(cfg.moving_yaw_velocity_reward_scale,4.)
        self.assertEqual(cfg.moving_yaw_velocity_error_variance,.04)
        self.assertEqual(cfg.standing_yaw_velocity_reward_scale,1.)
        self.assertEqual(cfg.feet_air_time_reward_scale,.7)


if __name__=='__main__':unittest.main()
