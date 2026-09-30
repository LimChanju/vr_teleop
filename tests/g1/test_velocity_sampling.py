"""Pure-axis curriculum, compatibility and saved-setting checks on CPU only."""
import copy
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

from scripts.g1.run import restore_velocity_settings, validate_velocity_command_options
from tests.g1.test_velocity_rewards import load_velocity_module, torch
from tests.g1.test_velocity_settings import configuration, metadata


@unittest.skipIf(torch is None, 'CPU PyTorch is required')
class VelocitySamplingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module=load_velocity_module()

    def fixture(self, count=100):
        env=self.module.G1VelocityEnv.__new__(self.module.G1VelocityEnv)
        env.cfg=self.module.G1VelocityEnvCfg()
        env.num_envs=count;env.device='cpu';env.step_dt=.02;env.common_step_counter=17
        env.external_mode=False;env.command_velocity=torch.zeros(count,3)
        return env

    def test_seeded_default_is_bitwise_identical_to_original_mixed_sampler(self):
        for count in (1,17,4096):
            for seed in (17,293):
                env=self.fixture(count)
                torch.manual_seed(seed)
                old=(2.*torch.rand(count,3)-1.)*torch.tensor((.3,.15,.4))
                standing=torch.rand(count)<.3;old[standing]=0.
                expected_next=torch.rand(8)
                torch.manual_seed(seed);env._sample_velocity(torch.arange(count))
                self.assertTrue(torch.equal(env.command_velocity,old))
                self.assertTrue(torch.equal(torch.rand(8),expected_next))
                self.assertTrue((env._velocity_last_sample_step==17).all())

    def test_pure_axis_proportions_bounds_and_independent_signs(self):
        env=self.fixture(100000);env.cfg.command_sampling='pure_axis'
        torch.manual_seed(1934);env._sample_velocity(torch.arange(env.num_envs))
        commands=env.command_velocity
        nonzero=commands!=0;counts=nonzero.sum(-1)
        self.assertTrue(((counts==0)|(counts==1)).all())
        self.assertAlmostEqual(float((counts==0).float().mean()),.3,delta=.005)
        moving=counts==1
        for axis,probability,lo,hi in zip(range(3),(5/14,5/14,4/14),(.1,.08,.15),(.3,.15,.4)):
            selected=nonzero[:,axis]
            self.assertAlmostEqual(float(selected.sum()/moving.sum()),probability,delta=.005)
            values=commands[selected,axis]
            self.assertTrue((values.abs()>=lo).all());self.assertTrue((values.abs()<=hi).all())
            self.assertAlmostEqual(float((values>0).float().mean()),.5,delta=.01)

    def test_only_requested_ids_change_and_external_mode_is_untouched(self):
        env=self.fixture(8);env.cfg.command_sampling='pure_axis';env.command_velocity.fill_(.012)
        env._sample_velocity(torch.tensor([1,5]))
        self.assertTrue(torch.equal(env.command_velocity[[0,2,3,4,6,7]],torch.full((6,3),.012)))
        before=env.command_velocity.clone();env.external_mode=True
        env._sample_velocity(torch.arange(8));self.assertTrue(torch.equal(env.command_velocity,before))

    def test_standing_extremes_and_minimum_equals_maximum(self):
        env=self.fixture(128);env.cfg.command_sampling='pure_axis'
        env.cfg.standing_fraction=1.;env._sample_velocity(torch.arange(128))
        self.assertFalse(env.command_velocity.any())
        env.cfg.standing_fraction=0.;env.cfg.command_max=(.1,.08,.15)
        env._sample_velocity(torch.arange(128))
        self.assertTrue(((env.command_velocity!=0).sum(-1)==1).all())
        torch.testing.assert_close(env.command_velocity.abs().max(0).values,torch.tensor([.1,.08,.15]))

    def test_generated_metadata_restores_mode_threshold_and_all_old_terms(self):
        env=self.fixture();env.cfg.command_sampling='pure_axis';env.cfg.moving_xy_threshold=.04
        saved=env.policy_metadata();cfg=configuration()
        restore_velocity_settings(cfg,saved)
        self.assertEqual(cfg.command_sampling,'pure_axis');self.assertEqual(cfg.moving_xy_threshold,.04)
        self.assertEqual(cfg.command_max,(.3,.15,.4));self.assertEqual(cfg.standing_fraction,.3)
        self.assertEqual(cfg.moving_linear_velocity_reward_scale,4.)
        self.assertEqual(cfg.moving_yaw_velocity_reward_scale,2.)
        self.assertEqual(cfg.feet_air_time_reward_scale,.5)
        self.assertEqual(cfg.both_feet_air_penalty_scale,.15)
        # Existing v2 metadata has no structured sampling/mask fields.
        legacy=metadata();restore_velocity_settings(cfg,legacy)
        self.assertEqual(cfg.command_sampling,'mixed');self.assertEqual(cfg.moving_xy_threshold,.08)
        self.assertEqual(cfg.moving_yaw_velocity_reward_scale,4.)

    def test_malformed_metadata_rejected_without_partial_mutation(self):
        env=self.fixture();env.cfg.command_sampling='pure_axis';saved=env.policy_metadata()
        mutations=[('mode','invalid'),('mode',None),('standing_fraction',.2),
                   ('pure_axis_minimum_absolute_commands',[0.,0.,0.]),
                   ('pure_axis_probabilities_given_moving',[.2,.4,.4])]
        for key,value in mutations:
            bad=copy.deepcopy(saved);bad['velocity_command_sampling'][key]=value
            cfg=configuration();before=vars(cfg).copy()
            with self.assertRaises(ValueError):restore_velocity_settings(cfg,bad)
            self.assertEqual(vars(cfg),before)
        for field,value in [('xy_norm_mps',float('nan')),('xy_norm_mps',0.),('xy_norm_mps',.301),
                            ('yaw_abs_radps',.2),('comparison','>=')]:
            bad=copy.deepcopy(saved);bad['velocity_reward_contract']['moving_thresholds'][field]=value
            with self.assertRaises(ValueError):restore_velocity_settings(configuration(),bad)
        bad=copy.deepcopy(saved);bad['training_velocity_limits']=[.3,.07,.4]
        with self.assertRaises(ValueError):restore_velocity_settings(configuration(),bad)

    def test_overrides_are_train_only_finite_bounded_and_require_valid_pure_limits(self):
        validate_velocity_command_options('evaluate',False)
        for mode,variant in [('evaluate',True),('teleop',True),('export',True),('train',False)]:
            with self.assertRaises(ValueError):validate_velocity_command_options(mode,variant,'pure_axis',.04)
        for value in (0,-.01,.301,float('inf'),float('nan'),True):
            with self.assertRaises(ValueError):validate_velocity_command_options('train',True,xy_threshold=value)
        for limits in ([.09,.15,.4],[.3,.079,.4],[.3,.15,.149],[.3,float('nan'),.4]):
            with self.assertRaises(ValueError):validate_velocity_command_options('train',True,'pure_axis',.04,limits)
        validate_velocity_command_options('train',True,'pure_axis',.3,[.1,.08,.15])

    def test_actual_runner_applies_explicit_overrides_after_saved_restoration(self):
        # Execute the runner's configuration block, with no AppLauncher import.
        source=Path(__file__).resolve().parents[2]/'scripts/g1/run.py'
        tree=ast.parse(source.read_text())
        block=next(node for node in ast.walk(tree) if isinstance(node,ast.If)
                   and ast.unparse(node.test)=='args.velocity_training'
                   and any(isinstance(child,ast.Call) and isinstance(child.func,ast.Name)
                           and child.func.id=='restore_velocity_settings' for child in ast.walk(node)))
        cfg=configuration()
        args=SimpleNamespace(velocity_training=True,command_sampling='pure_axis',moving_xy_threshold=.04,
                             yaw_tracking_sigma=None,yaw_reward_weight=None,velocity_evaluation=False)
        namespace={'cfg':cfg,'args':args,'checkpoint_meta':metadata(),
                   'restore_velocity_settings':restore_velocity_settings,
                   'validate_velocity_command_options':validate_velocity_command_options}
        exec(compile(ast.Module(body=[block],type_ignores=[]),str(source),'exec'),namespace)
        self.assertEqual(cfg.command_sampling,'pure_axis')
        self.assertEqual(cfg.moving_xy_threshold,.04)
        self.assertEqual(cfg.moving_yaw_velocity_reward_scale,4.)
        self.assertEqual(cfg.command_max,(.2,.1,.3))


if __name__=='__main__':unittest.main()
