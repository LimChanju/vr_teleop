"""No-GPU checks for finite completion and failure receipt preservation."""
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import unittest
from unittest.mock import patch

from scripts.g1 import validate_runtime as harness

try:
    import numpy as np
except ImportError:
    np = None


@unittest.skipIf(np is None, 'NumPy is needed in this selected test interpreter')
class CompletionTests(unittest.TestCase):
    def make_run(self, directory, *, steps=2000, rows=None, status='teleop_stopped'):
        directory.mkdir()
        (directory/'result.json').write_text(json.dumps({'mode':'teleop','status':status,'steps':steps}))
        count=steps if rows is None else rows
        np.savez_compressed(directory/'trace.npz',wall_time=np.arange(count,dtype=float)/50,
                            q=np.zeros((count,29)),target=np.zeros((count,3,3)),
                            dt=np.asarray(.02))
        return directory

    def validate(self, folder, expected=2000):
        return harness.validate_completion(folder,expected,sys.executable,dict(os.environ))

    def test_full_finite_run_checks_trace_with_selected_interpreter(self):
        with tempfile.TemporaryDirectory() as temp:
            output=self.make_run(Path(temp)/'valid')
            with patch.object(harness.subprocess,'run',wraps=harness.subprocess.run) as invoked:
                result=self.validate(output)
            self.assertEqual(result['completed_steps'],2000)
            self.assertEqual(result['trace_rows'],2000)
            self.assertEqual(invoked.call_args.args[0][:3],[sys.executable,'-I','-c'])

    def test_early_successful_exit_after_producer_phases_cannot_pass(self):
        # Reproduces the observed acceptance gap: 36s contains all state
        # phases but does not complete the requested 40s/2000-step rollout.
        with tempfile.TemporaryDirectory() as temp:
            output=self.make_run(Path(temp)/'truncated',steps=1800)
            with self.assertRaisesRegex(RuntimeError,'1800 steps; expected 2000'):
                self.validate(output)

    def test_result_claiming_full_steps_cannot_hide_short_trace(self):
        with tempfile.TemporaryDirectory() as temp:
            output=self.make_run(Path(temp)/'short_trace',rows=1800)
            with self.assertRaisesRegex(RuntimeError,'trace completion check failed'):
                self.validate(output)

    def test_any_non_dt_trace_field_must_have_all_rows(self):
        with tempfile.TemporaryDirectory() as temp:
            output=self.make_run(Path(temp)/'short_optional')
            np.savez_compressed(output/'trace.npz',wall_time=np.arange(2000),
                                root_position_w=np.zeros((1999,3)),dt=np.asarray(.02))
            with self.assertRaisesRegex(RuntimeError,'root_position_w'):
                self.validate(output)

    def test_interrupted_or_failed_simulator_result_is_not_completion(self):
        with tempfile.TemporaryDirectory() as temp:
            for status in ('interrupted','failed','running'):
                output=self.make_run(Path(temp)/status,status=status)
                with self.assertRaisesRegex(RuntimeError,'did not finish teleop'):
                    self.validate(output)


class ReceiptTests(unittest.TestCase):
    def test_early_subprocess_exit_writes_failure_receipt_without_existing_output(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);checkpoint=root/'model.pt';checkpoint.write_bytes(b'not loaded by fake process')
            (root/'nominal_targets.json').write_text('[]')
            output=root/'new_run'
            class Exited:
                returncode=7
                def poll(self):return self.returncode
            args=['validate_runtime.py','--checkpoint',str(checkpoint),'--output',str(output),
                  '--python',sys.executable]
            old_term=signal.getsignal(signal.SIGTERM)
            try:
                with patch.object(sys,'argv',args),patch.object(harness.subprocess,'Popen',return_value=Exited()):
                    with self.assertRaisesRegex(RuntimeError,'before readiness: 7'):
                        harness.main()
            finally:
                signal.signal(signal.SIGTERM,old_term)
            receipt=json.loads((output/'scenario_analysis_harness.json').read_text())
            self.assertEqual(receipt['status'],'execution_failed')
            self.assertIn('before readiness: 7',receipt['error'])
            self.assertFalse(receipt['physical_quest_verified'])
            self.assertTrue(output.with_name('new_run.sim.log').is_file())

    def test_receipt_never_overwrites_previous_receipt_or_physics_output(self):
        with tempfile.TemporaryDirectory() as temp:
            output=Path(temp)/'run';output.mkdir()
            physics=output/'result.json';physics.write_bytes(b'original physics output')
            harness.save_receipt(output,{'status':'execution_failed'})
            target=output/'scenario_analysis_harness.json';before=target.read_bytes()
            with self.assertRaises(FileExistsError):harness.save_receipt(output,{'status':'passed'})
            self.assertEqual(target.read_bytes(),before)
            self.assertEqual(physics.read_bytes(),b'original physics output')


if __name__=='__main__':unittest.main()
