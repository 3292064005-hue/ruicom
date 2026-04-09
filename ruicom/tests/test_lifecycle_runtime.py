import unittest

from ruikang_recon_baseline.lifecycle_runtime import ManagedRuntimeState


class ManagedRuntimeStateTest(unittest.TestCase):
    def test_unmanaged_nodes_start_active_and_reject_commands(self):
        runtime = ManagedRuntimeState(lifecycle_managed=False)
        self.assertEqual(runtime.state, 'ACTIVE')
        result = runtime.apply('pause')
        self.assertFalse(result.accepted)
        self.assertEqual(result.message, 'lifecycle_unmanaged')
        self.assertEqual(runtime.state, 'ACTIVE')

    def test_managed_lifecycle_walks_expected_states(self):
        runtime = ManagedRuntimeState(lifecycle_managed=True)
        self.assertEqual(runtime.state, 'IDLE')
        self.assertFalse(runtime.processing_allowed)

        result = runtime.apply('activate')
        self.assertTrue(result.accepted)
        self.assertEqual(runtime.state, 'ACTIVE')
        self.assertTrue(runtime.processing_allowed)

        result = runtime.apply('pause')
        self.assertEqual(runtime.state, 'PAUSED')
        self.assertFalse(runtime.processing_allowed)
        self.assertEqual(result.message, 'runtime_paused')

        result = runtime.apply('reset')
        self.assertEqual(runtime.state, 'IDLE')
        self.assertEqual(result.message, 'runtime_reset')

        result = runtime.apply('shutdown')
        self.assertEqual(runtime.state, 'SHUTDOWN')
        self.assertEqual(result.message, 'runtime_shutdown')

    def test_unknown_command_is_rejected(self):
        runtime = ManagedRuntimeState(lifecycle_managed=True)
        result = runtime.apply('bad')
        self.assertFalse(result.accepted)
        self.assertEqual(result.message, 'unknown_control_command')
