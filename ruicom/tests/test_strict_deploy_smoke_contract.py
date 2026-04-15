import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class StrictDeploySmokeContractTest(unittest.TestCase):
    def test_reference_runtime_smoke_uses_strict_deploy_profiles(self):
        text = (ROOT / 'tests/reference_field_runtime_smoke.test').read_text(encoding='utf-8')
        self.assertIn('config/profiles/reference_deploy/vision.yaml', text)
        self.assertIn('config/profiles/reference_deploy/recorder.yaml', text)
        self.assertIn('config/profiles/reference_deploy/mission.yaml', text)
        self.assertIn('tests/fixtures/models/fixed_enemy_detector.onnx', text)
        self.assertIn('config/profiles/reference_strict_smoke/vendor_actuator_device.yaml', text)
        profile = (ROOT / 'config/profiles/reference_strict_smoke/vendor_actuator_device.yaml').read_text(encoding='utf-8')
        self.assertIn('device_mode: external_ack', profile)
        self.assertNotIn('config/profiles/reference_smoke/vision.yaml', text)
        self.assertIn('managed_vendor_actuator_ack_device.py', text)
        self.assertIn('managed_scene_image_publisher.py', text)
        self.assertIn('managed_vendor_chassis_device.py', text)
        self.assertNotIn('fake_base_feedback_publisher.py', text)
        self.assertNotIn('fake_twist_publisher.py', text)
        self.assertNotIn('fake_move_base_action_server.py', text)
        self.assertNotIn('fake_vendor_actuator_ack_device.py', text)
        self.assertNotIn('fake_vendor_chassis_device.py', text)
        self.assertNotIn('enable_synthetic" value="true', text)

    def test_field_runtime_smoke_uses_strict_deploy_profiles(self):
        text = (ROOT / 'tests/field_runtime_smoke.test').read_text(encoding='utf-8')
        self.assertIn('config/profiles/field_deploy/vision.yaml', text)
        self.assertIn('config/profiles/field_deploy/recorder.yaml', text)
        self.assertIn('config/profiles/field_deploy/mission.yaml', text)
        self.assertIn('tests/fixtures/models/fixed_enemy_detector.onnx', text)
        self.assertIn('config/profiles/field_strict_smoke/vendor_actuator_device.yaml', text)
        profile = (ROOT / 'config/profiles/field_strict_smoke/vendor_actuator_device.yaml').read_text(encoding='utf-8')
        self.assertIn('device_mode: external_ack', profile)
        self.assertNotIn('config/profiles/field_smoke/vision.yaml', text)
        self.assertIn('managed_vendor_actuator_ack_device.py', text)
        self.assertIn('managed_scene_image_publisher.py', text)
        self.assertIn('managed_vendor_chassis_device.py', text)
        self.assertNotIn('fake_base_feedback_publisher.py', text)
        self.assertNotIn('fake_twist_publisher.py', text)
        self.assertNotIn('fake_move_base_action_server.py', text)
        self.assertNotIn('fake_vendor_actuator_ack_device.py', text)
        self.assertNotIn('fake_vendor_chassis_device.py', text)
        self.assertNotIn('enable_synthetic" value="true', text)

if __name__ == '__main__':
    unittest.main()
