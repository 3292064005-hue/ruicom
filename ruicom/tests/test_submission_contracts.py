import json
import tempfile
import unittest
from pathlib import Path

from ruikang_recon_baseline.recorder_core import (
    build_initial_operator_audit_state,
    record_runtime_evidence_operator_audit,
    summarize_operator_audit,
)
from ruikang_recon_baseline.submission_adapters import (
    load_and_validate_submission_contract,
    submit_official_report,
)
from ruikang_recon_baseline.system_manager_core import is_operator_control_mode


class SubmissionContractTests(unittest.TestCase):
    def test_file_drop_submission_adapter_writes_report_and_receipt(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            contract_path = Path(tmpdir) / 'contract.json'
            endpoint_path = Path(tmpdir) / 'judge' / 'official_report.json'
            report_path = Path(tmpdir) / 'local_report.json'
            contract_path.write_text(json.dumps({
                'adapter_type': 'file_drop',
                'endpoint_path': str(endpoint_path),
                'timeout_sec': 5.0,
            }), encoding='utf-8')
            contract = load_and_validate_submission_contract(str(contract_path))
            report = {'report_schema': 'unit', 'value': 1}
            receipt = submit_official_report(contract, report, report_path=str(report_path), submission_id='submission-1')
            self.assertEqual(receipt['adapter_type'], 'file_drop')
            self.assertEqual(receipt['status'], 'submitted')
            self.assertEqual(json.loads(endpoint_path.read_text(encoding='utf-8'))['value'], 1)
            self.assertEqual(receipt['submission_id'], 'submission-1')


class OperatorInterventionAuditTests(unittest.TestCase):
    def test_runtime_evidence_audit_tracks_manual_commands_and_takeovers(self):
        audit = build_initial_operator_audit_state()
        audit = record_runtime_evidence_operator_audit(audit, {
            'stamp': 1.0,
            'event_type': 'control_mode_changed',
            'current_control_mode': 'HANDLE',
            'details': {'previous_control_mode': 'COMMANDER'},
        })
        audit = record_runtime_evidence_operator_audit(audit, {
            'stamp': 2.0,
            'event_type': 'manual_command',
            'details': {'command': 'pause', 'accepted': True, 'resulting_state': 'PAUSED'},
        })
        audit = record_runtime_evidence_operator_audit(audit, {
            'stamp': 3.0,
            'event_type': 'estop_changed',
            'estop_active': True,
        })
        summary = summarize_operator_audit(audit)
        self.assertEqual(summary['manual_command_count'], 1)
        self.assertEqual(summary['manual_takeover_count'], 1)
        self.assertTrue(summary['operator_control_active'])
        self.assertTrue(summary['estop_active'])
        self.assertEqual(summary['estop_event_count'], 1)
        self.assertTrue(is_operator_control_mode('HANDLE'))
        self.assertFalse(is_operator_control_mode('COMMANDER'))


if __name__ == '__main__':
    unittest.main()
