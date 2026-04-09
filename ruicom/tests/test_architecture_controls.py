import unittest

from ruikang_recon_baseline.common import (
    ConfigurationError,
    SafetyStatus,
    VelocityCommand,
    class_schema_hash,
    coerce_class_count_mapping,
    dynamic_zone_result_to_legacy_payload,
    ensure_legacy_class_schema,
    flatten_count_matrix,
    resolve_output_root,
    sanitize_ros_namespace,
    unflatten_count_matrix,
    validate_dynamic_class_names,
    validate_named_region_contract,
    validate_profile_role,
    validate_route_frame_region_contract,
    zone_capture_payload_to_dynamic_payload,
)
from ruikang_recon_baseline.recorder_core import (
    build_summary_snapshot_payload,
    build_summary_snapshot_v2_payload,
    canonical_payload_hash,
    mission_state_payloads_match,
    projected_zone_payloads_match,
    should_accept_lane,
    should_accept_lane_for_source,
    snapshot_flush_allowed,
    validate_frame_region_counts_payload,
    validate_frame_region_counts_typed_payload,
    validate_health_payload,
    validate_legacy_zone_capture_schema,
    validate_mission_state_payload,
    validate_summary_class_names,
    validate_zone_capture_dynamic_payload,
    validate_zone_capture_payload,
    zone_capture_payloads_match,
    summarize_task_result_metrics,
)
from ruikang_recon_baseline.safety_core import classify_safety_health


class LegacyClassSchemaTest(unittest.TestCase):
    def test_legacy_schema_is_accepted(self):
        normalized = ensure_legacy_class_schema(['friendly', 'enemy', 'hostage'], owner='unit')
        self.assertEqual(normalized, ('friendly', 'enemy', 'hostage'))

    def test_non_legacy_schema_is_rejected(self):
        with self.assertRaises(ConfigurationError):
            ensure_legacy_class_schema(['friendly', 'enemy'], owner='unit')

    def test_summary_class_names_accepts_dynamic_schema(self):
        normalized = validate_summary_class_names(['friendly', 'enemy'])
        self.assertEqual(normalized, ('friendly', 'enemy'))


class DynamicSchemaUtilityTest(unittest.TestCase):
    def test_dynamic_class_names_allow_extended_schema(self):
        normalized = validate_dynamic_class_names(['friendly', 'enemy', 'hostage', 'unknown'], owner='unit')
        self.assertEqual(normalized, ('friendly', 'enemy', 'hostage', 'unknown'))

    def test_flatten_roundtrip_preserves_counts(self):
        region_names, counts_flat = flatten_count_matrix({
            'zone_a': {'friendly': 2, 'enemy': 1, 'hostage': 0},
            'zone_b': {'friendly': 0, 'enemy': 3, 'hostage': 1},
        }, ('friendly', 'enemy', 'hostage'))
        restored = unflatten_count_matrix(region_names, ('friendly', 'enemy', 'hostage'), counts_flat)
        self.assertEqual(restored['zone_a']['friendly'], 2)
        self.assertEqual(restored['zone_b']['enemy'], 3)

    def test_count_mapping_coerces_missing_keys_to_zero(self):
        counts = coerce_class_count_mapping({'friendly': 2}, ('friendly', 'enemy', 'hostage'))
        self.assertEqual(counts, {'friendly': 2, 'enemy': 0, 'hostage': 0})

    def test_output_root_namespace_derivation(self):
        self.assertEqual(sanitize_ros_namespace('/robot1/team_a/'), 'robot1/team_a')
        self.assertTrue(resolve_output_root('/tmp/recon', '/robot1', True).endswith('/tmp/recon/robot1'))
        self.assertEqual(resolve_output_root('/tmp/recon', '/', True), '/tmp/recon')

    def test_legacy_zone_payload_is_normalized_into_dynamic_shape(self):
        payload = zone_capture_payload_to_dynamic_payload({
            'zone_name': 'zone_a',
            'status': 'ok',
            'friendly': 2,
            'enemy': 1,
            'hostage': 0,
        }, ('friendly', 'enemy', 'hostage', 'unknown'))
        self.assertEqual(payload['class_names'], ['friendly', 'enemy', 'hostage', 'unknown'])
        self.assertEqual(payload['class_counts'], [2, 1, 0, 0])
        self.assertEqual(payload['counts']['unknown'], 0)

    def test_dynamic_zone_payload_preserves_route_id_and_schema_hash(self):
        payload = zone_capture_payload_to_dynamic_payload({
            'zone_name': 'zone_a',
            'route_id': 'zone_a__2',
            'status': 'ok',
            'friendly': 2,
            'enemy': 1,
            'hostage': 0,
        }, ('friendly', 'enemy', 'hostage', 'unknown'))
        self.assertEqual(payload['route_id'], 'zone_a__2')
        self.assertEqual(payload['class_schema_hash'], class_schema_hash(payload['class_names']))

    def test_dynamic_zone_payload_projects_back_to_legacy_shape(self):
        payload = dynamic_zone_result_to_legacy_payload({
            'zone_name': 'zone_a',
            'status': 'ok',
            'class_names': ['friendly', 'enemy', 'hostage', 'unknown'],
            'class_counts': [2, 1, 0, 4],
        })
        self.assertEqual(payload['friendly'], 2)
        self.assertEqual(payload['enemy'], 1)
        self.assertEqual(payload['hostage'], 0)
        self.assertEqual(payload['counts']['unknown'], 4)


class RecorderLanePolicyTest(unittest.TestCase):
    def test_auto_prefers_typed_once_seen(self):
        typed_seen = {'mission_state': False}
        self.assertTrue(should_accept_lane('auto', 'mission_state', typed_seen, 'json'))
        self.assertTrue(should_accept_lane('auto', 'mission_state', typed_seen, 'typed'))
        typed_seen['mission_state'] = True
        self.assertFalse(should_accept_lane('auto', 'mission_state', typed_seen, 'json'))
        self.assertTrue(should_accept_lane('auto', 'mission_state', typed_seen, 'typed'))

    def test_auto_prefers_typed_per_source(self):
        self.assertTrue(should_accept_lane_for_source('auto', 'json', False))
        self.assertTrue(should_accept_lane_for_source('auto', 'typed', False))
        self.assertFalse(should_accept_lane_for_source('auto', 'json', True))
        self.assertTrue(should_accept_lane_for_source('auto', 'typed', True))

    def test_snapshot_flush_policy(self):
        self.assertFalse(snapshot_flush_allowed(False, False, False, False))
        self.assertFalse(snapshot_flush_allowed(True, False, True, False))
        self.assertTrue(snapshot_flush_allowed(True, True, True, False))
        self.assertTrue(snapshot_flush_allowed(False, False, True, True))


class RecorderPayloadMatchTest(unittest.TestCase):
    def test_mission_state_payload_match_uses_fixed_fields_and_details_hash(self):
        authoritative = {
            'state': 'ACTIVE',
            'event': 'tick',
            'route_index': 0,
            'route_total': 1,
            'current_zone': 'zone_a',
            'schema_version': '2.2.0',
            'details': {'retry_count': 1, 'duration_sec': 0.5},
        }
        candidate = {
            'state': 'ACTIVE',
            'event': 'tick',
            'route_index': 0,
            'route_total': 1,
            'current_zone': 'zone_a',
            'schema_version': '2.2.0',
            'details': {'duration_sec': 0.5, 'retry_count': 1},
        }
        self.assertTrue(mission_state_payloads_match(authoritative, candidate))
        candidate['details']['retry_count'] = 2
        self.assertFalse(mission_state_payloads_match(authoritative, candidate))

    def test_zone_capture_payload_match_checks_dynamic_counts(self):
        authoritative = {
            'zone_name': 'zone_a',
            'status': 'ok',
            'capture_started_at': 1.0,
            'capture_finished_at': 2.0,
            'frame_count': 3,
            'frame_region': '',
            'failure_reason': '',
            'schema_version': '2.2.0',
            'class_names': ['friendly', 'enemy', 'hostage'],
            'class_counts': [1, 2, 0],
        }
        candidate = dict(authoritative)
        self.assertTrue(zone_capture_payloads_match(authoritative, candidate))
        candidate['class_counts'] = [1, 0, 0]
        self.assertFalse(zone_capture_payloads_match(authoritative, candidate))

    def test_projected_zone_payload_match_checks_legacy_projection(self):
        authoritative_dynamic = {
            'zone_name': 'zone_a',
            'status': 'ok',
            'class_names': ['friendly', 'enemy', 'hostage', 'unknown'],
            'class_counts': [1, 2, 0, 9],
        }
        matching_legacy = dynamic_zone_result_to_legacy_payload(authoritative_dynamic)
        mismatching_legacy = dict(matching_legacy, enemy=1)
        self.assertTrue(projected_zone_payloads_match(authoritative_dynamic, matching_legacy))
        self.assertFalse(projected_zone_payloads_match(authoritative_dynamic, mismatching_legacy))

    def test_canonical_payload_hash_is_order_invariant(self):
        left = canonical_payload_hash({'b': 1, 'a': {'x': 2, 'y': 3}})
        right = canonical_payload_hash({'a': {'y': 3, 'x': 2}, 'b': 1})
        self.assertEqual(left, right)


class SafetyHealthClassificationTest(unittest.TestCase):
    def test_idle_command_stale_is_info_by_default(self):
        status = SafetyStatus(
            stamp=1.0,
            mode='AUTO',
            selected_source='',
            estop_active=False,
            estop_fresh=True,
            command_fresh=False,
            reason='command_stale',
            output=VelocityCommand(),
            observed_source_count=0,
            fresh_source_count=0,
        )
        self.assertEqual(classify_safety_health(status, warn_on_idle_command_stale=False), ('info', 'command_idle'))

    def test_estop_is_warn(self):
        status = SafetyStatus(
            stamp=1.0,
            mode='AUTO',
            selected_source='',
            estop_active=True,
            estop_fresh=True,
            command_fresh=False,
            reason='estop_active',
            output=VelocityCommand(),
            observed_source_count=1,
            fresh_source_count=0,
        )
        self.assertEqual(classify_safety_health(status, warn_on_idle_command_stale=False), ('warn', 'estop_active'))


class RecorderPayloadValidationTest(unittest.TestCase):
    def test_mission_state_payload_requires_state(self):
        with self.assertRaises(ConfigurationError):
            validate_mission_state_payload({'event': 'x', 'route_index': 0, 'route_total': 1, 'details': {}}, topic_name='unit')

    def test_mission_state_payload_allows_dispatch_pending_without_current_route_id(self):
        payload = validate_mission_state_payload({
            'state': 'DISPATCH_PENDING',
            'event': 'mission_started',
            'route_index': 0,
            'route_total': 2,
            'current_zone': '',
            'current_route_id': '',
            'class_names': ['friendly', 'enemy', 'hostage'],
            'class_schema_hash': class_schema_hash(['friendly', 'enemy', 'hostage']),
            'details': {},
        }, topic_name='unit')
        self.assertEqual(payload['current_route_id'], '')

    def test_mission_state_payload_requires_current_route_id_once_zone_is_active(self):
        with self.assertRaises(ConfigurationError):
            validate_mission_state_payload({
                'state': 'DISPATCHED',
                'event': 'goal_dispatched',
                'route_index': 0,
                'route_total': 2,
                'current_zone': 'zone_a',
                'current_route_id': '',
                'class_names': ['friendly', 'enemy', 'hostage'],
                'class_schema_hash': class_schema_hash(['friendly', 'enemy', 'hostage']),
                'details': {},
            }, topic_name='unit')

    def test_zone_capture_payload_requires_zone_name(self):
        with self.assertRaises(ConfigurationError):
            validate_zone_capture_payload({'status': 'ok'}, topic_name='unit')

    def test_legacy_zone_capture_schema_requires_hash(self):
        with self.assertRaises(ConfigurationError):
            validate_legacy_zone_capture_schema({
                'zone_name': 'zone_a',
                'status': 'ok',
                'friendly': 1,
                'enemy': 0,
                'hostage': 0,
            }, ['friendly', 'enemy', 'hostage'], topic_name='unit')

    def test_legacy_zone_capture_schema_rejects_wrong_hash(self):
        with self.assertRaises(ConfigurationError):
            validate_legacy_zone_capture_schema({
                'zone_name': 'zone_a',
                'status': 'ok',
                'friendly': 1,
                'enemy': 0,
                'hostage': 0,
                'class_schema_hash': class_schema_hash(['friendly', 'enemy']),
            }, ['friendly', 'enemy', 'hostage'], topic_name='unit')

    def test_zone_capture_dynamic_requires_counts(self):
        with self.assertRaises(ConfigurationError):
            validate_zone_capture_dynamic_payload({'zone_name': 'zone_a', 'status': 'ok', 'class_names': ['friendly']}, topic_name='unit')

    def test_zone_capture_dynamic_rejects_class_count_length_mismatch(self):
        with self.assertRaises(ConfigurationError):
            validate_zone_capture_dynamic_payload({
                'zone_name': 'zone_a',
                'status': 'ok',
                'class_names': ['friendly', 'enemy'],
                'class_counts': [1],
            }, topic_name='unit')

    def test_zone_capture_dynamic_rejects_disagreeing_count_representations(self):
        with self.assertRaises(ConfigurationError):
            validate_zone_capture_dynamic_payload({
                'zone_name': 'zone_a',
                'status': 'ok',
                'class_names': ['friendly', 'enemy'],
                'class_counts': [1, 2],
                'counts': {'friendly': 1, 'enemy': 99},
            }, topic_name='unit')

    def test_frame_region_counts_requires_mapping(self):
        with self.assertRaises(ConfigurationError):
            validate_frame_region_counts_payload({'frame_region_counts': []}, topic_name='unit')

    def test_frame_region_counts_rejects_unknown_nested_class(self):
        with self.assertRaises(ConfigurationError):
            validate_frame_region_counts_payload({
                'frame_region_counts': {'zone_a': {'friendly': 1, 'neutral': 2}},
                'class_names': ['friendly', 'enemy'],
                'class_schema_hash': class_schema_hash(['friendly', 'enemy']),
            }, topic_name='unit')

    def test_frame_region_counts_typed_requires_flat_array_alignment(self):
        with self.assertRaises(ConfigurationError):
            validate_frame_region_counts_typed_payload({
                'region_names': ['zone_a'],
                'class_names': ['friendly', 'enemy'],
                'counts_flat': [1],
            }, topic_name='unit')

    def test_health_payload_normalizes_unknown_node(self):
        payload = validate_health_payload({'status': 'warn', 'message': 'x', 'details': {}}, topic_name='unit')
        self.assertEqual(payload['node'], 'unknown')


class SummarySnapshotContractTest(unittest.TestCase):
    def test_legacy_summary_snapshot_preserves_top_level_fields_and_nested_mission_state(self):
        payload = build_summary_snapshot_payload(
            generated_at_wall=123.0,
            schema_version='2.2.0',
            mission_state={
                'state': 'FINISHED',
                'event': 'mission_finished',
                'route_index': 3,
                'route_total': 3,
                'current_zone': '',
                'details': {'duration_sec': 8.2},
            },
            current_zone='',
            authoritative_input='auto',
            typed_seen={'mission_state': True},
            zone_results={
                'zone_a': {'friendly': 1, 'enemy': 2, 'hostage': 0},
                'zone_b': {'friendly': 0, 'enemy': 1, 'hostage': 1},
            },
            latest_frame_region_counts={'zone_a': {'friendly': 1, 'enemy': 1, 'hostage': 0}},
            last_health={'mission_manager_node': {'status': 'ok'}},
            terminal_state_seen=True,
        )
        self.assertEqual(payload['final_state'], 'FINISHED')
        self.assertEqual(payload['mission_state']['state'], 'FINISHED')
        self.assertEqual(payload['totals'], {'friendly': 1, 'enemy': 3, 'hostage': 1})

    def test_dynamic_summary_snapshot_rectangularizes_counts(self):
        payload = build_summary_snapshot_v2_payload(
            generated_at_wall=123.0,
            schema_version='2.2.0',
            mission_state={
                'state': 'FINISHED',
                'event': 'mission_finished',
                'route_index': 2,
                'route_total': 2,
                'current_route_id': '',
                'class_names': ['friendly', 'enemy', 'hostage'],
                'class_schema_hash': class_schema_hash(['friendly', 'enemy', 'hostage']),
            },
            current_zone='',
            authoritative_input='typed',
            typed_seen={'zone_capture_dynamic': True},
            class_names=['friendly', 'enemy', 'hostage'],
            zone_results_dynamic={
                'zone_a__1': {'zone_name': 'zone_a', 'route_id': 'zone_a__1', 'counts': {'friendly': 2, 'enemy': 1}},
            },
            latest_frame_region_counts={},
            last_health={},
            terminal_state_seen=True,
        )
        self.assertEqual(payload['mission_state']['state'], 'FINISHED')
        self.assertEqual(payload['zone_results_dynamic']['zone_a__1']['class_counts'], [2, 1, 0])
        self.assertEqual(payload['zone_results_dynamic']['zone_a__1']['route_id'], 'zone_a__1')
        self.assertEqual(payload['class_schema_hash'], class_schema_hash(['friendly', 'enemy', 'hostage']))
        self.assertEqual(payload['zone_results_dynamic']['zone_a__1']['class_schema_hash'], class_schema_hash(['friendly', 'enemy', 'hostage']))
        self.assertEqual(payload['totals_by_class'], {'friendly': 2, 'enemy': 1, 'hostage': 0})


    def test_dynamic_summary_snapshot_exposes_task_metrics(self):
        payload = build_summary_snapshot_v2_payload(
            generated_at_wall=123.0,
            schema_version='2.2.0',
            mission_state={
                'state': 'FINISHED',
                'event': 'mission_finished',
                'route_index': 2,
                'route_total': 2,
                'current_route_id': '',
                'class_names': ['friendly', 'enemy', 'hostage'],
                'class_schema_hash': class_schema_hash(['friendly', 'enemy', 'hostage']),
            },
            current_zone='',
            authoritative_input='typed',
            typed_seen={'zone_capture_dynamic': True},
            class_names=['friendly', 'enemy', 'hostage'],
            zone_results_dynamic={
                'zone_a__1': {
                    'zone_name': 'zone_a', 'route_id': 'zone_a__1', 'counts': {'friendly': 2, 'enemy': 1},
                    'task_type': 'recon_zone', 'objective_type': 'recon',
                },
                'zone_b__2': {
                    'zone_name': 'zone_b', 'route_id': 'zone_b__2', 'counts': {'enemy': 1},
                    'task_type': 'facility_attack', 'objective_type': 'facility_attack',
                    'action_summary': {'attempted': True, 'confirmed': True},
                },
            },
            latest_frame_region_counts={},
            last_health={},
            terminal_state_seen=True,
        )
        self.assertIn('task_results_dynamic', payload)
        self.assertEqual(payload['totals_by_task_type']['recon_zone'], 1)
        self.assertEqual(payload['totals_by_task_type']['facility_attack'], 1)
        self.assertEqual(len(payload['facility_actions']), 1)

    def test_zone_capture_normalization_preserves_task_evidence_fields(self):
        payload = zone_capture_payload_to_dynamic_payload({
            'zone_name': 'zone_a',
            'status': 'ok',
            'friendly': 1,
            'task_type': 'hazard_avoid',
            'objective_type': 'hazard_avoid',
            'mission_outcome': 'hazard_avoided',
            'hazard_summary': {'hazard_type': 'anti_tank_cone'},
        }, ['friendly', 'enemy', 'hostage'])
        self.assertEqual(payload['task_type'], 'hazard_avoid')
        self.assertEqual(payload['hazard_summary']['hazard_type'], 'anti_tank_cone')

    def test_summarize_task_result_metrics_extracts_hazard_and_facility_views(self):
        metrics = summarize_task_result_metrics({
            'hazard_a': {'task_type': 'hazard_avoid', 'objective_type': 'hazard_avoid', 'hazard_summary': {'hazard_type': 'anti_tank_cone'}},
            'facility_b': {'task_type': 'facility_attack', 'objective_type': 'facility_attack', 'action_summary': {'attempted': True}},
        })
        self.assertEqual(metrics['totals_by_task_type']['hazard_avoid'], 1)
        self.assertEqual(metrics['totals_by_task_type']['facility_attack'], 1)
        self.assertEqual(len(metrics['hazard_observations']), 1)
        self.assertEqual(len(metrics['facility_actions']), 1)

    def test_zone_capture_dynamic_normalization_recomputes_schema_hash(self):
        payload = zone_capture_payload_to_dynamic_payload({
            'zone_name': 'zone_a',
            'status': 'ok',
            'class_names': ['friendly', 'enemy', 'hostage'],
            'class_counts': [1, 2, 0],
            'class_schema_hash': 'bad-hash',
        }, ['friendly', 'enemy', 'hostage'])
        self.assertEqual(payload['class_schema_hash'], class_schema_hash(['friendly', 'enemy', 'hostage']))


if __name__ == '__main__':
    unittest.main()


class ProfileContractUtilityTest(unittest.TestCase):
    def test_profile_role_validation_rejects_unknown_value(self):
        with self.assertRaises(ConfigurationError):
            validate_profile_role('unknown', owner='unit')

    def test_route_frame_region_contract_requires_binding_when_enabled(self):
        with self.assertRaises(ConfigurationError):
            validate_route_frame_region_contract([{'name': 'zone_a', 'frame_region': ''}], require_binding=True, allowed_frame_regions=['zone_a'])

    def test_named_region_contract_rejects_expected_set_mismatch(self):
        with self.assertRaises(ConfigurationError):
            validate_named_region_contract([{'name': 'zone_a', 'x0': 0, 'y0': 0, 'x1': 5, 'y1': 5}], expected_region_names=['zone_b'], require_named_regions=True)


class ZoneCaptureDynamicTypedNormalizationAuditTests(unittest.TestCase):
    def test_zone_capture_dynamic_typed_normalization_preserves_extended_fields(self):
        import sys
        import types
        from types import SimpleNamespace

        sys.modules.setdefault('rospy', types.SimpleNamespace(logwarn_throttle=lambda *args, **kwargs: None))
        std_msgs_module = sys.modules.setdefault('std_msgs', types.ModuleType('std_msgs'))
        std_msgs_msg_module = sys.modules.setdefault('std_msgs.msg', types.ModuleType('std_msgs.msg'))
        if not hasattr(std_msgs_msg_module, 'String'):
            class _String:
                def __init__(self, data=''):
                    self.data = data
            std_msgs_msg_module.String = _String
        std_msgs_module.msg = std_msgs_msg_module

        from ruikang_recon_baseline.recorder_ingest import RecorderIngestPipeline

        class DummyOwner:
            def __init__(self):
                self.config = {'zone_capture_dynamic_topic': 'recon/zone_capture_result_dynamic'}
                self.events = []

            def _append(self, event_type, payload):
                self.events.append((event_type, payload))

        owner = DummyOwner()
        pipeline = RecorderIngestPipeline(owner)
        msg = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 12.5)),
            zone_name='zone_alpha',
            route_id='route_alpha',
            status='ok',
            class_names=['friendly', 'enemy', 'hostage'],
            class_counts=[1, 2, 0],
            capture_started_at=10.0,
            capture_finished_at=12.0,
            frame_count=3,
            frame_region='region_a',
            failure_reason='',
            schema_version='2.4.0',
            class_schema_hash=class_schema_hash(['friendly', 'enemy', 'hostage']),
            task_type='facility_attack',
            objective_type='facility_attack',
            mission_outcome='facility_action_confirmed',
            task_metadata_json='{"attack_mode":"report_only"}',
            position_estimates_json='[{"class_name":"enemy","position_label":"zone_alpha"}]',
            evidence_summary_json='{"position_observation_count": 1}',
            hazard_summary_json='{}',
            action_summary_json='{"attempted": true, "confirmed": true}',
        )
        payload = pipeline.normalize_zone_capture_dynamic_typed(msg)
        self.assertEqual(payload['task_type'], 'facility_attack')
        self.assertEqual(payload['objective_type'], 'facility_attack')
        self.assertEqual(payload['mission_outcome'], 'facility_action_confirmed')
        self.assertEqual(payload['task_metadata']['attack_mode'], 'report_only')
        self.assertEqual(payload['position_estimates'][0]['position_label'], 'zone_alpha')
        self.assertTrue(payload['action_summary']['confirmed'])

