"""ROS system manager node for lifecycle supervision and mission control."""

from __future__ import annotations

from typing import Dict, Optional

import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse

from .common import JsonCodec, SCHEMA_VERSION, require_positive_float
from .msg import HealthState, MissionState
from .system_manager_core import LIFECYCLE_STATES, SystemManagerCore, is_operator_control_mode, resolve_required_nodes
from .lifecycle_protocol import encode_lifecycle_control
from .time_core import NodeClock


class SystemManagerNode:
    """Coordinate stack readiness and mission lifecycle commands.

    The node supervises the health of vision/mission/recorder/safety participants,
    publishes lifecycle status, and exposes operator-facing services that translate
    into mission control commands.
    """

    def __init__(self):
        rospy.init_node('system_manager_node', anonymous=False)
        self.config = self._read_config()
        self.clock = NodeClock(self.config['time_source_mode'])
        self.core = SystemManagerCore(
            required_nodes=self.config['required_nodes'],
            ready_timeout_sec=self.config['ready_timeout_sec'],
            health_freshness_sec=self.config['health_freshness_sec'],
            auto_activate=self.config['auto_activate'],
            readiness_requirements=self.config['readiness_requirements'],
        )
        self.core.bootstrap(self.clock.now_business_sec())
        self.command_pub = rospy.Publisher(self.config['control_command_topic'], String, queue_size=10, latch=True)
        self.state_pub = rospy.Publisher(self.config['manager_state_topic'], String, queue_size=10, latch=True)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10)
        self.evidence_pub = rospy.Publisher(self.config['runtime_evidence_topic'], String, queue_size=10)
        rospy.Subscriber(self.config['health_typed_topic'], HealthState, self._health_typed_cb, queue_size=50)
        rospy.Subscriber(self.config['mission_state_typed_topic'], MissionState, self._mission_state_cb, queue_size=50)
        self._activate_srv = rospy.Service('~activate', Trigger, self._srv_activate)
        self._pause_srv = rospy.Service('~pause', Trigger, self._srv_pause)
        self._resume_srv = rospy.Service('~resume', Trigger, self._srv_resume)
        self._reset_srv = rospy.Service('~reset', Trigger, self._srv_reset)
        self._shutdown_srv = rospy.Service('~shutdown', Trigger, self._srv_shutdown)
        self.last_published_state = ''
        self.last_health_reason = ''
        self.last_health_details: Dict[str, object] = {}
        self.current_control_mode = ''
        self.estop_active = False
        self.evidence_event_seq = 0
        rospy.Subscriber(self.config['control_mode_topic'], String, self._control_mode_cb, queue_size=20)
        rospy.Subscriber(self.config['estop_topic'], Bool, self._estop_cb, queue_size=20)
        self._publish_health('ok', 'node_ready', {'required_nodes': list(self.config['required_nodes'])})
        rospy.loginfo('system_manager_node started. required_nodes=%s auto_activate=%s', self.config['required_nodes'], self.config['auto_activate'])

    def _read_config(self) -> dict:
        """Read and validate system-manager parameters."""
        config = {
            'health_topic': rospy.get_param('~health_topic', 'recon/health'),
            'health_typed_topic': rospy.get_param('~health_typed_topic', 'recon/health_typed'),
            'mission_state_typed_topic': rospy.get_param('~mission_state_typed_topic', 'recon/mission_state_typed'),
            'control_command_topic': rospy.get_param('~control_command_topic', 'recon/system_manager/command'),
            'manager_state_topic': rospy.get_param('~manager_state_topic', 'recon/system_manager/state'),
            'required_nodes': rospy.get_param('~required_nodes', ['vision_counter_node', 'mission_manager_node', 'mission_recorder_node', 'cmd_safety_mux_node', 'platform_bridge_node']),
            'enable_vision_component': bool(rospy.get_param('~enable_vision_component', True)),
            'enable_mission_component': bool(rospy.get_param('~enable_mission_component', True)),
            'enable_recorder_component': bool(rospy.get_param('~enable_recorder_component', True)),
            'enable_safety_component': bool(rospy.get_param('~enable_safety_component', True)),
            'enable_platform_bridge_component': bool(rospy.get_param('~enable_platform_bridge_component', True)),
            'ready_timeout_sec': float(rospy.get_param('~ready_timeout_sec', 10.0)),
            'health_freshness_sec': float(rospy.get_param('~health_freshness_sec', 3.0)),
            'publish_rate_hz': float(rospy.get_param('~publish_rate_hz', 5.0)),
            'auto_activate': bool(rospy.get_param('~auto_activate', True)),
            'time_source_mode': str(rospy.get_param('~time_source_mode', 'ros')).strip().lower(),
            'runtime_evidence_topic': str(rospy.get_param('~runtime_evidence_topic', 'recon/runtime/evidence')).strip() or 'recon/runtime/evidence',
            'control_mode_topic': str(rospy.get_param('~control_mode_topic', 'recon/control_mode')).strip() or 'recon/control_mode',
            'estop_topic': str(rospy.get_param('~estop_topic', 'recon/estop')).strip() or 'recon/estop',
            'readiness_requirements': rospy.get_param('~readiness_requirements', {
                'vision_counter_node': ['camera_topic_bound', 'camera_frame_fresh', 'detector_contract_satisfied', 'detector_manifest_grade_satisfied', 'field_asset_ready', 'vendor_runtime_contract_satisfied', 'vendor_bundle_preflight_satisfied'],
                'mission_manager_node': ['route_bound', 'navigation_contract_satisfied', 'field_asset_ready', 'navigation_runtime_probe_satisfied', 'vendor_runtime_contract_satisfied', 'vendor_bundle_preflight_satisfied'],
                'mission_recorder_node': ['writer_ready'],
                'cmd_safety_mux_node': ['command_path_bound', 'output_feedback_required_satisfied'],
                'platform_bridge_node': ['command_path_bound', 'command_flow_contract_satisfied', 'feedback_contract_satisfied', 'platform_runtime_probe_satisfied', 'vendor_runtime_contract_satisfied', 'vendor_bundle_preflight_satisfied'],
            }),
        }
        config['ready_timeout_sec'] = require_positive_float('ready_timeout_sec', config['ready_timeout_sec'])
        config['health_freshness_sec'] = require_positive_float('health_freshness_sec', config['health_freshness_sec'])
        config['publish_rate_hz'] = require_positive_float('publish_rate_hz', config['publish_rate_hz'])
        config['required_nodes'] = resolve_required_nodes(
            config['required_nodes'],
            {
                'vision': config['enable_vision_component'],
                'mission': config['enable_mission_component'],
                'recorder': config['enable_recorder_component'],
                'safety': config['enable_safety_component'],
                'bridge': config['enable_platform_bridge_component'],
            },
        )
        config['readiness_requirements'] = {
            str(node).strip(): [str(item).strip() for item in values if str(item).strip()]
            for node, values in dict(config['readiness_requirements']).items()
            if str(node).strip()
        }
        if config['time_source_mode'] not in ('ros', 'wall'):
            raise RuntimeError('time_source_mode must be ros or wall')
        return config

    def _publish_health(self, status: str, message: str, details: Optional[dict] = None) -> None:
        payload = {
            'stamp': self.clock.now_business_sec(),
            'node': 'system_manager_node',
            'status': str(status).strip(),
            'message': str(message).strip(),
            'schema_version': SCHEMA_VERSION,
            'details': details or {},
        }
        self.health_pub.publish(String(data=JsonCodec.dumps(payload)))
        typed = HealthState()
        typed.header.stamp = self.clock.now_ros_time()
        typed.header.frame_id = 'map'
        typed.node = payload['node']
        typed.status = payload['status']
        typed.message = payload['message']
        typed.schema_version = payload['schema_version']
        typed.details_json = JsonCodec.dumps(payload.get('details', {}))
        self.health_typed_pub.publish(typed)
        self.last_health_reason = payload['message']
        self.last_health_details = dict(payload.get('details', {}))

    def _publish_state(self, state: str, *, reason: str, command: str = '', details: Optional[dict] = None) -> None:
        payload = {
            'stamp': self.clock.now_business_sec(),
            'state': str(state).strip().upper(),
            'reason': str(reason).strip(),
            'command': str(command).strip().lower(),
            'schema_version': SCHEMA_VERSION,
            'details': details or {},
        }
        self.state_pub.publish(String(data=JsonCodec.dumps(payload)))
        evidence = {
            'stamp': payload['stamp'],
            'manager_state': payload['state'],
            'manager_reason': payload['reason'],
            'manager_command': payload['command'],
            'mission_state': self.core.last_mission_state,
            'current_control_mode': self.current_control_mode,
            'operator_control_active': is_operator_control_mode(self.current_control_mode),
            'estop_active': bool(self.estop_active),
            'required_nodes': list(self.config['required_nodes']),
            'node_health': {
                node: {
                    'status': snapshot.status,
                    'message': snapshot.message,
                    'stamp': snapshot.stamp,
                    'details': dict(snapshot.details or {}),
                }
                for node, snapshot in self.core.node_health.items()
            },
            'details': dict(details or {}),
        }
        self.evidence_pub.publish(String(data=JsonCodec.dumps(evidence)))
        self.last_published_state = payload['state']
        if command:
            target = str((details or {}).get('target_node', '')).strip()
            wire = encode_lifecycle_control(command, target=target, issued_by='system_manager_node', metadata={'reason': reason})
            self.command_pub.publish(String(data=wire))

    def _publish_runtime_event(self, event_type: str, details: Optional[dict] = None) -> None:
        details = dict(details or {})
        self.evidence_event_seq += 1
        payload = {
            'stamp': self.clock.now_business_sec(),
            'schema_version': SCHEMA_VERSION,
            'event_type': str(event_type).strip().lower(),
            'event_seq': int(self.evidence_event_seq),
            'mission_state': self.core.last_mission_state,
            'manager_state': self.core.state,
            'current_control_mode': self.current_control_mode,
            'manual_takeover': is_operator_control_mode(self.current_control_mode),
            'estop_active': bool(self.estop_active),
            'details': details,
        }
        self.evidence_pub.publish(String(data=JsonCodec.dumps(payload)))

    def _control_mode_cb(self, msg: String) -> None:
        new_mode = str(msg.data).strip().upper()
        previous_mode = self.current_control_mode
        if not new_mode:
            return
        if new_mode == previous_mode:
            self.current_control_mode = new_mode
            return
        self.current_control_mode = new_mode
        self._publish_runtime_event('control_mode_changed', {
            'previous_control_mode': previous_mode,
            'control_mode': new_mode,
            'manual_takeover': is_operator_control_mode(new_mode),
        })

    def _estop_cb(self, msg: Bool) -> None:
        new_value = bool(msg.data)
        if new_value == bool(self.estop_active):
            self.estop_active = new_value
            return
        self.estop_active = new_value
        self._publish_runtime_event('estop_changed', {'estop_active': new_value})

    def _health_typed_cb(self, msg: HealthState) -> None:
        if msg.node == 'system_manager_node':
            return
        try:
            details = JsonCodec.loads(msg.details_json) if msg.details_json else {}
        except Exception:
            details = {}
        self.core.observe_health(
            node=msg.node,
            status=msg.status,
            message=msg.message,
            stamp=msg.header.stamp.to_sec() if msg.header.stamp else self.clock.now_business_sec(),
            details=details,
        )

    def _mission_state_cb(self, msg: MissionState) -> None:
        self.core.observe_mission_state(msg.state)

    def _issue_manual_command(self, command: str, reason: str) -> TriggerResponse:
        decision = self.core.request_command(command, reason=reason)
        self._publish_state(decision.state, reason=decision.reason, command=decision.command, details=decision.details)
        self._publish_runtime_event('manual_command', {
            'command': command,
            'reason': reason,
            'accepted': True,
            'resulting_state': decision.state,
        })
        level = 'warn' if decision.state in ('PAUSED', 'BOOTSTRAP') else 'ok'
        if decision.state == 'SHUTDOWN':
            level = 'warn'
        self._publish_health(level, 'manual_{}'.format(command), {'state': decision.state})
        return TriggerResponse(success=True, message='{} -> {}'.format(command, decision.state))

    def _srv_activate(self, _req):
        return self._issue_manual_command('activate', 'manual_activate')

    def _srv_pause(self, _req):
        return self._issue_manual_command('pause', 'manual_pause')

    def _srv_resume(self, _req):
        return self._issue_manual_command('activate', 'manual_resume')

    def _srv_reset(self, _req):
        return self._issue_manual_command('reset', 'manual_reset')

    def _srv_shutdown(self, _req):
        return self._issue_manual_command('shutdown', 'manual_shutdown')

    def spin(self) -> None:
        """Run the lifecycle supervision loop until ROS shutdown."""
        rate = rospy.Rate(self.config['publish_rate_hz'])
        while not rospy.is_shutdown():
            try:
                decision = self.core.tick(self.clock.now_business_sec())
                if decision.command or decision.state != self.last_published_state:
                    self._publish_state(decision.state, reason=decision.reason, command=decision.command, details=decision.details)
                if decision.state == 'FAULT':
                    self._publish_health('error', decision.reason, decision.details)
                elif decision.state == 'DEGRADED':
                    self._publish_health('warn', decision.reason, decision.details)
                elif decision.state in ('READY', 'ACTIVE') and self.last_health_reason != decision.reason:
                    self._publish_health('ok', decision.reason, decision.details)
            except Exception as exc:
                self._publish_health('error', 'system_manager_step_failed', {'error': str(exc)})
                self._publish_state('FAULT', reason='system_manager_step_failed', details={'error': str(exc)})
            rate.sleep()
