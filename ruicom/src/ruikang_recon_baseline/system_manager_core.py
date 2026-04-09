"""Pure-Python lifecycle and supervisor helpers for the system manager node."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Mapping, Optional, Tuple


LIFECYCLE_STATES = ('BOOTSTRAP', 'READY', 'ACTIVE', 'PAUSED', 'DEGRADED', 'FAULT', 'SHUTDOWN')
TERMINAL_MISSION_STATES = {'TIMEOUT', 'FAULT', 'SHUTDOWN'}
ACTIVE_MISSION_STATES = {'DISPATCH_PENDING', 'DISPATCHED', 'ACTIVE', 'DWELL', 'FINISHED'}
NODE_RUNTIME_CONFIGURED_STATES = {'READY', 'ACTIVE', 'PAUSED', 'SHUTDOWN'}
NODE_RUNTIME_ACTIVE_STATES = {'ACTIVE'}

AUTO_CONTROL_MODES = {'AUTO', 'AUTONOMOUS', 'COMMANDER', 'MISSION', 'NAVIGATION'}
MANUAL_CONTROL_MODES = {'MANUAL', 'HANDLE', 'VOICE', 'PS2', 'TELEOP', 'REMOTE', 'JOYSTICK'}


def is_operator_control_mode(mode: str) -> bool:
    """Return whether a control mode semantically represents operator takeover."""
    normalized = str(mode).strip().upper()
    if not normalized:
        return False
    if normalized in AUTO_CONTROL_MODES:
        return False
    if normalized in MANUAL_CONTROL_MODES:
        return True
    return normalized not in ('UNKNOWN', 'UNSPECIFIED')


def is_autonomous_control_mode(mode: str) -> bool:
    """Return whether a control mode semantically represents autonomous control."""
    normalized = str(mode).strip().upper()
    return bool(normalized) and normalized in AUTO_CONTROL_MODES


@dataclass
class NodeHealthSnapshot:
    stamp: float = 0.0
    status: str = ''
    message: str = ''
    details: Dict[str, object] = field(default_factory=dict)


@dataclass
class SystemManagerDecision:
    state: str
    command: str = ''
    reason: str = ''
    details: Dict[str, object] = field(default_factory=dict)



def resolve_required_nodes(required_nodes: Iterable[str], enabled_components: Mapping[str, bool]) -> Tuple[str, ...]:
    role_map = {
        'vision_counter_node': 'vision',
        'mission_manager_node': 'mission',
        'mission_recorder_node': 'recorder',
        'cmd_safety_mux_node': 'safety',
        'platform_bridge_node': 'bridge',
    }
    resolved = []
    for item in required_nodes:
        name = str(item).strip()
        if not name:
            continue
        role = role_map.get(name)
        if role is not None and not bool(enabled_components.get(role, True)):
            continue
        resolved.append(name)
    if not resolved:
        raise ValueError('required_nodes resolved to an empty set after launch-time filtering')
    return tuple(resolved)


class SystemManagerCore:
    """Lifecycle state machine independent of ROS transport plumbing.

    The core now stages managed-node bring-up in two ordered phases:

    1. ``configure`` nodes from ``IDLE`` into ``READY`` in declaration order.
    2. ``activate`` nodes from ``READY``/``PAUSED`` into ``ACTIVE`` in the same order.

    Historical snapshots without explicit runtime-state metadata remain accepted;
    they are treated as already configured and ready so compatibility tests and
    older unmanaged nodes do not regress.
    """

    def __init__(self, *, required_nodes: Iterable[str], ready_timeout_sec: float, health_freshness_sec: float, auto_activate: bool, readiness_requirements: Mapping[str, Iterable[str]] | None = None):
        self.required_nodes = tuple(str(item).strip() for item in required_nodes if str(item).strip())
        if not self.required_nodes:
            raise ValueError('required_nodes must not be empty')
        self.ready_timeout_sec = float(ready_timeout_sec)
        self.health_freshness_sec = float(health_freshness_sec)
        if self.ready_timeout_sec <= 0.0:
            raise ValueError('ready_timeout_sec must be > 0')
        if self.health_freshness_sec <= 0.0:
            raise ValueError('health_freshness_sec must be > 0')
        self.auto_activate = bool(auto_activate)
        self.readiness_requirements = {
            str(node).strip(): tuple(str(item).strip() for item in values if str(item).strip())
            for node, values in dict(readiness_requirements or {}).items()
            if str(node).strip()
        }
        self.state = 'BOOTSTRAP'
        self.started_at = 0.0
        self.bootstrapped = False
        self.last_command = ''
        self.last_mission_state = 'IDLE'
        self.node_health: Dict[str, NodeHealthSnapshot] = {name: NodeHealthSnapshot() for name in self.required_nodes}

    def observe_health(self, *, node: str, status: str, message: str, stamp: float, details: Optional[Mapping[str, object]] = None) -> None:
        normalized_node = str(node).strip()
        if not normalized_node:
            return
        self.node_health.setdefault(normalized_node, NodeHealthSnapshot())
        self.node_health[normalized_node] = NodeHealthSnapshot(
            stamp=float(stamp),
            status=str(status).strip().lower(),
            message=str(message).strip(),
            details=dict(details or {}),
        )

    def observe_mission_state(self, mission_state: str) -> None:
        self.last_mission_state = str(mission_state).strip().upper() or 'IDLE'

    def bootstrap(self, now_sec: float) -> None:
        self.started_at = max(0.0, float(now_sec))
        self.bootstrapped = True
        self.state = 'BOOTSTRAP'
        self.last_command = ''

    def _snapshot_ready(self, snapshot: NodeHealthSnapshot, node: str) -> Tuple[bool, list[str]]:
        required = self.readiness_requirements.get(node, ())
        missing = []
        for key in required:
            if not bool(snapshot.details.get(key)):
                missing.append(key)
        return not missing, missing

    @staticmethod
    def _runtime_state(snapshot: NodeHealthSnapshot) -> str:
        return str(snapshot.details.get('runtime_state', '') or '').strip().upper()

    @staticmethod
    def _lifecycle_managed(snapshot: NodeHealthSnapshot) -> bool:
        return bool(snapshot.details.get('lifecycle_managed', False))

    def _node_configured(self, node: str) -> bool:
        snapshot = self.node_health.get(node, NodeHealthSnapshot())
        if not self._lifecycle_managed(snapshot):
            return True
        runtime_state = self._runtime_state(snapshot)
        if not runtime_state:
            return True
        return runtime_state in NODE_RUNTIME_CONFIGURED_STATES

    def _node_active(self, node: str) -> bool:
        snapshot = self.node_health.get(node, NodeHealthSnapshot())
        if not self._lifecycle_managed(snapshot):
            return True
        runtime_state = self._runtime_state(snapshot)
        if not runtime_state:
            return True
        return runtime_state in NODE_RUNTIME_ACTIVE_STATES

    def _next_configure_target(self) -> str:
        for node in self.required_nodes:
            if not self._node_configured(node):
                return node
        return ''

    def _next_activate_target(self) -> str:
        for node in self.required_nodes:
            if self._node_configured(node) and not self._node_active(node):
                return node
        return ''

    def _has_explicit_runtime_metadata(self) -> bool:
        return any(
            self._lifecycle_managed(self.node_health.get(node, NodeHealthSnapshot()))
            or bool(self._runtime_state(self.node_health.get(node, NodeHealthSnapshot())))
            for node in self.required_nodes
        )

    def _readiness_snapshot(self, now_sec: float) -> dict:
        stale_nodes: List[dict] = []
        semantic_blockers: List[dict] = []
        warn_nodes: List[Tuple[str, str]] = []
        runtime_states: Dict[str, str] = {}
        for node in self.required_nodes:
            snapshot = self.node_health.get(node, NodeHealthSnapshot())
            runtime_states[node] = self._runtime_state(snapshot)
            if snapshot.stamp <= 0.0 or (now_sec - snapshot.stamp) > self.health_freshness_sec:
                stale_nodes.append({'node': node, 'reason': 'stale'})
                continue
            ready, blockers = self._snapshot_ready(snapshot, node)
            if not ready:
                semantic_blockers.append({'node': node, 'reason': 'resource_not_ready', 'blockers': blockers})
            if snapshot.status in ('error', 'fatal'):
                warn_nodes.append((node, snapshot.status))
            elif snapshot.status in ('warn', 'warning'):
                warn_nodes.append((node, 'warn'))
        graph_ready = not stale_nodes
        semantic_ready = graph_ready and not semantic_blockers
        mission_ready = semantic_ready and not warn_nodes and self.last_mission_state not in TERMINAL_MISSION_STATES
        return {
            'graph_ready': graph_ready,
            'semantic_ready': semantic_ready,
            'mission_ready': mission_ready,
            'stale_nodes': stale_nodes,
            'semantic_blockers': semantic_blockers,
            'warn_nodes': warn_nodes,
            'runtime_states': runtime_states,
            'configure_target': self._next_configure_target() if semantic_ready else '',
            'activate_target': self._next_activate_target() if semantic_ready else '',
        }

    def request_command(self, command: str, *, reason: str) -> SystemManagerDecision:
        normalized = str(command).strip().lower()
        self.last_command = normalized
        if normalized == 'activate':
            self.state = 'ACTIVE'
        elif normalized == 'pause':
            self.state = 'PAUSED'
        elif normalized == 'reset':
            self.state = 'BOOTSTRAP'
        elif normalized == 'shutdown':
            self.state = 'SHUTDOWN'
        elif normalized == 'configure':
            self.state = 'READY'
        return SystemManagerDecision(state=self.state, command=normalized, reason=str(reason).strip())

    def tick(self, now_sec: float) -> SystemManagerDecision:
        if not self.bootstrapped:
            raise ValueError('bootstrap() must be called before tick()')
        if self.state == 'SHUTDOWN':
            return SystemManagerDecision(state='SHUTDOWN', reason='shutdown_requested')
        readiness = self._readiness_snapshot(now_sec)
        mission_state = self.last_mission_state
        if mission_state in TERMINAL_MISSION_STATES:
            self.state = 'FAULT'
            return SystemManagerDecision(state='FAULT', reason='mission_terminal_state', details={'mission_state': mission_state, 'readiness': readiness})

        missing_nodes = list(readiness['stale_nodes']) + list(readiness['semantic_blockers'])
        if missing_nodes:
            if (now_sec - self.started_at) >= self.ready_timeout_sec:
                self.state = 'FAULT'
                return SystemManagerDecision(state='FAULT', reason='startup_timeout', details={'missing_nodes': missing_nodes, 'readiness': readiness})
            self.state = 'BOOTSTRAP'
            return SystemManagerDecision(state='BOOTSTRAP', reason='waiting_for_ready', details={'missing_nodes': missing_nodes, 'readiness': readiness})

        warn_nodes = list(readiness['warn_nodes'])
        if any(level in ('error', 'fatal') for _, level in warn_nodes):
            self.state = 'FAULT'
            return SystemManagerDecision(state='FAULT', reason='node_error', details={'issues': warn_nodes, 'readiness': readiness})
        if warn_nodes:
            self.state = 'DEGRADED'
            return SystemManagerDecision(state='DEGRADED', reason='node_warning', details={'issues': warn_nodes, 'readiness': readiness})

        configure_target = str(readiness.get('configure_target', '')).strip()
        if configure_target:
            self.state = 'BOOTSTRAP'
            self.last_command = 'configure'
            return SystemManagerDecision(
                state='BOOTSTRAP',
                command='configure',
                reason='configure_node',
                details={'target_node': configure_target, 'readiness': readiness},
            )

        if mission_state == 'PAUSED':
            self.state = 'PAUSED'
            return SystemManagerDecision(state='PAUSED', reason='mission_paused', details={'readiness': readiness})
        if mission_state in ACTIVE_MISSION_STATES:
            self.state = 'ACTIVE'
            return SystemManagerDecision(state='ACTIVE', reason='mission_active', details={'mission_state': mission_state, 'readiness': readiness})

        if self.auto_activate:
            activate_target = str(readiness.get('activate_target', '')).strip()
            if activate_target:
                self.state = 'READY'
                self.last_command = 'activate'
                return SystemManagerDecision(
                    state='READY',
                    command='activate',
                    reason='activate_node',
                    details={'target_node': activate_target, 'readiness': readiness},
                )
            self.state = 'ACTIVE'
            if not self._has_explicit_runtime_metadata():
                self.last_command = 'activate'
                return SystemManagerDecision(state='ACTIVE', command='activate', reason='auto_activate_on_ready', details={'mission_state': mission_state, 'readiness': readiness})
            return SystemManagerDecision(state='ACTIVE', reason='all_nodes_active_idle', details={'mission_state': mission_state, 'readiness': readiness})

        if self.state in ('BOOTSTRAP', 'DEGRADED'):
            self.state = 'READY'
        return SystemManagerDecision(state=self.state, reason='healthy_idle', details={'mission_state': mission_state, 'readiness': readiness})
