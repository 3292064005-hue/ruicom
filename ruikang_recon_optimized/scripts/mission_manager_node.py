#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mission execution node with explicit navigation adapters and capture windows."""

from __future__ import annotations

import os
import time
from dataclasses import asdict
from typing import Callable, Dict, List, Optional

import actionlib
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from ruikang_recon_baseline.common import (
    CLASS_NAMES,
    SCHEMA_VERSION,
    ConfigurationError,
    Detection,
    DetectionFrame,
    JsonCodec,
    MissionStateSnapshot,
    PoseSnapshot,
    Waypoint,
    ZoneCaptureResult,
    quaternion_to_yaw_rad,
    require_positive_float,
    yaw_deg_to_quaternion_tuple,
)
from ruikang_recon_baseline.io_core import AsyncJsonlWriter
from ruikang_recon_baseline.mission_core import AggregationPolicy, ArrivalEvaluator, CaptureWindowAggregator
from ruikang_recon_baseline.msg import DetectionArray, MissionState, ZoneCapture


CANONICAL_NAV_STATUSES = {'IDLE', 'DISPATCHED', 'PENDING', 'ACTIVE', 'SUCCEEDED', 'PREEMPTED', 'ABORTED'}


class NavigationAdapterBase:
    """Base class for ROS navigation adapters."""

    def dispatch(self, waypoint: Waypoint) -> None:
        raise NotImplementedError

    def poll(self, now_sec: float) -> str:
        raise NotImplementedError

    def cancel(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        return None


class MoveBaseActionAdapter(NavigationAdapterBase):
    """Navigation adapter backed by ``move_base`` action results."""

    def __init__(self, action_name: str, wait_for_server_sec: float):
        self.client = actionlib.SimpleActionClient(action_name, MoveBaseAction)
        if not self.client.wait_for_server(rospy.Duration(wait_for_server_sec)):
            raise ConfigurationError('move_base action server unavailable: {}'.format(action_name))
        self.current_waypoint: Optional[Waypoint] = None

    def dispatch(self, waypoint: Waypoint) -> None:
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = waypoint.goal_frame
        goal.target_pose.pose.position.x = waypoint.x
        goal.target_pose.pose.position.y = waypoint.y
        z, w = yaw_deg_to_quaternion_tuple(waypoint.yaw_deg)[2:]
        goal.target_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)
        self.client.send_goal(goal)
        self.current_waypoint = waypoint

    def poll(self, now_sec: float) -> str:
        _ = now_sec
        state = self.client.get_state()
        if state == actionlib.GoalStatus.PENDING:
            return 'PENDING'
        if state in (actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PREEMPTING, actionlib.GoalStatus.RECALLING):
            return 'ACTIVE'
        if state == actionlib.GoalStatus.SUCCEEDED:
            return 'SUCCEEDED'
        if state in (actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.RECALLED):
            return 'PREEMPTED'
        if state in (actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED, actionlib.GoalStatus.LOST):
            return 'ABORTED'
        return 'ACTIVE'

    def cancel(self) -> None:
        self.client.cancel_all_goals()


class SimpleGoalTopicAdapter(NavigationAdapterBase):
    """Simple-topic navigation adapter for demo and no-action environments."""

    def __init__(self, topic_name: str, evaluator: ArrivalEvaluator, simulate_arrival_without_pose: bool, synthetic_arrival_delay_sec: float):
        self.publisher = rospy.Publisher(topic_name, PoseStamped, queue_size=5)
        self.evaluator = evaluator
        self.simulate_arrival_without_pose = bool(simulate_arrival_without_pose)
        self.synthetic_arrival_delay_sec = float(synthetic_arrival_delay_sec)
        self.current_waypoint: Optional[Waypoint] = None
        self.goal_sent_at = 0.0

    def dispatch(self, waypoint: Waypoint) -> None:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = waypoint.goal_frame
        msg.pose.position.x = waypoint.x
        msg.pose.position.y = waypoint.y
        z, w = yaw_deg_to_quaternion_tuple(waypoint.yaw_deg)[2:]
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)
        self.publisher.publish(msg)
        self.current_waypoint = waypoint
        self.goal_sent_at = time.time()

    def poll(self, now_sec: float) -> str:
        if self.current_waypoint is None:
            return 'IDLE'
        if self.evaluator.has_arrived(self.current_waypoint, now_sec):
            return 'SUCCEEDED'
        if self.simulate_arrival_without_pose and not self.evaluator.pose_fresh(now_sec) and (now_sec - self.goal_sent_at) >= self.synthetic_arrival_delay_sec:
            return 'SUCCEEDED'
        return 'ACTIVE'

    def cancel(self) -> None:
        return None


class GoalTopicStatusAdapter(NavigationAdapterBase):
    """Navigation adapter backed by a goal topic plus an external status topic.

    This adapter exists for non-``move_base`` stacks that still expose an explicit
    navigation state stream. The external status topic must publish one of the
    canonical values: ``PENDING``, ``ACTIVE``, ``SUCCEEDED``, ``PREEMPTED``,
    ``ABORTED`` or ``IDLE``.
    """

    def __init__(self, goal_topic: str, status_topic: str, status_timeout_sec: float):
        self.publisher = rospy.Publisher(goal_topic, PoseStamped, queue_size=5)
        self.status_timeout_sec = require_positive_float('navigation_status_timeout_sec', status_timeout_sec)
        self.current_waypoint: Optional[Waypoint] = None
        self.goal_sent_at = 0.0
        self.last_status = 'IDLE'
        self.last_status_stamp = 0.0
        self.subscriber = rospy.Subscriber(status_topic, String, self._status_callback, queue_size=20)

    def _status_callback(self, msg: String) -> None:
        raw = str(msg.data).strip().upper()
        if not raw:
            return
        normalized = raw.replace('NAVIGATION_', '').replace('STATUS_', '')
        if normalized not in CANONICAL_NAV_STATUSES:
            rospy.logwarn_throttle(2.0, 'Ignoring unsupported navigation status: %s', raw)
            return
        self.last_status = normalized
        self.last_status_stamp = time.time()

    def dispatch(self, waypoint: Waypoint) -> None:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = waypoint.goal_frame
        msg.pose.position.x = waypoint.x
        msg.pose.position.y = waypoint.y
        z, w = yaw_deg_to_quaternion_tuple(waypoint.yaw_deg)[2:]
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)
        self.publisher.publish(msg)
        self.current_waypoint = waypoint
        self.goal_sent_at = time.time()
        self.last_status = 'PENDING'
        self.last_status_stamp = self.goal_sent_at

    def poll(self, now_sec: float) -> str:
        if self.current_waypoint is None:
            return 'IDLE'
        if self.last_status_stamp <= 0.0:
            return 'PENDING'
        if (now_sec - self.last_status_stamp) > self.status_timeout_sec:
            return 'ABORTED'
        return self.last_status

    def cancel(self) -> None:
        self.current_waypoint = None
        self.last_status = 'PREEMPTED'
        self.last_status_stamp = time.time()

    def close(self) -> None:
        try:
            self.subscriber.unregister()
        except Exception:
            pass


class TfLookupPoseSource:
    """Resolve robot pose from the TF tree into the mission comparison frame."""

    def __init__(self, target_frame: str, source_frame: str, lookup_timeout_sec: float):
        if not str(target_frame).strip():
            raise ConfigurationError('tf_target_frame must not be empty')
        if not str(source_frame).strip():
            raise ConfigurationError('tf_source_frame must not be empty')
        self.target_frame = str(target_frame).strip()
        self.source_frame = str(source_frame).strip()
        self.lookup_timeout_sec = require_positive_float('tf_lookup_timeout_sec', lookup_timeout_sec)
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.buffer)

    def lookup(self) -> PoseSnapshot:
        transform = self.buffer.lookup_transform(
            self.target_frame,
            self.source_frame,
            rospy.Time(0),
            rospy.Duration(self.lookup_timeout_sec),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp.to_sec() if transform.header.stamp else time.time()
        return PoseSnapshot(
            stamp=stamp,
            frame_id=self.target_frame,
            x=float(translation.x),
            y=float(translation.y),
            yaw_rad=quaternion_to_yaw_rad(rotation.x, rotation.y, rotation.z, rotation.w),
            source='tf_lookup',
        )


class MissionManagerNode:
    """Execute mission route, manage navigation feedback and aggregate capture windows."""

    def __init__(self):
        rospy.init_node('mission_manager_node', anonymous=False)
        self.config = self._read_config()
        self.output_root = os.path.expanduser(self.config['output_root'])
        os.makedirs(self.output_root, exist_ok=True)
        self.event_writer = AsyncJsonlWriter(
            os.path.join(self.output_root, 'mission_manager_events.jsonl'),
            max_queue_size=int(self.config['writer_queue_size']),
            rotate_max_bytes=int(self.config['writer_rotate_max_bytes']),
            rotate_keep=int(self.config['writer_rotate_keep']),
        )
        self.route: List[Waypoint] = [Waypoint.from_dict(item, self.config['dwell_default_sec']) for item in self.config['route']]
        self.current_zone = ''
        self.route_index = 0
        self.retry_count = 0
        self.state = 'IDLE'
        self.state_since = 0.0
        self.mission_started_at = 0.0
        self.dispatch_started_at = 0.0
        self.current_capture_deadline = 0.0
        self.zone_results = {}
        self.latest_detection_frame: Optional[DetectionFrame] = None
        self.last_health_publish = 0.0
        self.last_tf_warning = 0.0
        self.arrival_evaluator = ArrivalEvaluator(
            comparison_frame=self.config['comparison_frame'],
            reach_tolerance_m=self.config['goal_reach_tolerance_m'],
            pose_timeout_sec=self.config['pose_timeout_sec'],
        )
        self.capture_aggregator = CaptureWindowAggregator(AggregationPolicy(
            class_names=CLASS_NAMES,
            reduction=self.config['capture_reduction'],
            min_valid_frames=self.config['capture_min_valid_frames'],
        ))
        self.tf_pose_source: Optional[TfLookupPoseSource] = None
        self._pose_subscribers = []
        self._configure_pose_source()
        self.nav_adapter = self._build_navigation_adapter()

        self.current_zone_pub = rospy.Publisher(self.config['current_zone_topic'], String, queue_size=5, latch=True)
        self.mission_state_json_pub = rospy.Publisher(self.config['mission_state_topic'], String, queue_size=10, latch=True)
        self.mission_state_pub = rospy.Publisher(self.config['mission_state_typed_topic'], MissionState, queue_size=10, latch=True)
        self.zone_capture_json_pub = rospy.Publisher(self.config['zone_capture_topic'], String, queue_size=10, latch=True)
        self.zone_capture_pub = rospy.Publisher(self.config['zone_capture_typed_topic'], ZoneCapture, queue_size=10, latch=True)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)

        rospy.Subscriber(self.config['detections_topic'], DetectionArray, self._detections_callback, queue_size=20)
        rospy.on_shutdown(self._on_shutdown)

        if self.config['auto_start']:
            self._start_mission()
        rospy.loginfo('mission_manager_node started. route_len=%d adapter=%s pose_source=%s', len(self.route), self.config['navigation_adapter_type'], self.config['pose_source_type'])

    def _read_config(self):
        config = {
            'current_zone_topic': rospy.get_param('~current_zone_topic', '/recon/current_zone'),
            'mission_state_topic': rospy.get_param('~mission_state_topic', '/recon/mission_state'),
            'mission_state_typed_topic': rospy.get_param('~mission_state_typed_topic', '/recon/mission_state_typed'),
            'zone_capture_topic': rospy.get_param('~zone_capture_topic', '/recon/zone_capture_result'),
            'zone_capture_typed_topic': rospy.get_param('~zone_capture_typed_topic', '/recon/zone_capture_result_typed'),
            'detections_topic': rospy.get_param('~detections_topic', '/recon/detections'),
            'health_topic': rospy.get_param('~health_topic', '/recon/health'),
            'output_root': rospy.get_param('~output_root', '~/.ros/ruikang_recon'),
            'auto_start': bool(rospy.get_param('~auto_start', True)),
            'publish_rate_hz': float(rospy.get_param('~publish_rate_hz', 5.0)),
            'goal_reach_tolerance_m': float(rospy.get_param('~goal_reach_tolerance_m', 0.35)),
            'pose_timeout_sec': float(rospy.get_param('~pose_timeout_sec', 1.5)),
            'comparison_frame': rospy.get_param('~comparison_frame', 'map'),
            'dwell_default_sec': float(rospy.get_param('~dwell_default_sec', 4.0)),
            'mission_timeout_sec': float(rospy.get_param('~mission_timeout_sec', 240.0)),
            'retry_limit': int(rospy.get_param('~retry_limit', 1)),
            'capture_reduction': rospy.get_param('~capture_reduction', 'median'),
            'capture_min_valid_frames': int(rospy.get_param('~capture_min_valid_frames', 2)),
            'navigation_adapter_type': rospy.get_param('~navigation_adapter_type', 'move_base_action'),
            'move_base_action_name': rospy.get_param('~move_base_action_name', '/move_base'),
            'wait_for_action_server_sec': float(rospy.get_param('~wait_for_action_server_sec', 3.0)),
            'simple_goal_topic': rospy.get_param('~simple_goal_topic', '/move_base_simple/goal'),
            'navigation_status_topic': rospy.get_param('~navigation_status_topic', '/recon/navigation_status'),
            'navigation_status_timeout_sec': float(rospy.get_param('~navigation_status_timeout_sec', 3.0)),
            'pose_source_type': rospy.get_param('~pose_source_type', 'amcl_pose'),
            'odom_topic': rospy.get_param('~odom_topic', '/odom'),
            'amcl_pose_topic': rospy.get_param('~amcl_pose_topic', '/amcl_pose'),
            'tf_target_frame': rospy.get_param('~tf_target_frame', rospy.get_param('~comparison_frame', 'map')),
            'tf_source_frame': rospy.get_param('~tf_source_frame', 'base_link'),
            'tf_lookup_timeout_sec': float(rospy.get_param('~tf_lookup_timeout_sec', 0.2)),
            'simulate_arrival_without_pose': bool(rospy.get_param('~simulate_arrival_without_pose', False)),
            'synthetic_arrival_delay_sec': float(rospy.get_param('~synthetic_arrival_delay_sec', 1.0)),
            'route': rospy.get_param('~route', []),
            'writer_queue_size': int(rospy.get_param('~writer_queue_size', 512)),
            'writer_rotate_max_bytes': int(rospy.get_param('~writer_rotate_max_bytes', 5 * 1024 * 1024)),
            'writer_rotate_keep': int(rospy.get_param('~writer_rotate_keep', 3)),
        }
        if not config['route']:
            raise ConfigurationError('mission route is empty')
        config['publish_rate_hz'] = require_positive_float('publish_rate_hz', config['publish_rate_hz'])
        config['goal_reach_tolerance_m'] = require_positive_float('goal_reach_tolerance_m', config['goal_reach_tolerance_m'])
        config['pose_timeout_sec'] = require_positive_float('pose_timeout_sec', config['pose_timeout_sec'])
        config['dwell_default_sec'] = require_positive_float('dwell_default_sec', config['dwell_default_sec'])
        config['mission_timeout_sec'] = require_positive_float('mission_timeout_sec', config['mission_timeout_sec'])
        config['wait_for_action_server_sec'] = require_positive_float('wait_for_action_server_sec', config['wait_for_action_server_sec'])
        config['navigation_status_timeout_sec'] = require_positive_float('navigation_status_timeout_sec', config['navigation_status_timeout_sec'])
        config['tf_lookup_timeout_sec'] = require_positive_float('tf_lookup_timeout_sec', config['tf_lookup_timeout_sec'])
        config['synthetic_arrival_delay_sec'] = require_positive_float('synthetic_arrival_delay_sec', config['synthetic_arrival_delay_sec'])
        if int(config['retry_limit']) < 0:
            raise ConfigurationError('retry_limit must be >= 0')
        if int(config['capture_min_valid_frames']) <= 0:
            raise ConfigurationError('capture_min_valid_frames must be > 0')
        if str(config['capture_reduction']) not in ('median', 'max'):
            raise ConfigurationError('capture_reduction must be one of: median, max')
        if not str(config['comparison_frame']).strip():
            raise ConfigurationError('comparison_frame must not be empty')
        if str(config['pose_source_type']).strip() not in ('odometry', 'amcl_pose', 'tf_lookup'):
            raise ConfigurationError('Unsupported pose_source_type: {}'.format(config['pose_source_type']))
        if str(config['navigation_adapter_type']).strip().lower() not in ('move_base_action', 'simple_topic', 'goal_topic_status'):
            raise ConfigurationError('Unsupported navigation_adapter_type: {}'.format(config['navigation_adapter_type']))
        return config

    def _configure_pose_source(self) -> None:
        pose_source_type = str(self.config['pose_source_type']).strip()
        if pose_source_type == 'odometry':
            self._pose_subscribers.append(rospy.Subscriber(self.config['odom_topic'], Odometry, self._odom_callback, queue_size=20))
            return
        if pose_source_type == 'amcl_pose':
            self._pose_subscribers.append(rospy.Subscriber(self.config['amcl_pose_topic'], PoseWithCovarianceStamped, self._amcl_callback, queue_size=20))
            return
        self.tf_pose_source = TfLookupPoseSource(
            target_frame=self.config['tf_target_frame'],
            source_frame=self.config['tf_source_frame'],
            lookup_timeout_sec=self.config['tf_lookup_timeout_sec'],
        )

    def _navigation_factory(self) -> Dict[str, Callable[[], NavigationAdapterBase]]:
        return {
            'move_base_action': lambda: MoveBaseActionAdapter(self.config['move_base_action_name'], self.config['wait_for_action_server_sec']),
            'simple_topic': lambda: SimpleGoalTopicAdapter(
                topic_name=self.config['simple_goal_topic'],
                evaluator=self.arrival_evaluator,
                simulate_arrival_without_pose=self.config['simulate_arrival_without_pose'],
                synthetic_arrival_delay_sec=self.config['synthetic_arrival_delay_sec'],
            ),
            'goal_topic_status': lambda: GoalTopicStatusAdapter(
                goal_topic=self.config['simple_goal_topic'],
                status_topic=self.config['navigation_status_topic'],
                status_timeout_sec=self.config['navigation_status_timeout_sec'],
            ),
        }

    def _build_navigation_adapter(self):
        adapter_type = str(self.config['navigation_adapter_type']).strip().lower()
        factory = self._navigation_factory()
        if adapter_type not in factory:
            raise ConfigurationError('Unsupported navigation_adapter_type: {}'.format(adapter_type))
        return factory[adapter_type]()

    def _publish_health(self, status: str, message: str, details: dict | None = None):
        now_sec = time.time()
        if status == 'ok' and (now_sec - self.last_health_publish) < 0.5:
            return
        payload = {
            'stamp': now_sec,
            'node': 'mission_manager_node',
            'status': status,
            'message': message,
            'schema_version': SCHEMA_VERSION,
            'current_zone': self.current_zone,
            'route_index': self.route_index,
        }
        if details:
            payload['details'] = details
        writer_error = self.event_writer.last_error
        if writer_error:
            payload.setdefault('details', {})['writer_error'] = writer_error
        dropped_messages = self.event_writer.dropped_messages
        if dropped_messages:
            payload.setdefault('details', {})['writer_dropped_messages'] = dropped_messages
        self.health_pub.publish(String(data=JsonCodec.dumps(payload)))
        self.last_health_publish = now_sec

    def _emit_state(self, event: str, state: Optional[str] = None, details: Optional[dict] = None):
        if state is not None:
            self.state = state
            self.state_since = time.time()
        snapshot = MissionStateSnapshot(
            stamp=time.time(),
            state=self.state,
            event=event,
            route_index=self.route_index,
            route_total=len(self.route),
            current_zone=self.current_zone,
            details={
                'zone_results': self.zone_results,
                **(details or {}),
            },
        )
        self.mission_state_json_pub.publish(String(data=JsonCodec.dumps(snapshot.to_dict())))
        msg = MissionState()
        msg.header.stamp = rospy.Time.from_sec(snapshot.stamp)
        msg.header.frame_id = self.config['comparison_frame']
        msg.state = snapshot.state
        msg.event = snapshot.event
        msg.route_index = snapshot.route_index
        msg.route_total = snapshot.route_total
        msg.current_zone = snapshot.current_zone
        msg.schema_version = snapshot.schema_version
        msg.details_json = JsonCodec.dumps(snapshot.details)
        self.mission_state_pub.publish(msg)
        self.event_writer.write({'type': 'mission_state', 'payload': snapshot.to_dict()})

    def _publish_zone_capture(self, result: ZoneCaptureResult):
        payload = result.to_dict()
        self.zone_capture_json_pub.publish(String(data=JsonCodec.dumps(payload)))
        msg = ZoneCapture()
        msg.header.stamp = rospy.Time.from_sec(result.capture_finished_at or time.time())
        msg.header.frame_id = self.config['comparison_frame']
        msg.zone_name = result.zone_name
        msg.status = result.status
        msg.friendly = result.friendly
        msg.enemy = result.enemy
        msg.hostage = result.hostage
        msg.capture_started_at = result.capture_started_at
        msg.capture_finished_at = result.capture_finished_at
        msg.frame_count = result.frame_count
        msg.frame_region = result.frame_region
        msg.failure_reason = result.failure_reason
        msg.schema_version = result.schema_version
        self.zone_capture_pub.publish(msg)
        self.event_writer.write({'type': 'zone_capture', 'payload': payload})

    def _detections_callback(self, msg: DetectionArray):
        detections = []
        for item in msg.detections:
            detections.append(Detection(
                class_name=item.class_name,
                score=float(item.score),
                x1=int(item.x1),
                y1=int(item.y1),
                x2=int(item.x2),
                y2=int(item.y2),
                frame_region=item.frame_region,
            ))
        self.latest_detection_frame = DetectionFrame(
            stamp=msg.header.stamp.to_sec() if msg.header.stamp else time.time(),
            frame_id=msg.header.frame_id,
            detector_type=msg.detector_type,
            schema_version=msg.schema_version,
            detections=detections,
            source_image_width=int(msg.source_image_width),
            source_image_height=int(msg.source_image_height),
        )
        if self.state == 'DWELL':
            self.capture_aggregator.feed(self.latest_detection_frame, capture_end_hint=self.current_capture_deadline)

    def _odom_callback(self, msg: Odometry):
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else time.time()
        orientation = msg.pose.pose.orientation
        pose = PoseSnapshot(
            stamp=stamp,
            frame_id=msg.header.frame_id,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw_rad=quaternion_to_yaw_rad(orientation.x, orientation.y, orientation.z, orientation.w),
            source='odometry',
        )
        self.arrival_evaluator.update_pose(pose)

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else time.time()
        orientation = msg.pose.pose.orientation
        pose = PoseSnapshot(
            stamp=stamp,
            frame_id=msg.header.frame_id,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw_rad=quaternion_to_yaw_rad(orientation.x, orientation.y, orientation.z, orientation.w),
            source='amcl_pose',
        )
        self.arrival_evaluator.update_pose(pose)

    def _refresh_pose_source(self) -> None:
        if self.tf_pose_source is None:
            return
        try:
            self.arrival_evaluator.update_pose(self.tf_pose_source.lookup())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException, tf2_ros.TimeoutException) as exc:
            now_sec = time.time()
            if (now_sec - self.last_tf_warning) >= 1.0:
                self.last_tf_warning = now_sec
                self._publish_health('warn', 'tf_pose_lookup_failed', {
                    'target_frame': self.config['tf_target_frame'],
                    'source_frame': self.config['tf_source_frame'],
                    'error': str(exc),
                })

    def _current_waypoint(self) -> Optional[Waypoint]:
        if self.route_index < 0 or self.route_index >= len(self.route):
            return None
        return self.route[self.route_index]

    def _start_mission(self):
        self.route_index = 0
        self.retry_count = 0
        self.zone_results = {}
        self.mission_started_at = time.time()
        self.current_zone = ''
        self._emit_state('mission_started', state='DISPATCH_PENDING')

    def _dispatch_waypoint(self, waypoint: Waypoint):
        self.current_zone = waypoint.name
        self.current_zone_pub.publish(String(data=waypoint.name))
        self.dispatch_started_at = time.time()
        self.nav_adapter.dispatch(waypoint)
        self._emit_state('goal_dispatched', state='DISPATCHED', details={'goal': asdict(waypoint), 'retry_count': self.retry_count})

    def _begin_dwell(self, waypoint: Waypoint):
        now_sec = time.time()
        self.capture_aggregator.start(waypoint.name, now_sec, frame_region=waypoint.frame_region)
        self.current_capture_deadline = now_sec + waypoint.dwell_sec
        self._emit_state('capture_window_started', state='DWELL', details={'capture_deadline': self.current_capture_deadline, 'frame_region': waypoint.frame_region})

    def _advance_after_result(self, result: ZoneCaptureResult):
        self.zone_results[result.zone_name] = result.to_dict()
        self._publish_zone_capture(result)
        self.route_index += 1
        self.retry_count = 0
        self.current_zone = ''
        self.current_zone_pub.publish(String(data=''))
        if self.route_index >= len(self.route):
            duration = time.time() - self.mission_started_at
            self._emit_state('mission_finished', state='FINISHED', details={'duration_sec': duration})
        else:
            self._emit_state('goto_next', state='DISPATCH_PENDING', details={'next_zone': self.route[self.route_index].name})

    def _handle_navigation_failure(self, waypoint: Waypoint, reason: str):
        if self.retry_count < self.config['retry_limit']:
            self.retry_count += 1
            self._emit_state('navigation_retry', state='DISPATCH_PENDING', details={'reason': reason, 'retry_count': self.retry_count})
            return
        result = ZoneCaptureResult(
            zone_name=waypoint.name,
            status='navigation_failure',
            friendly=0,
            enemy=0,
            hostage=0,
            capture_started_at=self.dispatch_started_at,
            capture_finished_at=time.time(),
            frame_count=0,
            frame_region=waypoint.frame_region,
            failure_reason=reason,
        )
        self._advance_after_result(result)

    def _poll_navigation(self, waypoint: Waypoint, now_sec: float):
        self._refresh_pose_source()
        status = self.nav_adapter.poll(now_sec)
        dispatch_elapsed = now_sec - self.dispatch_started_at
        if dispatch_elapsed > waypoint.timeout_sec:
            self.nav_adapter.cancel()
            self._handle_navigation_failure(waypoint, 'navigation_timeout')
            return
        if status == 'SUCCEEDED':
            self._begin_dwell(waypoint)
            return
        if status in ('ABORTED', 'PREEMPTED'):
            self._handle_navigation_failure(waypoint, status.lower())
            return
        if status in ('DISPATCHED', 'PENDING', 'ACTIVE'):
            new_state = 'ACTIVE' if status == 'ACTIVE' else 'DISPATCHED'
            if self.state != new_state:
                self._emit_state('navigation_{}'.format(status.lower()), state=new_state, details={'elapsed_sec': dispatch_elapsed})
            return
        self._publish_health('warn', 'navigation_unknown_status', {'status': status})

    def _poll_dwell(self, waypoint: Waypoint, now_sec: float):
        if self.latest_detection_frame is None:
            self._publish_health('warn', 'waiting_for_detections', {'zone': waypoint.name})
        if now_sec < self.current_capture_deadline:
            return
        result = self.capture_aggregator.finalize(status='ok')
        self._advance_after_result(result)

    def step(self):
        now_sec = time.time()
        if self.state == 'IDLE':
            return
        if self.mission_started_at and (now_sec - self.mission_started_at) > self.config['mission_timeout_sec']:
            self.nav_adapter.cancel()
            self._emit_state('mission_timeout', state='TIMEOUT', details={'mission_timeout_sec': self.config['mission_timeout_sec']})
            return
        if self.state in ('FINISHED', 'TIMEOUT', 'FAULT'):
            return
        waypoint = self._current_waypoint()
        if waypoint is None:
            self._emit_state('route_exhausted', state='FINISHED')
            return
        if self.state == 'DISPATCH_PENDING':
            self._dispatch_waypoint(waypoint)
            return
        if self.state in ('DISPATCHED', 'ACTIVE'):
            self._poll_navigation(waypoint, now_sec)
            self._publish_health('ok', 'navigation_running', {
                'adapter': self.config['navigation_adapter_type'],
                'pose_source': self.config['pose_source_type'],
            })
            return
        if self.state == 'DWELL':
            self._poll_dwell(waypoint, now_sec)
            self._publish_health('ok', 'capture_running', {'deadline': self.current_capture_deadline})
            return

    def _on_shutdown(self):
        try:
            self.nav_adapter.cancel()
            self.nav_adapter.close()
        except Exception:
            pass
        self.event_writer.close()

    def spin(self):
        rate = rospy.Rate(self.config['publish_rate_hz'])
        while not rospy.is_shutdown():
            try:
                self.step()
            except Exception as exc:
                rospy.logerr_throttle(2.0, 'mission_manager step failed: %s', exc)
                self._publish_health('error', 'mission_step_failed', {'error': str(exc)})
                self._emit_state('fault', state='FAULT', details={'error': str(exc)})
            rate.sleep()


if __name__ == '__main__':
    try:
        node = MissionManagerNode()
        node.spin()
    except (rospy.ROSInterruptException, ConfigurationError):
        raise
