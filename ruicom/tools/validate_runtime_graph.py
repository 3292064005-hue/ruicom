#!/usr/bin/env python3
"""Minimal live ROS graph validator.

This script is intentionally conservative: it validates the presence of expected
nodes/topics/actions without pretending to do full HIL. It is suitable for live
integration smoke runs on a prepared ROS environment.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from ruikang_recon_baseline.runtime_graph import build_runtime_graph_expectations  # noqa: E402


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Validate that a live ROS graph exposes expected nodes/topics/actions')
    parser.add_argument('--timeout-sec', type=float, default=5.0)
    parser.add_argument('--expect-node', action='append', default=[])
    parser.add_argument('--expect-topic', action='append', default=[])
    parser.add_argument('--expect-action', action='append', default=[])
    return parser.parse_args()


def _strip_ns(value: str) -> str:
    return str(value or '').strip().strip('/')


def _wait_for(condition, *, timeout_sec: float, sleep_sec: float = 0.2) -> bool:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if condition():
            return True
        time.sleep(sleep_sec)
    return condition()


def main() -> int:
    args = _parse_args()
    import rospy
    import rosnode
    import rostopic

    rospy.init_node('runtime_graph_validator', anonymous=True)
    expected = build_runtime_graph_expectations(
        enable_vision=False,
        enable_mission=False,
        enable_recorder=False,
        enable_safety=False,
        enable_platform_bridge=False,
    )
    expected['expected_nodes'].extend(_strip_ns(item) for item in args.expect_node)
    expected['expected_topics'].extend(_strip_ns(item) for item in args.expect_topic)
    expected['expected_actions'].extend(_strip_ns(item) for item in args.expect_action)
    expected['expected_nodes'] = sorted(set(item for item in expected['expected_nodes'] if item))
    expected['expected_topics'] = sorted(set(item for item in expected['expected_topics'] if item))
    expected['expected_actions'] = sorted(set(item for item in expected['expected_actions'] if item))

    def current_nodes():
        return {_strip_ns(item) for item in rosnode.get_node_names()}

    def current_topics():
        return {_strip_ns(name) for name, _ in rostopic.get_topic_list()}

    def current_actions():
        topics = current_topics()
        actions = set()
        for topic in topics:
            if topic.endswith('/goal'):
                actions.add(topic[:-5])
        return actions

    missing_nodes = []
    missing_topics = []
    missing_actions = []
    ok = _wait_for(
        lambda: not (
            [name for name in expected['expected_nodes'] if name not in current_nodes()] or
            [name for name in expected['expected_topics'] if name not in current_topics()] or
            [name for name in expected['expected_actions'] if name not in current_actions()]
        ),
        timeout_sec=args.timeout_sec,
    )
    if not ok:
        nodes = current_nodes()
        topics = current_topics()
        actions = current_actions()
        missing_nodes = [name for name in expected['expected_nodes'] if name not in nodes]
        missing_topics = [name for name in expected['expected_topics'] if name not in topics]
        missing_actions = [name for name in expected['expected_actions'] if name not in actions]
        raise SystemExit('missing runtime graph bindings: nodes={} topics={} actions={}'.format(missing_nodes, missing_topics, missing_actions))
    print('runtime graph validated: nodes={} topics={} actions={}'.format(expected['expected_nodes'], expected['expected_topics'], expected['expected_actions']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
