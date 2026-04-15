#!/usr/bin/env python3
"""Fail-fast guard for external vendor driver launch arguments."""
from __future__ import annotations

from pathlib import Path
import rospy


class VendorLaunchGuardNode:
    def __init__(self) -> None:
        rospy.init_node('vendor_launch_guard_node', anonymous=False)
        self.guard_name = str(rospy.get_param('~guard_name', 'vendor_launch')).strip() or 'vendor_launch'
        self.required_path = str(rospy.get_param('~required_path', '')).strip()
        self.require_non_empty = bool(rospy.get_param('~require_non_empty', False))
        self.must_exist = bool(rospy.get_param('~must_exist', False))
        self.reject_repo_paths = bool(rospy.get_param('~reject_repo_paths', False))
        self.repo_root = Path(str(rospy.get_param('~repo_root', '')).strip()).resolve() if str(rospy.get_param('~repo_root', '')).strip() else None
        self._validate()

    def _validate(self) -> None:
        if self.require_non_empty and not self.required_path:
            raise rospy.ROSInitException('%s requires a non-empty external launch path' % self.guard_name)
        if not self.required_path:
            return
        if '$(' in self.required_path:
            return
        target = Path(self.required_path).expanduser().resolve()
        if self.must_exist and not target.exists():
            raise rospy.ROSInitException('%s target does not exist: %s' % (self.guard_name, target))
        if self.reject_repo_paths and self.repo_root is not None and self.repo_root in target.parents:
            raise rospy.ROSInitException('%s must not point back into repository assets: %s' % (self.guard_name, target))

    def spin(self) -> None:
        rospy.spin()


if __name__ == '__main__':
    try:
        VendorLaunchGuardNode().spin()
    except rospy.ROSInterruptException:
        pass
