"""Runtime path and numeric utility helpers."""

from __future__ import annotations

import os
from pathlib import Path

from .domain_models import ConfigurationError



def expand_path(path: str) -> str:
    return os.path.abspath(os.path.expanduser(str(path)))



def resolve_package_relative_path(path: str, *, package_root: str | None = None) -> str:
    """Resolve a config path relative to the package root when needed.

    Args:
        path: Absolute, home-expanded or package-relative path.
        package_root: Optional explicit package root override.

    Returns:
        Absolute resolved path string.

    Raises:
        No explicit exception is raised.

    Boundary behavior:
        Empty paths resolve to an empty string so callers can keep optional-path
        semantics unchanged.
    """
    normalized = str(path or '').strip()
    if not normalized:
        return ''
    expanded = Path(expand_path(normalized))
    if expanded.is_absolute() and expanded.exists():
        return str(expanded)
    if os.path.isabs(os.path.expanduser(normalized)):
        return str(expanded)
    root = Path(package_root) if package_root else Path(__file__).resolve().parents[2]
    return str((root / normalized).resolve())



def sanitize_ros_namespace(namespace: str) -> str:
    """Normalize a ROS namespace for use in file-system paths.

    Args:
        namespace: ROS namespace such as ``/robot1/team``.

    Returns:
        Namespace without leading or trailing separators and without empty path
        segments.

    Raises:
        No explicit exception is raised.

    Boundary behavior:
        The root namespace resolves to an empty string so callers can choose
        whether to append it to a base output path.
    """
    normalized = str(namespace or '').strip()
    if normalized in ('', '/'):
        return ''
    cleaned = [segment for segment in normalized.split('/') if segment]
    return '/'.join(cleaned)



def resolve_output_root(output_root: str, namespace: str, output_root_use_namespace: bool) -> str:
    """Derive the concrete artifact root for one runtime instance.

    Args:
        output_root: Configured base artifact directory.
        namespace: Active ROS namespace.
        output_root_use_namespace: Whether the ROS namespace should be appended
            to the base directory.

    Returns:
        Absolute artifact root path.

    Raises:
        No explicit exception is raised.

    Boundary behavior:
        Empty or root namespaces never append a path fragment even when
        ``output_root_use_namespace`` is enabled.
    """
    expanded_root = expand_path(output_root)
    if not output_root_use_namespace:
        return expanded_root
    suffix = sanitize_ros_namespace(namespace)
    if not suffix:
        return expanded_root
    return os.path.join(expanded_root, suffix)



def require_positive_float(name: str, value: float, *, allow_zero: bool = False) -> float:
    value = float(value)
    if allow_zero:
        if value < 0.0:
            raise ConfigurationError('{} must be >= 0'.format(name))
    else:
        if value <= 0.0:
            raise ConfigurationError('{} must be > 0'.format(name))
    return value
