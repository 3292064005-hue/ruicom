"""Pure normalization helpers for navigation status transports."""

from __future__ import annotations

from .common import JsonCodec

CANONICAL_NAV_STATUSES = {'IDLE', 'DISPATCHED', 'PENDING', 'ACTIVE', 'SUCCEEDED', 'PREEMPTED', 'ABORTED'}
_ACTIONLIB_STATUS_MAP = {0: 'PENDING', 1: 'ACTIVE', 2: 'PREEMPTED', 3: 'SUCCEEDED', 4: 'ABORTED', 5: 'ABORTED', 6: 'PREEMPTED', 7: 'PREEMPTED', 8: 'ABORTED', 9: 'ABORTED'}


def normalize_navigation_status_payload(raw_payload: object) -> str:
    """Normalize plain-string or JSON status payloads to canonical states."""
    raw = str(raw_payload or '').strip()
    if not raw:
        return ''
    normalized = raw.upper().replace('NAVIGATION_', '').replace('STATUS_', '')
    if normalized in CANONICAL_NAV_STATUSES:
        return normalized
    if not raw.startswith('{'):
        return ''
    try:
        payload = JsonCodec.loads(raw)
    except Exception:
        return ''
    if not isinstance(payload, dict):
        return ''
    if 'status' in payload:
        direct = str(payload.get('status', '')).strip().upper()
        if direct in CANONICAL_NAV_STATUSES:
            return direct
    status_list = payload.get('status_list', [])
    if not isinstance(status_list, list) or not status_list:
        return ''
    latest = status_list[-1]
    if not isinstance(latest, dict):
        return ''
    code = latest.get('status')
    try:
        code = int(code)
    except Exception:
        code = None
    return _ACTIONLIB_STATUS_MAP.get(code, '')
