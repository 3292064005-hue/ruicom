"""Contract-driven official report submission adapters.

The runtime deliberately avoids baking in a single proprietary judge protocol.
Instead, ``judge_contract`` mode now loads an explicit submission contract and
executes one of a small set of audited transport adapters. This creates a real
submission boundary with receipts and failure semantics, rather than merely
writing a richer JSON file to an arbitrary path.
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import time
import urllib.error
import urllib.request
from typing import Any, Dict, Mapping

from .common import ConfigurationError, expand_path
from .io_core import atomic_write_json

VALID_SUBMISSION_ADAPTER_TYPES = ('file_drop', 'http_post', 'command_exec')


def _canonical_json_bytes(payload: Mapping[str, Any]) -> bytes:
    return json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(',', ':')).encode('utf-8')


def load_submission_contract(path: str) -> Dict[str, Any]:
    target = expand_path(path)
    if not target:
        raise ConfigurationError('official_report_contract_path must not be empty')
    if not os.path.isfile(target):
        raise ConfigurationError('official_report_contract_path does not exist: {}'.format(target))
    try:
        with open(target, 'r', encoding='utf-8') as handle:
            payload = json.load(handle)
    except Exception as exc:
        raise ConfigurationError('failed to load official report contract {}: {}'.format(target, exc))
    if not isinstance(payload, dict):
        raise ConfigurationError('official report contract must be a JSON object: {}'.format(target))
    payload['_contract_path'] = target
    return payload


def validate_submission_contract(payload: Mapping[str, Any]) -> Dict[str, Any]:
    if not isinstance(payload, Mapping):
        raise ConfigurationError('official report submission contract must be a mapping')
    adapter_type = str(payload.get('adapter_type', '')).strip().lower()
    if adapter_type not in VALID_SUBMISSION_ADAPTER_TYPES:
        raise ConfigurationError('submission adapter_type must be one of {}'.format(', '.join(VALID_SUBMISSION_ADAPTER_TYPES)))
    contract_path = str(payload.get('_contract_path', '')).strip()
    normalized: Dict[str, Any] = {
        'adapter_type': adapter_type,
        'contract_path': contract_path,
        'receipt_path': str(payload.get('receipt_path', '')).strip(),
        'timeout_sec': float(payload.get('timeout_sec', 5.0) or 5.0),
    }
    if normalized['timeout_sec'] <= 0.0:
        raise ConfigurationError('submission contract timeout_sec must be > 0')
    if adapter_type == 'file_drop':
        endpoint_path = str(payload.get('endpoint_path', '')).strip()
        if not endpoint_path:
            raise ConfigurationError('file_drop submission contract requires endpoint_path')
        normalized['endpoint_path'] = expand_path(endpoint_path)
    elif adapter_type == 'http_post':
        endpoint_url = str(payload.get('endpoint_url', '')).strip()
        if not endpoint_url.startswith(('http://', 'https://')):
            raise ConfigurationError('http_post submission contract requires endpoint_url starting with http:// or https://')
        headers = payload.get('headers', {}) or {}
        if not isinstance(headers, Mapping):
            raise ConfigurationError('http_post submission contract headers must be a mapping')
        success_status_codes = payload.get('success_status_codes', [200, 201, 202, 204])
        if not isinstance(success_status_codes, list) or not success_status_codes:
            raise ConfigurationError('http_post submission contract success_status_codes must be a non-empty list')
        normalized['endpoint_url'] = endpoint_url
        normalized['headers'] = {str(key).strip(): str(value) for key, value in dict(headers).items() if str(key).strip()}
        normalized['success_status_codes'] = [int(item) for item in success_status_codes]
        normalized['token_env_var'] = str(payload.get('token_env_var', '')).strip()
    elif adapter_type == 'command_exec':
        argv = payload.get('argv', None)
        if not isinstance(argv, list) or not argv:
            raise ConfigurationError('command_exec submission contract requires argv list')
        normalized['argv'] = [str(item) for item in argv if str(item).strip()]
        if not normalized['argv']:
            raise ConfigurationError('command_exec submission contract argv must not be empty')
        stdin_mode = str(payload.get('stdin_mode', 'json')).strip().lower() or 'json'
        if stdin_mode not in ('json', 'none'):
            raise ConfigurationError('command_exec submission contract stdin_mode must be json or none')
        extra_env = payload.get('extra_env', {}) or {}
        if not isinstance(extra_env, Mapping):
            raise ConfigurationError('command_exec submission contract extra_env must be a mapping')
        normalized['stdin_mode'] = stdin_mode
        normalized['extra_env'] = {str(key).strip(): str(value) for key, value in dict(extra_env).items() if str(key).strip()}
    return normalized


def load_and_validate_submission_contract(path: str) -> Dict[str, Any]:
    return validate_submission_contract(load_submission_contract(path))


class OfficialReportSubmissionAdapter:
    def __init__(self, contract: Mapping[str, Any]):
        self.contract = dict(contract)

    def submit(self, report: Mapping[str, Any], *, report_path: str, submission_id: str) -> Dict[str, Any]:
        raise NotImplementedError


class FileDropSubmissionAdapter(OfficialReportSubmissionAdapter):
    def submit(self, report: Mapping[str, Any], *, report_path: str, submission_id: str) -> Dict[str, Any]:
        endpoint_path = str(self.contract['endpoint_path']).strip()
        atomic_write_json(endpoint_path, dict(report))
        payload_bytes = _canonical_json_bytes(report)
        return {
            'adapter_type': 'file_drop',
            'status': 'submitted',
            'submitted_at_wall': time.time(),
            'endpoint_path': endpoint_path,
            'report_path': expand_path(report_path),
            'submission_id': str(submission_id).strip(),
            'report_sha256': hashlib.sha256(payload_bytes).hexdigest(),
            'report_size_bytes': len(payload_bytes),
        }


class HttpPostSubmissionAdapter(OfficialReportSubmissionAdapter):
    def submit(self, report: Mapping[str, Any], *, report_path: str, submission_id: str) -> Dict[str, Any]:
        body = _canonical_json_bytes(report)
        headers = {'Content-Type': 'application/json'}
        headers.update(dict(self.contract.get('headers', {}) or {}))
        token_env_var = str(self.contract.get('token_env_var', '')).strip()
        if token_env_var:
            token = str(os.environ.get(token_env_var, '')).strip()
            if not token:
                raise ConfigurationError('submission contract token_env_var {} is not set'.format(token_env_var))
            headers.setdefault('Authorization', 'Bearer {}'.format(token))
        if submission_id:
            headers.setdefault('X-Submission-ID', str(submission_id).strip())
        request = urllib.request.Request(
            str(self.contract['endpoint_url']).strip(),
            data=body,
            headers=headers,
            method='POST',
        )
        started = time.time()
        try:
            with urllib.request.urlopen(request, timeout=float(self.contract.get('timeout_sec', 5.0))) as response:
                response_body = response.read()
                status_code = int(getattr(response, 'status', response.getcode()))
                response_headers = dict(getattr(response, 'headers', {}))
        except urllib.error.HTTPError as exc:
            response_body = exc.read() if hasattr(exc, 'read') else b''
            raise ConfigurationError('http_post submission failed with status {}: {}'.format(exc.code, response_body[:512].decode('utf-8', errors='replace')))
        except Exception as exc:
            raise ConfigurationError('http_post submission failed: {}'.format(exc))
        if status_code not in set(int(item) for item in self.contract.get('success_status_codes', [200, 201, 202, 204])):
            raise ConfigurationError('http_post submission returned unexpected status {}'.format(status_code))
        response_text = response_body.decode('utf-8', errors='replace')
        try:
            response_payload = json.loads(response_text) if response_text else {}
        except Exception:
            response_payload = {'raw_text': response_text}
        return {
            'adapter_type': 'http_post',
            'status': 'submitted',
            'submitted_at_wall': time.time(),
            'duration_sec': max(0.0, time.time() - started),
            'endpoint_url': str(self.contract['endpoint_url']).strip(),
            'report_path': expand_path(report_path),
            'submission_id': str(submission_id).strip(),
            'http_status': status_code,
            'response_headers': response_headers,
            'response_payload': response_payload,
        }


class CommandExecSubmissionAdapter(OfficialReportSubmissionAdapter):
    def submit(self, report: Mapping[str, Any], *, report_path: str, submission_id: str) -> Dict[str, Any]:
        argv = list(self.contract.get('argv', []) or [])
        env = os.environ.copy()
        env.update(dict(self.contract.get('extra_env', {}) or {}))
        env['RECON_REPORT_PATH'] = expand_path(report_path)
        env['RECON_SUBMISSION_ID'] = str(submission_id).strip()
        body = _canonical_json_bytes(report)
        started = time.time()
        try:
            completed = subprocess.run(
                argv,
                input=body if self.contract.get('stdin_mode', 'json') == 'json' else None,
                capture_output=True,
                timeout=float(self.contract.get('timeout_sec', 5.0)),
                env=env,
                check=False,
            )
        except Exception as exc:
            raise ConfigurationError('command_exec submission failed: {}'.format(exc))
        stdout_text = completed.stdout.decode('utf-8', errors='replace')[-4096:]
        stderr_text = completed.stderr.decode('utf-8', errors='replace')[-4096:]
        if completed.returncode != 0:
            raise ConfigurationError('command_exec submission failed rc={} stderr={}'.format(completed.returncode, stderr_text))
        return {
            'adapter_type': 'command_exec',
            'status': 'submitted',
            'submitted_at_wall': time.time(),
            'duration_sec': max(0.0, time.time() - started),
            'argv': argv,
            'report_path': expand_path(report_path),
            'submission_id': str(submission_id).strip(),
            'returncode': int(completed.returncode),
            'stdout_tail': stdout_text,
            'stderr_tail': stderr_text,
        }


def build_submission_adapter(contract: Mapping[str, Any]) -> OfficialReportSubmissionAdapter:
    adapter_type = str(contract.get('adapter_type', '')).strip().lower()
    if adapter_type == 'file_drop':
        return FileDropSubmissionAdapter(contract)
    if adapter_type == 'http_post':
        return HttpPostSubmissionAdapter(contract)
    if adapter_type == 'command_exec':
        return CommandExecSubmissionAdapter(contract)
    raise ConfigurationError('unsupported submission adapter_type {}'.format(adapter_type))


def submit_official_report(contract: Mapping[str, Any], report: Mapping[str, Any], *, report_path: str, submission_id: str) -> Dict[str, Any]:
    normalized = validate_submission_contract(contract)
    adapter = build_submission_adapter(normalized)
    return adapter.submit(report, report_path=report_path, submission_id=submission_id)
