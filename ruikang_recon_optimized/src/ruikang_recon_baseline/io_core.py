"""I/O helpers with asynchronous JSONL writing and atomic final artifacts."""

from __future__ import annotations

import json
import os
import queue
import threading
from pathlib import Path
from typing import Any, Dict, Optional

from .common import expand_path


class AsyncJsonlWriter:
    def __init__(self, path: str, max_queue_size: int = 1024, rotate_max_bytes: int = 0, rotate_keep: int = 3):
        self.path = expand_path(path)
        self.max_queue_size = max(1, int(max_queue_size))
        self.rotate_max_bytes = int(rotate_max_bytes)
        self.rotate_keep = max(1, int(rotate_keep))
        Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        self._queue: queue.Queue = queue.Queue(maxsize=self.max_queue_size)
        self._stop_event = threading.Event()
        self._last_error_lock = threading.Lock()
        self._last_error: Optional[str] = None
        self._dropped_messages = 0
        self._thread = threading.Thread(target=self._worker, name='jsonl-writer', daemon=True)
        self._thread.start()

    @property
    def last_error(self) -> Optional[str]:
        with self._last_error_lock:
            return self._last_error

    @property
    def dropped_messages(self) -> int:
        return int(self._dropped_messages)

    def _set_last_error(self, text: str) -> None:
        with self._last_error_lock:
            self._last_error = text

    def _rotate_if_needed(self) -> None:
        if self.rotate_max_bytes <= 0:
            return
        if not os.path.isfile(self.path):
            return
        if os.path.getsize(self.path) < self.rotate_max_bytes:
            return
        for index in range(self.rotate_keep - 1, 0, -1):
            src = '{}.{}'.format(self.path, index)
            dst = '{}.{}'.format(self.path, index + 1)
            if os.path.exists(src):
                os.replace(src, dst)
        os.replace(self.path, '{}.1'.format(self.path))

    def _worker(self) -> None:
        while not self._stop_event.is_set() or not self._queue.empty():
            try:
                payload = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._rotate_if_needed()
                with open(self.path, 'a', encoding='utf-8') as handle:
                    handle.write(json.dumps(payload, ensure_ascii=False) + '\n')
            except Exception as exc:
                self._set_last_error(str(exc))
            finally:
                self._queue.task_done()

    def write(self, payload: Dict[str, Any]) -> None:
        if self._stop_event.is_set():
            return
        try:
            self._queue.put_nowait(payload)
            return
        except queue.Full:
            pass
        try:
            self._queue.get_nowait()
            self._queue.task_done()
            self._dropped_messages += 1
        except queue.Empty:
            pass
        try:
            self._queue.put_nowait({'type': 'writer_overflow', 'dropped_payload': True})
        except queue.Full:
            pass
        try:
            self._queue.put_nowait(payload)
        except queue.Full:
            self._set_last_error('writer queue remained full after overflow handling')

    def close(self, timeout_sec: float = 2.0) -> None:
        self._stop_event.set()
        try:
            self._queue.join()
        except Exception:
            pass
        self._thread.join(timeout=timeout_sec)


def atomic_write_json(path: str, payload: Dict[str, Any]) -> None:
    target = expand_path(path)
    tmp_path = '{}.tmp'.format(target)
    Path(target).parent.mkdir(parents=True, exist_ok=True)
    try:
        with open(tmp_path, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
        os.replace(tmp_path, target)
    except Exception:
        try:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)
        except OSError:
            pass
        raise
