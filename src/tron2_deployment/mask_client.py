"""Client for the prompt-driven segmentation service."""
from __future__ import annotations

import time
import uuid
from typing import Any

import numpy as np
import zmq

from tron2_deployment.mask_protocol import MaskEstimate, decode_response, encode_request


class MaskClient:
    def __init__(self, endpoint: str, *, timeout_ms: int = 120_000,
                 context: zmq.Context | None = None) -> None:
        if timeout_ms <= 0:
            raise ValueError("timeout_ms must be positive")
        self.endpoint = endpoint
        self.timeout_ms = int(timeout_ms)
        self.context = context or zmq.Context.instance()
        self.socket: zmq.Socket | None = None
        self._connect()

    def _connect(self) -> None:
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect(self.endpoint)

    def close(self) -> None:
        if self.socket is not None:
            self.socket.close()
            self.socket = None

    def segment(self, rgb: np.ndarray,
                prompt: dict[str, Any]) -> MaskEstimate:
        request_id = uuid.uuid4().hex
        frames = encode_request(
            rgb, prompt, request_id=request_id, timestamp_s=time.time())
        assert self.socket is not None
        self.socket.send_multipart(frames)
        if not self.socket.poll(self.timeout_ms, zmq.POLLIN):
            self.close()
            self._connect()
            raise TimeoutError(
                f"mask RPC timed out after {self.timeout_ms} ms")
        estimate = decode_response(self.socket.recv_multipart())
        if estimate.request_id != request_id:
            raise RuntimeError("mask response request_id mismatch")
        return estimate

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
