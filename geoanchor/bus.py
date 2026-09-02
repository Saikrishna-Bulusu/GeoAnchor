"""ZeroMQ transport. The only thing the three layers share.

Why a broker-less bus and not multiprocessing queues: the requirement is that
the layers do not affect each other. Queue children die with their parent.
Separate processes on PUB/SUB do not -- kill the processing layer and the data
layer keeps capturing, the output layer keeps logging, and both say so.

Two habits worth knowing:

* PUB drops rather than blocks. A slow subscriber loses frames instead of
  stalling the camera, which is the right trade at a 250 ms budget.
* PUB/SUB has a slow-joiner problem: anything sent before a subscriber has
  finished connecting is gone. Long-lived facts (the map packet, status) are
  therefore republished on a timer, never sent once.
"""
from __future__ import annotations

import json
import os
import time
from typing import Any, Iterable

import zmq

_CTX: zmq.Context | None = None


def context() -> zmq.Context:
    global _CTX
    if _CTX is None:
        _CTX = zmq.Context.instance()
    return _CTX


class BindError(RuntimeError):
    pass


class Publisher:
    """One per layer. Binds; subscribers connect to it."""

    def __init__(self, endpoint: str, sndhwm: int = 8):
        self.endpoint = endpoint
        self.sock = context().socket(zmq.PUB)
        self.sock.setsockopt(zmq.SNDHWM, sndhwm)
        self.sock.setsockopt(zmq.LINGER, 200)
        _prepare_ipc(endpoint)
        try:
            self.sock.bind(endpoint)
        except zmq.ZMQError as exc:
            raise BindError(f"{endpoint}: {exc}") from exc
        # Give SUBs a moment to attach before the first send.
        time.sleep(0.15)

    def send(self, topic: str, header: dict[str, Any], payload: bytes | None = None) -> bool:
        """Returns False when the message was dropped because a subscriber is
        behind. Dropping is the designed behaviour at a latency budget, but it
        has to be visible -- a silently thinning frame stream looks like a slow
        camera."""
        parts = [topic.encode(), json.dumps(header, default=_json_default).encode()]
        if payload is not None:
            parts.append(payload)
        try:
            self.sock.send_multipart(parts, flags=zmq.NOBLOCK)
            return True
        except zmq.Again:
            return False

    def close(self) -> None:
        self.sock.close(linger=200)


class Subscriber:
    """Connects to one or more publishers and filters by topic."""

    def __init__(self, endpoints: Iterable[str], topics: Iterable[str], rcvhwm: int = 8):
        self.sock = context().socket(zmq.SUB)
        self.sock.setsockopt(zmq.RCVHWM, rcvhwm)
        self.sock.setsockopt(zmq.LINGER, 0)
        for t in topics:
            self.sock.setsockopt(zmq.SUBSCRIBE, t.encode())
        self.endpoints = list(endpoints)
        for ep in self.endpoints:
            self.sock.connect(ep)
        self.poller = zmq.Poller()
        self.poller.register(self.sock, zmq.POLLIN)

    def recv(self, timeout_ms: int = 100):
        """Return (topic, header, payload) or None on timeout."""
        if not self.poller.poll(timeout_ms):
            return None
        return self._recv_now()

    def _recv_now(self):
        parts = self.sock.recv_multipart()
        topic = parts[0].decode()
        header = json.loads(parts[1].decode())
        payload = parts[2] if len(parts) > 2 else None
        return topic, header, payload

    def drain(self, timeout_ms: int = 100, keep_latest_of: Iterable[str] = ()):
        """Read everything queued, collapsing the named topics to their newest.

        This is how the processing layer avoids working through a backlog of
        stale frames: it always matches the freshest one and reports the rest
        as skipped, rather than falling further behind on every iteration.
        """
        latest = set(keep_latest_of)
        first = self.recv(timeout_ms)
        if first is None:
            return [], 0
        out: list[tuple] = [first]
        dropped = 0
        while True:
            if not self.poller.poll(0):
                break
            msg = self._recv_now()
            if msg[0] in latest:
                for i, existing in enumerate(out):
                    if existing[0] == msg[0]:
                        out[i] = msg
                        dropped += 1
                        break
                else:
                    out.append(msg)
            else:
                out.append(msg)
        return out, dropped

    def close(self) -> None:
        self.sock.close(linger=0)


class CommandServer:
    """Fire-and-forget control input. PULL, so it can never deadlock a layer.

    Acknowledgement happens out of band: a layer echoes its applied
    configuration in every status heartbeat, and the dashboard shows that,
    not what it asked for.
    """

    def __init__(self, endpoint: str):
        self.sock = context().socket(zmq.PULL)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.setsockopt(zmq.RCVHWM, 16)
        _prepare_ipc(endpoint)
        try:
            self.sock.bind(endpoint)
        except zmq.ZMQError as exc:
            raise BindError(f"{endpoint}: {exc}") from exc

    def poll(self) -> list[dict]:
        out = []
        while True:
            try:
                out.append(json.loads(self.sock.recv(flags=zmq.NOBLOCK).decode()))
            except zmq.Again:
                return out
            except (ValueError, UnicodeDecodeError):
                continue

    def close(self) -> None:
        self.sock.close(linger=0)


class CommandClient:
    def __init__(self, endpoint: str):
        self.sock = context().socket(zmq.PUSH)
        self.sock.setsockopt(zmq.LINGER, 200)
        self.sock.setsockopt(zmq.SNDHWM, 16)
        self.sock.connect(endpoint)

    def send(self, msg: dict) -> bool:
        try:
            self.sock.send(json.dumps(msg).encode(), flags=zmq.NOBLOCK)
            return True
        except zmq.Again:
            return False

    def close(self) -> None:
        self.sock.close(linger=200)


def _prepare_ipc(endpoint: str) -> None:
    if endpoint.startswith("ipc://"):
        os.makedirs(os.path.dirname(endpoint[6:]) or ".", exist_ok=True)


def _json_default(o):
    if hasattr(o, "tolist"):
        return o.tolist()
    if hasattr(o, "item"):
        return o.item()
    return str(o)
