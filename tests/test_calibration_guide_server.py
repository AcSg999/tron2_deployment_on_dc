"""The helper serves passive state and allows only explicit local actions."""
from http.client import HTTPConnection
import json
import threading

import pytest

from tron2_deployment.calibration_guide_server import make_server


class GuideStub:
    def __init__(self):
        self.calls = []

    def status(self):
        return {"saved_count": len(self.calls)}

    def preview(self):
        self.calls.append("preview")
        return self.status()

    def save(self):
        self.calls.append("save")
        return self.status()

    def solve(self):
        raise ValueError("Collect more varied views first")


@pytest.fixture
def local_server():
    guide = GuideStub()
    server = make_server(guide, port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield guide, server
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def request(server, method, path, *, headers=None, body=None):
    connection = HTTPConnection(*server.server_address, timeout=3)
    try:
        connection.request(method, path, body=body, headers=headers or {})
        response = connection.getresponse()
        return response.status, response.read(), response.headers
    finally:
        connection.close()


def test_page_and_status_do_not_capture(local_server):
    guide, server = local_server
    for path in ("/", "/calibration.js", "/calibration.css", "/api/status"):
        status, content, headers = request(server, "GET", path)
        assert status == 200 and content
        assert headers["Cache-Control"] == "no-store"
    assert guide.calls == []
    assert request(server, "GET", "/api/save")[0] == 404


@pytest.mark.parametrize("headers,body", [
    ({"Content-Type": "application/json", "Origin": "https://elsewhere.example"}, "{}"),
    ({"Content-Type": "application/json", "Host": "elsewhere.example"}, "{}"),
    ({"Content-Type": "text/plain"}, "{}"),
    ({"Content-Type": "application/json"}, '{"stage":"other"}'),
    ({"Content-Type": "application/json"}, "{}" + " " * 1024),
])
def test_requests_cannot_override_or_cross_site_trigger_capture(local_server, headers, body):
    guide, server = local_server
    assert request(server, "POST", "/api/save", headers=headers, body=body)[0] in (400, 403)
    assert guide.calls == []


def test_explicit_actions_and_actionable_error(local_server):
    guide, server = local_server
    headers = {"Content-Type": "application/json"}
    assert request(server, "POST", "/api/preview", headers=headers, body="{}")[0] == 200
    assert request(server, "POST", "/api/save", headers=headers, body="{}")[0] == 200
    status, body, _ = request(server, "POST", "/api/solve", headers=headers, body="{}")
    assert status == 400 and "more varied" in json.loads(body)["error"]
    assert guide.calls == ["preview", "save"]
