# Copyright (c) 2026 Vivek Raja. Original component.
# Licensed under PolyForm Noncommercial License 1.0.0 (see LICENSE-NONCOMMERCIAL.md).
# Commercial use requires the author's written permission. Attribution required (see NOTICE).
"""Tiny web UI to drive the Curiosity rover with natural language.

Serves ``index.html`` and a ``POST /prompt`` endpoint that runs the
strands-robots-ros2 agent (``Robot("curiosity")`` -> RosbridgeRuntime -> rover).
Run inside an environment that can reach the rosbridge WebSocket.

    pip install "strands-robots-ros2[rosbridge]" flask
    ROS_HOST=localhost PORT=5000 python server.py
    # open http://localhost:5000
"""

import os
import threading

from flask import Flask, jsonify, request, send_from_directory

from strands import Agent
from strands.models.bedrock import BedrockModel

from strands_robots_ros2 import Robot

_HERE = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__, static_folder=_HERE)
_lock = threading.Lock()

# One persistent rover + agent (the RosbridgeRuntime connects on first move).
_rover = Robot(
    "curiosity",
    robot="curiosity",
    host=os.environ.get("ROS_HOST", "localhost"),
    port=int(os.environ.get("ROS_PORT", "9090")),
)
_agent = Agent(
    model=BedrockModel(model_id=os.environ.get("MODEL", "us.amazon.nova-lite-v1:0")),
    tools=[_rover],
)


@app.route("/")
def index():
    return send_from_directory(_HERE, "index.html")


@app.route("/prompt", methods=["POST"])
def prompt():
    text = ((request.get_json(silent=True) or {}).get("prompt") or "").strip()
    if not text:
        return jsonify({"error": "empty prompt"}), 400
    # Serialize — a single rover, one command at a time.
    with _lock:
        try:
            result = _agent(text)
            return jsonify({"response": str(result)})
        except Exception as exc:  # noqa: BLE001
            return jsonify({"error": str(exc)}), 500


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=int(os.environ.get("PORT", "5000")), threaded=True)
