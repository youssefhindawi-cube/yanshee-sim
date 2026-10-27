"""Web GUI for the relay controller: FastAPI server (server.py) and the static page (static/).

    python3 scripts/web_gui.py          # or: from yanshee_sim.web import create_app, run
"""
from .server import create_app, make_server, run

__all__ = ["create_app", "make_server", "run"]
