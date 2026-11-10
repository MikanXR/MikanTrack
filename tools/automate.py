#!/usr/bin/env python3
"""Client for the MikanTrack automation command server.

Connects to the loopback automation port of a running MikanTrack.exe, sends
each positional argument as one command, and prints each framed reply.

The server is off by default. Launch the app with --automation-server (or
--automation-port=<n>, which implies it).

    python tools/automate.py "app info"
    python tools/automate.py "app resume" "until:app info:state Project" "screenshot window"
    python tools/automate.py "gui viewports" "gui float \\"Stage View\\" 2000 100"

Client-side arguments (not sent to the server):
    ready                        wait until the server answers "app info"
    sleep:<s>                    pause that many seconds instead of the default delay
    until:<command>:<expected>   poll the command until a reply line starts with
                                 the expected text (fails after --timeout seconds)

Exits nonzero if the connection cannot be opened, a reply times out, or an
until condition never becomes true.
"""

import argparse
import socket
import sys
import time

DEFAULT_PORT = 21140
CONNECT_RETRY_SECONDS = 0.25


class AutomationClient:
    def __init__(self, port, connect_timeout, reply_timeout):
        self.reply_timeout = reply_timeout
        deadline = time.monotonic() + connect_timeout
        while True:
            try:
                self.sock = socket.create_connection(("127.0.0.1", port), timeout=reply_timeout)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise SystemExit(
                        f"error: could not connect to 127.0.0.1:{port}\n"
                        "the automation server is off by default: launch MikanTrack.exe with "
                        "--automation-server"
                    )
                time.sleep(CONNECT_RETRY_SECONDS)
        self.buffer = b""

    def read_line(self):
        deadline = time.monotonic() + self.reply_timeout
        while b"\n" not in self.buffer:
            if time.monotonic() >= deadline:
                raise SystemExit("error: timed out waiting for a reply line")
            try:
                chunk = self.sock.recv(4096)
            except TimeoutError:
                raise SystemExit("error: timed out waiting for a reply line")
            except OSError:
                # A reset, or an abort when the app closes the listener
                raise SystemExit("error: server closed the connection")
            if not chunk:
                raise SystemExit("error: server closed the connection")
            self.buffer += chunk
        line, self.buffer = self.buffer.split(b"\n", 1)
        return line.decode("utf-8", errors="replace").rstrip("\r")

    def send_command(self, command):
        """Send one command and return the reply's content lines."""
        self.sock.sendall(command.encode("utf-8") + b"\n")
        count_line = self.read_line()
        try:
            count = int(count_line)
        except ValueError:
            raise SystemExit(f"error: expected a reply count line, got '{count_line}'")
        return [self.read_line() for _ in range(count)]


def main():
    parser = argparse.ArgumentParser(description="Drive the MikanTrack automation command server.")
    parser.add_argument("commands", nargs="+", help="commands to send, in order")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--wait", type=float, default=20.0, help="seconds to retry the initial connect")
    parser.add_argument("--timeout", type=float, default=60.0, help="seconds to wait for each reply")
    parser.add_argument("--delay", type=float, default=0.1, help="seconds to pause between commands")
    args = parser.parse_args()

    client = AutomationClient(args.port, args.wait, args.timeout)

    for index, command in enumerate(args.commands):
        if index > 0:
            time.sleep(args.delay)

        if command.startswith("sleep:"):
            time.sleep(float(command.split(":", 1)[1]))
            continue

        if command.startswith("until:"):
            _, poll_command, expected = command.split(":", 2)
            print(f"> until '{poll_command}' answers '{expected}'")
            deadline = time.monotonic() + args.timeout
            while True:
                lines = client.send_command(poll_command)
                if any(line.startswith(expected) for line in lines):
                    for line in lines:
                        print(line)
                    break
                if time.monotonic() >= deadline:
                    raise SystemExit(f"error: '{poll_command}' never answered '{expected}'")
                time.sleep(0.25)
            continue

        if command == "ready":
            command = "app info"

        print(f"> {command}")
        for line in client.send_command(command):
            print(line)


if __name__ == "__main__":
    main()
