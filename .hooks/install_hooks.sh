#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
chmod +x $SCRIPT_DIR/commit-msg.py

pre-commit autoupdate
pre-commit install
pre-commit install --hook-type commit-msg
