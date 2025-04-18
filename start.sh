#!/bin/bash

# Run flask app
export FLASK_APP=./checkin/app.py
./.venv/bin/python -m flask run --host=0.0.0.0 --port=5002