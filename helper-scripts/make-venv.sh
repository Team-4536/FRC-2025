#!/bin/bash

# Makes new virtual enviroment and install dependancies.
# This will delete previous venv directory

# TO RUN: `source helper-scripts/make-venv.sh`

rm -rf .venv # delete current virtual enviroment

uv venv # make new virtual enviroment
source .venv/bin/activate # activate venv
uv sync # install dependancies found in pyproject.toml (not src/pyproject.toml)
robotpy --main src sync # install dependancies found in src/pyproject.toml
