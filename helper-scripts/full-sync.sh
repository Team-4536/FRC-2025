#!/bin/bash

# Run this to upgrade both pyproject.toml files in this project
# This script is nessisary to prevent uv from deleting robotpy packages

# TO RUN: `source helper-scripts/full-sync.sh`

uv sync --inexact # sync but do no delete unnesisary packages (eg the robotpy packages)
robotpy --main src sync
