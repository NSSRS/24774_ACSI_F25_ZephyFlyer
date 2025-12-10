#!/bin/bash
# Setup script for Crazyflie environment
# Usage: source setup_env.sh

echo "Setting up Python virtual environment for Crazyflie..."

# Create venv if missing
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
fi

# Activate venv
source .venv/bin/activate

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt

echo "Environment ready. To activate manually later, run:"
echo "source .venv/bin/activate"
