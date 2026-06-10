#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

echo "=== Starting Environment Automation for Operation Touchdown ==="

# 1. Clear any broken dpkg locks or interrupted installations
echo "--> Checking and repairing system package locks..."
sudo dpkg --configure -a || true
sudo rm -f /var/lib/dpkg/lock-frontend /var/lib/dpkg/lock

# 2. Update package repositories and install system requirements
echo "--> Installing Ubuntu system dependencies (Python venv and Pip)..."
sudo apt update
sudo apt install python3.10-venv python3-pip python3-dev build-essential cmake -y
# 3. Clean up older virtual environment if it exists
if [ -d "venv" ]; then
    echo "--> Found existing venv folder. Removing to ensure clean setup..."
    rm -rf venv
fi

# 4. Create a fresh virtual environment
echo "--> Creating fresh virtual environment (venv)..."
python3 -m venv venv

# 5. Activate the environment and upgrade pip inside it
echo "--> Activating virtual environment and upgrading pip..."
source venv/bin/activate
pip install --upgrade pip

# 6. Install exact versions from requirements.txt
if [ -f "requirements.txt" ]; then
    echo "--> Installing specific Python dependencies from requirements.txt..."
    pip install -r requirements.txt
    echo "=== Environment Setup Completed Successfully! ==="
    echo "To enter your environment, run: source venv/bin/activate"
else
    echo "Error: requirements.txt not found. Skipping library installation."
    exit 1
fi