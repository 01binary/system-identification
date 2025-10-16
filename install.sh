#!/bin/bash
# Installation script for ARX System Identification

echo "Installing required packages..."
python3 -m pip install --break-system-packages -r requirements.txt

echo "Installation complete! You can now run:"
echo "./system-identification.py"
