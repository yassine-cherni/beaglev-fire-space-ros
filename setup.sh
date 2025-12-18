#!/bin/bash

set -e

# Download kas-container if not present
if [ ! -f "kas-container" ]; then
    wget https://raw.githubusercontent.com/siemens/kas/5.1/kas-container
    chmod +x kas-container
    echo "kas-container (version 5.1) downloaded and made executable."
else
    echo "kas-container already exists."
fi

# Create persistent build directories
mkdir -p downloads sstate-cache
echo "Persistent directories created: downloads and sstate-cache."

echo "Setup complete! Use ./kas-container shell kas/beaglev-fire-space-ros.yml to enter the build shell."
