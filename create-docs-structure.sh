#!/bin/bash
#
# Script to create the complete documentation structure
# for BeagleV-Fire Space-ROS project
#

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCS_DIR="${PROJECT_ROOT}/docs"

echo "Creating documentation structure..."

# Create docs directory
mkdir -p "${DOCS_DIR}"

# List of documentation files to create
DOC_FILES=(
    "overview.rst"
    "hardware.rst"
    "installation.rst"
    "building.rst"
    "architecture.rst"
    "recipes.rst"
    "customization.rst"
    "fpga.rst"
    "troubleshooting.rst"
    "contributing.rst"
)

# Check which files already exist
echo ""
echo "Documentation Status:"
echo "===================="
for doc in "${DOC_FILES[@]}"; do
    if [ -f "${DOCS_DIR}/${doc}" ]; then
        echo "[EXISTS] ${doc}"
    else
        echo "[MISSING] ${doc}"
    fi
done

echo ""
echo "Documentation structure check complete!"
echo ""
echo "To create missing files, use the artifacts provided."
echo "Place them in: ${DOCS_DIR}/"
echo ""
echo "Directory structure:"
tree -L 2 "${PROJECT_ROOT}" || ls -la "${PROJECT_ROOT}"