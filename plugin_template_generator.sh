#!/bin/bash

# Plugin Template Generator Script
# This script creates a new plugin package based on robotnik_pad_plugins template

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
TEMPLATE_DIR="${SCRIPT_DIR}/robotnik_pad_plugins"

echo "=== Plugin Template Generator ==="
echo ""

# Step 1: Ask for plugin name
read -p "Enter the name of the plugin (use underscores, e.g., my_custom_plugin): " PLUGIN_NAME

# Step 2: Validate that the plugin name contains underscores or is a valid format
if [[ ! "$PLUGIN_NAME" =~ ^[a-z][a-z0-9_]*$ ]]; then
    echo -e "${RED}Error: Plugin name must start with a lowercase letter and contain only lowercase letters, numbers, and underscores.${NC}"
    exit 1
fi

if [[ ! "$PLUGIN_NAME" =~ _ ]] && [[ ${#PLUGIN_NAME} -gt 1 ]]; then
    echo -e "${YELLOW}Warning: Plugin name doesn't contain underscores. It's recommended to use underscore_case (e.g., my_plugin).${NC}"
    read -p "Do you want to continue anyway? (y/n): " CONTINUE
    if [[ ! "$CONTINUE" =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 1
    fi
fi

# Step 3: Create package name from plugin name
PACKAGE_NAME="${PLUGIN_NAME}_pad_plugins"

echo ""
echo -e "${GREEN}Plugin name: ${PLUGIN_NAME}${NC}"
echo -e "${GREEN}Package name: ${PACKAGE_NAME}${NC}"
echo ""

# Check if template directory exists
if [ ! -d "$TEMPLATE_DIR" ]; then
    echo -e "${RED}Error: Template directory 'robotnik_pad_plugins' not found at ${TEMPLATE_DIR}${NC}"
    exit 1
fi

# Check if target directory already exists
TARGET_DIR="${SCRIPT_DIR}/${PACKAGE_NAME}"
if [ -d "$TARGET_DIR" ]; then
    echo -e "${RED}Error: Directory '${PACKAGE_NAME}' already exists!${NC}"
    exit 1
fi

# Step 4: Copy the template directory
echo "Copying template directory..."
cp -r "$TEMPLATE_DIR" "$TARGET_DIR"

# Step 5: Rename files from "movement" to plugin name
echo "Renaming files..."

# Convert plugin_name to different formats
# plugin_name -> PluginName (for class names)
PLUGIN_CLASS=$(echo "$PLUGIN_NAME" | sed -E 's/(^|_)([a-z])/\U\2/g')

# Find and rename files
cd "$TARGET_DIR"

# Rename header file
if [ -f "include/robotnik_pad_plugins/movement_plugin.h" ]; then
    mkdir -p "include/${PACKAGE_NAME}"
    mv "include/robotnik_pad_plugins/movement_plugin.h" "include/${PACKAGE_NAME}/${PLUGIN_NAME}_plugin.h"
    # Remove old directory if it's empty
    if [ -d "include/robotnik_pad_plugins" ]; then
        if [ -z "$(ls -A include/robotnik_pad_plugins)" ]; then
            rmdir "include/robotnik_pad_plugins" || true
        fi
    fi
fi

# Rename source file
if [ -f "src/movement_plugin.cpp" ]; then
    mv "src/movement_plugin.cpp" "src/${PLUGIN_NAME}_plugin.cpp"
fi

# Step 6: Replace content in files
echo "Replacing content in files..."

# Files to process
FILES_TO_PROCESS=(
    "package.xml"
    "CMakeLists.txt"
    "robotnik_pad_pluginlib.xml"
    "include/${PACKAGE_NAME}/${PLUGIN_NAME}_plugin.h"
    "src/${PLUGIN_NAME}_plugin.cpp"
)

for file in "${FILES_TO_PROCESS[@]}"; do
    if [ -f "$file" ]; then
        # Replace package name (robotnik_pad_plugins -> new_package_name)
        sed -i "s/robotnik_pad_plugins/${PACKAGE_NAME}/g" "$file"
        
        # Replace movement with plugin name (lowercase)
        sed -i "s/movement/${PLUGIN_NAME}/g" "$file"
        
        # Replace Movement with PluginName (class name format)
        sed -i "s/Movement/${PLUGIN_CLASS}/g" "$file"
        
        # Replace PadPluginMovement with PadPlugin<PluginName>
        sed -i "s/PadPluginMovement/PadPlugin${PLUGIN_CLASS}/g" "$file"
        
        # Fix the header guard
        HEADER_GUARD=$(echo "PAD_PLUGIN_${PLUGIN_NAME}_H" | tr '[:lower:]' '[:upper:]')
        sed -i "s/PAD_PLUGIN_MOVEMENT_H/${HEADER_GUARD}/g" "$file"
    fi
done

# Step 7: Add print statement to the plugin cpp file
echo "Adding print statement to plugin initialization..."
CPP_FILE="src/${PLUGIN_NAME}_plugin.cpp"

if [ -f "$CPP_FILE" ]; then
    # Add a print statement at the end of the initialize function
    # We need to find the specific instance in the initialize function, not in other functions
    # Look for the pattern within the initialize function context
    awk -v plugin_class="$PLUGIN_CLASS" '
        /void.*initialize\(/ { flag=1 }
        flag && /watchdog_activated_ = false;/ {
            print
            print "    RCLCPP_INFO(node_->get_logger(), \"" plugin_class " plugin initialized successfully!\");"
            flag=0
            next
        }
        { print }
    ' "$CPP_FILE" > "${CPP_FILE}.tmp" && mv "${CPP_FILE}.tmp" "$CPP_FILE"
fi

echo ""
echo -e "${GREEN}=== Plugin package created successfully! ===${NC}"
echo ""
echo "Package name: ${PACKAGE_NAME}"
echo "Location: ${TARGET_DIR}"
echo ""
echo "Files created:"
echo "  - ${TARGET_DIR}/package.xml"
echo "  - ${TARGET_DIR}/CMakeLists.txt"
echo "  - ${TARGET_DIR}/include/${PACKAGE_NAME}/${PLUGIN_NAME}_plugin.h"
echo "  - ${TARGET_DIR}/src/${PLUGIN_NAME}_plugin.cpp"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo "  1. Review the generated files"
echo "  2. Customize the plugin implementation in src/${PLUGIN_NAME}_plugin.cpp"
echo "  3. Build the package with: colcon build --packages-select ${PACKAGE_NAME}"
echo ""
