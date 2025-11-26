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
# Using awk for better portability across different systems
PLUGIN_CLASS=$(echo "$PLUGIN_NAME" | awk -F_ '{for(i=1;i<=NF;i++){$i=toupper(substr($i,1,1)) substr($i,2)}}1' OFS="")

# Find and rename files
cd "$TARGET_DIR"

# Rename header file
if [ -f "include/robotnik_pad_plugins/movement_plugin.h" ]; then
    mkdir -p "include/${PACKAGE_NAME}"
    mv "include/robotnik_pad_plugins/movement_plugin.h" "include/${PACKAGE_NAME}/${PLUGIN_NAME}_plugin.h"
    # Remove old directory if it's empty
    if [ -d "include/robotnik_pad_plugins" ]; then
        if [ -z "$(ls -A include/robotnik_pad_plugins)" ]; then
            # Use || true to prevent script termination if rmdir fails due to permissions
            # This is acceptable since the directory will be cleaned up or can be ignored
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

# Portable sed function that works on both Linux and macOS
sed_inplace() {
    local file="$1"
    local pattern="$2"
    sed "$pattern" "$file" > "${file}.tmp" && mv "${file}.tmp" "$file"
}

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
        sed_inplace "$file" "s/robotnik_pad_plugins/${PACKAGE_NAME}/g"
        
        # Replace movement with plugin name (lowercase)
        sed_inplace "$file" "s/movement/${PLUGIN_NAME}/g"
        
        # Replace Movement with PluginName (class name format)
        sed_inplace "$file" "s/Movement/${PLUGIN_CLASS}/g"
        
        # Replace PadPluginMovement with PadPlugin<PluginName>
        sed_inplace "$file" "s/PadPluginMovement/PadPlugin${PLUGIN_CLASS}/g"
        
        # Fix the header guard
        HEADER_GUARD=$(echo "PAD_PLUGIN_${PLUGIN_NAME}_H" | tr '[:lower:]' '[:upper:]')
        sed_inplace "$file" "s/PAD_PLUGIN_MOVEMENT_H/${HEADER_GUARD}/g"
    fi
done

# Step 7: Add print statement to the plugin cpp file
echo "Adding print statement to plugin initialization..."
CPP_FILE="src/${PLUGIN_NAME}_plugin.cpp"

if [ -f "$CPP_FILE" ]; then
    # Add a print statement at the end of the initialize function
    # We need to find the specific instance in the initialize function, not in other functions
    # Track brace depth to properly handle nested scopes
    awk -v plugin_class="$PLUGIN_CLASS" '
        /void.*initialize\(/ { 
            in_init=1
            brace_depth=0
            print
            next
        }
        in_init && /\{/ { brace_depth++ }
        in_init && /\}/ { 
            brace_depth--
            if (brace_depth == 0) {
                in_init=0
            }
        }
        in_init && /watchdog_activated_ = false;/ && !found {
            print
            print "    RCLCPP_INFO(node_->get_logger(), \"" plugin_class " plugin initialized successfully!\");"
            found=1
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
