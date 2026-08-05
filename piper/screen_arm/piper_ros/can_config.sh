#!/bin/bash

# Usage Instructions:

# 1. Prerequisites
#     Requires the installation of ip and ethtool tools in the system.
#     sudo apt install ethtool can-utils
#     Ensure that the gs_usb driver is correctly installed.

# 2. Background
#  This script aims to automatically manage and rename and activate CAN (Controller Area Network) interfaces.
#  It checks the number of CAN modules currently in the system and renames and activates CAN interfaces based on predefined USB ports.
#  This is particularly useful for systems with multiple CAN modules, especially when different CAN modules require specific names.

# 3. Main Functions
#  Check CAN module count: Ensure the number of CAN modules detected in the system matches the preset number.
#  Get USB port information: Retrieve USB port information for each CAN module using ethtool.
#  Validate USB ports: Check if the USB port for each CAN module matches the predefined port list.
#  Rename CAN interfaces: Rename CAN interfaces to target names based on predefined USB ports.

# 4. Script Configuration Instructions
#   Key configuration items in the script include the expected number of CAN modules, default CAN interface names, and baud rate settings:
#   1. Expected number of CAN modules:
#     EXPECTED_CAN_COUNT=1
#     This value determines the number of CAN modules that should be detected in the system.
#   2. Default CAN interface name for a single CAN module:
#     DEFAULT_CAN_NAME="${1:-can0}"
#     Can specify the default CAN interface name via command-line arguments, defaulting to can0 if no argument is provided.
#   3. Default bitrate for a single CAN module:
#     DEFAULT_BITRATE="${2:-500000}"
#     Can specify the bitrate via command-line arguments, defaulting to 500000 if no argument is provided.
#   4. Configuration for multiple CAN modules:
#     declare -A USB_PORTS
#     USB_PORTS["1-2:1.0"]="can_device_1:500000"
#     USB_PORTS["1-3:1.0"]="can_device_2:250000"
#     The keys represent USB ports, and values are interface names and bitrates, separated by a colon.

# 5. Usage Steps
#  1. Edit the script:
#   1. Modify predefined values:
#      - Predefined number of CAN modules: EXPECTED_CAN_COUNT=2, can be modified to the number of CAN modules inserted in the industrial computer
#      - If there's only one CAN module, set the parameters and skip to the next steps
#      - (For multiple CAN modules) Predefined USB ports and target interface names:
#          First, insert a CAN module into the expected USB port. Note that when initially configuring, insert one CAN module at a time in the industrial computer
#          Then execute `sudo ethtool -i can0 | grep bus`, and record the parameter after bus-info:
#          Then insert the next CAN module, ensuring it's not in the same USB port as the previous module, and repeat the previous step
#          (Actually, you can use one CAN module to plug into different USB ports, as module distinction is based on USB address)
#          After designing and recording the USB ports for all modules,
#          modify the USB ports (bus-info) and target interface names according to the actual situation.
#          can_device_1:500000, the former is the set CAN name, the latter is the set bitrate
#            declare -A USB_PORTS
#            USB_PORTS["1-2:1.0"]="can_device_1:500000"
#            USB_PORTS["1-3:1.0"]="can_device_2:250000"
#          What needs to be modified is the content within the double quotes of USB_PORTS["1-3:1.0"], changed to the parameter after bus-info:
#   2. Grant script execution permissions:
#       Open terminal, navigate to the script directory, execute the following command to grant execution permissions:
#       chmod +x can_config.sh
#   3. Run the script:
#     Execute the script with sudo, as it requires administrator permissions to modify network interfaces:
#       1. Single CAN module
#         1. Can specify default CAN interface name and bitrate via command-line arguments (default is can0 and 500000):
#           sudo bash ./can_config.sh [CAN interface name] [bitrate]
#           For example, specify interface name as my_can_interface, bitrate as 1000000:
#           sudo bash ./can_config.sh my_can_interface 1000000
#         2. Can specify CAN name by USB hardware address
#           sudo bash ./can_config.sh [CAN interface name] [bitrate] [USB hardware address]
#           For example, specify interface name as my_can_interface, bitrate as 1000000, USB hardware address as 1-3:1.0:
#           sudo bash ./can_config.sh my_can_interface 1000000 1-3:1.0
#           This means the CAN device at USB address 1-3:1.0 is named my_can_interface with bitrate 1000000
#       2. Multiple CAN modules
#         For multiple CAN modules, specify each CAN module's interface name and bitrate by setting the USB_PORTS array in the script.
#         No additional parameters needed, just run the script:
#         sudo ./can_config.sh

# Notes

#     Permission Requirements:
#         The script requires sudo permissions because renaming and setting network interfaces need administrator rights.
#         Ensure you have sufficient permissions to run the script.

#     Script Environment:
#         This script assumes running in a bash environment. Ensure your system uses bash, not other shells (like sh).
#         You can verify this by checking the script's Shebang line (#!/bin/bash).

#     USB Port Information:
#         Ensure the predefined USB port information (bus-info) matches the actual information output by ethtool in the system.
#         Use commands like sudo ethtool -i can0, sudo ethtool -i can1 to check bus-info for each CAN interface.

#     Interface Conflicts:
#         Ensure target interface names (like can_device_1, can_device_2) are unique and do not conflict with existing interfaces in the system.
#         If you want to modify the relationship between USB ports and interface names, adjust the USB_PORTS array accordingly.
#-------------------------------------------------------------------------------------------------#

# Pre-defined number of CAN modules
EXPECTED_CAN_COUNT=2

if [ "$EXPECTED_CAN_COUNT" -eq 1 ]; then
    # Default CAN name, can be set by command-line arguments
    DEFAULT_CAN_NAME="${1:-can0}"

    # Default bitrate for single CAN module, can be set by command-line arguments
    DEFAULT_BITRATE="${2:-1000000}"

    # USB hardware address (optional parameter)
    USB_ADDRESS="${3}"
fi

# Pre-defined USB ports, target interface names, and their bitrates (used when multiple CAN modules)
if [ "$EXPECTED_CAN_COUNT" -ne 1 ]; then
    declare -A USB_PORTS 
    USB_PORTS["1-1:1.0"]="can_left:1000000"
    USB_PORTS["1-3:1.0"]="can_right:1000000"
fi

# Get the number of CAN modules in the current system
CURRENT_CAN_COUNT=$(ip link show type can | grep -c "link/can")

# Check if the number of CAN modules matches the expected number
if [ "$CURRENT_CAN_COUNT" -ne "$EXPECTED_CAN_COUNT" ]; then
    echo "Error: Detected CAN module count ($CURRENT_CAN_COUNT) does not match expected count ($EXPECTED_CAN_COUNT)."
    exit 1
fi

# Load gs_usb module
# sudo modprobe gs_usb
# if [ $? -ne 0 ]; then
#     echo "Error: Unable to load gs_usb module."
#     exit 1
# fi

# Check if only one CAN module needs to be processed
if [ "$EXPECTED_CAN_COUNT" -eq 1 ]; then
    if [ -n "$USB_ADDRESS" ]; then
        echo "Detected USB hardware address parameter: $USB_ADDRESS"
        
        # Use ethtool to find the CAN interface corresponding to the USB hardware address
        INTERFACE_NAME=""
        for iface in $(ip -br link show type can | awk '{print $1}'); do
            BUS_INFO=$(sudo ethtool -i "$iface" | grep "bus-info" | awk '{print $2}')
            if [ "$BUS_INFO" == "$USB_ADDRESS" ]; then
                INTERFACE_NAME="$iface"
                break
            fi
        done
        
        if [ -z "$INTERFACE_NAME" ]; then
            echo "Error: Unable to find CAN interface corresponding to USB hardware address $USB_ADDRESS."
            exit 1
        else
            echo "Found interface corresponding to USB hardware address $USB_ADDRESS: $INTERFACE_NAME"
        fi
    else
        # Get the unique CAN interface
        INTERFACE_NAME=$(ip -br link show type can | awk '{print $1}')
        
        # Check if interface name was obtained
        if [ -z "$INTERFACE_NAME" ]; then
            echo "Error: Unable to detect CAN interface."
            exit 1
        fi

        echo "Expected only one CAN module, detected interface $INTERFACE_NAME"
    fi

    # Check if the current interface is already activated
    IS_LINK_UP=$(ip link show "$INTERFACE_NAME" | grep -q "UP" && echo "yes" || echo "no")

    # Get the current interface's bitrate
    CURRENT_BITRATE=$(ip -details link show "$INTERFACE_NAME" | grep -oP 'bitrate \K\d+')

    if [ "$IS_LINK_UP" == "yes" ] && [ "$CURRENT_BITRATE" -eq "$DEFAULT_BITRATE" ]; then
        echo "Interface $INTERFACE_NAME is already activated with bitrate $DEFAULT_BITRATE"
        
        # Check if interface name matches the default name
        if [ "$INTERFACE_NAME" != "$DEFAULT_CAN_NAME" ]; then
            echo "Renaming interface $INTERFACE_NAME to $DEFAULT_CAN_NAME"
            sudo ip link set "$INTERFACE_NAME" down
            sudo ip link set "$INTERFACE_NAME" name "$DEFAULT_CAN_NAME"
            sudo ip link set "$DEFAULT_CAN_NAME" up
            echo "Interface renamed to $DEFAULT_CAN_NAME and reactivated."
        else
            echo "Interface name is already $DEFAULT_CAN_NAME"
        fi
    else
        # If interface is not activated or bitrate is different, configure it
        if [ "$IS_LINK_UP" == "yes" ]; then
            echo "Interface $INTERFACE_NAME is activated, but bitrate is $CURRENT_BITRATE, which differs from the set $DEFAULT_BITRATE."
        else
            echo "Interface $INTERFACE_NAME is not activated or bitrate not set."
        fi
        
        # Set interface bitrate and activate
        sudo ip link set "$INTERFACE_NAME" down
        sudo ip link set "$INTERFACE_NAME" type can bitrate $DEFAULT_BITRATE
        sudo ip link set "$INTERFACE_NAME" up
        echo "Interface $INTERFACE_NAME has been reset to bitrate $DEFAULT_BITRATE and activated."
        
        # Rename interface to default name
        if [ "$INTERFACE_NAME" != "$DEFAULT_CAN_NAME" ]; then
            echo "Renaming interface $INTERFACE_NAME to $DEFAULT_CAN_NAME"
            sudo ip link set "$INTERFACE_NAME" down
            sudo ip link set "$INTERFACE_NAME" name "$DEFAULT_CAN_NAME"
            sudo ip link set "$DEFAULT_CAN_NAME" up
            echo "Interface renamed to $DEFAULT_CAN_NAME and reactivated."
        fi
    fi
else
    # Process multiple CAN modules

    # Check if number of USB ports matches expected CAN module count
    PREDEFINED_COUNT=${#USB_PORTS[@]}
    if [ "$EXPECTED_CAN_COUNT" -ne "$PREDEFINED_COUNT" ]; then
        echo "Error: Expected CAN module count ($EXPECTED_CAN_COUNT) does not match predefined USB port count ($PREDEFINED_COUNT)."
        exit 1
    fi

    # Iterate through all CAN interfaces
    for iface in $(ip -br link show type can | awk '{print $1}'); do
        # Use ethtool to get bus-info
        BUS_INFO=$(sudo ethtool -i "$iface" | grep "bus-info" | awk '{print $2}')
        
        if [ -z "$BUS_INFO" ];then
            echo "Error: Unable to obtain bus-info for interface $iface."
            continue
        fi
        
        echo "Interface $iface inserted in USB port $BUS_INFO"

        # Check if bus-info is in predefined USB port list
        if [ -n "${USB_PORTS[$BUS_INFO]}" ];then
            IFS=':' read -r TARGET_NAME TARGET_BITRATE <<< "${USB_PORTS[$BUS_INFO]}"
            
            # Check if current interface is activated
            IS_LINK_UP=$(ip link show "$iface" | grep -q "UP" && echo "yes" || echo "no")

            # Get current interface's bitrate
            CURRENT_BITRATE=$(ip -details link show "$iface" | grep -oP 'bitrate \K\d+')

            if [ "$IS_LINK_UP" == "yes" ] && [ "$CURRENT_BITRATE" -eq "$TARGET_BITRATE" ]; then
                echo "Interface $iface is already activated with bitrate $TARGET_BITRATE"
                
                # Check if interface name matches target name
                if [ "$iface" != "$TARGET_NAME" ]; then
                    echo "Renaming interface $iface to $TARGET_NAME"
                    sudo ip link set "$iface" down
                    sudo ip link set "$iface" name "$TARGET_NAME"
                    sudo ip link set "$TARGET_NAME" up
                    echo "Interface renamed to $TARGET_NAME and reactivated."
                else
                    echo "Interface name is already $TARGET_NAME"
                fi
            else
                # If interface is not activated or bitrate is different, configure it
                if [ "$IS_LINK_UP" == "yes" ]; then
                    echo "Interface $iface is activated, but bitrate is $CURRENT_BITRATE, which differs from the set $TARGET_BITRATE."
                else
                    echo "Interface $iface is not activated or bitrate not set."
                fi
                
                # Set interface bitrate and activate
                sudo ip link set "$iface" down
                sudo ip link set "$iface" type can bitrate $TARGET_BITRATE
                sudo ip link set "$iface" up
                echo "Interface $iface has been reset to bitrate $TARGET_BITRATE and activated."
                
                # Rename interface to target name
                if [ "$iface" != "$TARGET_NAME" ]; then
                    echo "Renaming interface $iface to $TARGET_NAME"
                    sudo ip link set "$iface" down
                    sudo ip link set "$iface" name "$TARGET_NAME"
                    sudo ip link set "$TARGET_NAME" up
                    echo "Interface renamed to $TARGET_NAME and reactivated."
                fi
            fi
        else
            echo "Error: Unknown USB port $BUS_INFO for interface $iface."
            exit 1
        fi
    done
fi

echo "All CAN interfaces successfully renamed and activated."
