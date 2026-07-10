#!/bin/bash
# Install Unitree SDK dependencies for robot_followao_radar conda environment
#
# Background:
# - Robot_Follow project has vendor/unitree_sdk2_python + cyclonedds installed
# - Robot_Follow_AO uses same bridge.py code but env lacks dependencies
# - Solution: install cyclonedds + configure PYTHONPATH for vendor SDK

set -e

echo "========================================================================"
echo "Installing Unitree SDK dependencies for robot_followao_radar"
echo "========================================================================"

# Step 1: Install cyclonedds (required by unitree_sdk2py)
echo ""
echo "[Step 1] Installing cyclonedds..."
conda run -n robot_followao_radar pip install cyclonedds==0.10.2

# Step 2: Verify vendor SDK exists
echo ""
echo "[Step 2] Checking vendor SDK..."
VENDOR_PATH="/home/fk/Robot_Follow_AO/vendor/unitree_sdk2_python"
if [ ! -d "$VENDOR_PATH" ]; then
    echo "⚠ WARNING: vendor/unitree_sdk2_python not found at $VENDOR_PATH"
    echo "  Please ensure the SDK is copied from Robot_Follow project:"
    echo "  cp -r /home/fk/Robot_Follow/vendor/unitree_sdk2_python /home/fk/Robot_Follow_AO/vendor/"
fi

# Step 3: Test import with PYTHONPATH
echo ""
echo "[Step 3] Testing SDK import..."
conda run -n robot_followao_radar --no-capture-output python -c "
import sys
sys.path.insert(0, '/home/fk/Robot_Follow_AO/vendor/unitree_sdk2_python')
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    print('✓ unitree_sdk2py import SUCCESS')
except Exception as e:
    print(f'✗ Import FAILED: {e}')
    exit(1)
"

echo ""
echo "========================================================================"
echo "Installation complete!"
echo "========================================================================"
echo ""
echo "Next steps:"
echo "1. Update your launch script to include vendor SDK in PYTHONPATH:"
echo ""
echo "   export PYTHONPATH=/home/fk/Robot_Follow_AO/src:/home/fk/Robot_Follow_AO/vendor/unitree_sdk2_python:\${PYTHONPATH}"
echo ""
echo "2. Run the main CLI:"
echo ""
echo "   conda run --no-capture-output -n robot_followao_radar \\"
echo "     python -u -m robot_follow.ao.live.cli \\"
echo "       --config configs/jetson_g1_ao_radar.yaml"
echo ""