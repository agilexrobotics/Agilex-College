"""Package containing robot configurations."""
import os
import toml

##
# Configuration for different assets.
##

# Conveniences to other module directories via relative paths
AGX_TELEOP_EXT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../"))
"""Path to the source agx_teleop directory."""

AGX_TELEOP_DESCRIPTION_DIR = os.path.join(AGX_TELEOP_EXT_DIR, "agx_description")
"""Path to the agx_description directory."""