#!/usr/bin/env python3
import yaml

# Read the backup file
with open('ompl_planning.yaml.backup', 'r') as f:
    data = yaml.safe_load(f)

# Wrap with ros__parameters
wrapped_data = {
    '/**': {
        'ros__parameters': data
    }
}

# Write the fixed file
with open('ompl_planning.yaml', 'w') as f:
    yaml.dump(wrapped_data, f, default_flow_style=False, sort_keys=False, width=1000)

print("Fixed ompl_planning.yaml")
