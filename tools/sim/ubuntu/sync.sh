#!/bin/bash
# Windows side: copy the working tree to the Ubuntu laptop without committing.
#   tools/sim/ubuntu/sync.sh            (run from the repository root, in Git Bash)
set -e
tar cf - sim tools docs CLAUDE.md 2>/dev/null | ssh agribot-ubuntu 'cd ~/AgriBot && tar xf - && find sim tools -name "*.py" -o -name "*.sh" -o -name "*.yaml" -o -name "*.xacro" -o -name "*.urdf" -o -name "*.sdf" -o -name "*.msg" -o -name "*.srv" -o -name "*.action" -o -name "*.xml" -o -name "*.cfg" | xargs sed -i "s/\r$//"'
echo synced
