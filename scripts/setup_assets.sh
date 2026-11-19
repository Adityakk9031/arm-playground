#!/usr/bin/env bash
# Fetch robot models (MJCF XML + meshes) from DeepMind's MuJoCo Menagerie.
# --depth 1 = shallow clone: latest snapshot only, no git history (much smaller).
set -euo pipefail
cd "$(dirname "$0")/.."

if [ -d mujoco_menagerie ]; then
  echo "mujoco_menagerie/ already exists, skipping clone."
else
  git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie mujoco_menagerie
fi
