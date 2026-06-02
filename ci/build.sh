#!/usr/bin/env bash
##########################################
# WARNING: DO NOT MODIFY - THIS HAS NO RELATIONSHIP WITH CODE
##########################################

set -euo pipefail

echo "[CI] Build start"

: "${BUILD_DIR:=build}"
: "${CMAKE_PATH:=cmake}"
: "${NINJA:=ninja}"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Save build job URL/ID for notify stage to build correct artifacts link
echo "${CI_JOB_URL:-}" > ci_build_job_url.txt
echo "${CI_JOB_ID:-}"  > ci_build_job_id.txt

# Clear old log
: > build.log

# Prepare a sed that strips CI_PROJECT_DIR prefix from every output line
# Example:
#   /home/.../software/<project>/Frameworks/xxx.c:123 -> Frameworks/xxx.c:123
prefix="${CI_PROJECT_DIR%/}/"
prefix_esc="$(printf '%s' "$prefix" | sed 's/[\/&]/\\&/g')"

filter_strip_path() {
  sed -u "s/^${prefix_esc}//"
}

echo "[CI] CMake configure..." | tee -a build.log

# Print to console AND log, while stripping the prefix from BOTH
"$CMAKE_PATH" -G Ninja .. 2>&1 \
  | filter_strip_path \
  | tee -a build.log

echo "[CI] Ninja build..." | tee -a build.log

"$NINJA" 2>&1 \
  | filter_strip_path \
  | tee -a build.log

echo "[CI] Build done" | tee -a build.log