#!/usr/bin/env bash
# Outputs key-value pairs consumed by Bazel workspace stamping.
# Keys prefixed with STABLE_ cause downstream rebuilds when their value changes.
# With --nostamp (default), Bazel substitutes empty strings, keeping the cache intact.
GIT_VERSION=$(git describe --tags --match '[0-9][0-9]Q[0-9]' --always 2>/dev/null \
  || echo "unknown")
echo "STABLE_GIT_VERSION ${GIT_VERSION}"

# OpenSTA is a submodule with its own history; report the sha that is actually
# checked out and compiled (matches what src/sta/CMakeLists.txt derives via
# get_git_head_revision) so sta::git_sha1 agrees between CMake and Bazel.
STA_GIT_SHA1=$(git -C src/sta rev-parse HEAD 2>/dev/null || echo "unknown")
echo "STABLE_STA_GIT_SHA1 ${STA_GIT_SHA1}"
