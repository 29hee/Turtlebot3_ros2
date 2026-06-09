#!/usr/bin/env bash
set -euo pipefail

#: Base documentation directories required by the harness layout.
readonly HARNESS_DIRS=(
  "docs/design-docs"
  "docs/exec-plans/active"
  "docs/exec-plans/completed"
  "docs/generated"
  "docs/product-specs"
  "docs/references"
  "scripts"
)

create_harness_dirs() {
  #: Create every harness documentation directory if it does not yet exist.
  local dir
  for dir in "${HARNESS_DIRS[@]}"; do
    mkdir -p "${dir}"
  done
}

create_harness_dirs
