#!/bin/sh
# Each implementation selection has process lifetime. GPL version 2 or later.
set -eu
for scenario in native native-failure windows-permissions optional contract codecs; do
  echo "Filesystem: $scenario"
  "$FILESYSTEM_TEST" "$scenario"
done
