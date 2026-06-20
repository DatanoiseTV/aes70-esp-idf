#!/usr/bin/env bash
#
# Build and run the AES70 host unit tests (plain C, no ESP-IDF). Emits gcov
# line-coverage for the tested sources when the toolchain provides gcov.
#
# Usage: ./run.sh            # build + run
#        CC=gcc ./run.sh     # pick a compiler
#
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
COMP="$(cd "$HERE/../.." && pwd)"          # components/aes70
SRC="$COMP/src"
OUT="$HERE/build"
CC="${CC:-cc}"

mkdir -p "$OUT"
rm -f "$OUT"/*.gcda "$OUT"/*.gcov

# Pure sources under test (transport + mDNS are replaced by stubs.c).
SOURCES=(
  "$SRC/aes70_ocp1.c" "$SRC/aes70_object.c" "$SRC/aes70_classes.c"
  "$SRC/aes70_dsp.c"  "$SRC/aes70_grouper.c" "$SRC/aes70_managers.c" "$SRC/aes70_subscription.c"
  "$SRC/aes70_device.c" "$HERE/stubs.c" "$HERE/test_aes70.c"
)
# -include the standard headers IDF pulls in transitively (the host shims are
# minimal), so the sources compile unchanged.
CFLAGS=(-std=gnu11 -g -O0 --coverage -Wall -Wextra
        -include stdlib.h -include string.h -include stdint.h
        -I"$COMP/include" -I"$SRC" -I"$HERE/shims")

objs=()
for s in "${SOURCES[@]}"; do
  o="$OUT/$(basename "${s%.c}").o"
  "$CC" "${CFLAGS[@]}" -c "$s" -o "$o"
  objs+=("$o")
done
"$CC" --coverage "${objs[@]}" -lm -o "$OUT/test_runner"

( cd "$OUT" && ./test_runner ); rc=$?

echo
echo "---- coverage (tested sources) ----"
if command -v gcov >/dev/null 2>&1; then
  ( cd "$OUT" && gcov -n aes70_object.o aes70_classes.o aes70_dsp.o aes70_device.o aes70_ocp1.o 2>/dev/null ) \
    | grep -A1 -E "File '.*aes70_(object|classes|dsp|device|ocp1)\.c'" | grep -E "File|Lines executed" || true
else
  echo "(gcov not found; skipping coverage report)"
fi
exit $rc
