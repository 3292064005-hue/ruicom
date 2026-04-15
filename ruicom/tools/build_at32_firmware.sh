#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
FW="$ROOT/firmware/mowen_chassis_controller/at32"
OUT="$FW/build"
mkdir -p "$OUT"
find_tool() {
  local explicit=${2:-}
  if [[ -n "$explicit" ]]; then
    printf '%s' "$explicit"
    return 0
  fi
  if command -v "$1" >/dev/null 2>&1; then
    command -v "$1"
    return 0
  fi
  return 1
}
CLANG=$(find_tool clang "${CLANG:-}" || find_tool /usr/local/swift/usr/bin/clang)
LLD=$(find_tool ld.lld "${LLD:-}" || find_tool /usr/local/swift/usr/bin/ld.lld)
OBJCOPY=$(find_tool llvm-objcopy "${OBJCOPY:-}" || find_tool /usr/local/swift/usr/bin/llvm-objcopy)
CFLAGS=(
  --target=armv7em-none-eabi
  -mcpu=cortex-m4
  -mthumb
  -ffreestanding
  -fdata-sections
  -ffunction-sections
  -Os
  -std=c11
  -I"$FW/include"
)
ASFLAGS=(--target=armv7em-none-eabi -mcpu=cortex-m4 -mthumb)
"$CLANG" "${CFLAGS[@]}" -c "$FW/src/at32_runtime.c" -o "$OUT/at32_runtime.o"
"$CLANG" "${CFLAGS[@]}" -c "$FW/src/board_port.c" -o "$OUT/board_port.o"
"$CLANG" "${CFLAGS[@]}" -c "$FW/src/runtime_support.c" -o "$OUT/runtime_support.o"
"$CLANG" "${CFLAGS[@]}" -c "$FW/src/system_at32f403a_407.c" -o "$OUT/system.o"
"$CLANG" "${CFLAGS[@]}" -c "$FW/src/main_at32.c" -o "$OUT/main.o"
"$CLANG" "${ASFLAGS[@]}" -c "$FW/startup/startup_at32f403a_407.S" -o "$OUT/startup.o"
"$LLD" -T "$FW/linker_at32f403avct7.ld" -o "$OUT/mowen_chassis_at32.elf" \
  "$OUT/startup.o" "$OUT/system.o" "$OUT/runtime_support.o" "$OUT/at32_runtime.o" "$OUT/board_port.o" "$OUT/main.o"
"$OBJCOPY" -O binary "$OUT/mowen_chassis_at32.elf" "$OUT/mowen_chassis_at32.bin"
echo "$OUT/mowen_chassis_at32.elf"
