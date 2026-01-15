#!/usr/bin/env bash
set -euo pipefail

ENV_FILE="${1:?env}"
CACHE_DIR="${2:?cache}"
BUILD_DIR="${3:?build}"
OUT_DIR="${4:?out}"

source "$ENV_FILE"
mkdir -p "$BUILD_DIR" "$OUT_DIR"

BASE_IMG="$BUILD_DIR/base.img"
WORK_IMG="$BUILD_DIR/work.img"

if [[ ! -f "$BASE_IMG" ]]; then
  echo "Missing $BASE_IMG. Run: make fetch && make unpack"
  exit 1
fi

cp -f "$BASE_IMG" "$WORK_IMG"
sync

bash "$(dirname "$0")/image_resize.sh" "$WORK_IMG" max "$EXPAND_TO"

# Если нужно — копируем drone workspace в образ ДО provision
if [[ "${INSTALL_DRONE:-false}" == "true" ]]; then
  if [[ -z "${DRONE_WS_SRC:-}" ]]; then
    echo "INSTALL_DRONE=true but DRONE_WS_SRC is empty"
    exit 2
  fi
  if [[ ! -d "$DRONE_WS_SRC" ]]; then
    echo "DRONE_WS_SRC not found: $DRONE_WS_SRC"
    exit 2
  fi

  # Копируем в /home/pi/catkin_ws (пользователь создастся в provision, но файлы уже будут)
  bash "$(dirname "$0")/image_chroot.sh" "$WORK_IMG" copy "$DRONE_WS_SRC" "/home/${PI_USER:-pi}/catkin_ws"
fi

TMP_PROV="$BUILD_DIR/provision.sh"
cp -a "$(dirname "$0")/provision.sh" "$TMP_PROV"

ENABLE_SSH="$ENABLE_SSH" \
PI_USER="$PI_USER" \
PI_PASSWORD="$PI_PASSWORD" \
EXTRA_APT_PACKAGES="$EXTRA_APT_PACKAGES" \
INSTALL_ROS="${INSTALL_ROS:-false}" \
ROS_DISTRO="${ROS_DISTRO:-noetic}" \
INSTALL_DRONE="${INSTALL_DRONE:-false}" \
bash "$(dirname "$0")/image_chroot.sh" "$WORK_IMG" exec "$TMP_PROV"

bash "$(dirname "$0")/image_resize.sh" "$WORK_IMG" min "$FINAL_FREE_MIB"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_IMG="$OUT_DIR/${OUT_NAME_PREFIX}_${STAMP}.img"
mv -f "$WORK_IMG" "$OUT_IMG"

echo "DONE: $OUT_IMG"
ls -lh "$OUT_IMG"
