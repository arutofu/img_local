#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

# ============================================================
# provision.sh (Clover-like)
# - fixes apt order + buster archived repos
# - pinned cmake 3.13.4-1 (buster)
# - adds systemd services: rosbridge(9090), drone(roslaunch), butterfly(57575)
# - fixes Wi-Fi AP: rfkill unblock before hostapd + dnsmasq waits for wlan0
# ============================================================

# -----------------------------
# Settings (env overrides)
# -----------------------------
PI_USER="${PI_USER:-pi}"
PI_PASSWORD="${PI_PASSWORD:-raspberry}"
HOST_NAME="${HOST_NAME:-raspberry}"

INSTALL_ROS="${INSTALL_ROS:-false}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"

INSTALL_DRONE="${INSTALL_DRONE:-false}"
CATKIN_WS="${CATKIN_WS:-/home/${PI_USER}/catkin_ws}"
DRONE_SRC_DIR="${DRONE_SRC_DIR:-${CATKIN_WS}/src}"

# Networking / services
ENABLE_WIFI_AP="${ENABLE_WIFI_AP:-true}"
WIFI_COUNTRY="${WIFI_COUNTRY:-DE}"
WIFI_CHANNEL="${WIFI_CHANNEL:-7}"
WIFI_AP_IP="${WIFI_AP_IP:-192.168.11.1}"
WIFI_AP_CIDR="${WIFI_AP_CIDR:-24}"
WIFI_DHCP_START="${WIFI_DHCP_START:-192.168.11.20}"
WIFI_DHCP_END="${WIFI_DHCP_END:-192.168.11.200}"
WIFI_DHCP_LEASE="${WIFI_DHCP_LEASE:-12h}"

ENABLE_MDNS="${ENABLE_MDNS:-true}"

ENABLE_WEB="${ENABLE_WEB:-true}"
WEB_ROOT_OVERRIDE="${WEB_ROOT_OVERRIDE:-}" # if empty -> use /home/pi/.ros/www

ENABLE_FILEBROWSER="${ENABLE_FILEBROWSER:-true}"
FILEBROWSER_PORT="${FILEBROWSER_PORT:-8090}"

AUTOLOGIN_TTY1="${AUTOLOGIN_TTY1:-true}"

EXTRA_APT_PACKAGES="${EXTRA_APT_PACKAGES:-}"

# Optional services
ENABLE_ROS_AUTOSTART="${ENABLE_ROS_AUTOSTART:-true}"
ENABLE_ROSBRIDGE="${ENABLE_ROSBRIDGE:-true}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
ENABLE_BUTTERFLY="${ENABLE_BUTTERFLY:-true}"
BUTTERFLY_PORT="${BUTTERFLY_PORT:-57575}"

# Clover pinned CMake (buster native)
CMAKE_VER="${CMAKE_VER:-3.13.4-1}"
CMAKE_DATA_VER="${CMAKE_DATA_VER:-3.13.4-1}"

# -----------------------------
# Logging
# -----------------------------
echo_stamp() {
  local text type
  text="$(date '+[%Y-%m-%d %H:%M:%S]') $1"
  type="${2:-INFO}"
  text="\e[1m${text}\e[0m"
  case "$type" in
    SUCCESS) text="\e[32m${text}\e[0m" ;;
    ERROR)   text="\e[31m${text}\e[0m" ;;
    *)       text="\e[34m${text}\e[0m" ;;
  esac
  echo -e "${text}"
}
info() { echo_stamp "$*" "INFO"; }
ok()   { echo_stamp "$*" "SUCCESS"; }
warn() { echo_stamp "$*" "ERROR"; }
die()  { warn "$*"; exit 1; }

have_cmd() { command -v "$1" >/dev/null 2>&1; }

require_root() {
  [[ "$(id -u)" == "0" ]] || die "Must be run as root (inside chroot). uid=$(id -u)"
}

print_kv() { printf "  - %-24s: %s\n" "$1" "$2"; }

# -----------------------------
# Retry helper (Clover-style)
# -----------------------------
my_travis_retry() {
  local result=0 count=1
  while [ "$count" -le 3 ]; do
    [ "$result" -ne 0 ] && {
      echo -e "\nThe command \"$*\" failed. Retrying, $count of 3.\n" >&2
    }
    ! { "$@"; result=$?; }
    [ "$result" -eq 0 ] && break
    count=$((count + 1))
    sleep 1
  done
  [ "$count" -gt 3 ] && echo -e "\nThe command \"$*\" failed 3 times.\n" >&2
  return "$result"
}

# -----------------------------
# Diagnostics
# -----------------------------
os_info() {
  info "OS / arch info"
  if [[ -f /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    print_kv "PRETTY_NAME" "${PRETTY_NAME:-unknown}"
    print_kv "VERSION_CODENAME" "${VERSION_CODENAME:-unknown}"
    print_kv "ID" "${ID:-unknown}"
  fi
  print_kv "ARCH" "$(dpkg --print-architecture 2>/dev/null || echo unknown)"
  print_kv "KERNEL" "$(uname -a 2>/dev/null || echo unknown)"
  print_kv "WHOAMI" "$(whoami)"
  print_kv "LANG" "${LANG:-<unset>}"
}

fs_sanity() {
  info "Filesystem sanity checks"
  [[ -d /etc ]] || die "/etc missing (broken rootfs)"
  [[ -d /var/lib/dpkg ]] || die "/var/lib/dpkg missing (dpkg DB missing)"
  [[ -e /dev/null ]] || die "/dev/null missing (is /dev bind-mounted?)"
  [[ -d /proc ]] || die "/proc missing"
  [[ -d /sys ]] || die "/sys missing"
}

# -----------------------------
# APT config + sources (do BEFORE any apt-get update)
# -----------------------------
apt_configure_archived_repos() {
  info "Configure apt retries + archive-friendly options"
  cat >/etc/apt/apt.conf.d/80-retries <<'EOF'
APT::Acquire::Retries "3";
Acquire::Retries "3";
Acquire::Check-Valid-Until "false";
Acquire::AllowReleaseInfoChange "true";
Acquire::AllowReleaseInfoChange::Suite "true";
Acquire::AllowReleaseInfoChange::Codename "true";
Acquire::AllowReleaseInfoChange::Version "true";
EOF
}

apt_fix_sources_buster() {
  info "Fix APT sources for Raspbian Buster (avoid dead raspbian.raspberrypi.org)"
  rm -f /etc/apt/sources.list.d/*.list 2>/dev/null || true

  cat >/etc/apt/sources.list <<'EOF'
deb http://legacy.raspbian.org/raspbian buster main contrib non-free rpi
deb http://archive.raspberrypi.org/debian buster main
EOF

  [[ -s /etc/apt/sources.list ]] || die "sources.list empty after rewrite"
  info "Current /etc/apt/sources.list:"
  sed -n '1,120p' /etc/apt/sources.list || true
}

apt_update() {
  info "apt-get update"
  LC_ALL=C my_travis_retry apt-get update -y --allow-releaseinfo-change
}

apt_install() {
  info "apt-get install: $*"
  LC_ALL=C my_travis_retry apt-get install -y --no-install-recommends "$@"
}

apt_remove() {
  info "apt-get remove: $*"
  LC_ALL=C my_travis_retry apt-get remove -y "$@" || true
}

apt_policy_dump() {
  info "APT sources + cmake policy (diagnostics)"
  echo "----- /etc/apt/sources.list -----"
  [[ -f /etc/apt/sources.list ]] && sed -n '1,200p' /etc/apt/sources.list || true
  echo "----- /etc/apt/sources.list.d/*.list -----"
  ls -la /etc/apt/sources.list.d 2>/dev/null || true
  for f in /etc/apt/sources.list.d/*.list; do
    [[ -f "$f" ]] || continue
    echo "----- $f -----"
    sed -n '1,200p' "$f" || true
  done
  echo "----- apt-cache policy cmake -----"
  (apt-cache policy cmake || true) | sed -n '1,160p'
  echo "----- dpkg -l | grep cmake -----"
  (dpkg -l | grep -E '^(ii|rc)\s+cmake(\s|$)|^(ii|rc)\s+cmake-data(\s|$)' || true)
}

# -----------------------------
# Locale fix
# -----------------------------
ensure_locales() {
  info "Ensure locales (ru_RU.UTF-8) to avoid apt/perl warnings"
  apt_install locales || true

  if [[ -f /etc/locale.gen ]]; then
    sed -i 's/^\s*#\s*\(ru_RU.UTF-8 UTF-8\)/\1/' /etc/locale.gen 2>/dev/null || true
    sed -i 's/^\s*#\s*\(en_US.UTF-8 UTF-8\)/\1/' /etc/locale.gen 2>/dev/null || true
  fi

  if have_cmd locale-gen; then
    LC_ALL=C locale-gen ru_RU.UTF-8 || true
    LC_ALL=C locale-gen en_US.UTF-8 || true
  fi

  if have_cmd update-locale; then
    LC_ALL=C update-locale LANG=ru_RU.UTF-8 LC_ALL=ru_RU.UTF-8 || true
  fi

  cat >/etc/default/locale <<'EOF'
LANG=ru_RU.UTF-8
LC_ALL=ru_RU.UTF-8
EOF

  export LANG=ru_RU.UTF-8
  info "Locale check:"
  (locale || true) | sed -n '1,80p'
}

# -----------------------------
# Add repos/keys like Clover (ROS + Coex)
# -----------------------------
add_ros_and_coex_repos_like_clover() {
  info "Install apt tools for keys/repos"
  apt_install ca-certificates wget curl gnupg dirmngr lsb-release apt-transport-https || true

  if ! apt-key list 2>/dev/null | grep -q "F42ED6FBAB17C654"; then
    info "Add ROS apt key (best-effort)"
    my_travis_retry apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
      --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 || true
  fi

  if [[ ! -f /etc/apt/sources.list.d/ros-latest.list ]]; then
    echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-latest.list
  fi

  if ! apt-key list 2>/dev/null | grep -qi "packages.coex.tech"; then
    info "Add Coex apt key (best-effort)"
    my_travis_retry wget -qO- 'http://packages.coex.tech/key.asc' | apt-key add - || true
  fi

  if ! grep -q "packages.coex.tech" /etc/apt/sources.list 2>/dev/null; then
    echo 'deb http://packages.coex.tech buster main' >> /etc/apt/sources.list
  fi
}

# -----------------------------
# CMake sanity
# -----------------------------
cmake_sanity_test() {
  have_cmd cmake || return 1
  have_cmd cc || return 1

  local tmpd
  tmpd="$(mktemp -d)"

  cat >"${tmpd}/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.0)
project(cmake_sanity C)
add_executable(hello main.c)
EOF
  cat >"${tmpd}/main.c" <<'EOF'
#include <stdio.h>
int main(){ puts("ok"); return 0; }
EOF

  if (cd "$tmpd" && cmake -G "Unix Makefiles" . >/dev/null 2>&1); then
    rm -rf "$tmpd"
    return 0
  fi

  echo "----- cmake sanity FAILED; logs -----" >&2
  [[ -f "${tmpd}/CMakeFiles/CMakeError.log" ]] && tail -n 120 "${tmpd}/CMakeFiles/CMakeError.log" >&2 || true
  rm -rf "$tmpd"
  return 1
}

install_cmake_like_clover() {
  info "Install CMake pinned like Clover: cmake-data=${CMAKE_DATA_VER} cmake=${CMAKE_VER}"

  apt_remove cmake cmake-data || true

  if ! apt_install "cmake-data=${CMAKE_DATA_VER}" "cmake=${CMAKE_VER}"; then
    apt_policy_dump
    die "Failed to install pinned CMake versions (cmake=${CMAKE_VER}, cmake-data=${CMAKE_DATA_VER})."
  fi

  apt-mark hold cmake cmake-data >/dev/null 2>&1 || true

  info "CMake version now: $(cmake --version | head -n 1 || true)"
  if ! cmake_sanity_test; then
    apt_policy_dump
    die "Pinned CMake installed, but sanity test still fails. Check /dev bind mounts and repo mix."
  fi
  ok "CMake sanity OK"
}

# -----------------------------
# Base deps + user/ssh
# -----------------------------
ensure_user_and_hostname() {
  info "Ensure user ${PI_USER} and hostname ${HOST_NAME}"
  apt_install sudo openssh-server || true

  if id "${PI_USER}" >/dev/null 2>&1; then
    info "User exists: ${PI_USER}"
  else
    useradd -m -s /bin/bash "${PI_USER}"
  fi

  if [[ -n "${PI_PASSWORD}" ]]; then
    echo "${PI_USER}:${PI_PASSWORD}" | chpasswd
  else
    passwd -d "${PI_USER}" >/dev/null 2>&1 || true
  fi

  cat >/etc/sudoers.d/010-"${PI_USER}"-nopasswd <<EOF
${PI_USER} ALL=(ALL) NOPASSWD:ALL
EOF
  chmod 0440 /etc/sudoers.d/010-"${PI_USER}"-nopasswd

  echo "${HOST_NAME}" > /etc/hostname
  hostname "${HOST_NAME}" 2>/dev/null || true
  sed -i "s/127.0.1.1.*/127.0.1.1\t${HOST_NAME}/g" /etc/hosts 2>/dev/null || true

  systemctl enable ssh 2>/dev/null || true

  if [[ -z "${PI_PASSWORD}" ]]; then
    if [[ -f /etc/ssh/sshd_config ]]; then
      sed -i 's/^#\?PasswordAuthentication\s\+.*/PasswordAuthentication yes/' /etc/ssh/sshd_config || true
      sed -i 's/^#\?PermitEmptyPasswords\s\+.*/PermitEmptyPasswords yes/' /etc/ssh/sshd_config || true
      grep -q '^PasswordAuthentication' /etc/ssh/sshd_config || echo 'PasswordAuthentication yes' >> /etc/ssh/sshd_config
      grep -q '^PermitEmptyPasswords' /etc/ssh/sshd_config || echo 'PermitEmptyPasswords yes' >> /etc/ssh/sshd_config
    fi
  fi
}

install_base_build_deps() {
  info "Install base build dependencies"
  apt_install \
    bash coreutils findutils grep sed gawk \
    build-essential make \
    git \
    python3 python3-pip python3-dev \
    pkg-config \
    ca-certificates curl wget \
    libxml2-dev || true

  have_cmd cc || die "C compiler not found after build-essential"
}

install_extra_packages() {
  if [[ -n "${EXTRA_APT_PACKAGES}" ]]; then
    info "Install EXTRA_APT_PACKAGES: ${EXTRA_APT_PACKAGES}"
    # shellcheck disable=SC2086
    LC_ALL=C my_travis_retry apt-get install -y --no-install-recommends ${EXTRA_APT_PACKAGES} || true
  fi
}

# -----------------------------
# ROS install (optional)
# -----------------------------
install_ros_if_needed() {
  if [[ "${INSTALL_ROS}" != "true" ]]; then
    info "INSTALL_ROS=false -> skip ROS install"
    return 0
  fi

  info "Install ROS: ros-${ROS_DISTRO}-ros-base + catkin"
  apt_update

  apt_install \
    "ros-${ROS_DISTRO}-ros-base" \
    "ros-${ROS_DISTRO}-catkin" \
    python3-rosdep || {
      apt_policy_dump
      die "ROS install failed (check repos/keys)."
    }

  [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || die "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash"
  ok "ROS installed: ${ROS_DISTRO}"
}

install_ros_web_tools() {
  # Эти пакеты нужны именно для WEB UI (topics.html) и т.п.
  if [[ "${INSTALL_ROS}" != "true" ]]; then
    info "INSTALL_ROS=false -> skip ROS web tools install"
    return 0
  fi

  info "Install ROS web tools (rosbridge-server)"
  apt_update
  apt_install "ros-${ROS_DISTRO}-rosbridge-server" || {
    apt_policy_dump
    die "Failed to install rosbridge-server"
  }
}

# -----------------------------
# Drone build (optional)
# -----------------------------
verify_catkin_workspace() {
  info "Verify catkin workspace layout"
  print_kv "CATKIN_WS" "${CATKIN_WS}"
  print_kv "DRONE_SRC_DIR" "${DRONE_SRC_DIR}"

  [[ -d "${CATKIN_WS}" ]] || die "CATKIN_WS missing: ${CATKIN_WS}"
  [[ -d "${DRONE_SRC_DIR}" ]] || die "src dir missing: ${DRONE_SRC_DIR}"

  local cnt
  cnt="$(find "${DRONE_SRC_DIR}" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l | tr -d ' ')"
  [[ "${cnt}" -gt 0 ]] || die "src dir looks empty: ${DRONE_SRC_DIR}"
}

build_drone_if_needed() {
  if [[ "${INSTALL_DRONE}" != "true" ]]; then
    info "INSTALL_DRONE=false -> skip drone build"
    return 0
  fi

  info "Build drone/catkin_ws requested"
  [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || die "ROS required for catkin build, missing /opt/ros/${ROS_DISTRO}/setup.bash"

  verify_catkin_workspace

  apt_install \
    "ros-${ROS_DISTRO}-nodelet" \
    "ros-${ROS_DISTRO}-pluginlib" \
    "ros-${ROS_DISTRO}-roscpp" \
    "ros-${ROS_DISTRO}-rospy" \
    "ros-${ROS_DISTRO}-std-msgs" \
    "ros-${ROS_DISTRO}-message-generation" \
    "ros-${ROS_DISTRO}-message-runtime" \
    "ros-${ROS_DISTRO}-geometry-msgs" \
    "ros-${ROS_DISTRO}-sensor-msgs" \
    "ros-${ROS_DISTRO}-geographic-msgs" \
    "ros-${ROS_DISTRO}-tf" \
    "ros-${ROS_DISTRO}-tf2" \
    "ros-${ROS_DISTRO}-tf2-geometry-msgs" \
    "ros-${ROS_DISTRO}-tf2-ros" \
    "ros-${ROS_DISTRO}-image-transport" \
    "ros-${ROS_DISTRO}-cv-bridge" \
    "ros-${ROS_DISTRO}-dynamic-reconfigure" \
    "ros-${ROS_DISTRO}-mavros-msgs" \
    libgeographic-dev \
    libopencv-dev || {
      apt_policy_dump
      die "Failed to install required deps for drone build"
    }

  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u

  have_cmd catkin_make || die "catkin_make not found after sourcing ROS"

  rm -rf "${CATKIN_WS}/build" "${CATKIN_WS}/devel" "${CATKIN_WS}/install"
  info "catkin_make"
  (cd "${CATKIN_WS}" && catkin_make 2>&1 | tee /var/log/drone_catkin_make.log) || {
    warn "catkin_make failed. Tail of /var/log/drone_catkin_make.log:"
    tail -n 200 /var/log/drone_catkin_make.log || true
    die "Drone build failed."
  }

  ok "Drone build OK"
}

# -----------------------------
# Console autologin
# -----------------------------
enable_console_autologin() {
  [[ "${AUTOLOGIN_TTY1}" == "true" ]] || { info "AUTOLOGIN_TTY1=false -> skip"; return 0; }
  info "Enable console autologin for ${PI_USER} on tty1"
  mkdir -p /etc/systemd/system/getty@tty1.service.d
  cat >/etc/systemd/system/getty@tty1.service.d/autologin.conf <<EOF
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin ${PI_USER} --noclear %I \$TERM
EOF
}

# -----------------------------
# mDNS (raspberry.local)
# -----------------------------
setup_mdns_avahi() {
  [[ "${ENABLE_MDNS}" == "true" ]] || { info "ENABLE_MDNS=false -> skip mDNS"; return 0; }
  info "Install/enable avahi-daemon for .local name (raspberry.local)"
  apt_install avahi-daemon avahi-utils || true
  systemctl enable avahi-daemon 2>/dev/null || true
  systemctl enable avahi-daemon.service 2>/dev/null || true
}

# -----------------------------
# Wi-Fi AP fixes
# -----------------------------
setup_wifi_rfkill_unblock_service() {
  [[ "${ENABLE_WIFI_AP}" == "true" ]] || return 0
  info "Add rfkill-unblock service (before hostapd)"
  cat >/etc/systemd/system/drone-wifi-unblock.service <<'EOF'
[Unit]
Description=Unblock Wi-Fi (rfkill) before AP services
Before=hostapd.service dnsmasq.service
Wants=sys-subsystem-rfkill-devices-phy0.device
After=sys-subsystem-rfkill-devices-phy0.device

[Service]
Type=oneshot
ExecStart=/usr/sbin/rfkill unblock all

[Install]
WantedBy=multi-user.target
EOF
  systemctl enable drone-wifi-unblock.service 2>/dev/null || true
}

setup_wifi_ap() {
  [[ "${ENABLE_WIFI_AP}" == "true" ]] || { info "ENABLE_WIFI_AP=false -> skip Wi-Fi AP"; return 0; }

  info "Setup Wi-Fi Access Point (hostapd/dnsmasq)"
  apt_install hostapd dnsmasq iptables iptables-persistent rfkill || true

  # Ensure rfkill unblock happens on boot BEFORE these
  setup_wifi_rfkill_unblock_service

  # Stop services during image build
  systemctl disable hostapd 2>/dev/null || true
  systemctl disable dnsmasq 2>/dev/null || true
  systemctl stop hostapd 2>/dev/null || true
  systemctl stop dnsmasq 2>/dev/null || true

  # Static IP for wlan0
  if [[ -f /etc/dhcpcd.conf ]]; then
    if ! grep -q "^interface wlan0" /etc/dhcpcd.conf; then
      cat >>/etc/dhcpcd.conf <<EOF

interface wlan0
  static ip_address=${WIFI_AP_IP}/${WIFI_AP_CIDR}
  nohook wpa_supplicant
EOF
    fi
  fi

  # dnsmasq DHCP for wlan0 (bind-dynamic = НЕ падать если wlan0 ещё не поднят)
  mkdir -p /etc/dnsmasq.d
  cat >/etc/dnsmasq.d/drone-ap.conf <<EOF
interface=wlan0
bind-dynamic
dhcp-range=${WIFI_DHCP_START},${WIFI_DHCP_END},255.255.255.0,${WIFI_DHCP_LEASE}
domain-needed
bogus-priv
EOF

  # hostapd config template
  mkdir -p /etc/hostapd
  cat >/etc/hostapd/hostapd.conf <<EOF
country_code=${WIFI_COUNTRY}
interface=wlan0
driver=nl80211
ssid=drone-0000
hw_mode=g
channel=${WIFI_CHANNEL}
ieee80211n=1
wmm_enabled=1
auth_algs=1
wpa=0
EOF

  # Ensure hostapd uses our config
  if [[ -f /etc/default/hostapd ]]; then
    sed -i 's/^#\?DAEMON_CONF=.*/DAEMON_CONF="\/etc\/hostapd\/hostapd.conf"/' /etc/default/hostapd || true
    grep -q '^DAEMON_CONF=' /etc/default/hostapd || echo 'DAEMON_CONF="/etc/hostapd/hostapd.conf"' >> /etc/default/hostapd
  else
    cat >/etc/default/hostapd <<'EOF'
DAEMON_CONF="/etc/hostapd/hostapd.conf"
EOF
  fi

  # systemd: make dnsmasq wait for wlan0
  mkdir -p /etc/systemd/system/dnsmasq.service.d
  cat >/etc/systemd/system/dnsmasq.service.d/override.conf <<'EOF'
[Unit]
After=sys-subsystem-net-devices-wlan0.device
Wants=sys-subsystem-net-devices-wlan0.device
EOF

  # Enable routing
  cat >/etc/sysctl.d/99-drone-ipforward.conf <<'EOF'
net.ipv4.ip_forward=1
EOF

  # NAT rules (wlan0 -> eth0)
  mkdir -p /etc/iptables
  cat >/etc/iptables/rules.v4 <<'EOF'
*filter
:INPUT ACCEPT [0:0]
:FORWARD ACCEPT [0:0]
:OUTPUT ACCEPT [0:0]
-A FORWARD -i wlan0 -o eth0 -j ACCEPT
-A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
COMMIT
*nat
:PREROUTING ACCEPT [0:0]
:INPUT ACCEPT [0:0]
:OUTPUT ACCEPT [0:0]
:POSTROUTING ACCEPT [0:0]
-A POSTROUTING -o eth0 -j MASQUERADE
COMMIT
EOF

  # First-boot SSID generator
  cat >/usr/local/sbin/drone-generate-ssid.sh <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
CONF=/etc/hostapd/hostapd.conf
STAMP=/var/lib/drone/ssid_generated
mkdir -p /var/lib/drone

[[ -f "$STAMP" ]] && exit 0

rand=$(shuf -i 0-9999 -n 1 2>/dev/null || echo $((RANDOM%10000)))
ssid=$(printf "drone-%04d" "$rand")

if [[ -f "$CONF" ]]; then
  if grep -q '^ssid=' "$CONF"; then
    sed -i "s/^ssid=.*/ssid=${ssid}/" "$CONF"
  else
    echo "ssid=${ssid}" >> "$CONF"
  fi
fi

echo "$ssid" > /var/lib/drone/wifi_ssid
touch "$STAMP"
EOF
  chmod +x /usr/local/sbin/drone-generate-ssid.sh

  cat >/etc/systemd/system/drone-generate-ssid.service <<'EOF'
[Unit]
Description=Generate Wi-Fi AP SSID (drone-XXXX) on first boot
Before=hostapd.service

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/drone-generate-ssid.sh

[Install]
WantedBy=multi-user.target
EOF

  systemctl enable drone-generate-ssid.service 2>/dev/null || true
  systemctl enable hostapd 2>/dev/null || true
  systemctl enable dnsmasq 2>/dev/null || true
}

# -----------------------------
# Filebrowser
# -----------------------------
install_filebrowser_noauth() {
  [[ "${ENABLE_FILEBROWSER}" == "true" ]] || { info "ENABLE_FILEBROWSER=false -> skip filebrowser"; return 0; }

  info "Install Filebrowser (no-auth)"
  local arch asset url tmpd
  arch="$(dpkg --print-architecture 2>/dev/null || echo armhf)"
  case "$arch" in
    armhf) asset="linux-armv7-filebrowser.tar.gz" ;;
    arm64) asset="linux-arm64-filebrowser.tar.gz" ;;
    amd64) asset="linux-amd64-filebrowser.tar.gz" ;;
    *)     asset="linux-armv7-filebrowser.tar.gz" ;;
  esac

  url="https://github.com/filebrowser/filebrowser/releases/latest/download/${asset}"
  tmpd="$(mktemp -d)"

  apt_install ca-certificates curl || true
  my_travis_retry curl -fsSL -o "${tmpd}/fb.tgz" "$url" || {
    rm -rf "$tmpd"
    die "Failed to download filebrowser from GitHub releases (${asset})"
  }
  tar -xzf "${tmpd}/fb.tgz" -C "$tmpd"
  install -m 0755 "${tmpd}/filebrowser" /usr/local/bin/filebrowser
  rm -rf "$tmpd"

  mkdir -p /var/lib/filebrowser
  chown -R "${PI_USER}:${PI_USER}" /var/lib/filebrowser || true

  cat >/etc/systemd/system/filebrowser.service <<EOF
[Unit]
Description=File Browser
After=network-online.target
Wants=network-online.target

[Service]
User=${PI_USER}
Group=${PI_USER}
ExecStart=/usr/local/bin/filebrowser \
  --noauth \
  --root=/home \
  --address=0.0.0.0 \
  --port=${FILEBROWSER_PORT} \
  --database=/var/lib/filebrowser/filebrowser.db
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

  systemctl enable filebrowser.service 2>/dev/null || true
}

# -----------------------------
# Nginx static web
# -----------------------------
setup_static_web_like_clover() {
  [[ "${ENABLE_WEB}" == "true" ]] || { info "ENABLE_WEB=false -> skip web"; return 0; }

  info "Setup static web hosting on :80 (nginx)"
  apt_install nginx || true
  systemctl enable nginx 2>/dev/null || true

  local www_root
  if [[ -n "${WEB_ROOT_OVERRIDE}" ]]; then
    www_root="${WEB_ROOT_OVERRIDE}"
  else
    www_root="/home/${PI_USER}/.ros/www"
  fi

  mkdir -p "$www_root"
  chown -R "${PI_USER}:${PI_USER}" "/home/${PI_USER}/.ros" || true

  cat >/etc/nginx/sites-available/drone <<EOF
server {
    listen 80 default_server;
    listen [::]:80 default_server;
    server_name ${HOST_NAME}.local ${HOST_NAME} _;
    root ${www_root};
    index index.html;

    location / {
        try_files \$uri \$uri/ =404;
    }
}
EOF
  ln -sf /etc/nginx/sites-available/drone /etc/nginx/sites-enabled/drone
  rm -f /etc/nginx/sites-enabled/default 2>/dev/null || true
}

# -----------------------------
# roswww_static update service
# -----------------------------
setup_roswww_static_update_service() {
  [[ "${ENABLE_WEB}" == "true" ]] || return 0
  info "Setup roswww_static update service (drone/www -> ~/.ros/www)"

  local srcdir
  srcdir="${CATKIN_WS}/src/drone/www"
  [[ -d "$srcdir" ]] || warn "Expected web source dir not found: ${srcdir} (still configuring services)"

  cat >/etc/default/drone-www <<EOF
ROS_DISTRO=${ROS_DISTRO}
ROSWWW_DEFAULT=drone
ROSWWW_STATIC_PATH=${srcdir}
EOF

  cat >/usr/local/sbin/drone-www-update.sh <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

if [[ -f /etc/default/drone-www ]]; then
  # shellcheck disable=SC1091
  . /etc/default/drone-www
fi

export ROSWWW_DEFAULT="${ROSWWW_DEFAULT:-drone}"
export ROSWWW_STATIC_PATH="${ROSWWW_STATIC_PATH:-/home/pi/catkin_ws/src/drone/www}"

setup_ros="/opt/ros/${ROS_DISTRO:-noetic}/setup.bash"
setup_ws="/home/pi/catkin_ws/devel/setup.bash"

set +u
[[ -f "$setup_ros" ]] && source "$setup_ros"
[[ -f "$setup_ws"  ]] && source "$setup_ws"
set -u

if [[ -x "/home/pi/catkin_ws/devel/lib/roswww_static/update" ]]; then
  exec "/home/pi/catkin_ws/devel/lib/roswww_static/update"
fi

exec rosrun roswww_static update
EOF
  chmod +x /usr/local/sbin/drone-www-update.sh
  chown "${PI_USER}:${PI_USER}" /usr/local/sbin/drone-www-update.sh || true

  cat >/etc/systemd/system/drone-www.service <<EOF
[Unit]
Description=Generate Drone web pages (roswww_static)
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
User=${PI_USER}
Group=${PI_USER}
ExecStart=/usr/local/sbin/drone-www-update.sh

[Install]
WantedBy=multi-user.target
EOF

  systemctl enable drone-www.service 2>/dev/null || true
}

# -----------------------------
# ROS systemd services (autostart like Clover image behavior)
# -----------------------------
write_ros_env_file() {
  cat >/etc/default/ros <<EOF
ROS_DISTRO=${ROS_DISTRO}
ROS_MASTER_URI=http://localhost:11311
# Для локальной машины это не критично; при желании можно заменить на HOST_NAME.local:
# ROS_HOSTNAME=${HOST_NAME}.local
EOF
}

setup_rosbridge_service() {
  [[ "${ENABLE_ROS_AUTOSTART}" == "true" ]] || { info "ENABLE_ROS_AUTOSTART=false -> skip ros services"; return 0; }
  [[ "${ENABLE_ROSBRIDGE}" == "true" ]] || { info "ENABLE_ROSBRIDGE=false -> skip rosbridge"; return 0; }

  info "Create rosbridge-websocket systemd service (ws://0.0.0.0:${ROSBRIDGE_PORT})"
  write_ros_env_file

  cat >/etc/systemd/system/rosbridge-websocket.service <<EOF
[Unit]
Description=ROS Bridge WebSocket (rosbridge_server)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
EnvironmentFile=/etc/default/ros
User=${PI_USER}
Group=${PI_USER}
WorkingDirectory=/home/${PI_USER}
ExecStart=/bin/bash -lc 'source /opt/ros/\${ROS_DISTRO}/setup.bash && roslaunch rosbridge_server rosbridge_websocket.launch address:=0.0.0.0 port:=${ROSBRIDGE_PORT}'
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

  systemctl enable rosbridge-websocket.service 2>/dev/null || true
}

setup_drone_launch_service() {
  [[ "${ENABLE_ROS_AUTOSTART}" == "true" ]] || return 0

  info "Create drone systemd service (roslaunch drone drone.launch)"
  write_ros_env_file

  cat >/etc/systemd/system/drone.service <<EOF
[Unit]
Description=Drone main ROS launch
After=network-online.target rosbridge-websocket.service
Wants=network-online.target
Wants=rosbridge-websocket.service

[Service]
Type=simple
EnvironmentFile=/etc/default/ros
User=${PI_USER}
Group=${PI_USER}
WorkingDirectory=${CATKIN_WS}
ExecStart=/bin/bash -lc 'source /opt/ros/\${ROS_DISTRO}/setup.bash && [[ -f ${CATKIN_WS}/devel/setup.bash ]] && source ${CATKIN_WS}/devel/setup.bash; roslaunch drone drone.launch'
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

  systemctl enable drone.service 2>/dev/null || true
}

setup_butterfly_service() {
  [[ "${ENABLE_BUTTERFLY}" == "true" ]] || { info "ENABLE_BUTTERFLY=false -> skip"; return 0; }
  info "Install butterfly web terminal + systemd service on :${BUTTERFLY_PORT}"

  # butterfly удобнее через pip (пакета apt на buster может не быть)
  apt_install python3-pip || true
  my_travis_retry pip3 install --no-cache-dir butterfly || true

  cat >/etc/systemd/system/butterfly.service <<EOF
[Unit]
Description=Butterfly Web Terminal
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${PI_USER}
Group=${PI_USER}
WorkingDirectory=/home/${PI_USER}
ExecStart=/bin/bash -lc 'exec butterfly --host=0.0.0.0 --port=${BUTTERFLY_PORT} --unsecure'
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

  systemctl enable butterfly.service 2>/dev/null || true
}

# -----------------------------
# Cleanup
# -----------------------------
cleanup() {
  info "Cleanup apt cache"
  apt-get clean || true
  rm -rf /var/lib/apt/lists/* || true
}

# ============================================================
# MAIN
# ============================================================
require_root
os_info
fs_sanity

apt_configure_archived_repos
apt_fix_sources_buster

apt_update
ensure_locales

install_base_build_deps

add_ros_and_coex_repos_like_clover
apt_update

install_cmake_like_clover

ensure_user_and_hostname
install_extra_packages

install_ros_if_needed
install_ros_web_tools
build_drone_if_needed

enable_console_autologin
setup_mdns_avahi

setup_wifi_ap
install_filebrowser_noauth

setup_static_web_like_clover
setup_roswww_static_update_service

# ROS autostart services
setup_rosbridge_service
setup_drone_launch_service
setup_butterfly_service

info "Generate web pages now (best-effort)"
su - "${PI_USER}" -c "/usr/local/sbin/drone-www-update.sh" || true
systemctl restart nginx 2>/dev/null || true

cleanup
ok "DONE"
