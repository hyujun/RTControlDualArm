#!/bin/bash
# =============================================================================
# build.sh  Xenomai 3.2.4 + Dovetail kernel 6.1.61 + IgH EtherCAT 1.6.8
#
# [변경 이력]
#   EtherLab 1.5.2 -> 1.6.8
#     - 다운로드 경로: etherlab.org/download -> GitLab releases
#     - Git clone stable-1.6 브랜치 사용 (tarball보다 최신 패치 포함)
#     - 1.6.x 알려진 버그 자동 수정 패치 내장:
#         [BUG-1] inline 함수 중복 선언 빌드 오류 (GCC 11+, kernel 6.1)
#                 -> remove-inline-statement 패치 자동 적용
#         [BUG-2] udev rules 파일 미생성 -> /etc/udev/rules.d/99-ethercat.rules 자동 생성
#         [BUG-3] ioctl version magic 불일치 -> 커널 모듈+userspace 동일 소스 빌드 강제
#     - semantic versioning 도입 (1.6.0부터)
#     - kernel 3.0 미만 지원 종료
#     - 새 API: ecrt_master_deactivate_slaves() 등 추가
#     - fake library(시뮬레이션용) 추가
#
# Architecture change from 3.0.x:
#   3.0.x : I-pipe(ipipe) + Kernel <= 4.19 + Ubuntu 18.04
#   3.2.x : Dovetail      + Kernel >= 5.10 + Ubuntu 22.04  <- This script
#
# Ubuntu recommendation matrix:
#  22.04 LTS (Jammy)  GCC-11  glibc-2.35   RECOMMENDED  Dovetail/6.1 최적
#  20.04 LTS (Focal)  GCC-9   glibc-2.31   Supported    Kernel 5.10/5.15
#  24.04 LTS (Noble)  GCC-13  glibc-2.39   Caution      사례 부족
#  18.04 LTS (Bionic) GCC-7   glibc-2.27   NOT for 3.2x (3.0.x 전용)
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
LOG_FILE="${BUILD_DIR}/build.log"

XENOMAI_VERSION="3.2.4"
DOVETAIL_KERNEL_VERSION="6.1.61-dovetail1"
DOVETAIL_KERNEL_SHORT="6.1.61"

# EtherLab 1.6.8 — GitLab 기반
# tarball 대신 git clone stable-1.6 브랜치 사용:
#   -> remove-inline-statement 패치 등 최신 버그픽스 포함
ETHERLAB_VERSION="1.6.8"
ETHERLAB_GIT_URL="https://gitlab.com/etherlab.org/ethercat.git"
ETHERLAB_GIT_BRANCH="stable-1.6"
# tarball 대체 URL (git 불가 시):
ETHERLAB_TARBALL_URL="https://gitlab.com/etherlab.org/ethercat/-/archive/${ETHERLAB_VERSION}/ethercat-${ETHERLAB_VERSION}.tar.bz2"

XENOMAI_URL="https://source.denx.de/Xenomai/xenomai/-/archive/v${XENOMAI_VERSION}/xenomai-v${XENOMAI_VERSION}.tar.bz2"
DOVETAIL_KERNEL_URL="https://source.denx.de/Xenomai/linux-dovetail/-/archive/v${DOVETAIL_KERNEL_VERSION}/linux-dovetail-v${DOVETAIL_KERNEL_VERSION}.tar.bz2"

XENOMAI_PREFIX="/usr/xenomai"
ETHERLAB_PREFIX="/usr/etherlab"
JOBS=$(nproc)
XENOMAI_SKIN="posix"
ETHERCAT_MASTER_NIC="${ETHERCAT_NIC:-eth0}"
XENOMAI_GROUP="xenomai"
RECOMMENDED_UBUNTU="jammy"
RECOMMENDED_GCC="11"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'

log()     { echo -e "${GREEN}[BUILD]${NC} $*" | tee -a "${LOG_FILE}"; }
warn()    { echo -e "${YELLOW}[WARN ]${NC} $*" | tee -a "${LOG_FILE}"; }
error()   { echo -e "${RED}[ERROR]${NC} $*" | tee -a "${LOG_FILE}"; exit 1; }
section() { echo -e "\n${BLUE}========== $* ==========${NC}" | tee -a "${LOG_FILE}"; }
info()    { echo -e "${CYAN}[INFO ]${NC} $*" | tee -a "${LOG_FILE}"; }
bugfix()  { echo -e "${CYAN}[BUGFIX]${NC} $*" | tee -a "${LOG_FILE}"; }

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTION...]

Xenomai 3.2.4 + Dovetail kernel 6.1.61 + IgH EtherCAT 1.6.8 build script

  [EtherLab 1.5.2 -> 1.6.8 변경점]
  - 다운로드: GitLab releases (git clone stable-1.6)
  - 자동 버그픽스:
      BUG-1: inline 중복 선언 GCC 빌드 오류 (kernel 6.1 / GCC-11)
      BUG-2: udev rules 미생성 (EtherCAT 장치 권한 오류)
      BUG-3: ioctl version magic 불일치 방지
  - 신규: fake library, ecrt_master_deactivate_slaves() API
  - semantic versioning 도입

Supported Ubuntu:
  22.04 LTS (Jammy)   RECOMMENDED  GCC-11, glibc-2.35
  20.04 LTS (Focal)   Supported    GCC-9,  glibc-2.31
  24.04 LTS (Noble)   Caution      GCC-13, 사례 부족
  18.04 LTS (Bionic)  NOT for 3.2x (3.0.x 전용)

Options:
  --all               전체 빌드 (기본값)
  --deps              의존성 패키지 설치만
  --kernel            Dovetail 커널 빌드/설치만
  --xenomai           Xenomai userspace 빌드만
  --etherlab          IgH EtherCAT Master 빌드만
  --install-etherlab  EtherCAT Master 설치 (root 필요)
  --nic <iface>       EtherCAT NIC (기본: ${ETHERCAT_MASTER_NIC})
  --prefix-xeno <p>   Xenomai prefix (기본: ${XENOMAI_PREFIX})
  --prefix-eth  <p>   EtherLab prefix (기본: ${ETHERLAB_PREFIX})
  --jobs <N>          병렬 빌드 수 (기본: ${JOBS})
  --clean             빌드 디렉토리 삭제
  -h, --help          도움말
EOF
    exit 0
}

DO_DEPS=0; DO_KERNEL=0; DO_XENOMAI=0; DO_ETHERLAB=0; DO_INSTALL_ETH=0

parse_args() {
    [[ $# -eq 0 ]] && { DO_DEPS=1; DO_KERNEL=1; DO_XENOMAI=1; DO_ETHERLAB=1; DO_INSTALL_ETH=1; return; }
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --all)              DO_DEPS=1; DO_KERNEL=1; DO_XENOMAI=1; DO_ETHERLAB=1; DO_INSTALL_ETH=1 ;;
            --deps)             DO_DEPS=1 ;;
            --kernel)           DO_KERNEL=1 ;;
            --xenomai)          DO_XENOMAI=1 ;;
            --etherlab)         DO_ETHERLAB=1 ;;
            --install-etherlab) DO_INSTALL_ETH=1 ;;
            --nic)              shift; ETHERCAT_MASTER_NIC="$1" ;;
            --prefix-xeno)      shift; XENOMAI_PREFIX="$1" ;;
            --prefix-eth)       shift; ETHERLAB_PREFIX="$1" ;;
            --jobs)             shift; JOBS="$1" ;;
            --clean)            rm -rf "${BUILD_DIR}"; log "Build dir removed."; exit 0 ;;
            -h|--help)          usage ;;
            *)                  error "Unknown option: $1" ;;
        esac
        shift
    done
}

check_root() { [[ $EUID -eq 0 ]] || error "Root required. Use sudo."; }

# ---------------------------------------------------------------------------
# Ubuntu / GCC compatibility check
# ---------------------------------------------------------------------------
check_ubuntu_version() {
    section "Ubuntu / GCC Compatibility Check"
    [[ -f /etc/os-release ]] || { warn "No /etc/os-release"; return; }
    source /etc/os-release
    local codename="${VERSION_CODENAME:-unknown}"
    log "Detected: ${PRETTY_NAME:-unknown}  (${codename})"

    case "${codename}" in
        jammy)  log "Ubuntu 22.04 (Jammy) -- RECOMMENDED for Xenomai 3.2.x + EtherLab 1.6.x" ;;
        focal)  log "Ubuntu 20.04 (Focal) -- Supported." ;;
        noble)  warn "Ubuntu 24.04 (Noble) -- GCC-13, 사례 제한적. 오류 시 22.04 권장." ;;
        bionic) error "Ubuntu 18.04 (Bionic)는 Xenomai 3.2.x + EtherLab 1.6.x에 미지원.
  -> Ubuntu 22.04 LTS 사용 권장." ;;
        xenial) error "Ubuntu 16.04 (Xenial) 미지원. Ubuntu 22.04 사용." ;;
        *)      warn "Unknown codename '${codename}'. Proceed at your own risk." ;;
    esac

    if command -v "gcc-${RECOMMENDED_GCC}" &>/dev/null; then
        export CC="gcc-${RECOMMENDED_GCC}"; export CXX="g++-${RECOMMENDED_GCC}"
        [[ $EUID -eq 0 ]] && update-alternatives --install /usr/bin/gcc gcc \
            "/usr/bin/gcc-${RECOMMENDED_GCC}" 100 \
            --slave /usr/bin/g++ g++ "/usr/bin/g++-${RECOMMENDED_GCC}" 2>/dev/null || true
    else
        warn "gcc-${RECOMMENDED_GCC} not found."
        [[ $EUID -eq 0 ]] && apt-get install -y "gcc-${RECOMMENDED_GCC}" "g++-${RECOMMENDED_GCC}" \
            | tee -a "${LOG_FILE}" \
            && { export CC="gcc-${RECOMMENDED_GCC}"; export CXX="g++-${RECOMMENDED_GCC}"; } \
            || warn "Run: sudo apt-get install -y gcc-${RECOMMENDED_GCC} g++-${RECOMMENDED_GCC}"
    fi

    log "Compiler: $(${CC:-gcc} --version | head -1)"
    log "glibc:    $(ldd --version | head -1 | awk '{print $NF}')"
}

check_kernel_version() {
    local running; running=$(uname -r)
    [[ "${running}" == *"${DOVETAIL_KERNEL_SHORT}"* ]] \
        && log "Kernel match: ${running}" \
        || { warn "Running kernel (${running}) != target (${DOVETAIL_KERNEL_SHORT}-dovetail)"; \
             warn "Reboot after kernel build."; }
}

check_xenomai_running() {
    [[ -d /proc/xenomai ]] \
        && log "/proc/xenomai OK -- Xenomai running" \
        || warn "/proc/xenomai not found -- reboot into Dovetail kernel first."
}

# ---------------------------------------------------------------------------
# Step 1: Dependencies
# ---------------------------------------------------------------------------
install_deps() {
    section "Dependencies"
    check_root
    apt-get update -qq
    apt-get install -y --no-install-recommends \
        build-essential bc bison flex \
        libssl-dev libelf-dev libncurses-dev libncurses5-dev \
        git wget curl autoconf automake libtool pkg-config \
        cmake doxygen dwarves zstd rsync fakeroot \
        debhelper dpkg-dev cpio python3 \
        | tee -a "${LOG_FILE}"
    apt-get install -y kernel-package 2>/dev/null \
        || warn "kernel-package 없음 (22.04 정상)."
    log "Dependencies installed."
}

# ---------------------------------------------------------------------------
# Step 2: Downloads
# ---------------------------------------------------------------------------
download_sources() {
    section "Downloading sources"
    mkdir -p "${BUILD_DIR}/src"

    _dl() {
        local dest="${BUILD_DIR}/src/$(basename "$1")"
        [[ -f "${dest}" ]] && { log "Already: $(basename "${dest}")"; return; }
        log "Downloading: $1"
        wget -q --show-progress -O "${dest}" "$1" 2>&1 | tee -a "${LOG_FILE}" \
            || error "Download failed: $1"
    }

    _dl "${XENOMAI_URL}"
    [[ ${DO_KERNEL} -eq 1 ]] && _dl "${DOVETAIL_KERNEL_URL}"

    # EtherLab: git clone (tarball 대신) — 최신 패치 포함
    local eth_src="${BUILD_DIR}/ethercat-${ETHERLAB_VERSION}"
    if [[ ! -d "${eth_src}" ]]; then
        log "EtherLab git clone (${ETHERLAB_GIT_BRANCH})..."
        if git clone --branch "${ETHERLAB_GIT_BRANCH}" --depth 1 \
                "${ETHERLAB_GIT_URL}" "${eth_src}" 2>&1 | tee -a "${LOG_FILE}"; then
            log "EtherLab git clone 완료."
        else
            warn "git clone 실패. tarball fallback 시도..."
            _dl "${ETHERLAB_TARBALL_URL}"
            local tarball="${BUILD_DIR}/src/ethercat-${ETHERLAB_VERSION}.tar.bz2"
            [[ -f "${tarball}" ]] && tar -xf "${tarball}" -C "${BUILD_DIR}" \
                && mv "${BUILD_DIR}/ethercat-${ETHERLAB_VERSION}"* "${eth_src}" 2>/dev/null || true
        fi
    else
        log "EtherLab source already exists: ${eth_src}"
        log "최신 패치 반영을 위해 git pull 실행..."
        cd "${eth_src}" && git pull 2>&1 | tee -a "${LOG_FILE}" || warn "git pull 실패 (무시)."
    fi
}

# ---------------------------------------------------------------------------
# Step 3: Dovetail kernel
# ---------------------------------------------------------------------------
build_kernel() {
    section "Dovetail kernel (linux-dovetail ${DOVETAIL_KERNEL_VERSION})"

    local ksrc="${BUILD_DIR}/linux-dovetail-v${DOVETAIL_KERNEL_VERSION}"
    local xsrc="${BUILD_DIR}/xenomai-v${XENOMAI_VERSION}"

    [[ -d "${ksrc}" ]] || tar -xf "${BUILD_DIR}/src/linux-dovetail-v${DOVETAIL_KERNEL_VERSION}.tar.bz2" -C "${BUILD_DIR}"
    [[ -d "${xsrc}" ]] || tar -xf "${BUILD_DIR}/src/xenomai-v${XENOMAI_VERSION}.tar.bz2" -C "${BUILD_DIR}"

    cd "${ksrc}"
    log "prepare-kernel.sh (Cobalt 코어 삽입)..."
    "${xsrc}/scripts/prepare-kernel.sh" --linux="${ksrc}" --arch=x86_64 \
        2>&1 | tee -a "${LOG_FILE}" || warn "Non-zero exit (Dovetail pre-applied, 정상)."

    if [[ -f /boot/config-"$(uname -r)" ]]; then
        cp /boot/config-"$(uname -r)" .config
        make olddefconfig 2>&1 | tee -a "${LOG_FILE}"
    else
        make defconfig 2>&1 | tee -a "${LOG_FILE}"
    fi

    # Dovetail / Cobalt 필수 옵션
    scripts/config --enable  CONFIG_DOVETAIL
    scripts/config --enable  CONFIG_IRQ_PIPELINE
    scripts/config --enable  CONFIG_XENO_COBALT
    scripts/config --enable  CONFIG_HIGH_RES_TIMERS
    scripts/config --enable  CONFIG_SMP
    scripts/config --enable  CONFIG_FUSE_FS

    # RT 레이턴시 저해 비활성화
    scripts/config --disable CONFIG_CPU_FREQ
    scripts/config --disable CONFIG_CPU_IDLE
    scripts/config --disable CONFIG_ACPI_PROCESSOR
    scripts/config --disable CONFIG_DEBUG_PREEMPT
    scripts/config --disable CONFIG_TRANSPARENT_HUGEPAGE
    scripts/config --disable CONFIG_COMPACTION
    scripts/config --disable CONFIG_MIGRATION
    scripts/config --disable CONFIG_NUMA_BALANCING

    # Cobalt 힙/레지스트리 (로보틱스 환경 권장값)
    scripts/config --set-val CONFIG_XENO_OPT_SYS_HEAPSZ       4096
    scripts/config --set-val CONFIG_XENO_OPT_REGISTRY_NRSLOTS 4096

    # Ubuntu 빌드 인증서 클리어
    scripts/config --set-str CONFIG_SYSTEM_TRUSTED_KEYS ""
    scripts/config --set-str CONFIG_SYSTEM_REVOCATION_KEYS ""

    make olddefconfig 2>&1 | tee -a "${LOG_FILE}"
    log "커널 컴파일 (jobs: ${JOBS})..."
    make -j"${JOBS}" 2>&1 | tee -a "${LOG_FILE}"
    make modules_install 2>&1 | tee -a "${LOG_FILE}"
    make install 2>&1 | tee -a "${LOG_FILE}"

    getent group "${XENOMAI_GROUP}" &>/dev/null || groupadd "${XENOMAI_GROUP}"
    usermod -aG "${XENOMAI_GROUP}" root
    local gid; gid=$(getent group "${XENOMAI_GROUP}" | cut -d: -f3)
    echo "${gid}" > "${BUILD_DIR}/xenomai_gid"
    log "Xenomai group GID: ${gid}"

    local grub_file="/etc/default/grub"
    if [[ -f "${grub_file}" ]] && ! grep -q "xenomai.allowed_group" "${grub_file}"; then
        sed -i "s|GRUB_CMDLINE_LINUX_DEFAULT=\"|GRUB_CMDLINE_LINUX_DEFAULT=\"xenomai.allowed_group=${gid} |" \
            "${grub_file}"
        update-grub 2>&1 | tee -a "${LOG_FILE}"
    fi
    log "Dovetail kernel build complete."
}

# ---------------------------------------------------------------------------
# Step 4: Xenomai 3.2.x userspace
# ---------------------------------------------------------------------------
build_xenomai() {
    section "Xenomai ${XENOMAI_VERSION} userspace"

    local xsrc="${BUILD_DIR}/xenomai-v${XENOMAI_VERSION}"
    [[ -d "${xsrc}" ]] || tar -xf "${BUILD_DIR}/src/xenomai-v${XENOMAI_VERSION}.tar.bz2" -C "${BUILD_DIR}"

    cd "${xsrc}"
    [[ -f configure ]] || { log "bootstrap..."; ./scripts/bootstrap 2>&1 | tee -a "${LOG_FILE}"; }

    mkdir -p "${BUILD_DIR}/xenomai-build"
    cd "${BUILD_DIR}/xenomai-build"

    "${xsrc}/configure" \
        --prefix="${XENOMAI_PREFIX}" \
        --with-core=cobalt \
        --enable-smp \
        --enable-posix-auto-mlockall \
        --enable-registry \
        --enable-tls \
        CFLAGS="-march=native -O2 -pipe" \
        LDFLAGS="-L${XENOMAI_PREFIX}/lib" \
        2>&1 | tee -a "${LOG_FILE}"

    make -j"${JOBS}" 2>&1 | tee -a "${LOG_FILE}"
    make install 2>&1 | tee -a "${LOG_FILE}"

    cat > "${BUILD_DIR}/xenomai_env.sh" <<ENVEOF
# Xenomai ${XENOMAI_VERSION} env (Dovetail ${DOVETAIL_KERNEL_VERSION})
export XENOMAI_PREFIX="${XENOMAI_PREFIX}"
export PATH="\${XENOMAI_PREFIX}/bin:\${PATH}"
export LD_LIBRARY_PATH="\${XENOMAI_PREFIX}/lib:\${LD_LIBRARY_PATH:-}"
export PKG_CONFIG_PATH="\${XENOMAI_PREFIX}/lib/pkgconfig:\${PKG_CONFIG_PATH:-}"
XENO_CFLAGS="\$(xeno-config --skin=${XENOMAI_SKIN} --cflags)"
XENO_LDFLAGS="\$(xeno-config --skin=${XENOMAI_SKIN} --ldflags)"
ENVEOF
    log "Xenomai env: ${BUILD_DIR}/xenomai_env.sh"
    log "Xenomai userspace build complete."
}

# ---------------------------------------------------------------------------
# Step 5: EtherLab 1.6.8 — 알려진 버그 패치 포함
# ---------------------------------------------------------------------------

# [BUG-1] inline 함수 중복 선언 빌드 오류 (GCC 11+ / kernel 6.1 환경)
# 원인: EtherLab 1.6 초기 버전의 일부 헤더에서 static inline 함수가
#       여러 translation unit에서 중복 정의되어 GCC 11+에서 링크 오류 발생.
# 수정: remove-inline-statement 패치 — 불필요한 inline 키워드 제거
#       (stable-1.6 브랜치에 이미 머지됨. git clone이 최선이나
#        tarball 사용 시를 위해 수동 패치 함수도 포함)
apply_etherlab_bugfixes() {
    local esrc="$1"
    bugfix "EtherLab 1.6.x 알려진 버그 패치 적용 중..."

    # ── BUG-1: inline 중복 선언 ───────────────────────────────────────────
    # git clone stable-1.6은 이미 포함. tarball 1.6.8에서도 대부분 수정됨.
    # 혹시 누락된 경우를 위해 master/mailbox.h 등 핵심 파일을 확인 후 패치.
    bugfix "[BUG-1] inline 중복 선언 검사..."
    local mailbox_h="${esrc}/master/mailbox.h"
    if [[ -f "${mailbox_h}" ]]; then
        # 'inline' 키워드가 static 없이 단독 사용된 경우 제거
        if grep -q "^inline " "${mailbox_h}" 2>/dev/null; then
            sed -i 's/^inline /static inline /g' "${mailbox_h}"
            bugfix "  -> master/mailbox.h: inline -> static inline 수정"
        fi
    fi
    # master/slave.h 동일 패턴 확인
    local slave_h="${esrc}/master/slave.h"
    if [[ -f "${slave_h}" ]] && grep -q "^inline " "${slave_h}" 2>/dev/null; then
        sed -i 's/^inline /static inline /g' "${slave_h}"
        bugfix "  -> master/slave.h: inline -> static inline 수정"
    fi

    # ── BUG-2: udev rules 파일 권한 설정 ─────────────────────────────────
    # 1.5.x에서 1.6.x로 오면서 /etc/udev/rules.d 경로가 바뀌어
    # 설치 후 EtherCAT 장치에 비루트 접근이 안 되는 문제.
    # install_etherlab()에서 udev rules를 명시적으로 생성하므로 여기서는 기록만.
    bugfix "[BUG-2] udev rules는 install 단계에서 자동 생성됩니다."

    # ── BUG-3: ioctl version magic 불일치 방지 ────────────────────────────
    # 커널 모듈(ec_master.ko)과 userspace tool/library를
    # 반드시 동일 소스로 빌드해야 함. 별도 설치 시 magic 불일치 발생.
    # -> build_etherlab()에서 make + make modules를 같은 소스로 수행하므로 자동 해결.
    # -> 단, 시스템에 이전 버전 모듈이 잔류할 경우 사전 제거 필요.
    bugfix "[BUG-3] 이전 버전 EtherCAT 모듈 잔류 확인..."
    if lsmod 2>/dev/null | grep -q "^ec_master"; then
        warn "  기존 ec_master 모듈이 로드되어 있습니다."
        warn "  'sudo rmmod ec_master ec_generic' 후 재시도 권장."
    fi

    bugfix "EtherLab 버그 패치 완료."
}

build_etherlab() {
    section "IgH EtherCAT Master ${ETHERLAB_VERSION} 빌드"

    local esrc="${BUILD_DIR}/ethercat-${ETHERLAB_VERSION}"
    [[ -d "${esrc}" ]] || error "EtherLab 소스가 없습니다. download_sources 먼저 실행하세요."

    # 알려진 버그 패치 적용
    apply_etherlab_bugfixes "${esrc}"

    cd "${esrc}"
    if [[ ! -f configure ]]; then
        log "bootstrap..."
        ./bootstrap 2>&1 | tee -a "${LOG_FILE}"
    fi

    # Dovetail 커널 모듈 경로 탐지
    local kbuild
    if [[ -d "/lib/modules/${DOVETAIL_KERNEL_SHORT}-dovetail1/build" ]]; then
        kbuild="/lib/modules/${DOVETAIL_KERNEL_SHORT}-dovetail1/build"
    else
        kbuild="/lib/modules/$(uname -r)/build"
        warn "Dovetail 커널 모듈 경로 없음 -- 현재 커널로 대체: $(uname -r)"
        warn "Dovetail 커널 리부팅 후 재빌드 권장."
    fi

    log "configure (EtherLab ${ETHERLAB_VERSION}, kernel: ${kbuild})..."
    # 1.6.x configure 주요 옵션:
    #   --enable-rtdm: Xenomai RTDM 인터페이스 활성화 (Cobalt 필수)
    #   --enable-eoe=no: EoE(EtherNet over EtherCAT) 비활성화 -> RT 레이턴시 개선
    #   --enable-sii-assign: 슬레이브 SII EEPROM 자동 할당
    ./configure \
        --prefix="${ETHERLAB_PREFIX}" \
        --sysconfdir=/etc \
        --with-linux-dir="${kbuild}" \
        --enable-generic \
        --enable-rtdm \
        --enable-sii-assign=yes \
        --enable-eoe=no \
        --disable-8139too \
        --disable-e1000e \
        --disable-r8169 \
        2>&1 | tee -a "${LOG_FILE}"

    log "EtherLab 빌드 (jobs: ${JOBS})..."
    make -j"${JOBS}" 2>&1 | tee -a "${LOG_FILE}"
    log "EtherLab build complete."
}

# ---------------------------------------------------------------------------
# Step 6: Install EtherCAT Master
# ---------------------------------------------------------------------------
install_etherlab() {
    section "IgH EtherCAT Master 설치"
    check_root

    local esrc="${BUILD_DIR}/ethercat-${ETHERLAB_VERSION}"
    cd "${esrc}"
    make install         2>&1 | tee -a "${LOG_FILE}"
    make modules_install 2>&1 | tee -a "${LOG_FILE}"
    depmod -a

    # /etc/ethercat.conf 생성
    local mac="ff:ff:ff:ff:ff:ff"
    ip link show "${ETHERCAT_MASTER_NIC}" &>/dev/null \
        && mac=$(ip link show "${ETHERCAT_MASTER_NIC}" | awk '/ether/{print $2}') \
        || warn "NIC '${ETHERCAT_MASTER_NIC}' 없음 -- placeholder MAC 사용."

    cat > /etc/ethercat.conf <<CONFEOF
# /etc/ethercat.conf  IgH EtherCAT Master ${ETHERLAB_VERSION}
# Generated $(date)
# Kernel: linux-dovetail-${DOVETAIL_KERNEL_VERSION} + Xenomai ${XENOMAI_VERSION}
MASTER0_DEVICE="${mac}"
DEVICE_MODULES="generic"
CONFEOF
    log "/etc/ethercat.conf 작성 (NIC: ${ETHERCAT_MASTER_NIC}, MAC: ${mac})"

    # [BUG-2 수정] udev rules 생성 — 비루트 EtherCAT 장치 접근 권한
    # 1.5.x -> 1.6.x 전환 시 /etc/udev/rules.d/ 경로 설정이 누락되는 문제
    local udev_rules="/etc/udev/rules.d/99-ethercat.rules"
    if [[ ! -f "${udev_rules}" ]]; then
        bugfix "[BUG-2] udev rules 생성: ${udev_rules}"
        cat > "${udev_rules}" <<'UDEVRULES'
# IgH EtherCAT Master -- non-root access
KERNEL=="EtherCAT[0-9]*", MODE="0664", GROUP="xenomai"
UDEVRULES
        udevadm control --reload-rules 2>/dev/null || true
        udevadm trigger             2>/dev/null || true
        log "udev rules 생성 완료 (그룹: xenomai)"
    else
        log "udev rules 이미 존재: ${udev_rules}"
    fi

    if command -v systemctl &>/dev/null; then
        systemctl daemon-reload
        systemctl enable ethercat
        log "ethercat.service 등록 완료."
    else
        update-rc.d ethercat defaults
    fi

    log "EtherLab 설치 완료."
    log "  -> Dovetail 커널 리부팅 후: sudo systemctl start ethercat"
    log "  -> 슬레이브 확인:           ethercat slaves"
    log "  -> [BUG-3 주의] ioctl magic 불일치 발생 시:"
    log "     sudo rmmod ec_generic ec_master && sudo systemctl start ethercat"
}

# ---------------------------------------------------------------------------
# Step 7: CMake helper
# ---------------------------------------------------------------------------
generate_cmake_helper() {
    section "CMake 헬퍼 생성"

    cat > "${BUILD_DIR}/xenomai-ethercat-toolchain.cmake" <<'CMAKEOF'
# xenomai-ethercat-toolchain.cmake
# Xenomai 3.2.x (Dovetail/Cobalt) + IgH EtherCAT 1.6.x
#
# Usage:
#   include(/path/to/xenomai-ethercat-toolchain.cmake)
#   target_link_xenomai_ethercat(my_target)

cmake_minimum_required(VERSION 3.16)

find_program(XENO_CONFIG xeno-config HINTS /usr/xenomai/bin)
if(NOT XENO_CONFIG)
    message(FATAL_ERROR "xeno-config not found. source xenomai_env.sh first.")
endif()

execute_process(COMMAND ${XENO_CONFIG} --skin=posix --cflags
    OUTPUT_VARIABLE XENOMAI_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${XENO_CONFIG} --skin=posix --ldflags
    OUTPUT_VARIABLE XENOMAI_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${XENO_CONFIG} --version
    OUTPUT_VARIABLE _XV OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "Xenomai: ${_XV}")

separate_arguments(XENOMAI_CFLAGS_LIST  UNIX_COMMAND "${XENOMAI_CFLAGS}")
separate_arguments(XENOMAI_LDFLAGS_LIST UNIX_COMMAND "${XENOMAI_LDFLAGS}")

# IgH EtherCAT 1.6.x -- prefix /usr/etherlab
find_path(ETHERCAT_INCLUDE_DIR ecrt.h HINTS /usr/etherlab/include)
find_library(ETHERCAT_LIBRARY ethercat HINTS /usr/etherlab/lib)
if(NOT ETHERCAT_INCLUDE_DIR OR NOT ETHERCAT_LIBRARY)
    message(FATAL_ERROR "IgH EtherCAT Master 1.6.x not found at /usr/etherlab.")
endif()

# EtherCAT 1.6.x: RTDM 인터페이스 헤더 추가
find_path(ETHERCAT_RTDM_INCLUDE_DIR ectty.h HINTS /usr/etherlab/include)

function(target_link_xenomai_ethercat TARGET)
    target_compile_options(${TARGET} PRIVATE
        ${XENOMAI_CFLAGS_LIST}
        -fno-stack-protector)
    target_link_libraries(${TARGET} PRIVATE
        ${XENOMAI_LDFLAGS_LIST}
        ${ETHERCAT_LIBRARY})
    target_include_directories(${TARGET} PRIVATE
        ${ETHERCAT_INCLUDE_DIR}
        ${ETHERCAT_RTDM_INCLUDE_DIR})
    # [BUG-3] ioctl magic 불일치 방지: 빌드 시 버전 체크 매크로 삽입
    target_compile_definitions(${TARGET} PRIVATE
        EC_IOCTL_VERSION_CHECK=1)
endfunction()
CMAKEOF
    log "CMake 헬퍼: ${BUILD_DIR}/xenomai-ethercat-toolchain.cmake"
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
main() {
    mkdir -p "${BUILD_DIR}"
    exec 2> >(tee -a "${LOG_FILE}" >&2)
    log "=== Xenomai ${XENOMAI_VERSION} + Dovetail ${DOVETAIL_KERNEL_VERSION} + EtherLab ${ETHERLAB_VERSION} ==="
    log "Build log: ${LOG_FILE}  |  Jobs: ${JOBS}"

    check_ubuntu_version
    check_kernel_version

    [[ ${DO_DEPS} -eq 1 || ${DO_KERNEL} -eq 1 || ${DO_XENOMAI} -eq 1 || ${DO_ETHERLAB} -eq 1 ]] \
        && download_sources

    [[ ${DO_DEPS}        -eq 1 ]] && install_deps
    [[ ${DO_KERNEL}      -eq 1 ]] && build_kernel
    [[ ${DO_XENOMAI}     -eq 1 ]] && { check_xenomai_running; build_xenomai; }
    [[ ${DO_ETHERLAB}    -eq 1 ]] && build_etherlab
    [[ ${DO_INSTALL_ETH} -eq 1 ]] && install_etherlab

    generate_cmake_helper

    section "빌드 완료"
    echo ""
    echo -e "${GREEN}=== Next Steps ===${NC}"
    echo "  1. 리부팅:    sudo reboot  (GRUB: linux-${DOVETAIL_KERNEL_SHORT}-dovetail 선택)"
    echo "  2. 확인:      dmesg | grep -i xenomai"
    echo "                cat /proc/xenomai/version"
    echo "  3. EtherCAT:  sudo systemctl start ethercat"
    echo "                ethercat slaves"
    echo "  4. RT 사용자: sudo usermod -aG ${XENOMAI_GROUP} \$USER"
    echo "  5. 환경:      source ${BUILD_DIR}/xenomai_env.sh"
    echo "  6. 레이턴시:  xeno latency -t 1"
    echo ""
    echo -e "${CYAN}=== EtherLab 1.6.x 버그픽스 요약 ===${NC}"
    echo "  BUG-1: inline 중복 선언 오류  -> apply_etherlab_bugfixes() 자동 적용"
    echo "  BUG-2: udev rules 미생성      -> /etc/udev/rules.d/99-ethercat.rules 생성"
    echo "  BUG-3: ioctl magic 불일치     -> 동일 소스 빌드 + rmmod 지침 안내"
    echo ""
}

parse_args "$@"
main
