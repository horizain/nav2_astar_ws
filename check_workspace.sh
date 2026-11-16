#!/usr/bin/env bash
# ============================================================
#  install 前缀自检 / 定点修复（nav2_astar_ws）
#
#  典型症状：
#    - source install/setup.bash 报
#        not found: ".../install/nav2_controller_lab/share/.../local_setup.bash"
#    - ros2 launch 报 package not found，但 install/<pkg>/ 目录明明存在
#    - AMENT_PREFIX_PATH 里间歇性缺少某个包，换一个终端又好了
#
#  根因（本机 2026-11-15 实测）：
#    ament_cmake 包用 --symlink-install 构建后，install/<pkg> 下的
#    local_setup.*、ament_index、cmake、lib 都是指向 build/<pkg> 的软链接。
#    之后只要 build/<pkg> 被清理（例如“僵尸进程大清洗”误删 build 目录），
#    这些链接就会悬挂：
#      * local_setup.dsv 悬挂 -> 该前缀不会加入 AMENT_PREFIX_PATH
#      * resource_index/packages/<pkg> 悬挂 -> ros2 pkg prefix 报 not found
#    普通 colcon build 往往先重建 build/<pkg> 里的目标文件，install 时软链接
#    又变“有效”了，于是继续保留链接；必须连 install/<pkg> 一起删掉再构建，
#    才会得到真实文件，彻底断掉对 build/ 的依赖。
#
#  用法：
#    ./check_workspace.sh          # 只检查，不修改任何文件
#    ./check_workspace.sh --fix    # 备份损坏的 install/<pkg>，清理 build/<pkg>，重建
#
#  --fix 的备份在 ${TMPDIR:-/tmp}/nav2_astar_ws_install_backup/，不会留在 install/。
# ============================================================
set -u

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
cd "$WS_DIR" || exit 1

CLEAN_PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$HOME/.local/bin"
BACKUP_ROOT="${TMPDIR:-/tmp}/nav2_astar_ws_install_backup"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
PARENT_SETUP="$HOME/ros2_jazzy/install/setup.bash"

MODE="check"
case "${1:-}" in
    ""|--check) MODE="check" ;;
    --fix)      MODE="fix" ;;
    -h|--help)
        awk 'NR > 1 && /^set -u$/{exit} NR > 1{print}' "$0"
        exit 0
        ;;
    *)
        echo "未知参数: $1（可用: --check / --fix）" >&2
        exit 2
        ;;
esac

mapfile -t PKGS < <(
    for f in src/*/package.xml; do
        [ -f "$f" ] || continue
        sed -n 's:.*<name>[[:space:]]*\([^<]*[^[:space:]<]\)[[:space:]]*</name>.*:\1:p' "$f" | head -n1
    done | sort -u
)

if [ "${#PKGS[@]}" -eq 0 ]; then
    echo "[ERROR] src/*/package.xml 里没有找到任何包" >&2
    exit 2
fi

# 在全新 shell 里按三级 source，输出：
#   __SOURCE_BEGIN__ / __SOURCE_END__ 之间的 "not found" 行
#   每个包的 PKG <name> => <prefix|NOT_FOUND>
clean_env_check() {
    env -i HOME="$HOME" PATH="$CLEAN_PATH" TERM=dumb bash --noprofile --norc -c '
        [ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
        if [ -f "$HOME/ros2_jazzy/install/setup.bash" ]; then
            source "$HOME/ros2_jazzy/install/setup.bash"
        fi
        echo "__SOURCE_BEGIN__"
        if [ -f "$1/install/setup.bash" ]; then
            source "$1/install/setup.bash"
        fi
        echo "__SOURCE_END__"
        shift
        for p in "$@"; do
            if pfx="$(ros2 pkg prefix "$p" 2>/dev/null)"; then
                printf "PKG %s => %s\n" "$p" "$pfx"
            else
                printf "PKG %s => NOT_FOUND\n" "$p"
            fi
        done
    ' _ "$WS_DIR" "${PKGS[@]}" 2>&1
}

# 返回该包 install 前缀的静态问题（空字符串=健康）
file_problems_for() {
    local pkg="$1"
    local pfx="$WS_DIR/install/$pkg"
    local n

    if [ ! -d "$pfx" ]; then
        echo "install/$pkg 不存在"
        return
    fi

    n="$(find "$pfx" -xtype l 2>/dev/null | wc -l)"
    if [ "$n" -gt 0 ]; then
        echo "有 $n 个悬挂软链接（目标 build/$pkg 已丢失）:"
        find "$pfx" -xtype l -printf '    %P -> %l\n' 2>/dev/null | head -n 3
    fi

    [ -f "$pfx/share/ament_index/resource_index/packages/$pkg" ] \
        || echo "share/ament_index/resource_index/packages/$pkg 缺失或悬挂"
    [ -f "$pfx/share/$pkg/local_setup.dsv" ] \
        || echo "share/$pkg/local_setup.dsv 缺失或悬挂（会丢 AMENT_PREFIX_PATH）"
}

echo "=================================================================="
echo " nav2_astar_ws 安装前缀自检"
echo " 工作空间: $WS_DIR"
echo " 模式    : $MODE"
echo "=================================================================="

declare -a BROKEN=()
declare -A SEEN=()

echo
echo "[1/3] install/<pkg> 文件完整性"
for pkg in "${PKGS[@]}"; do
    problems="$(file_problems_for "$pkg")"
    if [ -z "$problems" ]; then
        printf '  [OK]   %s\n' "$pkg"
    else
        printf '  [FAIL] %s\n' "$pkg"
        printf '%s\n' "$problems" | sed 's/^/         /'
        BROKEN+=("$pkg")
        SEEN["$pkg"]=1
    fi
done

echo
echo "[2/3] 干净 shell 三级 source + ros2 pkg prefix"
ENV_OUT="$(clean_env_check)"
printf '%s\n' "$ENV_OUT" | sed -n '/__SOURCE_BEGIN__/,/__SOURCE_END__/p' | grep -v '__SOURCE_' | sed 's/^/  | /'
if printf '%s\n' "$ENV_OUT" | grep -q '^not found:'; then
    printf '  [FAIL] source 过程中有 not found（见上面 | 开头的行）\n'
else
    printf '  [OK]   source 过程无 not found\n'
fi

for pkg in "${PKGS[@]}"; do
    expected="$WS_DIR/install/$pkg"
    got="$(printf '%s\n' "$ENV_OUT" | grep -F "PKG $pkg => " | head -n1)"
    got="${got#*=> }"
    if [ "$got" = "$expected" ]; then
        printf '  [OK]   ros2 pkg prefix %-26s => %s\n' "$pkg" "$got"
    else
        printf '  [FAIL] ros2 pkg prefix %-26s => %s（期望 %s）\n' \
            "$pkg" "${got:-<空>}" "$expected"
        if [ -z "${SEEN[$pkg]:-}" ]; then
            BROKEN+=("$pkg")
            SEEN["$pkg"]=1
        fi
    fi
done

echo
echo "[3/3] 结论"
if [ "${#BROKEN[@]}" -eq 0 ]; then
    echo "  [OK] 所有 install 前缀健康，三级 source 后所有包都能解析。"
    exit 0
fi

echo "  [FAIL] 以下包的 install 前缀损坏：${BROKEN[*]}"
echo
echo "  安全修复（会先备份到 $BACKUP_ROOT）："
echo "    cd \"$WS_DIR\" && ./check_workspace.sh --fix"
echo
echo "  等价手工步骤（以 nav2_controller_lab 为例）："
echo "    mv install/nav2_controller_lab /tmp/          # 不要只删 build/"
echo "    rm -rf build/nav2_controller_lab"
echo "    source /opt/ros/jazzy/setup.bash"
echo "    source ~/ros2_jazzy/install/setup.bash"
echo "    colcon build --packages-select nav2_controller_lab \\"
echo "      --cmake-args -DCMAKE_BUILD_TYPE=Release"
echo "    source install/setup.bash && ros2 pkg prefix nav2_controller_lab"
echo
echo "  注意：不要用 --symlink-install 重建，否则下次再删 build/ 还会复发。"

if [ "$MODE" != "fix" ]; then
    exit 1
fi

# ---------------- 定点修复 ----------------
echo
echo "=================================================================="
echo " 开始修复: ${BROKEN[*]}"
echo "=================================================================="

mkdir -p "$BACKUP_ROOT"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"

for pkg in "${BROKEN[@]}"; do
    if [ -d "install/$pkg" ]; then
        backup="$BACKUP_ROOT/${pkg}.${TIMESTAMP}.$$"
        echo "  备份 install/$pkg -> $backup"
        mv "install/$pkg" "$backup" || {
            echo "[ERROR] 备份失败，中止" >&2
            exit 1
        }
    fi
    echo "  清理 build/$pkg"
    rm -rf "build/$pkg"
done

echo
echo " 重建: ${BROKEN[*]}（普通 install，非 symlink-install）"
env -i HOME="$HOME" PATH="$CLEAN_PATH" TERM=dumb bash --noprofile --norc -c '
    [ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
    if [ -f "$HOME/ros2_jazzy/install/setup.bash" ]; then
        source "$HOME/ros2_jazzy/install/setup.bash"
    fi
    cd "$1" || exit 1
    shift
    colcon build --packages-select "$@" --cmake-args -DCMAKE_BUILD_TYPE=Release
' _ "$WS_DIR" "${BROKEN[@]}"
BUILD_RC=$?

if [ "$BUILD_RC" -ne 0 ]; then
    echo
    echo "[ERROR] 重建失败（rc=$BUILD_RC）。损坏前缀已备份到：$BACKUP_ROOT" >&2
    echo "        修好编译错误后重新执行: $0 --fix" >&2
    exit "$BUILD_RC"
fi

echo
echo "重建完成，重新执行自检："
exec bash "$0" --check
