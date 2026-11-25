#!/usr/bin/env bash
# Run in the dedicated Gazebo container after building the separate ROS overlay.
set -eo pipefail

output_dir=${1:?usage: run_iiwa14_gz_descent.sh OUTPUT_DIR none|white|all [SEED]}
mode=${2:?specify none, white or all}
if [[ ! ${ROS_DOMAIN_ID:-0} =~ ^[1-9][0-9]*$ ]]; then
    echo "Set a nonzero ROS_DOMAIN_ID" >&2
    exit 1
fi

here=$(cd "$(dirname "$0")" && pwd)
package=$(cd "$here/../.." && pwd)
repo=$(cd "$package/../.." && pwd)
work=${IIWA14_DATA_ROOT:-/workspace/work-docker-jazzy}
overlay=${IIWA14_GZ_OVERLAY:-/workspace/ros2-work-jazzy-iiwa14-gz}
mjcf="$work/model-forge/iiwa14/install/iiwa14.environment.xml"
goal="$work/tmp-iiwa14-test/descent_goal.json"
fitted="$repo/evaluation/force-torque/kuka-lbr-med14/compare/config/gz-ati-gamma-fitted.sdf"
source /opt/ros/jazzy/setup.bash
source "$overlay/install/setup.bash"
export PYTHONDONTWRITEBYTECODE=1

scratch=$(mktemp -d)
launch_pid=
stop_launch() {
    if [[ -n "$launch_pid" ]]; then
        # The launch process and its children use a dedicated process group.
        kill -INT -- "-$launch_pid" 2>/dev/null || true
        for _ in {1..30}; do
            if ! kill -0 -- "-$launch_pid" 2>/dev/null; then break; fi
            sleep 0.1
        done
        kill -TERM -- "-$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
        launch_pid=
    fi
}
cleanup() {
    stop_launch
    rm -f "$scratch/iiwa14.urdf"
    rmdir "$scratch"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 "$package/tools/mjcf_to_urdf.py" "$mjcf" base "$scratch/iiwa14.urdf" --robot iiwa14
mkdir -p "$output_dir"
set -o noclobber

run_trial() {
    local noise=$1 seed=$2 stem="$output_dir/descent-gz-$1"
    if [[ "$noise" == white ]]; then
        stem="$stem-s$seed"
    fi
    if [[ -e "$stem.npz" || -e "$stem.json" || -e "$stem.launch.log" ]]; then
        echo "Output already exists for $stem" >&2
        return 1
    fi
    setsid env --default-signal=INT,QUIT ros2 launch hakoniwa_ros2_control iiwa14_gz.launch.py \
        "mjcf:=$mjcf" "kinematic_urdf:=$scratch/iiwa14.urdf" "noise_sdf:=$fitted" \
        "noise:=$noise" "seed:=$seed" "real_time_factor:=${IIWA14_GZ_RTF:-1}" \
        > "$stem.launch.log" 2>&1 &
    launch_pid=$!
    python3 "$here/record_iiwa14_gz_descent.py" "$goal" "$stem.npz"
    stop_launch
}

case "$mode" in
    none) run_trial none 1 ;;
    white)
        seed=${3:?specify a seed}
        if [[ ! $seed =~ ^([1-9]|1[0-9]|20)$ ]]; then
            echo "Expected a white-noise seed from 1 to 20" >&2
            exit 1
        fi
        run_trial white "$seed"
        ;;
    all)
        run_trial none 1
        for seed in {1..20}; do
            run_trial white "$seed"
        done
        ;;
    *) echo "Expected none, white or all" >&2; exit 1 ;;
esac
