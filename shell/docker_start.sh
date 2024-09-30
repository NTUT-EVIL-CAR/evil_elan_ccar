# Set X display variables
set_x_display() {
    MOUNT_X="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix"
    xhost +local:root >/dev/null
}

# Main script execution
main() {
    set_x_display

    # Launch the container
    set -x
    docker run \
        -it \
        --rm \
        --net=host \
        --ipc=host \
        --pid=host \
        --gpus all \
        ${MOUNT_X} \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e=DISPLAY \
        -e=ROS_DOMAIN_ID \
        -v $(realpath ~/glim):/glim \
        -v $(realpath ~/glim_map):/glim_map \
        --entrypoint /bin/bash \
        koide3/glim_ros2:humble_cuda12.2
        # -c "source ~/ros2_ws/install/setup.bash && exec bash"
}

# Execute the main script
main
