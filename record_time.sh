#!/bin/bash

DEFAULT_TIME=10
storage_path="./bag_record/$(date +"%Y-%m-%d")/EVIL"
if [ ! -d "$(dirname "$storage_path")" ]; then
    mkdir -p "$(dirname "$storage_path")"
fi

function record_data() {
    gnome-terminal --tab -t "record" -- bash -c "rosbag record --duration=$1 -o $2 /can0/received_msg /can1/received_msg /can2/received_msg /cme_cam /rslidar_points /imu /fix /velodyne_points;"
    echo -e "Record successfully!\n"
}

while true; do
    read -p "Enter 'r' to record (default time $DEFAULT_TIME sec), or 'q' to quit: " key

    option=${key:0:1}
    setTime=${key:2}

    case $option in
        [rR])
            if [ -z "$setTime" ]; then
                time=$DEFAULT_TIME
                echo "No set time, record time => $DEFAULT_TIME sec"
            else
                time="$setTime"
                echo "Set record time => $setTime sec"
            fi
            
            echo "option = $option , setTime = $time"
            record_data "$time" "$storage_path"
            ;;
            
        [qQ])
            echo "Exiting the record."
            exit 0
            ;;

        *)
            echo "$key invalid"
            ;;
    esac
done