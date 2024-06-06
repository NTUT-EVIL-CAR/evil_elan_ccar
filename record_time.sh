#!/bin/bash

DEFAULT_TIME=10
storage_base_path="./bag_record/$(date +"%Y-%m-%d")"
situation=""

if [ ! -d  $storage_base_path ]; then
    mkdir -p  $storage_base_path
fi

function record_data() {
    record_situation=$2
    storage_path="${storage_base_path}/${record_situation}"
    gnome-terminal --tab -t "record" -- bash -c "rosbag record --duration=$1 -o $storage_path /can0/received_msg /can1/received_msg /can2/received_msg /cme_cam /rslidar_points /imu /fix /velodyne_points;"
    echo -e "Record successfully!\n"
}

function change_situation() {
    road_type=""
    weather=""
    time_period=""

    echo -e "+----------------------------------+"
    echo -e "|   situation|        0|          1|"
    echo -e "+----------------------------------+"
    echo -e "|   road_type| heighway| citystreet|"
    echo -e "|     weather|    sunny|      rainy|"
    echo -e "| time_period|      day|      night|"
    echo -e "+----------------------------------+"

    while true; do
        read -p "Enter road_type: " road_type
        case $road_type in
            0) road_type="heighway"; break ;;
            1) road_type="citystreet"; break ;;
            *) echo "invalid road_type" ;;
        esac        
    done

    while true; do
        read -p "Enter weather: " weather
        case $weather in
            0) weather="sunny"; break ;;
            1) weather="rainy"; break ;;
            *) echo "invalid weather" ;;
        esac
    done

    while true; do
        read -p "Enter time_period: " time_period
        case $time_period in
            0) time_period="day"; break ;;
            1) time_period="night"; break ;;
            *) echo "invalid time_period" ;;
        esac
    done

    situation="${road_type}_${weather}_${time_period}"
    echo -e "Setting situation as: $situation\n"
}

echo -e "Enter 'r' to record (default time $DEFAULT_TIME sec)\nEnter 'q' to quit\nEnter 'c' to change situation\nEnter 't' to record in tunnel\n"
change_situation

while true; do
    read -p "Enter option: " key

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
            
            record_data "$time" "$situation"
            ;;
            
        [qQ])
            echo -e "Exiting the record\n"
            exit 0
            ;;

        [cC])
            echo -e "Setting situation\n"
            change_situation
            ;;

        [tT])
            echo "In tunnel"
            if [ -z "$setTime" ]; then
                time=$DEFAULT_TIME
                echo "No set time, record time => $DEFAULT_TIME sec"
            else
                time="$setTime"
                echo "Set record time => $setTime sec"
            fi
            
            record_data "$time" "tunnel"
            ;;
        *)
            echo "$key invalid"
            ;;
    esac
done