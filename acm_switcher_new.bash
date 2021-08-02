OUTPUT=()
for tbl in $(dmesg | grep tty)
do
        OUTPUT+=($tbl)
done

#for i in "${OUTPUT[@]}"
#do
#       echo $i
#done
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
for (( idx=${#OUTPUT[@]}-1 ; idx>=0 ; idx-- )) ; do
        if [[ ${OUTPUT[idx]} == "ttyACM0:" ]]; then
                echo "ACM0!"
                gazebo Tools/sitl_gazebo/worlds/hitl_iris_acm0.world
                break
        fi
        if [[ ${OUTPUT[idx]} == "ttyACM1:" ]]; then
                echo "ACM1!"
                gazebo Tools/sitl_gazebo/worlds/hitl_iris_acm1.world
                break
        fi
        if [[ ${OUTPUT[idx]} == "ttyACM2:" ]]; then
                echo "ACM2!"
                gazebo Tools/sitl_gazebo/worlds/hitl_iris_acm2.world
                break
        fi
done

