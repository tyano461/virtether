KIND=$1

if [ ""$KIND = "" ] ; then
    sudo rmmod virtether
    sudo kill -9 $(pidof dummy_stub)
    make clean
else
    make
    sudo insmod virtether.ko
    sudo ifconfig eth0 up
    sudo ip a add 192.168.100.1/24 dev eth0
    sudo ./dummy_stub &

    cat << EOF
    try:
        ./sample_app
        then input to stdin
EOF
fi
