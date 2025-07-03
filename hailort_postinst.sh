#!/bin/bash

# Custom version of this script for Docker that does not attempt to start the service.

set -eEuo pipefail

readonly HAILORT_SERVICE_NAME="hailort.service"
readonly PKG_NAME="hailort"
readonly LOG="/var/log/${PKG_NAME}.deb.log"; echo "######### $(date) #########" >> $LOG
trap 'echo "Failed. Exited with status $?. See $LOG" | tee -a $LOG; echo "Failed at ${LINENO}" >> $LOG; pr -tn $0 | tail -n+$((LINENO - 3)) | head -n7 >> $LOG' ERR

readonly VERSION=$(dpkg -s ${PKG_NAME} | grep '^Version: ' | awk '{print $NF}')

function install_libhailort() {
    # adds a symlink: libhailort.so -> libhailort.so.x.y.z
    ln -sf /usr/lib/libhailort.so.${VERSION} /usr/lib/libhailort.so >> $LOG
}

function start_hailort_service(){
    echo "Starting $HAILORT_SERVICE_NAME"
    systemctl restart $HAILORT_SERVICE_NAME
    systemctl enable $HAILORT_SERVICE_NAME
    check_service_status "active"
}

function stop_hailort_service(){
    echo "Stopping $HAILORT_SERVICE_NAME"
    systemctl stop $HAILORT_SERVICE_NAME
    systemctl disable $HAILORT_SERVICE_NAME
    check_service_status "inactive"
}

function check_service_status(){
    expected_status=$1
    if (systemctl -q is-active $HAILORT_SERVICE_NAME); then
        actual_status="active"
    else
        actual_status="inactive"
    fi    

    if [ $expected_status != $actual_status ]; then
        echo "Error: $HAILORT_SERVICE_NAME state expected to be ${expected_status} but it is in ${actual_status}"
    fi
}

function activate_hailort_service_if_required(){
    if [ ! -f /.dockerenv ]; then 
        local prompt="Do you wish to activate hailort service? (required for most pyHailoRT use cases) [y/N]: "
        systemctl daemon-reload
        while true; do
            read -s -p "$prompt" -n 1 -t 10 reply || (( $? > 128 ))
            echo ""
            case $reply in
                Y|y) start_hailort_service; break;;
                N|n|"") stop_hailort_service; break;;
            esac
        done
    fi
}

function main() {
    install_libhailort
}

main $@
