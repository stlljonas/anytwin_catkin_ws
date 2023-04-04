#!/usr/bin/env bash

update_systemd_service() {
    dir=$1
    service_name=$2
    if systemctl is-active --quiet ${service_name}; then
        systemctl stop ${service_name}
    fi
    cp ${dir}/lib/systemd/system/${service_name} /lib/systemd/system/${service_name}
    systemctl daemon-reload
    systemctl enable ${service_name}
    #systemctl start ${service_name}
}

remove_systemd_service() {
    service_name=$1
    service_file=/lib/systemd/system/${service_name}
    if systemctl is-active --quiet ${service_name}; then
        systemctl stop ${service_name}
    fi
    if [[ -f ${service_file} ]]; then
        systemctl disable ${service_name}
        rm -f ${service_file}
    fi
    systemctl daemon-reload
}

update_systemd_service_template() {
    dir=$1
    service_name=$2

    service_template_name=${service_name%.service} # anymal-{SERVICE_NAME}@

    # disable running services first
    for instance_name in ${@:3}
    do
        specific_service_name=${service_template_name}${instance_name}.service
        if systemctl is-active --quiet ${specific_service_name}; then
            systemctl stop ${specific_service_name}
        fi
    done

    # copy service file
    cp ${dir}/lib/systemd/system/${service_name} /lib/systemd/system/${service_name}
    systemctl daemon-reload

    # enable the services
    for instance_name in ${@:3}
    do
        specific_service_name=${service_template_name}${instance_name}.service
        systemctl enable ${specific_service_name}
        #systemctl start ${specific_service_name}
    done
}
