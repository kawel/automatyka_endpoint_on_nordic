#!/usr/bin/env bash

get_time () {
    echo $(date +"%H:%M:%S")
}

get_date () {
    echo $(date +"%b-%d-%Y")
}

get_sw_revision () {
    echo $(git describe --abbrev=7 --dirty --always --tags)
}

generate_file () {
cat << _EOF_
#ifndef VERSION_GEN_H_
#define VERSION_GEN_H_

#define VER_TIME $(get_time)
#define VER_DATE $(get_date)
#define VER_GIT_REV $(get_sw_revision)

#endif
_EOF_
}

echo "Generate version_gen.h file."
generate_file > ./version/version_gen.h