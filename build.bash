#!/bin/bash

### setup operation ###
if [ $# -ne 1 -a $# -ne 3 ];
then
  echo "ERROR: args invalid"
  echo "USAGE: build.bash {all|rebuild|clean|disclean} [<TARGET> <APPNAME>]"
  exit 1
fi

if [ ${1} = "all" ];
then
  echo "INFO: operation is make all"
  MAKECMD="all"
elif [ ${1} = "rebuild" ];
then
  echo "INFO: operation is rebuild"
  MAKECMD="rebuild"
elif [ ${1} = "clean" ];
then
  echo "INFO: operation is clean"
  MAKECMD="clean"
elif [ ${1} = "distclean" ];
then
  echo "INFO: distclean operation will be done by root privilege"
  sudo rm -rf cmake_build/ mros2/ mbed-os/
  echo "INFO: distclean completed"
  exit 0
else
  echo "ERROR: args invalid"
  echo "USAGE: build.bash {all|rebuild|clean|disclean} [<TARGET> <APPNAME>]"
  exit 1
fi

if [ $# -eq 1 ];
then
  TARGET="NUCLEO_F767ZI"
  APPNAME="echoreply_string"
  echo "WARN: default set will be used for build"
else
  TARGET=$2
  APPNAME=$3
fi

echo "INFO: build setting"
echo "      TARGET=${TARGET}"
echo "      APPNAME=${APPNAME}"

### run mbed-tools ###
### deploy mbed environment ###
docker run --rm -it --mount=type=bind,source="$(pwd)",destination=/var/mbed -w /var/mbed \
  -e APPNAME=${APPNAME} ghcr.io/armmbed/mbed-os-env \
  /bin/bash -c "mbed-tools deploy"

### configure mbed project ###
docker run --rm -it --mount=type=bind,source="$(pwd)",destination=/var/mbed -w /var/mbed \
  -e APPNAME=${APPNAME} ghcr.io/armmbed/mbed-os-env \
  /bin/bash -c "mbed-tools configure -m ${TARGET} -t GCC_ARM"

### set the build parameter ###
docker run --rm -it --mount=type=bind,source="$(pwd)",destination=/var/mbed -w /var/mbed \
  -e APPNAME=${APPNAME} ghcr.io/armmbed/mbed-os-env \
  /bin/bash -c "cmake -S . -B cmake_build/${TARGET}/develop/GCC_ARM -GNinja"

### build ###
if [ ${MAKECMD} = "all" ];
then
  docker run --rm -it --mount=type=bind,source="$(pwd)",destination=/var/mbed -w /var/mbed \
    -e APPNAME=${APPNAME} ghcr.io/armmbed/mbed-os-env \
    /bin/bash -c "cmake --build cmake_build/${TARGET}/develop/GCC_ARM"
elif [ ${MAKECMD} = "rebuild" ];
then
  docker run --rm -it --mount=type=bind,source="$(pwd)",destination=/var/mbed -w /var/mbed \
    -e APPNAME=${APPNAME} ghcr.io/armmbed/mbed-os-env \
    /bin/bash -c "cmake --build cmake_build/${TARGET}/develop/GCC_ARM --clean-first"
elif [ ${MAKECMD} = "clean" ];
then
  docker run --rm -it --mount=type=bind,source="$(pwd)",destination=/var/mbed -w /var/mbed \
    -e APPNAME=${APPNAME} ghcr.io/armmbed/mbed-os-env \
    /bin/bash -c "cmake --build cmake_build/${TARGET}/develop/GCC_ARM --target clean"
fi
