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


### build a project ###
# deploy mbed environment by mbed-tools
docker run --rm -it --mount=type=bind,source="$(pwd)",destination=/var/mbed -w /var/mbed \
  -e APPNAME=${APPNAME} ghcr.io/armmbed/mbed-os-env \
  /bin/bash -c "mbed-tools deploy"

# configure mbed project by mbed-tools
docker run --rm -it --mount=type=bind,source="$(pwd)",destination=/var/mbed -w /var/mbed \
  -e APPNAME=${APPNAME} ghcr.io/armmbed/mbed-os-env \
  /bin/bash -c "mbed-tools configure -m ${TARGET} -t GCC_ARM"

# generate of header file for template functions of MsgType
MROS2DIR=../mros2
TEMPLATESGEN_FILE=${MROS2DIR}/mros2_header_generator/templates_generator.py

echo "INFO: generate header file for template functions of MsgType"
touch header_includer/header_includer.hpp
cd workspace
python ${TEMPLATESGEN_FILE} ${MROS2DIR} ${APPNAME}
if [ $? -eq 0 ];
then
  echo "INFO: header fille for template function of ${APPNAME}'s MsgType is successfully generated"
else
  echo "ERROR: failed to generate header fille for template function of ${APPNAME}'s MsgType"
  exit 1
fi
cd ..

# set the build parameter
docker run --rm -it --mount=type=bind,source="$(pwd)",destination=/var/mbed -w /var/mbed \
  -e APPNAME=${APPNAME} ghcr.io/armmbed/mbed-os-env \
  /bin/bash -c "cmake -S . -B cmake_build/${TARGET}/develop/GCC_ARM -GNinja"

# build (switch according to arg1)
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
