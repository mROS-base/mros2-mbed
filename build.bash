#!/bin/bash

### setup operation ###
if [ $# -ne 1 -a $# -ne 3 -a $# -ne 4 ];
then
  echo "ERROR: args invalid"
  echo "USAGE: build.bash {all|rebuild|clean|distclean} [<TARGET> <APPNAME>] [{native|docker}]"
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
  echo "USAGE: build.bash {all|rebuild|clean|distclean} [<TARGET> <APPNAME>] [{native|docker}]"
  exit 1
fi

if [ $# -eq 1 ];
then
  TARGET="NUCLEO_F767ZI"
  APPNAME="echoback_string"
  echo "WARN: default set will be used for build"
else
  TARGET=$2
  APPNAME=$3
fi

DOCKERCMD_PRE="docker run --rm -it --mount type=bind,source=$(pwd),destination=/var/mbed \
  -w /var/mbed -e APPNAME=${APPNAME} ghcr.io/armmbed/mbed-os-env \
  /bin/bash -c \""
DOCKERCMD_SUF="\""
if [ $# == 4 ];
then
  if [ ${4} = "native" ];
  then
    echo "INFO: build operation will be executed on native env"
    DOCKERCMD_PRE=""
    DOCKERCMD_SUF=""
    export APPNAME=${APPNAME}
  elif [ ${4} = "docker" ];
  then
    echo "INFO: build operation will be executed on dokcer env"
  else
    echo "ERROR: args invalid"
    echo "ERROR: \"native\" or \"docker\" should be specified as 4th arg"
    echo "USAGE: build.bash {all|rebuild|clean|distclean} [<TARGET> <APPNAME>]"
    exit 1
  fi
else
  echo "INFO: build operation will be executed on dokcer env"
fi

echo "INFO: build setting"
echo "      TARGET=${TARGET}"
echo "      APPNAME=${APPNAME}"


### build a project ###
# deploy mbed environment by mbed-tools
eval ${DOCKERCMD_PRE}mbed-tools deploy${DOCKERCMD_SUF}

# configure mbed project by mbed-tools
eval ${DOCKERCMD_PRE}mbed-tools configure -m ${TARGET} -t GCC_ARM${DOCKERCMD_SUF}

# set the build parameter
eval ${DOCKERCMD_PRE}cmake -S . -B cmake_build/${TARGET}/develop/GCC_ARM -GNinja${DOCKERCMD_SUF}

# build (switch according to arg1)
if [ ${MAKECMD} = "all" ];
then
  eval ${DOCKERCMD_PRE}cmake --build cmake_build/${TARGET}/develop/GCC_ARM${DOCKERCMD_SUF}
elif [ ${MAKECMD} = "rebuild" ];
then
  eval ${DOCKERCMD_PRE}cmake --build cmake_build/${TARGET}/develop/GCC_ARM --clean-first${DOCKERCMD_SUF}
elif [ ${MAKECMD} = "clean" ];
then
  eval ${DOCKERCMD_PRE}cmake --build cmake_build/${TARGET}/develop/GCC_ARM --target clean${DOCKERCMD_SUF}
fi
