#!/bin/bash

# akvirtualcamera, virtual camera for Mac and Windows.
# Copyright (C) 2021  Gonzalo Exequiel Pedone
#
# akvirtualcamera is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# akvirtualcamera is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with akvirtualcamera. If not, see <http://www.gnu.org/licenses/>.
#
# Web-Site: http://webcamoid.github.io/

if [ "${COMPILER}" = clang ]; then
    COMPILER_C=clang
    COMPILER_CXX=clang++
else
    COMPILER_C=gcc
    COMPILER_CXX=g++
fi

if [ -z "${DISABLE_CCACHE}" ]; then
    EXTRA_PARAMS="-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_OBJCXX_COMPILER_LAUNCHER=ccache"
fi

INSTALL_PREFIX=${PWD}/package-data-${COMPILER}

echo
echo "Building x64 virtual camera driver"
echo
buildDir=build-${COMPILER}-x64
mkdir -p ${buildDir}
x86_64-w64-mingw32-cmake \
    -LA \
    -S . \
    -B ${buildDir} \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_C_COMPILER="x86_64-w64-mingw32-${COMPILER_C}" \
    -DCMAKE_CXX_COMPILER="x86_64-w64-mingw32-${COMPILER_CXX}" \
    ${EXTRA_PARAMS}
cmake --build ${buildDir} --parallel ${NJOBS}
cmake --build ${buildDir} --target install
echo
echo "Building x86 virtual camera driver"
echo
buildDir=build-${COMPILER}-x86
mkdir -p ${buildDir}
i686-w64-mingw32-cmake \
    -LA \
    -S . \
    -B ${buildDir} \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_C_COMPILER="i686-w64-mingw32-${COMPILER_C}" \
    -DCMAKE_CXX_COMPILER="i686-w64-mingw32-${COMPILER_CXX}" \
    ${EXTRA_PARAMS}
cmake --build ${buildDir} --parallel ${NJOBS}
cmake --build ${buildDir} --target install
