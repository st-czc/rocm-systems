# Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
#
# SPDX-License-Identifier: MIT

target_sources(rocclr PRIVATE
  ${ROCCLR_SRC_DIR}/device/cpu/cpudevice.cpp
  ${ROCCLR_SRC_DIR}/device/cpu/cpukernel.cpp
  ${ROCCLR_SRC_DIR}/device/cpu/cpumemory.cpp
  ${ROCCLR_SRC_DIR}/device/cpu/cpuprogram.cpp
  ${ROCCLR_SRC_DIR}/device/cpu/cputhreadpool.cpp
  ${ROCCLR_SRC_DIR}/device/cpu/gridsync.cpp
  ${ROCCLR_SRC_DIR}/device/cpu/cpuvirtual.cpp)

target_compile_definitions(rocclr PUBLIC WITH_CPU_DEVICE)
