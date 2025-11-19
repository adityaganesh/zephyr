# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(i3g4250d.c i3g4250d_trigger.c)

zephyr_library_include_directories(../stmemsc)
