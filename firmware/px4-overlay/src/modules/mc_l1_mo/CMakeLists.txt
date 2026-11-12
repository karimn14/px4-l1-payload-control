px4_add_module(
    MODULE modules__mc_l1_mo
    MAIN mc_l1_mo
    COMPILE_FLAGS
        -Wno-cast-align
    SRCS
        mc_l1_mo_main.cpp
    MODULE_CONFIG
        module.yaml
    DEPENDS
        mathlib
        px4_work_queue
)
