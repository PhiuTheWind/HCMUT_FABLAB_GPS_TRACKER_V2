# File: components/sim_a7680c/CMakeLists.txt

idf_component_register(
        SRCS "sim_a7680c.c"
        INCLUDE_DIRS "."
        PRIV_REQUIRES driver gps_library)
