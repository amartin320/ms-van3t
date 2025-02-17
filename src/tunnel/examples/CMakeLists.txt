build_lib_example(
    NAME v2x-cam
    SOURCE_FILES v2x-cam.cc
    LIBRARIES_TO_LINK
    ${libautomotive}
    ${libcv2x}
    ${libtunnel}
    ${libwave}
    ${libtraci}
)

build_lib_example(
    NAME v2x-cam-emulation
    SOURCE_FILES v2x-cam-emulation.cc
    LIBRARIES_TO_LINK
    ${libautomotive}
    ${libcv2x}
    ${libtunnel}
    ${libwave}
    ${libtraci}
)