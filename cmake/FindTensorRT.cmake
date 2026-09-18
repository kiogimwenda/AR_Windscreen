# FindTensorRT.cmake — see docs/BUILD_GUIDE.md Part 5.2.
#
# TensorRT's .deb install on Debian ships headers and libraries into the standard multiarch paths
# but provides no CMake config package, so a straightforward find_path/find_library pair is all
# that is needed. On success this defines the imported target TensorRT::TensorRT carrying the
# include directory and the three libraries the host links: nvinfer, nvinfer_plugin, nvonnxparser.

include(FindPackageHandleStandardArgs)

find_path(TensorRT_INCLUDE_DIR
    NAMES NvInfer.h
    HINTS /usr/include/x86_64-linux-gnu /usr/include /usr/local/include)

find_library(TensorRT_NVINFER_LIBRARY
    NAMES nvinfer
    HINTS /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib)

find_library(TensorRT_NVINFER_PLUGIN_LIBRARY
    NAMES nvinfer_plugin
    HINTS /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib)

find_library(TensorRT_NVONNXPARSER_LIBRARY
    NAMES nvonnxparser
    HINTS /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib)

find_package_handle_standard_args(TensorRT
    REQUIRED_VARS
        TensorRT_INCLUDE_DIR
        TensorRT_NVINFER_LIBRARY
        TensorRT_NVINFER_PLUGIN_LIBRARY
        TensorRT_NVONNXPARSER_LIBRARY)

if(TensorRT_FOUND AND NOT TARGET TensorRT::TensorRT)
    add_library(TensorRT::TensorRT INTERFACE IMPORTED)
    set_target_properties(TensorRT::TensorRT PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${TensorRT_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES
            "${TensorRT_NVINFER_LIBRARY};${TensorRT_NVINFER_PLUGIN_LIBRARY};${TensorRT_NVONNXPARSER_LIBRARY}")
endif()

mark_as_advanced(TensorRT_INCLUDE_DIR TensorRT_NVINFER_LIBRARY
                 TensorRT_NVINFER_PLUGIN_LIBRARY TensorRT_NVONNXPARSER_LIBRARY)
