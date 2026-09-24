# FindOSRM — defines OSRM::osrm for the source-built libosrm.a in /usr/local (Part 2.7, Phase 0).
#
# ---------------------------------------------------------------------------------------------
# Why the guide's bare `osrm` in target_link_libraries is not enough
#
# libosrm.a is a STATIC library: an archive of object files, not a finished program. When the
# linker reaches an archive, it pulls out only the object files that define a symbol something
# still needs. Those object files carry their own undefined symbols (Boost.Thread,
# Boost.IOStreams, TBB, zlib), and nothing in a static archive records where those should come
# from. A shared library records its dependencies. An archive does not. The final link has to
# name them.
#
# So `-losrm` alone links only while nothing pulls an OSRM object in. That held while main.cpp was
# empty. It broke in Phase 3 once main.cpp used std::string: osrm.cpp.o carries weak copies of
# several std::string member functions, and since osrm comes before the C++ runtime on the link
# line, the linker took them from osrm.cpp.o and inherited its Boost.Thread references.
#
# OSRM's own pkg-config file (/usr/local/lib/pkgconfig/libosrm.pc) would normally supply this
# list, but it is broken: its Libs.private holds CMake target names (Boost::thread) rather than
# linker flags. The list below mirrors what that file intended, in a form CMake can use.
# ---------------------------------------------------------------------------------------------

find_library(OSRM_LIBRARY NAMES osrm REQUIRED)
find_path(OSRM_INCLUDE_DIR osrm/osrm.hpp REQUIRED)

find_package(Boost REQUIRED COMPONENTS date_time iostreams thread)
find_package(TBB REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)

if(NOT TARGET OSRM::osrm)
    add_library(OSRM::osrm STATIC IMPORTED)
    set_target_properties(OSRM::osrm PROPERTIES IMPORTED_LOCATION "${OSRM_LIBRARY}")
    # OSRM's headers include each other as "engine/...", "util/...", relative to include/osrm.
    target_include_directories(OSRM::osrm INTERFACE "${OSRM_INCLUDE_DIR}" "${OSRM_INCLUDE_DIR}/osrm")
    # The same defines OSRM was compiled with (from libosrm.pc's Cflags). Code that includes OSRM
    # headers without them sees a different Boost.Spirit/Phoenix configuration from the one inside
    # libosrm.a.
    target_compile_definitions(OSRM::osrm INTERFACE
        BOOST_SPIRIT_USE_PHOENIX_V3 BOOST_RESULT_OF_USE_DECLTYPE BOOST_PHOENIX_STL_TUPLE_H_)
    target_link_libraries(OSRM::osrm INTERFACE
        Boost::date_time Boost::iostreams Boost::thread TBB::tbb ZLIB::ZLIB Threads::Threads rt)
endif()
