project(implot CXX)
# dear imgui
set(IMPLOT_SOURCE_FILES
        implot.cpp
        implot_items.cpp)

set(IMPLOT_HEADER_FILES
        implot.h
        implot_internal.h)

add_library(implot STATIC ${IMPLOT_SOURCE_FILES} ${IMPLOT_HEADER_FILES})
target_include_directories(implot PUBLIC ..)
target_link_libraries(implot imgui)