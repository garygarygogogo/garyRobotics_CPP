project(imgui CXX)

# dear imgui
set(IMGUI_SOURCE_FILES
        imgui.cpp
        imgui_draw.cpp
        imgui_widgets.cpp
        imgui_tables.cpp
        imgui_demo.cpp
        imgui_impl_glfw.cpp
        imgui_impl_opengl2.cpp)

set(IMGUI_HEADER_FILES
        imconfig.h
        imgui.h
        imgui_internal.h
        imstb_rectpack.h
        imstb_textedit.h
        imstb_truetype.h
        imgui_impl_glfw.h)

add_library(imgui STATIC ${IMGUI_SOURCE_FILES} ${IMGUI_HEADER_FILES})
target_include_directories(imgui PUBLIC ..)
target_link_libraries(imgui ${OPENGL_LIBRARIES})
