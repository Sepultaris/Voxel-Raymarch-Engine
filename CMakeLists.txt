cmake_minimum_required(VERSION 3.28)

project(VoxelRaymarchEngine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(VOXEL_ENABLE_VALIDATION "Enable Vulkan validation layers in non-release builds" ON)
option(VOXEL_ENABLE_JOLT "Build the rigid-body physics subsystem with Jolt Physics" ON)
option(VOXEL_ENABLE_REFERENCE_TRAVERSAL
       "Build and upload binary/wide geodesic traversal backends for profiling" OFF)
option(VOXEL_BUILD_SCALE_LAB
       "Build the isolated f1024/R2 scalability laboratory executable" OFF)
option(VOXEL_BUILD_ADAPTIVE_SDF_LAB
       "Build the isolated adaptive fractal-planet SDF laboratory executable" OFF)
option(VOXEL_BUILD_FRACTAL_PLANET_SDF_LAB
       "Build the isolated true multiscale fractal-planet SDF laboratory executable" OFF)
option(VOXEL_BUILD_FRACTAL_VOXEL_SDF_LAB
       "Build the isolated adaptive voxelized fractal-planet SDF laboratory executable" OFF)
option(VOXEL_BUILD_PORTAL_LAB
       "Build the isolated curved-ray spherical wormhole portal laboratory executable" OFF)
option(VOXEL_BUILD_INTRINSIC_PORTAL_LAB
       "Build the isolated native-coordinate Ellis manifold laboratory executable" OFF)
option(VOXEL_BUILD_GLOBAL_METRIC_LAB
       "Build the isolated all-space static global metric-field laboratory" OFF)
option(VOXEL_BUILD_EIGHT_PLANET_SYSTEM_LAB
       "Build the isolated unpromoted eight-planet hex-LOD system laboratory" OFF)
set(VOXEL_IMGUI_SOURCE_DIR "" CACHE PATH "Path to a Dear ImGui source tree")
set(VOXEL_GEODESIC_FREQUENCY 512 CACHE STRING "Geodesic subdivision frequency")
set(VOXEL_TILE_RADIAL_LAYERS 32 CACHE STRING "Resident radial voxel layers per surface column")
set(VOXEL_RESIDENT_GEODESIC_PAGES 8192 CACHE STRING "Physical geodesic voxel pages kept resident")

find_package(Threads REQUIRED)

if(MINGW)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(VULKAN REQUIRED IMPORTED_TARGET vulkan)
    find_path(VULKAN_HEADER_DIR vulkan/vulkan.h HINTS "$ENV{VULKAN_SDK}/Include" REQUIRED)
    pkg_check_modules(SDL2_PKG REQUIRED sdl2)
    pkg_check_modules(OPENAL_PKG REQUIRED openal)

    find_path(SDL2_INCLUDE_DIR SDL.h HINTS ${SDL2_PKG_INCLUDE_DIRS} PATH_SUFFIXES SDL2 REQUIRED)
    find_library(SDL2_LIBRARY NAMES SDL2 HINTS ${SDL2_PKG_LIBRARY_DIRS} REQUIRED)
    find_file(SDL2_RUNTIME NAMES SDL2.dll HINTS "${SDL2_PKG_PREFIX}/bin" REQUIRED)
    add_library(voxel_sdl2 SHARED IMPORTED)
    set_target_properties(voxel_sdl2 PROPERTIES
        IMPORTED_IMPLIB "${SDL2_LIBRARY}"
        IMPORTED_LOCATION "${SDL2_RUNTIME}"
        INTERFACE_INCLUDE_DIRECTORIES "${SDL2_INCLUDE_DIR}"
    )

    find_path(OPENAL_INCLUDE_DIR AL/al.h HINTS ${OPENAL_PKG_INCLUDE_DIRS} REQUIRED)
    find_library(OPENAL_LIBRARY NAMES openal OpenAL32 HINTS ${OPENAL_PKG_LIBRARY_DIRS} REQUIRED)
    find_file(OPENAL_RUNTIME NAMES libopenal-1.dll OpenAL32.dll HINTS "${OPENAL_PKG_PREFIX}/bin" REQUIRED)
    add_library(voxel_openal SHARED IMPORTED)
    set_target_properties(voxel_openal PROPERTIES
        IMPORTED_IMPLIB "${OPENAL_LIBRARY}"
        IMPORTED_LOCATION "${OPENAL_RUNTIME}"
        INTERFACE_INCLUDE_DIRECTORIES "${OPENAL_INCLUDE_DIR}"
    )

    set(VOXEL_VULKAN_TARGET PkgConfig::VULKAN)
    set(VOXEL_SDL_TARGET voxel_sdl2)
    set(VOXEL_OPENAL_TARGET voxel_openal)
else()
    find_package(Vulkan REQUIRED)
    find_package(SDL2 CONFIG REQUIRED)
    find_package(OpenAL REQUIRED)
    set(VOXEL_VULKAN_TARGET Vulkan::Vulkan)
    set(VOXEL_SDL_TARGET SDL2::SDL2)
    set(VOXEL_OPENAL_TARGET OpenAL::OpenAL)
endif()

find_program(GLSLC_EXECUTABLE glslc HINTS "$ENV{VULKAN_SDK}/Bin" REQUIRED)
find_program(SPIRV_VAL_EXECUTABLE spirv-val HINTS "$ENV{VULKAN_SDK}/Bin" REQUIRED)

if(VOXEL_ENABLE_JOLT)
    include(FetchContent)
    set(DOUBLE_PRECISION ON CACHE BOOL "Use double precision Jolt positions" FORCE)
    set(CROSS_PLATFORM_DETERMINISTIC ON CACHE BOOL "Use deterministic Jolt simulation settings" FORCE)
    set(OVERRIDE_CXX_FLAGS OFF CACHE BOOL "Preserve the parent project's compiler flags" FORCE)
    set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "Do not promote dependency warnings" FORCE)
    set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "Keep development links fast" FORCE)
    set(DEBUG_RENDERER_IN_DEBUG_AND_RELEASE OFF CACHE BOOL "Use the engine's own debug renderer" FORCE)
    set(PROFILER_IN_DEBUG_AND_RELEASE OFF CACHE BOOL "Use the engine's profiler integration" FORCE)
    set(ENABLE_OBJECT_STREAM OFF CACHE BOOL "Disable unused Jolt object streaming" FORCE)
    set(ENABLE_INSTALL OFF CACHE BOOL "Do not install bundled Jolt" FORCE)
    set(JPH_USE_DX12 OFF CACHE BOOL "Disable Jolt GPU hair backends" FORCE)
    set(JPH_USE_VK OFF CACHE BOOL "Disable Jolt GPU hair backends" FORCE)
    set(JPH_USE_MTL OFF CACHE BOOL "Disable Jolt GPU hair backends" FORCE)
    FetchContent_Declare(
        jolt_physics
        GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
        GIT_TAG v5.6.0
        GIT_SHALLOW TRUE
        SOURCE_SUBDIR Build
    )
    FetchContent_MakeAvailable(jolt_physics)
endif()

if(NOT EXISTS "${VOXEL_IMGUI_SOURCE_DIR}/imgui.cpp")
    include(FetchContent)
    FetchContent_Declare(
        imgui_source
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG master
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(imgui_source)
    set(VOXEL_IMGUI_SOURCE_DIR "${imgui_source_SOURCE_DIR}")
endif()

add_library(voxel_imgui STATIC
    "${VOXEL_IMGUI_SOURCE_DIR}/imgui.cpp"
    "${VOXEL_IMGUI_SOURCE_DIR}/imgui_draw.cpp"
    "${VOXEL_IMGUI_SOURCE_DIR}/imgui_tables.cpp"
    "${VOXEL_IMGUI_SOURCE_DIR}/imgui_widgets.cpp"
    "${VOXEL_IMGUI_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp"
    "${VOXEL_IMGUI_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp"
)
target_include_directories(voxel_imgui PUBLIC
    "${VOXEL_IMGUI_SOURCE_DIR}"
    "${VOXEL_IMGUI_SOURCE_DIR}/backends"
)
if(MINGW)
    target_include_directories(voxel_imgui PUBLIC "${VULKAN_HEADER_DIR}")
endif()
target_link_libraries(voxel_imgui PUBLIC ${VOXEL_VULKAN_TARGET} ${VOXEL_SDL_TARGET})

set(VOXEL_SHADER_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/planet.comp"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/voxel_init.comp"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sand_claim.comp"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sand_commit.comp"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/voxel_edit.comp"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/geodesic_edit.comp"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/geodesic_terrain_init.comp"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/geodesic_far_field_init.comp"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/geodesic_stream_requests.comp"
)
set(VOXEL_SHADER_SHARED_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/shared/geodesic_material_shared.inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/shared/geodesic_terrain_shared.inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/shared/geodesic_tile_query_shared.inc"
)
if(VOXEL_ENABLE_REFERENCE_TRAVERSAL)
    set(VOXEL_REFERENCE_TRAVERSAL_VALUE 1)
else()
    set(VOXEL_REFERENCE_TRAVERSAL_VALUE 0)
endif()
function(add_voxel_shader_set target_name output_subdir lookup_width lookup_height scale_lab adaptive_sdf_lab fractal_planet_sdf_lab fractal_voxel_sdf_lab portal_lab intrinsic_portal_lab global_metric_lab system_lab)
    set(shader_outputs)
    foreach(shader IN LISTS VOXEL_SHADER_SOURCES)
        get_filename_component(shader_name "${shader}" NAME)
        set(shader_output "${CMAKE_CURRENT_BINARY_DIR}/${output_subdir}/${shader_name}.spv")
        add_custom_command(
            OUTPUT "${shader_output}"
            COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/${output_subdir}"
            COMMAND "${GLSLC_EXECUTABLE}" --target-env=vulkan1.2 -O
                    "-I${CMAKE_CURRENT_SOURCE_DIR}"
                    -DVOXEL_ENABLE_REFERENCE_TRAVERSAL=${VOXEL_REFERENCE_TRAVERSAL_VALUE}
                    -DVOXEL_TILE_LOOKUP_WIDTH=${lookup_width}
                    -DVOXEL_TILE_LOOKUP_HEIGHT=${lookup_height}
                    -DVOXEL_SCALE_LAB=${scale_lab}
                    -DVOXEL_ADAPTIVE_SDF_LAB=${adaptive_sdf_lab}
                    -DVOXEL_FRACTAL_PLANET_SDF_LAB=${fractal_planet_sdf_lab}
                    -DVOXEL_FRACTAL_VOXEL_SDF_LAB=${fractal_voxel_sdf_lab}
                    -DVOXEL_PORTAL_LAB=${portal_lab}
                    -DVOXEL_INTRINSIC_PORTAL_LAB=${intrinsic_portal_lab}
                    -DVOXEL_GLOBAL_METRIC_LAB=${global_metric_lab}
                    -DVOXEL_EIGHT_PLANET_SYSTEM_LAB=${system_lab}
                    "${shader}" -o "${shader_output}"
            COMMAND "${SPIRV_VAL_EXECUTABLE}" --target-env vulkan1.2 "${shader_output}"
            DEPENDS "${shader}" ${VOXEL_SHADER_SHARED_SOURCES}
            VERBATIM
        )
        list(APPEND shader_outputs "${shader_output}")
    endforeach()
    add_custom_target(${target_name} DEPENDS ${shader_outputs})
endfunction()

add_voxel_shader_set(voxel_shaders shaders 512 256 0 0 0 0 0 0 0 0)

set(VOXEL_ENGINE_SOURCES
    src/main.cpp
    src/app/application.cpp
    src/app/surface_camera_controller.cpp
    src/app/surface_player_collision.cpp
    src/audio/audio_engine.cpp
    src/physics/physics_engine.cpp
    src/planet/geodesic_topology.cpp
    src/planet/geodesic_topology_cache.cpp
    src/planet/scale_lab_guardrails.cpp
    src/render/vulkan_renderer.cpp
)
add_executable(voxel_engine ${VOXEL_ENGINE_SOURCES})
add_dependencies(voxel_engine voxel_shaders)
target_include_directories(voxel_engine PRIVATE src)
if(MINGW)
    target_include_directories(voxel_engine PRIVATE "${VULKAN_HEADER_DIR}")
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(geodesic_topology_test
        tests/geodesic_topology_test.cpp
        src/planet/geodesic_topology.cpp
    )
    target_include_directories(geodesic_topology_test PRIVATE src)
    if(MSVC)
        target_compile_options(geodesic_topology_test PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(geodesic_topology_test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME geodesic_topology_invariants COMMAND geodesic_topology_test)
    set_tests_properties(geodesic_topology_invariants PROPERTIES LABELS "milestone;cpu;topology")

    add_executable(geodesic_geometry_test
        tests/geodesic_geometry_test.cpp
        src/planet/geodesic_topology.cpp
    )
    target_include_directories(geodesic_geometry_test PRIVATE src)
    if(MSVC)
        target_compile_options(geodesic_geometry_test PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(geodesic_geometry_test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME geodesic_watertight_acceleration_rays COMMAND geodesic_geometry_test)
    set_tests_properties(geodesic_watertight_acceleration_rays PROPERTIES
        LABELS "milestone;cpu;geometry;bvh;wide-mask;dda")

    add_executable(geodesic_sparse_residency_test
        tests/geodesic_sparse_residency_test.cpp
        src/planet/geodesic_topology.cpp
    )
    target_include_directories(geodesic_sparse_residency_test PRIVATE src)
    if(MSVC)
        target_compile_options(geodesic_sparse_residency_test PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(geodesic_sparse_residency_test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME geodesic_sparse_residency COMMAND geodesic_sparse_residency_test)
    set_tests_properties(geodesic_sparse_residency PROPERTIES
        LABELS "milestone;cpu;sparse;residency")

    add_executable(geodesic_visual_consistency_test
        tests/geodesic_visual_consistency_test.cpp
    )
    target_include_directories(geodesic_visual_consistency_test PRIVATE src)
    if(MSVC)
        target_compile_options(geodesic_visual_consistency_test PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(geodesic_visual_consistency_test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME geodesic_visual_consistency COMMAND geodesic_visual_consistency_test)
    set_tests_properties(geodesic_visual_consistency PROPERTIES
        LABELS "milestone;cpu;render;residency;grazing;lod")

    add_executable(procedural_heightmap_test
        tests/procedural_heightmap_test.cpp
    )
    target_include_directories(procedural_heightmap_test PRIVATE src)
    if(MSVC)
        target_compile_options(procedural_heightmap_test PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(procedural_heightmap_test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME procedural_orthographic_heightmap COMMAND procedural_heightmap_test)
    set_tests_properties(procedural_orthographic_heightmap PROPERTIES
        LABELS "milestone;cpu;terrain;determinism;seams;occupancy")

    add_executable(camera_zoom_test
        tests/camera_zoom_test.cpp
    )
    target_include_directories(camera_zoom_test PRIVATE src)
    if(MSVC)
        target_compile_options(camera_zoom_test PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(camera_zoom_test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME close_surface_camera_zoom COMMAND camera_zoom_test)
    set_tests_properties(close_surface_camera_zoom PROPERTIES
        LABELS "milestone;cpu;camera;controls")

    add_executable(surface_camera_controller_test
        tests/surface_camera_controller_test.cpp
        src/app/surface_camera_controller.cpp
        src/app/surface_player_collision.cpp
        src/planet/geodesic_topology.cpp
    )
    target_include_directories(surface_camera_controller_test PRIVATE src)
    if(MSVC)
        target_compile_options(surface_camera_controller_test PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(surface_camera_controller_test PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
    add_test(NAME spherical_surface_camera_controller COMMAND surface_camera_controller_test)
    set_tests_properties(spherical_surface_camera_controller PROPERTIES
        LABELS "milestone;cpu;camera;surface;controls;seams")
endif()
target_link_libraries(voxel_engine PRIVATE
    ${VOXEL_VULKAN_TARGET}
    ${VOXEL_SDL_TARGET}
    ${VOXEL_OPENAL_TARGET}
    voxel_imgui
    Threads::Threads
)
if(WIN32)
    target_link_libraries(voxel_engine PRIVATE psapi)
endif()
if(VOXEL_ENABLE_JOLT)
    target_link_libraries(voxel_engine PRIVATE Jolt)
    target_compile_definitions(voxel_engine PRIVATE VOXEL_HAS_JOLT=1)
endif()
target_compile_definitions(voxel_engine PRIVATE
    SDL_MAIN_HANDLED
    VOXEL_SHADER_DIR="${CMAKE_CURRENT_BINARY_DIR}/shaders"
    VOXEL_GEODESIC_FREQUENCY=${VOXEL_GEODESIC_FREQUENCY}
    VOXEL_TILE_RADIAL_LAYERS=${VOXEL_TILE_RADIAL_LAYERS}
    VOXEL_RESIDENT_GEODESIC_PAGES=${VOXEL_RESIDENT_GEODESIC_PAGES}
    VOXEL_ENABLE_REFERENCE_TRAVERSAL=${VOXEL_REFERENCE_TRAVERSAL_VALUE}
    VOXEL_TILE_LOOKUP_WIDTH=512
    VOXEL_TILE_LOOKUP_HEIGHT=256
    VOXEL_SCALE_LAB=0
    VOXEL_ADAPTIVE_SDF_LAB=0
    VOXEL_FRACTAL_PLANET_SDF_LAB=0
    VOXEL_FRACTAL_VOXEL_SDF_LAB=0
    VOXEL_PORTAL_LAB=0
    VOXEL_INTRINSIC_PORTAL_LAB=0
    VOXEL_DEFAULT_PLANET_RADIUS=1.0F
    $<$<BOOL:${VOXEL_ENABLE_VALIDATION}>:VOXEL_ENABLE_VALIDATION=1>
)

if(VOXEL_BUILD_SCALE_LAB)
    add_voxel_shader_set(voxel_scale_lab_shaders shaders-scale-lab 1024 512 1 0 0 0 0 0 0 0)
    add_executable(voxel_engine_scale_lab ${VOXEL_ENGINE_SOURCES})
    add_dependencies(voxel_engine_scale_lab voxel_scale_lab_shaders)
    target_include_directories(voxel_engine_scale_lab PRIVATE src)
    if(MINGW)
        target_include_directories(voxel_engine_scale_lab PRIVATE "${VULKAN_HEADER_DIR}")
    endif()
    target_link_libraries(voxel_engine_scale_lab PRIVATE
        ${VOXEL_VULKAN_TARGET} ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET}
        voxel_imgui Threads::Threads)
    if(WIN32)
        target_link_libraries(voxel_engine_scale_lab PRIVATE psapi)
    endif()
    if(VOXEL_ENABLE_JOLT)
        target_link_libraries(voxel_engine_scale_lab PRIVATE Jolt)
        target_compile_definitions(voxel_engine_scale_lab PRIVATE VOXEL_HAS_JOLT=1)
    endif()
    target_compile_definitions(voxel_engine_scale_lab PRIVATE
        SDL_MAIN_HANDLED
        VOXEL_SHADER_DIR="${CMAKE_CURRENT_BINARY_DIR}/shaders-scale-lab"
        VOXEL_GEODESIC_FREQUENCY=1024
        VOXEL_TILE_RADIAL_LAYERS=32
        VOXEL_RESIDENT_GEODESIC_PAGES=8192
        VOXEL_ENABLE_REFERENCE_TRAVERSAL=0
        VOXEL_TILE_LOOKUP_WIDTH=1024
        VOXEL_TILE_LOOKUP_HEIGHT=512
        VOXEL_SCALE_LAB=1
        VOXEL_ADAPTIVE_SDF_LAB=0
        VOXEL_FRACTAL_PLANET_SDF_LAB=0
        VOXEL_FRACTAL_VOXEL_SDF_LAB=0
        VOXEL_PORTAL_LAB=0
        VOXEL_INTRINSIC_PORTAL_LAB=0
        VOXEL_SCALE_LAB_CACHE_DIR="${CMAKE_SOURCE_DIR}/cache/scale-lab"
        VOXEL_DEFAULT_PLANET_RADIUS=2.0F
        $<$<BOOL:${VOXEL_ENABLE_VALIDATION}>:VOXEL_ENABLE_VALIDATION=1>)
    if(MSVC)
        target_compile_options(voxel_engine_scale_lab PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(voxel_engine_scale_lab PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wno-missing-field-initializers)
    endif()
endif()

if(VOXEL_BUILD_ADAPTIVE_SDF_LAB)
    add_voxel_shader_set(voxel_adaptive_sdf_shaders shaders-adaptive-sdf 512 256 0 1 0 0 0 0 0 0)
    add_executable(voxel_engine_adaptive_sdf_lab ${VOXEL_ENGINE_SOURCES})
    add_dependencies(voxel_engine_adaptive_sdf_lab voxel_adaptive_sdf_shaders)
    target_include_directories(voxel_engine_adaptive_sdf_lab PRIVATE src)
    if(MINGW)
        target_include_directories(voxel_engine_adaptive_sdf_lab PRIVATE "${VULKAN_HEADER_DIR}")
    endif()
    target_link_libraries(voxel_engine_adaptive_sdf_lab PRIVATE
        ${VOXEL_VULKAN_TARGET} ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET}
        voxel_imgui Threads::Threads)
    if(WIN32)
        target_link_libraries(voxel_engine_adaptive_sdf_lab PRIVATE psapi)
    endif()
    if(VOXEL_ENABLE_JOLT)
        target_link_libraries(voxel_engine_adaptive_sdf_lab PRIVATE Jolt)
        target_compile_definitions(voxel_engine_adaptive_sdf_lab PRIVATE VOXEL_HAS_JOLT=1)
    endif()
    target_compile_definitions(voxel_engine_adaptive_sdf_lab PRIVATE
        SDL_MAIN_HANDLED
        VOXEL_SHADER_DIR="${CMAKE_CURRENT_BINARY_DIR}/shaders-adaptive-sdf"
        VOXEL_GEODESIC_FREQUENCY=512
        VOXEL_TILE_RADIAL_LAYERS=32
        VOXEL_RESIDENT_GEODESIC_PAGES=8192
        VOXEL_ENABLE_REFERENCE_TRAVERSAL=0
        VOXEL_TILE_LOOKUP_WIDTH=512
        VOXEL_TILE_LOOKUP_HEIGHT=256
        VOXEL_SCALE_LAB=0
        VOXEL_ADAPTIVE_SDF_LAB=1
        VOXEL_FRACTAL_PLANET_SDF_LAB=0
        VOXEL_FRACTAL_VOXEL_SDF_LAB=0
        VOXEL_PORTAL_LAB=0
        VOXEL_INTRINSIC_PORTAL_LAB=0
        VOXEL_DEFAULT_PLANET_RADIUS=1.0F
        $<$<BOOL:${VOXEL_ENABLE_VALIDATION}>:VOXEL_ENABLE_VALIDATION=1>)
    if(MSVC)
        target_compile_options(voxel_engine_adaptive_sdf_lab PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(voxel_engine_adaptive_sdf_lab PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wno-missing-field-initializers)
    endif()
endif()

if(VOXEL_BUILD_FRACTAL_PLANET_SDF_LAB)
    add_voxel_shader_set(voxel_fractal_planet_sdf_shaders shaders-fractal-planet-sdf 512 256 0 0 1 0 0 0 0 0)
    add_executable(voxel_engine_fractal_planet_sdf_lab ${VOXEL_ENGINE_SOURCES})
    add_dependencies(voxel_engine_fractal_planet_sdf_lab voxel_fractal_planet_sdf_shaders)
    target_include_directories(voxel_engine_fractal_planet_sdf_lab PRIVATE src)
    if(MINGW)
        target_include_directories(voxel_engine_fractal_planet_sdf_lab PRIVATE "${VULKAN_HEADER_DIR}")
    endif()
    target_link_libraries(voxel_engine_fractal_planet_sdf_lab PRIVATE
        ${VOXEL_VULKAN_TARGET} ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET}
        voxel_imgui Threads::Threads)
    if(WIN32)
        target_link_libraries(voxel_engine_fractal_planet_sdf_lab PRIVATE psapi)
    endif()
    if(VOXEL_ENABLE_JOLT)
        target_link_libraries(voxel_engine_fractal_planet_sdf_lab PRIVATE Jolt)
        target_compile_definitions(voxel_engine_fractal_planet_sdf_lab PRIVATE VOXEL_HAS_JOLT=1)
    endif()
    target_compile_definitions(voxel_engine_fractal_planet_sdf_lab PRIVATE
        SDL_MAIN_HANDLED
        VOXEL_SHADER_DIR="${CMAKE_CURRENT_BINARY_DIR}/shaders-fractal-planet-sdf"
        VOXEL_GEODESIC_FREQUENCY=512
        VOXEL_TILE_RADIAL_LAYERS=32
        VOXEL_RESIDENT_GEODESIC_PAGES=8192
        VOXEL_ENABLE_REFERENCE_TRAVERSAL=0
        VOXEL_TILE_LOOKUP_WIDTH=512
        VOXEL_TILE_LOOKUP_HEIGHT=256
        VOXEL_SCALE_LAB=0
        VOXEL_ADAPTIVE_SDF_LAB=0
        VOXEL_FRACTAL_PLANET_SDF_LAB=1
        VOXEL_FRACTAL_VOXEL_SDF_LAB=0
        VOXEL_PORTAL_LAB=0
        VOXEL_INTRINSIC_PORTAL_LAB=0
        VOXEL_DEFAULT_PLANET_RADIUS=1.0F
        $<$<BOOL:${VOXEL_ENABLE_VALIDATION}>:VOXEL_ENABLE_VALIDATION=1>)
    if(MSVC)
        target_compile_options(voxel_engine_fractal_planet_sdf_lab PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(voxel_engine_fractal_planet_sdf_lab PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wno-missing-field-initializers)
    endif()
endif()

if(VOXEL_BUILD_FRACTAL_VOXEL_SDF_LAB)
    add_voxel_shader_set(voxel_fractal_voxel_sdf_shaders shaders-fractal-voxel-sdf 512 256 0 0 1 1 0 0 0 0)
    add_executable(voxel_engine_fractal_voxel_sdf_lab ${VOXEL_ENGINE_SOURCES})
    add_dependencies(voxel_engine_fractal_voxel_sdf_lab voxel_fractal_voxel_sdf_shaders)
    target_include_directories(voxel_engine_fractal_voxel_sdf_lab PRIVATE src)
    if(MINGW)
        target_include_directories(voxel_engine_fractal_voxel_sdf_lab PRIVATE "${VULKAN_HEADER_DIR}")
    endif()
    target_link_libraries(voxel_engine_fractal_voxel_sdf_lab PRIVATE
        ${VOXEL_VULKAN_TARGET} ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET}
        voxel_imgui Threads::Threads)
    if(WIN32)
        target_link_libraries(voxel_engine_fractal_voxel_sdf_lab PRIVATE psapi)
    endif()
    if(VOXEL_ENABLE_JOLT)
        target_link_libraries(voxel_engine_fractal_voxel_sdf_lab PRIVATE Jolt)
        target_compile_definitions(voxel_engine_fractal_voxel_sdf_lab PRIVATE VOXEL_HAS_JOLT=1)
    endif()
    target_compile_definitions(voxel_engine_fractal_voxel_sdf_lab PRIVATE
        SDL_MAIN_HANDLED
        VOXEL_SHADER_DIR="${CMAKE_CURRENT_BINARY_DIR}/shaders-fractal-voxel-sdf"
        VOXEL_GEODESIC_FREQUENCY=512
        VOXEL_TILE_RADIAL_LAYERS=32
        VOXEL_RESIDENT_GEODESIC_PAGES=8192
        VOXEL_ENABLE_REFERENCE_TRAVERSAL=0
        VOXEL_TILE_LOOKUP_WIDTH=512
        VOXEL_TILE_LOOKUP_HEIGHT=256
        VOXEL_SCALE_LAB=0
        VOXEL_ADAPTIVE_SDF_LAB=0
        VOXEL_FRACTAL_PLANET_SDF_LAB=1
        VOXEL_FRACTAL_VOXEL_SDF_LAB=1
        VOXEL_PORTAL_LAB=0
        VOXEL_INTRINSIC_PORTAL_LAB=0
        VOXEL_DEFAULT_PLANET_RADIUS=1.0F
        $<$<BOOL:${VOXEL_ENABLE_VALIDATION}>:VOXEL_ENABLE_VALIDATION=1>)
    if(MSVC)
        target_compile_options(voxel_engine_fractal_voxel_sdf_lab PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(voxel_engine_fractal_voxel_sdf_lab PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wno-missing-field-initializers)
    endif()
endif()

if(VOXEL_BUILD_PORTAL_LAB)
    add_voxel_shader_set(voxel_portal_lab_shaders shaders-portal-lab 512 256 0 0 0 0 1 0 0 0)
    add_executable(voxel_engine_portal_lab ${VOXEL_ENGINE_SOURCES})
    add_dependencies(voxel_engine_portal_lab voxel_portal_lab_shaders)
    target_include_directories(voxel_engine_portal_lab PRIVATE src)
    if(MINGW)
        target_include_directories(voxel_engine_portal_lab PRIVATE "${VULKAN_HEADER_DIR}")
    endif()
    target_link_libraries(voxel_engine_portal_lab PRIVATE
        ${VOXEL_VULKAN_TARGET} ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET}
        voxel_imgui Threads::Threads)
    if(WIN32)
        target_link_libraries(voxel_engine_portal_lab PRIVATE psapi)
    endif()
    if(VOXEL_ENABLE_JOLT)
        target_link_libraries(voxel_engine_portal_lab PRIVATE Jolt)
        target_compile_definitions(voxel_engine_portal_lab PRIVATE VOXEL_HAS_JOLT=1)
    endif()
    target_compile_definitions(voxel_engine_portal_lab PRIVATE
        SDL_MAIN_HANDLED
        VOXEL_SHADER_DIR="${CMAKE_CURRENT_BINARY_DIR}/shaders-portal-lab"
        VOXEL_GEODESIC_FREQUENCY=512
        VOXEL_TILE_RADIAL_LAYERS=32
        VOXEL_RESIDENT_GEODESIC_PAGES=8192
        VOXEL_ENABLE_REFERENCE_TRAVERSAL=0
        VOXEL_TILE_LOOKUP_WIDTH=512
        VOXEL_TILE_LOOKUP_HEIGHT=256
        VOXEL_SCALE_LAB=0
        VOXEL_ADAPTIVE_SDF_LAB=0
        VOXEL_FRACTAL_PLANET_SDF_LAB=0
        VOXEL_FRACTAL_VOXEL_SDF_LAB=0
        VOXEL_PORTAL_LAB=1
        VOXEL_INTRINSIC_PORTAL_LAB=0
        VOXEL_DEFAULT_PLANET_RADIUS=1.0F
        $<$<BOOL:${VOXEL_ENABLE_VALIDATION}>:VOXEL_ENABLE_VALIDATION=1>)
    if(MSVC)
        target_compile_options(voxel_engine_portal_lab PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(voxel_engine_portal_lab PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wno-missing-field-initializers)
    endif()
endif()

if(VOXEL_BUILD_INTRINSIC_PORTAL_LAB)
    add_voxel_shader_set(voxel_intrinsic_portal_lab_shaders
        shaders-intrinsic-portal-lab 512 256 0 0 0 0 1 1 0 0)
    add_executable(voxel_engine_intrinsic_portal_lab ${VOXEL_ENGINE_SOURCES})
    add_dependencies(voxel_engine_intrinsic_portal_lab
        voxel_intrinsic_portal_lab_shaders)
    target_include_directories(voxel_engine_intrinsic_portal_lab PRIVATE src)
    if(MINGW)
        target_include_directories(voxel_engine_intrinsic_portal_lab PRIVATE
            "${VULKAN_HEADER_DIR}")
    endif()
    target_link_libraries(voxel_engine_intrinsic_portal_lab PRIVATE
        ${VOXEL_VULKAN_TARGET} ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET}
        voxel_imgui Threads::Threads)
    if(WIN32)
        target_link_libraries(voxel_engine_intrinsic_portal_lab PRIVATE psapi)
    endif()
    if(VOXEL_ENABLE_JOLT)
        target_link_libraries(voxel_engine_intrinsic_portal_lab PRIVATE Jolt)
        target_compile_definitions(voxel_engine_intrinsic_portal_lab PRIVATE
            VOXEL_HAS_JOLT=1)
    endif()
    target_compile_definitions(voxel_engine_intrinsic_portal_lab PRIVATE
        SDL_MAIN_HANDLED
        VOXEL_SHADER_DIR="${CMAKE_CURRENT_BINARY_DIR}/shaders-intrinsic-portal-lab"
        VOXEL_GEODESIC_FREQUENCY=512
        VOXEL_TILE_RADIAL_LAYERS=32
        VOXEL_RESIDENT_GEODESIC_PAGES=8192
        VOXEL_ENABLE_REFERENCE_TRAVERSAL=0
        VOXEL_TILE_LOOKUP_WIDTH=512
        VOXEL_TILE_LOOKUP_HEIGHT=256
        VOXEL_SCALE_LAB=0
        VOXEL_ADAPTIVE_SDF_LAB=0
        VOXEL_FRACTAL_PLANET_SDF_LAB=0
        VOXEL_FRACTAL_VOXEL_SDF_LAB=0
        VOXEL_PORTAL_LAB=1
        VOXEL_INTRINSIC_PORTAL_LAB=1
        VOXEL_DEFAULT_PLANET_RADIUS=1.0F
        $<$<BOOL:${VOXEL_ENABLE_VALIDATION}>:VOXEL_ENABLE_VALIDATION=1>)
    if(MSVC)
        target_compile_options(voxel_engine_intrinsic_portal_lab PRIVATE
            /W4 /permissive- /EHsc)
    else()
        target_compile_options(voxel_engine_intrinsic_portal_lab PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
            -Wno-missing-field-initializers)
    endif()
endif()

if(VOXEL_BUILD_GLOBAL_METRIC_LAB)
    add_voxel_shader_set(voxel_global_metric_lab_shaders
        shaders-global-metric-lab 512 256 0 0 0 0 1 1 1 0)
    set(VOXEL_GLOBAL_SPATIAL_AA_SHADER
        "${CMAKE_CURRENT_BINARY_DIR}/shaders-global-metric-lab/planet_spatial_aa.comp.spv")
    set(VOXEL_GLOBAL_SPATIAL_AA_RESOLVE_SHADER
        "${CMAKE_CURRENT_BINARY_DIR}/shaders-global-metric-lab/planet_spatial_aa_resolve.comp.spv")
    add_custom_command(
        OUTPUT "${VOXEL_GLOBAL_SPATIAL_AA_SHADER}"
        COMMAND ${CMAKE_COMMAND} -E make_directory
                "${CMAKE_CURRENT_BINARY_DIR}/shaders-global-metric-lab"
        COMMAND "${GLSLC_EXECUTABLE}" --target-env=vulkan1.2 -O
                "-I${CMAKE_CURRENT_SOURCE_DIR}"
                -DVOXEL_ENABLE_REFERENCE_TRAVERSAL=0
                -DVOXEL_TILE_LOOKUP_WIDTH=512
                -DVOXEL_TILE_LOOKUP_HEIGHT=256
                -DVOXEL_SCALE_LAB=0
                -DVOXEL_ADAPTIVE_SDF_LAB=0
                -DVOXEL_FRACTAL_PLANET_SDF_LAB=0
                -DVOXEL_FRACTAL_VOXEL_SDF_LAB=0
                -DVOXEL_PORTAL_LAB=1
                -DVOXEL_INTRINSIC_PORTAL_LAB=1
                -DVOXEL_GLOBAL_METRIC_LAB=1
                -DVOXEL_GLOBAL_SPATIAL_AA_PASS=1
                "${CMAKE_CURRENT_SOURCE_DIR}/shaders/planet.comp"
                -o "${VOXEL_GLOBAL_SPATIAL_AA_SHADER}"
        COMMAND "${SPIRV_VAL_EXECUTABLE}" --target-env vulkan1.2
                "${VOXEL_GLOBAL_SPATIAL_AA_SHADER}"
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/shaders/planet.comp"
                ${VOXEL_SHADER_SHARED_SOURCES}
        VERBATIM)
    add_custom_command(
        OUTPUT "${VOXEL_GLOBAL_SPATIAL_AA_RESOLVE_SHADER}"
        COMMAND ${CMAKE_COMMAND} -E make_directory
                "${CMAKE_CURRENT_BINARY_DIR}/shaders-global-metric-lab"
        COMMAND "${GLSLC_EXECUTABLE}" --target-env=vulkan1.2 -O
                "-I${CMAKE_CURRENT_SOURCE_DIR}"
                -DVOXEL_ENABLE_REFERENCE_TRAVERSAL=0
                -DVOXEL_TILE_LOOKUP_WIDTH=512
                -DVOXEL_TILE_LOOKUP_HEIGHT=256
                -DVOXEL_SCALE_LAB=0
                -DVOXEL_ADAPTIVE_SDF_LAB=0
                -DVOXEL_FRACTAL_PLANET_SDF_LAB=0
                -DVOXEL_FRACTAL_VOXEL_SDF_LAB=0
                -DVOXEL_PORTAL_LAB=1
                -DVOXEL_INTRINSIC_PORTAL_LAB=1
                -DVOXEL_GLOBAL_METRIC_LAB=1
                -DVOXEL_GLOBAL_SPATIAL_AA_PASS=2
                "${CMAKE_CURRENT_SOURCE_DIR}/shaders/planet.comp"
                -o "${VOXEL_GLOBAL_SPATIAL_AA_RESOLVE_SHADER}"
        COMMAND "${SPIRV_VAL_EXECUTABLE}" --target-env vulkan1.2
                "${VOXEL_GLOBAL_SPATIAL_AA_RESOLVE_SHADER}"
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/shaders/planet.comp"
                ${VOXEL_SHADER_SHARED_SOURCES}
        VERBATIM)
    add_custom_target(voxel_global_metric_spatial_aa_shader
        DEPENDS "${VOXEL_GLOBAL_SPATIAL_AA_SHADER}"
                "${VOXEL_GLOBAL_SPATIAL_AA_RESOLVE_SHADER}")
    add_executable(voxel_engine_global_metric_lab ${VOXEL_ENGINE_SOURCES})
    add_dependencies(voxel_engine_global_metric_lab
        voxel_global_metric_lab_shaders
        voxel_global_metric_spatial_aa_shader)
    target_include_directories(voxel_engine_global_metric_lab PRIVATE src)
    if(MINGW)
        target_include_directories(voxel_engine_global_metric_lab PRIVATE
            "${VULKAN_HEADER_DIR}")
    endif()
    target_link_libraries(voxel_engine_global_metric_lab PRIVATE
        ${VOXEL_VULKAN_TARGET} ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET}
        voxel_imgui Threads::Threads)
    if(WIN32)
        target_link_libraries(voxel_engine_global_metric_lab PRIVATE psapi)
    endif()
    if(VOXEL_ENABLE_JOLT)
        target_link_libraries(voxel_engine_global_metric_lab PRIVATE Jolt)
        target_compile_definitions(voxel_engine_global_metric_lab PRIVATE
            VOXEL_HAS_JOLT=1)
    endif()
    target_compile_definitions(voxel_engine_global_metric_lab PRIVATE
        SDL_MAIN_HANDLED
        VOXEL_SHADER_DIR="${CMAKE_CURRENT_BINARY_DIR}/shaders-global-metric-lab"
        VOXEL_GEODESIC_FREQUENCY=512
        VOXEL_TILE_RADIAL_LAYERS=32
        VOXEL_RESIDENT_GEODESIC_PAGES=8192
        VOXEL_ENABLE_REFERENCE_TRAVERSAL=0
        VOXEL_TILE_LOOKUP_WIDTH=512
        VOXEL_TILE_LOOKUP_HEIGHT=256
        VOXEL_SCALE_LAB=0
        VOXEL_ADAPTIVE_SDF_LAB=0
        VOXEL_FRACTAL_PLANET_SDF_LAB=0
        VOXEL_FRACTAL_VOXEL_SDF_LAB=0
        VOXEL_PORTAL_LAB=1
        VOXEL_INTRINSIC_PORTAL_LAB=1
        VOXEL_GLOBAL_METRIC_LAB=1
        VOXEL_DEFAULT_PLANET_RADIUS=1.0F
        $<$<BOOL:${VOXEL_ENABLE_VALIDATION}>:VOXEL_ENABLE_VALIDATION=1>)
    if(MSVC)
        target_compile_options(voxel_engine_global_metric_lab PRIVATE
            /W4 /permissive- /EHsc)
    else()
        target_compile_options(voxel_engine_global_metric_lab PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
            -Wno-missing-field-initializers)
    endif()
endif()

if(VOXEL_BUILD_EIGHT_PLANET_SYSTEM_LAB)
    add_voxel_shader_set(voxel_eight_planet_system_lab_shaders
        shaders-eight-planet-system-lab 512 256 0 0 0 0 0 0 0 1)
    add_executable(voxel_engine_eight_planet_system_lab ${VOXEL_ENGINE_SOURCES})
    add_dependencies(voxel_engine_eight_planet_system_lab
        voxel_eight_planet_system_lab_shaders)
    target_include_directories(voxel_engine_eight_planet_system_lab PRIVATE src)
    if(MINGW)
        target_include_directories(voxel_engine_eight_planet_system_lab PRIVATE
            "${VULKAN_HEADER_DIR}")
    endif()
    target_link_libraries(voxel_engine_eight_planet_system_lab PRIVATE
        ${VOXEL_VULKAN_TARGET} ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET}
        voxel_imgui Threads::Threads)
    if(WIN32)
        target_link_libraries(voxel_engine_eight_planet_system_lab PRIVATE psapi)
    endif()
    if(VOXEL_ENABLE_JOLT)
        target_link_libraries(voxel_engine_eight_planet_system_lab PRIVATE Jolt)
        target_compile_definitions(voxel_engine_eight_planet_system_lab PRIVATE
            VOXEL_HAS_JOLT=1)
    endif()
    target_compile_definitions(voxel_engine_eight_planet_system_lab PRIVATE
        SDL_MAIN_HANDLED
        VOXEL_SHADER_DIR="${CMAKE_CURRENT_BINARY_DIR}/shaders-eight-planet-system-lab"
        VOXEL_GEODESIC_FREQUENCY=512
        VOXEL_TILE_RADIAL_LAYERS=32
        VOXEL_RESIDENT_GEODESIC_PAGES=8192
        VOXEL_ENABLE_REFERENCE_TRAVERSAL=0
        VOXEL_TILE_LOOKUP_WIDTH=512
        VOXEL_TILE_LOOKUP_HEIGHT=256
        VOXEL_SCALE_LAB=0
        VOXEL_ADAPTIVE_SDF_LAB=0
        VOXEL_FRACTAL_PLANET_SDF_LAB=0
        VOXEL_FRACTAL_VOXEL_SDF_LAB=0
        VOXEL_PORTAL_LAB=0
        VOXEL_INTRINSIC_PORTAL_LAB=0
        VOXEL_GLOBAL_METRIC_LAB=0
        VOXEL_EIGHT_PLANET_SYSTEM_LAB=1
        VOXEL_DEFAULT_PLANET_RADIUS=1.0F
        $<$<BOOL:${VOXEL_ENABLE_VALIDATION}>:VOXEL_ENABLE_VALIDATION=1>)
    if(MSVC)
        target_compile_options(voxel_engine_eight_planet_system_lab PRIVATE
            /W4 /permissive- /EHsc)
    else()
        target_compile_options(voxel_engine_eight_planet_system_lab PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
            -Wno-missing-field-initializers)
    endif()
endif()

if(MSVC)
    target_compile_options(voxel_engine PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(voxel_engine PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wno-missing-field-initializers)
endif()

if(WIN32)
    foreach(runtime_target IN ITEMS ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET})
        add_custom_command(TARGET voxel_engine POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${runtime_target}>
                $<TARGET_FILE_DIR:voxel_engine>
            VERBATIM
        )
    endforeach()
    if(VOXEL_BUILD_SCALE_LAB)
        foreach(runtime_target IN ITEMS ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET})
            add_custom_command(TARGET voxel_engine_scale_lab POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:${runtime_target}>
                    $<TARGET_FILE_DIR:voxel_engine_scale_lab>
                VERBATIM)
        endforeach()
    endif()
    if(VOXEL_BUILD_ADAPTIVE_SDF_LAB)
        foreach(runtime_target IN ITEMS ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET})
            add_custom_command(TARGET voxel_engine_adaptive_sdf_lab POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:${runtime_target}>
                    $<TARGET_FILE_DIR:voxel_engine_adaptive_sdf_lab>
                VERBATIM)
        endforeach()
    endif()
    if(VOXEL_BUILD_FRACTAL_PLANET_SDF_LAB)
        add_test(NAME fractal_planet_gpu_orbit
                 COMMAND voxel_engine_fractal_planet_sdf_lab --fractal-planet-orbit)
        add_test(NAME fractal_planet_gpu_rotation
                 COMMAND voxel_engine_fractal_planet_sdf_lab --fractal-planet-rotation)
        add_test(NAME fractal_planet_gpu_surface_player
                 COMMAND voxel_engine_fractal_planet_sdf_lab --fractal-planet-surface)
        add_test(NAME fractal_planet_gpu_artifact_regression
                 COMMAND voxel_engine_fractal_planet_sdf_lab --fractal-planet-artifact)
        set_tests_properties(
            fractal_planet_gpu_orbit fractal_planet_gpu_rotation
            fractal_planet_gpu_surface_player PROPERTIES
            LABELS "fractal-planet-sdf;gpu;vulkan;performance;isolation"
            TIMEOUT 60
            RESOURCE_LOCK voxel_gpu)
        set_tests_properties(fractal_planet_gpu_artifact_regression PROPERTIES
            LABELS "fractal-planet-sdf;gpu;vulkan;artifact;grazing;seams;lod"
            TIMEOUT 240
            RESOURCE_LOCK voxel_gpu)
    endif()
    if(VOXEL_BUILD_FRACTAL_PLANET_SDF_LAB)
        foreach(runtime_target IN ITEMS ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET})
            add_custom_command(TARGET voxel_engine_fractal_planet_sdf_lab POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:${runtime_target}>
                    $<TARGET_FILE_DIR:voxel_engine_fractal_planet_sdf_lab>
                VERBATIM)
        endforeach()
    endif()
    if(VOXEL_BUILD_FRACTAL_VOXEL_SDF_LAB)
        add_test(NAME fractal_voxel_sdf_gpu_orbit
                 COMMAND voxel_engine_fractal_voxel_sdf_lab --fractal-voxel-orbit)
        add_test(NAME fractal_voxel_sdf_gpu_rotation
                 COMMAND voxel_engine_fractal_voxel_sdf_lab --fractal-voxel-rotation)
        add_test(NAME fractal_voxel_sdf_gpu_surface_player
                 COMMAND voxel_engine_fractal_voxel_sdf_lab --fractal-voxel-surface)
        add_test(NAME fractal_voxel_sdf_gpu_artifact_regression
                 COMMAND voxel_engine_fractal_voxel_sdf_lab --fractal-voxel-artifact)
        set_tests_properties(
            fractal_voxel_sdf_gpu_orbit fractal_voxel_sdf_gpu_rotation
            fractal_voxel_sdf_gpu_surface_player PROPERTIES
            LABELS "fractal-voxel-sdf;gpu;vulkan;performance;isolation"
            TIMEOUT 120 RESOURCE_LOCK voxel_gpu)
        set_tests_properties(fractal_voxel_sdf_gpu_artifact_regression PROPERTIES
            LABELS "fractal-voxel-sdf;gpu;vulkan;artifact;zoom;continuity"
            TIMEOUT 300 RESOURCE_LOCK voxel_gpu)
        foreach(runtime_target IN ITEMS ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET})
            add_custom_command(TARGET voxel_engine_fractal_voxel_sdf_lab POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:${runtime_target}>
                    $<TARGET_FILE_DIR:voxel_engine_fractal_voxel_sdf_lab>
                VERBATIM)
        endforeach()
    endif()
    if(VOXEL_BUILD_PORTAL_LAB)
        foreach(runtime_target IN ITEMS ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET})
            add_custom_command(TARGET voxel_engine_portal_lab POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:${runtime_target}>
                    $<TARGET_FILE_DIR:voxel_engine_portal_lab>
                VERBATIM)
        endforeach()
    endif()
    if(VOXEL_BUILD_INTRINSIC_PORTAL_LAB)
        foreach(runtime_target IN ITEMS ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET})
            add_custom_command(TARGET voxel_engine_intrinsic_portal_lab POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:${runtime_target}>
                    $<TARGET_FILE_DIR:voxel_engine_intrinsic_portal_lab>
                VERBATIM)
        endforeach()
    endif()
    if(VOXEL_BUILD_GLOBAL_METRIC_LAB)
        foreach(runtime_target IN ITEMS ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET})
            add_custom_command(TARGET voxel_engine_global_metric_lab POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:${runtime_target}>
                    $<TARGET_FILE_DIR:voxel_engine_global_metric_lab>
                VERBATIM)
        endforeach()
    endif()
    if(VOXEL_BUILD_EIGHT_PLANET_SYSTEM_LAB)
        foreach(runtime_target IN ITEMS ${VOXEL_SDL_TARGET} ${VOXEL_OPENAL_TARGET})
            add_custom_command(TARGET voxel_engine_eight_planet_system_lab POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:${runtime_target}>
                    $<TARGET_FILE_DIR:voxel_engine_eight_planet_system_lab>
                VERBATIM)
        endforeach()
    endif()
endif()

if(BUILD_TESTING)
    add_executable(intrinsic_ellis_manifold_test
        tests/intrinsic_ellis_manifold_test.cpp)
    target_include_directories(intrinsic_ellis_manifold_test PRIVATE src)
    add_test(NAME intrinsic_ellis_manifold_invariants
             COMMAND intrinsic_ellis_manifold_test)
    set_tests_properties(intrinsic_ellis_manifold_invariants PROPERTIES
        LABELS "intrinsic-portal;cpu;geodesic;parallel-transport;continuity;oracle")

    add_executable(global_metric_field_test tests/global_metric_field_test.cpp)
    target_include_directories(global_metric_field_test PRIVATE src)
    add_test(NAME global_metric_field_invariants COMMAND global_metric_field_test)
    set_tests_properties(global_metric_field_invariants PROPERTIES
        LABELS "global-metric;cpu;null;geodesic;atlas;continuity;oracle")

    add_executable(eight_planet_system_test
        tests/eight_planet_system_test.cpp
        src/planet/geodesic_topology.cpp)
    target_include_directories(eight_planet_system_test PRIVATE src)
    add_test(NAME eight_planet_system_invariants COMMAND eight_planet_system_test)
    set_tests_properties(eight_planet_system_invariants PROPERTIES
        LABELS "eight-planet-system;cpu;hex-lod;wormhole;finite-star;isolation")

    add_executable(portal_lab_test tests/portal_lab_test.cpp)
    target_include_directories(portal_lab_test PRIVATE src)
    add_test(NAME spherical_wormhole_portal_invariants COMMAND portal_lab_test)
    set_tests_properties(spherical_wormhole_portal_invariants PROPERTIES
        LABELS "portal-lab;cpu;ray;geodesic;continuity;finite;isolation")

    add_executable(fractal_voxel_sdf_test tests/fractal_voxel_sdf_test.cpp)
    target_include_directories(fractal_voxel_sdf_test PRIVATE src)
    add_test(NAME fractal_voxel_sdf_cache_invariants
             COMMAND fractal_voxel_sdf_test)
    set_tests_properties(fractal_voxel_sdf_cache_invariants PROPERTIES
        LABELS "fractal-voxel-sdf;cpu;zoom;cache;hysteresis;continuity;oracle")

    add_test(NAME vulkan_gpu_smoke COMMAND voxel_engine --smoke-test)
    set_tests_properties(vulkan_gpu_smoke PROPERTIES
        LABELS "milestone;gpu;vulkan"
        TIMEOUT 60
        RESOURCE_LOCK voxel_gpu)
    add_test(NAME vulkan_gpu_sparse_streaming COMMAND voxel_engine --stream-test)
    set_tests_properties(vulkan_gpu_sparse_streaming PROPERTIES
        LABELS "milestone;gpu;vulkan;sparse;streaming"
        TIMEOUT 60
        RESOURCE_LOCK voxel_gpu)
    add_test(NAME vulkan_gpu_surface_artifact_regression
             COMMAND voxel_engine --artifact-regression)
    set_tests_properties(vulkan_gpu_surface_artifact_regression PROPERTIES
        LABELS "milestone;gpu;vulkan;render;surface;grazing;artifact"
        TIMEOUT 240
        RESOURCE_LOCK voxel_gpu)
    if(VOXEL_BUILD_PORTAL_LAB)
        add_test(NAME portal_lab_gpu_smoke
                 COMMAND voxel_engine_portal_lab --smoke-test)
        add_test(NAME portal_lab_gpu_benchmark
                 COMMAND voxel_engine_portal_lab --benchmark)
        add_test(NAME portal_lab_gpu_artifact_regression
                 COMMAND voxel_engine_portal_lab --artifact-regression)
        set_tests_properties(portal_lab_gpu_smoke portal_lab_gpu_benchmark PROPERTIES
            LABELS "portal-lab;gpu;vulkan;performance;isolation"
            TIMEOUT 90 RESOURCE_LOCK voxel_gpu)
        set_tests_properties(portal_lab_gpu_artifact_regression PROPERTIES
            LABELS "portal-lab;gpu;vulkan;artifact;grazing;finite;isolation"
            TIMEOUT 240 RESOURCE_LOCK voxel_gpu)
    endif()
    if(VOXEL_BUILD_INTRINSIC_PORTAL_LAB)
        add_test(NAME intrinsic_portal_lab_gpu_smoke
                 COMMAND voxel_engine_intrinsic_portal_lab --smoke-test)
        add_test(NAME intrinsic_portal_lab_gpu_benchmark
                 COMMAND voxel_engine_intrinsic_portal_lab --benchmark)
        add_test(NAME intrinsic_portal_lab_gpu_artifact_regression
                 COMMAND voxel_engine_intrinsic_portal_lab
                         --intrinsic-traversal-regression)
        set_tests_properties(
            intrinsic_portal_lab_gpu_smoke intrinsic_portal_lab_gpu_benchmark
            PROPERTIES LABELS "intrinsic-portal;gpu;vulkan;performance;isolation"
            TIMEOUT 90 RESOURCE_LOCK voxel_gpu)
        set_tests_properties(intrinsic_portal_lab_gpu_artifact_regression PROPERTIES
            LABELS "intrinsic-portal;gpu;vulkan;artifact;finite;isolation"
            TIMEOUT 240 RESOURCE_LOCK voxel_gpu)
    endif()
    if(VOXEL_BUILD_GLOBAL_METRIC_LAB)
        add_test(NAME global_metric_lab_gpu_smoke
                 COMMAND voxel_engine_global_metric_lab --smoke-test
                         --headless-capture
                         "${CMAKE_CURRENT_BINARY_DIR}/test-artifacts/global-smoke.png"
                         --capture-frames 3)
        add_test(NAME global_metric_lab_gpu_benchmark
                 COMMAND voxel_engine_global_metric_lab --benchmark
                         --headless-capture
                         "${CMAKE_CURRENT_BINARY_DIR}/test-artifacts/global-benchmark.png"
                         --capture-frames 120)
        add_test(NAME global_metric_lab_gpu_traversal
                 COMMAND voxel_engine_global_metric_lab
                         --intrinsic-traversal-regression
                         --headless-capture
                         "${CMAKE_CURRENT_BINARY_DIR}/test-artifacts/global-traversal.png"
                         --capture-frames 160)
        add_test(NAME global_metric_lab_visual_media
                 COMMAND voxel_engine_global_metric_lab
                         --global-visual-regression
                         --headless-capture
                         "${CMAKE_CURRENT_BINARY_DIR}/test-artifacts/global-media.png"
                         --capture-frames 180)
        add_test(NAME global_metric_lab_visual_no_media
                 COMMAND voxel_engine_global_metric_lab
                         --global-visual-regression-no-media
                         --headless-capture
                         "${CMAKE_CURRENT_BINARY_DIR}/test-artifacts/global-no-media.png"
                         --capture-frames 180)
        set_tests_properties(global_metric_lab_gpu_smoke
            global_metric_lab_gpu_benchmark global_metric_lab_gpu_traversal
            global_metric_lab_visual_media global_metric_lab_visual_no_media
            PROPERTIES LABELS "global-metric;gpu;vulkan;geodesic;isolation"
            TIMEOUT 240 RESOURCE_LOCK voxel_gpu)
        set_tests_properties(global_metric_lab_visual_media
            global_metric_lab_visual_no_media PROPERTIES
            LABELS "global-metric;gpu;vulkan;geodesic;isolation;artifact;image;media"
            TIMEOUT 240 RESOURCE_LOCK voxel_gpu)
    endif()
    if(VOXEL_BUILD_EIGHT_PLANET_SYSTEM_LAB)
        foreach(capture_preset IN ITEMS overview near mixed-lod)
            add_test(NAME eight_planet_system_${capture_preset}
                COMMAND voxel_engine_eight_planet_system_lab
                    --headless-capture
                    "${CMAKE_CURRENT_BINARY_DIR}/test-artifacts/system-${capture_preset}.png"
                    --capture-frames 12 --system-capture ${capture_preset})
            set_tests_properties(eight_planet_system_${capture_preset} PROPERTIES
                LABELS "eight-planet-system;gpu;vulkan;hidden;isolation"
                TIMEOUT 120 RESOURCE_LOCK voxel_gpu)
        endforeach()
        add_test(NAME eight_planet_system_traversal-8-1
            COMMAND voxel_engine_eight_planet_system_lab
                --headless-capture
                "${CMAKE_CURRENT_BINARY_DIR}/test-artifacts/system-traversal-8-1.png"
                --capture-frames 96 --system-capture traversal-8-1)
        set_tests_properties(eight_planet_system_traversal-8-1 PROPERTIES
            LABELS "eight-planet-system;gpu;vulkan;wormhole;media;moving-frame;hidden;isolation"
            TIMEOUT 180 RESOURCE_LOCK voxel_gpu)
        add_test(NAME eight_planet_system_edit_persistence
            COMMAND voxel_engine_eight_planet_system_lab
                --headless-capture
                "${CMAKE_CURRENT_BINARY_DIR}/test-artifacts/system-edit-persistence.png"
                --capture-frames 48 --system-capture edit-persistence)
        set_tests_properties(eight_planet_system_edit_persistence PROPERTIES
            LABELS "eight-planet-system;gpu;vulkan;edits;eviction;collision;hidden;isolation"
            TIMEOUT 180 RESOURCE_LOCK voxel_gpu)
        add_test(NAME eight_planet_system_finest_differential
            COMMAND voxel_engine_eight_planet_system_lab
                --headless-capture
                "${CMAKE_CURRENT_BINARY_DIR}/test-artifacts/system-differential.png"
                --capture-frames 6 --system-capture differential)
        set_tests_properties(eight_planet_system_finest_differential PROPERTIES
            LABELS "eight-planet-system;gpu;vulkan;differential;seams;pentagons;wormhole;hidden;isolation"
            TIMEOUT 180 RESOURCE_LOCK voxel_gpu)
        add_test(NAME eight_planet_system_benchmark
            COMMAND voxel_engine_eight_planet_system_lab --benchmark
                --headless-capture
                "${CMAKE_CURRENT_BINARY_DIR}/test-artifacts/system-benchmark.png"
                --capture-frames 120 --system-capture overview)
        set_tests_properties(eight_planet_system_benchmark PROPERTIES
            LABELS "eight-planet-system;gpu;vulkan;performance;hidden;isolation"
            TIMEOUT 180 RESOURCE_LOCK voxel_gpu)
        add_test(NAME eight_planet_system_cycle_audit
            COMMAND voxel_engine_eight_planet_system_lab
                --headless-capture
                "${CMAKE_CURRENT_BINARY_DIR}/test-artifacts/system-cycle-audit.png"
                --capture-frames 12 --system-capture cycle-audit)
        set_tests_properties(eight_planet_system_cycle_audit PROPERTIES
            LABELS "eight-planet-system;gpu;vulkan;wormhole;all-endpoints;hidden;isolation"
            TIMEOUT 120 RESOURCE_LOCK voxel_gpu)
    endif()
    if(VOXEL_BUILD_ADAPTIVE_SDF_LAB)
        add_test(NAME adaptive_sdf_gpu_orbit
                 COMMAND voxel_engine_adaptive_sdf_lab --adaptive-sdf-orbit)
        add_test(NAME adaptive_sdf_gpu_rotation
                 COMMAND voxel_engine_adaptive_sdf_lab --adaptive-sdf-rotation)
        add_test(NAME adaptive_sdf_gpu_surface_player
                 COMMAND voxel_engine_adaptive_sdf_lab --adaptive-sdf-surface)
        add_test(NAME adaptive_sdf_gpu_artifact_regression
                 COMMAND voxel_engine_adaptive_sdf_lab --adaptive-sdf-artifact)
        set_tests_properties(
            adaptive_sdf_gpu_orbit adaptive_sdf_gpu_rotation
            adaptive_sdf_gpu_surface_player PROPERTIES
            LABELS "adaptive-sdf;gpu;vulkan;performance;isolation"
            TIMEOUT 60
            RESOURCE_LOCK voxel_gpu)
        set_tests_properties(adaptive_sdf_gpu_artifact_regression PROPERTIES
            LABELS "adaptive-sdf;gpu;vulkan;artifact;grazing;seams;lod"
            TIMEOUT 240
            RESOURCE_LOCK voxel_gpu)
    endif()
    add_custom_target(milestone_gates
        COMMAND "${CMAKE_CTEST_COMMAND}" --output-on-failure
        DEPENDS voxel_engine geodesic_topology_test geodesic_geometry_test
                geodesic_sparse_residency_test geodesic_visual_consistency_test
                procedural_heightmap_test camera_zoom_test
                surface_camera_controller_test
                geodesic_local_ao_test planet_lighting_test space_environment_test
                planet_atmosphere_test volumetric_clouds_test
                fractal_voxel_sdf_test portal_lab_test
                intrinsic_ellis_manifold_test
                scale_lab_guardrails_test geodesic_topology_cache_test
                geodesic_distance_bound_test voxel_shaders
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
        USES_TERMINAL
    )
    add_executable(scale_lab_guardrails_test
        tests/scale_lab_guardrails_test.cpp
        src/planet/scale_lab_guardrails.cpp)
    target_include_directories(scale_lab_guardrails_test PRIVATE src)
    add_test(NAME scale_lab_configuration_guardrails COMMAND scale_lab_guardrails_test)
    set_tests_properties(scale_lab_configuration_guardrails PROPERTIES
        LABELS "milestone;cpu;scale-lab;bounds;isolation")
    add_executable(geodesic_topology_cache_test
        tests/geodesic_topology_cache_test.cpp
        src/planet/geodesic_topology.cpp
        src/planet/geodesic_topology_cache.cpp)
    target_include_directories(geodesic_topology_cache_test PRIVATE src)
    add_test(NAME geodesic_topology_cache_integrity COMMAND geodesic_topology_cache_test)
    set_tests_properties(geodesic_topology_cache_integrity PROPERTIES
        LABELS "milestone;cpu;scale-lab;cache;watchdog;integrity")
    add_executable(geodesic_distance_bound_test
        tests/geodesic_distance_bound_test.cpp)
    target_include_directories(geodesic_distance_bound_test PRIVATE src)
    add_test(NAME geodesic_distance_bound_invariants COMMAND geodesic_distance_bound_test)
    set_tests_properties(geodesic_distance_bound_invariants PROPERTIES
        LABELS "milestone;cpu;scale-lab;bounds;seams;ulp;differential")
    add_executable(geodesic_local_ao_test tests/geodesic_local_ao_test.cpp)
    target_include_directories(geodesic_local_ao_test PRIVATE src)
    add_test(NAME geodesic_local_ao_invariants COMMAND geodesic_local_ao_test)
    set_tests_properties(geodesic_local_ao_invariants PROPERTIES
        LABELS "milestone;cpu;render;ao;image;differential;seams;residency;stability")
    add_executable(planet_lighting_test tests/planet_lighting_test.cpp)
    target_include_directories(planet_lighting_test PRIVATE src)
    add_test(NAME planet_lighting_invariants COMMAND planet_lighting_test)
    set_tests_properties(planet_lighting_invariants PROPERTIES
        LABELS "milestone;cpu;render;lighting;ao;material;residency;stability")
    add_executable(space_environment_test tests/space_environment_test.cpp)
    target_include_directories(space_environment_test PRIVATE src)
    add_test(NAME space_environment_invariants COMMAND space_environment_test)
    set_tests_properties(space_environment_invariants PROPERTIES
        LABELS "milestone;cpu;render;space;stars;horizon;translation;stability")
    add_executable(planet_atmosphere_test tests/planet_atmosphere_test.cpp)
    target_include_directories(planet_atmosphere_test PRIVATE src)
    add_test(NAME planet_atmosphere_invariants COMMAND planet_atmosphere_test)
    set_tests_properties(planet_atmosphere_invariants PROPERTIES
        LABELS "milestone;cpu;render;atmosphere;shell;aerial-perspective;stars;translation;stability")
    add_executable(volumetric_clouds_test tests/volumetric_clouds_test.cpp)
    target_include_directories(volumetric_clouds_test PRIVATE src)
    add_test(NAME volumetric_cloud_invariants COMMAND volumetric_clouds_test)
    set_tests_properties(volumetric_cloud_invariants PROPERTIES
        LABELS "milestone;cpu;render;clouds;shell;density;transmittance;shadows;translation;stability")
    add_executable(geodesic_micro_sdf_test tests/geodesic_micro_sdf_test.cpp)
    target_include_directories(geodesic_micro_sdf_test PRIVATE src)
    add_test(NAME geodesic_micro_sdf_invariants COMMAND geodesic_micro_sdf_test)
    set_tests_properties(geodesic_micro_sdf_invariants PROPERTIES
        LABELS "milestone;cpu;render;terrain;micro-sdf;bounds;determinism;seams;residency;stability")
    add_dependencies(milestone_gates geodesic_micro_sdf_test)
    add_executable(adaptive_planet_sdf_test tests/adaptive_planet_sdf_test.cpp)
    target_include_directories(adaptive_planet_sdf_test PRIVATE src)
    add_test(NAME adaptive_fractal_planet_sdf_invariants COMMAND adaptive_planet_sdf_test)
    set_tests_properties(adaptive_fractal_planet_sdf_invariants PROPERTIES
        LABELS "milestone;cpu;render;terrain;adaptive-sdf;bounds;determinism;seams;lod;stability")
    add_dependencies(milestone_gates adaptive_planet_sdf_test)
    add_executable(fractal_planet_sdf_test tests/fractal_planet_sdf_test.cpp)
    target_include_directories(fractal_planet_sdf_test PRIVATE src)
    add_test(NAME true_fractal_planet_sdf_invariants COMMAND fractal_planet_sdf_test)
    set_tests_properties(true_fractal_planet_sdf_invariants PROPERTIES
        LABELS "milestone;cpu;render;terrain;fractal-planet-sdf;bounds;determinism;seams;lod;stability")
    add_dependencies(milestone_gates fractal_planet_sdf_test)
endif()
