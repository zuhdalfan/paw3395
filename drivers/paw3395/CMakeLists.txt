# modules/button123/drivers/button123/CMakeLists.txt
zephyr_library()

zephyr_library_sources(
  paw3395.c
  paw3395_priv.c
)

zephyr_library_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    .
    )