zephyr_library()

zephyr_library_sources_ifdef(CONFIG_PAW3395 
    src/paw3395.c
    src/paw3395_priv.c)

zephyr_include_directories(
    include)