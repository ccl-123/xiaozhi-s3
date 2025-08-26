set(COMPONENT_SRCS
    "qmi8658.cc"
)

set(COMPONENT_ADD_INCLUDEDIRS
    "."
)

set(COMPONENT_REQUIRES
    driver
    esp_common
    freertos
)

register_component()
