set(component_srcs "src/bme280.c"
                   "src/sensor_driver.c"
                   "src/bme280_sensor_driver.c"
)

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver"
                       REQUIRES "")
