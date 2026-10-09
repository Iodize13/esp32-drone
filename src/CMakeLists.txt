idf_component_register(
    SRCS "main.c"
         "app/app_util.c" "app/attitude_est.c" "app/attitude_filter.c"
         "app/battery.c" "app/console.c" "app/mixer.c" "app/pid.c"
         "app/test_bench.c" "app/test_mapping.c" "app/test_pid.c"
         "drivers/adc_batt.c" "drivers/button.c" "drivers/imu_int.c"
         "drivers/motor_pwm.c" "drivers/mpu6500.c" "drivers/serial_keys.c"
    INCLUDE_DIRS "." "app" "drivers")
