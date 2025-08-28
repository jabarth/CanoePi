#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/cyw43_arch.h>

#include <FreeRTOS.h>
#include <task.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32_multi_array.h>

// Pin Definitions
// Rotary Encoder
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3
#define ENCODER_SW_PIN 4
// RGB LED
#define RGB_R_PIN 5
#define RGB_G_PIN 6
#define RGB_B_PIN 7
// RPM Sensor
#define RPM_PIN 8
// PWM Lights
#define LIGHT_PWM_PIN_START 10
#define NUM_LIGHT_CIRCUITS 6
// Bilge
#define BILGE_STATUS_PIN 9
#define BILGE_CONTROL_PIN 16
// ADC
#define OIL_LEVEL_PIN 26
#define VOLTAGE_PIN 27

// Micro-ROS variables
rcl_publisher_t oil_publisher;
rcl_publisher_t rpm_publisher;
rcl_publisher_t bilge_status_publisher;
rcl_publisher_t light_levels_publisher;
rcl_publisher_t voltage_publisher;
rcl_subscription_t bilge_control_subscriber;
rcl_subscription_t light_settings_subscriber;

std_msgs__msg__Float32 oil_msg;
std_msgs__msg__Int32 rpm_msg;
std_msgs__msg__Bool bilge_status_msg;
std_msgs__msg__Int32MultiArray light_levels_msg;
std_msgs__msg__Float32 voltage_msg;
std_msgs__msg__Bool bilge_control_msg;
std_msgs__msg__Int32MultiArray light_settings_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// FreeRTOS Task Handles
TaskHandle_t micro_ros_task_handle;
TaskHandle_t sensor_task_handle;
TaskHandle_t encoder_task_handle;

// --- Helper Functions ---
void on_bilge_control(const void * msin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msin;
    gpio_put(BILGE_CONTROL_PIN, msg->data);
}

void on_light_settings(const void * msin) {
    // Implementation for setting light PWM based on received array
}

// --- FreeRTOS Tasks ---
void micro_ros_task(void *p) {
    // Micro-ROS setup
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    // Create publishers
    rclc_publisher_init_default(&oil_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "oil_level");
    // ... initialize other publishers

    // Create subscribers
    rclc_subscription_init_default(&bilge_control_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "bilge_control");
    // ... initialize other subscribers

    // Create executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &bilge_control_subscriber, &bilge_control_msg, &on_bilge_control, ON_NEW_DATA);
    // ... add other subscribers

    while(true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void sensor_task(void *p) {
    // ADC setup
    adc_init();
    adc_gpio_init(OIL_LEVEL_PIN);
    adc_gpio_init(VOLTAGE_PIN);

    while(true) {
        // Read oil level
        adc_select_input(0); // ADC0 is GP26
        oil_msg.data = adc_read() * 3.3f / (1 << 12);
        rcl_publish(&oil_publisher, &oil_msg, NULL);

        // Read voltage
        adc_select_input(1); // ADC1 is GP27
        // NOTE: Add scaling factor for voltage divider
        voltage_msg.data = adc_read() * 3.3f / (1 << 12);
        rcl_publish(&voltage_publisher, &voltage_msg, NULL);

        // Read bilge status
        bilge_status_msg.data = gpio_get(BILGE_STATUS_PIN);
        rcl_publish(&bilge_status_publisher, &bilge_status_msg, NULL);

        // PIO for RPM - implementation needed
        // For now, publish a dummy value
        rpm_msg.data = 1500;
        rcl_publish(&rpm_publisher, &rpm_msg, NULL);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void encoder_task(void *p) {
    // GPIO setup for encoder
    // ...

    while(true) {
        // Read encoder and button presses
        // Update light PWM and publish light levels
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


int main() {
    stdio_init_all();

    // GPIO Init
    // ...

    // Create FreeRTOS tasks
    xTaskCreate(micro_ros_task, "micro_ros_task", 4096, NULL, 1, &micro_ros_task_handle);
    xTaskCreate(sensor_task, "sensor_task", 1024, NULL, 1, &sensor_task_handle);
    xTaskCreate(encoder_task, "encoder_task", 1024, NULL, 1, &encoder_task_handle);

    vTaskStartScheduler();

    while(1){};
    return 0;
}
