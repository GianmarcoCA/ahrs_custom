////////////////////////////////////////////////////////////////////////////////
/// @file 7_series_ahrs_example.c
///
/// @defgroup _7_series_ahrs_example_c 7-Series AHRS Example [C]
///
/// @ingroup examples_c
///
/// @brief Example setup program for the 3DM-CV7-AHRS, and 3DM-GV7-AHRS using C
///
/// @details This example shows a basic setup to configure the attitude filter
///          with magnetometer as the heading source to stream filter data for
///          the 3DM-CV7-AHRS, and 3DM-GV7-AHRS using C. This is not an
///          exhaustive example of all settings for those devices. If this
///          example does not meet your specific setup needs, please consult the
///          MIP SDK API documentation for the proper commands.
///
/// @section _7_series_ahrs_example_c_license License
///
/// @copyright Copyright (c) 2025 MicroStrain by HBK
///            Licensed under MIT License
///
/// @{
///

// Include the MicroStrain Serial connection header
#include <microstrain/connections/serial/serial_port.h>

// Include the MicroStrain logging header for custom logging
#include <microstrain/logging.h>

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.h>
#include <mip/mip_interface.h>
#include <mip/definitions/commands_3dm.h>
#include <mip/definitions/commands_base.h>
#include <mip/definitions/commands_filter.h>
#include <mip/definitions/data_sensor.h>
#include <mip/definitions/data_filter.h>
#include <mip/definitions/data_shared.h>

#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif // _MSC_VER

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <dirent.h>
#include <assert.h>

#ifdef _MSC_VER
// MSVC doesn't support pthread
// Wrapping basic pthread functionality through threads.h
#include <threads.h>
typedef thrd_t                     pthread_t;
typedef struct pthread_attr_t      pthread_attr_t;
typedef mtx_t                      pthread_mutex_t;
typedef struct pthread_mutexattr_t pthread_mutexattr_t;

int  pthread_mutex_init(pthread_mutex_t* m, const pthread_mutexattr_t* a);
int  pthread_mutex_destroy(pthread_mutex_t* m);
int  pthread_mutex_lock(pthread_mutex_t* m);
int  pthread_mutex_unlock(pthread_mutex_t* m);
int  pthread_create(pthread_t* th, const pthread_attr_t* attr, void* (*func)(void*), void* arg);
int  pthread_join(pthread_t t, void** res);
int  nanosleep(const struct timespec* request, struct timespec* remain);
void sched_yield();
#else
#include <pthread.h>
#endif // _MSC_VER


////////////////////////////////////////////////////////////////////////////////
// NOTE: Setting these globally for example purposes

/*********************************************************************
 * DEFINES
 */

// Constantes necesarias
#define UNIX_GPS_EPOCH_OFFSET 315964800L // Segundos entre 1970 y 1980
#define GPS_LEAP_SECONDS 18              // Diferencia actual entre UTC y GPS
#define SECONDS_PER_WEEK 604800L

#define CAPTURE_DURATION_TIME 20000 // Time en ms to capture Gyro Bias
#define MAGNETOMETER_FEATURE 0 // Set to 1 to include magnetometer data and AHRS mode, set to 0 to exclude magnetometer data and use vertical gyro mode

#define MAX_FILENAME_LENGTH 150 // Buffer size for file name

#define INDIVIDUAL_SCRIPT 1 // Set this option to enable individual script testing

#define MAX_PATH 512

#define DEFAULT_CONFIG 1 // Set this option to configure the device with the default settings for the 7-Series AHRS devices. 

#define SC_CONFIG  1 // Set this option to configure the device with the settings used for the SensorConnect application.

// TODO: Enable/disable data collection threading
/// @brief Use this to test the behaviors of threading
#define USE_THREADS true

/// @brief Maximum time between sample buffer output operations
static const mip_timeout SAMPLE_PRINT_PERIOD_MS = 50000;

// Constantes necesarias
#define UNIX_GPS_EPOCH_OFFSET 315964800L // Segundos entre 1970 y 1980
#define GPS_LEAP_SECONDS 18              // Diferencia actual entre UTC y GPS
#define SECONDS_PER_WEEK 604800L

/*********************************************************************
 * CONSTANTS
 */

/// @brief  Set the baudrate for the connection (Serial/USB)
/// @note For native serial connections this needs to be 115200 due to the device default settings command
/// Use mip_base_*_comm_speed() to write and save the baudrate on the device
static uint32_t BAUDRATE = 0;
//static const uint32_t BAUDRATE = 115200;
// TODO: Update to the desired streaming rate. Setting low for readability purposes
/// @brief Streaming rate in Hz (applies to both sensor and filter data)

#if SC_CONFIG == 0
static uint16_t SAMPLE_RATE_HZ = 100;
/// @brief OPTIONAL: Set specific sensor decimation (0 = auto-calculate from SAMPLE_RATE_HZ)
/// Use this only if you want a different rate for sensor data than filter data
static uint16_t SENSOR_DECIMATION = 0;  // 0 = use SAMPLE_RATE_HZ

/// @brief OPTIONAL: Set specific filter decimation (0 = auto-calculate from SAMPLE_RATE_HZ)
/// Use this only if you want a different rate for filter data than sensor data
static uint16_t FILTER_DECIMATION = 0;  // 0 = use SAMPLE_RATE_HZ
#endif

// TODO: Update to change the example run time
/// @brief Example run time
static const uint32_t RUN_TIME_SECONDS = 30;


/*********************************************************************
 * GLOBAL VARIABLES
 */ 
int fileSetCounter = 1; // Contador global para los archivos


/*********************************************************************
 * TYPEDEF STRUCTS
 */

// Estructura para mantener los archivos CSV abiertos
typedef struct {
    FILE *csv1;
    FILE *csv2;
} CsvFiles;

#define SAMPLE_BUFFER_SIZE 100


typedef struct {
    mip_timestamp timestamp;
    mip_shared_gps_timestamp_data        gpsTs;
    mip_sensor_scaled_accel_data         accel;
    mip_sensor_delta_velocity_data       deltaV;
    mip_sensor_comp_quaternion_data      q;
    mip_shared_reference_timestamp_data  inTime;
} sensorData_t;

typedef struct{
    mip_timestamp timestamp;
    mip_shared_gps_timestamp_data               gpsTs;
    mip_filter_status_data                      status;
    mip_filter_comp_angular_rate_data           ang;
    mip_filter_linear_accel_data                accel;
    mip_filter_attitude_quaternion_data         q;
    mip_filter_gravity_vector_data              g;
} filterData_t;

typedef union {
    uint8_t value;      // Acceso al dato completo (16 bits)
    struct {
        uint8_t gpsTs  : 1;
        uint8_t accel  : 1;
        uint8_t deltaV : 1;
        uint8_t q      : 1;
        uint8_t inTime : 1;
        // ... puedes nombrar hasta el bit15
    } bits;
} sensorR_t;


typedef union {
    uint8_t value;      // Acceso al dato completo (16 bits)
    struct {
        uint8_t gpsTs   : 1;
        uint8_t status  : 1;
        uint8_t ang     : 1;
        uint8_t accel   : 1;
        uint8_t q       : 1;
        uint8_t g       : 1;
        // ... puedes nombrar hasta el bit15
    } bits;
} filterR_t;


typedef struct {

    sensorData_t sample[SAMPLE_BUFFER_SIZE];
    size_t count;
    mip_timestamp  last_flush_time;
    sensorData_t pending_sensorData;
    sensorR_t    pending_sensor;
    pthread_mutex_t buffer_mutex;  // Mutex for thread-safe access
    FILE* output_files;
} sensor_buffer_t;

typedef struct {

    filterData_t sample[SAMPLE_BUFFER_SIZE];
    size_t count;
    mip_timestamp  last_flush_time;
    filterData_t pending_filterData;
    filterR_t    pending_filter;
    pthread_mutex_t buffer_mutex;  // Mutex for thread-safe access
    FILE* output_files;
} filter_buffer_t;



#if USE_THREADS
// Basic structure for thread data
typedef struct thread_data
{
    mip_interface* device;
    volatile bool  running;
} thread_data_t;
#endif // USE_THREADS


/*********************************************************************
 * PUBLIC FUNCTIONS
 */
static void flush_sensor_buffer(sensor_buffer_t* _buffer);
static void flush_filter_buffer(filter_buffer_t* _buffer);

// Forward declarations
static void log_callback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);
static void sensor_packet_callback(void* _user, const mip_packet_view* _packet_view, mip_timestamp _timestamp);
static void filter_packet_callback(void* _user, const mip_packet_view* _packet_view, mip_timestamp _timestamp);

#if USE_THREADS
// Threaded functions
static bool  update_device(mip_interface* _device, mip_timeout _wait_time, bool _from_cmd);
static void* data_collection_thread(void* _thread_data);
#endif // USE_THREADS

uint64_t get_unix_time_ns(void); 

// Función para generar un nombre de archivo único
void generateFileName(char *baseName, int counter, char *fileName);

// Función para crear y abrir tres archivos CSV
CsvFiles createCsvFiles(void);

// Función para cerrar los archivos CSV
void closeCsvFiles(FILE* files);

// Función para encontrar el puerto del dispositivo basado en una palabra clave
char* find_device(const char* keyword);

void sync_device_time(mip_interface* device) ;

uint64_t mip_gps_to_unix_ms(mip_shared_gps_timestamp_data gps_data);

/*********************************************************************
 * PRIVATE  FUNCTIONS
 */


 
////////////////////////////////////////////////////////////////////////////////
/// @addtogroup _7_series_ahrs_example_c
/// @{
///
#if SC_CONFIG == 0

// Capture gyro bias
static void capture_gyro_bias(mip_interface* _device);


// Sensor message format configuration
static void configure_sensor_message_format(mip_interface* _device);

// Filter message format configuration
static void configure_filter_message_format(mip_interface* _device);

#endif

// Filter initialization
static void initialize_filter(mip_interface* _device);

// Used for basic timestamping (since epoch in milliseconds)
// TODO: Update this to whatever timestamping method is desired
static mip_timestamp get_current_timestamp();

// Device callbacks used for reading and writing packets
static bool mip_interface_user_send_to_device(mip_interface* _device, const uint8_t* _data, size_t _length);
static bool mip_interface_user_recv_from_device(
    mip_interface* _device, uint8_t* _buffer, size_t _max_length, mip_timeout _wait_time, bool _from_cmd,
    size_t* _length_out, mip_timestamp* _timestamp_out
);

// Common device initialization procedure
static void initialize_device(mip_interface* _device, serial_port* _device_port, const uint32_t _baudrate);

// Utility functions the handle application closing and printing error messages
static void terminate(serial_port* _device_port, const char* _message, const bool _successful);

static void exit_from_command(const mip_interface* _device, const mip_cmd_result _cmd_result, const char* _format, ...);


int main(const int argc, const char* argv[])
{
    // Note: This is a compile-time way of checking that the proper logging level is enabled
    // Note: The max available logging level may differ in pre-packaged installations of the MIP SDK
    
    // Mark printf operations as unbuffered to flush with every operation
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    
    #ifndef MICROSTRAIN_LOGGING_ENABLED_INFO
    #error This example requires a logging level of at least MICROSTRAIN_LOGGING_LEVEL_INFO_ to work properly
    #endif // !MICROSTRAIN_LOGGING_ENABLED_INFO

#if USE_THREADS
    // Create a mutex for the logging callbacks when multi-threading
    fprintf(stdout, "Initializing the threading mutex.\n");
    pthread_mutex_t lock;
    if (pthread_mutex_init(&lock, NULL) != 0)
    {
        fprintf(stderr, "Failed to initialize the threading mutex!\n");
        fprintf(stdout, "Press 'Enter' to exit the program.\n");
        getc(stdin);
        return 1;
    }
#endif // USE_THREADS

    // Initialize the custom logger to print messages/errors as they occur
    // Note: The logging level parameter doesn't need to match the max logging level.
    // If the parameter is higher than the max level, higher-level logging functions will be ignored
#if USE_THREADS
    MICROSTRAIN_LOG_INIT(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, (void*)&lock);
#else
    MICROSTRAIN_LOG_INIT(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, NULL);
#endif // USE_THREADS

    // Unused parameters
    // Unused parameters
  
    
    // Initialize the connection
    MICROSTRAIN_LOG_INFO("Initializing the connection.\n");
    serial_port device_port;
    serial_port_init(&device_port);

#if INDIVIDUAL_SCRIPT == 1
    (void)argc;
    (void)argv;

    BAUDRATE = 460800;

    char* PORT_NAME = find_device("FTDI");

    MICROSTRAIN_LOG_INFO(" Connecting to the device on port :%s ", PORT_NAME);

    MICROSTRAIN_LOG_INFO("Ingrese el baudrate que desea configurar: ");

    BAUDRATE = 0;

    if (scanf("%u", &BAUDRATE) != 1) {
        MICROSTRAIN_LOG_INFO("Invalid input\n");
        return 1;
    }

    MICROSTRAIN_LOG_INFO(" Baudrate set to : %u .\n", BAUDRATE);
#else
    BAUDRATE = 460800;

    if (argc < 2) {
        fprintf(stderr, "Usage: %s <port>\n", argv[0]);
        return 1;
    }

    const char* PORT_NAME = argv[1];  // Usar argumento del padre
    MICROSTRAIN_LOG_INFO("Connecting to the device on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);
#endif

    // Open the connection to the device
    if (!serial_port_open(&device_port, PORT_NAME, BAUDRATE))
    {
        terminate(&device_port, "Could not open the connection!\n", false);
    }

    mip_interface device;
    initialize_device(&device, &device_port, BAUDRATE);

    sync_device_time(&device);


#if SC_CONFIG == 0
    // Set accel a 8g
    mip_3dm_write_sensor_range(&device, MIP_SENSOR_RANGE_TYPE_ACCEL, 3);
    mip_3dm_save_sensor_range(&device, MIP_SENSOR_RANGE_TYPE_ACCEL);

    // // Set gyro a 500 dps
    // mip_3dm_write_sensor_range(&device, MIP_SENSOR_RANGE_TYPE_GYRO,3);
    // mip_3dm_save_sensor_range(&device, MIP_SENSOR_RANGE_TYPE_GYRO);

    // Capture gyro bias
    capture_gyro_bias(&device);
    //mip_3dm_save_gyro_bias(&device); 

    // Configure the message format for sensor data
    configure_sensor_message_format(&device);

    // Configure the message format for filter data
    configure_filter_message_format(&device);
    
    mip_3dm_save_device_settings(&device);  
#endif
    
    // Initialize the navigation filter
    initialize_filter(&device);

    mip_cmd_result cmd_result;
    
    CsvFiles files = createCsvFiles();

    sensor_buffer_t sensor_buffer = {0};
    sensor_buffer.output_files = files.csv1;
    filter_buffer_t filter_buffer = {0};
    filter_buffer.output_files = files.csv2;

#if USE_THREADS
    if (pthread_mutex_init(&sensor_buffer.buffer_mutex, NULL) != 0)
    {
        MICROSTRAIN_LOG_ERROR("Failed to initialize sensor buffer mutex!\n");
        closeCsvFiles(sensor_buffer.output_files);
        terminate(&device_port, "Could not initialize sensor buffer mutex!\n", false);
    }
    if (pthread_mutex_init(&filter_buffer.buffer_mutex, NULL) != 0)
    {
        MICROSTRAIN_LOG_ERROR("Failed to initialize filter buffer mutex!\n");
        closeCsvFiles(filter_buffer.output_files);
        terminate(&device_port, "Could not initialize filter buffer mutex!\n", false);
    }
#endif // USE_THREADS

    MICROSTRAIN_LOG_INFO("Registering packet callbacks.\n");

    // SDK data processing flow triggered by mip_interface_update():
    //   1. SDK receives raw bytes from device
    //   2. SDK parses bytes into MIP packets
    //   3. SDK iterates over each field in the packet
    //   4. SDK invokes any registered field callbacks whose descriptors match
    //   5. SDK invokes packet callbacks with afterFields=true AFTER all field callbacks complete
    //
    // afterFields=true guarantees all fields in the packet are available when our callback
    // fires, which is required for our GPS (tow, week_number) cross-packet synchronization.
    // afterFields=false would fire before field extraction, leaving data unavailable.

    mip_dispatch_handler sensor_packet_handler;
    mip_interface_register_packet_callback(
        &device,
        &sensor_packet_handler,
        MIP_SENSOR_DATA_DESC_SET,
        false,  // afterFields=true: fires after all fields in this packet have been processed
        &sensor_packet_callback,
        &sensor_buffer
    );

    mip_dispatch_handler filter_packet_handler;
    mip_interface_register_packet_callback(
        &device,
        &filter_packet_handler,
        MIP_FILTER_DATA_DESC_SET,
        false,  // afterFields=true: fires after all fields in this packet have been processed
        &filter_packet_callback,
        &filter_buffer
    );

#if USE_THREADS
    MICROSTRAIN_LOG_INFO("Initializing the device update function for threading.\n");
    // Note: This allows the update function to be split into command and data updates across multiple threads
    mip_interface_set_update_function(&device, &update_device);

    thread_data_t thread_data = {.device = &device, .running = true};

    MICROSTRAIN_LOG_INFO("Creating the data collection thread.\n");
    pthread_t data_thread;
    pthread_create(&data_thread, NULL, data_collection_thread, (void*)&thread_data);
#endif // USE_THREADS


    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    MICROSTRAIN_LOG_INFO("Resuming the device.\n");
    cmd_result = mip_base_resume(&device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(&device, cmd_result, "Could not resume the device!\n");
    }

    
    const mip_timestamp loop_start_time = get_current_timestamp();
    
    while (1)
    {
        // Exit after the configured run time
        if (get_current_timestamp() - loop_start_time >= RUN_TIME_SECONDS * 1000) break;

        // Stress-test: send 100 pings per iteration while data collection runs in its own thread.
        // Each mip_base_ping() call invokes the device update function, which in threaded mode
        // (update_device) sleeps 5ms and returns true without calling mip_interface_default_update,
        // keeping command traffic isolated from the data collection thread.
        MICROSTRAIN_LOG_WARN("Running device stress test!\n");
        for (uint8_t counter = 0; counter < 100; ++counter)
        {
            mip_base_ping(&device);
        }
    }

    // Flush any remaining samples before exit
    flush_sensor_buffer(&sensor_buffer);
    flush_filter_buffer(&filter_buffer);
    closeCsvFiles(sensor_buffer.output_files);
    closeCsvFiles(filter_buffer.output_files);

#if USE_THREADS
    // Signal the data collection thread to stop
    thread_data.running = false;

    // Join the thread back before exiting the program
    MICROSTRAIN_LOG_INFO("Waiting for the thread to join.\n");
    void* thread_return_code = NULL; // Return code from the thread function (Unused)
    if (pthread_join(data_thread, &thread_return_code) != 0)
    {
        pthread_mutex_destroy(&lock);
        pthread_mutex_destroy(&filter_buffer.buffer_mutex);
        pthread_mutex_destroy(&sensor_buffer.buffer_mutex);
        // Do NOT free thread_return_code here: if join failed, the pointer was not written
        terminate(&device_port, "Failed to join the thread!\n", false);
    }

    pthread_mutex_destroy(&sensor_buffer.buffer_mutex);
    pthread_mutex_destroy(&filter_buffer.buffer_mutex);
    pthread_mutex_destroy(&lock);
    // thread_return_code is always NULL (data_collection_thread returns NULL), free is valid but a no-op
    (void)thread_return_code;
#else
    pthread_mutex_destroy(&sample_buffer.buffer_mutex);
#endif // USE_THREADS
    
    terminate(&device_port, "INS logging completed successfully.\n", true);

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup _7_series_ahrs_example_c
/// @{
///

////////////////////////////////////////////////////////////////////////////////
/// @brief Custom logging callback for MIP SDK message formatting and output
///
/// @details Processes and formats log messages from the MIP SDK based on
///          severity level. Routes messages to appropriate output streams -
///          errors and fatal messages go to stderr while other levels go to
///          stdout. Each message is prefixed with its severity level name.
///
/// @param _user Pointer to user data (unused in this implementation)
/// @param _level Log message severity level from microstrain_log_level enum
/// @param _format Printf-style format string for the message
/// @param _args Variable argument list containing message parameters
///
///
static void log_callback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args)
{
#if USE_THREADS
    pthread_mutex_t* lock = (pthread_mutex_t*)_user;
    if (lock)
        pthread_mutex_lock(lock);
#else
    (void)_user;  // Unused parameter
#endif // USE_THREADS

    switch (_level)
    {
        case MICROSTRAIN_LOG_LEVEL_FATAL:
        case MICROSTRAIN_LOG_LEVEL_ERROR:
        {
            fprintf(stderr, "%s: ", microstrain_logging_level_name(_level));
            vfprintf(stderr, _format, _args);
            fflush(stderr);
            break;
        }
        case MICROSTRAIN_LOG_LEVEL_WARN:
        case MICROSTRAIN_LOG_LEVEL_INFO:
        case MICROSTRAIN_LOG_LEVEL_DEBUG:
        case MICROSTRAIN_LOG_LEVEL_TRACE:
        {
            fprintf(stdout, "%s: ", microstrain_logging_level_name(_level));
            vfprintf(stdout, _format, _args);
            fflush(stdout);
            break;
        }
        case MICROSTRAIN_LOG_LEVEL_OFF:
        default:
        {
            break;
        }
    }

#if USE_THREADS
    if (lock)
        pthread_mutex_unlock(lock);
#endif // USE_THREADS
}

#if SC_CONFIG == 0
////////////////////////////////////////////////////////////////////////////////
/// @brief Captures and configures device gyro bias
///
/// @param _device Pointer to the initialized MIP device interface
///
///
static void capture_gyro_bias(mip_interface* _device)
{
    // Get the command queue so we can increase the reply timeout during the capture duration,
    // then reset it afterward
    mip_cmd_queue*    cmd_queue        = mip_interface_cmd_queue(_device);
    const mip_timeout previous_timeout = mip_cmd_queue_base_reply_timeout(cmd_queue);
    MICROSTRAIN_LOG_INFO("Initial command reply timeout is %dms.\n", previous_timeout);

    // Note: The default is 15 s (15,000 ms)
    const uint16_t capture_duration            = CAPTURE_DURATION_TIME;
    const uint16_t increased_cmd_reply_timeout = capture_duration + 1000;

    MICROSTRAIN_LOG_INFO("Increasing command reply timeout to %dms for capture gyro bias.\n", increased_cmd_reply_timeout);
    mip_cmd_queue_set_base_reply_timeout(cmd_queue, increased_cmd_reply_timeout);

    mip_vector3f gyro_bias = {
        0.0f, // X
        0.0f, // Y
        0.0f  // Z
    };

    // Note: When capturing gyro bias, the device needs to remain still on a flat surface
    MICROSTRAIN_LOG_WARN("About to capture gyro bias for %.2g seconds!\n", (float)capture_duration / 1000.0f);
    MICROSTRAIN_LOG_WARN("Please do not move the device during this time!\n");
    // MICROSTRAIN_LOG_WARN("Press 'Enter' when ready...");

    // // Wait for anything to be entered
    // const int confirm_capture = getc(stdin);
    // (void)confirm_capture; // Unused

    MICROSTRAIN_LOG_WARN("Capturing gyro bias...\n");
    const mip_cmd_result cmd_result = mip_3dm_capture_gyro_bias(
        _device,
        capture_duration, // Capture duration (ms)
        gyro_bias         // Gyro bias out (result of the capture)
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Failed to capture gyro bias!\n");
    }

    MICROSTRAIN_LOG_INFO("Capture gyro bias completed with result: [%f, %f, %f]\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    MICROSTRAIN_LOG_INFO("Reverting command reply timeout to %dms.\n", previous_timeout);
    mip_cmd_queue_set_base_reply_timeout(cmd_queue, previous_timeout);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Configures message format for filter data streaming
///
/// @details Sets up filter data output by:
///          1. Querying device base rate
///          2. Validating desired sample rate against base rate
///          3. Calculating proper decimation
///          4. Configuring message format with:
///             - GPS timestamp
///             - Filter status
///             - Euler angles
///
/// @param _device Pointer to the initialized MIP device interface
///
///

static void configure_sensor_message_format(mip_interface* _device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for sensor data.\n");
    uint16_t       sensor_base_rate;
    mip_cmd_result cmd_result = mip_3dm_get_base_rate(
        _device,
        MIP_SENSOR_DATA_DESC_SET, // Data descriptor set
        &sensor_base_rate         // Base rate out
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not get the base rate for filter data!\n");
    }

    // Supported sample rates can be any value from 1 up to the base rate
    // Note: Decimation can be anything from 1 to 65,565 (uint16_t::max)
    if (SAMPLE_RATE_HZ == 0 || SAMPLE_RATE_HZ > sensor_base_rate)
    {
        exit_from_command(
            _device,
            MIP_NACK_INVALID_PARAM,
            "Invalid sample rate of %dHz! Supported rates are [1, %d].\n",
            SAMPLE_RATE_HZ,
            sensor_base_rate
        );
    }

    // Calculate the decimation (stream rate) for the device based on its base rate
    // IMPORTANT: Decimation must be chosen to match desired rate as closely as possible
    // Actual rate will be: base_rate / decimation
    // For exact rate: decimation must divide base_rate evenly
    
    // Use user-specified decimation if set, otherwise calculate from SAMPLE_RATE_HZ
    uint16_t sensor_decimation;
    if (SENSOR_DECIMATION > 0)
    {
        sensor_decimation = SENSOR_DECIMATION;
        MICROSTRAIN_LOG_WARN("Using user-specified SENSOR_DECIMATION = %d\n", SENSOR_DECIMATION);
    }
    else
    {
        sensor_decimation = sensor_base_rate / SAMPLE_RATE_HZ;
    }
    
    const uint16_t actual_sensor_rate = sensor_base_rate / sensor_decimation;  // Calculate actual rate

    // GPS shared timestamp (MIP_DATA_DESC_SHARED_GPS_TIME, 0xFF/0xD3) uses decimation=1,
    // meaning it runs at base_rate (e.g. 200Hz) while sensor data runs at SAMPLE_RATE_HZ
    // (e.g. 100Hz). This 1:2 ratio means 2 GPS timestamps arrive per sensor sample, giving
    // higher time resolution for synchronization and interpolation. Both sensor and filter
    // descriptor sets use the same (tow, week_number) pair from this shared field as the
    // cross-packet correlation key.
    const uint16_t gps_time_decimation = sensor_decimation/2; // GPS timestamp at half the sensor rate for better synchronization resolution

    MICROSTRAIN_LOG_INFO(
        "SENSOR: Base=%dHz | GPS_ts=%dHz (dec=1) | Data=%dHz (dec=%d) | Requested=%dHz%s\n",
        sensor_base_rate,
        sensor_base_rate,          // GPS always at base_rate
        actual_sensor_rate,
        sensor_decimation,
        SAMPLE_RATE_HZ,
        (actual_sensor_rate != SAMPLE_RATE_HZ) ? " [MISMATCH!]" : " [OK]"
    );

    if (actual_sensor_rate != SAMPLE_RATE_HZ)
    {
        MICROSTRAIN_LOG_WARN(
            "SENSOR RATE MISMATCH! Actual=%dHz vs Requested=%dHz\n",
            actual_sensor_rate,
            SAMPLE_RATE_HZ
        );
        MICROSTRAIN_LOG_WARN(
            "  Fix: Set SENSOR_DECIMATION=%d to get exact %dHz\n",
            sensor_base_rate / actual_sensor_rate,
            actual_sensor_rate
        );
        MICROSTRAIN_LOG_WARN(
        "  OR set SAMPLE_RATE_HZ=%d to match decimation %d\n",
            actual_sensor_rate,
            sensor_decimation
        );
    }
    else
    {
        MICROSTRAIN_LOG_INFO("  ✓ Sensor rate matches requested rate\n");
    }


    // Descriptor rate is a pair of data descriptor set and decimation
    const mip_descriptor_rate sensor_descriptors[5] = {
        {MIP_DATA_DESC_SHARED_GPS_TIME,         gps_time_decimation},
        {MIP_DATA_DESC_SENSOR_ACCEL_SCALED,     sensor_decimation},
        {MIP_DATA_DESC_SENSOR_DELTA_VELOCITY,   sensor_decimation},
        {MIP_DATA_DESC_SENSOR_COMP_QUATERNION,  sensor_decimation},
        {MIP_DATA_DESC_SHARED_REFERENCE_TIME,   sensor_decimation}
        // {MIP_DATA_DESC_SENSOR_GYRO_SCALED,      sensor_decimation},
        // {MIP_DATA_DESC_SENSOR_MAG_SCALED,       sensor_decimation},
        // {MIP_DATA_DESC_SENSOR_DELTA_THETA,      sensor_decimation},
    };

    MICROSTRAIN_LOG_INFO("Configuring message format for sensor data.\n");
    cmd_result = mip_3dm_write_message_format(
        _device,
        MIP_SENSOR_DATA_DESC_SET,                                   // Data descriptor set
        sizeof(sensor_descriptors) / sizeof(sensor_descriptors[0]), // Number of descriptors to include
        sensor_descriptors                                          // Descriptor array
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not configure message format for sensor data!\n");
    }
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Configures message format for filter data streaming
///
/// @details Sets up filter data output by:
///          1. Querying device base rate
///          2. Validating desired sample rate against base rate
///          3. Calculating proper decimation
///          4. Configuring message format with:
///             - GPS timestamp
///             - Filter status
///             - Euler angles
///
/// @param _device Pointer to the initialized MIP device interface
///
///
static void configure_filter_message_format(mip_interface* _device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for filter data.\n");
    uint16_t       filter_base_rate;
    mip_cmd_result cmd_result = mip_3dm_get_base_rate(
        _device,
        MIP_FILTER_DATA_DESC_SET, // Data descriptor set
        &filter_base_rate         // Base rate out
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not get the base rate for filter data!\n");
    }

    // Supported sample rates can be any value from 1 up to the base rate
    // Note: Decimation can be anything from 1 to 65,565 (uint16_t::max)
    if (SAMPLE_RATE_HZ == 0 || SAMPLE_RATE_HZ > filter_base_rate)
    {
        exit_from_command(
            _device,
            MIP_NACK_INVALID_PARAM,
            "Invalid sample rate of %dHz! Supported rates are [1, %d].\n",
            SAMPLE_RATE_HZ,
            filter_base_rate
        );
    }

    // Calculate the decimation (stream rate) for the device based on its base rate
    // IMPORTANT: Decimation must be chosen to match desired rate as closely as possible
    // Actual rate will be: base_rate / decimation
    // For exact rate: decimation must divide base_rate evenly
    
    // Use user-specified decimation if set, otherwise calculate from SAMPLE_RATE_HZ
    uint16_t filter_decimation;
    if (FILTER_DECIMATION > 0)
    {
        filter_decimation = FILTER_DECIMATION;
        MICROSTRAIN_LOG_WARN("Using user-specified FILTER_DECIMATION = %d\n", FILTER_DECIMATION);
    }
    else
    {
        filter_decimation = filter_base_rate / SAMPLE_RATE_HZ;
    }
    
    const uint16_t actual_filter_rate = filter_base_rate / filter_decimation;  // Calculate actual rate
    // GPS shared timestamp (MIP_DATA_DESC_SHARED_GPS_TIME, 0xFF/0xD3) uses decimation=1,
    // meaning it runs at base_rate (e.g. 200Hz) while filter data runs at SAMPLE_RATE_HZ
    // (e.g. 100Hz). This 1:2 ratio means 2 GPS timestamps arrive per filter sample, giving
    // higher time resolution for synchronization and interpolation. Both sensor and filter
    // descriptor sets use the same (tow, week_number) pair from this shared field as the
    // cross-packet correlation key.
    const uint16_t filter_gps_time_decimation = filter_decimation/2; // GPS timestamp at half the filter rate for better synchronization resolution  

    MICROSTRAIN_LOG_INFO(
        "FILTER: Base=%dHz | GPS_ts=%dHz (dec=1) | Data=%dHz (dec=%d) | Requested=%dHz%s\n",
        filter_base_rate,
        filter_base_rate,          // GPS always at base_rate
        actual_filter_rate,
        filter_decimation,
        SAMPLE_RATE_HZ,
        (actual_filter_rate != SAMPLE_RATE_HZ) ? " [MISMATCH!]" : " [OK]"
    );

    if (actual_filter_rate != SAMPLE_RATE_HZ)
    {
        MICROSTRAIN_LOG_WARN(
            "FILTER RATE MISMATCH! Actual=%dHz vs Requested=%dHz\n",
            actual_filter_rate,
            SAMPLE_RATE_HZ
        );
        MICROSTRAIN_LOG_WARN(
            "  Fix: Set FILTER_DECIMATION=%d to get exact %dHz\n",
            filter_base_rate / actual_filter_rate,
            actual_filter_rate
        );
        MICROSTRAIN_LOG_WARN(
            "  OR set SAMPLE_RATE_HZ=%d to match decimation %d\n",
            actual_filter_rate,
            filter_decimation
        );
    }
    else
    {
        MICROSTRAIN_LOG_INFO("  ✓ Filter rate matches requested rate\n");
    }

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip_descriptor_rate filter_descriptors[6] = {
        {MIP_DATA_DESC_SHARED_GPS_TIME,                     filter_gps_time_decimation},
        {MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION,          filter_decimation},
        {MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE,     filter_decimation},
        {MIP_DATA_DESC_FILTER_ATT_QUATERNION,               filter_decimation},
        {MIP_DATA_DESC_FILTER_FILTER_STATUS,                filter_decimation},
        {MIP_DATA_DESC_FILTER_GRAVITY_VECTOR,               filter_decimation},
        // {MIP_DATA_DESC_FILTER_GYRO_BIAS,                    filter_decimation},
        // {MIP_DATA_DESC_SHARED_EVENT_SOURCE,                 filter_decimation},
    };


    MICROSTRAIN_LOG_INFO("Configuring message format for filter data.\n");
    cmd_result = mip_3dm_write_message_format(
        _device,
        MIP_FILTER_DATA_DESC_SET,                                   // Data descriptor set
        sizeof(filter_descriptors) / sizeof(filter_descriptors[0]), // Number of descriptors to include
        filter_descriptors                                          // Descriptor array
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not configure message format for filter data!\n");
    }
}

#endif



////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes and resets the navigation filter
///
/// @details Configures the filter by:
///          1. Enabling magnetometer aiding measurements
///          2. Resetting the filter to apply new settings
///
/// @param _device Pointer to the initialized MIP device interface
///
///
static void initialize_filter(mip_interface* _device)
{
    // Configure Filter Aiding Measurements
    
    mip_cmd_result cmd_result;

#if SC_CONFIG == 0

#if (MAGNETOMETER_FEATURE == 1)
    MICROSTRAIN_LOG_INFO("Enabling the aiding measurement source for magnetometer.\n");
    cmd_result = mip_filter_write_aiding_measurement_enable(
        _device,
        MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_MAGNETOMETER, // Aiding Source type
        true                                                                // Enabled
    );
    
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not enable the aiding measurement source for magnetometer!\n");
    }

#else
    MICROSTRAIN_LOG_INFO("Disabling the aiding measurement source for magnetometer.\n");
    cmd_result = mip_filter_write_aiding_measurement_enable(
        _device,
        MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_MAGNETOMETER, // Aiding Source type
        false                                                                // Disabled
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not disable the aiding measurement source for magnetometer!\n");
    }
#endif  


    MICROSTRAIN_LOG_INFO("Disabling the aiding measurement source for external heading.\n");
    cmd_result = mip_filter_write_aiding_measurement_enable(
        _device,
         MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_EXTERNAL_HEADING, // Aiding Source type
         false                                                                   // Disabled
    );


    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not disable the aiding measurement source for external heading!\n");
    }

#endif
    // Reset the filter
    // Note: This is good to do after filter setup is complete
    MICROSTRAIN_LOG_INFO("Attempting to reset the navigation filter.\n");
    cmd_result = mip_filter_reset(_device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not reset the navigation filter!\n");
    }

}


////////////////////////////////////////////////////////////////////////////////
/// @brief Gets the current system timestamp in milliseconds
///
/// @details Provides basic timestamping using system time:
///          - Returns milliseconds since Unix epoch
///          - Uses timespec_get() with UTC time base
///          - Returns 0 if time cannot be obtained
///
/// @note Update this function to use a different time source if needed for
///       your specific application requirements
///
/// @return Current system time in milliseconds since epoch
///
///
static mip_timestamp get_current_timestamp()
{
    struct timespec ts;

    // Get system UTC time since epoch
    if (timespec_get(&ts, TIME_UTC) != TIME_UTC)
    {
        return 0;
    }

    // Get the time in milliseconds
    return (mip_timestamp)ts.tv_sec * 1000 + (mip_timestamp)ts.tv_nsec / 1000000;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles sending packets to the device
///
/// @details Implements the MIP device interface send callback:
///          - Extracts serial port from device user pointer
///          - Validates connection state
///          - Writes data buffer to serial port
///
/// @param _device MIP device interface containing the connection
/// @param _data Buffer containing packet data to send
/// @param _length Number of bytes to send
///
/// @return True if send was successful, false otherwise
///
///
static bool mip_interface_user_send_to_device(mip_interface* _device, const uint8_t* _data, size_t _length)
{
    // Extract the serial port pointer that was used in the callback initialization
    serial_port* device_port = (serial_port*)mip_interface_user_pointer(_device);

    if (device_port == NULL)
    {
        MICROSTRAIN_LOG_ERROR("serial_port pointer not set in mip_interface_init().\n");
        return false;
    }

    // Get the bytes written to the device
    size_t bytes_written;

    // Send the packet to the device
    return serial_port_write(device_port, _data, _length, &bytes_written);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles receiving packets from the device
///
/// @details Implements the MIP device interface receive callback:
///          - Extracts serial port from device user pointer
///          - Validates connection state
///          - Reads available data into provided buffer
///          - Timestamps the received data
///
/// @param _device MIP device interface containing the connection
/// @param _buffer Buffer to store received data
/// @param _max_length Maximum number of bytes to read
/// @param _wait_time How long to wait for data in milliseconds
/// @param _from_cmd Whether this read is from a command response (unused)
/// @param _length_out Number of bytes actually read
/// @param _timestamp_out Timestamp when data was received
///
/// @return True if receive was successful, false otherwise
///
///
static bool mip_interface_user_recv_from_device(
    mip_interface* _device, uint8_t* _buffer, size_t _max_length, mip_timeout _wait_time, bool _from_cmd,
    size_t* _length_out, mip_timestamp* _timestamp_out
)
{
    // Unused parameter
    (void)_from_cmd;

    // Extract the serial port pointer that was used in the callback initialization
    serial_port* device_port = (serial_port*)mip_interface_user_pointer(_device);

    if (device_port == NULL)
    {
        MICROSTRAIN_LOG_ERROR("serial_port pointer not set in mip_interface_init().\n");
        return false;
    }

    // Get the time that the packet was received (system epoch UTC time in milliseconds)
    *_timestamp_out = get_current_timestamp();

    // Read the packet from the device
    return serial_port_read(device_port, _buffer, _max_length, (int)_wait_time, _length_out);
}



////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes and configures a MIP device interface
///
/// @details Performs a complete device initialization sequence:
///          1. Sets up a MIP device interface with specified timeouts and
///             callbacks
///          2. Verifies device communication with a ping command
///          3. Sets the device to idle mode to ensure reliable configuration
///          4. Queries and displays detailed device information
///          5. Loads default device settings for a known state
///
/// @param _device Pointer to a MIP device interface to initialize
/// @param _device_port Pointer to an initialized serial port for device
///                     communication
/// @param _baudrate Serial communication baudrate for the device
///
///
static void initialize_device(mip_interface* _device, serial_port* _device_port, const uint32_t _baudrate)
{
    MICROSTRAIN_LOG_INFO("Initializing the device interface.\n");
    


    mip_interface_init(
        _device,
        mip_timeout_from_baudrate(_baudrate), // Set the base timeout for commands (milliseconds)
        2000,                                 // Set the base timeout for command replies (milliseconds)
        &mip_interface_user_send_to_device,   // User-defined send packet callback
        &mip_interface_user_recv_from_device, // User-defined receive packet callback
        &mip_interface_default_update,        // Default update callback
        (void*)_device_port                   // Cast the device port for use in the callbacks
    );
    

    // Ping the device
    // Note: This is a good first step to make sure the device is present
    MICROSTRAIN_LOG_INFO("Pinging the device.\n");
    mip_cmd_result cmd_result = mip_base_ping(_device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not ping the device!\n");
    }

    // Set the device to Idle
    // Note: This is good to do during setup as high data traffic can cause commands to fail
    MICROSTRAIN_LOG_INFO("Setting the device to idle.\n");
    cmd_result = mip_base_set_idle(_device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not set the device to idle!\n");
    }

    // Print device info to make sure the correct device is being used
    MICROSTRAIN_LOG_INFO("Getting the device information.\n");
    mip_base_device_info device_info;
    cmd_result = mip_base_get_device_info(_device, &device_info);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not get the device information!\n");
    }

    // Extract the major minor and patch values
    const uint16_t major = device_info.firmware_version / 1000;
    const uint16_t minor = device_info.firmware_version / 100 % 10;
    const uint16_t patch = device_info.firmware_version % 100;

    // Firmware version format is x.x.xx
    char firmwareVersion[16] = {0};
    snprintf(firmwareVersion, sizeof(firmwareVersion) / sizeof(firmwareVersion[0]), "%d.%d.%02d", major, minor, patch);

    MICROSTRAIN_LOG_INFO("-------- Device Information --------\n");
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Name", device_info.model_name);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Model Number", device_info.model_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Serial Number", device_info.serial_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Lot Number", device_info.lot_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Options", device_info.device_options);
    MICROSTRAIN_LOG_INFO("%-16s | %16s\n", "Firmware Version", firmwareVersion);
    MICROSTRAIN_LOG_INFO("------------------------------------\n");

    // Load the default settings on the device
    // Note: This guarantees the device is in a known state
    // MICROSTRAIN_LOG_INFO("Loading device default settings.\n");
    // cmd_result = mip_3dm_default_device_settings(_device);

#if DEFAULT_CONFIG == 1 && SC_CONFIG == 0

    MICROSTRAIN_LOG_INFO("Loading device default settings.\n");
    cmd_result = mip_3dm_default_device_settings(_device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        // Note: Default settings will reset the baudrate to 115200 and may cause connection issues
        if (cmd_result == MIP_STATUS_TIMEDOUT && BAUDRATE != 115200)
        {
            MICROSTRAIN_LOG_WARN("On a native serial connections the baudrate needs to be 115200 for this example to run.\n");
        }

        exit_from_command(_device, cmd_result, "Could not load device default settings!\n");
    }

    mip_3dm_write_uart_baudrate(_device, 460800);
        
    mip_3dm_save_uart_baudrate(_device);

    sleep(1); // wait to save the baudrate

    // IMPORTANTE: Reconfigurar el puerto serial al nuevo baudrate  
    serial_port_set_baudrate(_device_port, 460800);  
    
    // Reconfigurar el puerto serial físico  
    serial_port_set_baudrate(_device_port, 460800);  
    
    // Actualizar solo el timeout del parser  
    mip_parser_set_timeout(  
        mip_interface_parser(_device),  
        mip_timeout_from_baudrate(460800)  
    );  
    
    // Actualizar solo el timeout de la cola de comandos  
    mip_cmd_queue_set_base_reply_timeout(  
        mip_interface_cmd_queue(_device),  
        2000  
    );
    
#elif DEFAULT_CONFIG == 0 && SC_CONFIG == 0

    MICROSTRAIN_LOG_INFO("Loading device settings.\n");
    cmd_result = mip_3dm_load_device_settings(_device);
    
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        // Note: Default settings will reset the baudrate to 115200 and may cause connection issues
    //        if (cmd_result == MIP_STATUS_TIMEDOUT && BAUDRATE != 115200)
        if (cmd_result == MIP_STATUS_TIMEDOUT )
        {
            MICROSTRAIN_LOG_WARN("On a native serial connections the baudrate needs to be 115200 for this example to run.\n");
        }
    
        exit_from_command(_device, cmd_result, "Could not load device default settings!\n");
    }


    mip_3dm_write_uart_baudrate(_device, 460800);
        
    mip_3dm_save_uart_baudrate(_device);

    sleep(1); // wait to save the baudrate
    // Reset the filter
    // Note: This is good to do after filter setup is complete
    MICROSTRAIN_LOG_INFO("Attempting to reset the navigation filter.\n");

    cmd_result = mip_filter_reset(_device);
    
    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not reset the navigation filter!\n");
    }
#endif

}



////////////////////////////////////////////////////////////////////////////////
/// @brief Handles graceful program termination and cleanup
///
/// @details Handles graceful shutdown when errors occur:
///          - Outputs provided error message
///          - Closes device connection if open
///          - Exits with appropriate status code
///
/// @param _device_port Serial port connection to close
/// @param _message Error message to display
/// @param _successful Whether termination is due to success or failure
///
///
static void terminate(serial_port* _device_port, const char* _message, const bool _successful)
{
    if (_message != NULL && strlen(_message) != 0)
    {
        if (_successful)
        {
            MICROSTRAIN_LOG_INFO("%s", _message);
        }
        else
        {
            MICROSTRAIN_LOG_ERROR("%s", _message);
        }
    }

    if (_device_port == NULL)
    {
        // Initialize the device interface with a serial port connection
        MICROSTRAIN_LOG_ERROR("Connection not set for the device interface. Cannot close the connection.\n");
    }
    else
    {
        if (serial_port_is_open(_device_port))
        {
            MICROSTRAIN_LOG_INFO("Closing the connection.\n");

            if (!serial_port_close(_device_port))
            {
                MICROSTRAIN_LOG_ERROR("Failed to close the connection!\n");
            }
        }
    }

    // MICROSTRAIN_LOG_INFO("Press 'Enter' to exit the program.\n");
    MICROSTRAIN_LOG_INFO("Exit the program.\n");

    // Make sure the console remains open
    // const int confirm_exit = getc(stdin);
    // (void)confirm_exit; // Unused

    if (!_successful)
    {
        exit(1);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles graceful program termination and command failure cleanup
///
/// @details Handles command failure scenarios:
///          - Formats and displays an error message with command result
///          - Closes device connection
///          - Exits with failure status
///
/// @param _device MIP device interface for the command that failed
/// @param _cmd_result Result code from a failed command
/// @param _format Printf-style format string for error message
/// @param ... Variable arguments for format string
///
///
static void exit_from_command(const mip_interface* _device, const mip_cmd_result _cmd_result, const char* _format, ...)
{
    if (_format != NULL && strlen(_format) != 0)
    {
        va_list args;
        va_start(args, _format);
        MICROSTRAIN_LOG_ERROR_V(_format, args);
        va_end(args);
    }

    MICROSTRAIN_LOG_ERROR("Command Result: (%d) %s.\n", _cmd_result, mip_cmd_result_to_string(_cmd_result));

    if (_device == NULL)
    {
        terminate(NULL, "", false);
    }
    else
    {
        // Get the connection pointer that was set during device initialization
        serial_port* device_port = (serial_port*)mip_interface_user_pointer(_device);

        terminate(device_port, "", false);
    }
}

uint64_t get_unix_time_ns() {
    struct timespec ts;

    // Obtiene el tiempo actual en segundos y nanosegundos desde la época UNIX (1 de enero de 1970)
    clock_gettime(CLOCK_REALTIME, &ts);

    // Devuelve el tiempo en nanosegundos (segundos * 1e9 + nanosegundos)
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}


void generateFileName(char *baseName, int counter, char *fileName) {
    sprintf(fileName, "%s_%d.csv", baseName, counter);
}

CsvFiles createCsvFiles() {
    CsvFiles files;
    char fileName[MAX_FILENAME_LENGTH];

    // Generar nombres para los tres archivos
    generateFileName("SensorData_", fileSetCounter, fileName);
    files.csv1 = fopen(fileName, "w");
    if (files.csv1 == NULL) {
        perror("Error to open file 1");
        exit(1);
    }

            
    //CSV Header 
    fprintf(files.csv1, 
            "Time,GPS_ts,GPS_ts:valid,"
            "scaledAccelX,scaledAccelY,scaledAccelZ,"
            "deltaVelX,deltaVelY,deltaVelZ,"
            "orientQuaternion[0],orientQuaternion[1],orientQuaternion[2],orientQuaternion[3],flag\n");

    generateFileName("FilterData_", fileSetCounter, fileName);
    files.csv2 = fopen(fileName, "w");
    if (files.csv2 == NULL) {
        perror("Error to open file 2");
        exit(1);
    }
            
    //CSV Header 
    fprintf(files.csv2, 
            "Time,fGPS_ts,fGPS_ts:valid,"
            "estLinearAccelX,estLinearAccelY,estLinearAccelZ,estLinearAccel:valid,"
            "estAngularRateX,estAngularRateY,estAngularRateZ,estAngularRate:valid,"
            "estGravityX,estGravityY,estGravityZ,estGravity:valid,"
            "estOrientQuaternion[0], estOrientQuaternion[1],estOrientQuaternion[2],estOrientQuaternion[3],estOrientQuaternion:valid,flag\n");

    return files;
}

void closeCsvFiles(FILE* files) {
    if (files) fclose(files);

}



char* find_device(const char* keyword) {
    static char result[MAX_PATH];
    const char* base = "/dev/serial/by-id/";

    DIR* dir = opendir(base);
    if (!dir) {
        perror("opendir");
        return NULL;
    }

    struct dirent* entry;

    while ((entry = readdir(dir)) != NULL) {
        if (strstr(entry->d_name, keyword) != NULL) {
            snprintf(result, MAX_PATH, "%s%s", base, entry->d_name);
            closedir(dir);
            return result;
        }
    }

    closedir(dir);
    return NULL;
}


static void flush_sensor_buffer(sensor_buffer_t* _buffer)
{

    bool flagS = false;
    uint64_t time=0;

    if (_buffer == NULL || _buffer->count == 0)
    {
        return;
    }

    MICROSTRAIN_LOG_INFO("Flushing %zu samples:\n", _buffer->count);

    for (size_t i = 0; i < _buffer->count; ++i)
    {
        const sensorData_t* idata = &_buffer->sample[i];
        time = mip_gps_to_unix_ms(idata->gpsTs);
    //CSV Header 
        fprintf(_buffer->output_files,
            "%" PRIu64 "," 
            "%10.3f,%u,"                              
            "%9.6f,%9.6f,%9.6f,"
            "%9.6f,%9.6f,%9.6f,"
            "%9.6f,%9.6f,%9.6f,%9.6f,%u,\n",                                                        
            time,
            idata->gpsTs.tow, idata->gpsTs.valid_flags,
            idata->accel.scaled_accel[0], idata->accel.scaled_accel[1], idata->accel.scaled_accel[2], 
            idata->deltaV.delta_velocity[0], idata->deltaV.delta_velocity[1], idata->deltaV.delta_velocity[2],
            idata->q.q[0], idata->q.q[1], idata->q.q[2], idata->q.q[3], flagS 
        );
        
    }

    if (_buffer->output_files != NULL)
    {
        fflush(_buffer->output_files);
    }


    _buffer->last_flush_time = _buffer->sample[_buffer->count - 1].timestamp;
    _buffer->count           = 0;
}


static void flush_filter_buffer(filter_buffer_t* _buffer)
{
    bool flagS = false;
    uint64_t time=0;
    if (_buffer == NULL || _buffer->count == 0)
    {
        return;
    }
    //CSV Header 


    MICROSTRAIN_LOG_INFO("Flushing %zu samples:\n", _buffer->count);

    for (size_t i = 0; i < _buffer->count; ++i)
    {
        const filterData_t* idata = &_buffer->sample[i];

        time = mip_gps_to_unix_ms(idata->gpsTs);

        fprintf(_buffer->output_files,
            "%" PRIu64 ","
            "%10.3f,%u,"
            "%9.6f,%9.6f,%9.6f,%u,"
            "%9.6f,%9.6f,%9.6f,%u,"
            "%9.6f,%9.6f,%9.6f,%u,"
            "%9.6f,%9.6f,%9.6f,%9.6f,%u,%u\n",
            time,
            idata->gpsTs.tow, (unsigned)idata->gpsTs.valid_flags,
            idata->accel.accel[0], idata->accel.accel[1], idata->accel.accel[2], (unsigned)idata->accel.valid_flags,
            idata->ang.gyro[0], idata->ang.gyro[1], idata->ang.gyro[2], (unsigned)idata->ang.valid_flags,
            idata->g.gravity[0], idata->g.gravity[1], idata->g.gravity[2], (unsigned)idata->g.valid_flags, 
            idata->q.q[0], idata->q.q[1], idata->q.q[2], idata->q.q[3], (unsigned)idata->q.valid_flags,flagS
        );

    }

    if (_buffer->output_files != NULL)
    {
        fflush(_buffer->output_files);
    }


    _buffer->last_flush_time = _buffer->sample[_buffer->count - 1].timestamp;
    _buffer->count           = 0;
}


static void sensor_packet_callback(void* _user, const mip_packet_view* _packet_view, mip_timestamp _timestamp)
{
    // This callback is registered for MIP_SENSOR_DATA_DESC_SET (0x80) with afterFields=true.
    // Fields expected per packet (configured in configure_sensor_message_format):
    //   MIP_DATA_DESC_SHARED_GPS_TIME      (0xFF/0xD3) — shared GPS timestamp, decimation=1 (base_rate)
    //   MIP_DATA_DESC_SENSOR_ACCEL_SCALED  (0x80/0x04) — scaled accelerometer data
    //   MIP_DATA_DESC_SENSOR_DELTA_VELOCITY(0x80/0x08) — delta velocity
    //   MIP_DATA_DESC_SENSOR_COMP_QUATERNION(0x80/0x0A) — complementary quaternion
    //   MIP_DATA_DESC_SHARED_REFERENCE_TIME(0xFF/0xD5) — monotonic reference timestamp (never jumps)
    //
    // MIP_DATA_DESC_SHARED_GPS_TIME belongs to MIP_SHARED_DATA_DESC_SET (0xFF) but is embedded
    // inside the 0x80 packet when configured via mip_3dm_write_message_format for that desc set.
    // The (tow, week_number) pair is identical to the one in the filter packet at the same moment,
    // making it the correct cross-packet synchronization key.

    sensor_buffer_t* sampleBuffer = (sensor_buffer_t*)_user;

    if (sampleBuffer == NULL) return;

#if USE_THREADS
    pthread_mutex_lock(&sampleBuffer->buffer_mutex);
#endif // USE_THREADS

    // Field object for iterating the packet and extracting each field
    mip_field_view field_view;
    mip_field_init_empty(&field_view);

    // Use per-buffer context instead of static locals (static locals are not thread-safe
    // and prevent correct reset between packets when threading is enabled)
    sensorData_t* sensorData = &sampleBuffer->pending_sensorData;
    sensorR_t*    sensor     = &sampleBuffer->pending_sensor;

    static uint64_t counter = 0;

    while (mip_field_next_in_packet(&field_view, _packet_view))
    {
        if(sensor->value == 0b00011111) break; // If all data has been extracted, break the loop
        
        switch( mip_field_field_descriptor(&field_view) ){
            
            case MIP_DATA_DESC_SHARED_GPS_TIME:
                    sensor->bits.gpsTs = extract_mip_shared_gps_timestamp_data_from_field(&field_view, &sensorData->gpsTs) ? 1 : 0;
                    MICROSTRAIN_LOG_INFO("GPS %u  | %10.3f\n", counter, sensorData->gpsTs.tow);
                break;

            case MIP_DATA_DESC_SENSOR_ACCEL_SCALED:
                if(sensor->bits.accel == 0){
                    sensor->bits.accel = extract_mip_sensor_scaled_accel_data_from_field(&field_view, &sensorData->accel) ? 1 : 0;
                    MICROSTRAIN_LOG_INFO("Acceleration %u\n", counter);
                }
                break;

            case MIP_DATA_DESC_SENSOR_DELTA_VELOCITY:
                if(sensor->bits.deltaV == 0) {
                    sensor->bits.deltaV = extract_mip_sensor_delta_velocity_data_from_field(&field_view, &sensorData->deltaV) ? 1 : 0;
                    //MICROSTRAIN_LOG_INFO("Delta Velocity %u\n", counter);
                }
                break;

             case MIP_DATA_DESC_SENSOR_COMP_QUATERNION:
                if(sensor->bits.q == 0) {
                    sensor->bits.q = extract_mip_sensor_comp_quaternion_data_from_field(&field_view, &sensorData->q) ? 1 : 0;
                    //MICROSTRAIN_LOG_INFO("Quaternion %u\n", counter);
                }
                break;

             case MIP_DATA_DESC_SHARED_REFERENCE_TIME:
                if(sensor->bits.inTime == 0) {
                    sensor->bits.inTime = extract_mip_shared_reference_timestamp_data_from_field(&field_view, &sensorData->inTime) ? 1 : 0;
                     //MICROSTRAIN_LOG_INFO("Reference Time %u  \n", counter);
                }
                break;

             default:
                break;
        }

    }
     counter++;
    if(sensor->value != 0b00011111)
    {
#if USE_THREADS
        pthread_mutex_unlock(&sampleBuffer->buffer_mutex);
#endif // USE_THREADS        
        return;
    }

    sensorData->timestamp = get_current_timestamp() ;
    
    //MICROSTRAIN_LOG_INFO("Received sensor packet with GPS tow=%.3f\n", sensorData->gpsTs.tow);
    
    // Synchronization via shared GPS timestamp (tow, week_number):
    // Both sensor and filter packets include MIP_DATA_DESC_SHARED_GPS_TIME at decimation=1.
    // The pair (tow, week_number) is the same value for packets that correspond to the same
    // physical moment, regardless of independent descriptor-set clocks or host-side jitter.
    if (sampleBuffer->count < SAMPLE_BUFFER_SIZE)
    {
        size_t i = sampleBuffer->count;
        sampleBuffer->sample[i] = *sensorData;

        sampleBuffer->count++;  
    }

    *sensor     = (sensorR_t){0};
    *sensorData = (sensorData_t){0};
    

    if (sampleBuffer->last_flush_time == 0)
    {
        sampleBuffer->last_flush_time = _timestamp;
    }

    if (
        sampleBuffer->count >= SAMPLE_BUFFER_SIZE ||
        (_timestamp >= sampleBuffer->last_flush_time &&
         _timestamp - sampleBuffer->last_flush_time >= SAMPLE_PRINT_PERIOD_MS)
    )
    {
        flush_sensor_buffer(sampleBuffer);
    }

#if USE_THREADS
    pthread_mutex_unlock(&sampleBuffer->buffer_mutex);
#endif // USE_THREADS
}


static void filter_packet_callback(void* _user, const mip_packet_view* _packet_view, mip_timestamp _timestamp)
{
    // This callback is registered for MIP_FILTER_DATA_DESC_SET (0x82) with afterFields=true.
    // Fields expected per packet (configured in configure_filter_message_format):
    //   MIP_DATA_DESC_SHARED_GPS_TIME             (0xFF/0xD3) — shared GPS timestamp, decimation=1 (base_rate)
    //   MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION  (0x82/0x0E) — linear acceleration estimate
    //   MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE (0x82/0x0E) — compensated angular rate
    //   MIP_DATA_DESC_FILTER_ATT_QUATERNION        (0x82/0x03) — attitude quaternion
    //   MIP_DATA_DESC_FILTER_FILTER_STATUS         (0x82/0x10) — filter status flags
    //   MIP_DATA_DESC_FILTER_GRAVITY_VECTOR        (0x82/0x13) — gravity vector
    //
    // The shared GPS (tow, week_number) from 0xFF/0xD3 matches the sensor packet's shared GPS
    // timestamp at the same physical moment, enabling cross-packet synchronization between
    // MIP_SENSOR_DATA_DESC_SET (0x80) and MIP_FILTER_DATA_DESC_SET (0x82).

    filter_buffer_t* sampleBuffer = (filter_buffer_t*)_user;

    if (sampleBuffer == NULL) return;

#if USE_THREADS
    pthread_mutex_lock(&sampleBuffer->buffer_mutex);
#endif // USE_THREADS

    // Field object for iterating the packet and extracting each field
    mip_field_view field_view;
    mip_field_init_empty(&field_view);

    // Use per-buffer context instead of static locals (static locals are not thread-safe
    // and prevent correct reset between packets when threading is enabled)
    filterData_t* filterData = &sampleBuffer->pending_filterData;
    filterR_t*    filter     = &sampleBuffer->pending_filter;
    
    static uint64_t counter = 0;


    
    while (mip_field_next_in_packet(&field_view, _packet_view))
    {
        if(filter->value == 0b00111111) break; // If all data has been extracted, break the loop
        
        switch(mip_field_field_descriptor(&field_view)){
            
            case MIP_DATA_DESC_SHARED_GPS_TIME:
                    filter->bits.gpsTs = extract_mip_shared_gps_timestamp_data_from_field(&field_view, &filterData->gpsTs) ? 1 : 0;
                    //MICROSTRAIN_LOG_INFO("GPS %u\n", counter);
                break;

            case MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION:
                if(filter->bits.accel == 0) {
                    filter->bits.accel = extract_mip_filter_linear_accel_data_from_field(&field_view, &filterData->accel) ? 1 : 0;
                    //MICROSTRAIN_LOG_INFO("Acceleration %u\n", counter);
                }
                break;

            case MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE:
                if(filter->bits.ang == 0) {
                    filter->bits.ang = extract_mip_filter_comp_angular_rate_data_from_field(&field_view, &filterData->ang) ? 1 : 0;
                    //MICROSTRAIN_LOG_INFO("Angular %u\n", counter);
                }
                break;

             case MIP_DATA_DESC_FILTER_ATT_QUATERNION:
                if(filter->bits.q == 0) {
                    filter->bits.q = extract_mip_filter_attitude_quaternion_data_from_field(&field_view, &filterData->q) ? 1 : 0;
                    //MICROSTRAIN_LOG_INFO("Quaternion %u\n", counter);
                }
                break;

             case MIP_DATA_DESC_FILTER_FILTER_STATUS:
                if(filter->bits.status == 0) {
                    filter->bits.status = extract_mip_filter_status_data_from_field(&field_view, &filterData->status) ? 1 : 0;
                    //MICROSTRAIN_LOG_INFO("Status %u\n", counter);
                }
                break;

             case MIP_DATA_DESC_FILTER_GRAVITY_VECTOR:
                if(filter->bits.g == 0) {
                    filter->bits.g = extract_mip_filter_gravity_vector_data_from_field(&field_view, &filterData->g) ? 1 : 0;
                    //MICROSTRAIN_LOG_INFO("Gravity %u\n", counter);
                }
                break;

             default:
                break;
        }

    }
    counter++;
    if(filter->value != 0b00111111)
    {
#if USE_THREADS
        pthread_mutex_unlock(&sampleBuffer->buffer_mutex);
#endif // USE_THREADS        
        return;
    }
    
    // Synchronization via shared GPS timestamp (tow, week_number):
    // Both sensor and filter packets include MIP_DATA_DESC_SHARED_GPS_TIME at decimation=1.
    // The pair (tow, week_number) is the same value for packets that correspond to the same
    // physical moment, regardless of independent descriptor-set clocks or host-side jitter.
    if (sampleBuffer->count < SAMPLE_BUFFER_SIZE)
    {
        size_t i = sampleBuffer->count;
        sampleBuffer->sample[i]  = *filterData;
        sampleBuffer->count++;
    }

    *filter     = (filterR_t){0};
    *filterData = (filterData_t){0};
    

    if (sampleBuffer->last_flush_time == 0)
    {
        sampleBuffer->last_flush_time = _timestamp;
    }

    if (sampleBuffer->count >= SAMPLE_BUFFER_SIZE || (_timestamp >= sampleBuffer->last_flush_time && _timestamp - sampleBuffer->last_flush_time >= SAMPLE_PRINT_PERIOD_MS))
    {
        flush_filter_buffer(sampleBuffer);
    }

#if USE_THREADS
    pthread_mutex_unlock(&sampleBuffer->buffer_mutex);
#endif // USE_THREADS
}

#if USE_THREADS

static bool update_device(mip_interface* _device, mip_timeout _wait_time, bool _from_cmd)
{
    // SDK data processing flujo (called from data_collection_thread when _from_cmd=false):
    //   1. mip_interface_default_update() reads bytes from the serial port
    //   2. SDK parses bytes into MIP packets
    //   3. SDK iterates over each field in the packet
    //   4. SDK matches field descriptors against registered callbacks
    //   5. SDK invokes packet callbacks with afterFields=true after all field callbacks complete
    //
    // When called from a command handler (_from_cmd=true), we skip the normal update and
    // sleep instead to avoid blocking the data collection thread. This keeps command traffic
    // separated from data traffic across two threads.
    if (!_from_cmd)
    {
        return mip_interface_default_update(_device, _wait_time, _from_cmd);
    }

    // Sleep 5ms when called from command context to save power.
    // Note: Waiting too long here will cause commands to timeout.
    const struct timespec ts = {
        .tv_sec  = 0,
        .tv_nsec = 5 * 1000000  // 5 milliseconds
    };
    nanosleep(&ts, NULL);

    // Must return true to avoid terminating the data collection thread.
    // Returning false may cause a race condition (see mip_interface_wait_for_reply).
    return true;
}


static void* data_collection_thread(void* _thread_data)
{
    MICROSTRAIN_LOG_INFO("Data collection thread created!\n");

    const thread_data_t* thread_data = (thread_data_t*)_thread_data;

    while (thread_data->running)
    {
        // Call mip_interface_update() which drives the full SDK processing flujo:
        // recv bytes → parse MIP packets → iterate fields → invoke callbacks.
        // Recommended wait time is 10ms (blocking read); use 0 for non-blocking.
        const bool updated = mip_interface_update(
            thread_data->device,
            10,   // Wait up to 10ms for data (recommended default)
            false // Not from command — triggers mip_interface_default_update
        );

        // Clean up and exit the thread on failed device updates
        if (!updated)
        {
            // Avoid deadlocks if the connection is closed
            mip_cmd_queue* cmd_queue = mip_interface_cmd_queue(thread_data->device);
            assert(cmd_queue);
            mip_cmd_queue_clear(cmd_queue);

            break;
        }

        sched_yield();
    }

    // Return value unused
    return NULL;
}

#endif // USE_THREADS




void sync_device_time(mip_interface* device) {
    // 1. Obtener tiempo Unix en milisegundos
    uint64_t unix_time_ms = get_current_timestamp();

    // 2. Convertir a segundos Unix
    double unix_seconds = unix_time_ms / 1000.0;

    // 3. Convertir Unix a Tiempo GPS (Segundos totales)
    // Se resta el offset de 10 años y se SUMAN los leap seconds
    uint32_t total_gps_seconds = (uint32_t)(unix_seconds - UNIX_GPS_EPOCH_OFFSET + GPS_LEAP_SECONDS);

    // 4. Calcular Semana y TOW
    uint32_t gps_week = total_gps_seconds / SECONDS_PER_WEEK;
    uint32_t gps_tow  = total_gps_seconds % SECONDS_PER_WEEK;

    // 5. Enviar al dispositivo
    mip_base_write_gps_time_update(device, MIP_BASE_GPS_TIME_UPDATE_COMMAND_FIELD_ID_WEEK_NUMBER, gps_week);
    mip_base_write_gps_time_update(device, MIP_BASE_GPS_TIME_UPDATE_COMMAND_FIELD_ID_TIME_OF_WEEK, gps_tow);
}


/**
 * Convierte el timestamp GPS del sensor a Unix en milisegundos (uint64_t)
 */
uint64_t mip_gps_to_unix_ms(mip_shared_gps_timestamp_data gps_data) {
    
    // 1. Validar que el tiempo sea confiable (Flags para TOW y Week Number)
    // El bit 0 es TOW válido, el bit 1 es Week Number válido.
    // if ((gps_data.valid_flags & 0x03) != 0x03) {
    //     return 0; 
    // }

    // 2. Constantes
    const double _ECONDS_PER_WEEK = 604800.0;
    const double UNIX_GPS_OFFSET  = 315964800.0; // Segundos entre 1970 y 1980
    const double LEAP_SECONDS     = 18.0;        // GPS a UTC

    // 3. Calcular segundos totales en double para preservar precisión decimal
    // (Semana * segundos_por_semana) + TOW + Offset - LeapSeconds
    double unix_seconds = (gps_data.week_number * _ECONDS_PER_WEEK) + 
                          gps_data.tow + 
                          UNIX_GPS_OFFSET - 
                          LEAP_SECONDS;

    // 4. Convertir a milisegundos y pasar a entero de 64 bits
    return (uint64_t)(unix_seconds * 1000.0);
}