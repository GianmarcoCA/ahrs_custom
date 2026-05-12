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
#include <stdint.h>    
#include <inttypes.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <dirent.h>


////////////////////////////////////////////////////////////////////////////////
// NOTE: Setting these globally for example purposes

/*********************************************************************
 * DEFINES
 */

#define CAPTURE_DURATION_TIME 20000 // Time en ms to capture Gyro Bias

#define MAGNETOMETER_FEATURE 0 // Set to 1 to include magnetometer data and AHRS mode, set to 0 to exclude magnetometer data and use vertical gyro mode

#define MAX_FILENAME_LENGTH 150 // Buffer size for file name

#define INDIVIDUAL_SCRIPT 1 // Set this option to enable individual script testing

#define MAX_PATH 512

#define DEFAULT_CONFIG 1 // Set this option to configure the device with the default settings for the 7-Series AHRS devices. 

#define SC_CONFIG  0 // Set this option to configure the device with the settings used for the SensorConnect application.


// Constantes necesarias
#define UNIX_GPS_EPOCH_OFFSET 315964800L // Segundos entre 1970 y 1980
#define GPS_LEAP_SECONDS 18              // Diferencia actual entre UTC y GPS
#define SECONDS_PER_WEEK 604800L

// TODO: Update to the desired streaming rate. Setting low for readability purposes
/// @brief Streaming rate in Hz (applies to both sensor and filter data)

#define SAMPLE_RATE_HZ  100

#define SAMPLE_GPS_RATE_HZ  200

/*********************************************************************
 * CONSTANTS
 */

// TODO: Update to the correct port name and baudrate
/// @brief  Set the port name for the connection (Serial/USB)
#ifdef _WIN32
static const char* PORT_NAME = "COM1";
#else  // Unix
//static const char* PORT_NAME = "/dev/ttyUSB1";
#endif // _WIN32

/// @brief  Set the baudrate for the connection (Serial/USB)
/// @note For native serial connections this needs to be 115200 due to the device default settings command
/// Use mip_base_*_comm_speed() to write and save the baudrate on the device
static uint32_t BAUDRATE = 0;

// TODO: Update to change the example run time
/// @brief Example run time
static const uint32_t RUN_TIME_SECONDS = 10*60*60;


/*********************************************************************
 * GLOBAL VARIABLES
 */ 


/*********************************************************************
 * TYPEDEF ENUMS
 */

typedef enum rData_state
{
    START = 0,
    RESET_FILTER,
    CREATE_NEW_CVS,
    FINISH,
    START_UP,
    MIP_IDLE,
} rData_state;

/*********************************************************************
 * TYPEDEF STRUCTS
 */

// Estructura para mantener los archivos CSV abiertos
typedef struct {
    FILE *csv1;
    FILE *csv2;
} CsvFiles;

typedef struct {
    bool flagStop;
    bool flagDirection;
} flag_plus_t;

typedef struct {
    mip_shared_gps_timestamp_data        gpsTs;
    mip_sensor_scaled_accel_data         accel;
    mip_sensor_delta_velocity_data       deltaV;
    mip_sensor_comp_quaternion_data      q;      
} sensorData_t;

typedef struct{
    mip_shared_gps_timestamp_data        gpsTs;
    mip_filter_linear_accel_data         accel;
    mip_filter_comp_angular_rate_data    ang;
    mip_filter_gravity_vector_data       g;
    mip_filter_attitude_quaternion_data  q;
} filterData_t;

typedef union {
    uint8_t value;      // Acceso al dato completo (16 bits)
    struct { 
        uint8_t gpsTs  : 1;
        uint8_t accel  : 1;
        uint8_t deltaV : 1;
        uint8_t q      : 1;
        // ... puedes nombrar hasta el bit15
    } bits;
} sensorR_t;

typedef union {
    uint8_t value;      // Acceso al dato completo (16 bits)
    struct {
        uint8_t gpsTs   : 1;
        uint8_t accel   : 1;
        uint8_t ang     : 1;
        uint8_t g       : 1;
        uint8_t q       : 1;
        // ... puedes nombrar hasta el bit15
    } bits;
} filterR_t;

typedef struct {
    sensorData_t sample;
    size_t count;
    mip_timestamp  last_flush_time;
    sensorData_t pending_sensorData;
    sensorR_t    pending_sensor;
    FILE* output_file;
    flag_plus_t flag;
} sensor_buffer_t;

typedef struct {
    filterData_t sample;
    size_t count;
    mip_timestamp  last_flush_time;
    filterData_t pending_filterData;
    filterR_t    pending_filter;
    FILE* output_file;
    flag_plus_t flag;
} filter_buffer_t;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

// Función para crear y abrir tres archivos CSV
void createCsvFiles(sensor_buffer_t* sensor_B, filter_buffer_t* filter_B, uint8_t count);

// Función para cerrar los archivos CSV
void closeCsvFiles(FILE* file1, FILE* file2); 

// Función para encontrar el puerto del dispositivo basado en una palabra clave
char* find_device(const char* keyword);

void sync_device_time(mip_interface* device);

uint64_t mip_gps_to_unix_ms(mip_shared_gps_timestamp_data gps_data);

void save_sensor_data(sensor_buffer_t* _buffer);

void save_filter_data(filter_buffer_t*  _buffer);


/*********************************************************************
 * PRIVATE  FUNCTIONS
 */

static void sensor_packet_callback(void* _user, const mip_packet_view* _packet_view, mip_timestamp _timestamp);

static void filter_packet_callback(void* _user, const mip_packet_view* _packet_view, mip_timestamp _timestamp);

// Custom logging handler callback
static void log_callback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);


#if SC_CONFIG == 0
// Capture gyro bias
static void capture_gyro_bias(mip_interface* _device);

// Sensor message format configuration
static void configure_sensor_message_format(mip_interface* _device);

// Filter message format configuration
static void configure_filter_message_format(mip_interface* _device);

// Filter initialization
static void initialize_filter(mip_interface* _device);
#endif



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
    #ifndef MICROSTRAIN_LOGGING_ENABLED_INFO
    #error This example requires a logging level of at least MICROSTRAIN_LOGGING_LEVEL_INFO_ to work properly
    #endif // !MICROSTRAIN_LOGGING_ENABLED_INFO

    // Initialize the custom logger to print messages/errors as they occur
    // Note: The logging level parameter doesn't need to match the max logging level.
    // If the parameter is higher than the max level, higher-level logging functions will be ignored
    MICROSTRAIN_LOG_INIT(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, NULL);

    
    // Initialize the connection
    MICROSTRAIN_LOG_INFO("Initializing the connection.\n");
    serial_port device_port;
    serial_port_init(&device_port);

#if INDIVIDUAL_SCRIPT == 1

    (void)argc;
    (void)argv;

    BAUDRATE = 460800;

    char* PORT_NAME = find_device("FTDI");

    //MICROSTRAIN_LOG_INFO(" Connecting to the device on port :%s ", PORT_NAME);

    MICROSTRAIN_LOG_INFO("Enter the baud rate: ");

    BAUDRATE = 0;

    if (scanf("%u", &BAUDRATE) != 1) {
        MICROSTRAIN_LOG_INFO("Invalid input\n");
        return 1;
    }

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
    //mip_3dm_save_sensor_range(&device, MIP_SENSOR_RANGE_TYPE_ACCEL);
    
    // Set gyro a 500 dps
    mip_3dm_write_sensor_range(&device, MIP_SENSOR_RANGE_TYPE_GYRO,3);
    // mip_3dm_save_sensor_range(&device, MIP_SENSOR_RANGE_TYPE_GYRO);

    // Capture gyro bias
    capture_gyro_bias(&device);
    //mip_3dm_save_gyro_bias(&device); 

    // Initialize the navigation filter
    initialize_filter(&device);

    // Configure the message format for sensor data
    configure_sensor_message_format(&device);
    
    // Configure the message format for filter data
    configure_filter_message_format(&device);
    
    mip_3dm_save_device_settings(&device);  
    
#endif
    
    MICROSTRAIN_LOG_INFO("Registering packet callbacks.\n");

    sensor_buffer_t sensorB = {0};
    filter_buffer_t filterB = {0};
    
    mip_dispatch_handler sensor_packet_handler;
    mip_dispatch_handler filter_packet_handler;

    mip_interface_register_packet_callback(
        &device,
        &sensor_packet_handler,
        MIP_SENSOR_DATA_DESC_SET,
        false,  // afterFields=true: fires after all fields in this packet have been processed
        &sensor_packet_callback,
        &sensorB
    );

    mip_interface_register_packet_callback(
        &device,
        &filter_packet_handler,
        MIP_FILTER_DATA_DESC_SET,
        false,  // afterFields=true: fires after all fields in this packet have been processed
        &filter_packet_callback,
        &filterB
    );
    
    mip_cmd_result cmd_result;

    MICROSTRAIN_LOG_INFO("Attempting to reset the navigation filter.\n");
    cmd_result = mip_filter_reset(&device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(&device, cmd_result, "Could not reset the navigation filter!\n");
    }

    MICROSTRAIN_LOG_INFO("Resuming the device.\n");
    cmd_result = mip_base_resume(&device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(&device, cmd_result, "Could not resume the device!\n");
    }
    
    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    
    
    int flags = fcntl(STDIN_FILENO, F_GETFL);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    rData_state rState = START;
    
    const mip_timestamp loop_start_time = get_current_timestamp();
    
    mip_timestamp previous_print_idle = loop_start_time;
    mip_timestamp current_print_idle = loop_start_time;
    
    int fileCounter = 1; // Contador para los archivos
    

    bool flagExit = false;

    while (1)
    {
        if(get_current_timestamp() - loop_start_time >= RUN_TIME_SECONDS * 1000) break; // Running loop . Exit after a predetermined time in seconds
        if(flagExit) break; 

        char c = 0;
        ssize_t n = read(STDIN_FILENO, &c, 1);

        if(n==1){

                if (c == 'n' || c == 'N') {
                    rState = CREATE_NEW_CVS;
                    MICROSTRAIN_LOG_INFO("Creating new INS files...\n");
                } 
                if (c == 'q' || c == 'Q')  {
                    rState = FINISH;
                    MICROSTRAIN_LOG_INFO("INS logging is finishing...\n");
                }
                if (c == 's' || c == 'S') {
                    rState = START_UP;
                    MICROSTRAIN_LOG_INFO("INS logging is starting up...\n");
                } 

                if (c == 'p' || c == 'P') {
                    sensorB.flag.flagStop = !sensorB.flag.flagStop;
                    filterB.flag.flagStop = !filterB.flag.flagStop;
                    MICROSTRAIN_LOG_INFO("Stop trigger\n");
                }
        }

        switch(rState)
        {
            case START:
                    MICROSTRAIN_LOG_INFO("INS logging  output data for %ds.\n", RUN_TIME_SECONDS);
                    createCsvFiles(&sensorB, &filterB, fileCounter++);
                    rState = MIP_IDLE;
            break;

            case CREATE_NEW_CVS:
                    sensorB.flag.flagStop = false;
                    filterB.flag.flagStop = false;

                    MICROSTRAIN_LOG_INFO("Setting the INS device to idle.\n");
                    cmd_result = mip_base_set_idle(&device);

                    if (!mip_cmd_result_is_ack(cmd_result))
                    {
                        exit_from_command(&device, cmd_result, "Could not set the INS device to idle!\n");
                    }
                    
                    if (fileCounter > 1) {
                        MICROSTRAIN_LOG_INFO("Closing previous INS files...\n");
                        closeCsvFiles(sensorB.output_file, filterB.output_file);
                    }

                    createCsvFiles(&sensorB, &filterB, fileCounter++);
                    MICROSTRAIN_LOG_INFO("INS files created.\n");

                    rState =RESET_FILTER;
                
            break;

            case RESET_FILTER:
                    
                    MICROSTRAIN_LOG_INFO("Resetting the INS filter...\n");
                    cmd_result = mip_filter_reset(&device);

                    if (!mip_cmd_result_is_ack(cmd_result))
                    {
                        exit_from_command(&device, cmd_result, "Could not reset the INS filter!\n");
                    }

                    sleep(5); // Sleep for x second to allow the filter reset to process before resuming the device

                    MICROSTRAIN_LOG_INFO("Resuming the INS device.\n");
                    cmd_result = mip_base_resume(&device);

                    if (!mip_cmd_result_is_ack(cmd_result))
                    {
                        exit_from_command(&device, cmd_result, "Could not resume the INS device!\n");
                    }

                    rState = MIP_IDLE;
                    
            break;


            case FINISH:

                    if (fileCounter > 1) {
                        MICROSTRAIN_LOG_INFO("Closing previous INS files...\n");
                        closeCsvFiles(sensorB.output_file, filterB.output_file);
                    }

                    MICROSTRAIN_LOG_INFO("Setting the INS device to idle.\n");

                    cmd_result = mip_base_set_idle(&device);

                    if (!mip_cmd_result_is_ack(cmd_result))
                    {
                        exit_from_command(&device, cmd_result, "Could not set the INS device to idle!\n");
                    }

                    flagExit = true;

            break;

            case START_UP:
                    // Update the device state
                    // Note: This will update the device callbacks to trigger the filter state change
                    // Note: The recommended default wait time is 10 ms, but could be 0 for non-blocking read operations
                    mip_interface_update(
                        &device,
                        0,   // Time to wait   
                        false // From command
                    );

            break;

            case MIP_IDLE:
                    current_print_idle = get_current_timestamp();
                    if(current_print_idle - previous_print_idle >= 60000){
                        MICROSTRAIN_LOG_INFO("Waiting the key 's' or 'S' to start INS logging.\n");
                        previous_print_idle = current_print_idle;
                    }
            break;

            default:
            break;
                    
        }

    }
    
    // Restoring stdin to blocking mode before exiting
    fcntl(STDIN_FILENO, F_SETFL, flags);
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
    // Unused parameter
    (void)_user;

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
    // Actual rate will be: base_rate / decimation
    // For exact rate: decimation must divide base_rate evenly
    // Calculate from SAMPLE_RATE_HZ
    const uint16_t sensor_decimation = sensor_base_rate / SAMPLE_RATE_HZ;
    const uint16_t gps_decimation = sensor_base_rate / SAMPLE_GPS_RATE_HZ;

    MICROSTRAIN_LOG_INFO(
        "SENSOR: Base=%dHz | Decimation=%d (Requested: %dHz)\n",
        sensor_base_rate,
        sensor_decimation,
        SAMPLE_RATE_HZ
    );

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip_descriptor_rate sensor_descriptors[4] = {
        {MIP_DATA_DESC_SHARED_GPS_TIME,         gps_decimation},
        {MIP_DATA_DESC_SENSOR_ACCEL_SCALED,     sensor_decimation},
        {MIP_DATA_DESC_SENSOR_DELTA_VELOCITY,   sensor_decimation},
        {MIP_DATA_DESC_SENSOR_COMP_QUATERNION,  sensor_decimation},
        //{MIP_DATA_DESC_SHARED_REFERENCE_TIME,   sensor_decimation}
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
    // For exact rate: decimaion must divide base_rate evenly
    // Calculate from SAMPLE_RATE_HZ
    const uint16_t filter_decimation = filter_base_rate / SAMPLE_RATE_HZ;  
    const uint16_t gps_decimation = filter_base_rate / SAMPLE_GPS_RATE_HZ;  

    MICROSTRAIN_LOG_INFO(
        "FILTER: Base=%dHz | Decimation=%d (Requested: %dHz)\n",
        filter_base_rate,
        filter_decimation,
        SAMPLE_RATE_HZ
    );

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip_descriptor_rate filter_descriptors[5] = {
        {MIP_DATA_DESC_SHARED_GPS_TIME,                     gps_decimation},
        {MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION,          filter_decimation},
        {MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE,     filter_decimation},
        {MIP_DATA_DESC_FILTER_GRAVITY_VECTOR,               filter_decimation},
        {MIP_DATA_DESC_FILTER_ATT_QUATERNION,               filter_decimation},
        //{MIP_DATA_DESC_FILTER_FILTER_STATUS,                filter_decimation},
        // {MIP_DATA_DESC_FILTER_GYRO_BIAS,                    filter_decimation},
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
#endif



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

    // IMPORTANTE: Reconfigurar el puerto serial al nuevo baudrate  
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


void createCsvFiles(sensor_buffer_t* sensor_B, filter_buffer_t* filter_B, uint8_t count) {
    CsvFiles files;
    char fileName[MAX_FILENAME_LENGTH];

    // Generar nombres para los tres archivos

    sprintf(fileName, "%s_%d.csv", "SensorData_", count);
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
            "orientQuaternion[0],orientQuaternion[1],orientQuaternion[2],orientQuaternion[3],flagStop\n");

    sprintf(fileName, "%s_%d.csv", "FilterData_", count);
    files.csv2 = fopen(fileName, "w");
    if (files.csv2 == NULL) {
        perror("Error to open file 2");
        exit(1);
    }

    fprintf(files.csv2, 
            "Time,fGPS_ts,fGPS_ts:valid,"
            "estLinearAccelX,estLinearAccelY,estLinearAccelZ,estLinearAccel:valid,"
            "estAngularRateX,estAngularRateY,estAngularRateZ,estAngularRate:valid,"
            "estGravityX,estGravityY,estGravityZ,estGravity:valid,"
            "estOrientQuaternion[0], estOrientQuaternion[1],estOrientQuaternion[2],estOrientQuaternion[3],estOrientQuaternion:valid,flagStop\n");
    
    sensor_B->output_file = files.csv1;
    filter_B->output_file = files.csv2;

    return ;
}

void closeCsvFiles(FILE* file1, FILE* file2) {
    if (file1) fclose(file1);
    if (file2) fclose(file2);
}

void save_sensor_data(sensor_buffer_t* _buffer)
{
    const sensorData_t* data = &_buffer->sample;

    uint64_t time = mip_gps_to_unix_ms(data->gpsTs);

    fprintf(_buffer->output_file,
        "%" PRIu64 "," 
        "%10.3f,%u,"                              
        "%9.6f,%9.6f,%9.6f,"
        "%9.6f,%9.6f,%9.6f,"
        "%9.6f,%9.6f,%9.6f,%9.6f,%u\n",                                                        
        time,
        data->gpsTs.tow, data->gpsTs.valid_flags,
        data->accel.scaled_accel[0], data->accel.scaled_accel[1], data->accel.scaled_accel[2], 
        data->deltaV.delta_velocity[0], data->deltaV.delta_velocity[1], data->deltaV.delta_velocity[2],
        data->q.q[0], data->q.q[1], data->q.q[2], data->q.q[3], _buffer->flag.flagStop 
    );

}

void save_filter_data(filter_buffer_t*  _buffer){

    const filterData_t* data = &_buffer->sample;

    uint64_t time = mip_gps_to_unix_ms(data->gpsTs);

    fprintf(_buffer->output_file,
        "%" PRIu64 ","
        "%10.3f,%u,"
        "%9.6f,%9.6f,%9.6f,%u,"
        "%9.6f,%9.6f,%9.6f,%u,"
        "%9.6f,%9.6f,%9.6f,%u,"
        "%9.6f,%9.6f,%9.6f,%9.6f,%u,%u\n",
        time,
        data->gpsTs.tow, data->gpsTs.valid_flags,
        data->accel.accel[0], data->accel.accel[1], data->accel.accel[2], data->accel.valid_flags,
        data->ang.gyro[0], data->ang.gyro[1], data->ang.gyro[2], data->ang.valid_flags,
        data->g.gravity[0], data->g.gravity[1], data->g.gravity[2], data->g.valid_flags, 
        data->q.q[0], data->q.q[1], data->q.q[2], data->q.q[3], data->q.valid_flags,_buffer->flag.flagStop 
    );

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


static void sensor_packet_callback(void* _user, const mip_packet_view* _packet_view, mip_timestamp _timestamp)
{
    (void)_timestamp;

    
    sensor_buffer_t* sampleBuffer = (sensor_buffer_t*)_user;
    
    if (sampleBuffer == NULL) return;
    
    // Field object for iterating the packet and extracting each field
    mip_field_view field_view;
    mip_field_init_empty(&field_view);
    
    // Use per-buffer context instead of static locals (static locals are not thread-safe
    // and prevent correct reset between packets when threading is enabled)
    sensorData_t* sensorData = &sampleBuffer->pending_sensorData;
    sensorR_t*    sensor     = &sampleBuffer->pending_sensor;
    
    static uint16_t counter = 0;
    
    

    while (mip_field_next_in_packet(&field_view, _packet_view))
    {

        if(sensor->value == 0b00001111) break; // If all data has been extracted, break the loop

        switch( mip_field_field_descriptor(&field_view) ){
            
            case MIP_DATA_DESC_SHARED_GPS_TIME:
                    sensor->bits.gpsTs = extract_mip_shared_gps_timestamp_data_from_field(&field_view, &sensorData->gpsTs) ? 1 : 0;
                    break;

            case MIP_DATA_DESC_SENSOR_ACCEL_SCALED:
                if(sensor->bits.accel == 0) sensor->bits.accel = extract_mip_sensor_scaled_accel_data_from_field(&field_view, &sensorData->accel) ? 1 : 0;
                break;

            case MIP_DATA_DESC_SENSOR_DELTA_VELOCITY:
                if(sensor->bits.deltaV == 0) sensor->bits.deltaV = extract_mip_sensor_delta_velocity_data_from_field(&field_view, &sensorData->deltaV) ? 1 : 0;
                break;

             case MIP_DATA_DESC_SENSOR_COMP_QUATERNION:
                if(sensor->bits.q == 0) sensor->bits.q = extract_mip_sensor_comp_quaternion_data_from_field(&field_view, &sensorData->q) ? 1 : 0;
                break;

             case MIP_DATA_DESC_SHARED_REFERENCE_TIME:
                break;

             default:
                break;
        }

    }

    counter ++ ;

    if(sensor->value != 0b00001111) return;

    sampleBuffer->sample = *sensorData;

    save_sensor_data(sampleBuffer);

    sampleBuffer->pending_sensor = (sensorR_t){0};
    sampleBuffer->pending_sensorData = (sensorData_t){0};

    //sampleBuffer->flagReady = true;

    return;

}


static void filter_packet_callback(void* _user, const mip_packet_view* _packet_view, mip_timestamp _timestamp)
{
    (void)_timestamp;

    filter_buffer_t* sampleBuffer = (filter_buffer_t*)_user;

    if (sampleBuffer == NULL) return;

    // Field object for iterating the packet and extracting each field
    mip_field_view field_view;
    mip_field_init_empty(&field_view);

    // Use per-buffer context instead of static locals (static locals are not thread-safe
    // and prevent correct reset between packets when threading is enabled)
    filterData_t* filterData = &sampleBuffer->pending_filterData;
    filterR_t*    filter     = &sampleBuffer->pending_filter;
    
    
    while (mip_field_next_in_packet(&field_view, _packet_view))
    {
        if(filter->value == 0b00011111) break; // If all data has been extracted, break the loop
        
        switch(mip_field_field_descriptor(&field_view)){
            
            case MIP_DATA_DESC_SHARED_GPS_TIME:
                    filter->bits.gpsTs = extract_mip_shared_gps_timestamp_data_from_field(&field_view, &filterData->gpsTs) ? 1 : 0;
                break;

            case MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION:
                if(filter->bits.accel == 0) filter->bits.accel = extract_mip_filter_linear_accel_data_from_field(&field_view, &filterData->accel) ? 1 : 0;
                break;

            case MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE:
                if(filter->bits.ang == 0) filter->bits.ang = extract_mip_filter_comp_angular_rate_data_from_field(&field_view, &filterData->ang) ? 1 : 0;
                break;
                
            case MIP_DATA_DESC_FILTER_GRAVITY_VECTOR:
                if(filter->bits.g == 0) filter->bits.g = extract_mip_filter_gravity_vector_data_from_field(&field_view, &filterData->g) ? 1 : 0;
                break;

             case MIP_DATA_DESC_FILTER_ATT_QUATERNION:
                if(filter->bits.q == 0) filter->bits.q = extract_mip_filter_attitude_quaternion_data_from_field(&field_view, &filterData->q) ? 1 : 0;
                break;
               
             case MIP_DATA_DESC_FILTER_FILTER_STATUS:
                break;

             default:
                break;
        }

    }

    if(filter->value != 0b00011111) return;

    sampleBuffer->sample  = *filterData;

    save_filter_data(sampleBuffer);
    sampleBuffer->pending_filter = (filterR_t){0};
    sampleBuffer->pending_filterData = (filterData_t){0};

    //sampleBuffer->flagReady = true;

    return;

}

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