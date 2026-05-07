#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>    // Para uint64_t
#include <inttypes.h>
#include <stdbool.h>


/*********************************************************************
 * DEFINES
 */

#define UART_PORT "/dev/ttyUSB1"    // Port Uart GPS
#define BUFFER_SIZE_GPS 300           // Max buffer size for reading GPS data
#define MAX_BUFFER_SIZE_GPS_LOG 350  // Max length for GPS log file names
#define CAPTURE_DURATION_TIME 20000 // ms
#define MAX_FILENAME_LENGTH_GPS 30


/*********************************************************************
 * CONSTANTS
 */

static const uint32_t RUN_TIME_SECONDS = 10*60*60;
/*********************************************************************
 * GLOBAL VARIABLES
 */ 

static bool flagStop = false; 


/*********************************************************************
 * LOCAL VARIABLES
 */ 

static int serialPortGPS;  // File descriptor for UART port
static uint64_t capture_time =0;
static int fileSetCounter = 1; // Counter Global for the files
static char bufferGPS[MAX_BUFFER_SIZE_GPS_LOG]; // buffer for GPS log data
static char stringGPS[BUFFER_SIZE_GPS]; // String to store the complete GPS data frame

/*********************************************************************
 * TYPEDEF ENUMS
 */

typedef enum rData_state
{
    START = 0,
    IDLE_STATE,
    CREATE_NEW_CVS,
    FINISH,
    START_UP,
} rData_state;

/*********************************************************************
 * TYPEDEF STRUCTS
 */


// Estruc for handling CSV files
typedef struct {
    FILE *csv1;
} CsvFiles;



/*********************************************************************
 * PUBLIC FUNCTIONS
 */

uint64_t get_unix_time_ns(void);

// Setting up the UART port for GPS data reading
int configure_uart(const char *port);

// Function to read GPS data from UART and capture complete NMEA sentences
bool read_log(void) ;

// Función para generar un nombre de archivo único
void generateFileName(char *baseName, int counter, char *fileName);

// Función para crear y abrir tres archivos CSV
CsvFiles createCsvFiles(void);

// Función para cerrar los archivos CSV
void closeCsvFiles(CsvFiles files);

// Función para escribir cadenas de texto formateadas en los archivos CSV
void writeDataToCsv(CsvFiles files, const char *line1);

// Función para guardar los datos en los archivos CSV creados
void saveDataToCsv(CsvFiles files);


/*********************************************************************
 * STATIC FUNCTIONS
 */

// Get the current timestamp in milliseconds
static uint64_t get_current_timestamp(void);


int main(int argc, char *argv[]){
    // Configurar puerto UART
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <port>\n", argv[0]);
        return 1;
    }
    const char *port = argv[1];
    
    serialPortGPS = configure_uart(port);
    if (serialPortGPS == -1) {
        return -1;  // Si el puerto no se puede abrir, salir
    }


    int flags = fcntl(STDIN_FILENO, F_GETFL);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    rData_state rState = START;
    CsvFiles currentFiles;
    
    // Get the start time of the device update loop to handle exiting the application
    const uint64_t loop_start_time = get_current_timestamp();

    uint64_t previous_print_idle = get_current_timestamp();
    uint64_t current_print_idle = 0;

    while (1)
    {
        if(get_current_timestamp() - loop_start_time >= RUN_TIME_SECONDS * 1000) break; // Running loop . Exit after a predetermined time in seconds

        char c = 0;
        ssize_t n = read(STDIN_FILENO, &c, 1);

        if(n==1){

                if (c == 'n' || c == 'N') {
                    rState = CREATE_NEW_CVS;
                    printf(" GPS : Creating new files...\n");
                } 
                if (c == 'q' || c == 'Q')  {
                    rState = FINISH;
                    printf(" GPS : Logging is finishing...\n");
                }
                if (c == 's' || c == 'S') {
                    rState = START_UP;
                    printf(" GPS : Logging is starting up...\n");
                } 

                if (c == 'p' || c == 'P') {

                    flagStop = !flagStop;
                    printf(" GPS : Stop trigger\n");
                }
        }

        switch(rState)
        {
            case START:
                    printf(" GPS : Logging  output data for %ds.\n", RUN_TIME_SECONDS);
                    currentFiles = createCsvFiles();
                    fileSetCounter++;
                    rState = IDLE_STATE;
            break;

            case CREATE_NEW_CVS:
                    {
                        flagStop = false;

                        if (fileSetCounter > 1) {
                            printf(" GPS : Closing previous GPS files...\n");
                            closeCsvFiles(currentFiles);
                        }

    
                        currentFiles = createCsvFiles();
    
                        fileSetCounter++;
                        printf(" GPS : Files created.\n");
    
                        rState = IDLE_STATE;
                    }

            break;

            case FINISH:
                        if (fileSetCounter > 1) {
                            printf(" GPS : Closing previous GPS files...\n");
                            closeCsvFiles(currentFiles);
                        }
                        goto go_out;  // Salir del bucle usando goto
            break;

            case START_UP:
                    
                    bool toSaveFlag = false;
                    toSaveFlag = read_log();

                    if(toSaveFlag){
                        saveDataToCsv(currentFiles);
                    }

            break;

            case IDLE_STATE:

                    current_print_idle = get_current_timestamp();
                    if(current_print_idle - previous_print_idle >= 5000){
                        printf(" GPS : Waiting the key 's' or 'S' to start GPS logging.\n");
                        previous_print_idle = current_print_idle;
                    }
            break;

            default:
            break;
                    
        }

    }
    
    go_out:

    fcntl(STDIN_FILENO, F_SETFL, flags);    // Restoring stdin to blocking mode before exiting
    close(serialPortGPS);  // Close the UART port before exiting
    printf(" GPS : Logging completed successfully.\n");

    return 0;
}




int configure_uart(const char *port) {
    int fd = open(port, O_RDONLY);
    if (fd == -1) {
        perror("Error al abrir el puerto UART");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);  

    //Setting baudrate to 4800 bps, 8N1 configuration
    cfsetispeed(&options, B4800);
    cfsetospeed(&options, B4800);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= CREAD | CLOCAL;

    tcsetattr(fd, TCSANOW, &options); 


    return fd;


}



bool read_log(void) {

    static uint16_t idx= 0;
    static bool tramaFound = false;
    char c = 0; 

    if(read(serialPortGPS, &c, 1) <= 0 ) return false; // If no data is read, return false


    if(idx > BUFFER_SIZE_GPS -1) tramaFound = false; // Avoid buffer overflow. If the frame is bigger than the buffer, discard it.
    

    switch (c)
    {
        case '$':
                memset(stringGPS,0,BUFFER_SIZE_GPS);
                capture_time = get_unix_time_ns();
                idx = 0;
                stringGPS[idx] = c;  
                idx++;
                tramaFound = true;
        break;

        case '\r':
        break;

        case '\n':
                
                if(tramaFound){
                    stringGPS[idx] = '\0';  // Finishing the string
                    tramaFound = false;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
                    return true;  // Indicate that a complete frame has been captured
                } 
        break;
          
        default:
            if(tramaFound){
                stringGPS[idx] = c;  
                idx++;
            }
        break;
    }

    return false;  // Indicate that the frame is not yet complete

}

void generateFileName(char *baseName, int counter, char *fileName) {
    sprintf(fileName, "%s_%d.csv", baseName, counter);
}

CsvFiles createCsvFiles(void) {
    CsvFiles files;
    char fileName[MAX_FILENAME_LENGTH_GPS];

    // Generating names for the one file
    generateFileName("GPSlog_", fileSetCounter, fileName);
    files.csv1 = fopen(fileName, "w");
    if (files.csv1 == NULL) {
        perror("Error to open file 1");
        exit(1);
    }

    //CSV Header 
    fprintf(files.csv1, 
            "Unix_time,"
            "GPS_data,flagStop\n");

    return files;
}

void closeCsvFiles(CsvFiles files) {
    if (files.csv1) fclose(files.csv1);

}

// Función para escribir cadenas de texto formateadas en los archivos CSV
void writeDataToCsv(CsvFiles files, const char *line1) {
    //fputs(line1, files.csv1);
    fputs(line1, files.csv1);


}


void saveDataToCsv(CsvFiles files){

    memset(bufferGPS,0,MAX_BUFFER_SIZE_GPS_LOG);
    
    uint8_t valueFlagStop = 0;

    if(flagStop) valueFlagStop = 1;
    else valueFlagStop = 0;

    snprintf(   bufferGPS,
                MAX_BUFFER_SIZE_GPS_LOG,
                "%" PRIu64 ","
                "%s,%u\n",                                                        
                capture_time,                                                       
                stringGPS,
                valueFlagStop                                                                          
            );
    //printf(" GPS : %s", bufferGPS); // Print the captured frame to the console    
    writeDataToCsv(files, bufferGPS);

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
static uint64_t get_current_timestamp(void)
{
    struct timespec ts;

    // Get system UTC time since epoch
    if (timespec_get(&ts, TIME_UTC) != TIME_UTC)
    {
        return 0;
    }

    // Get the time in milliseconds
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}


uint64_t get_unix_time_ns(void) {
    struct timespec ts;

    // Obtiene el tiempo actual en segundos y nanosegundos desde la época UNIX (1 de enero de 1970)
    clock_gettime(CLOCK_REALTIME, &ts);

    // Devuelve el tiempo en nanosegundos (segundos * 1e9 + nanosegundos)
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}