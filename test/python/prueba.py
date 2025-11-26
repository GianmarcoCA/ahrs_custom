import subprocess
import os
import csv
import time

# usar doxygen

print("Debug start...")

BINARY_PATH = "./build/examples/c/7_series/ahrs/7_series_ahrs_example_c"
THIS_FILE_ABS_PATH = os.path.dirname(os.path.abspath(__file__))
TEST_SCRIPT_PATH_REL_TO_PROJECT = "/test/python"
PROJECT_PATH = THIS_FILE_ABS_PATH.replace(TEST_SCRIPT_PATH_REL_TO_PROJECT, "")
BINARY_ABS_PATH = os.path.abspath(os.path.join(PROJECT_PATH, BINARY_PATH))

CSV_OUTPUT_FILE = os.path.abspath(os.path.join(PROJECT_PATH, "test/captures/logs.csv"))


print("THIS_FILE_ABS_PATH:", THIS_FILE_ABS_PATH)
print("PROJECT_PATH: ",PROJECT_PATH)
print("BINARY_ABS_PATH:", BINARY_ABS_PATH)
print("¿Existe el binario?:", os.path.exists(BINARY_ABS_PATH))
print("CSV_OUTPUT_FILE:", CSV_OUTPUT_FILE)

SUDO = "sudo"

with open(CSV_OUTPUT_FILE, "w", newline="") as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=';')
    spamwriter.writerow(["timestamp", "log"]) # encabezado
    
    process = subprocess.Popen(
            [SUDO, BINARY_PATH],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
    )

    for line in process.stdout:
        line = line.strip()
        print(line)
        
        spamwriter.writerow([time.time(), line])
        csvfile.flush()   # guardar en disco inmediatamente
        
process.wait()

print("CSV guardado en:", CSV_OUTPUT_FILE)
