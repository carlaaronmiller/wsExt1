from pymavlink import mavutil
from datetime import datetime
from SPFromC import SP_from_C
import bluerobotics_navigator as navigator
from fastapi import FastAPI
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse

import uvicorn
import asyncio
import time
import serial
import glob
import sys
import ms5837
import requests
# -------------------------------MAV CMD PARAMS-------------------------------
SENSOR_PWR_RELAY_CHANNEL = 6 # Zero indexed, physical port 7.
SAMPLE_TRIGGER_CHANNEL = 3 # Physical port 2 on Tealas Sampler, 3 on DAL WS.
LUMEN_CHANNEL = 9 # PWM channel for the lumen on Dal WS.
RELAY_ON = 1.0
RELAY_MID = 0.25
RELAY_OFF = 0
PWM_LOW = 225  # PWM values not in microsecond, but as fraction of 4096 block to reach standard 1100, 1500, and 1900 periods.
PWM_MID = 307
PWM_HIGH = 389
SERVO_PWM_FREQUENCY_HZ = 50
MAV_CMD_DO_REPEAT_SERVO_ID = 183
TARGET_SYSTEM = 1
TARGET_COMPONENT = 1
CONFIRMATION = 0
UNUSED_PARAM = 0
MAVLINK_PAUSE_TIME_SECONDS = 1
RELAY_PAUSE_TIME_SECONDS = 2
# -------------------------------AML LOOP PARAMS-------------------------------
REFRESH_PERIOD_SECONDS = 1
NO_DATA_VAL = -1
SENSOR_ERROR_VAL = -2
TEXT_BACKUP_HEADER = "Time, BAR30-Depth (m), BAR30-Temp (°C), AML Cond (mS/cm), AML Temp (°C), PSU (Calulated), AML Chloro (μg/L), AML Rho (ppb), AML Turb (NTU),  AML DO (μmol/L)\n"
# -------------------------------PHYSICAL CONSTANTS-------------------------------
STANDARD_ATMOSPHERIC_PRESSURE_HPA = 1013.25
DEG_C_PER_DEG_CENTI_C = 0.01
GRAV_ACC = 9.8
SALTWATER_DENSITY_KGM3 = 1023.6
SEC_TO_MICROSEC = 1e6
HPA_TO_PA = 100
HPA_TO_BAR = 100
DECI_PER_MILLI = 0.01
MINIMUM_SALT_WATER_COND = 0 #25 for Seawater # mS/cm
# -------------------------------CONNECTION SETUP-------------------------------
MASTER_ADDRESS = "tcp:127.0.0.1:5777"
BOOT_TIME = time.time()
# -------------------------------SENSOR COMM PARAMS-------------------------------
SENSOR_DICT = {
    "CT.X2": None,
    "Chloro-blue": None,
    "Rhodamine": None,
    "Turbidity": None,
    "Dissolved Oxygen": None,
}
SENSOR_BAUD_RATE = 9600
SENSOR_REBOOT_TIME_SECONDS = (
    10  # Healthy amount of time from power to actually streaming data.
)
SENSOR_CMD_WAIT_TIME_SECONDS = 1  # Give sensors time to respond to the cmd.
SENSOR_RESPONSE_TIMEOUT_SECONDS = (
    5  # If nothing after this amount of time, nothings coming.
)
MS5837_BUS = 6
BAR30_REFRESH_PERIOD_SECONDS = 0.25
DEPTH_SENSOR_PRINT_PERIOD = 1.0
AUTOPILOT_SHUTDOWN_SECONDS = 5

# -------------------------------FUNCTIONS-------------------------------
def power_cycle_sensors():  # Power cycles the sensors on defined port to trip RC Switch to high.
    print("Powering off sensors.")
    navigator.set_pwm_channel_value(SENSOR_PWR_RELAY_CHANNEL, PWM_LOW)
    time.sleep(SENSOR_REBOOT_TIME_SECONDS)
    print("Powering on sensors.")
    navigator.set_pwm_channel_value(SENSOR_PWR_RELAY_CHANNEL, PWM_HIGH)
    time.sleep(SENSOR_REBOOT_TIME_SECONDS)


def discover_devices():
    #power_cycle_sensors()  # Trigger the RC switch to give the sensors power.
    start_time = time.time()

    while time.time() - start_time < SENSOR_RESPONSE_TIMEOUT_SECONDS:
        usb_devices = glob.glob("/dev/ttyUSB*")
        if usb_devices:
            break
        time.sleep(SENSOR_CMD_WAIT_TIME_SECONDS)

    if not usb_devices:
        print(
            f"No USB devices detected after {SENSOR_RESPONSE_TIMEOUT_SECONDS} seconds. Exiting."
        )
        sys.exit(1)

    for dev in usb_devices:
        try:
            print(f"Probing {dev}...")
            ser = serial.Serial(
                dev, SENSOR_BAUD_RATE, timeout=SENSOR_CMD_WAIT_TIME_SECONDS
            )
            time.sleep(SENSOR_CMD_WAIT_TIME_SECONDS)
            ser.write(b"\r")
            time.sleep(SENSOR_CMD_WAIT_TIME_SECONDS)
            ser.write(b"display options\r")

            start_time = time.time()
            found = False
            while time.time() - start_time < SENSOR_RESPONSE_TIMEOUT_SECONDS:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                for s_name in SENSOR_DICT:
                    if SENSOR_DICT[s_name] is None and s_name in line:
                        print(f"{s_name} found on <{dev}>.")
                        SENSOR_DICT[s_name] = ser
                        found = True
                        time.sleep(SENSOR_REBOOT_TIME_SECONDS) #Give the sensor time to break out of menu.

            if not found:
                print(f"No matching sensor on {dev}. Closed.")
                ser.close()

        except serial.SerialException as e:
            print(f"Failed to connect to {dev}: {e}")

    return SENSOR_DICT


def get_sensor_line(ser_num):
    
    if not ser_num or not ser_num.is_open:
        print("Attempted to read from closed or invalid serial port.")
        return ""
    try:
        ser_num.flushInput()
        ser_num.flushOutput()
        ser_num.readline()  # discard partial/incomplete line
        return ser_num.readline().decode("utf-8", errors="ignore").strip()
    except serial.SerialException as e:
        print(f"Serial error during read: {e}")
        return ""


def get_ct_nums(sen):
    sensor_line = get_sensor_line(SENSOR_DICT[sen])
    split_line = sensor_line.split()
    ct_vals = [None] * 2
    if len(split_line) < 2:
        print(
            f"Unexpected sensor output for {sen}: '{sensor_line}'. Expected at least 2 values."
        )
        return [SENSOR_ERROR_VAL, SENSOR_ERROR_VAL]
    ct_vals[0] = float(split_line[0])  # C
    ct_vals[1] = round((float(split_line[1])), 2)  # T
    return ct_vals


def get_single_val(sen):
    sensor_line = get_sensor_line(SENSOR_DICT[sen])
    try:
        sensor_val = int(float(sensor_line))
    except ValueError:
        print(f"{sen} bad line -> {sensor_line!r}") 
        sensor_val = SENSOR_ERROR_VAL
    return sensor_val



def get_message(mavlink_conn, msg_type):
    msg = None
    while temp := mavlink_conn.recv_match(type=msg_type):
        msg = temp
    if msg is None:
        msg = mavlink_conn.recv_match(type=msg_type, blocking=True)
    return msg


def setup_text_backup():
    file_name = f"/usr/blueos/userdata/sensorData/{datetime.now().date()}.txt"
    backup_file = open(file_name, "a")
    backup_file.write(TEXT_BACKUP_HEADER)
    return backup_file


def write_to_backup(file, line):
    try:
        file.write(line + "\n")
    except Exception as e:
        print(f"Error writing to backup file: {e}")


# Depth Sensor reading loop:
bar30_depth = 0.0  # Global variables to pass between different loops.
bar30_temp = 0.0
bar30_mbar = 0.0
sal_psu = 0.0
aml_values = {
    "CT.X2": {"cond": -1.0, "temp": -1.0},
    "Chloro-blue": -1,
    "Rhodamine": -1,
    "Turbidity": -1,
    "Dissolved Oxygen": -1,
}
text_backup = None

async def depth_sensor_loop():
    global bar30_depth, bar30_temp, bar30_mbar
    bar30 = ms5837.MS5837_30BA(bus=MS5837_BUS)
    bar30.setFluidDensity(
        SALTWATER_DENSITY_KGM3
    )  # Set density for seawater, if using freshwater comment out or change.
    bar30.init()
    last_print = 0
    while True:
        if bar30.read():
            bar30_depth = round(bar30.depth(), 2)
            bar30_temp = round(bar30.temperature(), 2)
            bar30_mbar = round(bar30.pressure(), 3)
            if time.time() - last_print >= DEPTH_SENSOR_PRINT_PERIOD:
                print(f"Current Depth: {bar30_depth:.2f} m")
                print(f"Current Temperature: {bar30_temp:.2f} °C")
                last_print = time.time()
        else:
            print("Failed to read BAR30.")
        await asyncio.sleep(BAR30_REFRESH_PERIOD_SECONDS)


app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

class LumenCmd(BaseModel):
    duty_cycle: float

@app.post("/set_lumen")
async def set_lumen(cmd: LumenCmd):
    print(f"received set lumen: {cmd.duty_cycle}")
    navigator.set_pwm_channel_duty_cycle(LUMEN_CHANNEL, cmd.duty_cycle)
       


@app.post("/trigger_sample")
async def trigger_sample():
    global text_backup
    try:
        print("Triggering sample.", flush=True)
        navigator.set_pwm_channel_duty_cycle(SAMPLE_TRIGGER_CHANNEL, RELAY_OFF)
        await asyncio.sleep(RELAY_PAUSE_TIME_SECONDS)
        navigator.set_pwm_channel_duty_cycle(SAMPLE_TRIGGER_CHANNEL, RELAY_ON)

        if text_backup:
            timestamp = datetime.now().strftime("%H:%M:%S")
            text_backup.write(f"{timestamp}, SAMPLE TRIGGERED, depth={bar30_depth:.2f}, temp={bar30_temp:.2f}\n")
            text_backup.flush()

        return {"status": "done", "depth": bar30_depth, "temp": bar30_temp}
    except Exception as e:
        print(f"Trigger sample error: {e}", flush=True)
        raise

@app.get("/sensor_data")
async def sensor_data():
    flat_aml = {}
    for key, val in aml_values.items():
        if isinstance(val, dict):
            for subkey, subval in val.items():
                flat_aml[f"{key}_{subkey}"] = subval
        else:
            flat_aml[key] = val
    return {
        "depth": bar30_depth,
        "temp": bar30_temp,
        "pressure_mbar": bar30_mbar,
        "sal_calc": sal_psu,
        "aml": flat_aml,
    }

# Register service for blueos sidebar access.
@app.get("/register_service")
async def register_service():
    with open("/app/register_service", "r") as f:
        return JSONResponse(content=json.load(f))


# Serve index.html at root
@app.get("/")
async def root():
    return FileResponse("/app/index.html")


async def aml_parsing_loop():
    global text_backup, aml_values, sal_psu
    sen_dict = discover_devices()
    text_backup = setup_text_backup()
    text_line = ""
    # ------------------------- MAIN LOOP -----------------------------
    try:
        while True:
            text_line = datetime.now().strftime("%H:%M:%S")
            print(text_line)
            text_line += f",{bar30_depth:.2f},{bar30_temp:.2f}"

            for sen in sen_dict:
                if sen_dict[sen] is None:

                    text_line += f", {NO_DATA_VAL}"

                elif sen == "CT.X2":  # CT value handling  + Salinity Calc.
                    s_val = get_ct_nums(
                        sen
                    )  # Calculate Salinity (PSU) from Conductivity (mS/cm), Temp (deg C), P (bar).
                    aml_values["CT.X2"] = {"cond": s_val[0], "temp": s_val[1]}
                    sal_psu = round((
                        NO_DATA_VAL
                        if SENSOR_ERROR_VAL in s_val or s_val[0]<MINIMUM_SALT_WATER_COND
                        else SP_from_C(  # # SP_from_C( Conductivity(mS/cm), Temp (dgC), Pressure (dBar) )
                            [s_val[0]], [s_val[1]], [bar30_mbar * DECI_PER_MILLI]
                        )[0]
                    ),4)

                    print(f"CT: {s_val}, Sal(PSU): {sal_psu}")
                    text_line += f",{s_val[0]},{s_val[1]},{sal_psu}"

                else:  # Case single value sensor.
                    s_val = get_single_val(sen)
                    if s_val == SENSOR_ERROR_VAL:
                        print(f"Error found in sensor line {sen}. Removing sensor.")
                        SENSOR_DICT[sen] = None
                    elif s_val == NO_DATA_VAL:
                        print(f"No data received from {sen}. Removing sensor.")
                        SENSOR_DICT[sen] = None
                    else:
                        aml_values[sen] = s_val
                        text_line += f",{s_val}"
                        print(f"{sen}: {s_val}")

            print("\n")
            write_to_backup(text_backup, text_line)
            await asyncio.sleep(REFRESH_PERIOD_SECONDS)
    finally:
        if text_backup:
            text_backup.close()
        for s in SENSOR_DICT.values():
            if isinstance(s, serial.Serial) and s.is_open:
                s.close()


async def start_async_functions():
    # Kill ardupilot on startup — we don't need it
    try:
        r = requests.post("http://localhost/ardupilot-manager/v1.0/stop")
        print(f"Stopped ardupilot: {r.status_code}")
    except Exception as e:
        print(f"Ardupilot stop failed (may already be stopped): {e}")
    await asyncio.sleep(AUTOPILOT_SHUTDOWN_SECONDS) 
    # Initial setup for navigator RC switch.
    navigator.init()
    navigator.set_pwm_freq_hz(SERVO_PWM_FREQUENCY_HZ)
    navigator.set_pwm_channel_value(SENSOR_PWR_RELAY_CHANNEL, PWM_HIGH)
    navigator.set_pwm_channel_duty_cycle(SAMPLE_TRIGGER_CHANNEL, 1.0)
    navigator.set_pwm_channel_duty_cycle(LUMEN_CHANNEL, 0.0)
    navigator.set_pwm_enable(True)

    config = uvicorn.Config(app, host="0.0.0.0", port=9050, loop="asyncio")
    server = uvicorn.Server(config)

    depth_task = asyncio.create_task(depth_sensor_loop())
    aml_task = asyncio.create_task(aml_parsing_loop())

    try:
        await server.serve()          # returns when uvicorn catches SIGINT/SIGTERM
    finally:
        depth_task.cancel()
        aml_task.cancel()
        await asyncio.gather(depth_task, aml_task, return_exceptions=True)
        navigator.set_pwm_channel_duty_cycle(LUMEN_CHANNEL, RELAY_OFF)
        navigator.set_pwm_enable(False)   # leave the HAT in a known state


if __name__ == "__main__":
    asyncio.run(start_async_functions())
