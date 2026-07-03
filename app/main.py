from pymavlink import mavutil
from datetime import datetime
from SPFromC import SP_from_C
import bluerobotics_navigator as navigator
from fastapi import FastAPI
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
import json

import uvicorn
import asyncio
import time
import serial
import termios
import glob
import sys
import ms5837
import requests
import traceback

BOOT_TIME = time.time()

# -------------------------------HARDWARE PARAMS-------------------------------
SAMPLE_TRIGGER_CHANNEL = 3 # Physical port 2 on Tealas Sampler, 3 on DAL WS.
LUMEN_CHANNEL = 9 # PWM channel 10 for the lumen on Dal WS.

AML_PWR_RELAY_CHANNEL = 6 # Zero indexed, physical port 7.
RELAY_OFF = 0
RELAY_MID = 0.25
RELAY_ON = 1.0


SERVO_PWM_FREQUENCY_HZ = 50
PWM_LOW = 225  # PWM values not in microsecond, but as fraction of 4096 block to reach standard 1100, 1500, and 1900 periods.
PWM_MID = 307  # E.g. value = 4095 * pulse_duration (1500 us for mid pwm) / cycle_PERIOD_SECONDS( 1/50 Hz) = 307
PWM_HIGH = 389
# -------------------------------AML LOOP PARAMS-------------------------------
NO_DATA_VAL = -1
AML_ERROR_VAL = -2
TEXT_BACKUP_HEADER = "Time (local), BAR30-Depth (m), BAR30-Temp (°C), AML Cond (mS/cm), AML Temp (°C), PSU (Calulated), AML Chloro (μg/L), AML Rho (ppb), AML Turb (NTU),  AML DO (μmol/L)\n"
# -------------------------------PHYSICAL CONSTANTS-------------------------------
STANDARD_ATMOSPHERIC_PRESSURE_HPA = 1013.25
SALTWATER_DENSITY_KGM3 = 1023.6
MBAR_TO_DBAR = 0.01
MINIMUM_SALT_WATER_COND = 30 # mS/cm
# -------------------------------TIMING PARAMS-------------------------------
AUTOPILOT_SHUTDOWN_SECONDS = 30

AML_REFRESH_PERIOD_SECONDS = 1
AML_PRINT_PERIOD_SECONDS= 1
AML_REBOOT_TIME_SECONDS = (
    10  # Healthy amount of time from power to actually streaming data.
)
AML_CMD_WAIT_TIME_SECONDS = 1  # Give sensors time to respond to the cmd.
AML_RESPONSE_TIMEOUT_SECONDS = (
    5  # If nothing after this amount of time, nothings coming.
)

BAR30_REFRESH_PERIOD_SECONDS = 0.25
BAR30_REINIT_TIME_SECONDS = 5
BAR30_PRINT_PERIOD_SECONDS= 1.0

RELAY_PAUSE_TIME_SECONDS = 2
# -------------------------------SENSOR COMM PARAMS-------------------------------
AML_SENSOR_DICT = {
    "CT.X2": None,
    "Chloro-blue": None,
    "Rhodamine": None,
    "Turbidity": None,
    "Dissolved Oxygen": None,
}
AML_BAUD_RATE = 9600
MS5837_BUS = 6
# -------------------------------FUNCTIONS-------------------------------
def power_cycle_sensors():  # Power cycles the sensors on defined port to trip RC Switch to high.
    print("Powering off sensors.")
    navigator.set_pwm_channel_value(AML_PWR_RELAY_CHANNEL, PWM_LOW)
    time.sleep(AML_REBOOT_TIME_SECONDS)
    print("Powering on sensors.")
    navigator.set_pwm_channel_value(AML_PWR_RELAY_CHANNEL, PWM_HIGH)
    time.sleep(AML_REBOOT_TIME_SECONDS)


def discover_devices():
    # Reset AML_SENSOR_DICT before rediscovery so stale entries don't persist.
    for key in AML_SENSOR_DICT:
        AML_SENSOR_DICT[key] = None

    start_time = time.time()

    while time.time() - start_time < AML_RESPONSE_TIMEOUT_SECONDS:
        usb_devices = glob.glob("/dev/ttyUSB*")
        if usb_devices:
            break
        time.sleep(AML_CMD_WAIT_TIME_SECONDS)

    if not usb_devices:
        print(
            f"No USB devices detected after {AML_RESPONSE_TIMEOUT_SECONDS} seconds. Exiting."
        )
        sys.exit(1)

    for dev in usb_devices:
        try:
            print(f"Probing {dev}...")
            ser = serial.Serial(
                dev, AML_BAUD_RATE, timeout=AML_CMD_WAIT_TIME_SECONDS
            )
            time.sleep(AML_CMD_WAIT_TIME_SECONDS)
            ser.write(b"\r")
            time.sleep(AML_CMD_WAIT_TIME_SECONDS)
            ser.write(b"display options\r")
            option_display_text = ""
            start_time = time.time()
            sensor_found = False
            while time.time() - start_time < AML_RESPONSE_TIMEOUT_SECONDS:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                print(line)
                option_display_text += line
                if not line:
                    continue
                for s_name in AML_SENSOR_DICT:
                    if AML_SENSOR_DICT[s_name] is None and s_name in line:
                        print(f"{s_name} found on <{dev}>.")
                        AML_SENSOR_DICT[s_name] = ser
                        sensor_found = True

                if ">" in line:
                    print("Writing out display options.")
                    diagnotic_file_name = f"/usr/blueos/userdata/sensorData/{datetime.now().date()}-diag.txt"
                    diagnotic_file = open(diagnotic_file_name, "a")
                    diagnotic_file.write(f"{datetime.now().strftime('%H:%M:%S')}, {s_name} found on <{dev}>.")
                    diagnotic_file.write(option_display_text)
                    diagnotic_file.close()

            if not sensor_found:
                print(f"No matching sensor on {dev}. Closed.")
                ser.close()

        except serial.SerialException as e:
            print(f"Failed to connect to {dev}: {e}")

    time.sleep(10)
    print("Breaking out of discover.")
    return AML_SENSOR_DICT


def get_sensor_line(ser_num):
    if not ser_num or not ser_num.is_open:
        print("Attempted to read from closed or invalid serial port.")
        return ""
    try:
        ser_num.flushInput()
        ser_num.flushOutput()
        ser_num.readline()  # discard partial/incomplete line.
        return ser_num.readline().decode("utf-8", errors="ignore").strip()
    except (serial.SerialException, termios.error) as e:
        print(f"Serial error during read: {e}")
        try:
            ser_num.close()
        except Exception:
            pass
        return ""


def get_ct_nums(sen):
    sensor_line = get_sensor_line(AML_SENSOR_DICT[sen])
    # print(f"[CT DEBUG] raw line: {sensor_line!r}")  # debug print.
    split_line = sensor_line.split()
    if len(split_line) < 2:
        print(f"[CT DEBUG] too few fields ({len(split_line)}): {split_line}")
        return [AML_ERROR_VAL, AML_ERROR_VAL]
    try:
        return float(split_line[0]), round(float(split_line[1]), 2)
    except ValueError as e:
        print(f"[CT DEBUG] float conversion failed: {e} on {split_line}")
        return [AML_ERROR_VAL, AML_ERROR_VAL]


def get_single_val(sen):
    sensor_line = get_sensor_line(AML_SENSOR_DICT[sen])
    try:
        sensor_val = int(float(sensor_line))
    except ValueError:
        print(f"{sen} bad line -> {sensor_line!r}")
        sensor_val = AML_ERROR_VAL
    # print(f"[get_single_val DEBUG:] raw line: {sensor_val !r}")  # debug print.
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
bar30_depth_m = 0.0  # Global variables to pass between different loops.
bar30_depth_offset_m = 0.0
bar30_temp_c = 0.0
bar30_pressure_mbar = 0.0
bar30_pressure_offset_mbar = 0.0

calc_sal_psu = 0.0

aml_values = {
    "CT.X2": {"ct_cond_mscm": -1.0, "ct_temp_degc": -1.0},
    "Chloro-blue": -1,
    "Rhodamine": -1,
    "Turbidity": -1,
    "Dissolved Oxygen": -1,
}
text_backup = None
bar30_zeroed = False


async def depth_sensor_loop():
    global bar30_depth_m, bar30_temp_c, bar30_pressure_mbar, bar30_zeroed
    print("Staring BAR loop.",flush = True)
    bar30 = None
    while bar30 is None:
        try:
            
            b = ms5837.MS5837_30BA(bus=MS5837_BUS)
            b.setFluidDensity(
                SALTWATER_DENSITY_KGM3
            )  # Set density for seawater, if using freshwater comment out or change.
            b.init()
            bar30 = b
            print("BAR30 init successful.", flush = True)
        except Exception as e:
            print(f"BAR30 Init failed, retrying in 5s: {e}", flush =True)
            await asyncio.sleep(BAR30_REINIT_TIME_SECONDS)
    last_print_time = 0
    if(bar30_zeroed == False and bar30.read()): #Grab offset on first run.
        bar30_depth_reading_m = round(bar30.depth(), 2)
        bar30_depth_m_offset = bar30_depth_reading_m if bar30_depth_reading_m < 2.0 else 0
        bar30_pressure_reading_mbar = round(bar30.pressure(), 2)
        bar30_pressure_mbar_offset = bar30_pressure_reading_mbar if abs(bar30_pressure_reading_mbar - STANDARD_ATMOSPHERIC_PRESSURE_HPA) < 500 else 0
        bar30_zeroed = True
        print(f"BAR30 Depth Offset: {bar30_depth_m_offset:.2f} m")
        print(f"BAR30 Pressure Offset: {bar30_pressure_mbar_offset:.2f} mbar")
    while True:
        try: 
            bar30.read()
            bar30_depth_m = round(bar30.depth(), 2) - bar30_depth_m_offset
            bar30_temp_c = round(bar30.temperature(), 2)
            bar30_pressure_mbar = round(bar30.pressure(), 3) - bar30_pressure_mbar_offset
            if time.time() - last_print_time >= BAR30_PRINT_PERIOD_SECONDS:
                print(f"Current Depth: {bar30_depth_m:.2f} m", flush = True)
                print(f"Current Temperature: {bar30_temp_c:.2f} °C", flush = True)
                last_print_time = time.time()
        except Exception as e:
            print(f"Failed to read BAR30: {e}", flush=True)  
            try: #Try resetting the BAR30
                bar30 = ms5837.MS5837_30BA(bus=MS5837_BUS)
                bar30.setFluidDensity(SALTWATER_DENSITY_KGM3)
                bar30.init()
                bar30_zeroed = False  # Force re-zero on recovery
                print("BAR30 re-initialized after error", flush=True)
            except Exception as reinit_e:
                print(f"BAR30 re-init failed: {reinit_e}", flush=True)
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
    print("Triggering sample.")
    navigator.set_pwm_channel_duty_cycle(SAMPLE_TRIGGER_CHANNEL, RELAY_OFF)
    await asyncio.sleep(RELAY_PAUSE_TIME_SECONDS)  # non-blocking
    navigator.set_pwm_channel_duty_cycle(SAMPLE_TRIGGER_CHANNEL, RELAY_ON)

    if text_backup:
        timestamp = datetime.now().strftime("%H:%M:%S")
        text_backup.write(f"{timestamp}, SAMPLE TRIGGERED, depth={bar30_depth_m:.2f}, temp={bar30_temp_c:.2f}\n")
        text_backup.flush()

    return {"status": "done", "depth": bar30_depth_m, "temp": bar30_temp_c}


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
        "depth": bar30_depth_m,
        "temp": bar30_temp_c,
        "pressure_mbar": bar30_pressure_mbar,
        "sal_calc": calc_sal_psu,
        "aml": flat_aml,
    }


# Serve the register_service endpoint for BlueOS sidebar integration
@app.get("/register_service")
async def register_service():
    with open("/app/static/register_service", "r") as f:
        return JSONResponse(content=json.load(f))

# Serve index.html at root
@app.get("/")
async def root():
    return FileResponse("/app/static/index.html")


async def aml_parsing_loop():
    global text_backup, aml_values, calc_sal_psu
    print("AML parsing loop started.")
    try:
        sen_dict = discover_devices()
    except Exception as e:
        print(f"Device discovery failed: {e}")
        traceback.print_exc()
    try:
        text_backup = setup_text_backup()
    except Exception as e:
        print(f"Textbackup opening failed: {e}")
    print("Backup opened.")
    text_line = ""
    last_print_time = 0
    try:
        while True:
            try:
                # If all sensors have gone None, USB likely disconnected — wait and rediscover.
                if all(sen_dict[s] is None for s in sen_dict):
                    print("[AML] All sensors lost, waiting for USB reconnect...")
                    await asyncio.sleep(AML_REBOOT_TIME_SECONDS)
                    sen_dict = discover_devices()

                text_line = datetime.now().strftime("%H:%M:%S")
                print(text_line)
                text_line += f",{bar30_depth_m:.2f},{bar30_temp_c:.2f}"

                for sen in sen_dict:
                    if sen_dict[sen] is None:
                        text_line += f", {NO_DATA_VAL}"

                    elif sen == "CT.X2":
                        ct_cond_mscm, ct_temp_degc = get_ct_nums(sen)
                        # If CT read failed, mark disconnected to trigger rediscovery.
                        if ct_cond_mscm == AML_ERROR_VAL:
                            print("[AML] CT.X2 read failed, marking as disconnected.")
                            AML_SENSOR_DICT[sen] = None
                            sen_dict[sen] = None
                            text_line += f", {AML_ERROR_VAL}"
                            continue
                        aml_values["CT.X2"] = {"ct_cond_mscm": ct_cond_mscm, "ct_temp_degc": ct_temp_degc}
                        calc_sal_psu = (
                            NO_DATA_VAL
                            if ct_cond_mscm == AML_ERROR_VAL or ct_temp_degc == AML_ERROR_VAL or ct_cond_mscm < MINIMUM_SALT_WATER_COND
                            # SP_from_C( Conductivity(mS/cm), Temp (dgC), Pressure (dBar))
                            else round(SP_from_C([ct_cond_mscm], [ct_temp_degc], [bar30_pressure_mbar * MBAR_TO_DBAR])[0],3)  
                        )
                        if time.time() - last_print_time >= AML_PRINT_PERIOD_SECONDS:
                            print(f"CT: {ct_cond_mscm}, {ct_temp_degc}, Sal(PSU): {calc_sal_psu}", flush = True)
                            last_print_time = time.time()
                        text_line += f",{ct_cond_mscm},{ct_temp_degc},{calc_sal_psu}"

                    else:
                        s_val = get_single_val(sen)
                        if s_val == AML_ERROR_VAL:
                            print(f"Error found in sensor line {sen}. Removing sensor.")
                            AML_SENSOR_DICT[sen] = None
                            sen_dict[sen] = None
                        elif s_val == NO_DATA_VAL:
                            print(f"No data received from {sen}. Removing sensor.")
                            AML_SENSOR_DICT[sen] = None
                            sen_dict[sen] = None
                        else:
                            aml_values[sen] = s_val
                            text_line += f",{s_val}"
                            print(f"{sen}: {s_val}")

                write_to_backup(text_backup, text_line)

            except Exception as e:
                print(f"[AML LOOP ERROR] {e}")
                traceback.print_exc()

            await asyncio.sleep(AML_REFRESH_PERIOD_SECONDS)  # always runs, even after an error

    finally:
        if text_backup:
            text_backup.close()
        for s in AML_SENSOR_DICT.values():
            if isinstance(s, serial.Serial) and s.is_open:
                s.close()


async def start_async_functions():
    # Kill ardupilot on startup — we don't need it
    try:
        r = requests.post("http://localhost/ardupilot-manager/v1.0/stop")
        print(f"Stopped ardupilot: {r.status_code}")
    except Exception as e:
        print(f"Ardupilot stop failed (may already be stopped): {e}")
    await asyncio.sleep(AUTOPILOT_SHUTDOWN_SECONDS) #Todo: keep checking if autopilot is shut down before proceding.
    # Initial setup for navigator RC switch.
    navigator.init()
    navigator.set_pwm_freq_hz(SERVO_PWM_FREQUENCY_HZ)
    navigator.set_pwm_channel_value(AML_PWR_RELAY_CHANNEL, PWM_HIGH)
    navigator.set_pwm_channel_duty_cycle(SAMPLE_TRIGGER_CHANNEL, 1.0)
    navigator.set_pwm_enable(True)

    config = uvicorn.Config(app, host="0.0.0.0", port=9050, loop="asyncio")
    server = uvicorn.Server(config)

    depth_task = asyncio.create_task(depth_sensor_loop())
    aml_task = asyncio.create_task(aml_parsing_loop())

    def task_error_handler(task):
        try:
            if not task.cancelled() and task.exception():
                print(f"[TASK DIED] {task.get_name()}: {task.exception()}", flush=True)
                traceback.print_exception(type(task.exception()), task.exception(), task.exception().__traceback__)
        except Exception as e:
            print(f"[TASK CALLBACK ERROR] {e}", flush=True)

    depth_task.set_name("depth_sensor_loop")
    aml_task.set_name("aml_parsing_loop")
    depth_task.add_done_callback(task_error_handler)
    aml_task.add_done_callback(task_error_handler)

    try:
        await server.serve()
    finally:
        depth_task.cancel()
        aml_task.cancel()
        await asyncio.wait_for(asyncio.gather(depth_task, aml_task, return_exceptions=True), timeout=5.0)
        navigator.set_pwm_enable(False)


if __name__ == "__main__":
    asyncio.run(start_async_functions())
