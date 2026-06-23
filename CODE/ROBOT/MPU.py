import time
import math
import smbus

# ==========================
# MPU6050 CONFIG
# ==========================
MPU_ADDR = 0x68
bus = smbus.SMBus(1)

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B

ACCEL_SCALE = 16384.0   # ±2g
DT = 0.1
CALIB_SAMPLES = 200

ALPHA = 0.95
NOISE_FLOOR_GAL = 3.0

# baseline tự cập nhật khi robot đứng yên
BASELINE_UPDATE_TH_GAL = 2.0
BASELINE_ALPHA = 0.995

# ==========================
# GLOBAL
# ==========================
accel_offset = [0.0, 0.0, 0.0]

filtered_ax = 0.0
filtered_ay = 0.0
filtered_az = 0.0

baseline_g = 1.0

# ==========================
# MMI TABLE
# ==========================
MMI_TABLE = [
    (0, 1, "I", "Instrumental"),
    (1, 2, "II", "Very Weak"),
    (2, 5, "III", "Weak"),
    (5, 10, "IV", "Light"),
    (10, 25, "V", "Moderate"),
    (25, 50, "VI", "Strong"),
    (50, 100, "VII", "Very Strong"),
    (100, 250, "VIII", "Severe"),
]

def pga_to_mmi(pga):
    for lo, hi, lvl, name in MMI_TABLE:
        if lo <= pga < hi:
            return lvl, name
    return "IX+", "Extreme"

# ==========================
# MPU FUNCTIONS
# ==========================
def mpu_write(reg, value):
    bus.write_byte_data(MPU_ADDR, reg, value)

def mpu_read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg + 1)

    value = (high << 8) | low

    if value >= 0x8000:
        value = -((65535 - value) + 1)

    return value

def mpu_init():
    print("[MPU] Initializing MPU6050...")

    mpu_write(PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    # Sample rate = 100Hz
    mpu_write(SMPLRT_DIV, 0x09)

    # DLPF low-pass filter
    mpu_write(CONFIG, 0x04)

    # ±2g
    mpu_write(ACCEL_CONFIG, 0x00)

    print("[MPU] Ready.")

def read_raw_accel():
    raw_ax = mpu_read_word(ACCEL_XOUT_H)
    raw_ay = mpu_read_word(ACCEL_XOUT_H + 2)
    raw_az = mpu_read_word(ACCEL_XOUT_H + 4)
    return raw_ax, raw_ay, raw_az

def convert_accel(raw_ax, raw_ay, raw_az):
    ax = raw_ax / ACCEL_SCALE
    ay = raw_ay / ACCEL_SCALE
    az = raw_az / ACCEL_SCALE
    return ax, ay, az

# ==========================
# CALIBRATION
# ==========================
def calibrate_mpu():
    global accel_offset

    print("[MPU] Calibrating... Keep sensor still!")

    sx = sy = sz = 0.0

    for _ in range(CALIB_SAMPLES):
        raw_ax, raw_ay, raw_az = read_raw_accel()
        ax, ay, az = convert_accel(raw_ax, raw_ay, raw_az)

        sx += ax
        sy += ay
        sz += az

        time.sleep(DT)

    accel_offset[0] = sx / CALIB_SAMPLES
    accel_offset[1] = sy / CALIB_SAMPLES
    accel_offset[2] = (sz / CALIB_SAMPLES) - 1.0

    print("[MPU] Calibration done.")
    print(
        f"[MPU] Offset "
        f"ax={accel_offset[0]:.4f}, "
        f"ay={accel_offset[1]:.4f}, "
        f"az={accel_offset[2]:.4f}"
    )

# ==========================
# FILTER
# ==========================
def low_pass(new_value, old_value):
    return ALPHA * old_value + (1.0 - ALPHA) * new_value

# ==========================
# MAIN LOOP
# ==========================
def vibration_loop():
    global filtered_ax, filtered_ay, filtered_az
    global baseline_g

    mpu_init()
    calibrate_mpu()

    # Khởi tạo filter bằng giá trị thật sau calibration
    raw_ax, raw_ay, raw_az = read_raw_accel()
    ax, ay, az = convert_accel(raw_ax, raw_ay, raw_az)

    ax -= accel_offset[0]
    ay -= accel_offset[1]
    az -= accel_offset[2]

    filtered_ax = ax
    filtered_ay = ay
    filtered_az = az

    # Mốc đứng yên ban đầu
    baseline_g = math.sqrt(
        filtered_ax * filtered_ax +
        filtered_ay * filtered_ay +
        filtered_az * filtered_az
    )

    print(f"[MPU] Initial baseline = {baseline_g:.4f} g")
    print("[MPU] Start PGA/MMI monitoring...")

    while True:
        try:
            raw_ax, raw_ay, raw_az = read_raw_accel()
            ax, ay, az = convert_accel(raw_ax, raw_ay, raw_az)

            ax -= accel_offset[0]
            ay -= accel_offset[1]
            az -= accel_offset[2]

            filtered_ax = low_pass(ax, filtered_ax)
            filtered_ay = low_pass(ay, filtered_ay)
            filtered_az = low_pass(az, filtered_az)

            total_accel = math.sqrt(
                filtered_ax * filtered_ax +
                filtered_ay * filtered_ay +
                filtered_az * filtered_az
            )

            # So với baseline_g thay vì cố định 1.0g
            dynamic_g = abs(total_accel - baseline_g)

            pga_value = dynamic_g * 980.665

            if pga_value < NOISE_FLOOR_GAL:
                pga_value = 0.0

            # Khi rung rất nhỏ thì cập nhật lại mốc đứng yên
            if pga_value < BASELINE_UPDATE_TH_GAL:
                baseline_g = BASELINE_ALPHA * baseline_g + (1.0 - BASELINE_ALPHA) * total_accel

            mmi_level, mmi_category = pga_to_mmi(pga_value)

            print(
                f"[VIBRATION] "
                f"PGA={pga_value:.2f} gal | "
                f"MMI={mmi_level} - {mmi_category} | "
                f"baseline={baseline_g:.4f} g"
            )

            time.sleep(DT)

        except KeyboardInterrupt:
            print("\n[MPU] Stopped.")
            break

        except Exception as e:
            print("[MPU ERROR]", e)
            time.sleep(0.5)

# ==========================
# RUN
# ==========================
if __name__ == "__main__":
    vibration_loop()