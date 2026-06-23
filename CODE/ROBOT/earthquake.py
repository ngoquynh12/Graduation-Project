import time
import math
import mpu6050

# ----------------------------------------------------
# 2. Initialize MPU6050
# ----------------------------------------------------
mpu = mpu6050.mpu6050(0x68)
G = 9.80665   # 1g in m/s^2

# ----------------------------------------------------
# 3. Moving Average Filter (smooth vibrations)
# ----------------------------------------------------
def smooth(value, buffer, size=10):
    buffer.append(value)
    if len(buffer) > size:
        buffer.pop(0)
    return sum(buffer) / len(buffer)

ax_buf, ay_buf, az_buf = [], [], []


# ----------------------------------------------------
# 4. Calibration
# ----------------------------------------------------
def calibrate_accel(samples=200):
    print("Calibrating accelerometer (keep still)...")

    sum_x = sum_y = sum_z = 0
    for _ in range(samples):
        a = mpu.get_accel_data()
        sum_x += a['x']
        sum_y += a['y']
        sum_z += a['z']
        time.sleep(0.005)

    offset_x = sum_x / samples
    offset_y = sum_y / samples
    offset_z = (sum_z / samples) - G

    print("Calibration offsets:", offset_x, offset_y, offset_z)
    return offset_x, offset_y, offset_z

ox, oy, oz = calibrate_accel()


# ----------------------------------------------------
# 5. MMI Table
# ----------------------------------------------------
MMI_TABLE = [
    (0, 1, "I", "Instrumental"),
    (1, 2, "II", "Very Weak"),
    (2, 5, "III", "A Bit Weak"),
    (5, 10, "IV", "Weak"),
    (10, 25, "V", "Rather Powerful"),
    (25, 50, "VI", "Strong"),
    (50, 100, "VII", "Very Powerful"),
    (100, 250, "VIII", "Damage"),
    (250, 500, "IX", "Strong"),
    (500, 1000, "X", "Very Strong")
]

def pga_to_mmi(pga_gal):
    for low, high, mmi, cat in MMI_TABLE:
        if low <= pga_gal < high:
            return mmi, cat
    return "X+", "Extreme"


# ----------------------------------------------------
# 6. MAIN LOOP (Dynamic PGA)
# ----------------------------------------------------
print("\n--- Earthquake Detection + LoRa TX Started ---\n")

prev_a_total = None

while True:
    accel = mpu.get_accel_data()

    # Remove offsets ? convert to g
    ax = (accel['x'] - ox) / G
    ay = (accel['y'] - oy) / G
    az = (accel['z'] - oz) / G

    # Smoothing
    ax = smooth(ax, ax_buf)
    ay = smooth(ay, ay_buf)
    az = smooth(az, az_buf)

    # Magnitude
    a_total = math.sqrt(ax*ax + ay*ay + az*az)

    if prev_a_total is None:
        prev_a_total = a_total

    # Dynamic PGA
    dynamic_pga_g = abs(a_total - prev_a_total)
    prev_a_total = a_total

    # Convert to gal
    pga_gal = dynamic_pga_g * 980.665

    # Convert to MMI
    mmi, cat = pga_to_mmi(pga_gal)

    print(f"PGA: {pga_gal:.2f} gal | MMI: {mmi} - {cat}")

    time.sleep(0.2)
