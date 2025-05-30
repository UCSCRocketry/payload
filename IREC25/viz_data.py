import matplotlib.pyplot as plt
import re

# === CONFIGURATION ===
LOG_FILE = 'data/LOG_8.txt'

# === PARSE FUNCTION ===
def parse_log_file(filename):
    data = {
        'euler_x': [],
        'euler_y': [],
        'euler_z': [],
        'enc_raw': [],
        'enc_deg': [],
        'servo_pwm': []
    }

    with open(filename, 'r') as file:
        for line in file:
            if "IMU =>" not in line:
                continue

            match = re.search(
                r"X:\s*(-?\d+\.\d+)\s+Y:\s*(-?\d+\.\d+)\s+Z:\s*(-?\d+\.\d+).*?ENC:\s*(\d+)\s+\(([\d.]+)\s+deg\).*?PWM:\s*(\d+)",
                line
            )
            if match:
                euler_x, euler_y, euler_z = map(float, match.group(1, 2, 3))
                enc_raw = int(match.group(4))
                enc_deg = float(match.group(5))
                pwm = int(match.group(6))

                data['euler_x'].append(euler_x)
                data['euler_y'].append(euler_y)
                data['euler_z'].append(euler_z)
                data['enc_raw'].append(enc_raw)
                data['enc_deg'].append(enc_deg)
                data['servo_pwm'].append(pwm)
    return data

# === PLOT FUNCTION ===
def plot_data(data):
    time = list(range(len(data['euler_x'])))

    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(time, data['euler_x'], label='Euler X')
    plt.plot(time, data['euler_y'], label='Euler Y')
    plt.plot(time, data['euler_z'], label='Euler Z')
    plt.title('IMU Euler Angles')
    plt.ylabel('Degrees')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(time, data['enc_deg'], color='orange', label='Encoder Angle (deg)')
    plt.title('Encoder Angle')
    plt.ylabel('Degrees')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(time, data['servo_pwm'], color='green', label='Servo PWM')
    plt.title('Servo PWM Output')
    plt.xlabel('Time Step (x100ms)')
    plt.ylabel('PWM')
    plt.legend()

    plt.tight_layout()
    plt.show()

# === RUN ===
if __name__ == '__main__':
    data = parse_log_file(LOG_FILE)
    if data['euler_x']:
        plot_data(data)
    else:
        print("No data found or failed to parse log file.")
