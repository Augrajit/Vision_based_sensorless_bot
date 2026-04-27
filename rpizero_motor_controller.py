"""
rpizero_motor_controller.py — Runs on Raspberry Pi Zero 2W (physically on the robot)
MQTT-driven motor controller with watchdog, exponential smoothing, and GPIO output.
"""

import time
import json
import signal
import sys
import threading

# ========================== CONSTANTS ==========================
MQTT_BROKER_IP = "[IP_ADDRESS]"   # UPDATE THIS to RPi 4 IP
HEARTBEAT_TIMEOUT_MS = 500
PWM_FREQ = 1000
SMOOTHING_FACTOR = 0.3
CONTROL_LOOP_HZ = 30

# GPIO Pin Assignments (BCM numbering, TB6612FNG driver)
PWMA_PIN = 12   # Left motor PWM (hardware PWM pin)
AIN1_PIN = 23
AIN2_PIN = 24
PWMB_PIN = 13   # Right motor PWM (hardware PWM pin)
BIN1_PIN = 27
BIN2_PIN = 22
STBY_PIN = 25

# MQTT Topics
CMD_TOPIC = "factory/robot/01/cmd"
HEARTBEAT_TOPIC = "factory/robot/01/heartbeat"
STATUS_TOPIC = "factory/robot/01/status"

# ========================== GLOBALS ==========================
last_heartbeat_time = time.time()
watchdog_fired = False
target_la = 0
target_ra = 0
current_la = 0.0
current_ra = 0.0
running = True

pwm_a = None
pwm_b = None


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")


def setup_gpio():
    """Initialize all GPIO pins and PWM objects."""
    global pwm_a, pwm_b
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    for pin in [AIN1_PIN, AIN2_PIN, BIN1_PIN, BIN2_PIN, STBY_PIN, PWMA_PIN, PWMB_PIN]:
        GPIO.setup(pin, GPIO.OUT)

    # Enable the TB6612 driver
    GPIO.output(STBY_PIN, GPIO.HIGH)

    # Start PWM on motor pins
    pwm_a = GPIO.PWM(PWMA_PIN, PWM_FREQ)
    pwm_b = GPIO.PWM(PWMB_PIN, PWM_FREQ)
    pwm_a.start(0)
    pwm_b.start(0)
    log("GPIO initialized. Motor driver STANDBY=HIGH.")


def set_motors(la, ra):
    """
    Set motor speeds. la/ra in range [-255, 255].
    Positive = forward, negative = reverse.
    """
    import RPi.GPIO as GPIO

    la = max(-255, min(255, int(la)))
    ra = max(-255, min(255, int(ra)))

    # Left motor (A)
    if la > 0:
        GPIO.output(AIN1_PIN, GPIO.HIGH)
        GPIO.output(AIN2_PIN, GPIO.LOW)
    elif la < 0:
        GPIO.output(AIN1_PIN, GPIO.LOW)
        GPIO.output(AIN2_PIN, GPIO.HIGH)
    else:
        GPIO.output(AIN1_PIN, GPIO.LOW)
        GPIO.output(AIN2_PIN, GPIO.LOW)
    pwm_a.ChangeDutyCycle(abs(la) / 255.0 * 100.0)

    # Right motor (B)
    if ra > 0:
        GPIO.output(BIN1_PIN, GPIO.HIGH)
        GPIO.output(BIN2_PIN, GPIO.LOW)
    elif ra < 0:
        GPIO.output(BIN1_PIN, GPIO.LOW)
        GPIO.output(BIN2_PIN, GPIO.HIGH)
    else:
        GPIO.output(BIN1_PIN, GPIO.LOW)
        GPIO.output(BIN2_PIN, GPIO.LOW)
    pwm_b.ChangeDutyCycle(abs(ra) / 255.0 * 100.0)


def on_cmd_message(client, userdata, msg):
    """Handle incoming motor command from navigation server."""
    global target_la, target_ra
    try:
        payload = json.loads(msg.payload.decode())
        target_la = int(payload.get("la", 0))
        target_ra = int(payload.get("ra", 0))
    except Exception as e:
        log(f"CMD parse error: {e}")


def on_heartbeat_message(client, userdata, msg):
    """Record heartbeat timestamp from navigation server."""
    global last_heartbeat_time, watchdog_fired
    last_heartbeat_time = time.time()
    if watchdog_fired:
        watchdog_fired = False
        log("Heartbeat resumed — watchdog cleared.")


def on_connect(client, userdata, flags, rc):
    """MQTT connect callback — subscribe to topics."""
    if rc == 0:
        log("Connected to MQTT broker.")
        client.subscribe(CMD_TOPIC, qos=1)
        client.subscribe(HEARTBEAT_TOPIC, qos=1)
        log(f"Subscribed to {CMD_TOPIC} and {HEARTBEAT_TOPIC}")
    else:
        log(f"MQTT connection failed with code {rc}")


def watchdog_thread_func(client):
    """Check for heartbeat timeout every 100 ms. Stop motors if lost."""
    global watchdog_fired, target_la, target_ra
    while running:
        elapsed = time.time() - last_heartbeat_time
        if elapsed > HEARTBEAT_TIMEOUT_MS / 1000.0:
            if not watchdog_fired:
                watchdog_fired = True
                target_la = 0
                target_ra = 0
                set_motors(0, 0)
                log("WATCHDOG: Heartbeat lost! Motors stopped.")
                try:
                    client.publish(STATUS_TOPIC,
                                   json.dumps({"state": "watchdog_stop"}),
                                   qos=1)
                except Exception as e:
                    log(f"WATCHDOG publish error: {e}")
        time.sleep(0.1)


def smoothing_thread_func():
    """30 Hz control loop — exponentially smooth toward target speeds."""
    global current_la, current_ra
    interval = 1.0 / CONTROL_LOOP_HZ
    while running:
        current_la += (target_la - current_la) * SMOOTHING_FACTOR
        current_ra += (target_ra - current_ra) * SMOOTHING_FACTOR

        # Snap to zero if very small
        if abs(current_la) < 0.5:
            current_la = 0.0
        if abs(current_ra) < 0.5:
            current_ra = 0.0

        set_motors(int(current_la), int(current_ra))
        time.sleep(interval)


def status_publisher_thread_func(client):
    """Publish robot status every 500 ms."""
    while running:
        try:
            if abs(current_la) < 1 and abs(current_ra) < 1:
                payload = json.dumps({"state": "stopped"})
            else:
                payload = json.dumps({
                    "state": "moving",
                    "la": int(current_la),
                    "ra": int(current_ra)
                })
            client.publish(STATUS_TOPIC, payload, qos=1)
        except Exception as e:
            log(f"Status publish error: {e}")
        time.sleep(0.5)


def shutdown(signum, frame):
    """Graceful shutdown on SIGINT / SIGTERM."""
    global running
    import RPi.GPIO as GPIO
    log(f"Shutdown signal received ({signum}). Cleaning up...")
    running = False
    set_motors(0, 0)
    time.sleep(0.1)
    GPIO.cleanup()
    log("GPIO cleaned up.")
    try:
        mqtt_client.disconnect()
        log("MQTT disconnected.")
    except Exception:
        pass
    sys.exit(0)


if __name__ == "__main__":
    import paho.mqtt.client as mqtt

    log("=" * 50)
    log("RPi Zero 2W Motor Controller Starting...")
    log("=" * 50)

    # --- GPIO Setup ---
    setup_gpio()

    # --- Signal Handlers ---
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # --- MQTT Setup ---
    mqtt_client = mqtt.Client(client_id="rpizero_motor_01")
    mqtt_client.on_connect = on_connect
    mqtt_client.message_callback_add(CMD_TOPIC, on_cmd_message)
    mqtt_client.message_callback_add(HEARTBEAT_TOPIC, on_heartbeat_message)

    log(f"Connecting to MQTT broker at {MQTT_BROKER_IP}:1883 ...")
    try:
        mqtt_client.connect(MQTT_BROKER_IP, 1883, keepalive=60)
    except Exception as e:
        log(f"FATAL: Cannot connect to MQTT broker: {e}")
        sys.exit(1)

    mqtt_client.loop_start()

    # --- Launch daemon threads ---
    wd_thread = threading.Thread(target=watchdog_thread_func, args=(mqtt_client,), daemon=True)
    wd_thread.start()
    log("Watchdog thread started (100 ms check interval).")

    smooth_thread = threading.Thread(target=smoothing_thread_func, daemon=True)
    smooth_thread.start()
    log(f"Smoothing thread started ({CONTROL_LOOP_HZ} Hz).")

    status_thread = threading.Thread(target=status_publisher_thread_func, args=(mqtt_client,), daemon=True)
    status_thread.start()
    log("Status publisher thread started (500 ms interval).")

    log("Motor controller ready. Waiting for commands...")

    # Keep main thread alive
    try:
        while running:
            time.sleep(1.0)
    except KeyboardInterrupt:
        shutdown(signal.SIGINT, None)
