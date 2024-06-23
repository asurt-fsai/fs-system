import time
from helpers.intervalTimer import IntervalTimer

# Define a simple worker function
def worker_function():
    print(f"Worker function called at: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}")

# Set the interval (in seconds) for the timer
interval = 2  # 2 seconds

# Create an instance of the IntervalTimer
timer = IntervalTimer(interval, worker_function)

# Start the timer
print("Starting timer...")
timer.start()

# Let the timer run for a specified duration (e.g., 10 seconds)
test_duration = 10  # 10 seconds
time.sleep(test_duration)

# Stop the timer
print("Stopping timer...")
timer.stop()

print("Timer stopped.")

