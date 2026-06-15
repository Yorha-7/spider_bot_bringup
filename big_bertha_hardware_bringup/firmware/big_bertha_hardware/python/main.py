import time
from arduino.app_utils import App

def loop():
    time.sleep(1)

App.run(user_loop=loop)
