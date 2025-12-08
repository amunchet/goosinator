from gpiozero import LED
from time import sleep

def main():
    pin = LED(25)      # BCM numbering
    pin.on()           # drive HIGH
    sleep(0.25)
    pin.off()          # drive LOW

if __name__ == "__main__":
    main()
