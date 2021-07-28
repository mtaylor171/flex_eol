import time
import pigpio
import sys

class reader:

    def __init__(self, pi, gpio, pwm, pulses_per_rev = 1.0, weighting = 0.0, min_RPM = 5.0):

        self.pi = pi
        self.gpio = gpio
        self.pwm = pwm
        self.pulses_per_rev = pulses_per_rev
        self.rpm_data = []

        if min_RPM > 1000.0:
            min_RPM = 1000.0
        elif min_RPM < 1.0:
            min_RPM = 1.0

        self.min_RPM = min_RPM
        self._watchdog = 200 	# milliseconds

        if weighting < 0.0:
            weighting = 0.0
        elif weighting > 0.99:
            weighting = 0.99

        self._new = 1.0 - weighting
        self._old = weighting

        self._high_tick = None
        self._period = None

        pi.set_mode(gpio, pigpio.INPUT)

        self._cb = pi.callback(gpio, pigpio.RISING_EDGE, self._cbf)
        pi.set_watchdog(gpio, self._watchdog)

    def _cbf(self, gpio, level, tick):
        if level == 1:

            if self._high_tick is not None:
                t = pigpio.tickDiff(self._high_tick, tick)

                if self._period is not None:
                    self._period = (self._old * self._period) + (self._new * t)

                else:
                    self._period = t

            self._high_tick = tick

        elif level == 2:

            if self._period is not None:
                if self._period  < 2000000000:
                    self._period += (self._watchdog * 1000)
        
    def PWM(self, duty):
        self.pi.hardware_PWM(self.pwm, 25000, duty * 10000)
        
    def RPM(self):

        RPM = 0.0
        if self._period is not None:
            RPM = 60000000.0 / (self._period * self.pulses_per_rev)
            if RPM < self.min_RPM:
                RPM = 0.0
        return RPM

    def calc_rpm(self):
        temp_sum = 0
        if(len(self.rpm_data) ==0):
            return 0
        else:
            for i in range(0, len(self.rpm_data)):
                temp_sum += self.rpm_data[i]
            return ((temp_sum/2)/(len(self.rpm_data) - 1))



    def cancel(self):
        self.pi.hardware_PWM(self.pwm, 25000, 0)
        self.pi.set_watchdog(self.gpio, 0)
        self._cb.cancel()

def message_display(msg, desired_answer):
    while(1):
        if input(msg).lower() == desired_answer:
            return 1
        else:
            print('\033c')
            print("*****************************")
            print("Incorrect character entered.")
            print("*****************************")
            return 0

def main():

    RPM_GPIO = 4
    PWM_GPIO = 19
    RUN_TIME = 200
    DUTY = 95
    #RUN_TIME = int(input("Enter Duration: "))
    #DUTY = int(input("Enter Duty Cycle %: "))
    SAMPLE_TIME = 0.5

    print('\033c')
    print("*****************************")
    print("\nNURO FAN TESTING\n")
    print("This test will run the fan at 95 percent for 300s")
    print("To stop the test at anytime, hold 'CTRL + C'\n")
    print("*****************************\n")

    while(message_display("To begin testing, press '1' and ENTER: ", '1') != 1):
        pass

    pi = pigpio.pi()

    p = fan_main.reader(pi, RPM_GPIO, PWM_GPIO)
    
    p.PWM(DUTY)

    start = time.time()

    while (time.time() - start) < RUN_TIME:
        try:
        
            time.sleep(SAMPLE_TIME)

            RPM = p.RPM()
            if((time.time() - start) > 30):
                p.rpm_data.append(RPM)

            print('\033c')
            print("Time: {} ".format(round(time.time() - start), 1) + "RPM = {}".format(int(RPM+0.5)/2) + " (Press CTRL + C to STOP")
        
        except KeyboardInterrupt:
            print("*****************************")
            print("\nTest Cancelled\n")
            print("*****************************")
            p.cancel()
            rpm_avg = p.calc_rpm()
            print(f"Average RPM of Test: {rpm_avg}")
            return 0
        
        finally:
            pass
            
    print("*****************************")
    print("\nTest Duration Reached\n")
    print("\nThis program will restart 3 seconds...\n")
    print("*****************************")
    p.cancel()
    rpm_avg = p.calc_rpm()
    print(f"Average RPM of Test: {rpm_avg}")
    return 0

if __name__ == "__main__":
    
    import time
    import pigpio
    import fan_main
    
    while(1):
        main()
        while(message_display("To continue, press '2' and ENTER: ", '2') != 1):
            pass
