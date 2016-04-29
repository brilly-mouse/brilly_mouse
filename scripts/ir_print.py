#!/usr/bin/env python
import Adafruit_BBIO.ADC as ADC
from time import sleep
ADC.setup()

def median(lst):
    quotient, remainder = divmod(len(lst), 2)
    if remainder:
        return sorted(lst)[quotient]
    return float(sum(sorted(lst)[quotient - 1:quotient + 1]) / 2)

def get_value(pin):
    return median([ADC.read(pin) for i in range(5)])

while True:
    r = get_value("P9_36")
    f = get_value("P9_38")
    l = get_value("P9_40")
    # print(r, f, l)
    print('Front IR:', round(2076 / (f * 736.56 - 11), 2))
    print('Left IR:', round(1 / (l * 0.4464 + 0.011) - 1.8, 2))
    print('Right IR:', round(1 / (r * 0.4464 + 0.011) - 1.8, 2))
    print('-')
    sleep(0.2)
