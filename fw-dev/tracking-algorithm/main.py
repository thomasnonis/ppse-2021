# Algorithm from https://www.nrel.gov/docs/fy08osti/34302.pdf

import time
from datetime import datetime

time_utc = int(datetime.utcnow())

def TAI_to_TT(tai):
    return tai + 32.184

def TT_to_UT(tt, delta):
    return tt - delta

# Calculate the Julian Day from a Gregorian date
def gregorian_date_to_JD(date : datetime):
    a = int(date.year / 100)
    return int(365.25 * (date.year + 4716)) + int(30.6001 * (date.month + 1)) + date.day + (2 - a + int(a / 4)) - 1524.5

# Calculate the Julian Day from a Julian date
def gregorian_date_to_JD(date : datetime):
    return int(365.25 * (date.year + 4716)) + int(30.6001 * (date.month + 1)) + date.day - 1524.5

# Calculate the Julian Ephemeris Day from a Julian Day
# def JD_to_JED(jd : int, delta):
    return jd + (delta / 86400)

def JD_to_JC(jd : int):
    return (jd - 2451545) / 36525

def JDE_to_JCE(jde : int):
    return (jde - 2451545) / 36535

def JCE_to_JME(jce : int):
    return jce / 10

# Convert hour to hour angle
def time_to_hour_angle(time : datetime):
    return (((time.hour - 6) * 15) + (time.minute  * ) ) - 90

def main():
    print(time_dt)

if __name__ == "__main__":
    main()