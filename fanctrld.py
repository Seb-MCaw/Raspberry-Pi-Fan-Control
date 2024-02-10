#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#----------------------------------------------------------------------------------------------#
# Controls fan PWM depending on cpu/gpu temperature                                            #
#                                                                                              #
# - Config file at specified location specifies cooling profile for given time                 #
# - Specified cooling profiles give FAN% for a given temperature                               #
# - Specified fan scaling profile gives PWM duty cycle for given FAN%                          #
# - Main function runs a loop:                                                                 #
#       - checks temp and updates PWM every UPDATE_FAN_INTERVAL seconds                        #
#       - logs temp, FAN% and PWM duty cycle to specified file every LOG_INTERVAL seconds      #
#       - checks config file every CONFIG_UPDATE_INTERVAL seconds                              #
#----------------------------------------------------------------------------------------------#




import math
import re
import time
import datetime
import logging, logging.handlers
import atexit
import signal
import sys


# Setup GPIO
import pigpio # pigpio library  -  pigpiod daemon provides necessary root access for gpio
pi = pigpio.pi()




#-----Config-----#

# Location of profile config file
CONFIG_FILE_PATH = "/etc/fanctrl"
# Desired location logfile
LOG_FILE_PATH = "/var/log/fanctrl.log"

# The path at which to find the current CPU temperature
TEMPERATURE_PATH = "/sys/class/thermal/thermal_zone0/temp"


# Time in seconds betweeon fan speed updates
UPDATE_FAN_INTERVAL = 1
# Time in seconds between entries in log file
LOG_INTERVAL = 300
# Time interval in seconds at which to check the config file
CONFIG_UPDATE_INTERVAL = 3600


# BCM pin used to drive transistor's base
FAN_CTRL_PIN = 18
# Freq (Hz) of hardware PWM to use for fan control
FAN_PWM_FREQ = 25000

# A characteristic time (secs) for the exponential tendency towards the desired FAN%.
# The difference between the actual value and target value will halve in this time.
FAN_CHANGE_CHARACTERISTIC_TIME = 4
# 0 FAN% turns the fan off. After being turned on from this state, the fan will
# not turn off again until the temperature drops FAN_OFF_TEMP_HYSTERESIS
# degrees below the turn-on trigger.
FAN_OFF_TEMP_HYSTERESIS = 5


# Fan scaling profile to translate FAN% into PWM duty cycle (list of (FAN%, Duty cycle) tuples).
# Intermediate values are linearly interpolated.
# A FAN% of exactly 0 is a special case, translating automatically into a duty
# cycle of 0 regardless of scaling profile.
#
# This profile is for the Pi Fan with a 100uF capacitor at 25kHz PWM.
FAN_PROFILE = [(0, .096), (25, .12), (50, .15), (62.5, .19), (75, .26), (87.5, .45), (100, 1)]


# Cooling profiles to determine FAN% (0-100) for a given temperature (list of
# (degrees C, FAN%) tuples)).
#
# The entry with the lowest temperature also specifies the FAN% for all temperatures
# below it, and likewise for those above the highest temperature entry.
COOLING_PROFILES = {
	# Fan always off
	"Silent": [(0, 0)],
	# Fan off below 55, on at lowest speed above 55
	"VeryQuiet": [(55, 0), (56, .1)],
	# Fan off below 45, minimum up to 55, then ramps gradually up to 50% at 65
	"Quiet": [(45, 0), (46, .1), (55, .1), (65, 50)],
	# Minimum for 40-50, then ramp up to 100% at 70
	"Balanced": [(40, 0), (41, .1), (50, .1), (70, 100)],
	# Turn on at 55, suddenly ramp up above 75 to prevent any throttling
	"AggressiveThrottlingPrevention": [(55, 0), (56, .1), (75, .1), (80, 1)],
	# Attempts to keep temperature below 60 at the most
	"Max": [(35, 0), (36, .1), (40, 50), (50, 70), (60, 100)],
	# Fan on at 100% at all times
	"AlwaysFull": [(0, 100)]
}

# Reserve profile config to fall back on if a config file can't be found
DEFAULT_DAY_PROFILE = "Max"
DEFAULT_NIGHT_PROFILE = "VeryQuiet"
DEFAULT_NIGHT_HOURS = "22:30-10:00"

#----/Config-----#




def linearly_interpolate_from_profile(profile, input):
	"""
	Linearly interpolate profiles (either cooling or fan scaling) to get
	an output value corresponding to input.
	"""
	# Sort profile by ascending input values (if necessary)
	profile.sort(key=lambda x: x[0])

	# If input exactly equals multiple input values in the profile, return the
	# average of their outputs
	matching_outputs = [x[1] for x in profile if x[0] == input]
	if len(matching_outputs) > 0:
		return sum(matching_outputs) / len(matching_outputs)
	# If input is below the profile's first input value, return the first output value
	elif input < profile[0][0]:
		return profile[0][1]
	# Likewise, if input is above last input value, return the last output value
	elif input >= profile[-1][0]:
		return profile[-1][1]
	# Otherwise, linearly interpolate between values
	else:
		for prv, nxt in zip(profile, profile[1:]):
			if prv[0] <= input < nxt[0]:
				return (
					prv[1]
					+ (input - prv[0]) * (nxt[1] - prv[1]) / (nxt[0] - prv[0])
				)



class FanCtrl:
	"""An object to represent the current state of the fan and control system."""

	# The last measured temperature (in Celsius)
	current_temperature = 30
	# The current FAN% as last actually set to the fan (not the same as the
	# desired FAN%)
	#
	# Takes the literal value "OFF" in the special case when fan is actually off
	current_fan_percent = "OFF"
	# The last duty cycle set to GPIO
	current_PWM_duty_cycle = 0
	# The cooling profile currently in use
	current_cooling_profile = "Silent"
	# The next cooling profile to switch to
	next_cooling_profile = "Silent"
	# The datetime at which to switch current_cooling_profile to next_cooling_profile
	profile_switch_time = datetime.datetime.now()


	def __init__(self):
		pass

	def _set_fan_PWM(self, duty_cycle):
		"""Set the fan's PWM duty cycle to duty_cycle (between 0 and 1)."""
		if self.current_PWM_duty_cycle != duty_cycle:
			pi.hardware_PWM(FAN_CTRL_PIN, FAN_PWM_FREQ, int(1000000 * duty_cycle))
			self.current_PWM_duty_cycle = duty_cycle

	def _set_fan_percent(self, fan_percent, treat_zero_as_off=True):
		"""
		Set the fan at a FAN% of fan_percent (between 0 and 100).

		Turns the fan off at 0% unless treat_zero_as_off=False.
		"""
		if treat_zero_as_off and (fan_percent == 0):
			self._set_fan_PWM(0)
			self.current_fan_percent = "OFF"
		else:
			self._set_fan_PWM(linearly_interpolate_from_profile(FAN_PROFILE, fan_percent))
			self.current_fan_percent = fan_percent

	def _get_processor_temp(self):
		"""Return the current CPU/GPU temperature as a float in celsius."""
		temperature_file = open(TEMPERATURE_PATH, "r")
		temperature = float(temperature_file.read()) / 1000
		temperature_file.close()
		self.current_temperature = temperature
		return temperature


	def update_from_config(self):
		"""Update profile settings from the config file."""
		try:
			# Read the file into a list of lines
			config_lines = [line.rstrip("\n") for line in open(CONFIG_FILE_PATH)]
			# Split each line on whitespace (giving list of lists)
			config_lines = [line.split() for line in config_lines]
		except FileNotFoundError:
			# Use hard-coded defaults if config file can't be found
			logging.error("Could not find config file - using default config")
			config_lines = []

		# Initially set default strings
		day_profile = DEFAULT_DAY_PROFILE
		night_profile = DEFAULT_NIGHT_PROFILE
		(night_start_str, night_end_str) = DEFAULT_NIGHT_HOURS.split('-')

		# Pattern match valid lines and update strings accordingly
		night_hours_pattern = re.compile(
			"^([0-9]|0[0-9]|1[0-9]|2[0-3]):[0-5][0-9]-([0-9]|0[0-9]|1[0-9]|2[0-3]):[0-5][0-9]$"
		)
		for line in config_lines:
			if len(line) == 2:
				if (line[0] == "DayProfile") and (line[1] in COOLING_PROFILES):
					day_profile = line[1]
				elif (line[0] == "NightProfile") and (line[1] in COOLING_PROFILES):
					night_profile = line[1]
				elif (line[0] == "NightHours") and night_hours_pattern.match(line[1]):
					(night_start_str, night_end_str) = line[1].split('-')

		# Get night_start and night_end times for today
		now = datetime.datetime.now()
		rplce_args = {
			"day": now.day,
			"month": now.month,
			"year": now.year,
			"second": 0,
			"microsecond": 0
		}
		strptime = datetime.datetime.strptime
		night_start_today = strptime(night_start_str, "%H:%M").replace(**rplce_args)
		night_end_today = strptime(night_end_str, "%H:%M").replace(**rplce_args)

		# Determine whether or not it's currently nighttime and when that will next
		# change, and update self accordingly.
		if (
			(night_end_today <= now <= night_start_today)
			or (now <= night_start_today <= night_end_today)
		):
			# Currently daytime and changes at night_start_today
			self.current_cooling_profile = day_profile
			self.next_cooling_profile = night_profile
			self.profile_switch_time = night_start_today
		elif (
			(night_start_today <= now <= night_end_today)
			or (now <= night_end_today <= night_start_today)
		):
			# Currently nighttime and changes at night_end_today
			self.current_cooling_profile = night_profile
			self.next_cooling_profile = day_profile
			self.profile_switch_time = night_end_today
		elif (night_start_today <= night_end_today <= now):
			# Currently daytime and changes at the night start time tomorrow
			self.current_cooling_profile = day_profile
			self.next_cooling_profile = night_profile
			self.profile_switch_time = night_start_today + datetime.timedelta(days=1)
		else: # i.e. if (night_end_today <= night_start_today <= now)
			# Currently nighttime and changes at the night end time tomorrow
			self.current_cooling_profile = night_profile
			self.next_cooling_profile = day_profile
			self.profile_switch_time = night_end_today + datetime.timedelta(days=1)

	def update_fan_setting(self, change_rate_factor):
		"""
		Check current temperature and set fan according to self.current_cooling_profile.

		change_rate_factor is the multiplicative factor by which to change
		the difference between the current and target fan percents (to
		give an exponential convergence).
		"""
		# Change cooling profile if it's time to do so
		if datetime.datetime.now() >= self.profile_switch_time:
			self.current_cooling_profile = self.next_cooling_profile

		# Determine the FAN% given by the cooling profile
		temperature = self._get_processor_temp()
		profile_fan_percent = linearly_interpolate_from_profile(
			COOLING_PROFILES[self.current_cooling_profile],
			temperature
		)

		# Determine the FAN% to actually set, based on change_rate_factor.
		if self.current_fan_percent == "OFF":
			current_fan_percent = 0
		else:
			current_fan_percent = self.current_fan_percent
		fan_percent_to_set = (
			  current_fan_percent * change_rate_factor
			+ profile_fan_percent * (1 - change_rate_factor)
		)

		# If the profile calls for 0%, round any fan_percent_to_set less than 1%
		# to 0%, so that turning the fan off doesn't require waiting for the
		# exponential decay to reach exactly 0.
		if (profile_fan_percent == 0) and (fan_percent_to_set < 1):
			fan_percent_to_set = 0

		# Account for ON/OFF hysteresis
		treat_zero_as_off = True
		if (profile_fan_percent == 0) and (self.current_PWM_duty_cycle > 0):
			# If the fan is still running but the profile asks for the fan
			# to be off, find the lowest temp above temperature (if any)
			# where the fan turns on.
			sorted_profile = sorted(
				COOLING_PROFILES[self.current_cooling_profile],
				key=lambda x: x[0]
			)
			next_nonzero_fan_idx = next(
				(
					i for i, p in enumerate(sorted_profile)
					if (temperature <= p[0]) and (p[1] > 0)
				),
				None
			)
			if next_nonzero_fan_idx is not None and next_nonzero_fan_idx > 0:
				turn_on_temp = sorted_profile[next_nonzero_fan_idx-1][0]
				# Keep the fan on unless the current temperature is below
				# the turn-off temperature for the specified hysteresis
				treat_zero_as_off = (temperature + FAN_OFF_TEMP_HYSTERESIS <= turn_on_temp)
			else:
				# This is a situation other than that for which the hysteresis
				# is intended
				treat_zero_as_off = True

		# Finally, set fan percent as above.
		self._set_fan_percent(fan_percent_to_set, treat_zero_as_off)

	def log_current_state(self):
		"""
		Add a line to the log file detailing current profile, temperature, FAN% and duty cycle
		"""
		if self.current_fan_percent == "OFF":
			fan_perc = "  OFF"
		else:
			fan_perc = f"{self.current_fan_percent:05.2f}"
		logging.info(
			  f"Current fan control state:  "
			+ f"duty cycle = {self.current_PWM_duty_cycle:.4f}, "
			+ f"FAN% = {fan_perc}, "
			+ f"temperature = {self.current_temperature:04.1f}\N{DEGREE SIGN}C, "
			+ f"current profile = {self.current_cooling_profile}"
		)

	def startup(self):
		"""Perform startup procedure"""
		logging.info("Beginning startup procedure")

		# Update from the config file
		self.update_from_config()

		# Run fan at full for 5 secs to provide audible confirmation of initialisation
		logging.info("Running fan at full to audibly confirm initialisation")
		self._set_fan_percent(100)
		time.sleep(5)

		# Set fan to correct speed and log successful startup
		self.update_fan_setting(1e-6)
		logging.info("Startup procedure complete")


	def main_loop(self):
		"""Run the main control loop"""
		# First determine parameters necessary for looping (perform all calculations
		# in integer multiples of loop_period to avoid out-by-one errors that skip cycles).
		#
		# Seconds to wait between each iteration of the loop (millisecond resolution)
		millisec = 1e-3
		loop_period = millisec * math.gcd(
			math.gcd(
				int(UPDATE_FAN_INTERVAL / millisec),
				int(LOG_INTERVAL / millisec)
			),
			int(CONFIG_UPDATE_INTERVAL / millisec)
		)
		# The number of loops between each action
		update_fan_interval = int(UPDATE_FAN_INTERVAL / loop_period)
		log_interval = int(LOG_INTERVAL / loop_period)
		config_update_interval = int(CONFIG_UPDATE_INTERVAL / loop_period)

		# Determine initial loop_counter such that timings are round numbers
		# (ie hourly things happen on the dot of xx:00:00, etc)
		#
		# This doesn't account for daylight savings, so will only work properly
		# year-round for integer factors of 1 hour
		one_day = 24 * 60 * 60
		reference_time = int(time.time() / one_day) * one_day
		loop_counter = int((time.time() - reference_time) / loop_period)
		# Also precalculate the change rate factor for update_fan_setting()
		cr_factor = 2**(-update_fan_interval*loop_period / FAN_CHANGE_CHARACTERISTIC_TIME)

		# Then loop through the procedure indefinitely
		while True:
			if loop_counter % config_update_interval == 0:
				self.update_from_config()
			if loop_counter % update_fan_interval == 0:
				self.update_fan_setting(cr_factor)
			if loop_counter % log_interval == 0:
				self.log_current_state()
			# Sleep between iterations as required to preserve synchronisation
			# with wall-clock time
			loop_counter += 1
			time.sleep(reference_time + loop_counter*loop_period - time.time())

	def cleanup(self):
		# Turn fan off
		self._set_fan_percent(0)



#Create an instance of FanCtrl to use hereafter
Fan = FanCtrl()



def exit_handler():
	"""Handler to exit gracefully on close"""
	# Clean up FanCtrl object
	Fan.cleanup()

	# Log successful shutdown and close logging system
	logging.info("Graceful shutdown complete - terminating log\n" + "-"*100 + "\n\n")
	logging.shutdown()

def SIGTERM_handler(sig, frame):
	"""Signal handler to initiate a graceful exit when SIGTERM is received"""
	logging.info("SIGTERM received - attempting graceful shutdown")
	sys.exit()




if __name__ == "__main__":
	# Register exit_handler
	atexit.register(exit_handler)
	signal.signal(signal.SIGTERM, SIGTERM_handler)

	# Set up logging but use default (console output) if logfile not found.
	# Logs rotate at 00:00:00 every Sunday.
	try:
		logging.basicConfig(
			format="%(asctime)s\t%(levelname)s:\t%(message)s",
			datefmt="%Y/%m/%d %H:%M:%S",
			level=logging.DEBUG,
			handlers=[logging.handlers.TimedRotatingFileHandler(
				LOG_FILE_PATH,
				when="W6",
				backupCount=4,
				atTime=datetime.time(0, 0, 0)
			)]
		)
	except FileNotFoundError:
		logging.warning("Log file not found - reverting to default console output")
	logging.info("FanCtrl Started")

	try:
		# Start FanCtrl
		Fan.startup()
		# Start the main loop
		Fan.main_loop()

	# Ensure that any error (aside from SystemExit and KeyboardInterrupt) is
	# properly logged (and attempt cleanup)
	except Exception:
		# Log the error
		logging.critical("An unhandled exception occured:", exc_info=True)
		# Try to shutdown safely
		try:
			logging.info("Attempting safe shutdown")
			exit_handler()
		except:
			logging.critical("Safe shutdown failed:", exc_info=True)
		# Then re-raise the exception
		raise
