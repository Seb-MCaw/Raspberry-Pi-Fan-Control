Raspberry Pi Fan Control
========================

A simple self-contained script to control the fan of a Raspberry Pi based on
CPU/GPU temperature (both located on the main chip). It requires a control
output for the fan (via an NPN transistor or similar) on a PWM capable pin,
such that a high output turns the fan on.

The locations of accompanying log and config files are specified within the
script. The config file selects one of a number of preprogrammed fan profiles
for both daytime and nighttime, as well as specifying the timespan which
counts as nighttime. For example:

	DayProfile     MaxLifespan
	NightProfile   VeryQuiet
	NightHours     22:30-10:00
