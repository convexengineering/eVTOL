# Contains configuration-specific data
from gpkit import ureg

configurations = {}


configurations["Multirotor"] = {}
configurations["Multirotor"]["V_{cruise}"] = 50*ureg("mph")
configurations["Multirotor"]["L/D"] = 1.5
configurations["Multirotor"]["T/A"] = 3.75*ureg("lbf")/ureg("ft")**2


configurations["Autogyro"] = {}
configurations["Autogyro"]["V_{cruise}"] = 100*ureg("mph")
configurations["Autogyro"]["L/D"] = 3.5
configurations["Autogyro"]["T/A"] = 3.75*ureg("lbf")/ureg("ft")**2


configurations["Helicopter"] = {}
configurations["Helicopter"]["V_{cruise}"] = 100*ureg("mph")
configurations["Helicopter"]["L/D"] = 4.25
configurations["Helicopter"]["T/A"] = 4.5*ureg("lbf")/ureg("ft")**2


configurations["Tilt duct"] = {}
configurations["Tilt duct"]["V_{cruise}"] = 150*ureg("mph")
configurations["Tilt duct"]["L/D"] = 10.
configurations["Tilt duct"]["T/A"] = 40*ureg("lbf")/ureg("ft")**2


configurations["Coaxial heli"] = {}
configurations["Coaxial heli"]["V_{cruise}"] = 150*ureg("mph")
configurations["Coaxial heli"]["L/D"] = 5.5
configurations["Coaxial heli"]["T/A"] = 7*ureg("lbf")/ureg("ft")**2


configurations["Lift + cruise"] = {}
configurations["Lift + cruise"]["V_{cruise}"] = 150*ureg("mph")
configurations["Lift + cruise"]["L/D"] = 10
configurations["Lift + cruise"]["T/A"] = 15*ureg("lbf")/ureg("ft")**2


configurations["Tilt wing"] = {}
configurations["Tilt wing"]["V_{cruise}"] = 150*ureg("mph")
configurations["Tilt wing"]["L/D"] = 12
configurations["Tilt wing"]["T/A"] = 4.5*ureg("lbf")/ureg("ft")**2


configurations["Compound heli"] = {}
configurations["Compound heli"]["V_{cruise}"] = 150*ureg("mph")
configurations["Compound heli"]["L/D"] = 9
configurations["Compound heli"]["T/A"] = 7*ureg("lbf")/ureg("ft")**2


configurations["Tilt rotor"] = {}
configurations["Tilt rotor"]["V_{cruise}"] = 150*ureg("mph")
configurations["Tilt rotor"]["L/D"] = 14
configurations["Tilt rotor"]["T/A"] = 15*ureg("lbf")/ureg("ft")**2