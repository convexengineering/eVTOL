# Contains configuration-specific data
from gpkit import ureg

generic_data = {} #generic data (i.e. for all configs)
configuration_data = {} #configuration-specific data

generic_data["\eta_{cruise}"] = 0.85 #propulsive efficiency in cruise
generic_data["\eta_{electric}"] = 0.9 #electrical system efficiency
generic_data["weight_fraction"] = 0.55 #structural mass fraction
generic_data["C_m"] = 400*ureg.Wh/ureg.kg #battery energy density
generic_data["n"] = 1.0 #battery discharge parameter

generic_data["reserve_type"] = "FAA_day" #30-minute loiter
generic_data["autonomousEnabled"] = True #Vehicle autonomy enabled
generic_data["charger_power"] = 200*ureg.kW

generic_data["vehicle_cost_per_weight"] = 350*ureg.lbf**-1
generic_data["battery_cost_per_C"] = 400*ureg.kWh**-1
generic_data["pilot_wrap_rate"] = 70*ureg.hr**-1
generic_data["mechanic_wrap_rate"] = 60*ureg.hr**-1
generic_data["MMH_FH"] = 0.6
generic_data["deadhead_ratio"] = 0.2

generic_data["sizing_mission"] = {}
generic_data["sizing_mission"]["type"] = "piloted"
generic_data["sizing_mission"]["N_passengers"] = 3
generic_data["sizing_mission"]["range"] = 50*ureg.nautical_mile
generic_data["sizing_mission"]["t_{hover}"] = 120*ureg.s

generic_data["revenue_mission"] = {}
generic_data["revenue_mission"]["type"] = "piloted"
generic_data["revenue_mission"]["N_passengers"] = 2
generic_data["revenue_mission"]["range"] = 30*ureg.nautical_mile
generic_data["revenue_mission"]["t_{hover}"] = 30*ureg.s

generic_data["deadhead_mission"] = {}
generic_data["deadhead_mission"]["type"] = "autonomous"
generic_data["deadhead_mission"]["N_passengers"] = 0.00001
generic_data["deadhead_mission"]["range"] = 30*ureg.nautical_mile
generic_data["deadhead_mission"]["t_{hover}"] = 30*ureg.s

configuration_data["Multirotor"] = {}
configuration_data["Multirotor"]["V_{cruise}"] = 50*ureg("mph")
configuration_data["Multirotor"]["L/D"] = 1.5
configuration_data["Multirotor"]["T/A"] = 3.75*ureg("lbf")/ureg("ft")**2
configuration_data["Multirotor"]["Cl_{mean_{max}}"] = 0.6
configuration_data["Multirotor"]["N"] = 4
configuration_data["Multirotor"]["loiter_type"] = "hover"

configuration_data["Autogyro"] = {}
configuration_data["Autogyro"]["V_{cruise}"] = 100*ureg("mph")
configuration_data["Autogyro"]["L/D"] = 3.5
configuration_data["Autogyro"]["T/A"] = 3.75*ureg("lbf")/ureg("ft")**2
configuration_data["Autogyro"]["Cl_{mean_{max}}"] = 0.8
configuration_data["Autogyro"]["N"] = 1
configuration_data["Autogyro"]["loiter_type"] = "level_flight"

configuration_data["Helicopter"] = {}
configuration_data["Helicopter"]["V_{cruise}"] = 100*ureg("mph")
configuration_data["Helicopter"]["L/D"] = 4.25
configuration_data["Helicopter"]["T/A"] = 4.5*ureg("lbf")/ureg("ft")**2
configuration_data["Helicopter"]["Cl_{mean_{max}}"] = 0.6
configuration_data["Helicopter"]["N"] = 1
configuration_data["Helicopter"]["loiter_type"] = "hover"

configuration_data["Tilt duct"] = {}
configuration_data["Tilt duct"]["V_{cruise}"] = 150*ureg("mph")
configuration_data["Tilt duct"]["L/D"] = 10.
configuration_data["Tilt duct"]["T/A"] = 40*ureg("lbf")/ureg("ft")**2
configuration_data["Tilt duct"]["Cl_{mean_{max}}"] = 1.0
configuration_data["Tilt duct"]["N"] = 36
configuration_data["Tilt duct"]["loiter_type"] = "level_flight"

configuration_data["Coaxial heli"] = {}
configuration_data["Coaxial heli"]["V_{cruise}"] = 150*ureg("mph")
configuration_data["Coaxial heli"]["L/D"] = 5.5
configuration_data["Coaxial heli"]["T/A"] = 7*ureg("lbf")/ureg("ft")**2
configuration_data["Coaxial heli"]["Cl_{mean_{max}}"] = 0.6
configuration_data["Coaxial heli"]["N"] = 2
configuration_data["Coaxial heli"]["loiter_type"] = "hover"

configuration_data["Lift + cruise"] = {}
configuration_data["Lift + cruise"]["V_{cruise}"] = 150*ureg("mph")
configuration_data["Lift + cruise"]["L/D"] = 10
configuration_data["Lift + cruise"]["T/A"] = 15*ureg("lbf")/ureg("ft")**2
configuration_data["Lift + cruise"]["Cl_{mean_{max}}"] = 1.0
configuration_data["Lift + cruise"]["N"] = 8
configuration_data["Lift + cruise"]["loiter_type"] = "level_flight"

configuration_data["Tilt wing"] = {}
configuration_data["Tilt wing"]["V_{cruise}"] = 150*ureg("mph")
configuration_data["Tilt wing"]["L/D"] = 12
configuration_data["Tilt wing"]["T/A"] = 15*ureg("lbf")/ureg("ft")**2
configuration_data["Tilt wing"]["Cl_{mean_{max}}"] = 1.0
configuration_data["Tilt wing"]["N"] = 8
configuration_data["Tilt wing"]["loiter_type"] = "level_flight"

configuration_data["Compound heli"] = {}
configuration_data["Compound heli"]["V_{cruise}"] = 150*ureg("mph")
configuration_data["Compound heli"]["L/D"] = 9
configuration_data["Compound heli"]["T/A"] = 4.5*ureg("lbf")/ureg("ft")**2
configuration_data["Compound heli"]["Cl_{mean_{max}}"] = 0.8
configuration_data["Compound heli"]["N"] = 1
configuration_data["Compound heli"]["loiter_type"] = "level_flight"

configuration_data["Tilt rotor"] = {}
configuration_data["Tilt rotor"]["V_{cruise}"] = 150*ureg("mph")
configuration_data["Tilt rotor"]["L/D"] = 14
configuration_data["Tilt rotor"]["T/A"] = 15*ureg("lbf")/ureg("ft")**2
configuration_data["Tilt rotor"]["Cl_{mean_{max}}"] = 1.0
configuration_data["Tilt rotor"]["N"] = 12
configuration_data["Tilt rotor"]["loiter_type"] = "level_flight"