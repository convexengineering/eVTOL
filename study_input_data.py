# Contains configuration-specific data
from gpkit import ureg
from collections import OrderedDict
from copy import deepcopy

generic_data = {} #generic data (i.e. for all configs)
configs_OutOfOrder = {} #configuration-specific data (not ordered)
configuration_data = OrderedDict() #configuration-specific data (in order)

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

configs_OutOfOrder["Multirotor"] = {}
configs_OutOfOrder["Multirotor"]["V_{cruise}"] = 50*ureg("mph")
configs_OutOfOrder["Multirotor"]["L/D"] = 1.5
configs_OutOfOrder["Multirotor"]["T/A"] = 3.75*ureg("lbf")/ureg("ft")**2
configs_OutOfOrder["Multirotor"]["Cl_{mean_{max}}"] = 0.6
configs_OutOfOrder["Multirotor"]["N"] = 4
configs_OutOfOrder["Multirotor"]["loiter_type"] = "hover"

configs_OutOfOrder["Autogyro"] = {}
configs_OutOfOrder["Autogyro"]["V_{cruise}"] = 100*ureg("mph")
configs_OutOfOrder["Autogyro"]["L/D"] = 3.5
configs_OutOfOrder["Autogyro"]["T/A"] = 3.75*ureg("lbf")/ureg("ft")**2
configs_OutOfOrder["Autogyro"]["Cl_{mean_{max}}"] = 0.8
configs_OutOfOrder["Autogyro"]["N"] = 1
configs_OutOfOrder["Autogyro"]["loiter_type"] = "level_flight"

configs_OutOfOrder["Helicopter"] = {}
configs_OutOfOrder["Helicopter"]["V_{cruise}"] = 100*ureg("mph")
configs_OutOfOrder["Helicopter"]["L/D"] = 4.25
configs_OutOfOrder["Helicopter"]["T/A"] = 4.5*ureg("lbf")/ureg("ft")**2
configs_OutOfOrder["Helicopter"]["Cl_{mean_{max}}"] = 0.6
configs_OutOfOrder["Helicopter"]["N"] = 1
configs_OutOfOrder["Helicopter"]["loiter_type"] = "hover"

configs_OutOfOrder["Tilt duct"] = {}
configs_OutOfOrder["Tilt duct"]["V_{cruise}"] = 150*ureg("mph")
configs_OutOfOrder["Tilt duct"]["L/D"] = 10.
configs_OutOfOrder["Tilt duct"]["T/A"] = 40*ureg("lbf")/ureg("ft")**2
configs_OutOfOrder["Tilt duct"]["Cl_{mean_{max}}"] = 1.0
configs_OutOfOrder["Tilt duct"]["N"] = 36
configs_OutOfOrder["Tilt duct"]["loiter_type"] = "level_flight"

configs_OutOfOrder["Coaxial heli"] = {}
configs_OutOfOrder["Coaxial heli"]["V_{cruise}"] = 150*ureg("mph")
configs_OutOfOrder["Coaxial heli"]["L/D"] = 5.5
configs_OutOfOrder["Coaxial heli"]["T/A"] = 7*ureg("lbf")/ureg("ft")**2
configs_OutOfOrder["Coaxial heli"]["Cl_{mean_{max}}"] = 0.6
configs_OutOfOrder["Coaxial heli"]["N"] = 2
configs_OutOfOrder["Coaxial heli"]["loiter_type"] = "hover"

configs_OutOfOrder["Lift + cruise"] = {}
configs_OutOfOrder["Lift + cruise"]["V_{cruise}"] = 150*ureg("mph")
configs_OutOfOrder["Lift + cruise"]["L/D"] = 10
configs_OutOfOrder["Lift + cruise"]["T/A"] = 15*ureg("lbf")/ureg("ft")**2
configs_OutOfOrder["Lift + cruise"]["Cl_{mean_{max}}"] = 1.0
configs_OutOfOrder["Lift + cruise"]["N"] = 8
configs_OutOfOrder["Lift + cruise"]["loiter_type"] = "level_flight"

configs_OutOfOrder["Tilt wing"] = {}
configs_OutOfOrder["Tilt wing"]["V_{cruise}"] = 150*ureg("mph")
configs_OutOfOrder["Tilt wing"]["L/D"] = 12
configs_OutOfOrder["Tilt wing"]["T/A"] = 15*ureg("lbf")/ureg("ft")**2
configs_OutOfOrder["Tilt wing"]["Cl_{mean_{max}}"] = 1.0
configs_OutOfOrder["Tilt wing"]["N"] = 8
configs_OutOfOrder["Tilt wing"]["loiter_type"] = "level_flight"

configs_OutOfOrder["Compound heli"] = {}
configs_OutOfOrder["Compound heli"]["V_{cruise}"] = 150*ureg("mph")
configs_OutOfOrder["Compound heli"]["L/D"] = 9
configs_OutOfOrder["Compound heli"]["T/A"] = 4.5*ureg("lbf")/ureg("ft")**2
configs_OutOfOrder["Compound heli"]["Cl_{mean_{max}}"] = 0.8
configs_OutOfOrder["Compound heli"]["N"] = 1
configs_OutOfOrder["Compound heli"]["loiter_type"] = "level_flight"

configs_OutOfOrder["Tilt rotor"] = {}
configs_OutOfOrder["Tilt rotor"]["V_{cruise}"] = 150*ureg("mph")
configs_OutOfOrder["Tilt rotor"]["L/D"] = 14
configs_OutOfOrder["Tilt rotor"]["T/A"] = 15*ureg("lbf")/ureg("ft")**2
configs_OutOfOrder["Tilt rotor"]["Cl_{mean_{max}}"] = 1.0
configs_OutOfOrder["Tilt rotor"]["N"] = 12
configs_OutOfOrder["Tilt rotor"]["loiter_type"] = "level_flight"

#Put configurations in desired order

configuration_data["Lift + cruise"] = deepcopy(configs_OutOfOrder["Lift + cruise"])
configuration_data["Compound heli"] = deepcopy(configs_OutOfOrder["Compound heli"])
configuration_data["Tilt wing"] = deepcopy(configs_OutOfOrder["Tilt wing"])
configuration_data["Tilt rotor"] = deepcopy(configs_OutOfOrder["Tilt rotor"])
configuration_data["Helicopter"] = deepcopy(configs_OutOfOrder["Helicopter"])
configuration_data["Coaxial heli"] = deepcopy(configs_OutOfOrder["Coaxial heli"])
configuration_data["Multirotor"] = deepcopy(configs_OutOfOrder["Multirotor"])
configuration_data["Autogyro"] = deepcopy(configs_OutOfOrder["Autogyro"])
configuration_data["Tilt duct"] = deepcopy(configs_OutOfOrder["Tilt duct"])
