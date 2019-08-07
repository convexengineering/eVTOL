#Vehicle configuration top-level trade study

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/' + '..'))

import numpy as np
from gpkit import Model, ureg
from matplotlib import pyplot as plt
from aircraft_models import OnDemandAircraft 
from aircraft_models import OnDemandSizingMission, OnDemandRevenueMission
from aircraft_models import OnDemandDeadheadMission, OnDemandMissionCost
from study_input_data import generic_data, configuration_data
from noise_models import vortex_noise


B = generic_data["B"]
delta_S = generic_data["delta_S"]

# Delete some configurations
configs = configuration_data.copy()
del configs["Tilt duct"]
del configs["Multirotor"]
del configs["Autogyro"]
del configs["Helicopter"]
del configs["Coaxial heli"]

#Optimize remaining configurations
for config in configs:
	
	print "Solving configuration: " + config

	c = configs[config]

	problem_subDict = {}
	
	Aircraft = OnDemandAircraft(autonomousEnabled=generic_data["autonomousEnabled"])
	problem_subDict.update({
		Aircraft.L_D_cruise: c["L/D"], #estimated L/D in cruise
		Aircraft.eta_cruise: generic_data["\eta_{cruise}"], #propulsive efficiency in cruise
		Aircraft.tailRotor_power_fraction_hover: c["tailRotor_power_fraction_hover"],
		Aircraft.tailRotor_power_fraction_levelFlight: c["tailRotor_power_fraction_levelFlight"],
		Aircraft.cost_per_weight: generic_data["vehicle_cost_per_weight"], #vehicle cost per unit empty weight
		Aircraft.battery.C_m: generic_data["C_m"], #battery energy density
		Aircraft.battery.cost_per_C: generic_data["battery_cost_per_C"], #battery cost per unit energy capacity
		Aircraft.rotors.N: c["N"], #number of propellers
		Aircraft.rotors.Cl_mean_max: c["Cl_{mean_{max}}"], #maximum allowed mean lift coefficient
		Aircraft.structure.weight_fraction: c["weight_fraction"], #empty weight fraction
		Aircraft.electricalSystem.eta: generic_data["\eta_{electric}"], #electrical system efficiency	
	})

	SizingMission = OnDemandSizingMission(Aircraft,mission_type=generic_data["sizing_mission"]["type"],
		reserve_type=generic_data["reserve_type"])
	problem_subDict.update({
		SizingMission.mission_range: generic_data["sizing_mission"]["range"],#mission range
		SizingMission.V_cruise: c["V_{cruise}"],#cruising speed
		SizingMission.t_hover: generic_data["sizing_mission"]["t_{hover}"],#hover time
		SizingMission.T_A: c["T/A"],#disk loading
		SizingMission.passengers.N_passengers: generic_data["sizing_mission"]["N_{passengers}"],#Number of passengers
	})

	RevenueMission = OnDemandRevenueMission(Aircraft,mission_type=generic_data["revenue_mission"]["type"])
	problem_subDict.update({
		RevenueMission.mission_range: generic_data["revenue_mission"]["range"],#mission range
		RevenueMission.V_cruise: c["V_{cruise}"],#cruising speed
		RevenueMission.t_hover: generic_data["revenue_mission"]["t_{hover}"],#hover time
		RevenueMission.passengers.N_passengers: generic_data["revenue_mission"]["N_{passengers}"],#Number of passengers
		RevenueMission.time_on_ground.charger_power: generic_data["charger_power"], #Charger power
	})

	DeadheadMission = OnDemandDeadheadMission(Aircraft,mission_type=generic_data["deadhead_mission"]["type"])
	problem_subDict.update({
		DeadheadMission.mission_range: generic_data["deadhead_mission"]["range"],#mission range
		DeadheadMission.V_cruise: c["V_{cruise}"],#cruising speed
		DeadheadMission.t_hover: generic_data["deadhead_mission"]["t_{hover}"],#hover time
		DeadheadMission.passengers.N_passengers: generic_data["deadhead_mission"]["N_{passengers}"],#Number of passengers
		DeadheadMission.time_on_ground.charger_power: generic_data["charger_power"], #Charger power
	})

	MissionCost = OnDemandMissionCost(Aircraft,RevenueMission,DeadheadMission)
	problem_subDict.update({
		MissionCost.revenue_mission_costs.operating_expenses.pilot_cost.wrap_rate: generic_data["pilot_wrap_rate"],#pilot wrap rate
		MissionCost.revenue_mission_costs.operating_expenses.maintenance_cost.wrap_rate: generic_data["mechanic_wrap_rate"], #mechanic wrap rate
		MissionCost.revenue_mission_costs.operating_expenses.maintenance_cost.MMH_FH: generic_data["MMH_FH"], #maintenance man-hours per flight hour
		MissionCost.deadhead_mission_costs.operating_expenses.pilot_cost.wrap_rate: generic_data["pilot_wrap_rate"],#pilot wrap rate
		MissionCost.deadhead_mission_costs.operating_expenses.maintenance_cost.wrap_rate: generic_data["mechanic_wrap_rate"], #mechanic wrap rate
		MissionCost.deadhead_mission_costs.operating_expenses.maintenance_cost.MMH_FH: generic_data["MMH_FH"], #maintenance man-hours per flight hour
		MissionCost.deadhead_ratio: generic_data["deadhead_ratio"], #deadhead ratio
	})

	problem = Model(MissionCost["cost_per_trip"],
		[Aircraft, SizingMission, RevenueMission, DeadheadMission, MissionCost])
	problem.substitutions.update(problem_subDict)
	solution = problem.solve(verbosity=0)
	configs[config]["solution"] = solution
	
	#Noise computations
	T_perRotor = solution("T_perRotor_OnDemandSizingMission")[0]
	Q_perRotor = solution("Q_perRotor_OnDemandSizingMission")[0]
	R = solution("R")
	VT = solution("VT_OnDemandSizingMission")[0]
	s = solution("s")
	Cl_mean = solution("Cl_{mean_{max}}")
	N = solution("N")

	B = generic_data["B"]
	delta_S = generic_data["delta_S"]

	#Unweighted
	f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
		Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
		weighting="None")
	configs[config]["SPL"] = SPL
	configs[config]["f_{peak}"] = f_peak

	#A-weighted
	f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
		Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
		weighting="A")
	configs[config]["SPL_A"] = SPL

# solution = configs["Lift + cruise"]["solution"]
# print "Lift + cruise: L/D = %0.1f, T/A = %0.1f lbf/ft^2, cptpp = $%0.2f" \
# 	% (solution("L_D_cruise"), solution("T/A_OnDemandSizingMission").to(ureg.lbf/ureg.ft**2).magnitude, solution("cost_per_trip_per_passenger"))


# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(12, 11), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

y_pos = np.arange(len(configs))
labels = [""]*len(configs)
for i, config in enumerate(configs):
	if config == "Compound heli":
		labels[i] = config.replace(" ", "\n")  # Replace spaces with newlines
	else:
		labels[i] = config


style = {}
style["rotation"]         = -45
style["legend_ncols"]     = 2
style["bar_width_wide"]   = 0.7
style["bar_width_medium"] = 0.3
style["bar_width_narrow"] = 0.2
style["offsets"]          = [-0.25, 0, 0.25]
style["colors"]           = ["grey", "w", "k", "lightgrey"]

style["fontsize"] = {}
style["fontsize"]["xticks"]     = 14
style["fontsize"]["yticks"]     = 14
style["fontsize"]["xlabel"]     = 18
style["fontsize"]["ylabel"]     = 16
style["fontsize"]["title"]      = 16
style["fontsize"]["legend"]     = 11
style["fontsize"]["text_label"] = 18


# Mass breakdown
plt.subplot(3,3,1)
for i, config in enumerate(configs):
	
	m_empty     = configs[config]["solution"]("m_OnDemandAircraft/Structure").to(ureg.kg).magnitude
	m_battery   = configs[config]["solution"]("m_OnDemandAircraft/Battery").to(ureg.kg).magnitude
	TOGM        = configs[config]["solution"]("TOGM_OnDemandAircraft").to(ureg.kg).magnitude
	m_remainder = TOGM - m_empty - m_battery

	if i==0:
		plt.bar(i, m_empty,     align='center', bottom=0,                 alpha=1, color=style["colors"][0], edgecolor='k', label="Empty")
		plt.bar(i, m_battery,   align='center', bottom=m_empty,           alpha=1, color=style["colors"][1], edgecolor='k', label="Battery")
		plt.bar(i, m_remainder, align='center', bottom=m_empty+m_battery, alpha=1, color=style["colors"][2], edgecolor='k', label="Crew & passengers")
	else:
		plt.bar(i, m_empty,     align='center', bottom=0,                 alpha=1, color=style["colors"][0], edgecolor='k')
		plt.bar(i, m_battery,   align='center', bottom=m_empty,           alpha=1, color=style["colors"][1], edgecolor='k')
		plt.bar(i, m_remainder, align='center', bottom=m_empty+m_battery, alpha=1, color=style["colors"][2], edgecolor='k')

plt.grid()
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.7*ymax)
plt.xticks(y_pos, labels,     fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                   fontsize=style["fontsize"]["yticks"])
plt.ylabel('Mass (kg)',       fontsize=style["fontsize"]["ylabel"])
plt.title("Mass Breakdown",   fontsize=style["fontsize"]["title"])
plt.legend(loc="upper right", fontsize=style["fontsize"]["legend"], framealpha=1)


# Cost per seat km
plt.subplot(3,3,2)
for i, config in enumerate(configs):
	
	cpsk = configs[config]["solution"]("cost_per_seat_mile_OnDemandMissionCost").to(ureg.km**-1).magnitude
	plt.bar(i, cpsk, align='center', alpha=1, color='k', edgecolor='k')
		
plt.grid()
plt.xticks(y_pos, labels,             fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                           fontsize=style["fontsize"]["yticks"])
plt.ylabel('Cost ($US/km)',           fontsize=style["fontsize"]["ylabel"])
plt.title("Cost per Seat Kilometer",  fontsize=style["fontsize"]["title"])


# Sound pressure level (in hover) 
plt.subplot(3,3,3)

for i, config in enumerate(configs):
	SPL_sizing   = configs[config]["SPL"]
	SPL_sizing_A = configs[config]["SPL_A"]

	if i==0:
		plt.bar(i-0.2, SPL_sizing,   width=style["bar_width_medium"], align='center', alpha=1, color=style["colors"][0], edgecolor='k', label="Unweighted")
		plt.bar(i+0.2, SPL_sizing_A, width=style["bar_width_medium"], align='center', alpha=1, color=style["colors"][2], edgecolor='k', label="A-weighted")
	else:
		plt.bar(i-0.2, SPL_sizing,   width=style["bar_width_medium"], align='center', alpha=1, color=style["colors"][0], edgecolor='k')
		plt.bar(i+0.2, SPL_sizing_A, width=style["bar_width_medium"], align='center', alpha=1, color=style["colors"][2], edgecolor='k')

SPL_req = 62
plt.plot([np.min(y_pos)-1, np.max(y_pos)+1], [SPL_req, SPL_req], color="black", linewidth=3, linestyle="--", label="62 dBA")
plt.xlim(xmin=np.min(y_pos)-1,xmax=np.max(y_pos)+1)
plt.ylim(ymin=57, ymax=77)
plt.grid()
plt.xticks(y_pos, labels,     fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                   fontsize=style["fontsize"]["yticks"])
plt.ylabel('SPL (dB)',        fontsize=style["fontsize"]["ylabel"])
plt.title("Hover Sound (sizing mission)",   fontsize=style["fontsize"]["title"])
plt.legend(loc="lower right", fontsize=style["fontsize"]["legend"], framealpha=1)


#Rotor tip speed 
plt.subplot(3,3,4)
for i, config in enumerate(configs):
	VT = configs[config]["solution"]("VT_OnDemandSizingMission")[0].to(ureg.m/ureg.s).magnitude
	plt.bar(i, VT, align='center', alpha=1, color='k',edgecolor='k')

plt.grid()
plt.xticks(y_pos, labels,     fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                   fontsize=style["fontsize"]["yticks"])
plt.ylabel('Tip speed (m/s)', fontsize=style["fontsize"]["ylabel"])
plt.title("Rotor Tip Speed",  fontsize=style["fontsize"]["title"])


#Energy use by mission segment (sizing mission)
plt.subplot(3,3,5)
for i, config in enumerate(configs):
	solution = configs[config]["solution"]

	E_data = [dict() for x in range(3)]	
	
	E_data[0]["type"] = "Cruise"
	E_data[0]["value"] = solution("E_OnDemandSizingMission")[1]
	
	E_data[1]["type"] = "Hover"
	E_data[1]["value"] = solution("E_OnDemandSizingMission")[0] + solution("E_OnDemandSizingMission")[3]

	E_data[2]["type"] = "Reserve"
	E_data[2]["value"] = solution("E_OnDemandSizingMission")[2]

	bottom = 0
	for j,E in enumerate(E_data):
		E_value = E["value"].to(ureg.kWh).magnitude
		
		if i==0:
			plt.bar(i,E_value, align='center', bottom=bottom, alpha=1,color=style["colors"][j], edgecolor='k', label=E["type"])
		else:
			plt.bar(i,E_value, align='center', bottom=bottom, alpha=1,color=style["colors"][j], edgecolor='k')
		
		bottom += E_value

plt.grid()
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.4*ymax)
plt.xticks(y_pos, labels,     fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                   fontsize=style["fontsize"]["yticks"])
plt.ylabel('Energy (kWh)',    fontsize=style["fontsize"]["ylabel"])
plt.title("Energy Use",       fontsize=style["fontsize"]["title"])
plt.legend(loc="upper right", fontsize=style["fontsize"]["legend"], framealpha=1)


# Power draw by mission segment (sizing mission)
plt.subplot(3,3,6)
for i, config in enumerate(configs):
	solution = configs[config]["solution"]

	P_battery    = np.zeros(3)	
	P_battery[0] = solution("P_{battery}_OnDemandSizingMission")[1].to(ureg.kW).magnitude  # Cruise
	P_battery[1] = solution("P_{battery}_OnDemandSizingMission")[0].to(ureg.kW).magnitude  # Hover
	P_battery[2] = solution("P_{battery}_OnDemandSizingMission")[2].to(ureg.kW).magnitude  # Reserve
	
	for j,offset in enumerate(style["offsets"]):
		if i==0:
			if j==0:
				label = "Cruise"
			elif j==1:
				label = "Hover"
			elif j==2:
				label = "Reserve"

			plt.bar(i+offset, P_battery[j], align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k',label=label)
		else:
			plt.bar(i+offset, P_battery[j], align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.6*ymax)
plt.grid()
plt.xticks(y_pos, labels,       fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                     fontsize=style["fontsize"]["yticks"])
plt.ylabel('Power (kW)',        fontsize=style["fontsize"]["ylabel"])
plt.title("Battery Power Draw", fontsize=style["fontsize"]["title"])
plt.legend(loc="upper right",   fontsize=style["fontsize"]["legend"], framealpha=1)


# Vehicle Acquisition Costs
plt.subplot(3,3,7)
for i, config in enumerate(configs):
	
	c_vehicle  = configs[config]["solution"]("purchase_price_OnDemandAircraft") / 1e6
	c_avionics = configs[config]["solution"]("purchase_price_OnDemandAircraft/Avionics") / 1e6
	c_battery  = configs[config]["solution"]("purchase_price_OnDemandAircraft/Battery") / 1e6

	if i==0:
		plt.bar(i, c_vehicle,  bottom=0,                    align='center', alpha=1, color=style["colors"][0], edgecolor='k', label="Vehicle")
		plt.bar(i, c_avionics, bottom=c_vehicle,            align='center', alpha=1, color=style["colors"][1], edgecolor='k', label="Avionics")
		plt.bar(i, c_battery,  bottom=c_vehicle+c_avionics, align='center', alpha=1, color=style["colors"][2], edgecolor='k', label="Battery")
	else:
		plt.bar(i, c_vehicle,  bottom=0,                    align='center', alpha=1, color=style["colors"][0], edgecolor='k')
		plt.bar(i, c_avionics, bottom=c_vehicle,            align='center', alpha=1, color=style["colors"][1], edgecolor='k')
		plt.bar(i, c_battery,  bottom=c_vehicle+c_avionics, align='center', alpha=1, color=style["colors"][2], edgecolor='k')

plt.grid()
plt.xticks(y_pos, labels,         fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                       fontsize=style["fontsize"]["yticks"])
plt.ylabel('Cost ($millions US)', fontsize=style["fontsize"]["ylabel"])
plt.title("Acquisition Costs",    fontsize=style["fontsize"]["title"])
plt.legend(loc="lower left",      fontsize=style["fontsize"]["legend"], framealpha=1)


# Capital Expenses (revenue mission)
plt.subplot(3,3,8)
for i, config in enumerate(configs):
	
	c_vehicle  = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses/VehicleAcquisitionCost")
	c_avionics = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses/AvionicsAcquisitionCost")
	c_battery  = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses/BatteryAcquisitionCost")
	
	if i==0:
		plt.bar(i, c_vehicle,  bottom=0,                    align='center', alpha=1, color=style["colors"][0], edgecolor='k', label="Vehicle")
		plt.bar(i, c_avionics, bottom=c_vehicle,            align='center', alpha=1, color=style["colors"][1], edgecolor='k', label="Avionics")
		plt.bar(i, c_battery,  bottom=c_vehicle+c_avionics, align='center', alpha=1, color=style["colors"][2], edgecolor='k', label="Battery")
	else:
		plt.bar(i, c_vehicle,  bottom=0,                    align='center', alpha=1, color=style["colors"][0], edgecolor='k')
		plt.bar(i, c_avionics, bottom=c_vehicle,            align='center', alpha=1, color=style["colors"][1], edgecolor='k')
		plt.bar(i, c_battery,  bottom=c_vehicle+c_avionics, align='center', alpha=1, color=style["colors"][2], edgecolor='k')

plt.grid()
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.25*ymax)
plt.xticks(y_pos, labels,             fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                           fontsize=style["fontsize"]["yticks"])
plt.ylabel('Cost ($US/mission)',      fontsize=style["fontsize"]["ylabel"])
plt.title("Mission Capital Expenses", fontsize=style["fontsize"]["title"])
plt.legend(loc="upper right",         fontsize=style["fontsize"]["legend"], framealpha=1)


# Operating Expenses (revenue mission)
plt.subplot(3,3,9)
for i, config in enumerate(configs):
	
	c_pilot       = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses/PilotCost")
	c_maintenance = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses/MaintenanceCost")
	c_energy      = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses/EnergyCost")
	IOC           = configs[config]["solution"]("IOC_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses")
	
	if i==0:
		plt.bar(i, c_pilot,       bottom=0,                              align='center', alpha=1, color=style["colors"][0], edgecolor='k', label="Pilot")
		plt.bar(i, c_maintenance, bottom=c_pilot,                        align='center', alpha=1, color=style["colors"][1], edgecolor='k', label="Maintenance")
		plt.bar(i, c_energy,      bottom=c_pilot+c_maintenance,          align='center', alpha=1, color=style["colors"][2], edgecolor='k', label="Energy")
		plt.bar(i, IOC,           bottom=c_pilot+c_maintenance+c_energy, align='center', alpha=1, color=style["colors"][3], edgecolor='k', label="IOC")
	else:
		plt.bar(i, c_pilot,       bottom=0,                              align='center', alpha=1, color=style["colors"][0], edgecolor='k')
		plt.bar(i, c_maintenance, bottom=c_pilot,                        align='center', alpha=1, color=style["colors"][1], edgecolor='k')
		plt.bar(i, c_energy,      bottom=c_pilot+c_maintenance,          align='center', alpha=1, color=style["colors"][2], edgecolor='k')
		plt.bar(i, IOC,           bottom=c_pilot+c_maintenance+c_energy, align='center', alpha=1, color=style["colors"][3], edgecolor='k')

plt.grid()
plt.xticks(y_pos, labels,               fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                             fontsize=style["fontsize"]["yticks"])
plt.ylabel('Cost ($US/mission)',        fontsize=style["fontsize"]["ylabel"])
plt.title("Mission Operating Expenses", fontsize=style["fontsize"]["title"])
plt.legend(loc="lower left",            fontsize=style["fontsize"]["legend"], framealpha=1)


plt.tight_layout()
plt.subplots_adjust(left=0.08, right=0.99, bottom=0.10, top=0.97, hspace=0.7)
plt.savefig('config_tradeStudy_plot_01.pdf')


# Data output (to screen and to text file)
outputs       = ["Takeoff mass", "Empty mass", "Battery mass", "Mission time", "Flight time", "Charging time", "Trip cost per passenger", "Cost per seat km"]
output_units  = ["kg",           "kg",         "kg",           "minutes",      "minutes",     "minutes",       "dimensionless",           "km**-1"          ]          
output_spaces = ["\t",           "\t",         "\t",           "\t",           "\t",          "\t",            "",                        ""                ]

outputs       += ["Rotor tip speed", "Rotor tip Mach number", "Rotor figure of merit", "Hover SPL (unweighted)",  "Hover SPL (A-weighted)",  "Peak frequency"]
output_units  += ["m/s",             "dimensionless",         "dimensionless"        , "dimensionless",           "dimensionless",           "turn/s"        ]
output_spaces += ["\t",              "",                      "",                      "",                        "",                        "\t"            ]

output_string =  "Tabulated Data by Configuration\n"
output_string += "\n"
output_string += "Configuration\t\t"
for config in configs:
	output_string += config
	output_string += "\t"

output_string += "Units\n"
output_string += "\n"

for i, output in enumerate(outputs):
	
	units         =  output_units[i]
	output_space  =  output_spaces[i]
	output_string += output + "\t" + output_space

	for j, config in enumerate(configs):

		solution = configs[config]["solution"]
		
		if output == "Takeoff mass":
			var_string  = "TOGM_OnDemandAircraft"
			precision   = "%0.1f"

		elif output == "Empty mass":
			var_string  = "m_OnDemandAircraft/Structure"
			precision   = "%0.1f"

		elif output == "Battery mass":
			var_string  = "m_OnDemandAircraft/Battery"
			precision   = "%0.1f"

		elif output == "Mission time":
			var_string = "t_{mission}_OnDemandRevenueMission"
			precision  = "%0.1f"
		
		elif output == "Flight time":
			var_string = "t_{flight}_OnDemandRevenueMission"
			precision  = "%0.1f"

		elif output == "Charging time":
			var_string = "t_OnDemandRevenueMission/TimeOnGround"
			precision  = "%0.1f"
		
		elif output == "Trip cost per passenger":
			var_string  = "cost_per_trip_per_passenger"
			precision   = "%0.1f"

		elif output == "Cost per seat km":
			var_string  = "cost_per_seat_mile"
			precision   = "%0.2f"

		elif output == "Rotor tip speed":
			var_string  = "VT_OnDemandSizingMission"
			precision   = "%0.1f"

			output_string += precision % solution(var_string)[0].to(ureg(units)).magnitude
			output_string += "\t\t"

			continue

		elif output == "Rotor tip Mach number":
			var_string  = "MT_OnDemandSizingMission"
			precision   = "%0.2f"

			output_string += precision % solution(var_string)[0].to(ureg(units)).magnitude
			output_string += "\t\t"

			continue

		elif output == "Rotor figure of merit":
			var_string  = "FOM_OnDemandSizingMission"
			precision   = "%0.2f"

			output_string += precision % solution(var_string)[0].to(ureg(units)).magnitude
			output_string += "\t\t"

			continue

		elif output == "Hover SPL (unweighted)":

			precision = "%0.1f"

			output_string += precision % configs[config]["SPL"]
			output_string += "\t\t"

			continue

		elif output == "Hover SPL (A-weighted)":

			precision = "%0.1f"

			output_string += precision % configs[config]["SPL_A"]
			output_string += "\t\t"

			continue
		
		elif output == "Peak frequency":

			precision = "%0.0f"

			output_string += precision % configs[config]["f_{peak}"].to(ureg(units)).magnitude
			output_string += "\t\t"

			continue


		output_string += precision % solution(var_string).to(ureg(units)).magnitude
		output_string += "\t\t"

	output_string += units + "\n"

print "\n\n"
print output_string

text_file = open("config_trade_study_tabulatedData.txt", "w")
text_file.write(output_string)
text_file.close()
