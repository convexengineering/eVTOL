#Vehicle configuration top-level trade study

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + "/../models"))

import numpy as np
from gpkit                  import Model, ureg
from copy                   import deepcopy
from matplotlib             import pyplot as plt
from aircraft_models        import OnDemandAircraft
from mission_models         import OnDemandSizingMission, OnDemandRevenueMission, OnDemandDeadheadMission
from cost_models            import OnDemandMissionCost
from noise_models           import vortex_noise
from standard_substitutions import generic_data, configs

configs = deepcopy(configs)


#Optimize and do noise analysis
for config in configs:
	
	print "Solving configuration: " + config

	aircraft = OnDemandAircraft()
	aircraft = aircraft.standard_substitutions(config=config, autonomousEnabled=generic_data["autonomousEnabled"])

	sizing_mission = OnDemandSizingMission(aircraft=aircraft)
	sizing_mission = sizing_mission.standard_substitutions(piloted=generic_data["isSizingMissionPiloted"], reserve=generic_data["reserve"])

	revenue_mission = OnDemandRevenueMission(aircraft=aircraft)
	revenue_mission = revenue_mission.standard_substitutions(piloted=generic_data["isRevenueMissionPiloted"])

	deadhead_mission = OnDemandDeadheadMission(aircraft=aircraft)
	deadhead_mission = deadhead_mission.standard_substitutions(piloted=generic_data["isDeadheadMissionPiloted"])

	mission_cost = OnDemandMissionCost(aircraft=aircraft, revenue_mission=revenue_mission, deadhead_mission=deadhead_mission)
	mission_cost = mission_cost.standard_substitutions(isRevenueMissionPiloted=generic_data["isRevenueMissionPiloted"], isDeadheadMissionPiloted=generic_data["isDeadheadMissionPiloted"])

	objective_function = mission_cost.cpt
	problem            = Model(objective_function, [aircraft, sizing_mission, revenue_mission, deadhead_mission, mission_cost])
	solution           = problem.solve(verbosity=0)

	configs[config]["solution"] = solution

	# Noise computations
	T_perRotor = solution("T_perRotor_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	T_A        = solution("T/A_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	V_tip      = solution("v_{tip}_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	s          = solution("s_OnDemandAircraft/Rotors")
	Cl_mean    = solution("Cl_{mean}_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	N          = solution("N_OnDemandAircraft/Rotors")
	c_avg      = solution("c_{avg}_OnDemandAircraft/Rotors")
	t_avg      = solution("t_{avg}_OnDemandAircraft/Rotors")
	rho        = solution("\\rho_OnDemandSizingMission/HoverTakeoff/HoverFlightState/FixedStandardAtmosphere")
	
	delta_S = generic_data["delta_S"]
	St      = generic_data["Strouhal_number"]

	#Unweighted
	f_peak, SPL, spectrum = vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S, St, weighting="None")
	
	configs[config]["SPL"]      = SPL
	configs[config]["f_{peak}"] = f_peak

	#A-weighted
	f_peak, SPL, spectrum = vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S, St, weighting="A")	

	configs[config]["SPL_A"] = SPL


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
	
	m_airframe  = configs[config]["solution"]("m_OnDemandAircraft/Airframe").to(ureg.kg).magnitude
	m_battery   = configs[config]["solution"]("m_OnDemandAircraft/Battery").to(ureg.kg).magnitude
	MTOM        = configs[config]["solution"]("MTOM_OnDemandAircraft").to(ureg.kg).magnitude
	m_remainder = MTOM - m_airframe - m_battery

	if i==0:
		plt.bar(i, m_airframe,  align='center', bottom=0,                    alpha=1, color=style["colors"][0], edgecolor='k', label="Airframe")
		plt.bar(i, m_battery,   align='center', bottom=m_airframe,           alpha=1, color=style["colors"][1], edgecolor='k', label="Battery")
		plt.bar(i, m_remainder, align='center', bottom=m_airframe+m_battery, alpha=1, color=style["colors"][2], edgecolor='k', label="Crew & passengers")
	else:
		plt.bar(i, m_airframe,  align='center', bottom=0,                    alpha=1, color=style["colors"][0], edgecolor='k')
		plt.bar(i, m_battery,   align='center', bottom=m_airframe,           alpha=1, color=style["colors"][1], edgecolor='k')
		plt.bar(i, m_remainder, align='center', bottom=m_airframe+m_battery, alpha=1, color=style["colors"][2], edgecolor='k')

plt.grid()
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.7*ymax)
plt.xticks(y_pos, labels,     fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                   fontsize=style["fontsize"]["yticks"])
plt.ylabel('Mass (kg)',       fontsize=style["fontsize"]["ylabel"])
plt.title("Mass Breakdown",   fontsize=style["fontsize"]["title"])
plt.legend(loc="upper right", fontsize=style["fontsize"]["legend"], framealpha=1)


# Energy use by mission segment (sizing mission)
plt.subplot(3,3,2)
for i, config in enumerate(configs):
	solution = configs[config]["solution"]

	E_cruise  = solution("E_OnDemandSizingMission/Cruise/OnDemandAircraftLevelFlightFlightPerformance/BatteryPerformance").to(ureg.kWh).magnitude
	E_hover   = (solution("E_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/BatteryPerformance") \
		+ solution("E_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/BatteryPerformance")).to(ureg.kWh).magnitude
	E_reserve = solution("E_OnDemandSizingMission/Reserve/OnDemandAircraftLevelFlightFlightPerformance/BatteryPerformance").to(ureg.kWh).magnitude

	if i==0:
		plt.bar(i, E_cruise,  align='center', bottom=0,                alpha=1, color=style["colors"][0], edgecolor='k', label="Cruise")
		plt.bar(i, E_hover,   align='center', bottom=E_cruise,         alpha=1, color=style["colors"][1], edgecolor='k', label="Hover")
		plt.bar(i, E_reserve, align='center', bottom=E_cruise+E_hover, alpha=1, color=style["colors"][2], edgecolor='k', label="Reserve")
	else:
		plt.bar(i, E_cruise,  align='center', bottom=0,                alpha=1, color=style["colors"][0], edgecolor='k')
		plt.bar(i, E_hover,   align='center', bottom=E_cruise,         alpha=1, color=style["colors"][1], edgecolor='k')
		plt.bar(i, E_reserve, align='center', bottom=E_cruise+E_hover, alpha=1, color=style["colors"][2], edgecolor='k')
	

plt.grid()
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.4*ymax)
plt.xticks(y_pos, labels,     fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                   fontsize=style["fontsize"]["yticks"])
plt.ylabel('Energy (kWh)',    fontsize=style["fontsize"]["ylabel"])
plt.title("Energy Use",       fontsize=style["fontsize"]["title"])
plt.legend(loc="upper right", fontsize=style["fontsize"]["legend"], framealpha=1)


# Power draw by mission segment (sizing mission)
plt.subplot(3,3,3)
for i, config in enumerate(configs):
	solution = configs[config]["solution"]

	P_electric    = np.zeros(3)	
	P_electric[0] = solution("P_{electric}_OnDemandSizingMission/Cruise/LevelFlightState").to(ureg.kW).magnitude        # Cruise
	P_electric[1] = solution("P_{electric}_OnDemandSizingMission/HoverTakeoff/HoverFlightState").to(ureg.kW).magnitude  # Hover
	P_electric[2] = solution("P_{electric}_OnDemandSizingMission/Reserve/LevelFlightState").to(ureg.kW).magnitude       # Reserve
	
	for j,offset in enumerate(style["offsets"]):
		if i==0:
			if j==0:
				label = "Cruise"
			elif j==1:
				label = "Hover"
			elif j==2:
				label = "Reserve"

			plt.bar(i+offset, P_electric[j], align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k',label=label)
		else:
			plt.bar(i+offset, P_electric[j], align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.6*ymax)
plt.grid()
plt.xticks(y_pos, labels,       fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                     fontsize=style["fontsize"]["yticks"])
plt.ylabel('Power (kW)',        fontsize=style["fontsize"]["ylabel"])
plt.title("Battery Power Draw", fontsize=style["fontsize"]["title"])
plt.legend(loc="upper right",   fontsize=style["fontsize"]["legend"], framealpha=1)


# Mission time
plt.subplot(3,3,4)
for i, config in enumerate(configs):
	
	t_flight = configs[config]["solution"]("t_{flight}_OnDemandRevenueMission").to(ureg.min).magnitude
	t_charge = configs[config]["solution"]("t_{segment}_OnDemandRevenueMission/TimeOnGround").to(ureg.min).magnitude

	if i==0:
		plt.bar(i, t_flight, bottom=0,        align='center', alpha=1, color=style["colors"][0], edgecolor='k', label="Flight time")
		plt.bar(i, t_charge, bottom=t_flight, align='center', alpha=1, color=style["colors"][2], edgecolor='k', label="Charging time")
	else:
		plt.bar(i, t_flight, bottom=0,        align='center', alpha=1, color=style["colors"][0], edgecolor='k')
		plt.bar(i, t_charge, bottom=t_flight, align='center', alpha=1, color=style["colors"][2], edgecolor='k')

plt.grid()
plt.xticks(y_pos, labels,      fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                    fontsize=style["fontsize"]["yticks"])
plt.ylabel('Time (minutes)',   fontsize=style["fontsize"]["ylabel"])
plt.title("Mission Time",      fontsize=style["fontsize"]["title"])
plt.legend(loc="lower left",   fontsize=style["fontsize"]["legend"], framealpha=1)


# Trip cost 
plt.subplot(3,3,5)
for i, config in enumerate(configs):
	
	cpt_revenue  = configs[config]["solution"]("revenue_cost_per_trip")
	cpt_deadhead = configs[config]["solution"]("deadhead_cost_per_trip")

	if i==0:
		plt.bar(i, cpt_revenue,  bottom=0,           align='center', alpha=1, color=style["colors"][0], edgecolor='k', label="Revenue mission")
		plt.bar(i, cpt_deadhead, bottom=cpt_revenue, align='center', alpha=1, color=style["colors"][2], edgecolor='k', label="Deadhead effect")
	else:
		plt.bar(i, cpt_revenue,  bottom=0,           align='center', alpha=1, color=style["colors"][0], edgecolor='k')
		plt.bar(i, cpt_deadhead, bottom=cpt_revenue, align='center', alpha=1, color=style["colors"][2], edgecolor='k')

plt.grid()
plt.xticks(y_pos, labels,     fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                   fontsize=style["fontsize"]["yticks"])
plt.ylabel('Cost ($US/trip)', fontsize=style["fontsize"]["ylabel"])
plt.title("Trip Cost ",       fontsize=style["fontsize"]["title"])
plt.legend(loc="lower left",  fontsize=style["fontsize"]["legend"], framealpha=1)


# Cost per Passenger Kilometer
plt.subplot(3,3,6)
for i, config in enumerate(configs):
	
	cpsk = configs[config]["solution"]("cost_per_passenger_km_OnDemandMissionCost").to(ureg.km**-1).magnitude
	plt.bar(i, cpsk, align='center', alpha=1, color='k', edgecolor='k')
		
plt.grid()
plt.xticks(y_pos, labels,                  fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                                fontsize=style["fontsize"]["yticks"])
plt.ylabel('Cost ($US/km)',                fontsize=style["fontsize"]["ylabel"])
plt.title("Cost per Passenger Kilometer",  fontsize=style["fontsize"]["title"])


# Vehicle Purchase Price
plt.subplot(3,3,7)
for i, config in enumerate(configs):
	
	c_airframe = configs[config]["solution"]("purchase_price_OnDemandAircraft/Airframe") / 1e3
	c_avionics = configs[config]["solution"]("purchase_price_OnDemandAircraft/Avionics") / 1e3
	c_battery  = configs[config]["solution"]("purchase_price_OnDemandAircraft/Battery")  / 1e3

	if i==0:
		plt.bar(i, c_airframe, bottom=0,                     align='center', alpha=1, color=style["colors"][0], edgecolor='k', label="Airframe")
		plt.bar(i, c_avionics, bottom=c_airframe,            align='center', alpha=1, color=style["colors"][1], edgecolor='k', label="Avionics")
		plt.bar(i, c_battery,  bottom=c_airframe+c_avionics, align='center', alpha=1, color=style["colors"][2], edgecolor='k', label="Battery")
	else:
		plt.bar(i, c_airframe, bottom=0,                     align='center', alpha=1, color=style["colors"][0], edgecolor='k')
		plt.bar(i, c_avionics, bottom=c_airframe,            align='center', alpha=1, color=style["colors"][1], edgecolor='k')
		plt.bar(i, c_battery,  bottom=c_airframe+c_avionics, align='center', alpha=1, color=style["colors"][2], edgecolor='k')

plt.grid()
plt.xticks(y_pos, labels,           fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                         fontsize=style["fontsize"]["yticks"])
plt.ylabel('Price ($thousands US)', fontsize=style["fontsize"]["ylabel"])
plt.title("Purchase Price",         fontsize=style["fontsize"]["title"])
plt.legend(loc="lower left",        fontsize=style["fontsize"]["legend"], framealpha=1)


# Capital Expenses (revenue mission)
plt.subplot(3,3,8)
for i, config in enumerate(configs):
	
	c_airframe = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses/AirframeAcquisitionCost")
	c_avionics = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses/AvionicsAcquisitionCost")
	c_battery  = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses/BatteryAcquisitionCost")
	
	if i==0:
		plt.bar(i, c_airframe, bottom=0,                     align='center', alpha=1, color=style["colors"][0], edgecolor='k', label="Airframe")
		plt.bar(i, c_avionics, bottom=c_airframe,            align='center', alpha=1, color=style["colors"][1], edgecolor='k', label="Avionics")
		plt.bar(i, c_battery,  bottom=c_airframe+c_avionics, align='center', alpha=1, color=style["colors"][2], edgecolor='k', label="Battery")
	else:
		plt.bar(i, c_airframe, bottom=0,                     align='center', alpha=1, color=style["colors"][0], edgecolor='k')
		plt.bar(i, c_avionics, bottom=c_airframe,            align='center', alpha=1, color=style["colors"][1], edgecolor='k')
		plt.bar(i, c_battery,  bottom=c_airframe+c_avionics, align='center', alpha=1, color=style["colors"][2], edgecolor='k')

plt.grid()
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.35*ymax)
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
	IOC           = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses/IndirectOperatingCost")
	
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
plt.subplots_adjust(left=0.08, right=0.98, bottom=0.10, top=0.97, hspace=0.7)
plt.savefig('config_tradeStudy_plot_01.pdf')





"""
# Sound pressure level (in hover) 
plt.subplot(3,3,9)

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
plt.ylim(ymin=51, ymax=77)
plt.grid()
plt.xticks(y_pos, labels,     fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                   fontsize=style["fontsize"]["yticks"])
plt.ylabel('SPL (dB)',        fontsize=style["fontsize"]["ylabel"])
plt.title("Hover Sound (sizing mission)",   fontsize=style["fontsize"]["title"])
plt.legend(loc="lower right", fontsize=style["fontsize"]["legend"], framealpha=1)
"""

# Data output (to screen and to text file)
outputs       = ["Max takeoff mass", "Airframe mass", "Battery mass", "Mission time", "Flight time", "Charging time", "Purchase price",  "Trip cost",     "Cost per passenger-km"]
output_units  = ["kg",               "kg",            "kg",           "minutes",      "minutes",     "minutes",       "$US (thousands)", "dimensionless", "km**-1"               ]          
output_spaces = ["",                 "\t",            "\t",           "\t",           "\t",          "\t",            "\t",              "\t",            ""                     ]

outputs       += ["Rotor diameter", "Tip speed", "Tip Mach number", "Thrust coefficient", "Power coefficient", "Figure of merit"]
output_units  += ["m",              "m/s",       "dimensionless",   "dimensionless",      "dimensionless",     "dimensionless"  ]
output_spaces += ["\t",             "\t",        "\t",              "",                   "",                  "\t",            ]

outputs       += ["Hover SPL (unweighted)",  "Hover SPL (A-weighted)",  "Vortex peak frequency"]
output_units  += ["dimensionless",           "dimensionless",           "turn/s"               ]
output_spaces += ["",                        "",                        ""                     ]

output_string =  "Tabulated Data by Configuration\n"
output_string += "\n"
output_string += "Configuration\t\t\t"
for config in configs:
	output_string += config
	output_string += "\t"

output_string += "Units\n"
output_string += "\n"

for i, output in enumerate(outputs):
	
	units         =  output_units[i]
	output_space  =  output_spaces[i]
	output_string += output + "\t\t" + output_space

	for j, config in enumerate(configs):

		solution = configs[config]["solution"]
		
		if output == "Max takeoff mass":
			var_string  = "MTOM_OnDemandAircraft"
			precision   = "%0.0f"

		elif output == "Airframe mass":
			var_string  = "m_OnDemandAircraft/Airframe"
			precision   = "%0.0f"

		elif output == "Battery mass":
			var_string  = "m_OnDemandAircraft/Battery"
			precision   = "%0.0f"

		elif output == "Mission time":
			var_string = "t_{mission}_OnDemandRevenueMission"
			precision  = "%0.1f"
		
		elif output == "Flight time":
			var_string = "t_{flight}_OnDemandRevenueMission"
			precision  = "%0.1f"

		elif output == "Charging time":
			var_string = "t_{segment}_OnDemandRevenueMission/TimeOnGround"
			precision  = "%0.1f"
		
		elif output == "Trip cost":
			var_string  = "cost_per_trip"
			precision   = "%0.0f"

		elif output == "Cost per passenger-km":
			var_string  = "cost_per_passenger_km"
			precision   = "%0.2f"

		elif output == "Purchase price":
			var_strings = ["purchase_price_OnDemandAircraft/Airframe", "purchase_price_OnDemandAircraft/Avionics", "purchase_price_OnDemandAircraft/Battery"]
			precision   = "%0.0f"

			output_string += precision % (sum(solution(v) for v in var_strings).to(ureg.dimensionless).magnitude / 1e3)
			output_string += "\t\t"

			continue

		elif output == "Rotor diameter":
			var_string  = "D_OnDemandAircraft/Rotors"
			precision   = "%0.2f"

		elif output == "Tip speed":
			var_string  = "v_{tip}_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance"
			precision   = "%0.1f"

		elif output == "Tip Mach number":
			var_string  = "M_{tip}_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance"
			precision   = "%0.2f"

		elif output == "Thrust coefficient":
			var_string  = "CT_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance"
			precision   = "%0.4f"

		elif output == "Power coefficient":
			var_string  = "CP_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance"
			precision   = "%0.4f"

		elif output == "Figure of merit":
			var_string  = "FOM_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance"
			precision   = "%0.2f"

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
		
		elif output == "Vortex peak frequency":

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
