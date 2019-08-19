# Sweep to evaluate the sensitivity of each design to mission range

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../../models'))

import numpy as np
from gpkit                  import Model, ureg
from copy                   import deepcopy
from collections            import OrderedDict
from matplotlib             import pyplot as plt
from aircraft_models        import OnDemandAircraft
from mission_models         import OnDemandSizingMission, OnDemandRevenueMission, OnDemandDeadheadMission
from cost_models            import OnDemandMissionCost
from noise_models           import vortex_noise
from standard_substitutions import generic_data, configs


#set up range arrays (different for each configuration)

num_pts = 9

for config in configs:

	print "Solving configuration: " + config

	# Set up range arrays (different for each configuration)

	if (config == "Multirotor"):
		mission_range = np.linspace(1,  18, num_pts) * ureg.nautical_mile
	elif (config == "Helicopter"):
		mission_range = np.linspace(2,  36, num_pts) * ureg.nautical_mile
	elif (config == "Coaxial heli"):
		mission_range = np.linspace(1,  49, num_pts) * ureg.nautical_mile
	elif (config == "Compound heli"):
		mission_range = np.linspace(10, 82, num_pts) * ureg.nautical_mile
	elif (config == "Lift + cruise"):
		mission_range = np.linspace(10, 85, num_pts) * ureg.nautical_mile
	elif (config == "Tilt wing"):
		mission_range = np.linspace(15, 98, num_pts) * ureg.nautical_mile
	elif (config == "Tilt rotor"):
		mission_range = np.linspace(15, 110, num_pts) * ureg.nautical_mile

	c                 = configs[config]
	c["SPL_sizing_A"] = np.zeros(np.size(mission_range))

	aircraft = OnDemandAircraft()
	aircraft = aircraft.standard_substitutions(config=config, autonomousEnabled=generic_data["autonomousEnabled"])

	sizing_mission = OnDemandSizingMission(aircraft=aircraft)
	sizing_mission = sizing_mission.standard_substitutions(piloted=generic_data["isSizingMissionPiloted"], reserve=generic_data["reserve"])

	sizing_mission.substitutions.update({
		sizing_mission.cruise_segment.d_segment: ("sweep", mission_range.to(ureg.km).magnitude),  # Careful with units here
	})

	revenue_mission = OnDemandRevenueMission(aircraft=aircraft)
	revenue_mission = revenue_mission.standard_substitutions(piloted=generic_data["isRevenueMissionPiloted"])

	deadhead_mission = OnDemandDeadheadMission(aircraft=aircraft)
	deadhead_mission = deadhead_mission.standard_substitutions(piloted=generic_data["isDeadheadMissionPiloted"])

	mission_cost = OnDemandMissionCost(aircraft=aircraft, revenue_mission=revenue_mission, deadhead_mission=deadhead_mission)
	mission_cost = mission_cost.standard_substitutions(isRevenueMissionPiloted=generic_data["isRevenueMissionPiloted"], isDeadheadMissionPiloted=generic_data["isDeadheadMissionPiloted"])

	# All missions have the same range. Needed or else we can't sweep.
	range_constraints =  [sizing_mission.cruise_segment.d_segment == revenue_mission.cruise_segment.d_segment]
	range_constraints += [sizing_mission.cruise_segment.d_segment == deadhead_mission.cruise_segment.d_segment]

	objective_function = mission_cost.cpt
	problem            = Model(objective_function, [aircraft, sizing_mission, revenue_mission, deadhead_mission, mission_cost, range_constraints])

	del problem.substitutions["d_{segment}_OnDemandRevenueMission/Cruise"]   # These variables were set via substitution. They must be freed before model will solve.
	del problem.substitutions["d_{segment}_OnDemandDeadheadMission/Cruise"]  # These variables were set via substitution. They must be freed before model will solve.

	solution      = problem.solve(verbosity=0)
	c["solution"] = solution

	for i in range(len(mission_range)):

		# Noise computations (sizing mission)
		T_perRotor = solution("T_perRotor_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")[i]
		T_A        = solution("T/A_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")[i]
		V_tip      = solution("v_{tip}_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")[i]
		s          = solution("s_OnDemandAircraft/Rotors")[i]
		Cl_mean    = solution("Cl_{mean}_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")[i]
		N          = solution("N_OnDemandAircraft/Rotors")[i]
		c_avg      = solution("c_{avg}_OnDemandAircraft/Rotors")[i]
		t_avg      = solution("t_{avg}_OnDemandAircraft/Rotors")[i]
		rho        = solution("\\rho_OnDemandSizingMission/HoverTakeoff/HoverFlightState/FixedStandardAtmosphere")[i]
		
		delta_S = generic_data["delta_S"]
		St      = generic_data["Strouhal_number"]

		f_peak, c["SPL_sizing_A"][i], spectrum = vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S, St, weighting="A")


# Plotting commands
plt.ion()
plt.rc('axes', axisbelow=True)
plt.show()

style = {}
style["rotation"]         = -45
style["legend_ncols"]     = 2
style["bar_width_wide"]   = 0.7
style["bar_width_medium"] = 0.3
style["bar_width_narrow"] = 0.2
style["offsets"]          = [-0.25, 0, 0.25]
style["colors"]           = ["grey", "w", "k", "lightgrey"]

style["linestyle"]  = ["-","-","-","-","--","--","--","--"]
style["marker"]     = ["s","o","^","v","s","o","^","v"]
style["fillstyle"]  = ["full","full","full","full","none","none","none","none"]
style["markersize"] = 10

style["fontsize"] = {}
style["fontsize"]["xticks"]     = 14
style["fontsize"]["yticks"]     = 14
style["fontsize"]["xlabel"]     = 16
style["fontsize"]["ylabel"]     = 16
style["fontsize"]["title"]      = 16
style["fontsize"]["legend"]     = 11
style["fontsize"]["text_label"] = 18


# Takeoff mass
fig1 = plt.figure(figsize=(4, 3.7), dpi=80)
for i, config in enumerate(configs):

	solution      = configs[config]["solution"]
	mission_range = solution("d_{segment}_OnDemandSizingMission/Cruise").to(ureg.km).magnitude
	MTOM          = solution("MTOM_OnDemandAircraft").to(ureg.kg).magnitude

	plt.plot(mission_range, MTOM, color="black", linewidth=1.5,linestyle=style["linestyle"][i], marker=style["marker"][i],
		fillstyle=style["fillstyle"][i], markersize=style["markersize"], label=config)

plt.grid()
[ymin, ymax] = plt.gca().get_ylim()
plt.ylim(ymin=0, ymax=1.1*ymax)
plt.xticks(                       fontsize=style["fontsize"]["xticks"])
plt.yticks(                       fontsize=style["fontsize"]["yticks"])
plt.xlabel('Mission range (km)',  fontsize=style["fontsize"]["xlabel"])
plt.ylabel('Mass (kg)',           fontsize=style["fontsize"]["ylabel"])
# plt.title("Maximum Takeoff Mass", fontsize=style["fontsize"]["title"])
plt.legend(loc='lower right',     fontsize=style["fontsize"]["legend"], framealpha=1, numpoints=1)

plt.tight_layout()
plt.subplots_adjust(left=0.23, right=0.98, bottom=0.16, top=0.98)
plt.savefig('mission_range_plot_01_MaximumTakeoffMass.pdf')


# Mission time
fig2 = plt.figure(figsize=(4, 3.7), dpi=80)
for i, config in enumerate(configs):

	solution      = configs[config]["solution"]
	mission_range = solution("d_{segment}_OnDemandSizingMission/Cruise").to(ureg.km).magnitude
	t_mission     = solution("t_{mission}_OnDemandRevenueMission").to(ureg.min).magnitude

	plt.plot(mission_range, t_mission, color="black", linewidth=1.5,linestyle=style["linestyle"][i], marker=style["marker"][i],
		fillstyle=style["fillstyle"][i], markersize=style["markersize"], label=config)

plt.grid()
[ymin, ymax] = plt.gca().get_ylim()
plt.ylim(ymin=0, ymax=1.1*ymax)
plt.xticks(                      fontsize=style["fontsize"]["xticks"])
plt.yticks(                      fontsize=style["fontsize"]["yticks"])
plt.xlabel('Mission range (km)', fontsize=style["fontsize"]["xlabel"])
plt.ylabel('Time (minutes)',     fontsize=style["fontsize"]["ylabel"])
# plt.title("Mission Time",        fontsize=style["fontsize"]["title"])

plt.tight_layout()
plt.subplots_adjust(left=0.23, right=0.98, bottom=0.16, top=0.98)
plt.savefig('mission_range_plot_02_MissionTime.pdf')


# Trip cost per passenger-km
fig3 = plt.figure(figsize=(4, 3.7), dpi=80)
for i, config in enumerate(configs):

	solution      = configs[config]["solution"]
	mission_range = solution("d_{segment}_OnDemandSizingMission/Cruise").to(ureg.km).magnitude
	cppk          = solution("cost_per_passenger_km").to(ureg.km**-1).magnitude

	plt.plot(mission_range, cppk, color="black", linewidth=1.5,linestyle=style["linestyle"][i], marker=style["marker"][i],
		fillstyle=style["fillstyle"][i], markersize=style["markersize"], label=config)

plt.grid()
[ymin, ymax] = plt.gca().get_ylim()
plt.ylim(ymin=0, ymax=1.1*ymax)
plt.xticks(                               fontsize=style["fontsize"]["xticks"])
plt.yticks(                               fontsize=style["fontsize"]["yticks"])
plt.xlabel('Mission range (km)',          fontsize=style["fontsize"]["xlabel"])
plt.ylabel('Cost ($US/km)',               fontsize=style["fontsize"]["ylabel"])
# plt.title("Cost per Passenger Kilometer", fontsize=style["fontsize"]["title"])

plt.tight_layout()
plt.subplots_adjust(left=0.23, right=0.98, bottom=0.16, top=0.98)
plt.savefig('mission_range_plot_03_CostPerPassengerKilometer.pdf')



"""
# Sound in hover
plt.subplot(1,3,3)
for i, config in enumerate(configs):

	solution      = configs[config]["solution"]
	mission_range = solution("d_{segment}_OnDemandSizingMission/Cruise").to(ureg.km).magnitude
	SPL_sizing_A  = configs[config]["SPL_sizing_A"]

	plt.plot(mission_range, SPL_sizing_A, color="black", linewidth=1.5,linestyle=style["linestyle"][i], marker=style["marker"][i],
		fillstyle=style["fillstyle"][i], markersize=style["markersize"], label=config)

plt.grid()
[ymin, ymax] = plt.gca().get_ylim()
plt.ylim(ymin=0.95*ymin, ymax=1.01*ymax)
plt.xticks(                               fontsize=style["fontsize"]["xticks"])
plt.yticks(                               fontsize=style["fontsize"]["yticks"])
plt.xlabel('Mission range (km)',          fontsize=style["fontsize"]["xlabel"])
plt.ylabel('SPL (dBA)',                   fontsize=style["fontsize"]["ylabel"])
plt.title("Hover Sound (sizing mission)", fontsize=style["fontsize"]["title"])
"""