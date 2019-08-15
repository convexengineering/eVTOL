# Sensitivity to sizing assumptions for New York airport shuttle service

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


# Data specific to study
cases = OrderedDict()

for config in configs:

	cases[config] = OrderedDict()

	cases[config]["Case 1"] = {}
	cases[config]["Case 2"] = {}
	cases[config]["Case 3"] = {}

	cases[config]["Case 1"]["sizing_mission_range"] = 19. * ureg.nautical_mile
	cases[config]["Case 2"]["sizing_mission_range"] = 30. * ureg.nautical_mile
	cases[config]["Case 3"]["sizing_mission_range"] = 30. * ureg.nautical_mile

	cases[config]["Case 1"]["revenue_mission_range"] = 19. * ureg.nautical_mile
	cases[config]["Case 2"]["revenue_mission_range"] = 19. * ureg.nautical_mile
	cases[config]["Case 3"]["revenue_mission_range"] = 30. * ureg.nautical_mile

	cases[config]["Case 1"]["deadhead_mission_range"] = 19. * ureg.nautical_mile
	cases[config]["Case 2"]["deadhead_mission_range"] = 19. * ureg.nautical_mile
	cases[config]["Case 3"]["deadhead_mission_range"] = 30. * ureg.nautical_mile


# Optimize
for config in cases:
	
	print "Solving configuration: " + config

	for i, case in enumerate(cases[config]):
		
		c = cases[config][case]

		aircraft = OnDemandAircraft()
		aircraft = aircraft.standard_substitutions(config=config, autonomousEnabled=generic_data["autonomousEnabled"])

		sizing_mission = OnDemandSizingMission(aircraft=aircraft)
		sizing_mission = sizing_mission.standard_substitutions(piloted=generic_data["isSizingMissionPiloted"], reserve=generic_data["reserve"])

		sizing_mission.substitutions.update({
			sizing_mission.cruise_segment.d_segment: c["sizing_mission_range"],
		})

		revenue_mission = OnDemandRevenueMission(aircraft=aircraft)
		revenue_mission = revenue_mission.standard_substitutions(piloted=generic_data["isRevenueMissionPiloted"])

		revenue_mission.substitutions.update({
			revenue_mission.cruise_segment.d_segment: c["revenue_mission_range"],
		})

		deadhead_mission = OnDemandDeadheadMission(aircraft=aircraft)
		deadhead_mission = deadhead_mission.standard_substitutions(piloted=generic_data["isDeadheadMissionPiloted"])

		deadhead_mission.substitutions.update({
			deadhead_mission.cruise_segment.d_segment: c["deadhead_mission_range"],
		})

		mission_cost = OnDemandMissionCost(aircraft=aircraft, revenue_mission=revenue_mission, deadhead_mission=deadhead_mission)
		mission_cost = mission_cost.standard_substitutions(isRevenueMissionPiloted=generic_data["isRevenueMissionPiloted"], isDeadheadMissionPiloted=generic_data["isDeadheadMissionPiloted"])

		objective_function = mission_cost.cpt
		problem            = Model(objective_function, [aircraft, sizing_mission, revenue_mission, deadhead_mission, mission_cost])
		solution           = problem.solve(verbosity=0)

		c["solution"] = solution

		# Noise computations (sizing mission)
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

		f_peak, SPL, spectrum = vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S, St, weighting="A")

		c["SPL_sizing_A"]      = SPL
		c["f_{peak}"]          = f_peak
		c["spectrum_sizing_A"] = spectrum

		#Noise computations (revenue mission)
		T_perRotor = solution("T_perRotor_OnDemandRevenueMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
		T_A        = solution("T/A_OnDemandRevenueMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
		V_tip      = solution("v_{tip}_OnDemandRevenueMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
		Cl_mean    = solution("Cl_{mean}_OnDemandRevenueMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
		rho        = solution("\\rho_OnDemandRevenueMission/HoverTakeoff/HoverFlightState/FixedStandardAtmosphere")

		f_peak, SPL, spectrum = vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S, St, weighting="A")
		
		c["SPL_revenue_A"]      = SPL
		c["f_{peak}"]           = f_peak
		c["spectrum_revenue_A"] = spectrum


# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(11,4), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

y_pos = np.arange(len(configs))
labels = [""]*len(configs)
for i, config in enumerate(configs):
	if config == "Compound heli":
		labels[i] = config.replace(" ", "\n")  # Replace spaces with newlines
	else:
		labels[i] = config

xmin = np.min(y_pos) - 0.7
xmax = np.max(y_pos) + 0.7

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

# Takeoff mass
plt.subplot(1,3,1)
for i,config in enumerate(cases):

	for j, case in enumerate(cases[config]):

		offset   = style["offsets"][j]
		solution = cases[config][case]["solution"]
		MTOM     = solution("MTOM").to(ureg.kg).magnitude

		sizing_mission_range  = solution("d_{segment}_OnDemandSizingMission/Cruise").to(ureg.km).magnitude
		revenue_mission_range = solution("d_{segment}_OnDemandRevenueMission/Cruise").to(ureg.km).magnitude

		label = "%0.0f km sizing mission;\n%0.0f km revenue mission" % (sizing_mission_range, revenue_mission_range)

		if i==0:
			plt.bar(i+offset, MTOM, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k', label=label)
		else:
			plt.bar(i+offset, MTOM, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

plt.xlim(xmin=xmin, xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.0*ymax)
plt.grid()
plt.xticks(y_pos, labels,         fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                       fontsize=style["fontsize"]["yticks"])
plt.ylabel('Mass (kg)',           fontsize=style["fontsize"]["ylabel"])
plt.title("Maximum Takeoff Mass", fontsize=style["fontsize"]["title"])
plt.legend(loc="lower left",      fontsize=style["fontsize"]["legend"], framealpha=1)


# Mission time
plt.subplot(1,3,2)
for i,config in enumerate(cases):

	for j, case in enumerate(cases[config]):

		offset    = style["offsets"][j]
		solution  = cases[config][case]["solution"]
		t_flight  = solution("t_{flight}_OnDemandRevenueMission").to(ureg.min).magnitude
		t_mission = solution("t_{mission}_OnDemandRevenueMission").to(ureg.min).magnitude

		label = "Flight time = %0.1f min" % t_flight

		if i==0:
			plt.bar(i+offset, t_mission, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k', label=label)
		else:
			plt.bar(i+offset, t_mission, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.0*ymax)
plt.grid()
plt.xticks(y_pos, labels,    fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                  fontsize=style["fontsize"]["yticks"])
plt.ylabel('Time (minutes)', fontsize=style["fontsize"]["ylabel"])
plt.title("Mission Time",    fontsize=style["fontsize"]["title"])
plt.legend(loc="lower left", fontsize=style["fontsize"]["legend"], framealpha=1)


# Trip cost
plt.subplot(1,3,3)
for i,config in enumerate(cases):

	for j, case in enumerate(cases[config]):

		offset   = style["offsets"][j]
		solution = cases[config][case]["solution"]
		cpt      = solution("cost_per_trip")

		plt.bar(i+offset, cpt, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.0*ymax)
plt.grid()
plt.xticks(y_pos, labels, fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(               fontsize=style["fontsize"]["yticks"])
plt.ylabel('Cost ($US)',  fontsize=style["fontsize"]["ylabel"])
plt.title("Trip Cost",    fontsize=style["fontsize"]["title"])


plt.tight_layout()
plt.subplots_adjust(left=0.09, right=0.95, bottom=0.28, top=0.92)
plt.savefig('new_york_heli_plot_01.pdf')


"""
# Hover SPL (revenue mission)
plt.subplot(1,3,3)
for i,config in enumerate(cases):

	for j, case in enumerate(cases[config]):

		offset = style["offsets"][j]
		SPL_A  = cases[config][case]["SPL_revenue_A"]

		plt.bar(i+offset, SPL_A, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

SPL_req = 62
plt.plot([np.min(y_pos)-1, np.max(y_pos)+1], [SPL_req, SPL_req], color="black", linewidth=3, linestyle="--", label="62 dBA")

plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymin=57, ymax=ymax-1)
plt.grid()
plt.xticks(y_pos, labels,                  fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                                fontsize=style["fontsize"]["yticks"])
plt.ylabel('SPL (dBA)',                    fontsize=style["fontsize"]["ylabel"])
plt.title("Hover Sound (revenue mission)", fontsize=style["fontsize"]["title"])
plt.legend(loc="lower left",               fontsize=style["fontsize"]["legend"], framealpha=1)
"""

