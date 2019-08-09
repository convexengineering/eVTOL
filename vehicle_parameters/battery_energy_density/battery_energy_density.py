# Sweep to evaluate the sensitivity of each design to battery energy density

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


num_pts = 8

#Optimize
for config in configs:

	print "Solving configuration: " + config

	#set up e arrays (different for each configuration)
	if config == "Helicopter":
		e = np.linspace(640, 700, num_pts) * ureg.Wh/ureg.kg
	elif config == "Coaxial heli":
		e = np.linspace(570, 700, num_pts) * ureg.Wh/ureg.kg
	elif config == "Lift + cruise":
		e = np.linspace(290, 700, num_pts) * ureg.Wh/ureg.kg
	elif config == "Compound heli":
		e = np.linspace(300, 700, num_pts) * ureg.Wh/ureg.kg
	elif config == "Tilt rotor":
		e = np.linspace(250, 700, num_pts) * ureg.Wh/ureg.kg
	elif config == "Tilt wing":
		e = np.linspace(270, 700, num_pts) * ureg.Wh/ureg.kg

	c                 = configs[config]
	c["SPL_sizing_A"] = np.zeros(np.size(e))

	aircraft = OnDemandAircraft()
	aircraft = aircraft.standard_substitutions(config=config, autonomousEnabled=generic_data["autonomousEnabled"])

	aircraft.substitutions.update({
		aircraft.battery.e: ("sweep", e.to(ureg.Wh/ureg.kg).magnitude)  # Watch the units here.
	})

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
	
	c["solution"] = solution

	for i in range(len(e)):

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
fig1 = plt.figure(figsize=(11,4), dpi=80)
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
plt.subplot(1,3,1)
for i, config in enumerate(configs):

	solution = configs[config]["solution"]
	e        = solution("e_OnDemandAircraft/Battery").to(ureg.Wh/ureg.kg).magnitude
	MTOM     = solution("MTOM_OnDemandAircraft").to(ureg.kg).magnitude

	plt.plot(e, MTOM, color="black", linewidth=1.5,linestyle=style["linestyle"][i], marker=style["marker"][i],
		fillstyle=style["fillstyle"][i], markersize=style["markersize"], label=config)

plt.grid()
[ymin, ymax] = plt.gca().get_ylim()
plt.ylim(ymin=0, ymax=1.1*ymax)
plt.xticks(                          fontsize=style["fontsize"]["xticks"])
plt.yticks(                          fontsize=style["fontsize"]["yticks"])
plt.xlabel('Energy density (Wh/kg)', fontsize=style["fontsize"]["xlabel"])
plt.ylabel('Mass (kg)',              fontsize=style["fontsize"]["ylabel"])
plt.title("Maximum Takeoff Mass",    fontsize=style["fontsize"]["title"])
plt.legend(loc='lower left',         fontsize=style["fontsize"]["legend"], framealpha=1, numpoints=1)


# Trip cost per passenger-km
plt.subplot(1,3,2)
for i, config in enumerate(configs):

	solution = configs[config]["solution"]
	e        = solution("e_OnDemandAircraft/Battery").to(ureg.Wh/ureg.kg).magnitude
	cppk     = solution("cost_per_passenger_km").to(ureg.km**-1).magnitude

	plt.plot(e, cppk, color="black", linewidth=1.5,linestyle=style["linestyle"][i], marker=style["marker"][i],
		fillstyle=style["fillstyle"][i], markersize=style["markersize"], label=config)

plt.grid()
[ymin, ymax] = plt.gca().get_ylim()
plt.ylim(ymin=0, ymax=1.1*ymax)
plt.xticks(                               fontsize=style["fontsize"]["xticks"])
plt.yticks(                               fontsize=style["fontsize"]["yticks"])
plt.xlabel('Energy density (Wh/kg)',      fontsize=style["fontsize"]["xlabel"])
plt.ylabel('Cost ($US/km)',               fontsize=style["fontsize"]["ylabel"])
plt.title("Cost per Passenger Kilometer", fontsize=style["fontsize"]["title"])


# Sound in hover
plt.subplot(1,3,3)
for i, config in enumerate(configs):

	solution     = configs[config]["solution"]
	e            = solution("e_OnDemandAircraft/Battery").to(ureg.Wh/ureg.kg).magnitude
	SPL_sizing_A = configs[config]["SPL_sizing_A"]

	plt.plot(e, SPL_sizing_A, color="black", linewidth=1.5,linestyle=style["linestyle"][i], marker=style["marker"][i],
		fillstyle=style["fillstyle"][i], markersize=style["markersize"], label=config)

plt.grid()
[ymin, ymax] = plt.gca().get_ylim()
plt.ylim(ymin=0.95*ymin, ymax=1.01*ymax)
plt.xticks(                               fontsize=style["fontsize"]["xticks"])
plt.yticks(                               fontsize=style["fontsize"]["yticks"])
plt.xlabel('Energy density (Wh/kg)',      fontsize=style["fontsize"]["xlabel"])
plt.ylabel('SPL (dBA)',                   fontsize=style["fontsize"]["ylabel"])
plt.title("Hover Sound (sizing mission)", fontsize=style["fontsize"]["title"])


plt.tight_layout()
plt.subplots_adjust(left=0.09, right=0.95, bottom=0.16, top=0.92)
plt.savefig('battery_energy_density_plot_01.pdf')