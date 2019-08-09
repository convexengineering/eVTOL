#Sensitivity to reserve requirement

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

	cases[config]["2-nmi diversion"]  = {}
	cases[config]["20-minute loiter"] = {}
	cases[config]["30-minute loiter"] = {}


# Optimize
for config in cases:
	
	print "Solving configuration: " + config

	for i, reserve in enumerate(cases[config]):
		
		c = cases[config][reserve]

		aircraft = OnDemandAircraft()
		aircraft = aircraft.standard_substitutions(config=config, autonomousEnabled=generic_data["autonomousEnabled"])

		sizing_mission = OnDemandSizingMission(aircraft=aircraft)
		sizing_mission = sizing_mission.standard_substitutions(piloted=generic_data["isSizingMissionPiloted"], reserve=reserve)
		
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

	for j, reserve in enumerate(cases[config]):

		offset   = style["offsets"][j]
		solution = cases[config][reserve]["solution"]
		MTOM     = solution("MTOM").to(ureg.kg).magnitude

		if i==0:
			plt.bar(i+offset, MTOM, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k', label=reserve)
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


# Cost per passenger-km
plt.subplot(1,3,2)
for i,config in enumerate(cases):

	for j, reserve in enumerate(cases[config]):

		offset   = style["offsets"][j]
		solution = cases[config][reserve]["solution"]
		cppk     = solution("cost_per_passenger_km").to(ureg.km**-1).magnitude

		plt.bar(i+offset, cppk, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.0*ymax)
plt.grid()
plt.xticks(y_pos, labels,                 fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                               fontsize=style["fontsize"]["yticks"])
plt.ylabel('Cost ($US/km)',               fontsize=style["fontsize"]["ylabel"])
plt.title("Cost per Passenger Kilometer", fontsize=style["fontsize"]["title"])


# Hover SPL (revenue mission)
plt.subplot(1,3,3)
for i,config in enumerate(cases):

	for j, reserve in enumerate(cases[config]):

		offset       = style["offsets"][j]
		SPL_sizing_A = cases[config][reserve]["SPL_sizing_A"]

		plt.bar(i+offset, SPL_sizing_A, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

SPL_req = 62
plt.plot([np.min(y_pos)-1, np.max(y_pos)+1], [SPL_req, SPL_req], color="black", linewidth=3, linestyle="--", label="62 dBA")

plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymin=57, ymax=ymax-1)
plt.grid()
plt.xticks(y_pos, labels,                 fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                               fontsize=style["fontsize"]["yticks"])
plt.ylabel('SPL (dBA)',                   fontsize=style["fontsize"]["ylabel"])
plt.title("Hover Sound (sizing mission)", fontsize=style["fontsize"]["title"])
plt.legend(loc="lower left",              fontsize=style["fontsize"]["legend"], framealpha=1)

plt.tight_layout()
plt.subplots_adjust(left=0.09, right=0.95, bottom=0.28, top=0.92)
plt.savefig('reserve_requirement_plot_01.pdf')




















		
"""

# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(12,12), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

y_pos = np.arange(len(configs))
labels = [""]*len(configs)
for i, config in enumerate(configs):
	labels[i] = config

xmin = np.min(y_pos) - 0.7
xmax = np.max(y_pos) + 0.7

offset_array = [-0.3,0,0.3]
width = 0.2
colors = ["grey", "w", "k"]

#Maximum takeoff weight
plt.subplot(2,2,1)
for i,config in enumerate(configs):
	for j,reserve_type in enumerate(configs[config]):
		c = configs[config][reserve_type]
		offset = offset_array[j]
		TOGW = c["TOGW"].to(ureg.lbf).magnitude

		if (i == 0):
			if (reserve_type == "Uber"):
				label = reserve_type + " (2-nmi diversion)"
			elif (reserve_type == "FAA_heli"):
				label = "FAA helicopter VFR (20-min loiter)"
			elif (reserve_type == "FAA_aircraft"):
				label = "FAA aircraft VFR (30-min loiter)"

			plt.bar(i+offset,TOGW,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,TOGW,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.3*ymax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Maximum Takeoff Weight",fontsize = 18)
plt.legend(loc='upper right', fontsize = 12,framealpha=1)


#Battery weight
plt.subplot(2,2,2)
for i,config in enumerate(configs):
	for j,reserve_type in enumerate(configs[config]):
		c = configs[config][reserve_type]
		offset = offset_array[j]
		W_battery = c["W_{battery}"].to(ureg.lbf).magnitude

		if (i == 0):
			if (reserve_type == "Uber"):
				label = reserve_type + " (2-nmi diversion)"
			elif (reserve_type == "FAA_heli"):
				label = "FAA helicopter VFR (20-min loiter)"
			elif (reserve_type == "FAA_aircraft"):
				label = "FAA aircraft VFR (30-min loiter)"

			plt.bar(i+offset,W_battery,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,W_battery,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.25*ymax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Battery Weight",fontsize = 18)
plt.legend(loc='upper right', fontsize = 12,framealpha=1)


#Trip cost per passenger 
plt.subplot(2,2,3)
for i,config in enumerate(configs):
	for j,reserve_type in enumerate(configs[config]):
		c = configs[config][reserve_type]
		offset = offset_array[j]
		cptpp = c["cost_per_trip_per_passenger"]

		if (i == 0):
			if (reserve_type == "Uber"):
				label = reserve_type + " (2-nmi diversion)"
			elif (reserve_type == "FAA_heli"):
				label = "FAA helicopter VFR (20-min loiter)"
			elif (reserve_type == "FAA_aircraft"):
				label = "FAA aircraft VFR (30-min loiter)"

			plt.bar(i+offset,cptpp,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,cptpp,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.25*ymax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Cost per Trip, per Passenger",fontsize = 18)
plt.legend(loc='upper right', fontsize = 12,framealpha=1)


#Sound pressure level (in hover) 
plt.subplot(2,2,4)
for i,config in enumerate(configs):
	for j,reserve_type in enumerate(configs[config]):
		c = configs[config][reserve_type]
		offset = offset_array[j]
		SPL_sizing = c["SPL_A"]

		if (i == 0):
			if (reserve_type == "Uber"):
				label = reserve_type + " (2-nmi diversion)"
			elif (reserve_type == "FAA_heli"):
				label = "FAA helicopter VFR (20-min loiter)"
			elif (reserve_type == "FAA_aircraft"):
				label = "FAA aircraft VFR (30-min loiter)"

			plt.bar(i+offset,SPL_sizing,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,SPL_sizing,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

SPL_req = 62
plt.plot([np.min(y_pos)-1,np.max(y_pos)+1],[SPL_req, SPL_req],
	color="black", linewidth=3, linestyle="-")

plt.ylim(ymin = 57,ymax = 80)
plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('SPL (dBA)', fontsize = 16)
plt.title("Sound Pressure Level in Hover",fontsize = 18)
plt.legend(loc='upper right', fontsize = 12,framealpha=1)

if generic_data["autonomousEnabled"]:
	autonomy_string = "autonomy enabled"
else:
	autonomy_string = "pilot required"

title_str = "Aircraft parameters: battery energy density = %0.0f Wh/kg; %0.0f rotor blades; %s\n" \
	% (generic_data["C_m"].to(ureg.Wh/ureg.kg).magnitude, B, autonomy_string) \
	+ "Sizing mission (%s): range = %0.0f nmi; %0.0f passengers; %0.0fs hover time" \
	% (generic_data["sizing_mission"]["type"], generic_data["sizing_mission"]["range"].to(ureg.nautical_mile).magnitude,\
	 generic_data["sizing_mission"]["N_{passengers}"], generic_data["sizing_mission"]["t_{hover}"].to(ureg.s).magnitude)\
	+ "\n"\
	+ "Revenue mission (%s): range = %0.0f nmi; %0.1f passengers; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (generic_data["revenue_mission"]["type"], generic_data["revenue_mission"]["range"].to(ureg.nautical_mile).magnitude, \
	 generic_data["revenue_mission"]["N_{passengers}"], generic_data["revenue_mission"]["t_{hover}"].to(ureg.s).magnitude,\
	 generic_data["charger_power"].to(ureg.kW).magnitude) \
	+ "Deadhead mission (%s): range = %0.0f nmi; %0.1f passengers; %0.0fs hover time; no reserve; deadhead ratio = %0.1f" \
	% (generic_data["deadhead_mission"]["type"], generic_data["deadhead_mission"]["range"].to(ureg.nautical_mile).magnitude, \
	 generic_data["deadhead_mission"]["N_{passengers}"], generic_data["deadhead_mission"]["t_{hover}"].to(ureg.s).magnitude,\
	 generic_data["deadhead_ratio"])

plt.suptitle(title_str,fontsize = 13.5)
plt.tight_layout()
plt.subplots_adjust(left=0.08,right=0.96,bottom=0.10,top=0.87)

"""