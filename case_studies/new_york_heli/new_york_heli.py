#Sensitivity to sizing assumptions for New York airport shuttle service

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../..'))

import numpy as np
from gpkit import Model, ureg
from matplotlib import pyplot as plt
from aircraft_models import OnDemandAircraft 
from aircraft_models import OnDemandSizingMission, OnDemandRevenueMission
from aircraft_models import OnDemandDeadheadMission, OnDemandMissionCost
from study_input_data import generic_data, configuration_data
from copy import deepcopy
from collections import OrderedDict

#General data
eta_cruise = generic_data["\eta_{cruise}"] 
eta_electric = generic_data["\eta_{electric}"]
weight_fraction = generic_data["weight_fraction"]
C_m = generic_data["C_m"]
n = generic_data["n"]

reserve_type = generic_data["reserve_type"]
autonomousEnabled = generic_data["autonomousEnabled"]
charger_power = generic_data["charger_power"]

vehicle_cost_per_weight = generic_data["vehicle_cost_per_weight"]
battery_cost_per_C = generic_data["battery_cost_per_C"]
pilot_wrap_rate = generic_data["pilot_wrap_rate"]
mechanic_wrap_rate = generic_data["mechanic_wrap_rate"]
MMH_FH = generic_data["MMH_FH"]
deadhead_ratio = generic_data["deadhead_ratio"]

sizing_mission_type = generic_data["sizing_mission"]["type"]
sizing_N_passengers = generic_data["sizing_mission"]["N_passengers"]
#sizing_mission_range = generic_data["sizing_mission"]["range"]
sizing_t_hover = generic_data["sizing_mission"]["t_{hover}"]

revenue_mission_type = generic_data["revenue_mission"]["type"]
revenue_N_passengers = generic_data["revenue_mission"]["N_passengers"]
#revenue_mission_range = generic_data["revenue_mission"]["range"]
revenue_t_hover = generic_data["revenue_mission"]["t_{hover}"]

deadhead_mission_type = generic_data["deadhead_mission"]["type"]
deadhead_N_passengers = generic_data["deadhead_mission"]["N_passengers"]
#deadhead_mission_range = generic_data["deadhead_mission"]["range"]
deadhead_t_hover = generic_data["deadhead_mission"]["t_{hover}"]


# Data specific to study
configs = OrderedDict()
case_array = ["Case 1","Case 2","Case 3"]
sizing_mission_range_array = [15, 30, 30]*ureg.nautical_mile
revenue_mission_range_array = [15, 15, 30]*ureg.nautical_mile
deadhead_mission_range_array = [15, 15, 30]*ureg.nautical_mile

for config in configuration_data:
	configs[config] = OrderedDict()
	for case in case_array:
		configs[config][case] = configuration_data[config].copy()

#Delete unwanted configurations
del configs["Multirotor"]["Case 1"]
del configs["Multirotor"]["Case 2"]
del configs["Multirotor"]["Case 3"]

del configs["Autogyro"]["Case 1"]
del configs["Autogyro"]["Case 2"]
del configs["Autogyro"]["Case 3"]

del configs["Helicopter"]["Case 1"]
del configs["Helicopter"]["Case 2"]
del configs["Helicopter"]["Case 3"]

del configs["Tilt duct"]["Case 1"]
del configs["Tilt duct"]["Case 2"]
del configs["Tilt duct"]["Case 3"]

del configs["Coaxial heli"]["Case 1"]
del configs["Coaxial heli"]["Case 2"]
del configs["Coaxial heli"]["Case 3"]


#Delete configurations that will not be evaluated
pared_configs = deepcopy(configs)
for config in configs:
	if configs[config] == {}:
		del pared_configs[config]
configs = deepcopy(pared_configs)

#Optimize remaining configurations
for config in configs:
	
	print "Solving configuration: " + config

	for i, case in enumerate(configs[config]):
		
		c = configs[config][case]

		V_cruise = c["V_{cruise}"]
		L_D_cruise = c["L/D"]
		T_A = c["T/A"]
		Cl_mean_max = c["Cl_{mean_{max}}"]
		N = c["N"]
		loiter_type = c["loiter_type"]
		tailRotor_power_fraction_hover = c["tailRotor_power_fraction_hover"]
		tailRotor_power_fraction_levelFlight = c["tailRotor_power_fraction_levelFlight"]

		sizing_mission_range = sizing_mission_range_array[i]
		revenue_mission_range = revenue_mission_range_array[i]
		deadhead_mission_range = deadhead_mission_range_array[i]

		Aircraft = OnDemandAircraft(N=N,L_D_cruise=L_D_cruise,eta_cruise=eta_cruise,C_m=C_m,
			Cl_mean_max=Cl_mean_max,weight_fraction=weight_fraction,n=n,eta_electric=eta_electric,
			cost_per_weight=vehicle_cost_per_weight,cost_per_C=battery_cost_per_C,
			autonomousEnabled=autonomousEnabled)

		SizingMission = OnDemandSizingMission(Aircraft,mission_range=sizing_mission_range,
			V_cruise=V_cruise,N_passengers=sizing_N_passengers,t_hover=sizing_t_hover,
			reserve_type=reserve_type,mission_type=sizing_mission_type,loiter_type=loiter_type,
			tailRotor_power_fraction_hover=tailRotor_power_fraction_hover,
			tailRotor_power_fraction_levelFlight=tailRotor_power_fraction_levelFlight)
		SizingMission.substitutions.update({SizingMission.fs0.topvar("T/A"):T_A})
	
		RevenueMission = OnDemandRevenueMission(Aircraft,mission_range=revenue_mission_range,
			V_cruise=V_cruise,N_passengers=revenue_N_passengers,t_hover=revenue_t_hover,
			charger_power=charger_power,mission_type=revenue_mission_type,
			tailRotor_power_fraction_hover=tailRotor_power_fraction_hover,
			tailRotor_power_fraction_levelFlight=tailRotor_power_fraction_levelFlight)

		DeadheadMission = OnDemandDeadheadMission(Aircraft,mission_range=deadhead_mission_range,
			V_cruise=V_cruise,N_passengers=deadhead_N_passengers,t_hover=deadhead_t_hover,
			charger_power=charger_power,mission_type=deadhead_mission_type,
			tailRotor_power_fraction_hover=tailRotor_power_fraction_hover,
			tailRotor_power_fraction_levelFlight=tailRotor_power_fraction_levelFlight)

		MissionCost = OnDemandMissionCost(Aircraft,RevenueMission,DeadheadMission,
			pilot_wrap_rate=pilot_wrap_rate,mechanic_wrap_rate=mechanic_wrap_rate,MMH_FH=MMH_FH,
			deadhead_ratio=deadhead_ratio)
	
		problem = Model(MissionCost["cost_per_trip"],
			[Aircraft, SizingMission, RevenueMission, DeadheadMission, MissionCost])
	
		solution = problem.solve(verbosity=0)

		configs[config][case]["solution"] = solution

		configs[config][case]["MTOW"] = solution("MTOW_OnDemandAircraft")
		configs[config][case]["W_{battery}"] = solution("W_OnDemandAircraft/Battery")
		configs[config][case]["cost_per_trip_per_passenger"] = solution("cost_per_trip_per_passenger_OnDemandMissionCost")
		configs[config][case]["SPL_sizing"] = 20*np.log10(solution("p_{ratio}_OnDemandSizingMission")[0])
		configs[config][case]["SPL_revenue"] = 20*np.log10(solution("p_{ratio}_OnDemandRevenueMission")[0])



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

legend_labels = [""]*np.size(sizing_mission_range_array)
for i,val in enumerate(legend_labels):
	legend_labels[i] = "%0.0f nm sizing mission; %0.0f nm revenue mission" \
		% (sizing_mission_range_array[i].to(ureg.nautical_mile).magnitude,
			revenue_mission_range_array[i].to(ureg.nautical_mile).magnitude)
	if revenue_mission_range_array[i] != deadhead_mission_range_array[i]:
		raise ValueError("Revenue and deadhead missions must be the same length.")


#Maximum takeoff weight
plt.subplot(2,2,1)
for i,config in enumerate(configs):
	for j,case in enumerate(configs[config]):
		c = configs[config][case]
		offset = offset_array[j]
		MTOW = c["MTOW"].to(ureg.lbf).magnitude

		if (i == 0):
			label = legend_labels[j]
			plt.bar(i+offset,MTOW,align='center',alpha=1,width=width,color=colors[j],
				label=label)
		else:
			plt.bar(i+offset,MTOW,align='center',alpha=1,width=width,color=colors[j])

plt.grid()
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.3*ymax)
plt.title("Maximum Takeoff Weight",fontsize = 18)
plt.legend(loc='upper left', fontsize = 12)


#Trip cost per passenger 
plt.subplot(2,2,2)
for i,config in enumerate(configs):
	for j,case in enumerate(configs[config]):
		c = configs[config][case]
		offset = offset_array[j]
		cptpp = c["cost_per_trip_per_passenger"]

		if (i == 0):
			label = legend_labels[j]
			plt.bar(i+offset,cptpp,align='center',alpha=1,width=width,color=colors[j],
				label=label)
		else:
			plt.bar(i+offset,cptpp,align='center',alpha=1,width=width,color=colors[j])

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.3*ymax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Cost per Trip, per Passenger",fontsize = 18)
plt.legend(loc='upper left', fontsize = 12)


#Sound pressure level (in hover, sizing mission) 
plt.subplot(2,2,3)
for i,config in enumerate(configs):
	for j,case in enumerate(configs[config]):
		c = configs[config][case]
		offset = offset_array[j]
		SPL_sizing = c["SPL_sizing"]

		if (i == 0):
			label = legend_labels[j]
			plt.bar(i+offset,SPL_sizing,align='center',alpha=1,width=width,color=colors[j],
				label=label)
		else:
			plt.bar(i+offset,SPL_sizing,align='center',alpha=1,width=width,color=colors[j])

SPL_req = 62
plt.plot([np.min(y_pos)-1,np.max(y_pos)+1],[SPL_req, SPL_req],
	color="black", linewidth=3, linestyle="-")
plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymin = 57,ymax = ymax + 1)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('SPL (dB)', fontsize = 16)
plt.title("Sound Pressure Level (sizing mission)",fontsize = 18)
plt.legend(loc='upper left', fontsize = 12)

#Sound pressure level (in hover, sizing mission) 
plt.subplot(2,2,4)
for i,config in enumerate(configs):
	for j,case in enumerate(configs[config]):
		c = configs[config][case]
		offset = offset_array[j]
		SPL_revenue = c["SPL_revenue"]

		if (i == 0):
			label = legend_labels[j]
			plt.bar(i+offset,SPL_revenue,align='center',alpha=1,width=width,color=colors[j],
				label=label)
		else:
			plt.bar(i+offset,SPL_revenue,align='center',alpha=1,width=width,color=colors[j])

SPL_req = 62
plt.plot([np.min(y_pos)-1,np.max(y_pos)+1],[SPL_req, SPL_req],
	color="black", linewidth=3, linestyle="-")
plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymin = 57,ymax = ymax + 1)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('SPL (dB)', fontsize = 16)
plt.title("Sound Pressure Level (revenue mission)",fontsize = 18)
plt.legend(loc='upper left', fontsize = 12)


if reserve_type == "FAA_day" or reserve_type == "FAA_night":
	num = solution("t_{loiter}_OnDemandSizingMission").to(ureg.minute).magnitude
	if reserve_type == "FAA_day":
		reserve_type_string = "FAA day VFR (%0.0f-minute loiter time)" % num
	elif reserve_type == "FAA_night":
		reserve_type_string = "FAA night VFR (%0.0f-minute loiter time)" % num
elif reserve_type == "Uber":
	num = solution["constants"]["R_{divert}_OnDemandSizingMission"].to(ureg.nautical_mile).magnitude
	reserve_type_string = " (%0.0f-nm diversion distance)" % num


if autonomousEnabled:
	autonomy_string = "autonomy enabled"
else:
	autonomy_string = "pilot required"

title_str = "Aircraft parameters: structural mass fraction = %0.2f; battery energy density = %0.0f Wh/kg; %s\n" \
	% (weight_fraction, C_m.to(ureg.Wh/ureg.kg).magnitude, autonomy_string) \
	+ "Sizing mission (%s): %0.0f passengers; %0.0fs hover time; reserve type = " \
	% (sizing_mission_type, sizing_N_passengers, sizing_t_hover.to(ureg.s).magnitude) \
	+ reserve_type_string + "\n"\
	+ "Revenue mission (%s): %0.1f passengers; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (revenue_mission_type, revenue_N_passengers, revenue_t_hover.to(ureg.s).magnitude, charger_power.to(ureg.kW).magnitude) \
	+ "Deadhead mission (%s): %0.1f passengers; %0.0fs hover time; no reserve; deadhead ratio = %0.1f" \
	% (deadhead_mission_type, deadhead_N_passengers, deadhead_t_hover.to(ureg.s).magnitude, deadhead_ratio)

plt.suptitle(title_str,fontsize = 14)
plt.tight_layout()
plt.subplots_adjust(left=0.08,right=0.98,bottom=0.10,top=0.87)
plt.savefig('new_york_heli_plot_01.pdf')
