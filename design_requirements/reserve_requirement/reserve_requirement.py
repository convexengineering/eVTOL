#Sensitivity to reserve requirement

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

#reserve_type = generic_data["reserve_type"]
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
sizing_mission_range = generic_data["sizing_mission"]["range"]
sizing_t_hover = generic_data["sizing_mission"]["t_{hover}"]

revenue_mission_type = generic_data["revenue_mission"]["type"]
revenue_N_passengers = generic_data["revenue_mission"]["N_passengers"]
revenue_mission_range = generic_data["revenue_mission"]["range"]
revenue_t_hover = generic_data["revenue_mission"]["t_{hover}"]

deadhead_mission_type = generic_data["deadhead_mission"]["type"]
deadhead_N_passengers = generic_data["deadhead_mission"]["N_passengers"]
deadhead_mission_range = generic_data["deadhead_mission"]["range"]
deadhead_t_hover = generic_data["deadhead_mission"]["t_{hover}"]


# Data specific to study
configs = OrderedDict()
reserve_type_array = ["Uber","FAA_day","FAA_night"]

for config in configuration_data:
	configs[config] = {}
	for reserve_type in reserve_type_array:
		configs[config][reserve_type] = configuration_data[config].copy()

#Delete unwanted configurations
del configs["Multirotor"]["Uber"]
del configs["Multirotor"]["FAA_day"]
del configs["Multirotor"]["FAA_night"]

del configs["Autogyro"]["Uber"]
del configs["Autogyro"]["FAA_day"]
del configs["Autogyro"]["FAA_night"]

del configs["Helicopter"]["FAA_day"]
del configs["Helicopter"]["FAA_night"]

del configs["Tilt duct"]["Uber"]
del configs["Tilt duct"]["FAA_day"]
del configs["Tilt duct"]["FAA_night"]

del configs["Coaxial heli"]["FAA_day"]
del configs["Coaxial heli"]["FAA_night"]


#Delete configurations that will not be evaluated
pared_configs = deepcopy(configs)
for config in configs:
	if configs[config] == {}:
		del pared_configs[config]
configs = deepcopy(pared_configs)

#Optimize remaining configurations
for config in configs:
	
	print "Solving configuration: " + config

	for reserve_type in configs[config]:
		
		c = configs[config][reserve_type]

		V_cruise = c["V_{cruise}"]
		L_D_cruise = c["L/D"]
		T_A = c["T/A"]
		Cl_mean_max = c["Cl_{mean_{max}}"]
		N = c["N"]
		loiter_type = c["loiter_type"]

		Aircraft = OnDemandAircraft(N=N,L_D_cruise=L_D_cruise,eta_cruise=eta_cruise,C_m=C_m,
			Cl_mean_max=Cl_mean_max,weight_fraction=weight_fraction,n=n,eta_electric=eta_electric,
			cost_per_weight=vehicle_cost_per_weight,cost_per_C=battery_cost_per_C,
			autonomousEnabled=autonomousEnabled)

		SizingMission = OnDemandSizingMission(Aircraft,mission_range=sizing_mission_range,
			V_cruise=V_cruise,N_passengers=sizing_N_passengers,t_hover=sizing_t_hover,
			reserve_type=reserve_type,mission_type=sizing_mission_type,loiter_type=loiter_type)
		SizingMission.substitutions.update({SizingMission.fs0.topvar("T/A"):T_A})
	
		RevenueMission = OnDemandRevenueMission(Aircraft,mission_range=revenue_mission_range,
			V_cruise=V_cruise,N_passengers=revenue_N_passengers,t_hover=revenue_t_hover,
			charger_power=charger_power,mission_type=revenue_mission_type)

		DeadheadMission = OnDemandDeadheadMission(Aircraft,mission_range=deadhead_mission_range,
			V_cruise=V_cruise,N_passengers=deadhead_N_passengers,t_hover=deadhead_t_hover,
			charger_power=charger_power,mission_type=deadhead_mission_type)

		MissionCost = OnDemandMissionCost(Aircraft,RevenueMission,DeadheadMission,
			pilot_wrap_rate=pilot_wrap_rate,mechanic_wrap_rate=mechanic_wrap_rate,MMH_FH=MMH_FH,
			deadhead_ratio=deadhead_ratio)
	
		problem = Model(MissionCost["cost_per_trip"],
			[Aircraft, SizingMission, RevenueMission, DeadheadMission, MissionCost])
	
		solution = problem.solve(verbosity=0)

		configs[config][reserve_type]["solution"] = solution

		configs[config][reserve_type]["MTOW"] = solution("MTOW_OnDemandAircraft")
		configs[config][reserve_type]["W_{battery}"] = solution("W_OnDemandAircraft/Battery")
		configs[config][reserve_type]["cost_per_trip_per_passenger"] = solution("cost_per_trip_per_passenger_OnDemandMissionCost")
		configs[config][reserve_type]["SPL"] = 20*np.log10(solution("p_{ratio}_OnDemandSizingMission")[0])



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
		MTOW = c["MTOW"].to(ureg.lbf).magnitude

		if (i == 0):
			if (reserve_type == "Uber"):
				label = reserve_type + " (2-nm diversion)"
			elif (reserve_type == "FAA_day"):
				label = "FAA day VFR (30-min loiter)"
			elif (reserve_type == "FAA_night"):
				label = "FAA night VFR (45-min loiter)"

			plt.bar(i+offset,MTOW,align='center',alpha=1,width=width,color=colors[j],
				label=label)
		else:
			plt.bar(i+offset,MTOW,align='center',alpha=1,width=width,color=colors[j])

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
plt.ylim(ymax=14000)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Maximum Takeoff Weight",fontsize = 18)
plt.legend(loc='upper right', fontsize = 12)


#Battery weight
plt.subplot(2,2,2)
for i,config in enumerate(configs):
	for j,reserve_type in enumerate(configs[config]):
		c = configs[config][reserve_type]
		offset = offset_array[j]
		W_battery = c["W_{battery}"].to(ureg.lbf).magnitude

		if (i == 0):
			if (reserve_type == "Uber"):
				label = reserve_type + " (2-nm diversion)"
			elif (reserve_type == "FAA_day"):
				label = "FAA day VFR (30-min loiter)"
			elif (reserve_type == "FAA_night"):
				label = "FAA night VFR (45-min loiter)"

			plt.bar(i+offset,W_battery,align='center',alpha=1,width=width,color=colors[j],
				label=label)
		else:
			plt.bar(i+offset,W_battery,align='center',alpha=1,width=width,color=colors[j])

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
plt.ylim(ymax=5000)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Battery Weight",fontsize = 18)
plt.legend(loc='upper right', fontsize = 12)


#Trip cost per passenger 
plt.subplot(2,2,3)
for i,config in enumerate(configs):
	for j,reserve_type in enumerate(configs[config]):
		c = configs[config][reserve_type]
		offset = offset_array[j]
		cptpp = c["cost_per_trip_per_passenger"]

		if (i == 0):
			if (reserve_type == "Uber"):
				label = reserve_type + " (2-nm diversion)"
			elif (reserve_type == "FAA_day"):
				label = "FAA day VFR (30-min loiter)"
			elif (reserve_type == "FAA_night"):
				label = "FAA night VFR (45-min loiter)"

			plt.bar(i+offset,cptpp,align='center',alpha=1,width=width,color=colors[j],
				label=label)
		else:
			plt.bar(i+offset,cptpp,align='center',alpha=1,width=width,color=colors[j])

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
plt.ylim(ymax=300)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Cost per Trip, per Passenger",fontsize = 18)
plt.legend(loc='upper right', fontsize = 12)


#Sound pressure level (in hover) 
plt.subplot(2,2,4)
for i,config in enumerate(configs):
	for j,reserve_type in enumerate(configs[config]):
		c = configs[config][reserve_type]
		offset = offset_array[j]
		SPL_sizing = c["SPL"]

		if (i == 0):
			if (reserve_type == "Uber"):
				label = reserve_type + " (2-nm diversion)"
			elif (reserve_type == "FAA_day"):
				label = "FAA day VFR (30-min loiter)"
			elif (reserve_type == "FAA_night"):
				label = "FAA night VFR (45-min loiter)"

			plt.bar(i+offset,SPL_sizing,align='center',alpha=1,width=width,color=colors[j],
				label=label)
		else:
			plt.bar(i+offset,SPL_sizing,align='center',alpha=1,width=width,color=colors[j])

SPL_req = 62
plt.plot([np.min(y_pos)-1,np.max(y_pos)+1],[SPL_req, SPL_req],
	color="black", linewidth=3, linestyle="-")
plt.ylim(ymin = 57,ymax = 85)
plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('SPL (dB)', fontsize = 16)
plt.title("Sound Pressure Level in Hover",fontsize = 18)
plt.legend(loc='upper right', fontsize = 12)


if autonomousEnabled:
	autonomy_string = "autonomy enabled"
else:
	autonomy_string = "pilot required"

title_str = "Aircraft parameters: structural mass fraction = %0.2f; battery energy density = %0.0f Wh/kg; %s\n" \
	% (weight_fraction, C_m.to(ureg.Wh/ureg.kg).magnitude, autonomy_string) \
	+ "Sizing mission (%s): range = %0.0f nm; %0.0f passengers; %0.0fs hover time\n" \
	% (sizing_mission_type, sizing_mission_range.to(ureg.nautical_mile).magnitude, sizing_N_passengers,\
		sizing_t_hover.to(ureg.s).magnitude) \
	+ "Revenue mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (revenue_mission_type, revenue_mission_range.to(ureg.nautical_mile).magnitude, \
		revenue_N_passengers, revenue_t_hover.to(ureg.s).magnitude, charger_power.to(ureg.kW).magnitude) \
	+ "Deadhead mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; deadhead ratio = %0.1f" \
	% (deadhead_mission_type, deadhead_mission_range.to(ureg.nautical_mile).magnitude, \
		deadhead_N_passengers, deadhead_t_hover.to(ureg.s).magnitude, deadhead_ratio)

plt.suptitle(title_str,fontsize = 14)

plt.tight_layout()#makes sure subplots are spaced neatly
plt.subplots_adjust(left=0.08,right=0.96,bottom=0.10,top=0.87)#adds space at the top for the title
