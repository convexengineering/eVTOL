#Sensitivity to number of passengers

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
#sizing_N_passengers = generic_data["sizing_mission"]["N_passengers"]
sizing_mission_range = generic_data["sizing_mission"]["range"]
sizing_time_in_hover = generic_data["sizing_mission"]["time_in_hover"]

revenue_mission_type = generic_data["revenue_mission"]["type"]
#revenue_N_passengers = generic_data["revenue_mission"]["N_passengers"]
revenue_mission_range = generic_data["revenue_mission"]["range"]
revenue_time_in_hover = generic_data["revenue_mission"]["time_in_hover"]

deadhead_mission_type = generic_data["deadhead_mission"]["type"]
#deadhead_N_passengers = generic_data["deadhead_mission"]["N_passengers"]
deadhead_mission_range = generic_data["deadhead_mission"]["range"]
deadhead_time_in_hover = generic_data["deadhead_mission"]["time_in_hover"]


# Delete some configurations
configs = configuration_data.copy()
del configs["Tilt duct"]
del configs["Multirotor"]
del configs["Autogyro"]

#Data specific to study
sizing_N_passengers_array = np.linspace(1,5,3)
passenger_ratio = 2./3
revenue_N_passengers_array = passenger_ratio*sizing_N_passengers_array
deadhead_N_passengers_array = passenger_ratio*sizing_N_passengers_array


#Optimize remaining configurations
for config in configs:
	
	print "Solving configuration: " + config

	c = configs[config]

	V_cruise = c["V_{cruise}"]
	L_D_cruise = c["L/D"]
	T_A = c["T/A"]
	Cl_mean_max = c["Cl_{mean_{max}}"]
	N = c["N"]

	configs[config]["MTOW"] = np.zeros(np.size(sizing_N_passengers_array))
	configs[config]["W_{battery}"] = np.zeros(np.size(sizing_N_passengers_array))
	configs[config]["cost_per_trip_per_passenger"] = np.zeros(np.size(sizing_N_passengers_array))
	configs[config]["SPL"] = np.zeros(np.size(sizing_N_passengers_array))

	for i,sizing_N_passengers in enumerate(sizing_N_passengers_array):
		revenue_N_passengers = revenue_N_passengers_array[i]
		deadhead_N_passengers = deadhead_N_passengers_array[i]

		Aircraft = OnDemandAircraft(N=N,L_D_cruise=L_D_cruise,eta_cruise=eta_cruise,C_m=C_m,
			Cl_mean_max=Cl_mean_max,weight_fraction=weight_fraction,n=n,eta_electric=eta_electric,
			cost_per_weight=vehicle_cost_per_weight,cost_per_C=battery_cost_per_C,
			autonomousEnabled=autonomousEnabled)

		SizingMission = OnDemandSizingMission(Aircraft,mission_range=sizing_mission_range,
			V_cruise=V_cruise,N_passengers=sizing_N_passengers,time_in_hover=sizing_time_in_hover,
			reserve_type=reserve_type,mission_type=sizing_mission_type)
		SizingMission.substitutions.update({SizingMission.fs0.topvar("T/A"):T_A})
	
		RevenueMission = OnDemandRevenueMission(Aircraft,mission_range=revenue_mission_range,
			V_cruise=V_cruise,N_passengers=revenue_N_passengers,time_in_hover=revenue_time_in_hover,
			charger_power=charger_power,mission_type=revenue_mission_type)

		DeadheadMission = OnDemandDeadheadMission(Aircraft,mission_range=deadhead_mission_range,
			V_cruise=V_cruise,N_passengers=deadhead_N_passengers,time_in_hover=deadhead_time_in_hover,
			charger_power=charger_power,mission_type=deadhead_mission_type)

		MissionCost = OnDemandMissionCost(Aircraft,RevenueMission,DeadheadMission,
			pilot_wrap_rate=pilot_wrap_rate,mechanic_wrap_rate=mechanic_wrap_rate,MMH_FH=MMH_FH,
			deadhead_ratio=deadhead_ratio)
	
		problem = Model(MissionCost["cost_per_trip"],
			[Aircraft, SizingMission, RevenueMission, DeadheadMission, MissionCost])
	
		solution = problem.solve(verbosity=0)

		configs[config]["MTOW"][i] = solution("MTOW_OnDemandAircraft").to(ureg.lbf).magnitude
		configs[config]["W_{battery}"][i] = solution("W_OnDemandAircraft/Battery").to(ureg.lbf).magnitude
		configs[config]["cost_per_trip_per_passenger"][i] = solution("cost_per_trip_per_passenger_OnDemandMissionCost")
		configs[config]["SPL"][i] = 20*np.log10(solution("p_{ratio}_OnDemandSizingMission"))

	configs[config]["MTOW"] = configs[config]["MTOW"]*ureg.lbf
	configs[config]["W_{battery}"] = configs[config]["W_{battery}"]*ureg.lbf



# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(12,12), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

y_pos = np.arange(len(configs))
labels = [""]*len(configs)
for i, config in enumerate(configs):
	labels[i] = config

offset_array = [-0.3,0,0.3]
width = 0.2
colors = ["grey", "w", "k"]

#Maximum takeoff weight
plt.subplot(2,2,1)
for i, config in enumerate(configs):
	c = configs[config]
	for j,offset in enumerate(offset_array):
		MTOW = c["MTOW"][j].to(ureg.lbf).magnitude

		if (i == 0):
			label = "%0.0f passengers" % sizing_N_passengers_array[j]
			plt.bar(i+offset,MTOW,align='center',alpha=1,width=width,color=colors[j],
				label=label)
		else:
			plt.bar(i+offset,MTOW,align='center',alpha=1,width=width,color=colors[j])


plt.grid()
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Maximum Takeoff Weight",fontsize = 18)
plt.legend(loc='upper left', fontsize = 12)




#Battery weight
plt.subplot(2,2,2)
for i, config in enumerate(configs):
	c = configs[config]
	for j,offset in enumerate(offset_array):
		W_battery = c["W_{battery}"][j].to(ureg.lbf).magnitude
		if (i == 0):
			label = "%0.0f passengers" % sizing_N_passengers_array[j]
			plt.bar(i+offset,W_battery,align='center',alpha=1,width=width,color=colors[j],
				label=label)
		else:
			plt.bar(i+offset,W_battery,align='center',alpha=1,width=width,color=colors[j])

plt.grid()
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Battery Weight",fontsize = 18)
plt.legend(loc='upper left', fontsize = 12)



#Trip cost per passenger 
plt.subplot(2,2,3)
for i, config in enumerate(configs):
	c = configs[config]
	for j,offset in enumerate(offset_array):
		cptpp = c["cost_per_trip_per_passenger"][j]
		if (i == 0):
			label = "%0.0f passengers" % sizing_N_passengers_array[j]
			plt.bar(i+offset,cptpp,align='center',alpha=1,width=width,color=colors[j],
				label=label)
		else:
			plt.bar(i+offset,cptpp,align='center',alpha=1,width=width,color=colors[j])

plt.grid()
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Cost per Trip, per Passenger",fontsize = 18)
plt.legend(loc='upper left', fontsize = 12)



#Sound pressure level (in hover) 
plt.subplot(2,2,4)
for i, config in enumerate(configs):
	c = configs[config]
	for j,offset in enumerate(offset_array):
		SPL_sizing = c["SPL"][j]
		if (i == 0):
			label = "%0.0f passengers" % sizing_N_passengers_array[j]
			plt.bar(i+offset,SPL_sizing,align='center',alpha=1,width=width,color=colors[j],
				label=label)
		else:
			plt.bar(i+offset,SPL_sizing,align='center',alpha=1,width=width,color=colors[j])

SPL_req = 62
plt.plot([np.min(y_pos)-1,np.max(y_pos)+1],[SPL_req, SPL_req],
	color="black", linewidth=3, linestyle="-")
plt.ylim(ymin = 57, ymax = 75)
plt.grid()
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('SPL (dB)', fontsize = 16)
plt.title("Sound Pressure Level in Hover",fontsize = 18)
plt.legend(loc='lower right', fontsize = 12)



if reserve_type == "FAA":
	num = solution["constants"]["t_{loiter}_OnDemandSizingMission"].to(ureg.minute).magnitude
	reserve_type_string = " (%0.0f-minute loiter time)" % num
if reserve_type == "Uber":
	num = solution["constants"]["R_{divert}_OnDemandSizingMission"].to(ureg.nautical_mile).magnitude
	reserve_type_string = " (%0.0f-nm diversion distance)" % num

if autonomousEnabled:
	autonomy_string = "autonomy enabled"
else:
	autonomy_string = "pilot required"

title_str = "Aircraft parameters: structural mass fraction = %0.2f; battery energy density = %0.0f Wh/kg; %s\n" \
	% (weight_fraction, C_m.to(ureg.Wh/ureg.kg).magnitude, autonomy_string) \
	+ "Sizing mission (%s): range = %0.0f nm; %0.0fs hover time; reserve type = " \
	% (sizing_mission_type, sizing_mission_range.to(ureg.nautical_mile).magnitude, sizing_time_in_hover.to(ureg.s).magnitude) \
	+ reserve_type + reserve_type_string + "\n"\
	+ "Revenue mission (%s): range = %0.0f nm; passenger ratio = %0.1f; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (revenue_mission_type, revenue_mission_range.to(ureg.nautical_mile).magnitude, \
		passenger_ratio, revenue_time_in_hover.to(ureg.s).magnitude, charger_power.to(ureg.kW).magnitude) \
	+ "Deadhead mission (%s): range = %0.0f nm; passenger ratio = %0.1f; %0.0fs hover time; no reserve; deadhead ratio = %0.1f" \
	% (deadhead_mission_type, deadhead_mission_range.to(ureg.nautical_mile).magnitude, \
		passenger_ratio, deadhead_time_in_hover.to(ureg.s).magnitude, deadhead_ratio)

plt.suptitle(title_str,fontsize = 14)

plt.tight_layout()#makes sure subplots are spaced neatly
plt.subplots_adjust(left=0.07,right=0.98,bottom=0.10,top=0.87)#adds space at the top for the title
