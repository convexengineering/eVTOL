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
from noise_models import vortex_noise


# Data specific to study
configs = OrderedDict()
case_array = ["Case 1","Case 2","Case 3"]
sizing_mission_range_array = [19, 30, 30]*ureg.nautical_mile
revenue_mission_range_array = [19, 19, 30]*ureg.nautical_mile
deadhead_mission_range_array = [19, 19, 30]*ureg.nautical_mile

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
			SizingMission.mission_range: sizing_mission_range_array[i],#mission range
			SizingMission.V_cruise: c["V_{cruise}"],#cruising speed
			SizingMission.t_hover: generic_data["sizing_mission"]["t_{hover}"],#hover time
			SizingMission.T_A: c["T/A"],#disk loading
			SizingMission.passengers.N_passengers: generic_data["sizing_mission"]["N_{passengers}"],#Number of passengers
		})

		RevenueMission = OnDemandRevenueMission(Aircraft,mission_type=generic_data["revenue_mission"]["type"])
		problem_subDict.update({
			RevenueMission.mission_range: revenue_mission_range_array[i],#mission range
			RevenueMission.V_cruise: c["V_{cruise}"],#cruising speed
			RevenueMission.t_hover: generic_data["revenue_mission"]["t_{hover}"],#hover time
			RevenueMission.passengers.N_passengers: generic_data["revenue_mission"]["N_{passengers}"],#Number of passengers
			RevenueMission.time_on_ground.charger_power: generic_data["charger_power"], #Charger power
		})

		DeadheadMission = OnDemandDeadheadMission(Aircraft,mission_type=generic_data["deadhead_mission"]["type"])
		problem_subDict.update({
			DeadheadMission.mission_range: deadhead_mission_range_array[i],#mission range
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
		configs[config][case]["solution"] = solution

		configs[config][case]["TOGW"] = solution("TOGW_OnDemandAircraft")
		configs[config][case]["W_{battery}"] = solution("W_OnDemandAircraft/Battery")
		configs[config][case]["cost_per_trip_per_passenger"] = solution("cost_per_trip_per_passenger_OnDemandMissionCost")

		#Noise computations (sizing mission)
		T_perRotor = solution("T_perRotor_OnDemandSizingMission")[0]
		Q_perRotor = solution("Q_perRotor_OnDemandSizingMission")[0]
		R = solution("R")
		VT = solution("VT_OnDemandSizingMission")[0]
		s = solution("s")
		Cl_mean = solution("Cl_{mean_{max}}")
		N = solution("N")

		B = generic_data["B"]
		delta_S = generic_data["delta_S"]

		#A-weighted
		f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
			weighting="A")
		configs[config][case]["SPL_sizing_A"] = SPL
		configs[config][case]["f_{peak}"] = f_peak
		configs[config][case]["spectrum_sizing_A"] = spectrum

		#Noise computations (revenue mission)
		T_perRotor = solution("T_perRotor_OnDemandRevenueMission")[0]
		Q_perRotor = solution("Q_perRotor_OnDemandRevenueMission")[0]
		R = solution("R")
		VT = solution("VT_OnDemandRevenueMission")[0]
		s = solution("s")
		Cl_mean = solution("Cl_{mean_{max}}")
		N = solution("N")

		B = generic_data["B"]
		delta_S = generic_data["delta_S"]

		#A-weighted
		f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
			weighting="A")
		configs[config][case]["SPL_revenue_A"] = SPL
		configs[config][case]["f_{peak}"] = f_peak
		configs[config][case]["spectrum_revenue_A"] = spectrum


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
	legend_labels[i] = "%0.0f nmi sizing mission; %0.0f nmi revenue mission" \
		% (sizing_mission_range_array[i].to(ureg.nautical_mile).magnitude,
			revenue_mission_range_array[i].to(ureg.nautical_mile).magnitude)
	if revenue_mission_range_array[i] != deadhead_mission_range_array[i]:
		raise ValueError("Revenue and deadhead missions must be the same length.")


#Takeoff gross weight
plt.subplot(2,2,1)
for i,config in enumerate(configs):
	for j,case in enumerate(configs[config]):
		c = configs[config][case]
		offset = offset_array[j]
		TOGW = c["TOGW"].to(ureg.lbf).magnitude

		if (i == 0):
			label = legend_labels[j]
			plt.bar(i+offset,TOGW,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,TOGW,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

plt.grid()
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.3*ymax)
plt.title("Takeoff Gross Weight",fontsize = 18)
plt.legend(loc='upper left',framealpha=1, fontsize = 12)


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
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,cptpp,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.3*ymax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Cost per Trip, per Passenger",fontsize = 18)
plt.legend(loc='upper left',framealpha=1, fontsize = 12)


#Sound pressure level (in hover, sizing mission) 
plt.subplot(2,2,3)
for i,config in enumerate(configs):
	for j,case in enumerate(configs[config]):
		c = configs[config][case]
		offset = offset_array[j]
		SPL_sizing = c["SPL_sizing_A"]

		if (i == 0):
			label = legend_labels[j]
			plt.bar(i+offset,SPL_sizing,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,SPL_sizing,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

SPL_req = 62
plt.plot([np.min(y_pos)-1,np.max(y_pos)+1],[SPL_req, SPL_req],
	color="black", linewidth=3, linestyle="-")
plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymin = 57,ymax = ymax + 1)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('SPL (dBA)', fontsize = 16)
plt.title("Sound Pressure Level (sizing mission)",fontsize = 18)
plt.legend(loc='upper left',framealpha=1, fontsize = 12)

#Sound pressure level (in hover, revenue mission) 
plt.subplot(2,2,4)
for i,config in enumerate(configs):
	for j,case in enumerate(configs[config]):
		c = configs[config][case]
		offset = offset_array[j]
		SPL_revenue = c["SPL_revenue_A"]

		if (i == 0):
			label = legend_labels[j]
			plt.bar(i+offset,SPL_revenue,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,SPL_revenue,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

SPL_req = 62
plt.plot([np.min(y_pos)-1,np.max(y_pos)+1],[SPL_req, SPL_req],
	color="black", linewidth=3, linestyle="-")
plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymin = 57,ymax = ymax + 1)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('SPL (dBA)', fontsize = 16)
plt.title("Sound Pressure Level (revenue mission)",fontsize = 18)
plt.legend(loc='upper left',framealpha=1, fontsize = 12)

if generic_data["reserve_type"] == "FAA_aircraft" or generic_data["reserve_type"] == "FAA_heli":
	num = solution("t_{loiter}_OnDemandSizingMission").to(ureg.minute).magnitude
	if generic_data["reserve_type"] == "FAA_aircraft":
		reserve_type_string = "FAA aircraft VFR (%0.0f-minute loiter time)" % num
	elif generic_data["reserve_type"] == "FAA_heli":
		reserve_type_string = "FAA helicopter VFR (%0.0f-minute loiter time)" % num
elif generic_data["reserve_type"] == "Uber":
	num = solution["constants"]["R_{divert}_OnDemandSizingMission"].to(ureg.nautical_mile).magnitude
	reserve_type_string = " (%0.0f-nm diversion distance)" % num

if generic_data["autonomousEnabled"]:
	autonomy_string = "autonomy enabled"
else:
	autonomy_string = "pilot required"

title_str = "Aircraft parameters: battery energy density = %0.0f Wh/kg; %0.0f rotor blades; %s\n" \
	% (generic_data["C_m"].to(ureg.Wh/ureg.kg).magnitude, B, autonomy_string) \
	+ "Sizing mission (%s): %0.0f passengers; %0.0fs hover time; reserve type = " \
	% (generic_data["sizing_mission"]["type"],\
	 generic_data["sizing_mission"]["N_{passengers}"], generic_data["sizing_mission"]["t_{hover}"].to(ureg.s).magnitude)\
	+ reserve_type_string + "\n"\
	+ "Revenue mission (%s): %0.1f passengers; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (generic_data["revenue_mission"]["type"],\
	 generic_data["revenue_mission"]["N_{passengers}"], generic_data["revenue_mission"]["t_{hover}"].to(ureg.s).magnitude,\
	 generic_data["charger_power"].to(ureg.kW).magnitude) \
	+ "Deadhead mission (%s): %0.1f passengers; %0.0fs hover time; no reserve; deadhead ratio = %0.1f" \
	% (generic_data["deadhead_mission"]["type"], \
	 generic_data["deadhead_mission"]["N_{passengers}"], generic_data["deadhead_mission"]["t_{hover}"].to(ureg.s).magnitude,\
	 generic_data["deadhead_ratio"])

plt.suptitle(title_str,fontsize = 14.0)
plt.tight_layout()
plt.subplots_adjust(left=0.08,right=0.98,bottom=0.10,top=0.87)
plt.savefig('new_york_heli_plot_01.pdf')
