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
from noise_models import vortex_noise


# Delete some configurations
configs = configuration_data.copy()
del configs["Tilt duct"]
del configs["Multirotor"]
del configs["Autogyro"]
del configs["Helicopter"]
del configs["Coaxial heli"]

#Data specific to study
sizing_N_passengers_array = np.linspace(1,5,3)
load_factor = 2./3
revenue_N_passengers_array = load_factor*sizing_N_passengers_array


#Optimize remaining configurations
for config in configs:
	
	print "Solving configuration: " + config

	c = configs[config]

	configs[config]["TOGW"] = np.zeros(np.size(sizing_N_passengers_array))
	configs[config]["W_{battery}"] = np.zeros(np.size(sizing_N_passengers_array))
	configs[config]["cost_per_trip_per_passenger"] = np.zeros(np.size(sizing_N_passengers_array))
	configs[config]["SPL_A"] = np.zeros(np.size(sizing_N_passengers_array))

	for i,sizing_N_passengers in enumerate(sizing_N_passengers_array):
		revenue_N_passengers = revenue_N_passengers_array[i]

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
			SizingMission.passengers.N_passengers: sizing_N_passengers,#Number of passengers
		})

		RevenueMission = OnDemandRevenueMission(Aircraft,mission_type=generic_data["revenue_mission"]["type"])
		problem_subDict.update({
			RevenueMission.mission_range: generic_data["revenue_mission"]["range"],#mission range
			RevenueMission.V_cruise: c["V_{cruise}"],#cruising speed
			RevenueMission.t_hover: generic_data["revenue_mission"]["t_{hover}"],#hover time
			RevenueMission.passengers.N_passengers: revenue_N_passengers,#Number of passengers
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

		configs[config]["TOGW"][i] = solution("TOGW_OnDemandAircraft").to(ureg.lbf).magnitude
		configs[config]["W_{battery}"][i] = solution("W_OnDemandAircraft/Battery").to(ureg.lbf).magnitude
		configs[config]["cost_per_trip_per_passenger"][i] = solution("cost_per_trip_per_passenger_OnDemandMissionCost")

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

		#A-weighted
		f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
			weighting="A")
		configs[config]["SPL_A"][i] = SPL

	configs[config]["TOGW"] = configs[config]["TOGW"]*ureg.lbf
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

xmin = np.min(y_pos) - 0.7
xmax = np.max(y_pos) + 0.7

offset_array = [-0.3,0,0.3]
width = 0.2
colors = ["grey", "w", "k"]

#Maximum takeoff weight
plt.subplot(2,2,1)
for i, config in enumerate(configs):
	c = configs[config]
	for j,offset in enumerate(offset_array):
		TOGW = c["TOGW"][j].to(ureg.lbf).magnitude

		if (i == 0):
			if (sizing_N_passengers_array[j] == 1):
				label = "%0.0f passenger" % sizing_N_passengers_array[j]
			else:
				label = "%0.0f passengers" % sizing_N_passengers_array[j]
			plt.bar(i+offset,TOGW,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,TOGW,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')


plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.4*ymax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Maximum Takeoff Weight",fontsize = 18)
plt.legend(loc='upper right', fontsize = 12,framealpha=1)


#Battery weight
plt.subplot(2,2,2)
for i, config in enumerate(configs):
	c = configs[config]
	for j,offset in enumerate(offset_array):
		W_battery = c["W_{battery}"][j].to(ureg.lbf).magnitude
		if (i == 0):
			if (sizing_N_passengers_array[j] == 1):
				label = "%0.0f passenger" % sizing_N_passengers_array[j]
			else:
				label = "%0.0f passengers" % sizing_N_passengers_array[j]
			plt.bar(i+offset,W_battery,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,W_battery,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Battery Weight",fontsize = 18)
plt.legend(loc='upper right', fontsize = 12,framealpha=1)


#Trip cost per passenger 
plt.subplot(2,2,3)
for i, config in enumerate(configs):
	c = configs[config]
	for j,offset in enumerate(offset_array):
		cptpp = c["cost_per_trip_per_passenger"][j]
		if (i == 0):
			if (sizing_N_passengers_array[j] == 1):
				label = "%0.0f passenger" % sizing_N_passengers_array[j]
			else:
				label = "%0.0f passengers" % sizing_N_passengers_array[j]
			plt.bar(i+offset,cptpp,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,cptpp,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.2*ymax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Cost per Trip, per Passenger",fontsize = 18)
plt.legend(loc='upper right', fontsize = 12,framealpha=1)


#Sound pressure level (in hover) 
plt.subplot(2,2,4)
for i, config in enumerate(configs):
	c = configs[config]
	for j,offset in enumerate(offset_array):
		SPL_sizing = c["SPL_A"][j]
		if (i == 0):
			if (sizing_N_passengers_array[j] == 1):
				label = "%0.0f passenger" % sizing_N_passengers_array[j]
			else:
				label = "%0.0f passengers" % sizing_N_passengers_array[j]
			plt.bar(i+offset,SPL_sizing,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,SPL_sizing,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

SPL_req = 62
plt.plot([np.min(y_pos)-1,np.max(y_pos)+1],[SPL_req, SPL_req],
	color="black", linewidth=3, linestyle="-")
plt.ylim(ymin = 57,ymax = 83)
plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('SPL (dBA)', fontsize = 16)
plt.title("Sound Pressure Level in Hover",fontsize = 18)
plt.legend(loc='upper right', fontsize = 12, framealpha=1)

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
	+ "Sizing mission (%s): range = %0.0f nmi; %0.0fs hover time; reserve type = " \
	% (generic_data["sizing_mission"]["type"], generic_data["sizing_mission"]["range"].to(ureg.nautical_mile).magnitude,\
	 generic_data["sizing_mission"]["t_{hover}"].to(ureg.s).magnitude)\
	+ reserve_type_string + "\n"\
	+ "Revenue mission (%s): range = %0.0f nmi; load factor = %0.2f; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (generic_data["revenue_mission"]["type"], generic_data["revenue_mission"]["range"].to(ureg.nautical_mile).magnitude, \
	 load_factor, generic_data["revenue_mission"]["t_{hover}"].to(ureg.s).magnitude,\
	 generic_data["charger_power"].to(ureg.kW).magnitude) \
	+ "Deadhead mission (%s): range = %0.0f nmi; %0.1f passengers; %0.0fs hover time; no reserve; deadhead ratio = %0.1f" \
	% (generic_data["deadhead_mission"]["type"], generic_data["deadhead_mission"]["range"].to(ureg.nautical_mile).magnitude, \
	 generic_data["deadhead_mission"]["N_{passengers}"], generic_data["deadhead_mission"]["t_{hover}"].to(ureg.s).magnitude,\
	 generic_data["deadhead_ratio"])

plt.suptitle(title_str,fontsize = 13.5)
plt.tight_layout()
plt.subplots_adjust(left=0.08,right=0.98,bottom=0.10,top=0.87)
plt.savefig('number_of_passengers_plot_01.pdf')
