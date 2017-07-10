#Case study on the configuration of the Joby S4

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/' + '..'))

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
sizing_N_passengers = generic_data["sizing_mission"]["N_passengers"]
sizing_mission_range = 100*ureg.nautical_mile
sizing_time_in_hover = generic_data["sizing_mission"]["time_in_hover"]

revenue_mission_type = generic_data["revenue_mission"]["type"]
revenue_N_passengers = generic_data["revenue_mission"]["N_passengers"]
revenue_mission_range = 50*ureg.nautical_mile
revenue_time_in_hover = generic_data["revenue_mission"]["time_in_hover"]

deadhead_mission_type = generic_data["deadhead_mission"]["type"]
deadhead_N_passengers = generic_data["deadhead_mission"]["N_passengers"]
deadhead_mission_range = 50*ureg.nautical_mile
deadhead_time_in_hover = generic_data["deadhead_mission"]["time_in_hover"]


# Delete some configurations

configs = {}

V_cruise = 200*ureg.mph

configs["12 rotors"] = configuration_data["Tilt rotor"].copy()
configs["12 rotors"]["V_{cruise}"] = V_cruise
configs["12 rotors"]["T/A"] = 16.3*ureg.lbf/ureg.ft**2
configs["12 rotors"]["N"] = 12

configs["6 rotors"] = configuration_data["Tilt rotor"].copy()
configs["6 rotors"]["V_{cruise}"] = V_cruise
configs["6 rotors"]["T/A"] = 8.15*ureg.lbf/ureg.ft**2
configs["6 rotors"]["N"] = 6


#Optimize 
for config in configs:
	
	print "Solving configuration: " + config

	c = configs[config]

	V_cruise = c["V_{cruise}"]
	L_D_cruise = c["L/D"]
	T_A = c["T/A"]
	Cl_mean_max = c["Cl_{mean_{max}}"]
	N = c["N"]

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
	configs[config]["solution"] = solution


# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(12,12), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

y_pos = np.arange(len(configs))
labels = [""]*len(configs)
for i, config in enumerate(configs):
	labels[i] = config

#Maximum takeoff weight
plt.subplot(3,2,1)
for i, config in enumerate(configs):
	MTOW = configs[config]["solution"]("MTOW_OnDemandAircraft").to(ureg.lbf).magnitude
	plt.bar(i,MTOW,align='center',alpha=1,width=0.8,color='k')
plt.grid()
plt.xticks(y_pos, labels, rotation=-45,fontsize=12)
plt.xlim(xmin = np.min(y_pos)-0.8,xmax = np.max(y_pos)+0.8)
plt.ylabel('Weight (lbf)', fontsize = 14)
plt.title("Maximum Takeoff Weight",fontsize = 16)

#Battery weight
plt.subplot(3,2,2)
for i, config in enumerate(configs):
	W_battery = configs[config]["solution"]("W_OnDemandAircraft/Battery").to(ureg.lbf).magnitude
	plt.bar(i,W_battery,align='center',alpha=1,width=0.8,color='k')
plt.grid()
plt.xticks(y_pos, labels, rotation=-45,fontsize=12)
plt.xlim(xmin = np.min(y_pos)-0.8,xmax = np.max(y_pos)+0.8)
plt.ylabel('Weight (lbf)', fontsize = 14)
plt.title("Battery Weight",fontsize = 16)

#Sound pressure level in hover (sizing mission) 
plt.subplot(3,2,3)
for i, config in enumerate(configs):
	SPL_sizing  = 20*np.log10(configs[config]["solution"]("p_{ratio}_OnDemandSizingMission"))
	plt.bar(i,SPL_sizing,align='center',alpha=1,width=0.8,color='k')

SPL_req = 62
plt.plot([np.min(y_pos)-1,np.max(y_pos)+1],[SPL_req, SPL_req],
	color="black", linewidth=3, linestyle="-")
#plt.ylim(ymin = 57, ymax = 76)
plt.ylim(ymin = 57)
plt.grid()
plt.xticks(y_pos, labels, rotation=-45,fontsize=12)
plt.xlim(xmin = np.min(y_pos)-0.8,xmax = np.max(y_pos)+0.8)
plt.ylabel('SPL (dB)', fontsize = 14)
plt.title("Sound Pressure Level in Hover",fontsize = 16)

#Trip cost per passenger 
plt.subplot(3,2,4)
for i, config in enumerate(configs):
	cptpp = configs[config]["solution"]("cost_per_trip_per_passenger_OnDemandMissionCost")
	plt.bar(i,cptpp,align='center',alpha=1,width=0.8,color='k')
plt.grid()
plt.xticks(y_pos, labels, rotation=-45,fontsize=12)
plt.xlim(xmin = np.min(y_pos)-0.8,xmax = np.max(y_pos)+0.8)
plt.ylabel('Cost ($US)', fontsize = 14)
plt.title("Cost per Trip, per Passenger",fontsize = 16)

#Cost per seat mile 
plt.subplot(3,2,5)
for i, config in enumerate(configs):
	cpsm = configs[config]["solution"]("cost_per_seat_mile_OnDemandMissionCost").to(ureg.mile**-1).magnitude
	plt.bar(i,cpsm,align='center',alpha=1,width=0.8,color='k')
plt.grid()
plt.xticks(y_pos, labels, rotation=-45,fontsize=12)
plt.xlim(xmin = np.min(y_pos)-0.8,xmax = np.max(y_pos)+0.8)
plt.ylabel('Cost ($US/mile)', fontsize = 14)
plt.title("Cost per Seat Mile",fontsize = 16)

#Cost per mission
plt.subplot(3,2,6)
for i, config in enumerate(configs):
	c_capital = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses")
	c_operating = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses")
	p1 = plt.bar(i,c_capital,bottom=0,align='center',alpha=1,width=0.8,color="k")
	p2 = plt.bar(i,c_operating,bottom=c_capital,align='center',alpha=1,width=0.8,color="w",hatch="\\")

plt.xticks(y_pos, labels, rotation=-45,fontsize=12)
plt.xlim(xmin = np.min(y_pos)-0.8,xmax = np.max(y_pos)+0.8)
plt.ylabel('Cost per mission ($US)', fontsize = 14)
plt.grid()
plt.title("Cost breakdown (revenue mission only)",fontsize = 16)
plt.legend((p1[0],p2[0]),("Capital expenses (amortized)","Operating expenses"),
	loc='lower left', fontsize = 12)


if (reserve_type == "FAA_day"):
	num = solution["constants"]["t_{loiter}_OnDemandSizingMission"].to(ureg.minute).magnitude
	reserve_type_string = "FAA day VFR (%0.0f-minute loiter time)" % num
elif (reserve_type == "FAA_night"):
	num = solution["constants"]["t_{loiter}_OnDemandSizingMission"].to(ureg.minute).magnitude
	reserve_type_string = "FAA night VFR (%0.0f-minute loiter time)" % num
elif reserve_type == "Uber":
	num = solution["constants"]["R_{divert}_OnDemandSizingMission"].to(ureg.nautical_mile).magnitude
	reserve_type_string = "Uber (%0.0f-nm diversion distance)" % num

if autonomousEnabled:
	autonomy_string = "autonomy enabled"
else:
	autonomy_string = "pilot required"

title_str = "Aircraft parameters: cruising speed = %0.0f knots; structural mass fraction = %0.2f; battery energy density = %0.0f Wh/kg; %s\n" \
	% (V_cruise.to(ureg.knot).magnitude, weight_fraction, C_m.to(ureg.Wh/ureg.kg).magnitude, autonomy_string) \
	+ "Sizing mission (%s): range = %0.0f nm; %0.0f passengers; %0.0fs hover time; reserve type = " \
	% (sizing_mission_type, sizing_mission_range.to(ureg.nautical_mile).magnitude, sizing_N_passengers, sizing_time_in_hover.to(ureg.s).magnitude) \
	+ reserve_type_string + "\n"\
	+ "Revenue mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (revenue_mission_type, revenue_mission_range.to(ureg.nautical_mile).magnitude, \
		revenue_N_passengers, revenue_time_in_hover.to(ureg.s).magnitude, charger_power.to(ureg.kW).magnitude) \
	+ "Deadhead mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; deadhead ratio = %0.1f" \
	% (deadhead_mission_type, deadhead_mission_range.to(ureg.nautical_mile).magnitude, \
		deadhead_N_passengers, deadhead_time_in_hover.to(ureg.s).magnitude, deadhead_ratio)

plt.suptitle(title_str,fontsize = 12)

plt.tight_layout()#makes sure subplots are spaced neatly
plt.subplots_adjust(left=0.07,right=0.99,bottom=0.07,top=0.88)#adds space at the top for the title
