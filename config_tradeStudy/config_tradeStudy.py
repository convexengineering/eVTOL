#Vehicle configuration top-level trade study

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/' + '..'))

import numpy as np
from gpkit import Model, ureg
from matplotlib import pyplot as plt
from aircraft_models import OnDemandAircraft 
from aircraft_models import OnDemandSizingMission, OnDemandRevenueMission
from aircraft_models import OnDemandDeadheadMission, OnDemandMissionCost
from configuration_data import configurations

#General data
N = 12 #number of propellers. Required, but has no effect since T/A is constrained
eta_cruise = 0.85 #propulsive efficiency in cruise
eta_electric = 0.9 #electrical system efficiency
weight_fraction = 0.5 #structural mass fraction
C_m = 400*ureg.Wh/ureg.kg #battery energy density
n=1.0#battery discharge parameter
reserve_type = "Uber"

sizing_mission_range = 50*ureg.nautical_mile
revenue_mission_range = 30*ureg.nautical_mile
deadhead_mission_range = 30*ureg.nautical_mile

sizing_time_in_hover = 120*ureg.s
revenue_time_in_hover = 30*ureg.s
deadhead_time_in_hover = 30*ureg.s

autonomousEnabled = "Yes"
sizing_mission_type = "piloted"
revenue_mission_type = "piloted"
deadhead_mission_type = "piloted"

sizing_N_passengers = 3
revenue_N_passengers = 2
deadhead_N_passengers = 0.00001

charger_power=200*ureg.kW

vehicle_cost_per_weight=350*ureg.lbf**-1
battery_cost_per_C = 400*ureg.kWh**-1
pilot_wrap_rate = 70*ureg.hr**-1
mechanic_wrap_rate = 60*ureg.hr**-1
MMH_FH = 0.6
deadhead_ratio = 0.2


# Delete configurations that won't solve
configs = configurations.copy()
del configs["Tilt duct"]
del configs["Multirotor"]

#Optimize remaining configurations
for config in configs:
	
	print "Solving configuration: " + config

	c = configs[config]

	V_cruise = c["V_{cruise}"]
	V_loiter = V_cruise #approximation
	L_D = c["L/D"]
	T_A = c["T/A"]
	Cl_mean_max = c["Cl_{mean_{max}}"]

	Aircraft = OnDemandAircraft(N=N,L_D=L_D,eta_cruise=eta_cruise,C_m=C_m,
		Cl_mean_max=Cl_mean_max,weight_fraction=weight_fraction,n=n,eta_electric=eta_electric,
		cost_per_weight=vehicle_cost_per_weight,cost_per_C=battery_cost_per_C,
		autonomousEnabled=autonomousEnabled)

	SizingMission = OnDemandSizingMission(Aircraft,mission_range=sizing_mission_range,
		V_cruise=V_cruise,V_loiter=V_loiter,N_passengers=sizing_N_passengers,
		time_in_hover=sizing_time_in_hover,reserve_type=reserve_type,
		mission_type=sizing_mission_type)
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
fig1 = plt.figure(figsize=(17,11), dpi=80)
plt.show()

y_pos = np.arange(len(configs))
labels = [""]*len(configs)
for i, config in enumerate(configs):
	labels[i] = config


#Maximum takeoff weight
plt.subplot(2,2,1)
for i, config in enumerate(configs):
	MTOW = configs[config]["solution"]["variables"]["MTOW_OnDemandAircraft"].to(ureg.lbf).magnitude
	plt.bar(i,MTOW,align='center',alpha=1)
plt.grid()
plt.xticks(y_pos, labels, rotation=-60)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Maximum Takeoff Weight",fontsize = 20)

#Battery weight
plt.subplot(2,2,2)
for i, config in enumerate(configs):
	W_battery = configs[config]["solution"]["variables"]["W_OnDemandAircraft/Battery"].to(ureg.lbf).magnitude
	plt.bar(i,W_battery,align='center',alpha=1)
plt.grid()
plt.xticks(y_pos, labels, rotation=-60)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Battery Weight",fontsize = 20)

#Trip cost per passenger 
plt.subplot(2,2,3)
for i, config in enumerate(configs):
	cptpp = configs[config]["solution"]["variables"]["cost_per_trip_per_passenger_OnDemandMissionCost"]
	plt.bar(i,cptpp,align='center',alpha=1)
plt.grid()
plt.xticks(y_pos, labels, rotation=-60)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Cost per Trip, per Passenger",fontsize = 20)

#Sound pressure level (in hover) 
plt.subplot(2,2,4)
for i, config in enumerate(configs):
	SPL_sizing  = np.array(20*np.log10(configs[config]["solution"]["variables"]["p_{ratio}_OnDemandSizingMission"]))
	plt.bar(i,SPL_sizing,align='center',alpha=1)

SPL_req = 62
plt.plot([np.min(y_pos)-1,np.max(y_pos)+1],[SPL_req, SPL_req],
	color="black", linewidth=3, linestyle="-")
plt.ylim(ymin = 57, ymax = 75)
plt.grid()
plt.xticks(y_pos, labels, rotation=-60)
plt.ylabel('SPL (dB)', fontsize = 16)
plt.title("Sound Pressure Level in Hover",fontsize = 20)


if reserve_type == "FAA":
	num = solution["constants"]["t_{loiter}_OnDemandSizingMission"].to(ureg.minute).magnitude
	reserve_type_string = " (%0.0f-minute loiter time)" % num
if reserve_type == "Uber":
	num = solution["constants"]["R_{divert}_OnDemandSizingMission"].to(ureg.nautical_mile).magnitude
	reserve_type_string = " (%0.0f-nm diversion distance)" % num

if autonomousEnabled == "Yes":
	autonomy_string = "autonomy enabled"
if autonomousEnabled == "No":
	autonomy_string = "pilot required"

title_str = "Aircraft parameters: structural mass fraction = %0.2f; battery energy density = %0.0f Wh/kg, %s\n" \
	% (weight_fraction, C_m.to(ureg.Wh/ureg.kg).magnitude, autonomy_string) \
	+ "Sizing mission (%s): range = %0.0f nm; %0.0f passengers; %0.0fs hover time; reserve type = " \
	% (sizing_mission_type, sizing_mission_range.to(ureg.nautical_mile).magnitude, sizing_N_passengers, sizing_time_in_hover.to(ureg.s).magnitude) \
	+ reserve_type + reserve_type_string + "\n"\
	+ "Revenue mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve\n" \
	% (revenue_mission_type, revenue_mission_range.to(ureg.nautical_mile).magnitude, revenue_N_passengers, revenue_time_in_hover.to(ureg.s).magnitude) \
	+ "Deadhead mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve" \
	% (deadhead_mission_type, deadhead_mission_range.to(ureg.nautical_mile).magnitude, deadhead_N_passengers, deadhead_time_in_hover.to(ureg.s).magnitude)

plt.suptitle(title_str,fontsize = 16)

plt.tight_layout()#makes sure subplots are spaced neatly
plt.subplots_adjust(left=0.05,right=0.95,bottom=0.125,top=0.84)#adds space at the top for the title

'''
#Cost breakdown plot

fig2 = plt.figure(figsize=(17,11), dpi=80)
plt.show()

for i, config in enumerate(configs):
	c_vehicle = configs[config]["solution"]["variables"]["c_{vehicle}_OnDemandMissionCost"]/typical_N_passengers
	c_energy = configs[config]["solution"]["variables"]["c_{energy}_OnDemandMissionCost"]/typical_N_passengers
	c_pilot = configs[config]["solution"]["variables"]["c_{pilot}_OnDemandMissionCost"]/typical_N_passengers
	c_maintenance = configs[config]["solution"]["variables"]["c_{maintenance}_OnDemandMissionCost"]/typical_N_passengers
	
	p1 = plt.bar(i,c_vehicle,bottom=0,align='center',alpha=1,color="b",hatch="/")
	p2 = plt.bar(i,c_energy,bottom=c_vehicle,align='center',alpha=1,color="r",hatch="\\")
	p3 = plt.bar(i,c_pilot,bottom=c_vehicle+c_energy, align='center',alpha=1,color="k",hatch="-")
	p4 = plt.bar(i,c_maintenance,bottom=c_vehicle+c_energy+c_pilot,align='center',alpha=1,color="g",hatch="|")

plt.xticks(y_pos, labels, rotation=-60,fontsize=14)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.legend((p1[0],p2[0],p3[0],p4[0]), \
	("Vehicle amortized cost","Energy cost","Pilot cost","Maintenance cost"),\
	loc='upper left', fontsize = 16)

plt.title(title_str,fontsize = 16)
plt.tight_layout()
#plt.subplots_adjust(left=0.05,right=0.95,bottom=0.125,top=0.86)#adds space at the top for the title
'''