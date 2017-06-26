# Sweep to evaluate the sensitivity of each design to battery energy density

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/' + '..'))

import numpy as np
from gpkit import Model, ureg
from matplotlib import pyplot as plt
from aircraft_models import SimpleOnDemandAircraft 
from aircraft_models import OnDemandSizingMission, OnDemandTypicalMission
from aircraft_models import OnDemandMissionCost
from configuration_data import configurations

#General data
N = 6 #number of propellers. Required, but has no effect since T/A is constrained
eta_cruise = 0.85 #propulsive efficiency in cruise
eta_electric = 0.95 #electrical system efficiency
weight_fraction = 0.5 #structural mass fraction
N_crew = 1
n=1.0#battery discharge parameter
reserve_type = "Uber"

sizing_mission_range = 50*ureg.nautical_mile
typical_mission_range = 30*ureg.nautical_mile

sizing_time_in_hover=120*ureg.s
typical_time_in_hover=30*ureg.s

sizing_N_passengers = 3
typical_N_passengers = 2

cost_per_weight=112*ureg.lbf**-1
pilot_salary = 40*ureg.hr**-1
mechanic_salary=30*ureg.hr**-1

# Delete configurations that won't solve
configs = configurations.copy()
del configs["Tilt duct"]
del configs["Multirotor"]
del configs["Autogyro"]

#set up C_m array
C_m = np.linspace(300,600,10)*ureg.Wh/ureg.kg #battery energy density
C_m = ("sweep",C_m)


#Optimize remaining configurations
for config in configs:
	
	print "Solving configuration: " + config

	c = configs[config]

	V_cruise = c["V_{cruise}"]
	V_loiter = V_cruise #approximation
	L_D = c["L/D"]
	T_A = c["T/A"]
	Cl_mean_max = c["Cl_{mean_{max}}"]

	Aircraft = SimpleOnDemandAircraft(N=N,L_D=L_D,eta_cruise=eta_cruise,C_m=C_m,
		Cl_mean_max=Cl_mean_max,weight_fraction=weight_fraction,N_crew=N_crew,n=n,
		eta_electric=eta_electric)

	SizingMission = OnDemandSizingMission(Aircraft,mission_range=sizing_mission_range,
		V_cruise=V_cruise,V_loiter=V_loiter,N_passengers=sizing_N_passengers,
		time_in_hover=sizing_time_in_hover,reserve_type=reserve_type)
	SizingMission.substitutions.update({SizingMission.fs0.topvar("T/A"):T_A})

	TypicalMission = OnDemandTypicalMission(Aircraft,mission_range=typical_mission_range,
		V_cruise=V_cruise,N_passengers=typical_N_passengers,time_in_hover=typical_time_in_hover)

	MissionCost = OnDemandMissionCost(Aircraft,TypicalMission,cost_per_weight=cost_per_weight,
		pilot_salary=pilot_salary,mechanic_salary=mechanic_salary)
	
	problem = Model(MissionCost["cost_per_trip"],
		[Aircraft, SizingMission, TypicalMission, MissionCost])
	solution = problem.solve(verbosity=0)

	configs[config]["MTOW"] = solution["variables"]["MTOW_SimpleOnDemandAircraft"]
	configs[config]["W_{battery}"] = solution["variables"]["W_SimpleOnDemandAircraft/Battery"]
	configs[config]["cost_per_trip_per_passenger"] = solution["variables"]["cost_per_trip_per_passenger_OnDemandMissionCost"]
	configs[config]["SPL"] = np.array(20*np.log10(solution["variables"]["p_{ratio}_OnDemandSizingMission"]))
		

# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(17,11), dpi=80)
plt.show()

style = {}
style["linestyle"] = ["-","-","-","-","--","--","--","--"]
style["marker"] = ["s","o","^","v","s","o","^","v"]
style["fillstyle"] = ["full","full","full","full","none","none","none","none"]
style["markersize"] = 10

C_m = C_m[1] #convert back to array from tuple

#Maximum takeoff weight
plt.subplot(2,2,1)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(C_m.to(ureg.Wh/ureg.kg).magnitude,c["MTOW"].to(ureg.lbf).magnitude,
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.ylim(ymin=0)
plt.xlabel('Battery energy density (Wh/kg)', fontsize = 16)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Maximum Takeoff Weight",fontsize = 20)
plt.legend(numpoints = 1,loc='upper right', fontsize = 12)

#Battery weight
plt.subplot(2,2,2)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(C_m.to(ureg.Wh/ureg.kg).magnitude,c["W_{battery}"].to(ureg.lbf).magnitude,
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.ylim(ymin=0)
plt.xlabel('Battery energy density (Wh/kg)', fontsize = 16)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Battery Weight",fontsize = 20)
plt.legend(numpoints = 1,loc='upper right', fontsize = 12)

#Trip cost per passenger
plt.subplot(2,2,3)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(C_m.to(ureg.Wh/ureg.kg).magnitude,c["cost_per_trip_per_passenger"],
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.ylim(ymin=0)
plt.xlabel('Battery energy density (Wh/kg)', fontsize = 16)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Cost per Trip, per Passenger",fontsize = 20)
plt.legend(numpoints = 1,loc='upper right', fontsize = 12)

#Sound pressure level (in hover)
plt.subplot(2,2,4)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(C_m.to(ureg.Wh/ureg.kg).magnitude,c["SPL"],
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.xlabel('Battery energy density (Wh/kg)', fontsize = 16)
plt.ylabel('SPL (dB)', fontsize = 16)
plt.title("Sound Pressure Level in Hover",fontsize = 20)
plt.legend(numpoints = 1,loc='upper right', fontsize = 12)


if reserve_type == "FAA":
	num = solution["constants"]["t_{loiter}_OnDemandSizingMission"].to(ureg.minute).magnitude
	reserve_type_string = " (%0.0f-minute loiter time)" % num
if reserve_type == "Uber":
	num = solution["constants"]["R_{divert}_OnDemandSizingMission"].to(ureg.nautical_mile).magnitude
	reserve_type_string = " (%0.0f-nm diversion distance)" % num

title_str = "Aircraft parameters: structural mass fraction = %0.2f\n" % weight_fraction \
	+ "Sizing-mission parameters: range = %0.0f nm; %0.0f passengers; %0.0fs hover time; reserve type = " \
	% (sizing_mission_range.to(ureg.nautical_mile).magnitude, sizing_N_passengers, sizing_time_in_hover.to(ureg.s).magnitude) \
	+ reserve_type + reserve_type_string + "\n"\
	+ "Typical-mission parameters: range = %0.0f nm; %0.0f passengers; %0.0fs hover time; no reserve" \
	% (typical_mission_range.to(ureg.nautical_mile).magnitude, typical_N_passengers, typical_time_in_hover.to(ureg.s).magnitude)

plt.suptitle(title_str,fontsize = 16)

plt.tight_layout()#makes sure subplots are spaced neatly
plt.subplots_adjust(left=0.05,right=0.95,bottom=0.05,top=0.86)#adds space at the top for the title
