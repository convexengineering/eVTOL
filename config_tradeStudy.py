import numpy as np
from gpkit import Model, ureg
from matplotlib import pyplot as plt
from aircraft_models import SimpleOnDemandAircraft 
from aircraft_models import OnDemandSizingMission, OnDemandTypicalMission
from configuration_data import configurations

#General data
N = 12 #number of propellers. Required, but has no effect since T/A is constrained
eta_cruise = 0.85 #propulsive efficiency in cruise
eta_electric = 0.95 #electrical system efficiency
weight_fraction = 0.5 #structural mass fraction
C_m = 400*ureg.Wh/ureg.kg #battery energy density
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

#Optimize remaining configurations
for config in configs:
	
	print "Solving configuration: " + config

	c = configs[config]

	V_cruise = c["V_{cruise}"]
	V_loiter = V_cruise #approximation
	L_D = c["L/D"]
	T_A = c["T/A"]

	Aircraft = SimpleOnDemandAircraft(N=N,L_D=L_D,eta_cruise=eta_cruise,C_m=C_m,
		weight_fraction=weight_fraction,N_crew=N_crew,n=n,eta_electric=eta_electric)

	SizingMission = OnDemandSizingMission(Aircraft,mission_range=sizing_mission_range,
		V_cruise=V_cruise,V_loiter=V_loiter,N_passengers=sizing_N_passengers,
		time_in_hover=sizing_time_in_hover,reserve_type=reserve_type)
	SizingMission.substitutions.update({SizingMission.fs0.topvar("T/A"):T_A,
		SizingMission.fs2.topvar("T/A"):T_A,SizingMission.fs3.topvar("T/A"):T_A,
		SizingMission.fs5.topvar("T/A"):T_A})

	TypicalMission = OnDemandTypicalMission(Aircraft,mission_range=typical_mission_range,
		V_cruise=V_cruise,N_passengers=typical_N_passengers,time_in_hover=typical_time_in_hover,
		cost_per_weight=cost_per_weight,pilot_salary=pilot_salary,mechanic_salary=mechanic_salary)
	
	problem = Model(TypicalMission["cost_per_trip"],
		[Aircraft, SizingMission, TypicalMission])
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
	MTOW = configs[config]["solution"]["variables"]["MTOW_SimpleOnDemandAircraft"].to(ureg.lbf).magnitude
	plt.bar(i,MTOW,align='center',alpha=1)

plt.xticks(y_pos, labels, rotation=-60)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Maximum Takeoff Weight",fontsize = 20)

#Battery weight
plt.subplot(2,2,2)
for i, config in enumerate(configs):
	W_battery = configs[config]["solution"]["variables"]["W_SimpleOnDemandAircraft/Battery"].to(ureg.lbf).magnitude
	plt.bar(i,W_battery,align='center',alpha=1)

plt.xticks(y_pos, labels, rotation=-60)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Battery Weight",fontsize = 20)

#Trip cost per passenger 
plt.subplot(2,2,3)
for i, config in enumerate(configs):
	cptpp = configs[config]["solution"]["variables"]["cost_per_trip_per_passenger_OnDemandTypicalMission"]
	plt.bar(i,cptpp,align='center',alpha=1)

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

plt.xticks(y_pos, labels, rotation=-60)
plt.ylabel('SPL (dB)', fontsize = 16)
plt.title("Sound Pressure Level in Hover",fontsize = 20)


if reserve_type == "FAA":
	num = solution["constants"]["t_{loiter}_OnDemandSizingMission"].to(ureg.minute).magnitude
	reserve_type_string = " (%0.0f-minute loiter time)" % num
if reserve_type == "Uber":
	num = solution["constants"]["R_{divert}_OnDemandSizingMission"].to(ureg.nautical_mile).magnitude
	reserve_type_string = " (%0.0f-nm diversion distance)" % num

title_str = "Aircraft parameters: structural mass fraction = %0.2f; battery energy density = %0.0f Wh/kg\n" \
	% (weight_fraction, C_m.to(ureg.Wh/ureg.kg).magnitude) \
	+ "Sizing-mission parameters: range = %0.0f nm; %0.0f passengers; %0.0fs hover time; reserve type = " \
	% (sizing_mission_range.to(ureg.nautical_mile).magnitude, sizing_N_passengers, sizing_time_in_hover.to(ureg.s).magnitude) \
	+ reserve_type + reserve_type_string + "\n"\
	+ "Typical-mission parameters: range = %0.0f nm; %0.0f passengers; %0.0fs hover time; no reserve" \
	% (typical_mission_range.to(ureg.nautical_mile).magnitude, typical_N_passengers, typical_time_in_hover.to(ureg.s).magnitude)

plt.suptitle(title_str,fontsize = 16)

plt.tight_layout()#makes sure subplots are spaced neatly
plt.subplots_adjust(left=0.05,right=0.95,bottom=0.125,top=0.86)#adds space at the top for the title