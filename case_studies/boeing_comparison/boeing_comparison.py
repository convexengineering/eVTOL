#Vehicle configuration top-level trade study

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

import matplotlib as mpl
mpl.style.use("classic")

#Data from the Boeing study
boeing_data = {}

boeing_data["C_m"] = (400/0.8)*ureg.Wh/ureg.kg

boeing_data["sizing_mission"] = {}
boeing_data["sizing_mission"]["type"] = "piloted"
boeing_data["sizing_mission"]["N_passengers"] = 3
boeing_data["sizing_mission"]["range"] = 87*ureg.nautical_mile
boeing_data["sizing_mission"]["t_{hover}"] = 2*ureg.minute

boeing_data["Lift + cruise"] = {}
boeing_data["Lift + cruise"]["V_{cruise}"] = 150*ureg("mph")
boeing_data["Lift + cruise"]["L/D"] = 9.1
boeing_data["Lift + cruise"]["T/A"] = 7.3*ureg("lbf")/ureg("ft")**2
boeing_data["Lift + cruise"]["loiter_type"] = "level_flight"
boeing_data["Lift + cruise"]["MTOW"] = 3710*ureg.lbf
boeing_data["Lift + cruise"]["W_{battery}"] = 948*ureg.lbf
boeing_data["Lift + cruise"]["P_{cruise}"] = 199*ureg.hp
boeing_data["Lift + cruise"]["P_{hover}"] = 389*ureg.hp

boeing_data["Tilt rotor"] = {}
boeing_data["Tilt rotor"]["V_{cruise}"] = 150*ureg("mph")
boeing_data["Tilt rotor"]["L/D"] = 11.0
boeing_data["Tilt rotor"]["T/A"] = 12.8*ureg("lbf")/ureg("ft")**2
boeing_data["Tilt rotor"]["loiter_type"] = "level_flight"
boeing_data["Tilt rotor"]["MTOW"] = 3930*ureg.lbf
boeing_data["Tilt rotor"]["W_{battery}"] = 965*ureg.lbf
boeing_data["Tilt rotor"]["P_{cruise}"] = 187*ureg.hp
boeing_data["Tilt rotor"]["P_{hover}"] = 542*ureg.hp

boeing_data["Helicopter"] = {}
boeing_data["Helicopter"]["V_{cruise}"] = 150*ureg("mph")
boeing_data["Helicopter"]["L/D"] = 7.84
boeing_data["Helicopter"]["T/A"] = 4.1*ureg("lbf")/ureg("ft")**2
boeing_data["Helicopter"]["loiter_type"] = "level_flight"
boeing_data["Helicopter"]["MTOW"] = 3470*ureg.lbf
boeing_data["Helicopter"]["W_{battery}"] = 1170*ureg.lbf
boeing_data["Helicopter"]["P_{cruise}"] = 250*ureg.hp
boeing_data["Helicopter"]["P_{hover}"] = 347*ureg.hp

sizing_mission_type = generic_data["sizing_mission"]["type"]
sizing_N_passengers = generic_data["sizing_mission"]["N_passengers"]
sizing_mission_range = generic_data["sizing_mission"]["range"]
sizing_t_hover = generic_data["sizing_mission"]["t_{hover}"]


#General data
eta_cruise = generic_data["\eta_{cruise}"] 
eta_electric = generic_data["\eta_{electric}"]
C_m = boeing_data["C_m"]
n = generic_data["n"]
B = generic_data["B"]

reserve_type = generic_data["reserve_type"]
autonomousEnabled = generic_data["autonomousEnabled"]
charger_power = generic_data["charger_power"]

vehicle_cost_per_weight = generic_data["vehicle_cost_per_weight"]
battery_cost_per_C = generic_data["battery_cost_per_C"]
pilot_wrap_rate = generic_data["pilot_wrap_rate"]
mechanic_wrap_rate = generic_data["mechanic_wrap_rate"]
MMH_FH = generic_data["MMH_FH"]
deadhead_ratio = generic_data["deadhead_ratio"]

delta_S = generic_data["delta_S"]

sizing_mission_type = boeing_data["sizing_mission"]["type"]
sizing_N_passengers = boeing_data["sizing_mission"]["N_passengers"]
sizing_mission_range = boeing_data["sizing_mission"]["range"]
sizing_t_hover = boeing_data["sizing_mission"]["t_{hover}"]

revenue_mission_type = generic_data["revenue_mission"]["type"]
revenue_N_passengers = generic_data["revenue_mission"]["N_passengers"]
revenue_mission_range = generic_data["revenue_mission"]["range"]
revenue_t_hover = generic_data["revenue_mission"]["t_{hover}"]

deadhead_mission_type = generic_data["deadhead_mission"]["type"]
deadhead_N_passengers = generic_data["deadhead_mission"]["N_passengers"]
deadhead_mission_range = generic_data["deadhead_mission"]["range"]
deadhead_t_hover = generic_data["deadhead_mission"]["t_{hover}"]

# Delete some configurations
configs = configuration_data.copy()
del configs["Tilt duct"]
del configs["Multirotor"]
del configs["Autogyro"]
#del configs["Helicopter"]
del configs["Coaxial heli"]
del configs["Tilt wing"]
del configs["Compound heli"]

#Optimize remaining configurations
for config in configs:
	
	print "Solving configuration: " + config

	c = configs[config]
	
	V_cruise = boeing_data[config]["V_{cruise}"]
	L_D_cruise = boeing_data[config]["L/D"]
	T_A = boeing_data[config]["T/A"]
	
	Cl_mean_max = c["Cl_{mean_{max}}"]
	N = c["N"]
	loiter_type = boeing_data[config]["loiter_type"]
	tailRotor_power_fraction_hover = c["tailRotor_power_fraction_hover"]
	tailRotor_power_fraction_levelFlight = c["tailRotor_power_fraction_levelFlight"]
	weight_fraction = c["weight_fraction"]

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
	configs[config]["solution"] = solution


for config in configs:
	MTOW_diff_percent = configs[config]["solution"]("MTOW_OnDemandAircraft")-boeing_data[config]["MTOW"]
	MTOW_diff_percent = (MTOW_diff_percent/boeing_data[config]["MTOW"]).to(ureg.dimensionless)
	print "%s GP weight increase: %0.1f%%" % (config, MTOW_diff_percent*100)


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

offset = 0.2
width = 0.3
colors = ["grey", "w", "k"]

#Maximum takeoff weight
plt.subplot(2,2,1)
for i,config in enumerate(configs):
	
	GP_data = configs[config]["solution"]
	
	MTOW_GP = GP_data("MTOW_OnDemandAircraft").to(ureg.lbf).magnitude
	MTOW_boeing = boeing_data[config]["MTOW"].to(ureg.lbf).magnitude
	
	if i == 0:
		plt.bar(i-offset,MTOW_GP,align='center',alpha=1,width=width,
			color=colors[0],label="GP data")
		plt.bar(i+offset,MTOW_boeing,align='center',alpha=1,width=width,
			color=colors[2],label="Boeing data")
	else:
		plt.bar(i-offset,MTOW_GP,align='center',alpha=1,width=width,
			color=colors[0])
		plt.bar(i+offset,MTOW_boeing,align='center',alpha=1,width=width,
			color=colors[2])
	

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.2*ymax)

plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Maximum Takeoff Weight",fontsize = 18)
plt.legend(loc='upper left', fontsize = 12)

#Battery weight
plt.subplot(2,2,2)
for i,config in enumerate(configs):
	
	GP_data = configs[config]["solution"]
	
	W_battery_GP = GP_data("W_OnDemandAircraft/Battery").to(ureg.lbf).magnitude
	W_battery_boeing = boeing_data[config]["W_{battery}"].to(ureg.lbf).magnitude
	
	if i == 0:
		plt.bar(i-offset,W_battery_GP,align='center',alpha=1,width=width,
			color=colors[0],label="GP data")
		plt.bar(i+offset,W_battery_boeing,align='center',alpha=1,width=width,
			color=colors[2],label="Boeing data")
	else:
		plt.bar(i-offset,W_battery_GP,align='center',alpha=1,width=width,
			color=colors[0])
		plt.bar(i+offset,W_battery_boeing,align='center',alpha=1,width=width,
			color=colors[2])
	

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.05*ymax)

plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Battery Weight",fontsize = 18)
plt.legend(loc='upper left', fontsize = 12)


#Power consumption (cruise)
plt.subplot(2,2,3)
for i,config in enumerate(configs):
	
	GP_data = configs[config]["solution"]
	
	P_cruise_GP = GP_data("P_{battery}_OnDemandSizingMission")[1].to(ureg.kW).magnitude
	P_cruise_boeing = boeing_data[config]["P_{cruise}"].to(ureg.kW).magnitude
	
	if i == 0:
		plt.bar(i-offset,P_cruise_GP,align='center',alpha=1,width=width,
			color=colors[0],label="GP data")
		plt.bar(i+offset,P_cruise_boeing,align='center',alpha=1,width=width,
			color=colors[2],label="Boeing data")
	else:
		plt.bar(i-offset,P_cruise_GP,align='center',alpha=1,width=width,
			color=colors[0])
		plt.bar(i+offset,P_cruise_boeing,align='center',alpha=1,width=width,
			color=colors[2])
	

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.1*ymax)

plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('Power (kW)', fontsize = 16)
plt.title("Cruise Power (sizing mission)",fontsize = 18)
plt.legend(loc='upper left', fontsize = 12)


#Power consumption (hover)
plt.subplot(2,2,4)
for i,config in enumerate(configs):
	
	GP_data = configs[config]["solution"]
	
	P_hover_GP = GP_data("P_{battery}_OnDemandSizingMission")[0].to(ureg.kW).magnitude
	P_hover_boeing = boeing_data[config]["P_{hover}"].to(ureg.kW).magnitude
	
	if i == 0:
		plt.bar(i-offset,P_hover_GP,align='center',alpha=1,width=width,
			color=colors[0],label="GP data")
		plt.bar(i+offset,P_hover_boeing,align='center',alpha=1,width=width,
			color=colors[2],label="Boeing data")
	else:
		plt.bar(i-offset,P_hover_GP,align='center',alpha=1,width=width,
			color=colors[0])
		plt.bar(i+offset,P_hover_boeing,align='center',alpha=1,width=width,
			color=colors[2])
	

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.1*ymax)

plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.ylabel('Power (kW)', fontsize = 16)
plt.title("Hover Power (sizing mission)",fontsize = 18)
plt.legend(loc='upper left', fontsize = 12)


if reserve_type == "FAA_aircraft" or reserve_type == "FAA_heli":
	num = solution("t_{loiter}_OnDemandSizingMission").to(ureg.minute).magnitude
	if reserve_type == "FAA_aircraft":
		reserve_type_string = "FAA aircraft VFR (%0.0f-minute loiter time)" % num
	elif reserve_type == "FAA_heli":
		reserve_type_string = "FAA helicopter VFR (%0.0f-minute loiter time)" % num
elif reserve_type == "Uber":
	num = solution["constants"]["R_{divert}_OnDemandSizingMission"].to(ureg.nautical_mile).magnitude
	reserve_type_string = " (%0.0f-nm diversion distance)" % num


if autonomousEnabled:
	autonomy_string = "autonomy enabled"
else:
	autonomy_string = "pilot required"


title_str = "Aircraft parameters: battery energy density = %0.0f Wh/kg; %0.0f rotor blades; %s\n" \
	% (C_m.to(ureg.Wh/ureg.kg).magnitude, B, autonomy_string) \
	+ "Sizing mission: range = %0.0f nm; %0.0f passengers; %0.0fs hover time; reserve type = " \
	% (sizing_mission_range.to(ureg.nautical_mile).magnitude, sizing_N_passengers,\
		sizing_t_hover.to(ureg.s).magnitude) + reserve_type_string + "\n" \
	+ "Revenue mission: range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (revenue_mission_range.to(ureg.nautical_mile).magnitude, \
		revenue_N_passengers, revenue_t_hover.to(ureg.s).magnitude, charger_power.to(ureg.kW).magnitude) \
	+ "Deadhead mission: range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve" \
	% (deadhead_mission_range.to(ureg.nautical_mile).magnitude, \
		deadhead_N_passengers, deadhead_t_hover.to(ureg.s).magnitude)

plt.suptitle(title_str,fontsize = 14)
plt.tight_layout()
plt.subplots_adjust(left=0.08,right=0.99,bottom=0.10,top=0.88)
plt.savefig('boeing_comparison_plot_01.pdf')
