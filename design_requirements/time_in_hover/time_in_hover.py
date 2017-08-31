# Sweep to evaluate the sensitivity of each design to time in hover (sizing mission)

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

#General data
eta_cruise = generic_data["\eta_{cruise}"] 
eta_electric = generic_data["\eta_{electric}"]
weight_fraction = generic_data["weight_fraction"]
C_m = generic_data["C_m"]
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

sizing_mission_type = generic_data["sizing_mission"]["type"]
sizing_N_passengers = generic_data["sizing_mission"]["N_passengers"]
sizing_mission_range = generic_data["sizing_mission"]["range"]
#sizing_t_hover = generic_data["sizing_mission"]["t_{hover}"]

revenue_mission_type = generic_data["revenue_mission"]["type"]
revenue_N_passengers = generic_data["revenue_mission"]["N_passengers"]
revenue_mission_range = generic_data["revenue_mission"]["range"]
revenue_t_hover = generic_data["revenue_mission"]["t_{hover}"]

deadhead_mission_type = generic_data["deadhead_mission"]["type"]
deadhead_N_passengers = generic_data["deadhead_mission"]["N_passengers"]
deadhead_mission_range = generic_data["deadhead_mission"]["range"]
deadhead_t_hover = generic_data["deadhead_mission"]["t_{hover}"]

# Delete certain configurations
configs = configuration_data.copy()
del configs["Tilt duct"]
del configs["Multirotor"]
del configs["Autogyro"]
del configs["Helicopter"]
del configs["Coaxial heli"]

#Data specific to study
sizing_t_hover_array = np.linspace(30,240,9)*ureg.s


#Optimize remaining configurations
for config in configs:

	print "Solving configuration: " + config

	c = configs[config]

	V_cruise = c["V_{cruise}"]
	L_D_cruise = c["L/D"]
	T_A = c["T/A"]
	Cl_mean_max = c["Cl_{mean_{max}}"]
	N = c["N"]
	loiter_type = c["loiter_type"]
	tailRotor_power_fraction_hover = c["tailRotor_power_fraction_hover"]
	tailRotor_power_fraction_levelFlight = c["tailRotor_power_fraction_levelFlight"]

	configs[config]["MTOW"] = np.zeros(np.size(sizing_t_hover_array))
	configs[config]["W_{battery}"] = np.zeros(np.size(sizing_t_hover_array))
	configs[config]["cost_per_trip_per_passenger"] = np.zeros(np.size(sizing_t_hover_array))
	configs[config]["SPL_A"] = np.zeros(np.size(sizing_t_hover_array))

	for i,sizing_t_hover in enumerate(sizing_t_hover_array):

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
			pilot_wrap_rate=pilot_wrap_rate,mechanic_wrap_rate=mechanic_wrap_rate,MMH_FH=MMH_FH,\
			deadhead_ratio=deadhead_ratio)

		problem = Model(MissionCost["cost_per_trip"],
			[Aircraft, SizingMission, RevenueMission, DeadheadMission, MissionCost])

		solution = problem.solve(verbosity=0)

		configs[config]["MTOW"][i] = solution("MTOW_OnDemandAircraft").to(ureg.lbf).magnitude
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

		#A-weighted
		f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
			weighting="A")
		configs[config]["SPL_A"][i] = SPL

	configs[config]["MTOW"] = configs[config]["MTOW"]*ureg.lbf
	configs[config]["W_{battery}"] = configs[config]["W_{battery}"]*ureg.lbf


# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(12,12), dpi=80)
plt.show()

style = {}
style["linestyle"] = ["-","-","-","-","--","--","--","--"]
style["marker"] = ["s","o","^","v","s","o","^","v"]
style["fillstyle"] = ["full","full","full","full","none","none","none","none"]
style["markersize"] = 10

#Maximum takeoff weight
plt.subplot(2,2,1)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(sizing_t_hover_array.to(ureg.s).magnitude,c["MTOW"].to(ureg.lbf).magnitude,
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.ylim(ymin=0)
plt.xlabel('Sizing time in hover (seconds)', fontsize = 16)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Maximum Takeoff Weight",fontsize = 20)
plt.legend(numpoints = 1,loc='upper left', fontsize = 12)

#Battery weight
plt.subplot(2,2,2)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(sizing_t_hover_array.to(ureg.s).magnitude,c["W_{battery}"].to(ureg.lbf).magnitude,
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.ylim(ymin=0)
plt.xlabel('Sizing time in hover (seconds)', fontsize = 16)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Battery Weight",fontsize = 20)
plt.legend(numpoints = 1,loc='upper left', fontsize = 12)

#Trip cost per passenger
plt.subplot(2,2,3)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(sizing_t_hover_array.to(ureg.s).magnitude,c["cost_per_trip_per_passenger"],
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.ylim(ymin=0)
plt.xlabel('Sizing time in hover (seconds)', fontsize = 16)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Cost per Trip, per Passenger",fontsize = 20)
plt.legend(numpoints = 1,loc='upper left', fontsize = 12)

#Sound pressure level (in hover)
plt.subplot(2,2,4)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(sizing_t_hover_array.to(ureg.s).magnitude,c["SPL_A"],
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.ylim(ymin=58,ymax = 85)
plt.xlabel('Sizing time in hover (seconds)', fontsize = 16)
plt.ylabel('SPL (dBA)', fontsize = 16)
plt.title("Sound Pressure Level in Hover",fontsize = 20)
plt.legend(numpoints = 1,loc='upper left', fontsize = 12)


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


title_str = "Aircraft parameters: structural mass fraction = %0.2f; battery energy density = %0.0f Wh/kg; %0.0f rotor blades; %s\n" \
	% (weight_fraction, C_m.to(ureg.Wh/ureg.kg).magnitude, B, autonomy_string) \
	+ "Sizing mission (%s): range = %0.0f nm; %0.0f passengers; reserve type = " \
	% (sizing_mission_type, sizing_mission_range.to(ureg.nautical_mile).magnitude, sizing_N_passengers) \
	+ reserve_type_string + "\n"\
	+ "Revenue mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (revenue_mission_type, revenue_mission_range.to(ureg.nautical_mile).magnitude, \
		revenue_N_passengers, revenue_t_hover.to(ureg.s).magnitude, charger_power.to(ureg.kW).magnitude) \
	+ "Deadhead mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; deadhead ratio = %0.1f" \
	% (deadhead_mission_type, deadhead_mission_range.to(ureg.nautical_mile).magnitude, \
		deadhead_N_passengers, deadhead_t_hover.to(ureg.s).magnitude, deadhead_ratio)

plt.suptitle(title_str,fontsize = 14)
plt.tight_layout()
plt.subplots_adjust(left=0.08,right=0.98,bottom=0.05,top=0.87)
plt.savefig('time_in_hover_plot_01.pdf')