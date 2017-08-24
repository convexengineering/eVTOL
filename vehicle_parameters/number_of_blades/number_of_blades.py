#Sensitivity study to number of propeller blades. 
#Only affects periodic noise, since solidity is constant.

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
from noise_models import periodic_noise, vortex_noise, noise_weighting

#General data
eta_cruise = generic_data["\eta_{cruise}"] 
eta_electric = generic_data["\eta_{electric}"]
weight_fraction = generic_data["weight_fraction"]
C_m = generic_data["C_m"]
n = generic_data["n"]
#B = generic_data["B"]
x = 500*ureg.ft

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

# Delete some configurations
configs = configuration_data.copy()
del configs["Tilt duct"]
del configs["Multirotor"]
del configs["Autogyro"]

del configs["Helicopter"]
del configs["Coaxial heli"]

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
	

#Noise computations for varying theta (delta-S = constant)
B_array = [3,4,5,6]
theta_array = np.linspace(91,175,10)*ureg.degree

for config in configs:

	configs[config]["theta"] = {}
	
	configs[config]["theta"]["periodic"] = {}
	configs[config]["theta"]["periodic"]["f_fund"] = np.zeros([np.size(B_array),np.size(theta_array)])*ureg.turn/ureg.s
	configs[config]["theta"]["periodic"]["SPL"] = np.zeros([np.size(B_array),np.size(theta_array)])
	
	T_perRotor = configs[config]["solution"]("T_perRotor_OnDemandSizingMission")[0]
	Q_perRotor = configs[config]["solution"]("Q_perRotor_OnDemandSizingMission")[0]
	R = configs[config]["solution"]("R")
	VT = configs[config]["solution"]("VT_OnDemandSizingMission")[0]
	s = configs[config]["solution"]("s")
	Cl_mean = configs[config]["solution"]("Cl_{mean_{max}}")
	N = configs[config]["solution"]("N")

	#Periodic noise calculations
	for i,B in enumerate(B_array):
		for j,theta in enumerate(theta_array):

			f_peak, SPL, spectrum = periodic_noise(T_perRotor,Q_perRotor,R,VT,s,N,B,
				theta=theta,delta_S=x,h=0*ureg.ft,t_c=0.12,num_harmonics=5)

			configs[config]["theta"]["periodic"]["f_fund"][i,j] = f_peak
			configs[config]["theta"]["periodic"]["SPL"][i,j] = SPL

	#Vortex noise computations
	configs[config]["theta"]["vortex"] = {}

	f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,Cl_mean=Cl_mean,
		N=N,delta_S=x,h=0*ureg.ft,t_c=0.12,St=0.28)

	configs[config]["theta"]["vortex"]["f_peak"] = f_peak
	configs[config]["theta"]["vortex"]["SPL"] = SPL
	configs[config]["theta"]["vortex"]["spectrum"] = spectrum


# Plotting commands
plt.ion()
plt.rc('axes', axisbelow=True)

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

title_str = "Aircraft parameters: structural mass fraction = %0.2f; battery energy density = %0.0f Wh/kg; %s\n" \
	% (weight_fraction, C_m.to(ureg.Wh/ureg.kg).magnitude, autonomy_string) \
	+ "Sizing mission (%s): range = %0.0f nm; %0.0f passengers; %0.0fs hover time; reserve type = " \
	% (sizing_mission_type, sizing_mission_range.to(ureg.nautical_mile).magnitude, sizing_N_passengers, sizing_t_hover.to(ureg.s).magnitude) \
	+ reserve_type_string + "\n"\
	+ "Revenue mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (revenue_mission_type, revenue_mission_range.to(ureg.nautical_mile).magnitude, \
		revenue_N_passengers, revenue_t_hover.to(ureg.s).magnitude, charger_power.to(ureg.kW).magnitude) \
	+ "Deadhead mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; deadhead ratio = %0.1f" \
	% (deadhead_mission_type, deadhead_mission_range.to(ureg.nautical_mile).magnitude, \
		deadhead_N_passengers, deadhead_t_hover.to(ureg.s).magnitude, deadhead_ratio)


fig1 = plt.figure(figsize=(12,12), dpi=80)
plt.show()

style = {}
style["linestyle"] = ["-","-","-","-","--","--","--","--"]
style["marker"] = ["s","o","^","v","s","o","^","v"]
style["fillstyle"] = ["full","full","full","full","none","none","none","none"]
style["markersize"] = 10

for i, config in enumerate(configs):
	
	c = configs[config]
	
	SPL_vortex = c["theta"]["vortex"]["SPL"]*np.ones(np.size(theta_array))
	
	plt.subplot(2,2,i+1)

	for j,B in enumerate(B_array):
		SPL_periodic = c["theta"]["periodic"]["SPL"][j,:]
		periodic_label = "Periodic noise (%0.0f blades)" % B
		plt.plot(theta_array.to(ureg.degree).magnitude,SPL_periodic,color="black",
			linewidth=1.5,linestyle=style["linestyle"][j],marker=style["marker"][j],
			markersize=style["markersize"],fillstyle=style["fillstyle"][j],
			label=periodic_label)
	
	plt.plot(theta_array.to(ureg.degree).magnitude,SPL_vortex,'k--',linewidth=3,
		label="Vortex noise")
	plt.ylim(ymin=0)
	plt.grid()
	plt.xlabel('$\Theta$ (degrees)', fontsize = 16)
	plt.ylabel('SPL (dB)', fontsize = 16)
	plt.title(config, fontsize = 18)
	plt.legend(loc="lower left")

plt.suptitle(title_str,fontsize = 13.5)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.94,bottom=0.08,top=0.87)
plt.savefig('number_of_blades_plot_01.pdf')
