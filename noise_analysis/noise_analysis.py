#Noise analysis as part of vehicle top-level trade study

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
from noise_models import periodic_noise, vortex_noise, noise_weighting

#General data
eta_cruise = generic_data["\eta_{cruise}"] 
eta_electric = generic_data["\eta_{electric}"]
weight_fraction = generic_data["weight_fraction"]
C_m = generic_data["C_m"]
n = generic_data["n"]
B = generic_data["B"]
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

	Aircraft = OnDemandAircraft(N=N,L_D_cruise=L_D_cruise,eta_cruise=eta_cruise,C_m=C_m,
		Cl_mean_max=Cl_mean_max,weight_fraction=weight_fraction,n=n,eta_electric=eta_electric,
		cost_per_weight=vehicle_cost_per_weight,cost_per_C=battery_cost_per_C,
		autonomousEnabled=autonomousEnabled)

	SizingMission = OnDemandSizingMission(Aircraft,mission_range=sizing_mission_range,
		V_cruise=V_cruise,N_passengers=sizing_N_passengers,t_hover=sizing_t_hover,
		reserve_type=reserve_type,mission_type=sizing_mission_type,loiter_type=loiter_type)
	SizingMission.substitutions.update({SizingMission.fs0.topvar("T/A"):T_A})
	
	RevenueMission = OnDemandRevenueMission(Aircraft,mission_range=revenue_mission_range,
		V_cruise=V_cruise,N_passengers=revenue_N_passengers,t_hover=revenue_t_hover,
		charger_power=charger_power,mission_type=revenue_mission_type)

	DeadheadMission = OnDemandDeadheadMission(Aircraft,mission_range=deadhead_mission_range,
		V_cruise=V_cruise,N_passengers=deadhead_N_passengers,t_hover=deadhead_t_hover,
		charger_power=charger_power,mission_type=deadhead_mission_type)

	MissionCost = OnDemandMissionCost(Aircraft,RevenueMission,DeadheadMission,
		pilot_wrap_rate=pilot_wrap_rate,mechanic_wrap_rate=mechanic_wrap_rate,MMH_FH=MMH_FH,
		deadhead_ratio=deadhead_ratio)
	
	problem = Model(MissionCost["cost_per_trip"],
		[Aircraft, SizingMission, RevenueMission, DeadheadMission, MissionCost])
	
	solution = problem.solve(verbosity=0)
	configs[config]["solution"] = solution
	

#Noise computations for varying theta (delta-S = constant)
theta_array = np.linspace(91,175,50)*ureg.degree

for config in configs:

	configs[config]["theta"] = {}
	
	configs[config]["theta"]["periodic"] = {}
	configs[config]["theta"]["periodic"]["f_fund"] = np.zeros(np.size(theta_array))*ureg.turn/ureg.s
	configs[config]["theta"]["periodic"]["SPL"] = np.zeros(np.size(theta_array))
	configs[config]["theta"]["periodic"]["spectrum"] = [{} for i in range(np.size(theta_array))]


	T_perRotor = configs[config]["solution"]("T_perRotor_OnDemandSizingMission")[0]
	Q_perRotor = configs[config]["solution"]("Q_perRotor_OnDemandSizingMission")[0]
	R = configs[config]["solution"]("R")
	VT = configs[config]["solution"]("VT_OnDemandSizingMission")[0]
	s = configs[config]["solution"]("s")
	Cl_mean = configs[config]["solution"]("Cl_{mean_{max}}")
	N = configs[config]["solution"]("N")

	#Periodic noise calculations
	for i,theta in enumerate(theta_array):
		
		f_peak, SPL, spectrum = periodic_noise(T_perRotor,Q_perRotor,R,VT,s,N,B,theta=theta,
			delta_S=x,h=0*ureg.ft,t_c=0.12,num_harmonics=20)

		configs[config]["theta"]["periodic"]["f_fund"][i] = f_peak
		configs[config]["theta"]["periodic"]["SPL"][i] = SPL
		configs[config]["theta"]["periodic"]["spectrum"][i] = spectrum

	#Vortex noise computations
	configs[config]["theta"]["vortex"] = {}

	f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,Cl_mean=Cl_mean,
		N=N,delta_S=x,h=0*ureg.ft,t_c=0.12,St=0.28)

	configs[config]["theta"]["vortex"]["f_peak"] = f_peak
	configs[config]["theta"]["vortex"]["SPL"] = SPL
	configs[config]["theta"]["vortex"]["spectrum"] = spectrum


#Computations for varying y (delta-S is not constant)
y_array = np.linspace(50,3000,50)*ureg.ft

for config in configs:

	configs[config]["y"] = {}
	
	configs[config]["y"]["periodic"] = {}
	configs[config]["y"]["periodic"]["f_fund"] = np.zeros(np.size(y_array))*ureg.turn/ureg.s
	configs[config]["y"]["periodic"]["SPL"] = np.zeros(np.size(y_array))
	configs[config]["y"]["periodic"]["spectrum"] = [{} for i in range(np.size(y_array))]

	configs[config]["y"]["vortex"] = {}
	configs[config]["y"]["vortex"]["f_peak"] = np.zeros(np.size(y_array))*ureg.turn/ureg.s
	configs[config]["y"]["vortex"]["SPL"] = np.zeros(np.size(y_array))
	configs[config]["y"]["vortex"]["spectrum"] = [{} for i in range(np.size(y_array))]

	T_perRotor = configs[config]["solution"]("T_perRotor_OnDemandSizingMission")[0]
	Q_perRotor = configs[config]["solution"]("Q_perRotor_OnDemandSizingMission")[0]
	R = configs[config]["solution"]("R")
	VT = configs[config]["solution"]("VT_OnDemandSizingMission")[0]
	s = configs[config]["solution"]("s")
	Cl_mean = configs[config]["solution"]("Cl_{mean_{max}}")
	N = configs[config]["solution"]("N")

	#Noise calculations
	for i,y in enumerate(y_array):

		theta = 180*ureg.degree - (np.arctan(y/x)*ureg.radian).to(ureg.degree)
		delta_S = np.sqrt(x**2 + y**2)

		#Periodic noise
		f_peak, SPL, spectrum = periodic_noise(T_perRotor,Q_perRotor,R,VT,s,N,B,theta=theta,
			delta_S=delta_S,h=0*ureg.ft,t_c=0.12,num_harmonics=20)
		configs[config]["y"]["periodic"]["f_fund"][i] = f_peak
		configs[config]["y"]["periodic"]["SPL"][i] = SPL
		configs[config]["y"]["periodic"]["spectrum"][i] = spectrum

		#Vortex noise
		f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28)
		configs[config]["y"]["vortex"]["f_peak"][i] = f_peak
		configs[config]["y"]["vortex"]["SPL"][i] = SPL
		configs[config]["y"]["vortex"]["spectrum"][i] = spectrum

# Plotting commands
plt.ion()

#Plot showing noise spectra (both periodic & vortex) for a sample value of y
fig1 = plt.figure(figsize=(12,12), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

#Find value of y closest to that desired
y_desired = 150*ureg.feet
idx = (np.abs(y_array - y_desired)).argmin()
y_selected = y_array[idx]

for i, config in enumerate(configs):
	
	c = configs[config]	
	
	#Periodic noise (1st harmonic only)
	periodic_f_spectrum = c["y"]["periodic"]["spectrum"][idx]["f"][0:2]
	periodic_SPL_spectrum = c["y"]["periodic"]["spectrum"][idx]["SPL"][0:2]

	#Vortex noise
	vortex_f_spectrum = c["y"]["vortex"]["spectrum"][idx]["f"]
	vortex_SPL_spectrum = c["y"]["vortex"]["spectrum"][idx]["SPL"]

	#A-weighting spectrum
	f_min_rev_per_s = np.min(periodic_f_spectrum.to(ureg.turn/ureg.s).magnitude)
	f_max_rev_per_s = np.max(vortex_f_spectrum.to(ureg.turn/ureg.s).magnitude)
	f_dBA_offset = np.linspace(f_min_rev_per_s,f_max_rev_per_s,100)*ureg.turn/ureg.s
	dBA_offset = noise_weighting(f_dBA_offset,np.zeros(np.shape(f_dBA_offset)))
	
	ax = fig1.add_subplot(2,2,i+1)

	periodic_label = "Periodic noise (y = %0.0f ft)" % y_selected.to(ureg.foot).magnitude
	vortex_label = "Vortex noise (y = %0.0f ft)" % y_selected.to(ureg.foot).magnitude
	
	for j,SPL in enumerate(periodic_SPL_spectrum):
		f = periodic_f_spectrum[j].to(ureg.turn/ureg.s).magnitude
		if j == 0:
			ax.bar(f,SPL,width=0.2*f,align="center",color='k',label=periodic_label)
		else:
			ax.bar(f,SPL,width=0.2*f,align="center",color='k')
	
	ax.plot(vortex_f_spectrum.to(ureg.turn/ureg.s).magnitude,vortex_SPL_spectrum,
		'k-',linewidth=2,label=vortex_label)
	plt.xlabel('Frequency (Hz)', fontsize = 16)
	plt.ylabel('SPL (dB)', fontsize = 16)
	
	#ymax = np.max(SPL_spectrum) + 5
	#plt.ylim(ymax=ymax)

	ax2 = ax.twinx()
	ax2.plot(f_dBA_offset.to(ureg.turn/ureg.s).magnitude,dBA_offset,'k--',linewidth=2,
		label="A-weighting offset")
	plt.ylabel('SPL offset (dBA)', fontsize = 16)
	
	plt.xscale('log')
	plt.grid()
	plt.title(config, fontsize = 18)

	lines, labels = ax.get_legend_handles_labels()
	lines2, labels2 = ax2.get_legend_handles_labels()
	ax2.legend(lines + lines2, labels + labels2, fontsize=13,loc="lower right")


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

title_str = "Aircraft parameters: %0.0f blades; structural mass fraction = %0.2f; battery energy density = %0.0f Wh/kg; %s\n" \
	% (B, weight_fraction, C_m.to(ureg.Wh/ureg.kg).magnitude, autonomy_string) \
	+ "Sizing mission (%s): range = %0.0f nm; %0.0f passengers; %0.0fs hover time; reserve type = " \
	% (sizing_mission_type, sizing_mission_range.to(ureg.nautical_mile).magnitude, sizing_N_passengers, sizing_t_hover.to(ureg.s).magnitude) \
	+ reserve_type_string + "\n"\
	+ "Revenue mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (revenue_mission_type, revenue_mission_range.to(ureg.nautical_mile).magnitude, \
		revenue_N_passengers, revenue_t_hover.to(ureg.s).magnitude, charger_power.to(ureg.kW).magnitude) \
	+ "Deadhead mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; deadhead ratio = %0.1f" \
	% (deadhead_mission_type, deadhead_mission_range.to(ureg.nautical_mile).magnitude, \
		deadhead_N_passengers, deadhead_t_hover.to(ureg.s).magnitude, deadhead_ratio)


plt.suptitle(title_str,fontsize = 13.5)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.94,bottom=0.08,top=0.87)
plt.savefig('noise_analysis_plot_01.pdf')


fig2 = plt.figure(figsize=(12,12), dpi=80)
plt.show()

for i, config in enumerate(configs):
	
	c = configs[config]

	f_fund = c["theta"]["periodic"]["f_fund"][0]
	SPL_periodic = c["theta"]["periodic"]["SPL"]
	SPL_vortex = c["theta"]["vortex"]["SPL"]*np.ones(np.size(theta_array))
	
	plt.subplot(2,2,i+1)
	plt.plot(theta_array.to(ureg.degree).magnitude,SPL_periodic,'k-',linewidth=3,
		label="Periodic noise")
	plt.plot(theta_array.to(ureg.degree).magnitude,SPL_vortex,'k--',linewidth=3,
		label="Vortex noise")
	plt.grid()
	plt.xlabel('$\Theta$ (degrees)', fontsize = 16)
	plt.ylabel('SPL (dB)', fontsize = 16)
	plt.title(config, fontsize = 18)
	plt.legend(loc="lower left")

plt.suptitle(title_str,fontsize = 13.5)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.94,bottom=0.08,top=0.87)
plt.savefig('noise_analysis_plot_02.pdf')


fig3 = plt.figure(figsize=(12,12), dpi=80)
plt.show()

for i, config in enumerate(configs):
	
	c = configs[config]

	f_fund = c["y"]["periodic"]["f_fund"][0]
	SPL_periodic = c["y"]["periodic"]["SPL"]
	SPL_vortex = c["y"]["vortex"]["SPL"]*np.ones(np.size(theta_array))
	
	plt.subplot(2,2,i+1)
	plt.plot(y_array.to(ureg.ft).magnitude,SPL_periodic,'k-',linewidth=3,
		label="Periodic noise")
	plt.plot(y_array.to(ureg.ft).magnitude,SPL_vortex,'k--',linewidth=3,
		label="Vortex noise")
	plt.grid()
	plt.xlabel('y (feet)', fontsize = 16)
	plt.ylabel('SPL (dB)', fontsize = 16)
	plt.title(config, fontsize = 18)
	plt.legend(loc="lower right")

plt.suptitle(title_str,fontsize = 13.5)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.94,bottom=0.08,top=0.87)
plt.savefig('noise_analysis_plot_03.pdf')