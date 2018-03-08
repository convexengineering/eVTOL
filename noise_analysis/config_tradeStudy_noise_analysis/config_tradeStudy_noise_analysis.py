#Noise analysis as part of vehicle top-level trade study

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
from noise_models import rotational_noise, vortex_noise, noise_weighting

# Delete some configurations
configs = configuration_data.copy()
del configs["Tilt duct"]
del configs["Multirotor"]
del configs["Autogyro"]
del configs["Helicopter"]
del configs["Coaxial heli"]

#Optimize remaining configurations
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
		SizingMission.passengers.N_passengers: generic_data["sizing_mission"]["N_{passengers}"],#Number of passengers
	})

	RevenueMission = OnDemandRevenueMission(Aircraft,mission_type=generic_data["revenue_mission"]["type"])
	problem_subDict.update({
		RevenueMission.mission_range: generic_data["revenue_mission"]["range"],#mission range
		RevenueMission.V_cruise: c["V_{cruise}"],#cruising speed
		RevenueMission.t_hover: generic_data["revenue_mission"]["t_{hover}"],#hover time
		RevenueMission.passengers.N_passengers: generic_data["revenue_mission"]["N_{passengers}"],#Number of passengers
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


B = generic_data["B"]
x = 500*ureg.ft

#Noise computations for varying theta (delta-S = constant)
print
print "Noise computations for varying theta"
theta_array = np.linspace(91,175,20)*ureg.degree

for config in configs:

	configs[config]["theta"] = {}
	
	configs[config]["theta"]["rotational"] = {}
	configs[config]["theta"]["rotational"]["f_fund"] = np.zeros(np.size(theta_array))*ureg.turn/ureg.s
	configs[config]["theta"]["rotational"]["SPL"] = np.zeros(np.size(theta_array))
	configs[config]["theta"]["rotational"]["spectrum"] = [{} for i in range(np.size(theta_array))]

	configs[config]["theta"]["rotational_A"] = {}
	configs[config]["theta"]["rotational_A"]["f_fund"] = np.zeros(np.size(theta_array))*ureg.turn/ureg.s
	configs[config]["theta"]["rotational_A"]["SPL"] = np.zeros(np.size(theta_array))
	configs[config]["theta"]["rotational_A"]["spectrum"] = [{} for i in range(np.size(theta_array))]

	T_perRotor = configs[config]["solution"]("T_perRotor_OnDemandSizingMission")[0]
	Q_perRotor = configs[config]["solution"]("Q_perRotor_OnDemandSizingMission")[0]
	R = configs[config]["solution"]("R")
	VT = configs[config]["solution"]("VT_OnDemandSizingMission")[0]
	s = configs[config]["solution"]("s")
	Cl_mean = configs[config]["solution"]("Cl_{mean_{max}}")
	N = configs[config]["solution"]("N")

	#Rotational noise calculations
	for i,theta in enumerate(theta_array):
		
		#Unweighted
		f_peak, SPL, spectrum = rotational_noise(T_perRotor,Q_perRotor,R,VT,s,N,B,theta=theta,
			delta_S=x,h=0*ureg.ft,t_c=0.12,num_harmonics=10,weighting="None")
		configs[config]["theta"]["rotational"]["f_fund"][i] = f_peak
		configs[config]["theta"]["rotational"]["SPL"][i] = SPL
		configs[config]["theta"]["rotational"]["spectrum"][i] = spectrum

		#A-weighted
		f_peak, SPL, spectrum = rotational_noise(T_perRotor,Q_perRotor,R,VT,s,N,B,theta=theta,
			delta_S=x,h=0*ureg.ft,t_c=0.12,num_harmonics=10,weighting="A")
		configs[config]["theta"]["rotational_A"]["f_fund"][i] = f_peak
		configs[config]["theta"]["rotational_A"]["SPL"][i] = SPL
		configs[config]["theta"]["rotational_A"]["spectrum"][i] = spectrum

	#Vortex noise computations
	configs[config]["theta"]["vortex"] = {}
	configs[config]["theta"]["vortex_A"] = {}

	#Unweighted
	f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,Cl_mean=Cl_mean,
		N=N,B=B,delta_S=x,h=0*ureg.ft,t_c=0.12,St=0.28,weighting="None")
	configs[config]["theta"]["vortex"]["f_peak"] = f_peak
	configs[config]["theta"]["vortex"]["SPL"] = SPL
	configs[config]["theta"]["vortex"]["spectrum"] = spectrum

	#A-weighted
	f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,Cl_mean=Cl_mean,
		N=N,B=B,delta_S=x,h=0*ureg.ft,t_c=0.12,St=0.28,weighting="A")
	configs[config]["theta"]["vortex_A"]["f_peak"] = f_peak
	configs[config]["theta"]["vortex_A"]["SPL"] = SPL
	configs[config]["theta"]["vortex_A"]["spectrum"] = spectrum


#Computations for varying y (delta-S is not constant)
print "Noise computations for varying y"
y_array = np.linspace(50,3000,20)*ureg.ft

for config in configs:

	configs[config]["y"] = {}
	
	configs[config]["y"]["rotational"] = {}
	configs[config]["y"]["rotational"]["f_fund"] = np.zeros(np.size(y_array))*ureg.turn/ureg.s
	configs[config]["y"]["rotational"]["SPL"] = np.zeros(np.size(y_array))
	configs[config]["y"]["rotational"]["spectrum"] = [{} for i in range(np.size(y_array))]

	configs[config]["y"]["rotational_A"] = {}
	configs[config]["y"]["rotational_A"]["f_fund"] = np.zeros(np.size(y_array))*ureg.turn/ureg.s
	configs[config]["y"]["rotational_A"]["SPL"] = np.zeros(np.size(y_array))
	configs[config]["y"]["rotational_A"]["spectrum"] = [{} for i in range(np.size(y_array))]

	configs[config]["y"]["vortex"] = {}
	configs[config]["y"]["vortex"]["f_peak"] = np.zeros(np.size(y_array))*ureg.turn/ureg.s
	configs[config]["y"]["vortex"]["SPL"] = np.zeros(np.size(y_array))
	configs[config]["y"]["vortex"]["spectrum"] = [{} for i in range(np.size(y_array))]

	configs[config]["y"]["vortex_A"] = {}
	configs[config]["y"]["vortex_A"]["f_peak"] = np.zeros(np.size(y_array))*ureg.turn/ureg.s
	configs[config]["y"]["vortex_A"]["SPL"] = np.zeros(np.size(y_array))
	configs[config]["y"]["vortex_A"]["spectrum"] = [{} for i in range(np.size(y_array))]

	configs[config]["y"]["total"] = {}
	configs[config]["y"]["total"]["SPL"] = np.zeros(np.size(y_array))
	configs[config]["y"]["total"]["SPL_A"] = np.zeros(np.size(y_array))

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

		#Rotational noise (unweighted)
		f_peak, SPL, spectrum = rotational_noise(T_perRotor,Q_perRotor,R,VT,s,N,B,theta=theta,
			delta_S=delta_S,h=0*ureg.ft,t_c=0.12,num_harmonics=20,weighting="None")
		configs[config]["y"]["rotational"]["f_fund"][i] = f_peak
		configs[config]["y"]["rotational"]["SPL"][i] = SPL
		configs[config]["y"]["rotational"]["spectrum"][i] = spectrum

		#Rotational noise (A-weighted)
		f_peak, SPL, spectrum = rotational_noise(T_perRotor,Q_perRotor,R,VT,s,N,B,theta=theta,
			delta_S=delta_S,h=0*ureg.ft,t_c=0.12,num_harmonics=20,weighting="A")
		configs[config]["y"]["rotational_A"]["f_fund"][i] = f_peak
		configs[config]["y"]["rotational_A"]["SPL"][i] = SPL
		configs[config]["y"]["rotational_A"]["spectrum"][i] = spectrum

		#Vortex noise (unweighted)
		f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,weighting="None")
		configs[config]["y"]["vortex"]["f_peak"][i] = f_peak
		configs[config]["y"]["vortex"]["SPL"][i] = SPL
		configs[config]["y"]["vortex"]["spectrum"][i] = spectrum

		#Vortex noise (A-weighted)
		f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,weighting="A")
		configs[config]["y"]["vortex_A"]["f_peak"][i] = f_peak
		configs[config]["y"]["vortex_A"]["SPL"][i] = SPL
		configs[config]["y"]["vortex_A"]["spectrum"][i] = spectrum

		#Total noise
		p_ratio_squared = 10**(configs[config]["y"]["rotational"]["SPL"][i]/10)\
			+ 10**(configs[config]["y"]["vortex"]["SPL"][i]/10)
		p_ratio_squared_A = 10**(configs[config]["y"]["rotational_A"]["SPL"][i]/10)\
			+ 10**(configs[config]["y"]["vortex_A"]["SPL"][i]/10)

		configs[config]["y"]["total"]["SPL"][i] = 10*np.log10(p_ratio_squared)
		configs[config]["y"]["total"]["SPL_A"][i] = 10*np.log10(p_ratio_squared_A)




# Plotting commands
plt.ion()

#Plot showing noise spectra (both rotational & vortex) for a sample value of y
fig1 = plt.figure(figsize=(12,12), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

#Find value of y closest to that desired
y_desired = 1000*ureg.feet
idx = (np.abs(y_array - y_desired)).argmin()
y_selected = y_array[idx]

for i, config in enumerate(configs):
	
	c = configs[config]	
	
	#Rotational noise (1st harmonic only)
	rotational_f = c["y"]["rotational"]["spectrum"][idx]["f"][0]
	rotational_SPL = c["y"]["rotational"]["spectrum"][idx]["SPL"][0]
	rotational_SPL_A = noise_weighting(rotational_f,rotational_SPL)

	#Vortex noise
	vortex_f_spectrum = c["y"]["vortex"]["spectrum"][idx]["f"]
	vortex_SPL_spectrum = c["y"]["vortex"]["spectrum"][idx]["SPL"]
	vortex_SPL_A_spectrum = noise_weighting(vortex_f_spectrum,vortex_SPL_spectrum)

	#A-weighting spectrum
	f_min_rev_per_s = rotational_f.to(ureg.turn/ureg.s).magnitude
	f_max_rev_per_s = np.max(vortex_f_spectrum.to(ureg.turn/ureg.s).magnitude)
	f_dBA_offset = np.linspace(f_min_rev_per_s,f_max_rev_per_s,100)*ureg.turn/ureg.s
	dBA_offset = noise_weighting(f_dBA_offset,np.zeros(np.shape(f_dBA_offset)))
	
	ax = fig1.add_subplot(2,2,i+1)

	ax.bar(rotational_f.to(ureg.turn/ureg.s).magnitude,rotational_SPL,align="center",
		color='k',width=0.2*rotational_f.to(ureg.turn/ureg.s).magnitude,
		label="Rotational noise")
	#ax.bar(1.15*rotational_f.to(ureg.turn/ureg.s).magnitude,rotational_SPL_A,align="center",
	#	color='grey',width=0.3*rotational_f.to(ureg.turn/ureg.s).magnitude,
	#	label="A-weighted rotational noise")
	
	ax.plot(vortex_f_spectrum.to(ureg.turn/ureg.s).magnitude,vortex_SPL_spectrum,
		'k-',linewidth=2,label="Vortex noise")
	#ax.plot(vortex_f_spectrum.to(ureg.turn/ureg.s).magnitude,vortex_SPL_A_spectrum,
	#	'k-.',linewidth=2,label="A-weighted vortex noise")

	plt.xticks(fontsize=12)
	plt.yticks(fontsize=12)
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
	subtitle_str = config + " (y = %0.0f ft)" % y_selected.to(ureg.ft).magnitude
	plt.title(subtitle_str, fontsize = 18)

	lines, labels = ax.get_legend_handles_labels()
	lines2, labels2 = ax2.get_legend_handles_labels()
	ax2.legend(lines + lines2, labels + labels2, fontsize=13,loc="lower right", framealpha=1)


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
	+ "Sizing mission (%s): range = %0.0f nmi; %0.0f passengers; %0.0fs hover time; reserve type = " \
	% (generic_data["sizing_mission"]["type"], generic_data["sizing_mission"]["range"].to(ureg.nautical_mile).magnitude,\
	 generic_data["sizing_mission"]["N_{passengers}"], generic_data["sizing_mission"]["t_{hover}"].to(ureg.s).magnitude)\
	+ reserve_type_string + "\n"\
	+ "Revenue mission (%s): range = %0.0f nmi; %0.1f passengers; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (generic_data["revenue_mission"]["type"], generic_data["revenue_mission"]["range"].to(ureg.nautical_mile).magnitude, \
	 generic_data["revenue_mission"]["N_{passengers}"], generic_data["revenue_mission"]["t_{hover}"].to(ureg.s).magnitude,\
	 generic_data["charger_power"].to(ureg.kW).magnitude) \
	+ "Deadhead mission (%s): range = %0.0f nmi; %0.1f passengers; %0.0fs hover time; no reserve; deadhead ratio = %0.1f" \
	% (generic_data["deadhead_mission"]["type"], generic_data["deadhead_mission"]["range"].to(ureg.nautical_mile).magnitude, \
	 generic_data["deadhead_mission"]["N_{passengers}"], generic_data["deadhead_mission"]["t_{hover}"].to(ureg.s).magnitude,\
	 generic_data["deadhead_ratio"])


plt.suptitle(title_str,fontsize = 13)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.94,bottom=0.08,top=0.87)
plt.savefig('config_tradeStudy_noise_analysis_plot_01.pdf')

#Plot of SPL vs. theta
fig2 = plt.figure(figsize=(12,12), dpi=80)
plt.show()

for i, config in enumerate(configs):
	
	c = configs[config]
	
	SPL_rotational = c["theta"]["rotational"]["SPL"]
	SPL_vortex = c["theta"]["vortex"]["SPL"]*np.ones(np.size(theta_array))
	SPL_rotational_A = c["theta"]["rotational_A"]["SPL"]
	SPL_vortex_A = c["theta"]["vortex_A"]["SPL"]*np.ones(np.size(theta_array))

	#SPL from first harmonic only
	SPL_rotational_m1 = np.zeros(np.size(c["theta"]["rotational"]["spectrum"]))
	SPL_rotational_m1_A = np.zeros(np.size(c["theta"]["rotational_A"]["spectrum"]))
	
	for j,spectrum in enumerate(c["theta"]["rotational"]["spectrum"]):
		SPL_rotational_m1[j] = spectrum["SPL"][0]

	for j,spectrum in enumerate(c["theta"]["rotational_A"]["spectrum"]):
		SPL_rotational_m1_A[j] = spectrum["SPL"][0]
	
	plt.subplot(2,2,i+1)
	plt.plot(theta_array.to(ureg.degree).magnitude,SPL_rotational,'k-.',linewidth=3,
		label="Rotational noise (unweighted)")
	plt.plot(theta_array.to(ureg.degree).magnitude,SPL_rotational_m1,color="k",linestyle="none",
		marker="o",fillstyle="full",markersize=10,label="Rotational noise (unweighted; m=1 only)")
	plt.plot(theta_array.to(ureg.degree).magnitude,SPL_rotational_A,'k:',linewidth=3,
		label="Rotational noise (A-weighted)")
	plt.plot(theta_array.to(ureg.degree).magnitude,SPL_rotational_m1_A,color="k",linestyle="none",
		marker="s",fillstyle="full",markersize=10,label="Rotational noise (A-weighted; m=1 only)")
	plt.plot(theta_array.to(ureg.degree).magnitude,SPL_vortex,'k--',linewidth=3,
		label="Vortex noise (unweighted)")
	plt.plot(theta_array.to(ureg.degree).magnitude,SPL_vortex_A,'k-',linewidth=3,
		label="Vortex noise (A-weighted)")

	plt.ylim(ymin = 0,ymax=np.max(SPL_vortex)+10)
	plt.grid()
	plt.xticks(fontsize=12)
	plt.yticks(fontsize=12)
	plt.xlabel('$\Theta$ (degrees)', fontsize = 16)
	plt.ylabel('SPL (dB)', fontsize = 16)
	plt.title(config, fontsize = 18)
	plt.legend(loc="lower left",fontsize=12, framealpha=1)

plt.suptitle(title_str,fontsize = 13)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.94,bottom=0.08,top=0.87)
plt.savefig('config_tradeStudy_noise_analysis_plot_02.pdf')


fig3 = plt.figure(figsize=(12,12), dpi=80)
plt.show()

for i, config in enumerate(configs):
	
	c = configs[config]

	f_fund = c["y"]["rotational"]["f_fund"][0]
	SPL_rotational = c["y"]["rotational"]["SPL"]
	SPL_rotational_A = c["y"]["rotational_A"]["SPL"]
	SPL_vortex_A = c["y"]["vortex_A"]["SPL"]
	SPL_total_A = c["y"]["total"]["SPL_A"]
	
	plt.subplot(2,2,i+1)
	#plt.plot(y_array.to(ureg.ft).magnitude,SPL_rotational,'k--',linewidth=3,
	#	label="Rotational noise (unweighted)")
	plt.plot(y_array.to(ureg.ft).magnitude,SPL_rotational_A,'k-.',linewidth=3,
		label="Rotational noise (A-weighted)")
	plt.plot(y_array.to(ureg.ft).magnitude,SPL_vortex_A,linestyle="none",color="k",
		marker="o",fillstyle="full",markersize=10,label="Vortex noise (A-weighted)")
	plt.plot(y_array.to(ureg.ft).magnitude,SPL_total_A,'k-',linewidth=3,
		label="Total noise (A-weighted)")
	
	plt.grid()
	plt.ylim(ymin=0,ymax=max(SPL_total_A)+10)
	plt.xticks(fontsize=12)
	plt.yticks(fontsize=12)
	plt.xlabel('y (feet)', fontsize = 16)
	plt.ylabel('SPL (dB)', fontsize = 16)
	plt.title(config, fontsize = 18)
	plt.legend(loc="lower right", fontsize=13,framealpha=1)

plt.suptitle(title_str,fontsize = 13)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.94,bottom=0.08,top=0.87)
plt.savefig('config_tradeStudy_noise_analysis_plot_03.pdf')


#Display noise spectrum for different values of theta
fig4 = plt.figure(figsize=(12,12), dpi=80)
plt.show()

config = "Lift + cruise"
c = configs[config]

theta_plot_values = np.linspace(100,160,4)*ureg.degree #desired angles

for i, theta_desired in enumerate(theta_plot_values):
	
	theta_idx = (np.abs(theta_array - theta_desired)).argmin()
	theta = theta_array[theta_idx]

	rotational_f_spectrum = c["theta"]["rotational"]["spectrum"][theta_idx]["f"][0:5]
	rotational_SPL_spectrum = c["theta"]["rotational"]["spectrum"][theta_idx]["SPL"][0:5]
	
	SPL_min = np.min(rotational_SPL_spectrum) - 10
	SPL_min = np.max([SPL_min,0])#Only show SPL values above this

	plt.subplot(2,2,i+1)

	for j,SPL in enumerate(rotational_SPL_spectrum):
		f = rotational_f_spectrum[j].to(ureg.turn/ureg.s).magnitude
		plt.bar(f,SPL,width=50,align="center",color='k')

	plt.grid()
	plt.ylim(ymin=SPL_min)
	plt.xticks(fontsize=12)
	plt.yticks(fontsize=12)
	plt.xlabel('f (Hz)', fontsize = 16)
	plt.ylabel('SPL (dB)', fontsize = 16)
	subtitle_str = config + " ($\Theta$ = %0.1f$^\circ$)" % theta.to(ureg.degree).magnitude
	plt.title(subtitle_str, fontsize = 18)

plt.suptitle(title_str,fontsize = 13)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.94,bottom=0.08,top=0.87)
plt.savefig('config_tradeStudy_noise_analysis_plot_04.pdf')