# Noise analysis as part of vehicle top-level trade study

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + "/../../models"))

import numpy as np
from gpkit                  import Model, ureg
from copy                   import deepcopy
from matplotlib             import pyplot as plt
from aircraft_models        import OnDemandAircraft
from mission_models         import OnDemandSizingMission, OnDemandRevenueMission, OnDemandDeadheadMission
from cost_models            import OnDemandMissionCost
from noise_models           import rotational_noise, vortex_noise, noise_weighting
from standard_substitutions import generic_data, configs

configs = deepcopy(configs)

x = 500*ureg.ft

#Optimize
for config in configs:
	
	print "Solving configuration: " + config

	aircraft = OnDemandAircraft()
	aircraft = aircraft.standard_substitutions(config=config, autonomousEnabled=generic_data["autonomousEnabled"])

	sizing_mission = OnDemandSizingMission(aircraft=aircraft)
	sizing_mission = sizing_mission.standard_substitutions(piloted=generic_data["isSizingMissionPiloted"], reserve=generic_data["reserve"])

	revenue_mission = OnDemandRevenueMission(aircraft=aircraft)
	revenue_mission = revenue_mission.standard_substitutions(piloted=generic_data["isRevenueMissionPiloted"])

	deadhead_mission = OnDemandDeadheadMission(aircraft=aircraft)
	deadhead_mission = deadhead_mission.standard_substitutions(piloted=generic_data["isDeadheadMissionPiloted"])

	mission_cost = OnDemandMissionCost(aircraft=aircraft, revenue_mission=revenue_mission, deadhead_mission=deadhead_mission)
	mission_cost = mission_cost.standard_substitutions(isRevenueMissionPiloted=generic_data["isRevenueMissionPiloted"], isDeadheadMissionPiloted=generic_data["isDeadheadMissionPiloted"])

	objective_function = mission_cost.cpt
	problem            = Model(objective_function, [aircraft, sizing_mission, revenue_mission, deadhead_mission, mission_cost])
	solution           = problem.solve(verbosity=0)

	configs[config]["solution"] = solution



#Noise computations for varying theta (delta-S = constant)
print
print "Noise computations for varying theta"
theta_array = np.linspace(91, 175, 20) * ureg.degree

for config in configs:

	configs[config]["theta"] = {}
	
	configs[config]["theta"]["rotational"]             = {}
	configs[config]["theta"]["rotational"]["f_fund"]   = np.zeros(np.size(theta_array)) * ureg.turn / ureg.s
	configs[config]["theta"]["rotational"]["SPL"]      = np.zeros(np.size(theta_array))
	configs[config]["theta"]["rotational"]["spectrum"] = [{} for i in range(np.size(theta_array))]

	configs[config]["theta"]["rotational_A"]             = {}
	configs[config]["theta"]["rotational_A"]["f_fund"]   = np.zeros(np.size(theta_array)) * ureg.turn / ureg.s
	configs[config]["theta"]["rotational_A"]["SPL"]      = np.zeros(np.size(theta_array))
	configs[config]["theta"]["rotational_A"]["spectrum"] = [{} for i in range(np.size(theta_array))]

	solution = configs[config]["solution"]

	T_perRotor = solution("T_perRotor_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	Q_perRotor = solution("Q_perRotor_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	T_A        = solution("T/A_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	Cl_mean    = solution("Cl_{mean}_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	R          = solution("R_OnDemandAircraft/Rotors")
	s          = solution("s_OnDemandAircraft/Rotors")
	omega      = solution("\\omega_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	V_tip      = solution("v_{tip}_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	c_avg      = solution("c_{avg}_OnDemandAircraft/Rotors")
	t_avg      = solution("t_{avg}_OnDemandAircraft/Rotors")
	N          = solution("N_OnDemandAircraft/Rotors")
	B          = solution("B_OnDemandAircraft/Rotors")
	rho        = solution("\\rho_OnDemandSizingMission/HoverTakeoff/HoverFlightState/FixedStandardAtmosphere")
	a          = solution("a_OnDemandSizingMission/HoverTakeoff/HoverFlightState/FixedStandardAtmosphere")

	# Rotational noise calculations (vary theta; fixed delta_S)
	for i,theta in enumerate(theta_array):
		
		# Unweighted
		rotational_data         = rotational_noise(T_perRotor, Q_perRotor, R, omega, c_avg, t_avg, N, B, rho, a, theta=theta, delta_S=x, num_harmonics=10, weighting="None")
		[f_peak, SPL, spectrum] = rotational_data

		configs[config]["theta"]["rotational"]["f_fund"][i]   = f_peak
		configs[config]["theta"]["rotational"]["SPL"][i]      = SPL
		configs[config]["theta"]["rotational"]["spectrum"][i] = spectrum

		# A-weighted
		rotational_data         = rotational_noise(T_perRotor, Q_perRotor, R, omega, c_avg, t_avg, N, B, rho, a, theta=theta, delta_S=x, num_harmonics=10, weighting="A")
		[f_peak, SPL, spectrum] = rotational_data

		configs[config]["theta"]["rotational_A"]["f_fund"][i]   = f_peak
		configs[config]["theta"]["rotational_A"]["SPL"][i]      = SPL
		configs[config]["theta"]["rotational_A"]["spectrum"][i] = spectrum

	# Vortex noise computations
	configs[config]["theta"]["vortex"]   = {}
	configs[config]["theta"]["vortex_A"] = {}

	# Unweighted
	vortex_data             = vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S=x, St=generic_data["Strouhal_number"], weighting="None")
	[f_peak, SPL, spectrum] = vortex_data
	
	configs[config]["theta"]["vortex"]["f_peak"]   = f_peak
	configs[config]["theta"]["vortex"]["SPL"]      = SPL
	configs[config]["theta"]["vortex"]["spectrum"] = spectrum

	#A-weighted
	vortex_data             = vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S=x, St=generic_data["Strouhal_number"], weighting="A")
	[f_peak, SPL, spectrum] = vortex_data
	
	configs[config]["theta"]["vortex_A"]["f_peak"]   = f_peak
	configs[config]["theta"]["vortex_A"]["SPL"]      = SPL
	configs[config]["theta"]["vortex_A"]["spectrum"] = spectrum


#Computations for varying y (delta-S is not constant)
print "Noise computations for varying y"
y_array = np.linspace(50, 3000, 20) * ureg.ft

for config in configs:

	configs[config]["y"] = {}
	
	configs[config]["y"]["rotational"]             = {}
	configs[config]["y"]["rotational"]["f_fund"]   = np.zeros(np.size(y_array)) * ureg.turn / ureg.s
	configs[config]["y"]["rotational"]["SPL"]      = np.zeros(np.size(y_array))
	configs[config]["y"]["rotational"]["spectrum"] = [{} for i in range(np.size(y_array))]

	configs[config]["y"]["rotational_A"]             = {}
	configs[config]["y"]["rotational_A"]["f_fund"]   = np.zeros(np.size(y_array)) * ureg.turn / ureg.s
	configs[config]["y"]["rotational_A"]["SPL"]      = np.zeros(np.size(y_array))
	configs[config]["y"]["rotational_A"]["spectrum"] = [{} for i in range(np.size(y_array))]

	configs[config]["y"]["vortex"]             = {}
	configs[config]["y"]["vortex"]["f_peak"]   = np.zeros(np.size(y_array)) * ureg.turn / ureg.s
	configs[config]["y"]["vortex"]["SPL"]      = np.zeros(np.size(y_array))
	configs[config]["y"]["vortex"]["spectrum"] = [{} for i in range(np.size(y_array))]

	configs[config]["y"]["vortex_A"]             = {}
	configs[config]["y"]["vortex_A"]["f_peak"]   = np.zeros(np.size(y_array)) * ureg.turn / ureg.s
	configs[config]["y"]["vortex_A"]["SPL"]      = np.zeros(np.size(y_array))
	configs[config]["y"]["vortex_A"]["spectrum"] = [{} for i in range(np.size(y_array))]

	configs[config]["y"]["total"]          = {}
	configs[config]["y"]["total"]["SPL"]   = np.zeros(np.size(y_array))
	configs[config]["y"]["total"]["SPL_A"] = np.zeros(np.size(y_array))

	solution = configs[config]["solution"]

	T_perRotor = solution("T_perRotor_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	Q_perRotor = solution("Q_perRotor_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	T_A        = solution("T/A_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	Cl_mean    = solution("Cl_{mean}_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	R          = solution("R_OnDemandAircraft/Rotors")
	s          = solution("s_OnDemandAircraft/Rotors")
	omega      = solution("\\omega_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	V_tip      = solution("v_{tip}_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	c_avg      = solution("c_{avg}_OnDemandAircraft/Rotors")
	t_avg      = solution("t_{avg}_OnDemandAircraft/Rotors")
	N          = solution("N_OnDemandAircraft/Rotors")
	B          = solution("B_OnDemandAircraft/Rotors")
	rho        = solution("\\rho_OnDemandSizingMission/HoverTakeoff/HoverFlightState/FixedStandardAtmosphere")
	a          = solution("a_OnDemandSizingMission/HoverTakeoff/HoverFlightState/FixedStandardAtmosphere")

	#Noise calculations
	for i,y in enumerate(y_array):

		theta   = 180*ureg.degree - (np.arctan(y/x)*ureg.radian).to(ureg.degree)
		delta_S = np.sqrt(x**2 + y**2)

		# Rotational noise (unweighted)
		rotational_data         = rotational_noise(T_perRotor, Q_perRotor, R, omega, c_avg, t_avg, N, B, rho, a, theta=theta, delta_S=delta_S, num_harmonics=10, weighting="None")
		[f_peak, SPL, spectrum] = rotational_data

		configs[config]["y"]["rotational"]["f_fund"][i]   = f_peak
		configs[config]["y"]["rotational"]["SPL"][i]      = SPL
		configs[config]["y"]["rotational"]["spectrum"][i] = spectrum

		# Rotational noise (A-weighted)
		rotational_data         = rotational_noise(T_perRotor, Q_perRotor, R, omega, c_avg, t_avg, N, B, rho, a, theta=theta, delta_S=delta_S, num_harmonics=10, weighting="A")
		[f_peak, SPL, spectrum] = rotational_data

		configs[config]["y"]["rotational_A"]["f_fund"][i]   = f_peak
		configs[config]["y"]["rotational_A"]["SPL"][i]      = SPL
		configs[config]["y"]["rotational_A"]["spectrum"][i] = spectrum

		# Vortex noise (unweighted)
		vortex_data             = vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S=delta_S, St=generic_data["Strouhal_number"], weighting="None")
		[f_peak, SPL, spectrum] = vortex_data
		
		configs[config]["y"]["vortex"]["f_peak"][i]   = f_peak
		configs[config]["y"]["vortex"]["SPL"][i]      = SPL
		configs[config]["y"]["vortex"]["spectrum"][i] = spectrum

		# Vortex noise (A-weighted)
		vortex_data             = vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S=delta_S, St=generic_data["Strouhal_number"], weighting="A")
		[f_peak, SPL, spectrum] = vortex_data
		
		configs[config]["y"]["vortex_A"]["f_peak"][i]   = f_peak
		configs[config]["y"]["vortex_A"]["SPL"][i]      = SPL
		configs[config]["y"]["vortex_A"]["spectrum"][i] = spectrum

		#Total noise
		p_ratio_squared   = 10**(configs[config]["y"]["rotational"]["SPL"][i]/10)   + 10**(configs[config]["y"]["vortex"]["SPL"][i]/10)
		p_ratio_squared_A = 10**(configs[config]["y"]["rotational_A"]["SPL"][i]/10) + 10**(configs[config]["y"]["vortex_A"]["SPL"][i]/10)

		configs[config]["y"]["total"]["SPL"][i]   = 10 * np.log10(p_ratio_squared)
		configs[config]["y"]["total"]["SPL_A"][i] = 10 * np.log10(p_ratio_squared_A)


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
	f_dBA_offset = np.linspace(f_min_rev_per_s,f_max_rev_per_s,100) * ureg.turn / ureg.s
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

plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.94,bottom=0.08,top=0.87)
plt.savefig('config_tradeStudy_noise_analysis_plot_04.pdf')


# Plot showing noise spectra (both rotational & vortex) for a sample value of y. Lift+cruise vehicle only.
fig5 = plt.figure(figsize=(6,5), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

ax = plt.subplot(1,1,1)
ax.set_axisbelow(True)

#Find value of y closest to that desired
y_desired  = 200 * ureg.m
idx        = (np.abs(y_array - y_desired)).argmin()
y_selected = y_array[idx]


c = configs["Lift + cruise"]

# Plot rotational noise harmonics
for i in range(3):

	rotational_f   = c["y"]["rotational"]["spectrum"][idx]["f"][i].to(ureg.turn/ureg.s).magnitude
	rotational_SPL = c["y"]["rotational"]["spectrum"][idx]["SPL"][i]

	if i==0:
		ax.bar(rotational_f, rotational_SPL, align="center", color='k', width=0.3*rotational_f, label="Rotational noise")
	else:
		ax.bar(rotational_f, rotational_SPL, align="center", color='k', width=0.3*rotational_f)

# Plot vortex noise spectrum
vortex_f_spectrum   = c["y"]["vortex"]["spectrum"][idx]["f"].to(ureg.turn/ureg.s).magnitude
vortex_SPL_spectrum = c["y"]["vortex"]["spectrum"][idx]["SPL"]

ax.plot(vortex_f_spectrum, vortex_SPL_spectrum, 'k-', linewidth=3, label="Vortex noise")

plt.grid(zorder=0)
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)
plt.xlabel('Frequency (Hz)', fontsize=16)
plt.ylabel('SPL (dB)', fontsize = 16)
	
#ymax = np.max(SPL_spectrum) + 5
#plt.ylim(ymax=ymax)

ax2 = ax.twinx()
ax2.set_axisbelow(True)

# Plot A-weighting spectrum on alternate axes
f_min = c["y"]["rotational"]["spectrum"][idx]["f"][0].to(ureg.turn/ureg.s).magnitude
f_max = np.max(vortex_f_spectrum)

f_dBA_offset = np.linspace(f_min_rev_per_s, f_max_rev_per_s, 100) * ureg.turn / ureg.s
dBA_offset   = noise_weighting(f_dBA_offset,np.zeros(np.shape(f_dBA_offset)))

ax2.plot(f_dBA_offset.to(ureg.turn/ureg.s).magnitude, dBA_offset,'k--',linewidth=3, label="A-weighting offset")
plt.ylabel('SPL offset (dBA)', fontsize=16)
	
plt.xscale('log')
ax2.tick_params(axis="y", labelsize=16)
subtitle_str = "Noise spectrum at y = %0.0f m \n(lift + cruise configuration)" % y_selected.to(ureg.m).magnitude
plt.title(subtitle_str, fontsize = 18)

lines, labels   = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax2.legend(lines + lines2, labels + labels2, fontsize=16, loc="lower right", framealpha=1)

plt.tight_layout()
plt.subplots_adjust(left=0.12,right=0.85,bottom=0.14,top=0.88)
plt.savefig('config_tradeStudy_noise_analysis_plot_05.pdf')


# Plot showing A-weighted noise. Lift+cruise vehicle only.
fig6 = plt.figure(figsize=(6,5), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

f_fund           = c["y"]["rotational"]["f_fund"][0]
SPL_rotational   = c["y"]["rotational"]["SPL"]
SPL_rotational_A = c["y"]["rotational_A"]["SPL"]
SPL_vortex_A     = c["y"]["vortex_A"]["SPL"]
SPL_total_A      = c["y"]["total"]["SPL_A"]
	
plt.plot(y_array.to(ureg.m).magnitude,  SPL_rotational_A, 'k--',linewidth=3, label="Rotational noise")
plt.plot(y_array.to(ureg.m).magnitude,  SPL_vortex_A,     linestyle="none",color="k",marker="o",fillstyle="full",markersize=10,label="Vortex noise")
plt.plot(y_array.to(ureg.m).magnitude,  SPL_total_A,      'k-',linewidth=3, label="Total")
	
plt.grid()
plt.ylim(ymin=0,ymax=max(SPL_total_A)+10)
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)
plt.xlabel('y (m)', fontsize= 16)
plt.ylabel('SPL (dBA)', fontsize=16)
plt.title("SPL vs. Observer Location \n(lift + cruise configuration)", fontsize=18)
plt.legend(loc="lower right", fontsize=16,framealpha=1)

plt.tight_layout()
plt.subplots_adjust(left=0.12,right=0.99,bottom=0.13,top=0.88)
plt.savefig('config_tradeStudy_noise_analysis_plot_06.pdf')
