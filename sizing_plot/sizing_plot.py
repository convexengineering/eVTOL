#CGenerate carpet plot for vehicle sizing

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
from noise_models import vortex_noise

#General data
eta_cruise = generic_data["\eta_{cruise}"] 
eta_electric = generic_data["\eta_{electric}"]
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
sizing_t_hover = generic_data["sizing_mission"]["t_{hover}"]

revenue_mission_type = generic_data["revenue_mission"]["type"]
revenue_N_passengers = generic_data["revenue_mission"]["N_passengers"]
revenue_mission_range = generic_data["revenue_mission"]["range"]
revenue_t_hover = generic_data["revenue_mission"]["t_{hover}"]

deadhead_mission_type = generic_data["deadhead_mission"]["type"]
deadhead_N_passengers = generic_data["deadhead_mission"]["N_passengers"]
deadhead_mission_range = generic_data["deadhead_mission"]["range"]
deadhead_t_hover = generic_data["deadhead_mission"]["t_{hover}"]

numrows = 6
L_D_array = np.linspace(7,15,numrows)
T_A_array = np.linspace(4.5,16,numrows)
L_D_array, T_A_array = np.meshgrid(L_D_array, T_A_array)
T_A_array = T_A_array*ureg.lbf/ureg.ft**2

MTOW_array = np.zeros(np.shape(L_D_array))
cptpp_array = np.zeros(np.shape(L_D_array))
SPL_array = np.zeros(np.shape(L_D_array))
SPL_A_array = np.zeros(np.shape(L_D_array))

#Optimize 
configs = configuration_data.copy()
config = "Lift + cruise" #pull other data from this configuration
c = configs[config]


for i, T_A in enumerate(T_A_array[:,0]):
	for j, L_D_cruise in enumerate(L_D_array[0]):
		
		V_cruise = c["V_{cruise}"]
		Cl_mean_max = c["Cl_{mean_{max}}"]
		N = c["N"]
		loiter_type = c["loiter_type"]
		tailRotor_power_fraction_hover = c["tailRotor_power_fraction_hover"]
		tailRotor_power_fraction_levelFlight = c["tailRotor_power_fraction_levelFlight"]
		weight_fraction = c["weight_fraction"]

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

		MTOW_array[i,j] = solution("MTOW_OnDemandAircraft").to(ureg.lbf).magnitude
		cptpp_array[i,j] = solution("cost_per_trip_per_passenger_OnDemandMissionCost")
		
		#Noise computations
		T_perRotor = solution("T_perRotor_OnDemandSizingMission")[0]
		Q_perRotor = solution("Q_perRotor_OnDemandSizingMission")[0]
		R = solution("R")
		VT = solution("VT_OnDemandSizingMission")[0]
		s = solution("s")
		Cl_mean = solution("Cl_{mean_{max}}")
		N = solution("N")
		
		#Unweighted
		f_peak, SPL_array[i,j], spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
			weighting="None")
		
		#A-weighted
		f_peak, SPL_A_array[i,j], spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
			weighting="A")

MTOW_array = MTOW_array*ureg.lbf


#Generate sizing plot

plt.ion()
fig1 = plt.figure(figsize=(12,12), dpi=80)
plt.show()

#First set of lines
for i,L_D in enumerate(L_D_array[0,:]):
	cptpp_row = cptpp_array[:,i]
	SPL_A_row = SPL_A_array[:,i]
	plt.plot(cptpp_row,SPL_A_row,'k-',linewidth=2)
	
	x = cptpp_row[0]
	y = SPL_A_row[0]
	label = "L/D = %0.1f" % L_D
	plt.text(x+0.3,y-0.3,label,fontsize=16,rotation=-45)

#Second set of lines
for i,T_A in enumerate(T_A_array[:,0]):
	cptpp_row = cptpp_array[i,:]
	SPL_A_row = SPL_A_array[i,:]
	plt.plot(cptpp_row,SPL_A_row,'k-',linewidth=2)
	
	x = cptpp_row[-1]
	y = SPL_A_row[-1]
	label = "T/A = %0.1f lbf/ft$^2$" % T_A.to(ureg.lbf/ureg.ft**2).magnitude
	plt.text(x-15,y,label,fontsize=16,rotation=0)

	
locs,labels = plt.xticks()
new_xticks = [""]*len(locs)
for i,loc in enumerate(locs):
	new_xticks[i] = "\$%0.2f" % loc
plt.xticks(locs,new_xticks,fontsize=14)
plt.yticks(fontsize=14)

plt.grid()
[xmin,xmax] = plt.gca().get_xlim()
plt.xlim(xmin=xmin-15)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymin=ymin-1.5)
plt.xlabel('Cost per trip, per passenger', fontsize = 16)
plt.ylabel('SPL (dBA)', fontsize = 16)

if reserve_type == "FAA_aircraft" or reserve_type == "FAA_heli":
	num = solution("t_{loiter}_OnDemandSizingMission").to(ureg.minute).magnitude
	if reserve_type == "FAA_aircraft":
		reserve_type_string = "FAA aircraft VFR (%0.0f-minute loiter)" % num
	elif reserve_type == "FAA_heli":
		reserve_type_string = "FAA helicopter VFR (%0.0f-minute loiter)" % num
elif reserve_type == "Uber":
	num = solution["constants"]["R_{divert}_OnDemandSizingMission"].to(ureg.nautical_mile).magnitude
	reserve_type_string = " (%0.0f-nm diversion distance)" % num

if autonomousEnabled:
	autonomy_string = "autonomy enabled"
else:
	autonomy_string = "pilot required"

title_str = "Aircraft parameters: structural mass fraction = %0.2f; battery energy density = %0.0f Wh/kg; cruising speed = %0.0f mph\n" \
	% (weight_fraction, C_m.to(ureg.Wh/ureg.kg).magnitude,V_cruise.to(ureg.mph).magnitude) \
	+ "%0.0f rotors; %0.0f rotor blades; mean lift coefficient = %0.1f; %s. %s configuration.\n" \
	% (N, B, Cl_mean_max, autonomy_string,config) \
	+ "Sizing mission (%s): range = %0.0f nm; %0.0f passengers; %0.0fs hover time; reserve type = " \
	% (sizing_mission_type, sizing_mission_range.to(ureg.nautical_mile).magnitude, sizing_N_passengers, sizing_t_hover.to(ureg.s).magnitude) \
	+ reserve_type_string + "\n" \
	+ "Revenue mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (revenue_mission_type, revenue_mission_range.to(ureg.nautical_mile).magnitude, \
		revenue_N_passengers, revenue_t_hover.to(ureg.s).magnitude, charger_power.to(ureg.kW).magnitude) \
	+ "Deadhead mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; deadhead ratio = %0.1f" \
	% (deadhead_mission_type, deadhead_mission_range.to(ureg.nautical_mile).magnitude, \
		deadhead_N_passengers, deadhead_t_hover.to(ureg.s).magnitude, deadhead_ratio)

plt.title(title_str,fontsize = 13)
plt.tight_layout()
plt.subplots_adjust(left=0.07,right=0.96,bottom=0.07,top=0.9)
plt.savefig('sizing_plot_01.pdf')


#Sizing-plot data output (to text file)
output_data = open("sizing_plot_data.txt","w")

output_data.write("Sizing plot data\n\n")
output_data.write("Configuration: %s\n\n" % config)

output_data.write("L/D (dimensionless)\n\n")
for i in range(np.size(L_D_array[0])):
	for j, L_D in enumerate(L_D_array[i]):
		output_data.write("%0.2f\t" % L_D)
	output_data.write("\n")

output_data.write("\n")
output_data.write("T/A (lbf/ft^2)\n\n")
for i in range(np.size(T_A_array[0])):
	for j, T_A in enumerate(T_A_array[i]):
		T_A = T_A.to(ureg.lbf/ureg.ft**2).magnitude
		output_data.write("%0.2f\t" % T_A)
	output_data.write("\n")
	
output_data.write("\n")
output_data.write("MTOW (lbf)\n\n")
for i in range(np.size(MTOW_array[0])):
	for j, MTOW in enumerate(MTOW_array[i]):
		MTOW = MTOW.to(ureg.lbf).magnitude
		output_data.write("%0.2f\t" % MTOW)
	output_data.write("\n")
	
output_data.write("\n")
output_data.write("Cost per trip, per passenger\n\n")
for i in range(np.size(cptpp_array[0])):
	for j, cptpp in enumerate(cptpp_array[i]):
		output_data.write("%0.2f\t" % cptpp)
	output_data.write("\n")

output_data.write("\n")
output_data.write("SPL (unweighted)\n\n")
for i in range(np.size(SPL_array[0])):
	for j, SPL in enumerate(SPL_array[i]):
		output_data.write("%0.2f\t" % SPL)
	output_data.write("\n")

output_data.write("\n")
output_data.write("SPL (A-weighted)\n\n")
for i in range(np.size(SPL_A_array[0])):
	for j, SPL_A in enumerate(SPL_A_array[i]):
		output_data.write("%0.2f\t" % SPL_A)
	output_data.write("\n")

output_data.close()

