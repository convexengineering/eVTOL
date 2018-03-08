#Sensitivity study to number of propeller blades. 
#Only affects rotational noise, since solidity is constant.

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
	

#Noise computations for varying theta (delta-S = constant)
B_array = [3,4,5,6]
theta_array = np.linspace(91,175,10)*ureg.degree

for config in configs:

	configs[config]["theta"] = {}
	
	configs[config]["theta"]["rotational"] = {}
	configs[config]["theta"]["rotational"]["f_fund"] = np.zeros([np.size(B_array),np.size(theta_array)])*ureg.turn/ureg.s
	configs[config]["theta"]["rotational"]["SPL_A"] = np.zeros([np.size(B_array),np.size(theta_array)])
	
	T_perRotor = configs[config]["solution"]("T_perRotor_OnDemandSizingMission")[0]
	Q_perRotor = configs[config]["solution"]("Q_perRotor_OnDemandSizingMission")[0]
	R = configs[config]["solution"]("R")
	VT = configs[config]["solution"]("VT_OnDemandSizingMission")[0]
	s = configs[config]["solution"]("s")
	Cl_mean = configs[config]["solution"]("Cl_{mean_{max}}")
	N = configs[config]["solution"]("N")

	delta_S = generic_data["delta_S"]

	#Rotational noise calculations (A-weighted)
	for i,B in enumerate(B_array):
		for j,theta in enumerate(theta_array):

			f_peak, SPL, spectrum = rotational_noise(T_perRotor,Q_perRotor,R,VT,s,N=N,B=B,
				theta=theta,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,num_harmonics=10,
				weighting="A")

			configs[config]["theta"]["rotational"]["f_fund"][i,j] = f_peak
			configs[config]["theta"]["rotational"]["SPL_A"][i,j] = SPL

	#Vortex noise computations (A-weighted)
	configs[config]["theta"]["vortex"] = {}

	f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
		Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
		weighting="A")

	configs[config]["theta"]["vortex"]["f_peak"] = f_peak
	configs[config]["theta"]["vortex"]["SPL_A"] = SPL
	configs[config]["theta"]["vortex"]["spectrum"] = spectrum


# Plotting commands
plt.ion()
plt.rc('axes', axisbelow=True)
fig1 = plt.figure(figsize=(12,12), dpi=80)
plt.show()

style = {}
style["linestyle"] = ["-","-","-","-","--","--","--","--"]
style["marker"] = ["s","o","^","v","s","o","^","v"]
style["fillstyle"] = ["full","full","full","full","none","none","none","none"]
style["markersize"] = 10

for i, config in enumerate(configs):
	
	c = configs[config]
	
	SPL_vortex = c["theta"]["vortex"]["SPL_A"]*np.ones(np.size(theta_array))
	
	plt.subplot(2,2,i+1)

	for j,B in enumerate(B_array):
		SPL_rotational = c["theta"]["rotational"]["SPL_A"][j,:]
		rotational_label = "Rotational noise (%0.0f blades)" % B
		plt.plot(theta_array.to(ureg.degree).magnitude,SPL_rotational,color="black",
			linewidth=1.5,linestyle=style["linestyle"][j],marker=style["marker"][j],
			markersize=style["markersize"],fillstyle=style["fillstyle"][j],
			label=rotational_label)
	
	plt.plot(theta_array.to(ureg.degree).magnitude,SPL_vortex,'k--',linewidth=3,
		label="Vortex noise")
	plt.ylim(ymin=0)
	plt.grid()
	plt.xlabel('$\Theta$ (degrees)', fontsize = 16)
	plt.ylabel('SPL (dBA)', fontsize = 16)
	plt.title(config, fontsize = 18)
	plt.legend(loc="lower left",fontsize=14,framealpha=1)


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

title_str = "Aircraft parameters: battery energy density = %0.0f Wh/kg; %s\n" \
	% (generic_data["C_m"].to(ureg.Wh/ureg.kg).magnitude, autonomy_string) \
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


plt.suptitle(title_str,fontsize = 13.0)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.94,bottom=0.08,top=0.87)
plt.savefig('number_of_blades_plot_01.pdf')
