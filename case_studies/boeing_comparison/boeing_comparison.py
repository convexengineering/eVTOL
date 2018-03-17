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
from copy import deepcopy
from collections import OrderedDict
from standard_atmosphere import stdatmo

#Data from the Boeing study
boeing_data = {}

boeing_data["C_m"] = 400*ureg.Wh/ureg.kg
boeing_data["sizing_mission"] = {}
boeing_data["sizing_mission"]["range"] = 87*ureg.nautical_mile

boeing_data["Lift + cruise"] = {}
boeing_data["Lift + cruise"]["V_{cruise}"] = 150*ureg("mph")
boeing_data["Lift + cruise"]["L/D"] = 9.1
boeing_data["Lift + cruise"]["T/A"] = 7.3*ureg("lbf")/ureg("ft")**2
boeing_data["Lift + cruise"]["N"] = 8
boeing_data["Lift + cruise"]["TOGW"] = 3710*ureg.lbf
boeing_data["Lift + cruise"]["W_{battery}"] = 948*ureg.lbf
boeing_data["Lift + cruise"]["P_{cruise}"] = 199*ureg.hp
boeing_data["Lift + cruise"]["P_{hover}"] = 389*ureg.hp
boeing_data["Lift + cruise"]["VT_{hover}"] = 518*ureg.ft/ureg.s
boeing_data["Lift + cruise"]["R"] = 4.5*ureg.ft

boeing_data["Tilt rotor"] = {}
boeing_data["Tilt rotor"]["V_{cruise}"] = 150*ureg("mph")
boeing_data["Tilt rotor"]["L/D"] = 11.0
boeing_data["Tilt rotor"]["T/A"] = 12.8*ureg("lbf")/ureg("ft")**2
boeing_data["Tilt rotor"]["N"] = 8
boeing_data["Tilt rotor"]["TOGW"] = 3930*ureg.lbf
boeing_data["Tilt rotor"]["W_{battery}"] = 965*ureg.lbf
boeing_data["Tilt rotor"]["P_{cruise}"] = 187*ureg.hp
boeing_data["Tilt rotor"]["P_{hover}"] = 542*ureg.hp
boeing_data["Tilt rotor"]["VT_{hover}"] = 481*ureg.ft/ureg.s
boeing_data["Tilt rotor"]["R"] = 3.5*ureg.ft

boeing_data["Helicopter"] = {}
boeing_data["Helicopter"]["V_{cruise}"] = 150*ureg("mph")
boeing_data["Helicopter"]["L/D"] = 7.84
boeing_data["Helicopter"]["T/A"] = 4.1*ureg("lbf")/ureg("ft")**2
boeing_data["Helicopter"]["N"] = 1
boeing_data["Helicopter"]["TOGW"] = 3470*ureg.lbf
boeing_data["Helicopter"]["W_{battery}"] = 1170*ureg.lbf
boeing_data["Helicopter"]["P_{cruise}"] = 250*ureg.hp
boeing_data["Helicopter"]["P_{hover}"] = 347*ureg.hp
boeing_data["Helicopter"]["VT_{hover}"] = 683*ureg.ft/ureg.s
boeing_data["Helicopter"]["R"] = 16.5*ureg.ft

boeing_data["Helicopter"]["tailRotor_power_fraction_hover"] = 0.10
boeing_data["Helicopter"]["tailRotor_power_fraction_levelFlight"] = 0.08

#Generic data
generic_data["C_m"] = boeing_data["C_m"]
generic_data["sizing_mission"]["range"] = boeing_data["sizing_mission"]["range"]


# Data specific to study
configs = OrderedDict()
config_array = ["Lift + cruise", "Tilt rotor","Helicopter"]
case_array = ["GP model","GP model (with $\omega$ constraint)"]

#Atmospheric data (needed for thrust coefficient)
rho = stdatmo(h=0*ureg.ft)["\rho"] #air density

for config in config_array:
	
	configs[config] = OrderedDict()
	boeing_data[config]["\omega_{hover}"] = boeing_data[config]["VT_{hover}"]/boeing_data[config]["R"]
	boeing_data[config]["\omega_{hover}"] = boeing_data[config]["\omega_{hover}"].to(ureg.rpm)
	
	#Thrust coefficient
	boeing_data[config]["CT"] = boeing_data[config]["T/A"]/(0.5*rho*boeing_data[config]["VT_{hover}"]**2)
	boeing_data[config]["CT"] = boeing_data[config]["CT"].to(ureg.dimensionless)
	
	for case in case_array:
		configs[config][case] = configuration_data[config].copy()
		configs[config][case]["V_{cruise}"] = boeing_data[config]["V_{cruise}"]
		configs[config][case]["L/D"] = boeing_data[config]["L/D"]
		configs[config][case]["T/A"] = boeing_data[config]["T/A"]
		configs[config][case]["N"] = boeing_data[config]["N"]

		if config == "Helicopter":
			configs[config][case]["tailRotor_power_fraction_hover"] = boeing_data["Helicopter"]["tailRotor_power_fraction_hover"]
			configs[config][case]["tailRotor_power_fraction_levelFlight"] = boeing_data["Helicopter"]["tailRotor_power_fraction_levelFlight"]

#Set rotor solidities
for case in case_array:
	configs["Lift + cruise"][case]["s"] = 0.1
	configs["Tilt rotor"][case]["s"] = configs["Lift + cruise"][case]["s"]*boeing_data["Tilt rotor"]["CT"]/boeing_data["Lift + cruise"]["CT"]
	configs["Tilt rotor"][case]["s"] = configs["Tilt rotor"][case]["s"].magnitude
	configs["Helicopter"][case]["s"] = 0.1

configs["Tilt rotor"]["GP model (with $\omega$ constraint)"]["Cl_{mean_{max}}"] = 2.0
print "TILT ROTOR MEAN LIFT COEFFICIENT MODIFIED."

#Optimize remaining configurations
for config in configs:
	
	print "Solving configuration: " + config

	for case in case_array:

		c = configs[config][case]
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
		
		#Allow battery to use all of its energy, and set solidity
		problem_subDict.update({
			Aircraft.battery.usable_energy_fraction: 1.,
			Aircraft.rotors.s: c["s"],
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

		#Ensure total payload weight is exactly 800 lbf
		problem_subDict.update({
			SizingMission.crew.W_oneCrew: 200*ureg.lbf
		})


		if case == "GP model (with $\omega$ constraint)":
			problem_subDict.update({
				#SizingMission.VT[i]: boeing_data[config]["VT_{hover}"] for i in range(np.size(SizingMission.VT))
				SizingMission.omega[i]: boeing_data[config]["\omega_{hover}"] for i in range(np.size(SizingMission.omega))
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
		configs[config][case]["solution"] = solution

		#Noise computations (sizing mission)
		T_perRotor = solution("T_perRotor_OnDemandSizingMission")[0]
		Q_perRotor = solution("Q_perRotor_OnDemandSizingMission")[0]
		R = solution("R")
		VT = solution("VT_OnDemandSizingMission")[0]
		s = solution("s")
		Cl_mean = solution("Cl_{mean_{max}}")
		N = solution("N")

		B = generic_data["B"]
		delta_S = generic_data["delta_S"]

		#A-weighted
		f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
			weighting="A")
		configs[config][case]["SPL_sizing_A"] = SPL
		configs[config][case]["f_{peak}"] = f_peak
		configs[config][case]["spectrum_sizing_A"] = spectrum



print "% GP Takeoff Gross Weight Increases"
print
print "Case\t\t",
for case in case_array:
	print case + "\t",
print
for config in configs:
	print config + "\t",
	for case in case_array:
		TOGW_diff = configs[config][case]["solution"]("TOGW_OnDemandAircraft")-boeing_data[config]["TOGW"]
		TOGW_diff_percent = 100*(TOGW_diff/boeing_data[config]["TOGW"]).to(ureg.dimensionless)
		print "%0.1f%%\t\t" % TOGW_diff_percent,
	print


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

offset_array = [-0.3,0,0.3]
width = 0.2
colors = ["grey", "w", "k"]

legend_labels = list(case_array)
legend_labels.append("Boeing data")

#Takeoff gross weight
plt.subplot(3,2,1)
for i,config in enumerate(configs):
	for j,case in enumerate(configs[config]):
		c = configs[config][case]
		offset = offset_array[j]

		TOGW = c["solution"]("TOGW").to(ureg.lbf).magnitude

		if (i == 0):
			label = legend_labels[j]
			plt.bar(i+offset,TOGW,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,TOGW,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')
	plt.bar(i+offset_array[-1],boeing_data[config]["TOGW"].to(ureg.lbf).magnitude,
		align='center',alpha=1,width=width,color=colors[-1],edgecolor='k')

plt.grid()
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.7*ymax)
plt.title("Takeoff Gross Weight",fontsize = 18)
plt.legend(legend_labels,loc='upper left',framealpha=1, fontsize = 12)

#Battery weight
plt.subplot(3,2,2)
for i,config in enumerate(configs):
	for j,case in enumerate(configs[config]):
		c = configs[config][case]
		offset = offset_array[j]

		W_battery = c["solution"]("W_OnDemandAircraft/Battery").to(ureg.lbf).magnitude

		if (i == 0):
			label = legend_labels[j]
			plt.bar(i+offset,W_battery,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,W_battery,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')
	plt.bar(i+offset_array[-1],boeing_data[config]["W_{battery}"].to(ureg.lbf).magnitude,
		align='center',alpha=1,width=width,color=colors[-1],edgecolor='k')

plt.grid()
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.4*ymax)
plt.title("Battery Weight",fontsize = 18)
plt.legend(legend_labels,loc='upper left',framealpha=1, fontsize = 12)

#Rotor tip speed
plt.subplot(3,2,3)
for i,config in enumerate(configs):
	for j,case in enumerate(configs[config]):
		c = configs[config][case]
		offset = offset_array[j]

		VT_sizing = c["solution"]("VT_OnDemandSizingMission").to(ureg.ft/ureg.s).magnitude

		if (i == 0):
			label = legend_labels[j]
			plt.bar(i+offset,VT_sizing,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,VT_sizing,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')
	plt.bar(i+offset_array[-1],boeing_data[config]["VT_{hover}"].to(ureg.ft/ureg.s).magnitude,
		align='center',alpha=1,width=width,color=colors[-1],edgecolor='k')

plt.grid()
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Tip speed (ft/s)', fontsize = 16)
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.4*ymax)
plt.title("Rotor Tip Speed",fontsize = 18)
plt.legend(legend_labels,loc='upper left',framealpha=1, fontsize = 12)

#Sound pressure level in hover
plt.subplot(3,2,4)
for i,config in enumerate(configs):
	for j,case in enumerate(configs[config]):
		c = configs[config][case]
		offset = offset_array[j]

		SPL_sizing_A = c["SPL_sizing_A"]

		if (i == 0):
			label = legend_labels[j]
			plt.bar(i+offset,SPL_sizing_A,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,SPL_sizing_A,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')
	#plt.bar(i+offset_array[-1],boeing_data[config]["VT_{hover}"].to(ureg.ft/ureg.s).magnitude,align='center',alpha=1,width=width,color=colors[-1],edgecolor='k')

plt.grid()
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('SPL (dBA)', fontsize = 16)
plt.xlim(xmin=xmin,xmax=xmax)
plt.ylim(ymin=60, ymax = 78)
plt.title("Sound Pressure Level in Hover",fontsize = 18)
plt.legend(legend_labels[:-1],loc='upper left',framealpha=1, fontsize = 12)


#Power consumption (cruise)
plt.subplot(3,2,5)
for i,config in enumerate(configs):
	for j,case in enumerate(configs[config]):
		c = configs[config][case]
		offset = offset_array[j]

		P_cruise = c["solution"]("P_{battery}_OnDemandSizingMission")[1].to(ureg.kW).magnitude

		if (i == 0):
			label = legend_labels[j]
			plt.bar(i+offset,P_cruise,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,P_cruise,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')
	plt.bar(i+offset_array[-1],boeing_data[config]["P_{cruise}"].to(ureg.kW).magnitude,
		align='center',alpha=1,width=width,color=colors[-1],edgecolor='k')

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.5*ymax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Power (kW)', fontsize = 16)
plt.title("Cruise Power (sizing mission)",fontsize = 18)
plt.legend(legend_labels,loc='upper left',framealpha=1, fontsize = 12)

#Power consumption (hover)
plt.subplot(3,2,6)
for i,config in enumerate(configs):
	for j,case in enumerate(configs[config]):
		c = configs[config][case]
		offset = offset_array[j]

		P_hover = c["solution"]("P_{battery}_OnDemandSizingMission")[0].to(ureg.kW).magnitude

		if (i == 0):
			label = legend_labels[j]
			plt.bar(i+offset,P_hover,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,P_hover,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')
	plt.bar(i+offset_array[-1],boeing_data[config]["P_{hover}"].to(ureg.kW).magnitude,
		align='center',alpha=1,width=width,color=colors[-1],edgecolor='k')

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax = 1.8*ymax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Power (kW)', fontsize = 16)
plt.title("Hover Power (sizing mission)",fontsize = 18)
plt.legend(legend_labels,loc='upper left',framealpha=1, fontsize = 12)

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


plt.suptitle(title_str,fontsize = 13.0)
plt.tight_layout()
plt.subplots_adjust(left=0.08,right=0.99,bottom=0.10,top=0.88)
plt.savefig('boeing_comparison_plot_01.pdf')