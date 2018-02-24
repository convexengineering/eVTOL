#Case study on the configuration of the Joby S4

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

#Generic data

generic_data["sizing_mission"]["range"] = 100*ureg.nautical_mile
generic_data["revenue_mission"]["range"] = 50*ureg.nautical_mile
generic_data["deadhead_mission"]["range"] = 50*ureg.nautical_mile


# Delete some configurations

configs = {}

V_cruise = 200*ureg.mph

configs["12 rotors"] = configuration_data["Tilt rotor"].copy()
configs["12 rotors"]["V_{cruise}"] = V_cruise
configs["12 rotors"]["T/A"] = 16.3*ureg.lbf/ureg.ft**2
configs["12 rotors"]["N"] = 12

configs["6 rotors"] = configuration_data["Tilt rotor"].copy()
configs["6 rotors"]["V_{cruise}"] = V_cruise
configs["6 rotors"]["T/A"] = 8.15*ureg.lbf/ureg.ft**2
configs["6 rotors"]["N"] = 6


#Optimize 
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
		SizingMission.passengers.N_passengers: generic_data["sizing_mission"]["N_passengers"],#Number of passengers
	})

	RevenueMission = OnDemandRevenueMission(Aircraft,mission_type=generic_data["revenue_mission"]["type"])
	problem_subDict.update({
		RevenueMission.mission_range: generic_data["revenue_mission"]["range"],#mission range
		RevenueMission.V_cruise: c["V_{cruise}"],#cruising speed
		RevenueMission.t_hover: generic_data["revenue_mission"]["t_{hover}"],#hover time
		RevenueMission.passengers.N_passengers: generic_data["revenue_mission"]["N_passengers"],#Number of passengers
		RevenueMission.time_on_ground.charger_power: generic_data["charger_power"], #Charger power
	})

	DeadheadMission = OnDemandDeadheadMission(Aircraft,mission_type=generic_data["deadhead_mission"]["type"])
	problem_subDict.update({
		DeadheadMission.mission_range: generic_data["deadhead_mission"]["range"],#mission range
		DeadheadMission.V_cruise: c["V_{cruise}"],#cruising speed
		DeadheadMission.t_hover: generic_data["deadhead_mission"]["t_{hover}"],#hover time
		DeadheadMission.passengers.N_passengers: generic_data["deadhead_mission"]["N_passengers"],#Number of passengers
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

	#Noise computations
	T_perRotor = solution("T_perRotor_OnDemandSizingMission")[0]
	Q_perRotor = solution("Q_perRotor_OnDemandSizingMission")[0]
	R = solution("R")
	VT = solution("VT_OnDemandSizingMission")[0]
	s = solution("s")
	Cl_mean = solution("Cl_{mean_{max}}")
	N = solution("N")

	B = generic_data["B"]
	delta_S = generic_data["delta_S"]

	#Unweighted
	f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
		Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
		weighting="None")
	configs[config]["SPL"] = SPL
	configs[config]["f_{peak}"] = f_peak
	configs[config]["spectrum"] = spectrum

	#A-weighted
	f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
		Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
		weighting="A")
	configs[config]["SPL_A"] = SPL
	configs[config]["spectrum_A"] = spectrum


# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(12,12), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

y_pos = np.arange(len(configs))
labels = [""]*len(configs)
for i, config in enumerate(configs):
	labels[i] = config

long_width = 0.4
short_width = 0.2

offset_array = [-0.3,0,0.3]
colors = ["grey", "w", "k"]

#Maximum takeoff weight
plt.subplot(2,2,1)
for i, config in enumerate(configs):
	TOGW = configs[config]["solution"]("TOGW_OnDemandAircraft").to(ureg.lbf).magnitude
	plt.bar(i,TOGW,align='center',alpha=1,width=long_width,color='k',edgecolor='k')
plt.grid()
plt.xticks(y_pos, labels, rotation=-45,fontsize=12)
plt.yticks(fontsize=12)
plt.xlim(xmin = np.min(y_pos)-0.8,xmax = np.max(y_pos)+0.8)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.1*ymax)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Takeoff Gross Weight",fontsize = 16)

#Battery weight
plt.subplot(2,2,2)
for i, config in enumerate(configs):
	W_battery = configs[config]["solution"]("W_OnDemandAircraft/Battery").to(ureg.lbf).magnitude
	plt.bar(i,W_battery,align='center',alpha=1,width=long_width,color='k',edgecolor='k')
plt.grid()
plt.xticks(y_pos, labels, rotation=-45,fontsize=12)
plt.yticks(fontsize=12)
plt.xlim(xmin = np.min(y_pos)-0.8,xmax = np.max(y_pos)+0.8)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Battery Weight",fontsize = 16)

#Energy use by mission segment (sizing mission)
plt.subplot(2,2,3)
for i, config in enumerate(configs):
	sol = configs[config]["solution"]

	E_data = [dict() for x in range(3)]	
	
	E_data[0]["type"] = "Cruise"
	E_data[0]["value"] = sol("E_OnDemandSizingMission")[1]
	
	E_data[1]["type"] = "Hover"
	E_data[1]["value"] = sol("E_OnDemandSizingMission")[0] \
		+ sol("E_OnDemandSizingMission")[3]
	
	E_data[2]["type"] = "Reserve"
	E_data[2]["value"] = sol("E_OnDemandSizingMission")[2]
	
	bottom = 0
	for j,E in enumerate(E_data):
		E_value = E["value"].to(ureg.kWh).magnitude
		if (i == 0):
			plt.bar(i,E_value,align='center',bottom=bottom,alpha=1,width=long_width,
				color=colors[j],edgecolor='k',label=E["type"])
		else:
			plt.bar(i,E_value,align='center',bottom=bottom,alpha=1,width=long_width,
				color=colors[j],edgecolor='k')
		bottom = bottom + E_value

plt.grid()
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.xlim(xmin = np.min(y_pos)-0.8,xmax = np.max(y_pos)+0.8)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.2*ymax)
plt.ylabel('Energy (kWh)', fontsize = 16)
plt.title("Energy Use",fontsize = 18)
plt.legend(loc='upper right',framealpha=1, fontsize = 12)

#Power consumption by mission segment (sizing mission)
plt.subplot(2,2,4)
for i, config in enumerate(configs):
	sol = configs[config]["solution"]

	P_battery = np.zeros(3)	
	P_battery[0] = sol("P_{battery}_OnDemandSizingMission")[1].to(ureg.kW).magnitude#cruise
	P_battery[1] = sol("P_{battery}_OnDemandSizingMission")[0].to(ureg.kW).magnitude#hover
	P_battery[2] = sol("P_{battery}_OnDemandSizingMission")[2].to(ureg.kW).magnitude#reserve
	
	for j,offset in enumerate(offset_array):
		if (i == 0):
			if (j == 0):
				label = "Cruise"
			elif (j == 1):
				label = "Hover"
			elif (j == 2):
				label = "Reserve"

			plt.bar(i+offset,P_battery[j],align='center',alpha=1,width=short_width,
				color=colors[j],edgecolor='k',label=label)
		else:
			plt.bar(i+offset,P_battery[j],align='center',alpha=1,width=short_width,
				color=colors[j],edgecolor='k')

plt.grid()
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.xlim(xmin = np.min(y_pos)-0.8,xmax = np.max(y_pos)+0.8)
plt.ylabel('Power (kW)', fontsize = 16)
plt.title("Power Consumption",fontsize = 18)
plt.legend(loc='upper right',framealpha=1, fontsize = 12)

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
	+ "Sizing mission (%s): range = %0.0f nm; %0.0f passengers; %0.0fs hover time; reserve type = " \
	% (generic_data["sizing_mission"]["type"], generic_data["sizing_mission"]["range"].to(ureg.nautical_mile).magnitude,\
	 generic_data["sizing_mission"]["N_passengers"], generic_data["sizing_mission"]["t_{hover}"].to(ureg.s).magnitude)\
	+ reserve_type_string + "\n"\
	+ "Revenue mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (generic_data["revenue_mission"]["type"], generic_data["revenue_mission"]["range"].to(ureg.nautical_mile).magnitude, \
	 generic_data["revenue_mission"]["N_passengers"], generic_data["revenue_mission"]["t_{hover}"].to(ureg.s).magnitude,\
	 generic_data["charger_power"].to(ureg.kW).magnitude) \
	+ "Deadhead mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; deadhead ratio = %0.1f" \
	% (generic_data["deadhead_mission"]["type"], generic_data["deadhead_mission"]["range"].to(ureg.nautical_mile).magnitude, \
	 generic_data["deadhead_mission"]["N_passengers"], generic_data["deadhead_mission"]["t_{hover}"].to(ureg.s).magnitude,\
	 generic_data["deadhead_ratio"])


plt.suptitle(title_str,fontsize = 12.5)
plt.tight_layout()
plt.subplots_adjust(left=0.08,right=0.99,bottom=0.07,top=0.87)
plt.savefig('joby_config_plot_01.pdf')

#Additional parameters plot
fig2 = plt.figure(figsize=(12,12), dpi=80)
plt.show()

#Trip cost per passenger 
plt.subplot(2,2,1)
for i, config in enumerate(configs):
	cptpp = configs[config]["solution"]("cost_per_trip_per_passenger_OnDemandMissionCost")
	plt.bar(i,cptpp,align='center',alpha=1,width=long_width,color='k',edgecolor='k')
plt.grid()
plt.xticks(y_pos, labels, rotation=-45,fontsize=12)
plt.yticks(fontsize=12)
plt.xlim(xmin = np.min(y_pos)-0.8,xmax = np.max(y_pos)+0.8)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Cost per Trip, per Passenger",fontsize = 16)

#Cost per mission
plt.subplot(2,2,2)
for i, config in enumerate(configs):
	c_capital = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses")
	c_operating = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses")
	p1 = plt.bar(i,c_capital,bottom=0,align='center',alpha=1,width=long_width,color="k",edgecolor='k')
	p2 = plt.bar(i,c_operating,bottom=c_capital,align='center',alpha=1,width=long_width,color="lightgrey",edgecolor='k')

plt.grid()
plt.xticks(y_pos, labels, rotation=-45,fontsize=12)
plt.yticks(fontsize=12)
plt.xlim(xmin = np.min(y_pos)-0.8,xmax = np.max(y_pos)+0.8)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.1*ymax)
plt.ylabel('Cost per mission ($US)', fontsize = 16)
plt.title("Cost breakdown (revenue mission only)",fontsize = 16)
plt.legend((p1[0],p2[0]),("Capital expenses (amortized)","Operating expenses"),
	loc='upper right',framealpha=1, fontsize = 12)

#Vortex-noise peak frequency
plt.subplot(2,2,3)
for i, config in enumerate(configs):
	f_peak = configs[config]["f_{peak}"].to(ureg.turn/ureg.s).magnitude
	plt.bar(i,f_peak,bottom=0,align='center',alpha=1,width=long_width,color='k',edgecolor='k')
plt.grid()
#plt.yscale('log')
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.xlim(xmin = np.min(y_pos)-0.8,xmax = np.max(y_pos)+0.8)
plt.ylabel('Peak frequency (Hz)', fontsize = 16)
plt.title("Vortex-Noise Peak Frequency",fontsize = 18)


#Sound pressure level in hover
plt.subplot(2,2,4)
for i, config in enumerate(configs):

	SPL_sizing  = configs[config]["SPL"]
	SPL_sizing_A  = configs[config]["SPL_A"]

	if i == 0:
		plt.bar(i-0.2,SPL_sizing,width=0.3,align='center',alpha=1,color='grey',edgecolor='k',
			label="Unweighted")
		plt.bar(i+0.2,SPL_sizing_A,width=0.3,align='center',alpha=1,color='k',edgecolor='k',
			label="A-weighted")
	else:
		plt.bar(i-0.2,SPL_sizing,width=0.3,align='center',alpha=1,color='grey',edgecolor='k')
		plt.bar(i+0.2,SPL_sizing_A,width=0.3,align='center',alpha=1,color='k',edgecolor='k')

SPL_req = 62
plt.plot([np.min(y_pos)-1,np.max(y_pos)+1],[SPL_req, SPL_req],
	color="black", linewidth=3, linestyle="-")
#plt.ylim(ymin = 57, ymax = 76)
plt.ylim(ymin = 57)
plt.grid()
plt.xticks(y_pos, labels, rotation=-45,fontsize=12)
plt.yticks(fontsize=12)
plt.xlim(xmin = np.min(y_pos)-0.8,xmax = np.max(y_pos)+0.8)
plt.ylabel('SPL (dB)', fontsize = 16)
plt.title("Sound Pressure Level in Hover",fontsize = 16)
plt.legend(loc="upper right",framealpha=1,fontsize=12)


plt.suptitle(title_str,fontsize = 12.5)
plt.tight_layout()
plt.subplots_adjust(left=0.08,right=0.99,bottom=0.07,top=0.87)
plt.savefig('joby_config_plot_02.pdf')
