# Sweep to evaluate the sensitivity of mission costs to deadhead ratio

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

# Delete certain configurations
configs = configuration_data.copy()
del configs["Tilt duct"]
del configs["Multirotor"]
del configs["Autogyro"]
del configs["Helicopter"]
del configs["Coaxial heli"]

#Data specific to study
deadhead_ratio_array = np.linspace(0.1,0.5,9)

#Optimize remaining configurations
for config in configs:

	print "Solving configuration: " + config

	c = configs[config]

	configs[config]["TOGW"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["W_{battery}"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["cost_per_trip_per_passenger"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["SPL_A"] = np.zeros(np.size(deadhead_ratio_array))
	
	configs[config]["purchase_price"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["NdNr"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["amortized_capex_revenue"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["amortized_capex_deadhead"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["amortized_capex"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["amortized_opex_revenue"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["amortized_opex_deadhead"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["amortized_opex"] = np.zeros(np.size(deadhead_ratio_array))

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

	for i,deadhead_ratio in enumerate(deadhead_ratio_array):

		MissionCost = OnDemandMissionCost(Aircraft,RevenueMission,DeadheadMission)
		problem_subDict.update({
			MissionCost.revenue_mission_costs.operating_expenses.pilot_cost.wrap_rate: generic_data["pilot_wrap_rate"],#pilot wrap rate
			MissionCost.revenue_mission_costs.operating_expenses.maintenance_cost.wrap_rate: generic_data["mechanic_wrap_rate"], #mechanic wrap rate
			MissionCost.revenue_mission_costs.operating_expenses.maintenance_cost.MMH_FH: generic_data["MMH_FH"], #maintenance man-hours per flight hour
			MissionCost.deadhead_mission_costs.operating_expenses.pilot_cost.wrap_rate: generic_data["pilot_wrap_rate"],#pilot wrap rate
			MissionCost.deadhead_mission_costs.operating_expenses.maintenance_cost.wrap_rate: generic_data["mechanic_wrap_rate"], #mechanic wrap rate
			MissionCost.deadhead_mission_costs.operating_expenses.maintenance_cost.MMH_FH: generic_data["MMH_FH"], #maintenance man-hours per flight hour
			MissionCost.deadhead_ratio: deadhead_ratio, #deadhead ratio
		})

		problem = Model(MissionCost["cost_per_trip"],
			[Aircraft, SizingMission, RevenueMission, DeadheadMission, MissionCost])
		problem.substitutions.update(problem_subDict)
		solution = problem.solve(verbosity=0)
		configs[config]["solution"] = solution

		configs[config]["TOGW"][i] = solution("TOGW_OnDemandAircraft").to(ureg.lbf).magnitude
		configs[config]["W_{battery}"][i] = solution("W_OnDemandAircraft/Battery").to(ureg.lbf).magnitude
		configs[config]["cost_per_trip_per_passenger"][i] = solution("cost_per_trip_per_passenger_OnDemandMissionCost")
		
		c_vehicle = solution("purchase_price_OnDemandAircraft")
		c_avionics = solution("purchase_price_OnDemandAircraft/Avionics")
		c_battery = solution("purchase_price_OnDemandAircraft/Battery")
		configs[config]["purchase_price"][i] = c_vehicle + c_avionics + c_battery

		NdNr = solution("N_{deadhead}/N_{typical}_OnDemandMissionCost")
		configs[config]["NdNr"][i] = NdNr
		
		amortized_capex_revenue = solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses")
		amortized_capex_deadhead = NdNr*solution("cost_per_mission_OnDemandMissionCost/DeadheadMissionCost/CapitalExpenses")
		configs[config]["amortized_capex_revenue"][i] = amortized_capex_revenue 
		configs[config]["amortized_capex_deadhead"][i] = amortized_capex_deadhead
		configs[config]["amortized_capex"][i] = amortized_capex_revenue + amortized_capex_deadhead

		amortized_opex_revenue = solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses")
		amortized_opex_deadhead = NdNr*solution("cost_per_mission_OnDemandMissionCost/DeadheadMissionCost/OperatingExpenses")
		configs[config]["amortized_opex_revenue"][i] = amortized_opex_revenue 
		configs[config]["amortized_opex_deadhead"][i] = amortized_opex_deadhead
		configs[config]["amortized_opex"][i] = amortized_opex_revenue + amortized_opex_deadhead

	configs[config]["TOGW"] = configs[config]["TOGW"]*ureg.lbf
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

#Vehicle acquisition cost
plt.subplot(2,2,1)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(deadhead_ratio_array,c["purchase_price"]/1e6,
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymin=0,ymax = 1.2*ymax)
plt.xticks(fontsize=12)
plt.yticks(fontsize=12)
plt.xlabel('Deadhead ratio', fontsize = 16)
plt.ylabel('Cost ($millions US)', fontsize = 16)
plt.title("Acquisition Cost",fontsize = 20)
plt.legend(numpoints = 1,loc='lower right', fontsize = 12, framealpha=1)

#Trip cost per passenger
plt.subplot(2,2,2)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(deadhead_ratio_array,c["cost_per_trip_per_passenger"],
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.ylim(ymin=0)
plt.xticks(fontsize=12)
plt.yticks(fontsize=12)
plt.xlabel('Deadhead ratio', fontsize = 16)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Cost per Trip, per Passenger",fontsize = 20)
plt.legend(numpoints = 1,loc='lower right', fontsize = 12, framealpha=1)


#Amortized capital expenses per mission
plt.subplot(2,2,3)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(deadhead_ratio_array,c["amortized_capex"],
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.ylim(ymin=0)
plt.xticks(fontsize=12)
plt.yticks(fontsize=12)
plt.xlabel('Deadhead ratio', fontsize = 16)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Capital Expenses per Trip",fontsize = 20)
plt.legend(numpoints = 1,loc='lower right', fontsize = 12, framealpha=1)

#Amortized operating expenses per mission
plt.subplot(2,2,4)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(deadhead_ratio_array,c["amortized_opex"],
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.ylim(ymin=0)
plt.xticks(fontsize=12)
plt.yticks(fontsize=12)
plt.xlabel('Deadhead ratio', fontsize = 16)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Operating Expenses per Trip",fontsize = 20)
plt.legend(numpoints = 1,loc='lower right', fontsize = 12, framealpha=1)

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
	+ "Deadhead mission (%s): range = %0.0f nmi; %0.1f passengers; %0.0fs hover time; no reserve" \
	% (generic_data["deadhead_mission"]["type"], generic_data["deadhead_mission"]["range"].to(ureg.nautical_mile).magnitude, \
	 generic_data["deadhead_mission"]["N_{passengers}"], generic_data["deadhead_mission"]["t_{hover}"].to(ureg.s).magnitude)

plt.suptitle(title_str,fontsize = 13.0)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.98,bottom=0.05,top=0.87)
plt.savefig('deadhead_ratio_plot_01.pdf')
