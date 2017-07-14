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

#General data
eta_cruise = generic_data["\eta_{cruise}"] 
eta_electric = generic_data["\eta_{electric}"]
weight_fraction = generic_data["weight_fraction"]
C_m = generic_data["C_m"]
n = generic_data["n"]

reserve_type = generic_data["reserve_type"]
autonomousEnabled = generic_data["autonomousEnabled"]
charger_power = generic_data["charger_power"]

vehicle_cost_per_weight = generic_data["vehicle_cost_per_weight"]
battery_cost_per_C = generic_data["battery_cost_per_C"]
pilot_wrap_rate = generic_data["pilot_wrap_rate"]
mechanic_wrap_rate = generic_data["mechanic_wrap_rate"]
MMH_FH = generic_data["MMH_FH"]
#deadhead_ratio = generic_data["deadhead_ratio"]

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

	V_cruise = c["V_{cruise}"]
	L_D_cruise = c["L/D"]
	T_A = c["T/A"]
	Cl_mean_max = c["Cl_{mean_{max}}"]
	N = c["N"]
	loiter_type = c["loiter_type"]

	configs[config]["MTOW"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["W_{battery}"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["cost_per_trip_per_passenger"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["SPL"] = np.zeros(np.size(deadhead_ratio_array))
	
	configs[config]["purchase_price"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["NdNr"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["amortized_capex_revenue"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["amortized_capex_deadhead"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["amortized_capex"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["amortized_opex_revenue"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["amortized_opex_deadhead"] = np.zeros(np.size(deadhead_ratio_array))
	configs[config]["amortized_opex"] = np.zeros(np.size(deadhead_ratio_array))

	for i,deadhead_ratio in enumerate(deadhead_ratio_array):

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
			pilot_wrap_rate=pilot_wrap_rate,mechanic_wrap_rate=mechanic_wrap_rate,MMH_FH=MMH_FH,\
			deadhead_ratio=deadhead_ratio)

		problem = Model(MissionCost["cost_per_trip"],
			[Aircraft, SizingMission, RevenueMission, DeadheadMission, MissionCost])

		solution = problem.solve(verbosity=0)

		configs[config]["MTOW"][i] = solution("MTOW_OnDemandAircraft").to(ureg.lbf).magnitude
		configs[config]["W_{battery}"][i] = solution("W_OnDemandAircraft/Battery").to(ureg.lbf).magnitude
		configs[config]["cost_per_trip_per_passenger"][i] = solution("cost_per_trip_per_passenger_OnDemandMissionCost")
		configs[config]["SPL"][i] = 20*np.log10(solution("p_{ratio}_OnDemandSizingMission")[0])

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

#Vehicle acquisition cost
plt.subplot(2,2,1)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(deadhead_ratio_array,c["purchase_price"]/1e6,
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.ylim(ymin=0,ymax=1.5)
plt.xlabel('Deadhead ratio', fontsize = 16)
plt.ylabel('Cost ($millions US)', fontsize = 16)
plt.title("Acquisition Cost",fontsize = 20)
plt.legend(numpoints = 1,loc='lower right', fontsize = 12)

#Trip cost per passenger
plt.subplot(2,2,2)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(deadhead_ratio_array,c["cost_per_trip_per_passenger"],
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.ylim(ymin=0)
plt.xlabel('Deadhead ratio', fontsize = 16)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Cost per Trip, per Passenger",fontsize = 20)
plt.legend(numpoints = 1,loc='lower right', fontsize = 12)


#Amortized capital expenses per mission
plt.subplot(2,2,3)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(deadhead_ratio_array,c["amortized_capex"],
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.ylim(ymin=0)
plt.xlabel('Deadhead ratio', fontsize = 16)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Capital Expenses per Trip",fontsize = 20)
plt.legend(numpoints = 1,loc='lower right', fontsize = 12)

#Amortized operating expenses per mission
plt.subplot(2,2,4)
for i, config in enumerate(configs):
	c = configs[config]
	plt.plot(deadhead_ratio_array,c["amortized_opex"],
		color="black",linewidth=1.5,linestyle=style["linestyle"][i],marker=style["marker"][i],
		fillstyle=style["fillstyle"][i],markersize=style["markersize"],label=config)
plt.grid()
plt.ylim(ymin=0)
plt.xlabel('Deadhead ratio', fontsize = 16)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Operating Expenses per Trip",fontsize = 20)
plt.legend(numpoints = 1,loc='lower right', fontsize = 12)


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
	+ "Deadhead mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve" \
	% (deadhead_mission_type, deadhead_mission_range.to(ureg.nautical_mile).magnitude, \
		deadhead_N_passengers, deadhead_t_hover.to(ureg.s).magnitude)


plt.suptitle(title_str,fontsize = 13.5)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.98,bottom=0.05,top=0.87)