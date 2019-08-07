#Vehicle designs for different time frames

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
from collections import OrderedDict
from noise_models import vortex_noise


# Data specific to study
configs = OrderedDict()
time_frame_array = ["Initial","Near term","Long term"]

time_frame_data = {}
time_frame_data["Initial"] = {}
time_frame_data["Near term"] = {}
time_frame_data["Long term"] = {}

time_frame_data["Initial"]["autonomousEnabled"] = False
time_frame_data["Near term"]["autonomousEnabled"] = True
time_frame_data["Long term"]["autonomousEnabled"] = True

time_frame_data["Initial"]["C_m"] = 400*ureg.Wh/ureg.kg
time_frame_data["Near term"]["C_m"] = 450*ureg.Wh/ureg.kg
time_frame_data["Long term"]["C_m"] = 500*ureg.Wh/ureg.kg

time_frame_data["Initial"]["sizing_mission_type"] = "piloted"
time_frame_data["Near term"]["sizing_mission_type"] = "piloted"
time_frame_data["Long term"]["sizing_mission_type"] = "piloted"

time_frame_data["Initial"]["revenue_mission_type"] = "piloted"
time_frame_data["Near term"]["revenue_mission_type"] = "piloted"
time_frame_data["Long term"]["revenue_mission_type"] = "autonomous"

time_frame_data["Initial"]["deadhead_mission_type"] = "piloted"
time_frame_data["Near term"]["deadhead_mission_type"] = "autonomous"
time_frame_data["Long term"]["deadhead_mission_type"] = "autonomous"

time_frame_data["Initial"]["deadhead_ratio"] = 0.5
time_frame_data["Near term"]["deadhead_ratio"] = 0.35
time_frame_data["Long term"]["deadhead_ratio"] = 0.2

time_frame_data["Initial"]["vehicle_cost_per_weight"] = 600*ureg.lbf**-1
time_frame_data["Near term"]["vehicle_cost_per_weight"] = 400*ureg.lbf**-1
time_frame_data["Long term"]["vehicle_cost_per_weight"] = 200*ureg.lbf**-1

time_frame_data["Initial"]["battery_cost_per_C"] = 400*ureg.kWh**-1
time_frame_data["Near term"]["battery_cost_per_C"] = 200*ureg.kWh**-1
time_frame_data["Long term"]["battery_cost_per_C"] = 100*ureg.kWh**-1


for config in configuration_data:
	configs[config] = OrderedDict()
	for time_frame in time_frame_array:
		configs[config][time_frame] = configuration_data[config].copy()

#Delete unwanted configurations
del configs["Multirotor"]
del configs["Autogyro"]
del configs["Tilt duct"]
del configs["Helicopter"]
del configs["Coaxial heli"]

#Optimize remaining configurations
for config in configs:
	
	print "Solving configuration: " + config

	for time_frame in configs[config]:
		
		c = configs[config][time_frame]

		problem_subDict = {}

		Aircraft = OnDemandAircraft(autonomousEnabled=time_frame_data[time_frame]["autonomousEnabled"])
		problem_subDict.update({
			Aircraft.L_D_cruise: c["L/D"], #estimated L/D in cruise
			Aircraft.eta_cruise: generic_data["\eta_{cruise}"], #propulsive efficiency in cruise
			Aircraft.tailRotor_power_fraction_hover: c["tailRotor_power_fraction_hover"],
			Aircraft.tailRotor_power_fraction_levelFlight: c["tailRotor_power_fraction_levelFlight"],
			Aircraft.cost_per_weight: time_frame_data[time_frame]["vehicle_cost_per_weight"], #vehicle cost per unit empty weight
			Aircraft.battery.C_m: time_frame_data[time_frame]["C_m"], #battery energy density
			Aircraft.battery.cost_per_C: time_frame_data[time_frame]["battery_cost_per_C"], #battery cost per unit energy capacity
			Aircraft.rotors.N: c["N"], #number of propellers
			Aircraft.rotors.Cl_mean_max: c["Cl_{mean_{max}}"], #maximum allowed mean lift coefficient
			Aircraft.structure.weight_fraction: c["weight_fraction"], #empty weight fraction
			Aircraft.electricalSystem.eta: generic_data["\eta_{electric}"], #electrical system efficiency	
		})

		SizingMission = OnDemandSizingMission(Aircraft,mission_type=time_frame_data[time_frame]["sizing_mission_type"],
			reserve_type=generic_data["reserve_type"])
		problem_subDict.update({
			SizingMission.mission_range: generic_data["sizing_mission"]["range"],#mission range
			SizingMission.V_cruise: c["V_{cruise}"],#cruising speed
			SizingMission.t_hover: generic_data["sizing_mission"]["t_{hover}"],#hover time
			SizingMission.T_A: c["T/A"],#disk loading
			SizingMission.passengers.N_passengers: generic_data["sizing_mission"]["N_{passengers}"],#Number of passengers
		})

		RevenueMission = OnDemandRevenueMission(Aircraft,mission_type=time_frame_data[time_frame]["revenue_mission_type"])
		problem_subDict.update({
			RevenueMission.mission_range: generic_data["revenue_mission"]["range"],#mission range
			RevenueMission.V_cruise: c["V_{cruise}"],#cruising speed
			RevenueMission.t_hover: generic_data["revenue_mission"]["t_{hover}"],#hover time
			RevenueMission.passengers.N_passengers: generic_data["revenue_mission"]["N_{passengers}"],#Number of passengers
			RevenueMission.time_on_ground.charger_power: generic_data["charger_power"], #Charger power
		})

		DeadheadMission = OnDemandDeadheadMission(Aircraft,mission_type=time_frame_data[time_frame]["deadhead_mission_type"])
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
			MissionCost.deadhead_ratio: time_frame_data[time_frame]["deadhead_ratio"], #deadhead ratio
		})

		problem = Model(MissionCost["cost_per_trip"],
			[Aircraft, SizingMission, RevenueMission, DeadheadMission, MissionCost])
		problem.substitutions.update(problem_subDict)
		solution = problem.solve(verbosity=0)

		configs[config][time_frame]["solution"] = solution

		configs[config][time_frame]["TOGW"] = solution("TOGW_OnDemandAircraft")
		configs[config][time_frame]["TOGM"] = solution("TOGM_OnDemandAircraft")
		configs[config][time_frame]["W_{battery}"] = solution("W_OnDemandAircraft/Battery")
		configs[config][time_frame]["cost_per_trip_per_passenger"] = solution("cost_per_trip_per_passenger_OnDemandMissionCost")
		configs[config][time_frame]["cost_per_seat_mile"] = solution("cost_per_seat_mile_OnDemandMissionCost")
		configs[config][time_frame]["SPL"] = 20*np.log10(solution("p_{ratio}_OnDemandSizingMission")[0])

		c_vehicle = solution("purchase_price_OnDemandAircraft")
		c_avionics = solution("purchase_price_OnDemandAircraft/Avionics")
		c_battery = solution("purchase_price_OnDemandAircraft/Battery")
		configs[config][time_frame]["purchase_price"] = c_vehicle + c_avionics + c_battery

		NdNr = solution("N_{deadhead}/N_{typical}_OnDemandMissionCost")
		configs[config][time_frame]["NdNr"] = NdNr

		amortized_pilot_cost_revenue = solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses/PilotCost")
		amortized_pilot_cost_deadhead = solution("cost_per_mission_OnDemandMissionCost/DeadheadMissionCost/OperatingExpenses/PilotCost")
		configs[config][time_frame]["amortized_capex_revenue"] = amortized_pilot_cost_revenue
		configs[config][time_frame]["amortized_capex_deadhead"] = amortized_pilot_cost_deadhead
		configs[config][time_frame]["amortized_pilot_cost"] = amortized_pilot_cost_revenue + amortized_pilot_cost_deadhead

		amortized_capex_revenue = solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses")
		amortized_capex_deadhead = NdNr*solution("cost_per_mission_OnDemandMissionCost/DeadheadMissionCost/CapitalExpenses")
		configs[config][time_frame]["amortized_capex_revenue"] = amortized_capex_revenue
		configs[config][time_frame]["amortized_capex_deadhead"] = amortized_capex_deadhead
		configs[config][time_frame]["amortized_capex"] = amortized_capex_revenue + amortized_capex_deadhead

		amortized_opex_revenue = solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses")
		amortized_opex_deadhead = NdNr*solution("cost_per_mission_OnDemandMissionCost/DeadheadMissionCost/OperatingExpenses")
		configs[config][time_frame]["amortized_opex_revenue"] = amortized_opex_revenue
		configs[config][time_frame]["amortized_opex_deadhead"] = amortized_opex_deadhead
		configs[config][time_frame]["amortized_opex"] = amortized_opex_revenue + amortized_opex_deadhead

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
		configs[config][time_frame]["SPL"] = SPL
		configs[config][time_frame]["f_{peak}"] = f_peak
		configs[config][time_frame]["spectrum"] = spectrum

		#A-weighted
		f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
			weighting="A")
		configs[config][time_frame]["SPL_A"] = SPL
		configs[config][time_frame]["spectrum_A"] = spectrum


# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(12, 7), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

y_pos = np.arange(len(configs))
labels = [""]*len(configs)
for i, config in enumerate(configs):
	if config == "Compound heli":
		labels[i] = config.replace(" ", "\n")  # Replace spaces with newlines
	else:
		labels[i] = config

style = {}
style["rotation"]         = -45
style["legend_ncols"]     = 2
style["bar_width_wide"]   = 0.7
style["bar_width_medium"] = 0.3
style["bar_width_narrow"] = 0.2
style["offsets"]          = [-0.25, 0, 0.25]
style["colors"]           = ["grey", "w", "k", "lightgrey"]

style["fontsize"] = {}
style["fontsize"]["xticks"]     = 14
style["fontsize"]["yticks"]     = 14
style["fontsize"]["xlabel"]     = 18
style["fontsize"]["ylabel"]     = 16
style["fontsize"]["title"]      = 16
style["fontsize"]["legend"]     = 11
style["fontsize"]["text_label"] = 18

#Maximum takeoff mass
plt.subplot(2,3,1)
for i, config in enumerate(configs):
	for j, time_frame in enumerate(configs[config]):
		
		c      = configs[config][time_frame]
		offset = style["offsets"][j]
		TOGM   = c["TOGM"].to(ureg.kg).magnitude

		if i==0:
			plt.bar(i+offset, TOGM, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k', label=time_frame)
		else:
			plt.bar(i+offset, TOGM, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

plt.grid()
[xmin,xmax] = plt.gca().get_xlim()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.0*ymax)
plt.xticks(y_pos, labels, rotation=style["rotation"], fontsize=style["fontsize"]["xticks"])
plt.yticks(fontsize=style["fontsize"]["yticks"])
plt.ylabel('Mass (kg)', fontsize=style["fontsize"]["ylabel"])
plt.title("Maximum Takeoff Mass", fontsize=style["fontsize"]["title"])
plt.legend(loc='lower right', framealpha=1, fontsize=style["fontsize"]["legend"])


# Cost per seat km
plt.subplot(2,3,2)
for i, config in enumerate(configs):
	for j, time_frame in enumerate(configs[config]):
		
		c      = configs[config][time_frame]
		offset = style["offsets"][j]
		cpsk   = c["cost_per_seat_mile"].to(ureg.mile**-1).magnitude

		if i==0:
			plt.bar(i+offset, cpsk, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k', label=time_frame)
		else:
			plt.bar(i+offset, cpsk, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

plt.grid()
[xmin,xmax] = plt.gca().get_xlim()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.3*ymax)
plt.xticks(y_pos, labels, rotation=style["rotation"], fontsize=style["fontsize"]["xticks"])
plt.yticks(fontsize=style["fontsize"]["yticks"])
plt.ylabel('Cost ($US/km)', fontsize=style["fontsize"]["ylabel"])
plt.title("Cost per Seat Kilometer", fontsize=style["fontsize"]["title"])
plt.legend(loc='upper right', framealpha=1, fontsize=style["fontsize"]["legend"])



plt.tight_layout()
plt.subplots_adjust(left=0.08,right=0.99,bottom=0.10,top=0.88)
plt.savefig('time_frame_plot_01.pdf')

"""
#Battery weight
plt.subplot(2,2,2)
for i,config in enumerate(configs):
	for j,time_frame in enumerate(configs[config]):
		c = configs[config][time_frame]
		offset = offset_array[j]
		W_battery = c["W_{battery}"].to(ureg.lbf).magnitude

		if (i == 0):
			label = time_frame
			plt.bar(i+offset,W_battery,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,W_battery,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Battery Weight",fontsize = 18)
plt.legend(loc='upper right',framealpha=1, fontsize = 12)


#Trip cost per passenger 
plt.subplot(2,2,3)
for i,config in enumerate(configs):
	for j,time_frame in enumerate(configs[config]):
		c = configs[config][time_frame]
		offset = offset_array[j]
		cptpp = c["cost_per_trip_per_passenger"]

		if (i == 0):
			label = time_frame
			plt.bar(i+offset,cptpp,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,cptpp,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Cost per Trip, per Passenger",fontsize = 18)
plt.legend(loc='upper right',framealpha=1, fontsize = 12)


#Sound pressure level (in hover) 
plt.subplot(2,2,4)
for i,config in enumerate(configs):
	for j,time_frame in enumerate(configs[config]):
		c = configs[config][time_frame]
		offset = offset_array[j]
		SPL_sizing_A = c["SPL_A"]

		if (i == 0):
			label = time_frame
			plt.bar(i+offset,SPL_sizing_A,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,SPL_sizing_A,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

SPL_req = 62
plt.plot([np.min(y_pos)-1,np.max(y_pos)+1],[SPL_req, SPL_req],
	color="black", linewidth=3, linestyle="-")
plt.ylim(ymin = 57,ymax = 85)
plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('SPL (dBA)', fontsize = 16)
plt.title("Sound Pressure Level in Hover",fontsize = 18)
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


title_str = "Aircraft parameters: %0.0f rotor blades\n" % B \
	+ "Sizing mission: range = %0.0f nmi; %0.0f passengers; %0.0fs hover time; reserve type = " \
	% (generic_data["sizing_mission"]["range"].to(ureg.nautical_mile).magnitude, generic_data["sizing_mission"]["N_{passengers}"],\
		generic_data["sizing_mission"]["t_{hover}"].to(ureg.s).magnitude) + reserve_type_string + "\n" \
	+ "Revenue mission: range = %0.0f nmi; %0.1f passengers; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (generic_data["revenue_mission"]["range"].to(ureg.nautical_mile).magnitude, \
		generic_data["revenue_mission"]["N_{passengers}"], generic_data["revenue_mission"]["t_{hover}"].to(ureg.s).magnitude, generic_data["charger_power"].to(ureg.kW).magnitude) \
	+ "Deadhead mission: range = %0.0f nmi; %0.1f passengers; %0.0fs hover time; no reserve" \
	% (generic_data["deadhead_mission"]["range"].to(ureg.nautical_mile).magnitude, \
		generic_data["deadhead_mission"]["N_{passengers}"], generic_data["deadhead_mission"]["t_{hover}"].to(ureg.s).magnitude)

plt.suptitle(title_str,fontsize = 14)
plt.tight_layout()
plt.subplots_adjust(left=0.08,right=0.99,bottom=0.10,top=0.88)
plt.savefig('time_frame_plot_01.pdf')


#Cost breakdown plot
fig2 = plt.figure(figsize=(12,12), dpi=80)
plt.show()

#Vehicle acquisition cost
plt.subplot(2,2,1)
for i,config in enumerate(configs):
	for j,time_frame in enumerate(configs[config]):
		c = configs[config][time_frame]
		offset = offset_array[j]
		purchase_price = c["purchase_price"]/1e6
		if (i == 0):
			label = time_frame
			plt.bar(i+offset,purchase_price,align='center',alpha=1,width=width,
				color=colors[j],edgecolor='k',label=label)
		else:
			plt.bar(i+offset,purchase_price,align='center',alpha=1,width=width,
				color=colors[j],edgecolor='k')

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.1*ymax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Cost ($millions US)', fontsize = 16)
plt.title("Acquisition Cost",fontsize = 18)
plt.legend(loc='upper right',framealpha=1, fontsize = 12)

#Cost per seat mile
plt.subplot(2,2,2)
for i,config in enumerate(configs):
	for j,time_frame in enumerate(configs[config]):
		c = configs[config][time_frame]
		offset = offset_array[j]
		cpsm = c["cost_per_seat_mile"].to(ureg.mile**-1).magnitude
		if (i == 0):
			label = time_frame
			plt.bar(i+offset,cpsm,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k',label=label)
		else:
			plt.bar(i+offset,cpsm,align='center',alpha=1,width=width,color=colors[j],
				edgecolor='k')

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.1*ymax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Cost ($US/mile)', fontsize = 16)
plt.title("Cost per Seat Mile",fontsize = 18)
plt.legend(loc='upper right',framealpha=1, fontsize = 12)

#Amortized capital expenses per mission
plt.subplot(2,2,3)
for i,config in enumerate(configs):
	for j,time_frame in enumerate(configs[config]):
		c = configs[config][time_frame]
		offset = offset_array[j]
		amortized_capex = c["amortized_capex"]
		if (i == 0):
			label = time_frame
			plt.bar(i+offset,amortized_capex,align='center',alpha=1,width=width,
				color=colors[j],edgecolor='k',label=label)
		else:
			plt.bar(i+offset,amortized_capex,align='center',alpha=1,width=width,
				color=colors[j],edgecolor='k')

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Capital Expenses per Trip",fontsize = 18)
plt.legend(loc='upper right',framealpha=1, fontsize = 12)

#Amortized operating expenses per mission
plt.subplot(2,2,4)
for i,config in enumerate(configs):
	for j,time_frame in enumerate(configs[config]):
		c = configs[config][time_frame]
		offset = offset_array[j]
		amortized_opex = c["amortized_opex"]

		if (i == 0):
			label = time_frame
			plt.bar(i+offset,amortized_opex,align='center',alpha=1,width=width,
				color=colors[j],edgecolor='k',label=label)
		else:
			plt.bar(i+offset,amortized_opex,align='center',alpha=1,width=width,
				color=colors[j],edgecolor='k')

plt.grid()
plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.1*ymax)
plt.xticks(y_pos, labels, rotation=-45, fontsize=12)
plt.yticks(fontsize=12)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Operating Expenses per Trip",fontsize = 18)
plt.legend(loc='upper right',framealpha=1, fontsize = 12)

plt.suptitle(title_str,fontsize = 14)
plt.tight_layout()
plt.subplots_adjust(left=0.07,right=0.99,bottom=0.10,top=0.88)
plt.savefig('time_frame_plot_02_costBreakdown.pdf')
"""