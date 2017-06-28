#Vehicle configuration top-level trade study

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/' + '..'))

import numpy as np
from gpkit import Model, ureg
from matplotlib import pyplot as plt
from aircraft_models import OnDemandAircraft 
from aircraft_models import OnDemandSizingMission, OnDemandRevenueMission
from aircraft_models import OnDemandDeadheadMission, OnDemandMissionCost
from configuration_data import configurations

#General data
N = 12 #number of propellers. Required, but has no effect since T/A is constrained
eta_cruise = 0.85 #propulsive efficiency in cruise
eta_electric = 0.9 #electrical system efficiency
weight_fraction = 0.5 #structural mass fraction
C_m = 400*ureg.Wh/ureg.kg #battery energy density
n=1.0#battery discharge parameter
reserve_type = "Uber"

sizing_mission_range = 50*ureg.nautical_mile
revenue_mission_range = 30*ureg.nautical_mile
deadhead_mission_range = 30*ureg.nautical_mile

sizing_time_in_hover = 120*ureg.s
revenue_time_in_hover = 30*ureg.s
deadhead_time_in_hover = 30*ureg.s

autonomousEnabled = "Yes"
sizing_mission_type = "piloted"
revenue_mission_type = "piloted"
deadhead_mission_type = "autonomous"

sizing_N_passengers = 3
revenue_N_passengers = 2
deadhead_N_passengers = 0.00001

charger_power=200*ureg.kW

vehicle_cost_per_weight=350*ureg.lbf**-1
battery_cost_per_C = 400*ureg.kWh**-1
pilot_wrap_rate = 70*ureg.hr**-1
mechanic_wrap_rate = 60*ureg.hr**-1
MMH_FH = 0.6
deadhead_ratio = 0.2


# Delete configurations that won't solve
configs = configurations.copy()
del configs["Tilt duct"]
del configs["Multirotor"]

#Optimize remaining configurations
for config in configs:
	
	print "Solving configuration: " + config

	c = configs[config]

	V_cruise = c["V_{cruise}"]
	V_loiter = V_cruise #approximation
	L_D = c["L/D"]
	T_A = c["T/A"]
	Cl_mean_max = c["Cl_{mean_{max}}"]

	Aircraft = OnDemandAircraft(N=N,L_D=L_D,eta_cruise=eta_cruise,C_m=C_m,
		Cl_mean_max=Cl_mean_max,weight_fraction=weight_fraction,n=n,eta_electric=eta_electric,
		cost_per_weight=vehicle_cost_per_weight,cost_per_C=battery_cost_per_C,
		autonomousEnabled=autonomousEnabled)

	SizingMission = OnDemandSizingMission(Aircraft,mission_range=sizing_mission_range,
		V_cruise=V_cruise,V_loiter=V_loiter,N_passengers=sizing_N_passengers,
		time_in_hover=sizing_time_in_hover,reserve_type=reserve_type,
		mission_type=sizing_mission_type)
	SizingMission.substitutions.update({SizingMission.fs0.topvar("T/A"):T_A})
	
	RevenueMission = OnDemandRevenueMission(Aircraft,mission_range=revenue_mission_range,
		V_cruise=V_cruise,N_passengers=revenue_N_passengers,time_in_hover=revenue_time_in_hover,
		charger_power=charger_power,mission_type=revenue_mission_type)

	DeadheadMission = OnDemandDeadheadMission(Aircraft,mission_range=deadhead_mission_range,
		V_cruise=V_cruise,N_passengers=deadhead_N_passengers,time_in_hover=deadhead_time_in_hover,
		charger_power=charger_power,mission_type=deadhead_mission_type)

	MissionCost = OnDemandMissionCost(Aircraft,RevenueMission,DeadheadMission,
		pilot_wrap_rate=pilot_wrap_rate,mechanic_wrap_rate=mechanic_wrap_rate,MMH_FH=MMH_FH,
		deadhead_ratio=deadhead_ratio)
	
	problem = Model(MissionCost["cost_per_trip"],
		[Aircraft, SizingMission, RevenueMission, DeadheadMission, MissionCost])
	
	solution = problem.solve(verbosity=0)
	configs[config]["solution"] = solution


# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(17,11), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

y_pos = np.arange(len(configs))
labels = [""]*len(configs)
for i, config in enumerate(configs):
	labels[i] = config

#Maximum takeoff weight
plt.subplot(2,2,1)
for i, config in enumerate(configs):
	MTOW = configs[config]["solution"]["variables"]["MTOW_OnDemandAircraft"].to(ureg.lbf).magnitude
	plt.bar(i,MTOW,align='center',alpha=1)
plt.grid()
plt.xticks(y_pos, labels, rotation=-60)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Maximum Takeoff Weight",fontsize = 20)

#Battery weight
plt.subplot(2,2,2)
for i, config in enumerate(configs):
	W_battery = configs[config]["solution"]["variables"]["W_OnDemandAircraft/Battery"].to(ureg.lbf).magnitude
	plt.bar(i,W_battery,align='center',alpha=1)
plt.grid()
plt.xticks(y_pos, labels, rotation=-60)
plt.ylabel('Weight (lbf)', fontsize = 16)
plt.title("Battery Weight",fontsize = 20)

#Trip cost per passenger 
plt.subplot(2,2,3)
for i, config in enumerate(configs):
	cptpp = configs[config]["solution"]["variables"]["cost_per_trip_per_passenger_OnDemandMissionCost"]
	plt.bar(i,cptpp,align='center',alpha=1)
plt.grid()
plt.xticks(y_pos, labels, rotation=-60)
plt.ylabel('Cost ($US)', fontsize = 16)
plt.title("Cost per Trip, per Passenger",fontsize = 20)

#Sound pressure level (in hover) 
plt.subplot(2,2,4)
for i, config in enumerate(configs):
	SPL_sizing  = np.array(20*np.log10(configs[config]["solution"]["variables"]["p_{ratio}_OnDemandSizingMission"]))
	plt.bar(i,SPL_sizing,align='center',alpha=1)

SPL_req = 62
plt.plot([np.min(y_pos)-1,np.max(y_pos)+1],[SPL_req, SPL_req],
	color="black", linewidth=3, linestyle="-")
plt.ylim(ymin = 57, ymax = 75)
plt.grid()
plt.xticks(y_pos, labels, rotation=-60)
plt.ylabel('SPL (dB)', fontsize = 16)
plt.title("Sound Pressure Level in Hover",fontsize = 20)


if reserve_type == "FAA":
	num = solution["constants"]["t_{loiter}_OnDemandSizingMission"].to(ureg.minute).magnitude
	reserve_type_string = " (%0.0f-minute loiter time)" % num
if reserve_type == "Uber":
	num = solution["constants"]["R_{divert}_OnDemandSizingMission"].to(ureg.nautical_mile).magnitude
	reserve_type_string = " (%0.0f-nm diversion distance)" % num

if autonomousEnabled == "Yes":
	autonomy_string = "autonomy enabled"
if autonomousEnabled == "No":
	autonomy_string = "pilot required"

title_str = "Aircraft parameters: structural mass fraction = %0.2f; battery energy density = %0.0f Wh/kg; %s\n" \
	% (weight_fraction, C_m.to(ureg.Wh/ureg.kg).magnitude, autonomy_string) \
	+ "Sizing mission (%s): range = %0.0f nm; %0.0f passengers; %0.0fs hover time; reserve type = " \
	% (sizing_mission_type, sizing_mission_range.to(ureg.nautical_mile).magnitude, sizing_N_passengers, sizing_time_in_hover.to(ureg.s).magnitude) \
	+ reserve_type + reserve_type_string + "\n"\
	+ "Revenue mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; charger power = %0.0f kW\n" \
	% (revenue_mission_type, revenue_mission_range.to(ureg.nautical_mile).magnitude, \
		revenue_N_passengers, revenue_time_in_hover.to(ureg.s).magnitude, charger_power.to(ureg.kW).magnitude) \
	+ "Deadhead mission (%s): range = %0.0f nm; %0.1f passengers; %0.0fs hover time; no reserve; deadhead ratio = %0.1f" \
	% (deadhead_mission_type, deadhead_mission_range.to(ureg.nautical_mile).magnitude, \
		deadhead_N_passengers, deadhead_time_in_hover.to(ureg.s).magnitude, deadhead_ratio)

plt.suptitle(title_str,fontsize = 16)

plt.tight_layout()#makes sure subplots are spaced neatly
plt.subplots_adjust(left=0.05,right=0.95,bottom=0.125,top=0.84)#adds space at the top for the title


#Cost breakdown plot

fig2 = plt.figure(figsize=(17,11), dpi=80)
plt.show()

#Revenue and deadhead costs

plt.subplot(2,2,1)
for i, config in enumerate(configs):
	
	c_revenue = configs[config]["solution"]("revenue_cost_per_trip_OnDemandMissionCost")
	c_deadhead = configs[config]["solution"]("deadhead_cost_per_trip_OnDemandMissionCost")
	
	p1 = plt.bar(i,c_revenue,bottom=0,align='center',alpha=1,color="b",hatch="/")
	p2 = plt.bar(i,c_deadhead,bottom=c_revenue,align='center',alpha=1,color="r",hatch="\\")

plt.xticks(y_pos, labels, rotation=-60,fontsize=14)
plt.ylabel('Cost per trip ($US)', fontsize = 16)
plt.grid()
plt.title("Revenue-generating and Deadhead Costs",fontsize = 16)
plt.legend((p1[0],p2[0]),("Revenue-generating cost","Deadhead cost"),
	loc='upper right', fontsize = 14)

plt.subplot(2,2,2)
for i, config in enumerate(configs):
	
	c_capital = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses")
	c_operating = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses")
	
	p1 = plt.bar(i,c_capital,bottom=0,align='center',alpha=1,color="b",hatch="/")
	p2 = plt.bar(i,c_operating,bottom=c_capital,align='center',alpha=1,color="r",hatch="\\")

plt.xticks(y_pos, labels, rotation=-60,fontsize=14)
plt.ylabel('Cost per mission ($US)', fontsize = 16)
plt.grid()
plt.title("Cost breakdown (revenue mission only)",fontsize = 16)
plt.legend((p1[0],p2[0]),("Capital expenses (amortized)","Operating expenses"),
	loc='upper right', fontsize = 14)

plt.subplot(2,2,3)
for i, config in enumerate(configs):
	
	c_vehicle = configs[config]["solution"]("purchase_price_OnDemandAircraft")/1000
	c_avionics = configs[config]["solution"]("purchase_price_OnDemandAircraft/Avionics")/1000
	c_battery = configs[config]["solution"]("purchase_price_OnDemandAircraft/Battery")/1000
	
	p1 = plt.bar(i,c_vehicle,bottom=0,align='center',alpha=1,color="b",hatch="/")
	p2 = plt.bar(i,c_avionics,bottom=c_vehicle,align='center',alpha=1,color="r",hatch="\\")
	p3 = plt.bar(i,c_battery,bottom=c_avionics+c_vehicle, align='center',alpha=1,color="k",hatch="-")

plt.xticks(y_pos, labels, rotation=-60,fontsize=14)
plt.ylabel('Acquisition cost ($thousands US)', fontsize = 16)
plt.grid()
plt.title("Capital Expenses",fontsize = 16)
plt.legend((p1[0],p2[0],p3[0]),("Vehicle","Avionics","Battery"),
	loc='upper left', fontsize = 14)

plt.subplot(2,2,4)
for i, config in enumerate(configs):
	
	c_pilot = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses/PilotCost")
	c_maintenance = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses/MaintenanceCost")
	c_energy = configs[config]["solution"]("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses/PilotCost")
	IOC = configs[config]["solution"]("IOC_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses")
	
	p1 = plt.bar(i,c_pilot,bottom=0,align='center',alpha=1,color="b",hatch="/")
	p2 = plt.bar(i,c_maintenance,bottom=c_pilot,align='center',alpha=1,color="r",hatch="\\")
	p3 = plt.bar(i,c_energy,bottom=c_maintenance+c_pilot, align='center',alpha=1,color="k",hatch="-")
	p4 = plt.bar(i,IOC,bottom=c_energy+c_maintenance+c_pilot,align='center',alpha=1,color="g",hatch="|")

plt.xticks(y_pos, labels, rotation=-60,fontsize=14)
plt.ylabel('Cost per mission ($US)', fontsize = 16)
plt.grid()
plt.title("Operating Expenses (revenue mission only)",fontsize = 16)
plt.legend((p1[0],p2[0],p3[0],p4[0]),("Pilot","Maintanance","Energy","IOC"),
	loc='upper left', fontsize = 14)

cost_title_str = "Aircraft parameters: aircraft cost ratio = \$%0.0f per lb; battery cost ratio = \$%0.0f per kWh; %s\n" \
	% (vehicle_cost_per_weight.to(ureg.lbf**-1).magnitude, \
		battery_cost_per_C.to(ureg.kWh**-1).magnitude, autonomy_string) \
	+ "Pilot wrap rate = \$%0.0f/hour; mechanic wrap rate = \$%0.0f/hour; MMH per FH = %0.1f; deadhead ratio = %0.1f" \
	% (pilot_wrap_rate.to(ureg.hr**-1).magnitude, mechanic_wrap_rate.to(ureg.hr**-1).magnitude, \
		MMH_FH, deadhead_ratio)

plt.suptitle(cost_title_str,fontsize = 16)
plt.tight_layout()
plt.subplots_adjust(left=0.05,right=0.95,bottom=0.14,top=0.89)#adds space at the top for the title
