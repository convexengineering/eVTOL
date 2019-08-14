#Vehicle designs for different time frames

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../../models'))

import numpy as np
from gpkit                  import Model, ureg
from copy                   import deepcopy
from collections            import OrderedDict
from matplotlib             import pyplot as plt
from aircraft_models        import OnDemandAircraft
from mission_models         import OnDemandSizingMission, OnDemandRevenueMission, OnDemandDeadheadMission
from cost_models            import OnDemandMissionCost
from noise_models           import vortex_noise
from standard_substitutions import generic_data, configs


# Data specific to study
cases = OrderedDict()

for config in configs:

	cases[config] = OrderedDict()

	cases[config]["Initial"]   = {}
	cases[config]["Near term"] = {}
	cases[config]["Long term"] = {}

	cases[config]["Initial"]["autonomousEnabled"]   = False
	cases[config]["Near term"]["autonomousEnabled"] = True
	cases[config]["Long term"]["autonomousEnabled"] = True

	cases[config]["Initial"]["e"]   = 400 * ureg.Wh / ureg.kg
	cases[config]["Near term"]["e"] = 450 * ureg.Wh / ureg.kg
	cases[config]["Long term"]["e"] = 500 * ureg.Wh / ureg.kg

	cases[config]["Initial"]["isSizingMissionPiloted"]   = True
	cases[config]["Near term"]["isSizingMissionPiloted"] = True
	cases[config]["Long term"]["isSizingMissionPiloted"] = True

	cases[config]["Initial"]["isRevenueMissionPiloted"]   = True
	cases[config]["Near term"]["isRevenueMissionPiloted"] = True
	cases[config]["Long term"]["isRevenueMissionPiloted"] = False

	cases[config]["Initial"]["isDeadheadMissionPiloted"]   = True
	cases[config]["Near term"]["isDeadheadMissionPiloted"] = False
	cases[config]["Long term"]["isDeadheadMissionPiloted"] = False

	cases[config]["Initial"]["deadhead_ratio"]   = 0.5
	cases[config]["Near term"]["deadhead_ratio"] = 0.35
	cases[config]["Long term"]["deadhead_ratio"] = 0.2

	cases[config]["Initial"]["airframe_cost_per_weight"]   = 600 * ureg.lbf**-1
	cases[config]["Near term"]["airframe_cost_per_weight"] = 400 * ureg.lbf**-1
	cases[config]["Long term"]["airframe_cost_per_weight"] = 200 * ureg.lbf**-1

	cases[config]["Initial"]["battery_cost_per_energy"]   = 400 * ureg.kWh**-1
	cases[config]["Near term"]["battery_cost_per_energy"] = 200 * ureg.kWh**-1
	cases[config]["Long term"]["battery_cost_per_energy"] = 100 * ureg.kWh**-1


# Optimize
for config in cases:
	
	print "Solving configuration: " + config

	for i, case in enumerate(cases[config]):
		
		c = cases[config][case]

		aircraft = OnDemandAircraft()
		aircraft = aircraft.standard_substitutions(config=config, autonomousEnabled=c["autonomousEnabled"])

		aircraft.substitutions.update({
			aircraft.airframe.cost_per_weight: c["airframe_cost_per_weight"],
			aircraft.battery.e:                c["e"],
			aircraft.battery.cost_per_energy:  c["battery_cost_per_energy"],
		})

		sizing_mission = OnDemandSizingMission(aircraft=aircraft)
		sizing_mission = sizing_mission.standard_substitutions(piloted=c["isSizingMissionPiloted"], reserve=generic_data["reserve"])

		revenue_mission = OnDemandRevenueMission(aircraft=aircraft)
		revenue_mission = revenue_mission.standard_substitutions(piloted=c["isRevenueMissionPiloted"])

		deadhead_mission = OnDemandDeadheadMission(aircraft=aircraft)
		deadhead_mission = deadhead_mission.standard_substitutions(piloted=c["isDeadheadMissionPiloted"])

		mission_cost = OnDemandMissionCost(aircraft=aircraft, revenue_mission=revenue_mission, deadhead_mission=deadhead_mission)
		mission_cost = mission_cost.standard_substitutions(isRevenueMissionPiloted=c["isRevenueMissionPiloted"], isDeadheadMissionPiloted=c["isDeadheadMissionPiloted"])

		mission_cost.substitutions.update({
			mission_cost.deadhead_ratio: c["deadhead_ratio"]
		})

		objective_function = mission_cost.cpt
		problem            = Model(objective_function, [aircraft, sizing_mission, revenue_mission, deadhead_mission, mission_cost])
		solution           = problem.solve(verbosity=0)

		c["solution"] = solution

		print case + "aircraft cost: %0.1f per kg" % solution("cost_per_mass_OnDemandAircraft/Airframe").to(ureg.kg**-1).magnitude

		# Noise computations (sizing mission)
		T_perRotor = solution("T_perRotor_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
		T_A        = solution("T/A_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
		V_tip      = solution("v_{tip}_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
		s          = solution("s_OnDemandAircraft/Rotors")
		Cl_mean    = solution("Cl_{mean}_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
		N          = solution("N_OnDemandAircraft/Rotors")
		c_avg      = solution("c_{avg}_OnDemandAircraft/Rotors")
		t_avg      = solution("t_{avg}_OnDemandAircraft/Rotors")
		rho        = solution("\\rho_OnDemandSizingMission/HoverTakeoff/HoverFlightState/FixedStandardAtmosphere")
		
		delta_S = generic_data["delta_S"]
		St      = generic_data["Strouhal_number"]

		f_peak, SPL, spectrum = vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S, St, weighting="A")

		c["SPL_sizing_A"]      = SPL
		c["f_{peak}"]          = f_peak
		c["spectrum_sizing_A"] = spectrum

		#Noise computations (revenue mission)
		T_perRotor = solution("T_perRotor_OnDemandRevenueMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
		T_A        = solution("T/A_OnDemandRevenueMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
		V_tip      = solution("v_{tip}_OnDemandRevenueMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
		Cl_mean    = solution("Cl_{mean}_OnDemandRevenueMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
		rho        = solution("\\rho_OnDemandRevenueMission/HoverTakeoff/HoverFlightState/FixedStandardAtmosphere")

		f_peak, SPL, spectrum = vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S, St, weighting="A")
		
		c["SPL_revenue_A"]      = SPL
		c["f_{peak}"]           = f_peak
		c["spectrum_revenue_A"] = spectrum


# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(11,8), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

y_pos = np.arange(len(configs))
labels = [""]*len(configs)
for i, config in enumerate(configs):
	if config == "Compound heli":
		labels[i] = config.replace(" ", "\n")  # Replace spaces with newlines
	else:
		labels[i] = config

xmin = np.min(y_pos) - 0.7
xmax = np.max(y_pos) + 0.7

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

# Takeoff mass
plt.subplot(2,3,1)
for i,config in enumerate(cases):

	for j, case in enumerate(cases[config]):

		offset   = style["offsets"][j]
		solution = cases[config][case]["solution"]
		MTOM     = solution("MTOM").to(ureg.kg).magnitude

		if i==0:
			plt.bar(i+offset, MTOM, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k', label=case)
		else:
			plt.bar(i+offset, MTOM, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

plt.xlim(xmin=xmin, xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.0*ymax)
plt.grid()
plt.xticks(y_pos, labels,         fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                       fontsize=style["fontsize"]["yticks"])
plt.ylabel('Mass (kg)',           fontsize=style["fontsize"]["ylabel"])
plt.title("Maximum Takeoff Mass", fontsize=style["fontsize"]["title"])
plt.legend(loc="lower left",      fontsize=style["fontsize"]["legend"], framealpha=1)


# Cost per passenger-km
plt.subplot(2,3,2)
for i,config in enumerate(cases):

	for j, case in enumerate(cases[config]):

		offset   = style["offsets"][j]
		solution = cases[config][case]["solution"]
		cppk     = solution("cost_per_passenger_km_OnDemandMissionCost").to(ureg.km**-1).magnitude

		plt.bar(i+offset, cppk, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.0*ymax)
plt.grid()
plt.xticks(y_pos, labels,                  fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                                fontsize=style["fontsize"]["yticks"])
plt.ylabel('Cost ($US/km)',                fontsize=style["fontsize"]["ylabel"])
plt.title("Cost per Passenger Kilometer",  fontsize=style["fontsize"]["title"])


# Hover SPL (revenue mission)
plt.subplot(2,3,3)
for i,config in enumerate(cases):

	for j, case in enumerate(cases[config]):

		offset = style["offsets"][j]
		SPL_A  = cases[config][case]["SPL_sizing_A"]

		plt.bar(i+offset, SPL_A, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

SPL_req = 62
plt.plot([np.min(y_pos)-1, np.max(y_pos)+1], [SPL_req, SPL_req], color="black", linewidth=3, linestyle="--", label="62 dBA")

plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymin=57, ymax=ymax-1)
plt.grid()
plt.xticks(y_pos, labels,                  fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                                fontsize=style["fontsize"]["yticks"])
plt.ylabel('SPL (dBA)',                    fontsize=style["fontsize"]["ylabel"])
plt.title("Hover Sound (sizing mission)", fontsize=style["fontsize"]["title"])
plt.legend(loc="lower left",               fontsize=style["fontsize"]["legend"], framealpha=1)


# Acquisition costs
plt.subplot(2,3,4)
for i,config in enumerate(cases):

	for j, case in enumerate(cases[config]):

		offset     = style["offsets"][j]
		solution   = cases[config][case]["solution"]
		c_airframe = solution("purchase_price_OnDemandAircraft/Airframe") / 1e6
		c_avionics = solution("purchase_price_OnDemandAircraft/Avionics") / 1e6
		c_battery  = solution("purchase_price_OnDemandAircraft/Battery")  / 1e6
		c_total    = c_airframe + c_avionics + c_battery

		plt.bar(i+offset, c_total, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.0*ymax)
plt.grid()
plt.xticks(y_pos, labels,         fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                       fontsize=style["fontsize"]["yticks"])
plt.ylabel('Cost ($millions US)', fontsize=style["fontsize"]["ylabel"])
plt.title("Acquisition Costs",    fontsize=style["fontsize"]["title"])


# Capital Expenses per Trip
plt.subplot(2,3,5)
for i,config in enumerate(cases):

	for j, case in enumerate(cases[config]):

		offset    = style["offsets"][j]
		solution  = cases[config][case]["solution"]

		NdNr                     =        solution("N_{deadhead}/N_{revenue}_OnDemandMissionCost")
		amortized_capex_revenue  =        solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses")
		amortized_capex_deadhead = NdNr * solution("cost_per_mission_OnDemandMissionCost/DeadheadMissionCost/CapitalExpenses")
		amortized_capex          = amortized_capex_revenue + amortized_capex_deadhead

		plt.bar(i+offset, amortized_capex, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.0*ymax)
plt.grid()
plt.xticks(y_pos, labels,          fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                        fontsize=style["fontsize"]["yticks"])
plt.ylabel('Cost ($US/trip)',      fontsize=style["fontsize"]["ylabel"])
plt.title("Trip Capital Expenses", fontsize=style["fontsize"]["title"])


# Operating Expenses per Mission
plt.subplot(2,3,6)
for i,config in enumerate(cases):

	for j, case in enumerate(cases[config]):

		offset    = style["offsets"][j]
		solution  = cases[config][case]["solution"]

		NdNr                    =        solution("N_{deadhead}/N_{revenue}_OnDemandMissionCost")
		amortized_opex_revenue  =        solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses")
		amortized_opex_deadhead = NdNr * solution("cost_per_mission_OnDemandMissionCost/DeadheadMissionCost/OperatingExpenses")
		amortized_opex          = amortized_opex_revenue + amortized_opex_deadhead
		plt.bar(i+offset, amortized_opex, align='center', alpha=1, width=style["bar_width_narrow"], color=style["colors"][j], edgecolor='k')

plt.xlim(xmin=xmin,xmax=xmax)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymax=1.0*ymax)
plt.grid()
plt.xticks(y_pos, labels,            fontsize=style["fontsize"]["xticks"], rotation=style["rotation"])
plt.yticks(                          fontsize=style["fontsize"]["yticks"])
plt.ylabel('Cost ($US/trip)',        fontsize=style["fontsize"]["ylabel"])
plt.title("Trip Operating Expenses", fontsize=style["fontsize"]["title"])


plt.tight_layout()
plt.subplots_adjust(left=0.09,right=0.96, bottom=0.15, top=0.96)
plt.savefig('time_frame_plot_01.pdf')