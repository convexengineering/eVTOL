#Generate air ambulance data (assuming a tilt-rotor configuration)

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/..'))

import numpy as np
from gpkit import Variable, Model, Vectorize, ureg
from matplotlib import pyplot as plt
from aircraft_models import OnDemandAircraft
from mission_profiles import AirAmbulanceSizingMission, AirAmbulanceMissionCost
from study_input_data import generic_data, configuration_data
from noise_models import vortex_noise

#Unput assumptions
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
pilot_wrap_rate = 0.0001*ureg.hr**-1 #Pilot cost not included
mechanic_wrap_rate = generic_data["mechanic_wrap_rate"]
MMH_FH = generic_data["MMH_FH"]
deadhead_ratio = generic_data["deadhead_ratio"]
	
delta_S = generic_data["delta_S"]
	
config = "Tilt rotor"
print "Solving configuration: " + config
configs = configuration_data.copy()
c = configs[config]
	
V_cruise = c["V_{cruise}"]
L_D_cruise = c["L/D"]
T_A = c["T/A"]
Cl_mean_max = c["Cl_{mean_{max}}"]
N = c["N"]
loiter_type = c["loiter_type"]
tailRotor_power_fraction_hover = c["tailRotor_power_fraction_hover"]
tailRotor_power_fraction_levelFlight = c["tailRotor_power_fraction_levelFlight"]
weight_fraction = c["weight_fraction"]

#Mission parameter (over which to iterate)
mission_range_array = np.linspace(20,90,10)*ureg.nautical_mile #range of the 1st, 2nd cruise segments

output_data = [{} for i in range(np.size(mission_range_array))]

#
for i,mission_range in enumerate(mission_range_array):
	
	t_hover_array = [30,120,30,30,30,30]*ureg.second
	
	Aircraft = OnDemandAircraft(N=N,L_D_cruise=L_D_cruise,eta_cruise=eta_cruise,C_m=C_m,
		Cl_mean_max=Cl_mean_max,weight_fraction=weight_fraction,n=n,eta_electric=eta_electric,
		cost_per_weight=vehicle_cost_per_weight,cost_per_C=battery_cost_per_C,
		autonomousEnabled=autonomousEnabled)

	SizingMission = AirAmbulanceSizingMission(Aircraft,V_cruise=V_cruise,N_crew=3,
		N_passengers=1,reserve_type=reserve_type,loiter_type=loiter_type,
		tailRotor_power_fraction_hover=tailRotor_power_fraction_hover,
		tailRotor_power_fraction_levelFlight=tailRotor_power_fraction_levelFlight)
	SizingMission.substitutions.update({SizingMission.fs0.topvar("T/A"):T_A})
	
	segment_range_array = [mission_range.to(ureg.nmi).magnitude,\
		mission_range.to(ureg.nmi).magnitude,10]*ureg.nautical_mile
	
	for j,segment_range in enumerate(segment_range_array):
		SizingMission.substitutions.update({SizingMission.topvar("segment_range")[j]:segment_range})
	for j,t_hover in enumerate(t_hover_array):
		SizingMission.substitutions.update({SizingMission.topvar("t_{hover}")[j]:t_hover})
	
	MissionCost = AirAmbulanceMissionCost(Aircraft,SizingMission,pilot_wrap_rate=pilot_wrap_rate)
	
	problem = Model(MissionCost.topvar("cost_per_mission"),[Aircraft, SizingMission, MissionCost])
	solution = problem.solve(verbosity=0)
	output_data[i] = solution

	
#Print data to text file
text_file = open("air_ambulance_data.txt","w")

text_file.write("Configuration: %s\n" % config)
text_file.write("\n")
text_file.write("The mission profile includes three cruise segments. ")
text_file.write("The range of the first two segments is set equal, and is the independent variable. ")
text_file.write("The third segment is %0.1f nmi long.\n" \
	% segment_range_array[-1].to(ureg.nmi).magnitude)
text_file.write("\n")
text_file.write("Note: pilot, crew are not included in cost estimates.\n")
text_file.write("\n")

text_file.write("R_segment (nmi)\tMTOW (lbf)\tW_{battery} (lbf)\tt_{mission} (minutes)\t")
text_file.write("Mission cost ($)\tPurchase price ($)\t")
text_file.write("\n")

for i,range in enumerate(mission_range_array):
	sol = output_data[i]
	
	text_file.write("%0.1f\t" % range.to(ureg.nmi).magnitude)
	text_file.write("%0.1f\t" % sol("MTOW").to(ureg.lbf).magnitude)
	text_file.write("%0.1f\t" % sol("W_OnDemandAircraft/Battery").to(ureg.lbf).magnitude)
	text_file.write("%0.1f\t" % sol("t_{mission}").to(ureg.minute).magnitude)
	text_file.write("%0.2f\t" % sol("cost_per_mission_AirAmbulanceMissionCost"))
	
	purchase_price = sol("purchase_price_OnDemandAircraft")\
		+ sol("purchase_price_OnDemandAircraft/Battery") + sol("purchase_price_OnDemandAircraft/Avionics")
	text_file.write("%0.2f\t" % purchase_price)
	
	text_file.write("\n")
	
text_file.close()




