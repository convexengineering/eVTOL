#CGenerate carpet plot for vehicle sizing

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../../models'))

import numpy as np
from gpkit                  import Model, ureg
from copy                   import deepcopy
from matplotlib             import pyplot as plt
from aircraft_models        import OnDemandAircraft
from mission_models         import OnDemandSizingMission, OnDemandRevenueMission, OnDemandDeadheadMission
from cost_models            import OnDemandMissionCost
from noise_models           import vortex_noise
from standard_substitutions import generic_data, configs

from scipy.interpolate      import interp2d

configs = deepcopy(configs)
del configs["Compound heli"]

#Data from Boeing study
boeing_data = {}

boeing_data["Lift + cruise (Duffy et al.)"]        = {}
boeing_data["Lift + cruise (Duffy et al.)"]["L/D"] = 9.1
boeing_data["Lift + cruise (Duffy et al.)"]["T/A"] = 7.3*ureg("lbf")/ureg("ft")**2

boeing_data["Tilt rotor (Duffy et al.)"]        = {}
boeing_data["Tilt rotor (Duffy et al.)"]["L/D"] = 11.0
boeing_data["Tilt rotor (Duffy et al.)"]["T/A"] = 12.8*ureg("lbf")/ureg("ft")**2

'''
boeing_data["Helicopter"] = {}
boeing_data["Helicopter"]["L/D"] = 7.26
boeing_data["Helicopter"]["T/A"] = 4.1*ureg("lbf")/ureg("ft")**2
'''

#Instantiate arrays
numrows                  = 4
L_D_array                = np.linspace(9, 15, numrows)
T_A_max_array            = np.linspace(4, 16, numrows)
L_D_array, T_A_max_array = np.meshgrid(L_D_array, T_A_max_array)
T_A_max_array            = T_A_max_array * ureg.lbf / ureg.ft**2

MTOM_array  = np.zeros(np.shape(L_D_array))
MTOW_array  = np.zeros(np.shape(L_D_array))
cptpp_array = np.zeros(np.shape(L_D_array))
cppk_array  = np.zeros(np.shape(L_D_array))
SPL_array   = np.zeros(np.shape(L_D_array))
SPL_A_array = np.zeros(np.shape(L_D_array))

#Optimize 

sizing_plot_config = "Lift + cruise" #pull other data from this configuration
config             = sizing_plot_config 

c = configs[config]

for i, T_A_max in enumerate(T_A_max_array[:,0]):
	
	for j, L_D_cruise in enumerate(L_D_array[0]):

		aircraft = OnDemandAircraft()
		aircraft = aircraft.standard_substitutions(config=config, autonomousEnabled=generic_data["autonomousEnabled"])

		aircraft.substitutions.update({
			aircraft.L_D_cruise:     L_D_cruise,
			aircraft.rotors.T_A_max: T_A_max,
			})

		sizing_mission = OnDemandSizingMission(aircraft=aircraft)
		sizing_mission = sizing_mission.standard_substitutions(piloted=generic_data["isSizingMissionPiloted"], reserve=generic_data["reserve"])

		revenue_mission = OnDemandRevenueMission(aircraft=aircraft)
		revenue_mission = revenue_mission.standard_substitutions(piloted=generic_data["isRevenueMissionPiloted"])

		deadhead_mission = OnDemandDeadheadMission(aircraft=aircraft)
		deadhead_mission = deadhead_mission.standard_substitutions(piloted=generic_data["isDeadheadMissionPiloted"])

		mission_cost = OnDemandMissionCost(aircraft=aircraft, revenue_mission=revenue_mission, deadhead_mission=deadhead_mission)
		mission_cost = mission_cost.standard_substitutions(isRevenueMissionPiloted=generic_data["isRevenueMissionPiloted"], isDeadheadMissionPiloted=generic_data["isDeadheadMissionPiloted"])

		objective_function = mission_cost.cpt
		problem            = Model(objective_function, [aircraft, sizing_mission, revenue_mission, deadhead_mission, mission_cost])
		solution           = problem.solve(verbosity=0)

		MTOM_array[i,j]  = solution("MTOM_OnDemandAircraft").to(ureg.kg).magnitude
		MTOW_array[i,j]  = solution("MTOW_OnDemandAircraft").to(ureg.lbf).magnitude
		cptpp_array[i,j] = solution("cost_per_trip_per_passenger_OnDemandMissionCost")
		cppk_array[i,j]  = solution("cost_per_passenger_km_OnDemandMissionCost").to(ureg.km**-1).magnitude
		
		#Noise computations
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
		
		f_peak, SPL_array[i,j],   spectrum = vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S, St, weighting="None")  # Unweighted
		f_peak, SPL_A_array[i,j], spectrum = vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S, St, weighting="A")  # Unweighted

		# print "T/A = %0.4f lbf/ft^2; L/D = %0.4f; cptpp = $%0.2f" \
		# 	% (solution("T/A_OnDemandSizingMission").to(ureg.lbf/ureg.ft**2).magnitude, solution("L_D_cruise"), solution("cost_per_trip_per_passenger"))

		# print "cptpp values: $%0.2f, $%0.2f, $%0.2f" % (cptpp_array[i,j], solution("cost_per_trip_per_passenger"), solution("cost_per_trip_per_passenger_OnDemandMissionCost"))


MTOM_array = MTOM_array * ureg.kg
MTOW_array = MTOW_array * ureg.lbf
cppk_array = cppk_array * ureg.km**-1

# Add Boeing inputs to configs
for config in boeing_data:
	configs[config] = boeing_data[config]
	
# Set up the bilinear interpolation functions
cptpp_interp = interp2d(L_D_array, T_A_max_array.to(ureg.N/ureg.m**2).magnitude, cptpp_array,                          kind="cubic")
cppk_interp  = interp2d(L_D_array, T_A_max_array.to(ureg.N/ureg.m**2).magnitude, cppk_array.to(ureg.km**-1).magnitude, kind="cubic")
SPL_A_interp = interp2d(L_D_array, T_A_max_array.to(ureg.N/ureg.m**2).magnitude, SPL_A_array,                          kind="cubic")	
	
# Estimated cptpp, cpsk and SPL_A
for config in configs:
	try:
		# This will not work for the Boeing configs
		aircraft = OnDemandAircraft()
		aircraft = aircraft.standard_substitutions(config=config, autonomousEnabled=generic_data["autonomousEnabled"])

		sizing_mission = OnDemandSizingMission(aircraft=aircraft)
		sizing_mission = sizing_mission.standard_substitutions(piloted=generic_data["isSizingMissionPiloted"], reserve=generic_data["reserve"])

		objective_function = aircraft.MTOM
		problem            = Model(objective_function, [aircraft, sizing_mission])
		solution           = problem.solve(verbosity=0)

		L_D     = solution("(L/D)_{cruise}")
		T_A_max = solution("(T/A)_{max}").to(ureg.N/ureg.m**2).magnitude

	except:

		L_D     = boeing_data[config]["L/D"]
		T_A_max = boeing_data[config]["T/A"].to(ureg.N/ureg.m**2).magnitude
	
	configs[config]["cptpp"] = cptpp_interp(L_D, T_A_max)
	configs[config]["cppk"]  = cppk_interp( L_D, T_A_max)
	configs[config]["SPL_A"] = SPL_A_interp(L_D, T_A_max)

# Generate sizing plot
plt.ion()
fig1 = plt.figure(figsize=(11,5), dpi=80)
plt.show()

style = {}
style["marker"]     = ["s","^","v","s","o","^","v"]
style["fillstyle"]  = ["full","full","full","none","none","none","none"]
style["markersize"] = 16

#First set of lines
for i,L_D in enumerate(L_D_array[0,:]):
	cptpp_row = cptpp_array[:,i]
	SPL_A_row = SPL_A_array[:,i]
	plt.plot(cptpp_row,SPL_A_row,'k-',linewidth=2)
	
	x = cptpp_row[0]
	y = SPL_A_row[0]
	label = "L/D = %0.1f" % L_D
	plt.text(x-5,y-1,label,fontsize=16,rotation=45)

#Second set of lines
for i,T_A in enumerate(T_A_max_array[:,0]):
	cptpp_row = cptpp_array[i,:]
	SPL_A_row = SPL_A_array[i,:]
	plt.plot(cptpp_row,SPL_A_row,'k-',linewidth=2)
	
	x = cptpp_row[-1]
	y = SPL_A_row[-1]
	label = "T/A = %0.0f N/m$^2$" % T_A.to(ureg.N/ureg.m**2).magnitude
	plt.text(x-11,y-0.2,label,fontsize=16,rotation=0)

#Configuration data
for i,config in enumerate(configs):
	cptpp = configs[config]["cptpp"]
	SPL_A = configs[config]["SPL_A"]
	plt.plot(cptpp,SPL_A,'k',marker=style["marker"][i],fillstyle=style["fillstyle"][i],\
		linestyle="None", markersize=style["markersize"],markeredgewidth=3,label=config)
	
plt.grid()
[xmin,xmax] = plt.gca().get_xlim()
plt.xlim(xmin=xmin-10,xmax=xmax+3)
[ymin,ymax] = plt.gca().get_ylim()
plt.ylim(ymin=ymin-5,ymax=ymax+1)

locs,labels = plt.xticks()
new_xticks = [""]*len(locs)
for i,loc in enumerate(locs):
	new_xticks[i] = "\$%0.0f" % loc
plt.xticks(locs,new_xticks,fontsize=16)
plt.yticks(fontsize=18)

plt.xlabel('Cost per trip, per passenger', fontsize=20)
plt.ylabel('SPL (dBA)', fontsize = 20)
plt.legend(numpoints = 1,loc='lower right', framealpha=1, fontsize = 14)

plt.tight_layout()
plt.subplots_adjust(left=0.10,right=0.97,bottom=0.14,top=0.97)
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
for i in range(np.size(T_A_max_array[0])):
	for j, T_A in enumerate(T_A_max_array[i]):
		T_A = T_A.to(ureg.lbf/ureg.ft**2).magnitude
		output_data.write("%0.2f\t" % T_A)
	output_data.write("\n")
	
output_data.write("\n")
output_data.write("MTOM (lbf)\n\n")
for i in range(np.size(MTOM_array[0])):
	for j, MTOM in enumerate(MTOM_array[i]):
		MTOM = MTOM.to(ureg.lbf).magnitude
		output_data.write("%0.2f\t" % MTOM)
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

