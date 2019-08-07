#CGenerate carpet plot for vehicle sizing

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
from scipy.interpolate import interp2d
from copy import deepcopy

#Data from Boeing study
boeing_data = {}

boeing_data["Lift + cruise"] = {}
boeing_data["Lift + cruise"]["L/D"] = 9.1
boeing_data["Lift + cruise"]["T/A"] = 7.3*ureg("lbf")/ureg("ft")**2

boeing_data["Tilt rotor"] = {}
boeing_data["Tilt rotor"]["L/D"] = 11.0
boeing_data["Tilt rotor"]["T/A"] = 12.8*ureg("lbf")/ureg("ft")**2

'''
boeing_data["Helicopter"] = {}
boeing_data["Helicopter"]["L/D"] = 7.26
boeing_data["Helicopter"]["T/A"] = 4.1*ureg("lbf")/ureg("ft")**2
'''

#Instantiate arrays
numrows = 4
L_D_array = np.linspace(9,15,numrows)
T_A_array = np.linspace(4,16,numrows)
L_D_array, T_A_array = np.meshgrid(L_D_array, T_A_array)
T_A_array = T_A_array*ureg.lbf/ureg.ft**2

TOGW_array = np.zeros(np.shape(L_D_array))
cptpp_array = np.zeros(np.shape(L_D_array))
SPL_array = np.zeros(np.shape(L_D_array))
SPL_A_array = np.zeros(np.shape(L_D_array))

#Optimize 
configs = deepcopy(configuration_data)
del configs["Tilt duct"]
del configs["Multirotor"]
del configs["Autogyro"]
del configs["Helicopter"]
del configs["Coaxial heli"]
del configs["Compound heli"]

sizing_plot_config = "Lift + cruise" #pull other data from this configuration
config = sizing_plot_config 

c = configs[config]

for i, T_A in enumerate(T_A_array[:,0]):
	for j, L_D_cruise in enumerate(L_D_array[0]):
		
		problem_subDict = {}

		Aircraft = OnDemandAircraft(autonomousEnabled=generic_data["autonomousEnabled"])
		problem_subDict.update({
			Aircraft.L_D_cruise: L_D_cruise, #estimated L/D in cruise
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
			SizingMission.T_A: T_A,#disk loading
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

		TOGW_array[i,j] = solution("TOGW_OnDemandAircraft").to(ureg.lbf).magnitude
		cptpp_array[i,j] = solution("cost_per_trip_per_passenger_OnDemandMissionCost")
		
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
		f_peak, SPL_array[i,j], spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
			weighting="None")
		
		#A-weighted
		f_peak, SPL_A_array[i,j], spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
			weighting="A")

		print "T/A = %0.4f lbf/ft^2; L/D = %0.4f; cptpp = $%0.2f" \
			% (solution("T/A_OnDemandSizingMission").to(ureg.lbf/ureg.ft**2).magnitude, solution("L_D_cruise"), solution("cost_per_trip_per_passenger"))

		# print "cptpp values: $%0.2f, $%0.2f, $%0.2f" % (cptpp_array[i,j], solution("cost_per_trip_per_passenger"), solution("cost_per_trip_per_passenger_OnDemandMissionCost"))


TOGW_array = TOGW_array*ureg.lbf

#Add Boeing inputs to configs
for config in boeing_data:
	label = config + " (Duffy et al.)"
	configs[label] = boeing_data[config]

	
#Set up the bilinear interpolation functions
cptpp_interp = interp2d(L_D_array,T_A_array.to(ureg.N/ureg.m**2).magnitude,\
	cptpp_array,kind="cubic")
SPL_A_interp = interp2d(L_D_array,T_A_array.to(ureg.N/ureg.m**2).magnitude,\
	SPL_A_array,kind="cubic")
	
	
#Estimated cptpp and SPL_A
for config in configs:
	L_D = configs[config]["L/D"]
	T_A = configs[config]["T/A"].to(ureg.N/ureg.m**2)
	
	configs[config]["cptpp"] = cptpp_interp(L_D,T_A)
	configs[config]["SPL_A"] = SPL_A_interp(L_D,T_A)

#Generate sizing plot
plt.ion()
fig1 = plt.figure(figsize=(11,5), dpi=80)
plt.show()

style = {}
style["marker"] = ["s","^","v","s","o","^","v"]
style["fillstyle"] = ["full","full","full","none","none","none","none"]
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
for i,T_A in enumerate(T_A_array[:,0]):
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
for i in range(np.size(T_A_array[0])):
	for j, T_A in enumerate(T_A_array[i]):
		T_A = T_A.to(ureg.lbf/ureg.ft**2).magnitude
		output_data.write("%0.2f\t" % T_A)
	output_data.write("\n")
	
output_data.write("\n")
output_data.write("TOGW (lbf)\n\n")
for i in range(np.size(TOGW_array[0])):
	for j, TOGW in enumerate(TOGW_array[i]):
		TOGW = TOGW.to(ureg.lbf).magnitude
		output_data.write("%0.2f\t" % TOGW)
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

