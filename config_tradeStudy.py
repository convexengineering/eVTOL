import math
import numpy as np
from gpkit import Variable, Model, Vectorize, ureg
from standard_atmosphere import stdatmo
from matplotlib import pyplot as plt

from aircraft_models import SimpleOnDemandAircraft 
from aircraft_models import OnDemandSizingMission, OnDemandTypicalMission


#General data
N = 12 #number of propellers. Required, but has no effect since T/A is constrained
eta_cruise = 0.85 #propulsive efficiency in cruise
eta_electric = 0.95 #electrical system efficiency
weight_fraction = 0.5 #structural mass fraction
C_m = 400*ureg.Wh/ureg.kg #battery energy density
N_crew = 1
n=1.0#battery discharge parameter
reserve_type = "Uber"

sizing_mission_range = 100*ureg.nautical_mile
typical_mission_range = 40*ureg.nautical_mile

sizing_time_in_hover=120*ureg.s
typical_time_in_hover=30*ureg.s

sizing_N_passengers = 3
typical_N_passengers = 2

cost_per_weight=112*ureg.lbf**-1
pilot_salary = 40*ureg.hr**-1
mechanic_salary=30*ureg.hr**-1


# Configuration-specific data
configs = {}

'''
configs["Multirotor"] = {}
configs["Multirotor"]["V_{cruise}"] = 50*ureg("mph")
configs["Multirotor"]["L/D"] = 1.5
configs["Multirotor"]["T/A"] = 3.75*ureg("lbf")/ureg("ft")**2
'''
'''
configs["Autogyro"] = {}
configs["Autogyro"]["V_{cruise}"] = 100*ureg("mph")
configs["Autogyro"]["L/D"] = 3.5
configs["Autogyro"]["T/A"] = 3.75*ureg("lbf")/ureg("ft")**2
'''
'''
configs["Helicopter"] = {}
configs["Helicopter"]["V_{cruise}"] = 100*ureg("mph")
configs["Helicopter"]["L/D"] = 4.25
configs["Helicopter"]["T/A"] = 4.5*ureg("lbf")/ureg("ft")**2
'''
'''
configs["Tilt duct"] = {}
configs["Tilt duct"]["V_{cruise}"] = 150*ureg("mph")
configs["Tilt duct"]["L/D"] = 10.
configs["Tilt duct"]["T/A"] = 40*ureg("lbf")/ureg("ft")**2
'''

'''
configs["Coaxial heli"] = {}
configs["Coaxial heli"]["V_{cruise}"] = 150*ureg("mph")
configs["Coaxial heli"]["L/D"] = 5.5
configs["Coaxial heli"]["T/A"] = 7*ureg("lbf")/ureg("ft")**2
'''


configs["Lift + cruise"] = {}
configs["Lift + cruise"]["V_{cruise}"] = 150*ureg("mph")
configs["Lift + cruise"]["L/D"] = 10
configs["Lift + cruise"]["T/A"] = 15*ureg("lbf")/ureg("ft")**2


configs["Tilt wing"] = {}
configs["Tilt wing"]["V_{cruise}"] = 150*ureg("mph")
configs["Tilt wing"]["L/D"] = 12
configs["Tilt wing"]["T/A"] = 4.5*ureg("lbf")/ureg("ft")**2


configs["Compound heli"] = {}
configs["Compound heli"]["V_{cruise}"] = 150*ureg("mph")
configs["Compound heli"]["L/D"] = 9
configs["Compound heli"]["T/A"] = 7*ureg("lbf")/ureg("ft")**2


configs["Tilt rotor"] = {}
configs["Tilt rotor"]["V_{cruise}"] = 150*ureg("mph")
configs["Tilt rotor"]["L/D"] = 14
configs["Tilt rotor"]["T/A"] = 15*ureg("lbf")/ureg("ft")**2

for config in configs:
	
	print "Solving configuration: " + config

	c = configs[config]

	V_cruise = c["V_{cruise}"]
	V_loiter = V_cruise #approximation
	L_D = c["L/D"]
	T_A = c["T/A"]

	Aircraft = SimpleOnDemandAircraft(N=N,L_D=L_D,eta_cruise=eta_cruise,C_m=C_m,
		weight_fraction=weight_fraction,N_crew=N_crew,n=n,eta_electric=eta_electric)

	SizingMission = OnDemandSizingMission(Aircraft,mission_range=sizing_mission_range,
		V_cruise=V_cruise,V_loiter=V_loiter,N_passengers=sizing_N_passengers,
		time_in_hover=sizing_time_in_hover,reserve_type=reserve_type)
	SizingMission.substitutions.update({SizingMission.fs0.topvar("T/A"):T_A,
		SizingMission.fs2.topvar("T/A"):T_A,SizingMission.fs3.topvar("T/A"):T_A,
		SizingMission.fs5.topvar("T/A"):T_A})

	TypicalMission = OnDemandTypicalMission(Aircraft,mission_range=typical_mission_range,
		V_cruise=V_cruise,N_passengers=typical_N_passengers,time_in_hover=typical_time_in_hover,
		cost_per_weight=cost_per_weight,pilot_salary=pilot_salary,mechanic_salary=mechanic_salary)
	
	problem = Model(TypicalMission["cost_per_trip"],
		[Aircraft, SizingMission, TypicalMission])
	solution = problem.solve(verbosity=0)

	configs[config]["solution"] = solution

# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(11,8), dpi=80)
plt.show()

#FOM vs. VT
plt.subplot(2,2,1)
y_pos = np.arange(len(configs))
labels = [""]*len(configs)

for i, config in enumerate(configs):
	MTOW = configs[config]["solution"]["variables"]["MTOW_SimpleOnDemandAircraft"].to(ureg.lbf).magnitude
	plt.bar(i,MTOW,align='center',alpha=0.5)
	labels[i] = config

plt.xticks(y_pos, labels, rotation=-45)
plt.ylabel('MTOW (lbf)', fontsize = 16)
plt.title("Maximum Takeoff Weight",fontsize = 20)