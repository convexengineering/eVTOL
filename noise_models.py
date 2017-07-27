#Functions for rotor noise prediction

import math
import numpy as np
from gpkit import Model, ureg
from matplotlib import pyplot as plt
from aircraft_models import OnDemandAircraft 
from aircraft_models import OnDemandSizingMission, OnDemandRevenueMission
from aircraft_models import OnDemandDeadheadMission, OnDemandMissionCost
from standard_atmosphere import stdatmo
from study_input_data import generic_data, configuration_data

def vortex_noise(T_perRotor,A,VT,s,Cl_mean,N,x=500*ureg.ft,h=0*ureg.ft,t_c=0.12,St=0.28):
	
	k2 = 1.206e-2 * ureg.s**3/ureg.ft**3
	pi = math.pi
	
	atmospheric_data = stdatmo(h)
	rho = atmospheric_data["\rho"].to(ureg.kg/ureg.m**3)
	
	T_total = T_perRotor*N
	p_ratio = k2*(VT/(rho*x))*np.sqrt((T_total/s)*(T_perRotor/A))
	SPL = 20*np.log10(p_ratio)

	V_07 = 0.7*VT
	R = np.sqrt(A/pi) #rotor radius
	c = (pi*s*R)/N #Rotor blade chord
	alpha = Cl_mean/(2*pi) #Angle of attack (average)
	h = t_c*c*np.cos(alpha) + c*np.sin(alpha) #blade projected thickness
	f_peak_Hz = St*V_07/h #peak frequency (Hz)

	spectrum = {}
	spectrum["f_Hz"] = f_peak_Hz*[0.5,1,2,4,8,16]
	offsets_dB = [7.92,4.17,8.33,8.75,12.92,13.33]
	spectrum["SPL"] = SPL*np.ones(np.shape(offsets_dB)) - offsets_dB

	return f_peak_Hz, SPL, spectrum

def noise_weighting(f,SPL,type="A"):
	#Noise weighting function. Currently, only A-weighting is implemented.
	if type == "A":
		f = f.to(ureg.Hz).magnitude
		numerator = 12194**2*f**4
		denominator = (f**2+20.6**2)*(f**2+12194**2)*np.sqrt((f**2+107.7**2)*(f**2+737.9**2))
		R = numerator/denominator
		A = 20*np.log10(R) + 2.00

	dBA = SPL + A
	return dBA


if __name__=="__main__":
	
	configs = configuration_data.copy()
	config = "Tilt rotor"

	print "Solving configuration: " + config

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
	deadhead_ratio = generic_data["deadhead_ratio"]

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

	c = configs[config]

	V_cruise = c["V_{cruise}"]
	L_D_cruise = c["L/D"]
	T_A = c["T/A"]
	Cl_mean_max = c["Cl_{mean_{max}}"]
	N = c["N"]
	loiter_type = c["loiter_type"]

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
		pilot_wrap_rate=pilot_wrap_rate,mechanic_wrap_rate=mechanic_wrap_rate,MMH_FH=MMH_FH,
		deadhead_ratio=deadhead_ratio)
	
	problem = Model(MissionCost["cost_per_trip"],
		[Aircraft, SizingMission, RevenueMission, DeadheadMission, MissionCost])
	
	solution = problem.solve(verbosity=0)
	SPL_solution = 20*np.log10(solution("p_{ratio}_OnDemandSizingMission")[0])

	T_perRotor = solution("T_perRotor_OnDemandSizingMission")[0]
	A = solution("A")
	VT = solution("VT_OnDemandSizingMission")[0]
	s = solution("s")
	Cl_mean = solution("Cl_{mean_{max}}")
	N = solution("N")

	f_peak_Hz, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,A=A,VT=VT,s=s,
		Cl_mean=Cl_mean,N=N,x=500*ureg.ft,h=0*ureg.ft,t_c=0.12,St=0.28)

	dBA_offset = noise_weighting(f_peak_Hz,0)


	print "Vortex noise SPL: %0.1f dB" % SPL
	print "Peak frequency: %0.1f Hz" % f_peak_Hz.to(ureg.Hz).magnitude
	print "dBA offset at peak: %0.1f dB" % dBA_offset


