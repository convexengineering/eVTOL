#Functions for rotor noise prediction

import math
import numpy as np
from scipy.special import jv
from gpkit import Model, ureg
from matplotlib import pyplot as plt
from aircraft_models import OnDemandAircraft 
from aircraft_models import OnDemandSizingMission, OnDemandRevenueMission
from aircraft_models import OnDemandDeadheadMission, OnDemandMissionCost
from standard_atmosphere import stdatmo
from study_input_data import generic_data, configuration_data


def rotational_noise(T_perRotor,Q_perRotor,R,VT,s,N,B,theta=175*ureg.degree,delta_S=500*ureg.ft,h=0*ureg.ft,
	t_c=0.12,num_harmonics=10,weighting="None"):

	pi = math.pi
	P0 = (2e-5)*ureg.Pa

	atmospheric_data = stdatmo(h)
	rho = atmospheric_data["\rho"].to(ureg.kg/ureg.m**3)
	a = atmospheric_data["a"].to(ureg.m/ureg.s)

	R_eff = 0.8*R #Effective rotor radius
	A = pi*(R**2) #rotor disk area
	c = (pi*s*R)/B #Rotor blade chord
	omega = (VT/R).to(ureg.rad/ureg.s)#blade angular velocity (rad/s)
	t = t_c*c #blade thickness

	spectrum = {}
	spectrum["m"] = range(1,num_harmonics+1,1)
	spectrum["f"] = np.zeros(num_harmonics)*ureg.rad/ureg.s
	spectrum["SPL"] = np.zeros(num_harmonics)

	p_ratio_squared = 0

	for i, m in enumerate(spectrum["m"]):
		spectrum["f"][i] = m*B*omega
		
		bessel_argument = (m*B*omega/a)*R_eff*np.sin(theta)
		bessel_argument = bessel_argument.to(ureg.dimensionless)
		bessel_term = jv(m*B,bessel_argument)

		#RMS acoustic pressures
		P_mL = ((m*B*omega)/(2*np.sqrt(2)*pi*a*delta_S))*(T_perRotor*np.cos(theta) \
			- Q_perRotor*a/(omega*R_eff**2))*bessel_term #loading
		P_mT = ((-rho*((m*B*omega)**2)*B)/(3*np.sqrt(2)*pi*delta_S))*c*t*R_eff*bessel_term #thickness

		p_ratio = np.sqrt(N)*np.abs(P_mL + P_mT)/P0
		p_ratio_squared = p_ratio_squared + p_ratio**2

		spectrum["SPL"][i] = 20*np.log10(p_ratio)

	SPL = 10*np.log10(p_ratio_squared)
	f_fundamental = spectrum["f"][0]

	if weighting == "A":
		spectrum["SPL"] = noise_weighting(spectrum["f"],spectrum["SPL"],type="A")
		
		p_ratio_squared = 0
		for i,SPL in enumerate(spectrum["SPL"]):
			p_ratio_squared += 10**(SPL/10)

		SPL = 10*np.log10(p_ratio_squared)

	return f_fundamental, SPL, spectrum


def vortex_noise(T_perRotor,R,VT,s,Cl_mean,N,B,delta_S=500*ureg.ft,h=0*ureg.ft,t_c=0.12,St=0.28,
	weighting="None"):
	
	k2 = 1.206e-2 * ureg.s**3/ureg.ft**3
	pi = math.pi

	V_07 = 0.7*VT
	A = pi*(R**2) #rotor disk area
	c = (pi*s*R)/B #Rotor blade chord
	alpha = Cl_mean/(2*pi) #Angle of attack (average)
	t_proj = t_c*c*np.cos(alpha) + c*np.sin(alpha) #blade projected thickness
	
	f_peak = (St*V_07/t_proj)*ureg.turn #peak frequency 
	f_peak = f_peak.to(ureg.rad/ureg.s) #convert to rad/s

	atmospheric_data = stdatmo(h)
	rho = atmospheric_data["\rho"].to(ureg.kg/ureg.m**3)
	
	p_ratio = k2*(VT/(rho*delta_S))*np.sqrt((T_perRotor*N/s)*(T_perRotor/A))
	SPL = 20*np.log10(p_ratio)

	spectrum = {}
	spectrum["f"] = f_peak*[0.5,1,2,4,8,16]
	offsets_dB = [7.92,4.17,8.33,8.75,12.92,13.33]

	spectrum["SPL"] = SPL*np.ones(np.shape(offsets_dB)) - offsets_dB

	if weighting == "A":
		
		#Apply A-weighting to the spectrum
		spectrum["SPL"] = noise_weighting(spectrum["f"],spectrum["SPL"],type="A")

		fr = (spectrum["f"]/f_peak).to(ureg.dimensionless) #frequency ratio array
		
		weighted_p_ratio_squared = 0
		for i in range(0,np.size(fr)-1):
			fr1 = fr[i]
			fr2 = fr[i+1]
			SPL1 = spectrum["SPL"][i]
			SPL2 = spectrum["SPL"][i+1]

			a = (SPL2-SPL1)/(np.log10(fr2)-np.log10(fr1))
			b = SPL2 - a*np.log10(fr2)

			leading_term = (10**(b/10))/((a/10) + 1)
			fr_term = fr2**((a/10) + 1) - fr1**((a/10) + 1)

			weighted_p_ratio_squared += leading_term*fr_term

		SPL = 10*np.log10(weighted_p_ratio_squared)

	return f_peak, SPL, spectrum

def noise_weighting(f,SPL,type="A"):
	#Noise weighting function. Currently, only A-weighting is implemented.
	if type == "A":
		f = f.to(ureg.turn/ureg.s).magnitude
		numerator = 12194**2*f**4
		denominator = (f**2+20.6**2)*(f**2+12194**2)*np.sqrt((f**2+107.7**2)*(f**2+737.9**2))
		R = numerator/denominator
		A = 20*np.log10(R) + 2.00

	dBA = SPL + A
	return dBA


if __name__=="__main__":
	
	configs = configuration_data.copy()
	config = "Lift + cruise"

	print "Solving configuration: " + config

	#General data
	eta_cruise = generic_data["\eta_{cruise}"] 
	eta_electric = generic_data["\eta_{electric}"]
	weight_fraction = generic_data["weight_fraction"]
	C_m = generic_data["C_m"]
	n = generic_data["n"]
	B = generic_data["B"]

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
	Q_perRotor = solution("Q_perRotor_OnDemandSizingMission")[0]
	R = solution("R")
	VT = solution("VT_OnDemandSizingMission")[0]
	s = solution("s")
	Cl_mean = solution("Cl_{mean_{max}}")
	N = solution("N")
	theta = 91.*ureg.degree
	delta_S = 500*ureg.ft
	weighting = "A"
	
	noise = {}
	noise["rotational"] = {}
	noise["vortex"] = {}

	noise["rotational"]["f_fund"], noise["rotational"]["SPL"], noise["rotational"]["spectrum"]\
		= rotational_noise(T_perRotor,Q_perRotor,R,VT,s,N,B,theta=theta,delta_S=delta_S,
			h=0*ureg.ft,t_c=0.12,num_harmonics=10,weighting=weighting)

	noise["vortex"]["f_peak"], noise["vortex"]["SPL"], noise["vortex"]["spectrum"]\
		= vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,Cl_mean=Cl_mean,N=N,B=B,
			delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,weighting=weighting)

	noise["rotational"]["dBA_offset"] = noise_weighting(noise["rotational"]["f_fund"],0)
	noise["vortex"]["dBA_offset"] = noise_weighting(noise["vortex"]["f_peak"],0)

	print "%0.0f blades; theta = %0.1f degrees; weighting = %s" \
		% (B, theta.to(ureg.degree).magnitude, weighting)
	print 
	print "Noise Type \t\tRotational \tVortex"
	print "Peak Frequency (Hz)\t%0.1f\t\t%0.1f" % \
		(noise["rotational"]["f_fund"].to(ureg.turn/ureg.s).magnitude,noise["vortex"]["f_peak"].to(ureg.turn/ureg.s).magnitude)
	print "SPL (dB)\t\t%0.1f\t\t%0.1f" % \
		(noise["rotational"]["SPL"],noise["vortex"]["SPL"])
	print "dBA offset at peak:\t%0.1f\t\t%0.1f" % \
		(noise["rotational"]["dBA_offset"],noise["vortex"]["dBA_offset"])
