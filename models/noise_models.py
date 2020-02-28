# Functions for rotor noise prediction
from __future__ import print_function
import numpy as np
import math
pi = math.pi

from scipy.special       import jv
from gpkit               import Model, ureg
from matplotlib          import pyplot as plt
from aircraft_models     import OnDemandAircraft
from mission_models      import OnDemandSizingMission, OnDemandRevenueMission, OnDemandDeadheadMission
from cost_models         import OnDemandMissionCost
from standard_atmosphere import stdatmo


def rotational_noise(T_perRotor, Q_perRotor, R, omega, c_avg, t_avg, N, B, rho, a, theta=175*ureg.degree, delta_S=500*ureg.ft, num_harmonics=10, weighting="None"):

	P_ref = 2e-5 * ureg.Pa  # Reference pressure
	R_eff = 0.8  * R        # Effective rotor radius

	omega = omega.to(ureg.radian / ureg.s)

	spectrum = {}
	spectrum["m"]   = range(1, num_harmonics+1, 1)
	spectrum["f"]   = np.zeros(num_harmonics) * ureg.rad/ureg.s
	spectrum["SPL"] = np.zeros(num_harmonics)

	p_ratio_squared = np.zeros(num_harmonics)

	#Compute unweighted spectrum
	for i, m in enumerate(spectrum["m"]):
		spectrum["f"][i] = m*B*omega

		bessel_argument = ((m*B*omega/a) * R_eff * np.sin(theta)).to(ureg.dimensionless)
		bessel_term     = jv(m*B, bessel_argument)

		#RMS acoustic pressures
		P_mL = ((m*B*omega)/(2*np.sqrt(2)*pi*a*delta_S)) * (T_perRotor*np.cos(theta) - (Q_perRotor*a)/(omega*R_eff**2)) * bessel_term  # Loading pressure
		P_mT = ((-rho * ((m*B*omega)**2) * B)/(3*np.sqrt(2)*pi*delta_S)) * c_avg * t_avg * R_eff * bessel_term                         # Thickness pressure

		p_ratio_squared[i] = N   * ((P_mL/P_ref)**2 + (P_mT/P_ref)**2)
		spectrum["SPL"][i] = 10. * np.log10(p_ratio_squared[i])

	#Apply weighting schemes
	if weighting == "None":
		pass
	elif weighting == "A":
		spectrum["SPL"] = noise_weighting(spectrum["f"], spectrum["SPL"], weighting="A")
	else:
		error_string = "Noise weighting scheme " + weighting + " not recognized."
		raise AttributeError(error_string)

	#Calculate overall SPL
	p_ratio_squared_sum = 0
	for i,SPL in enumerate(spectrum["SPL"]):
		p_ratio_squared_sum += 10**(SPL/10)

	SPL = 10*np.log10(p_ratio_squared_sum)

	f_fundamental = spectrum["f"][0]
	return f_fundamental, SPL, spectrum


def vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S=500*ureg.ft, St=0.28, weighting="None"):

	k2 = 1.206e-2 * ureg.s**3/ureg.ft**3

	V_07   = 0.7 * V_tip
	alpha  = Cl_mean / (2*pi)                           # Angle of attack (average)
	t_proj = t_avg*np.cos(alpha) + c_avg*np.sin(alpha)  # Blade projected thickness

	f_peak = (St*V_07/t_proj) * ureg.turn  # Peak frequency
	f_peak = f_peak.to(ureg.rad/ureg.s)    # Convert to rad/s

	p_ratio = k2 * (V_tip/(rho*delta_S))*np.sqrt((T_perRotor*N/s)*(T_A))
	SPL     = 20 * np.log10(p_ratio)

	spectrum = {}
	spectrum["f"] = f_peak * [0.5,  1,    2,    4,    8,     16   ]
	offsets_dB    =          [7.92, 4.17, 8.33, 8.75, 12.92, 13.33]

	spectrum["SPL"] = SPL*np.ones(np.shape(offsets_dB)) - offsets_dB

	if weighting == "A":

		#Apply A-weighting to the spectrum
		spectrum["SPL"] = noise_weighting(spectrum["f"], spectrum["SPL"], weighting="A")

		fr = (spectrum["f"]/f_peak).to(ureg.dimensionless)  # Frequency ratio array

		# Interpolate in log-space
		weighted_p_ratio_squared = 0
		for i in range(0,np.size(fr)-1):
			fr1 = fr[i]
			fr2 = fr[i+1]
			SPL1 = spectrum["SPL"][i]
			SPL2 = spectrum["SPL"][i+1]

			a = (SPL2-SPL1)/(np.log10(fr2)-np.log10(fr1))
			b = SPL2 - a*np.log10(fr2)

			leading_term = (10**(b/10))/((a/10) + 1)
			fr_term      = fr2**((a/10) + 1) - fr1**((a/10) + 1)

			weighted_p_ratio_squared += leading_term*fr_term

		SPL = 10*np.log10(weighted_p_ratio_squared)

	return f_peak, SPL, spectrum


def noise_weighting(f, SPL, weighting="A"):

	# Noise weighting function. Currently, only A-weighting is implemented.
	if weighting == "A":
		f           = f.to(ureg.turn/ureg.s).magnitude
		numerator   = 12194**2*f**4
		denominator = (f**2+20.6**2)*(f**2+12194**2)*np.sqrt((f**2+107.7**2)*(f**2+737.9**2))
		R           = numerator / denominator
		weight      = 20*np.log10(R) + 2.00

	else:
		error_string = "Noise weighting scheme " + weighting + " not recognized."
		raise AttributeError(error_string)

	dBA = SPL + weight
	return dBA


if __name__=="__main__":

	config = "Lift + cruise"
	print()
	print("Solving configuration: " + config)

	aircraft = OnDemandAircraft()
	aircraft = aircraft.standard_substitutions(config="Compound heli", autonomousEnabled=True)

	sizing_mission = OnDemandSizingMission(aircraft=aircraft)
	sizing_mission = sizing_mission.standard_substitutions(piloted=True, reserve="20-minute loiter")

	revenue_mission = OnDemandRevenueMission(aircraft=aircraft)
	revenue_mission = revenue_mission.standard_substitutions(piloted=True)

	deadhead_mission = OnDemandDeadheadMission(aircraft=aircraft)
	deadhead_mission = deadhead_mission.standard_substitutions(piloted=False)

	mission_cost = OnDemandMissionCost(aircraft=aircraft, revenue_mission=revenue_mission, deadhead_mission=deadhead_mission)
	mission_cost = mission_cost.standard_substitutions(isRevenueMissionPiloted=True, isDeadheadMissionPiloted=False)

	objective_function = mission_cost.cpt
	problem            = Model(objective_function, [aircraft, sizing_mission, revenue_mission, deadhead_mission, mission_cost])
	solution           = problem.solve(verbosity=0)

	T_perRotor = solution("T_perRotor_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	Q_perRotor = solution("Q_perRotor_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	T_A        = solution("T/A_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	Cl_mean    = solution("Cl_{mean}_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	R          = solution("R_OnDemandAircraft/Rotors")
	s          = solution("s_OnDemandAircraft/Rotors")
	omega      = solution("\\omega_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	V_tip      = solution("V_{tip}_OnDemandSizingMission/HoverTakeoff/OnDemandAircraftHoverPerformance/RotorsPerformance")
	c_avg      = solution("c_{avg}_OnDemandAircraft/Rotors")
	t_avg      = solution("t_{avg}_OnDemandAircraft/Rotors")
	N          = solution("N_OnDemandAircraft/Rotors")
	B          = solution("B_OnDemandAircraft/Rotors")
	rho        = solution("\\rho_OnDemandSizingMission/HoverTakeoff/HoverFlightState/FixedStandardAtmosphere")
	a          = solution("a_OnDemandSizingMission/HoverTakeoff/HoverFlightState/FixedStandardAtmosphere")

	theta     = 91. * ureg.degree
	delta_S   = 500 * ureg.ft
	weighting = "A"

	noise               = {}
	noise["rotational"] = {}
	noise["vortex"]     = {}

	rotational_data = rotational_noise(T_perRotor, Q_perRotor, R, omega, c_avg, t_avg, N, B, rho, a, theta, delta_S, num_harmonics=10, weighting=weighting)
	[noise["rotational"]["f_fund"], noise["rotational"]["SPL"], noise["rotational"]["spectrum"]] = rotational_data

	vortex_data = vortex_noise(T_perRotor, T_A, V_tip, s, Cl_mean, N, c_avg, t_avg, rho, delta_S, St=0.28, weighting=weighting)
	[noise["vortex"]["f_peak"], noise["vortex"]["SPL"], noise["vortex"]["spectrum"]] = vortex_data

	noise["rotational"]["dBA_offset"] = noise_weighting(noise["rotational"]["f_fund"], 0)  # Noise offsets. How much does the weighting scheme affect the results?
	noise["vortex"]["dBA_offset"]     = noise_weighting(noise["vortex"]["f_peak"],     0)  # Noise offsets. How much does the weighting scheme affect the results?

	print("%0.0f blades; theta = %0.1f degrees; weighting = %s" % (B, theta.to(ureg.degree).magnitude, weighting))
	print()
	print("Noise Type          \tRotational \tVortex")
	print("Peak Frequency (Hz) \t%0.1f      \t%0.1f" % (noise["rotational"]["f_fund"].to(ureg.turn/ureg.s).magnitude, noise["vortex"]["f_peak"].to(ureg.turn/ureg.s).magnitude))
	print("SPL (dB)            \t%0.1f      \t%0.1f" % (noise["rotational"]["SPL"],                                   noise["vortex"]["SPL"]))
	print("dBA offset at peak: \t%0.1f      \t%0.1f" % (noise["rotational"]["dBA_offset"],                            noise["vortex"]["dBA_offset"]))
