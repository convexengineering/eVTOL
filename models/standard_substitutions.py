# Substitution functions, for both generic and configuration-specific inputs

import numpy as np
from gpkit import ureg

def on_demand_aircraft_substitutions(aircraft, config="Lift + cruise", autonomousEnabled=False):

	aircraft.substitutions.update({
		
		aircraft.g:               9.807 * ureg.m / ureg.s**2,
		aircraft.eta_levelFlight: 0.85,                        # Propulsive efficiency in level flight
		
		aircraft.airframe.cost_per_weight: 350.   * ureg.lbf**-1,
		aircraft.airframe.lifetime:        20000. * ureg.hour,

		aircraft.avionics.lifetime:  20000 * ureg.hour,
		
		aircraft.battery.E_frac:     0.8,
		aircraft.battery.e:          400. * ureg.Wh / ureg.kg,
		aircraft.battery.p:          3.   * ureg.kW / ureg.kg,
		aircraft.battery.cost_per_E: 400. * ureg.kWh**-1,
		aircraft.battery.cycle_life: 2000.,

		aircraft.rotors.B:      5.,   # Number of rotor blades
		aircraft.rotors.s:      0.1,
		aircraft.rotors.ki:     1.2, 
		aircraft.rotors.Cd0:    0.01,
		aircraft.rotors.MT_max: 0.9,

		aircraft.electrical_system.eta: 0.9,
	})

	
	if autonomousEnabled:
		aircraft.substitutions.update({
			aircraft.avionics.purchase_price: 60000.,
		})
	else:
		aircraft.substitutions.update({
			aircraft.avionics.purchase_price: 1.,  # Negligibly small
		})

	if config == "Multirotor":
		aircraft.substitutions.update({
			aircraft.empty_mass_fraction: 0.43,
			aircraft.v_cruise:            50. * ureg.mph,
			aircraft.L_D_cruise:          1.5,

			aircraft.tailRotor_power_fraction_hover:       0.002,
			aircraft.tailRotor_power_fraction_levelFlight: 0.002,

			aircraft.rotors.N:           8.,
			aircraft.rotors.T_A_max:     3.75 * ureg.lbf / ureg.ft**2,
			aircraft.rotors.Cl_mean_max: 0.6,
		})

	elif config == "Autogyro":
		aircraft.substitutions.update({
			aircraft.empty_mass_fraction: 0.5,
			aircraft.v_cruise:            100. * ureg.mph,
			aircraft.L_D_cruise:          3.5,

			aircraft.tailRotor_power_fraction_hover:       0.002,
			aircraft.tailRotor_power_fraction_levelFlight: 0.002,

			aircraft.rotors.N:           1,
			aircraft.rotors.T_A_max:     3.75 * ureg.lbf / ureg.ft**2,
			aircraft.rotors.Cl_mean_max: 0.8,
		})

	elif config == "Helicopter":
		aircraft.substitutions.update({
			aircraft.empty_mass_fraction: 0.43,
			aircraft.v_cruise:            100. * ureg.mph,
			aircraft.L_D_cruise:          4.25,

			aircraft.tailRotor_power_fraction_hover:       0.15,
			aircraft.tailRotor_power_fraction_levelFlight: 0.15,

			aircraft.rotors.N:           1.,
			aircraft.rotors.T_A_max:     4.5 * ureg.lbf / ureg.ft**2,
			aircraft.rotors.Cl_mean_max: 0.6,
		})

	elif config == "Tilt duct":
		aircraft.substitutions.update({
			aircraft.empty_mass_fraction: 0.55,
			aircraft.v_cruise:            150. * ureg.mph,
			aircraft.L_D_cruise:          10.,

			aircraft.tailRotor_power_fraction_hover:       0.002,
			aircraft.tailRotor_power_fraction_levelFlight: 0.002,

			aircraft.rotors.N:           36.,
			aircraft.rotors.T_A_max:     40. * ureg.lbf / ureg.ft**2,
			aircraft.rotors.Cl_mean_max: 1.0,
		})

	elif config == "Coaxial heli":
		aircraft.substitutions.update({
			aircraft.empty_mass_fraction: 0.43,
			aircraft.v_cruise:            150. * ureg.mph,
			aircraft.L_D_cruise:          5.5,

			aircraft.tailRotor_power_fraction_hover:       0.002,
			aircraft.tailRotor_power_fraction_levelFlight: 0.002,

			aircraft.rotors.N:           2.0,
			aircraft.rotors.T_A_max:     7. * ureg.lbf / ureg.ft**2,
			aircraft.rotors.Cl_mean_max: 0.6,
		})

	elif config == "Lift + cruise":
		aircraft.substitutions.update({
			aircraft.empty_mass_fraction: 0.53,
			aircraft.v_cruise:            150. * ureg.mph,
			aircraft.L_D_cruise:          10.,

			aircraft.tailRotor_power_fraction_hover:       0.002,
			aircraft.tailRotor_power_fraction_levelFlight: 0.002,

			aircraft.rotors.N:           8.,
			aircraft.rotors.T_A_max:     15. * ureg.lbf / ureg.ft**2,
			aircraft.rotors.Cl_mean_max: 1.0,
		})

	elif config == "Tilt wing":
		aircraft.substitutions.update({
			aircraft.empty_mass_fraction: 0.55,
			aircraft.v_cruise:            150. * ureg.mph,
			aircraft.L_D_cruise:          12.,

			aircraft.tailRotor_power_fraction_hover:       0.002,
			aircraft.tailRotor_power_fraction_levelFlight: 0.002,

			aircraft.rotors.N:           8.,
			aircraft.rotors.T_A_max:     15. * ureg.lbf / ureg.ft**2,
			aircraft.rotors.Cl_mean_max: 1.0,
		})

	elif config == "Compound heli":
		aircraft.substitutions.update({
			aircraft.empty_mass_fraction: 0.5,
			aircraft.v_cruise:            150. * ureg.mph,
			aircraft.L_D_cruise:          9.,

			aircraft.tailRotor_power_fraction_hover:       0.15,
			aircraft.tailRotor_power_fraction_levelFlight: 0.10,

			aircraft.rotors.N:           1.,
			aircraft.rotors.T_A_max:     4.5 * ureg.lbf / ureg.ft**2,
			aircraft.rotors.Cl_mean_max: 0.8,
		})

	elif config == "Tilt rotor":
		aircraft.substitutions.update({
			aircraft.empty_mass_fraction: 0.55,
			aircraft.v_cruise:            150. * ureg.mph,
			aircraft.L_D_cruise:          14.,

			aircraft.tailRotor_power_fraction_hover:       0.002,
			aircraft.tailRotor_power_fraction_levelFlight: 0.002,

			aircraft.rotors.N:           12.,
			aircraft.rotors.T_A_max:     15. * ureg.lbf / ureg.ft**2,
			aircraft.rotors.Cl_mean_max: 1.0,
		})
	
	else:
		error_string = "Configuration " + config + " not recognized."
		raise ValueError(error_string)


	return aircraft

def on_demand_sizing_mission_substitutions(mission, autonomous=False, reserve="20-minute loiter"):
	
	mission.substitutions.update({
		mission.crew.W_unit:       190. * ureg.lbf,
		mission.passengers.W_unit: 200. * ureg.lbf,
		mission.passengers.N:      3.0,

		mission.takeoff_segment.t_segment: 2.0 * ureg.minute,
		mission.cruise_segment.d_segment:  50. * ureg.nautical_mile,
		mission.landing_segment.t_segment: 2.0 * ureg.minute,
	})
	
	if autonomous:
		mission.substitutions.update({
			mission.crew.N:  0.001,  # Negligibly small
		})

	else:
		mission.substitutions.update({
			mission.crew.N:  1,
		})


	if reserve == "20-minute loiter":
		mission.substitutions.update({
			mission.loiter_segment.t_segment: 20. * ureg.minute,
		})

	elif reserve == "30-minute loiter":
		mission.substitutions.update({
			mission.loiter_segment.t_segment: 30. * ureg.minute,
		})

	elif reserve == "2-nmi diversion":
		mission.substitutions.update({
			mission.loiter_segment.d_segment: 2. * ureg.nautical_mile,
		})

	else:
		error_string = "Reserve type " + reserve + " not recognized."
		raise ValueError(error_string)

	return mission