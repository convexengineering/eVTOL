# Aircraft models.

import math
pi = math.pi
import numpy as np
from gpkit                  import Variable, Model, Vectorize, ureg
from standard_substitutions import on_demand_aircraft_substitutions

class OnDemandAircraft(Model):
	
	def hover_performance(self, state):
		return OnDemandAircraftHoverPerformance(self, state)

	def level_flight_performance(self, state):
		return OnDemandAircraftLevelFlightFlightPerformance(self, state)

	def standard_substitutions(self, config="Lift + cruise", autonomousEnabled=True):
		return on_demand_aircraft_substitutions(aircraft=self, config=config, autonomousEnabled=autonomousEnabled)

	def setup(self):

		self.airframe          = airframe          = Airframe()
		self.avionics          = avionics          = Avionics()
		self.battery           = battery           = Battery()
		self.rotors            = rotors            = Rotors()
		self.electrical_system = electrical_system = ElectricalSystem()

		self.components      = components      = [airframe, avionics, battery, rotors, electrical_system]
		self.mass_components = mass_components = [airframe,           battery]
		self.cost_components = cost_components = [airframe, avionics, battery]
		
		self.MTOM = MTOM = Variable("MTOM", "kg",    "Aircraft maximum takeoff mass")
		self.MTOW = MTOW = Variable("MTOW", "N",     "Aircraft maximum takeoff weight")
		self.g    = g    = Variable("g",    "m/s^2", "Gravitational acceleration")

		self.empty_mass_fraction = empty_mass_fraction = Variable("empty_mass_fraction", "-", "Empty mass fraction")

		self.m_noPassengersOrCrew = m_noPassengersOrCrew = Variable("m_{noPassengersOrCrew}", "kg", "Aircraft mass without passengers or crew")
		self.W_noPassengersOrCrew = W_noPassengersOrCrew = Variable("W_{noPassengersOrCrew}", "N",  "Aircraft weight without passengers or crew")
		
		self.v_cruise = v_cruise = Variable("v_{cruise}", "m/s", "Cruise speed")
		self.v_loiter = v_loiter = Variable("v_{loiter}", "m/s", "Loiter speed")

		self.L_D_cruise = L_D_cruise = Variable("(L/D)_{cruise}", "-", "Cruise L/D ratio")
		self.L_D_loiter = L_D_loiter = Variable("(L/D)_{loiter}", "-", "Loiter L/D ratio")

		self.eta_levelFlight = eta_levelFlight = Variable("\eta_{levelFlight}",  "-", "Level-flight propulsive efficiency (proeller efficiency)")

		tailRotor_power_fraction_hover       = Variable("tailRotor_power_fraction_hover",       "-", "Tail-rotor power as a fraction of lifting-rotors power (hover)")
		tailRotor_power_fraction_levelFlight = Variable("tailRotor_power_fraction_levelFlight", "-", "Tail-rotor power as a fraction of lifting-rotors power (level flight)")

		self.tailRotor_power_fraction_hover       = tailRotor_power_fraction_hover
		self.tailRotor_power_fraction_levelFlight = tailRotor_power_fraction_levelFlight

		constraints =  [self.components]
		constraints += [c.W                  == g * c.m for c in mass_components]   # Avoids the need to include this in each sub-model
		constraints += [m_noPassengersOrCrew >= sum(c.m for c in mass_components)]  # Mass summation

		constraints += [
			MTOW                 == g * MTOM,
			W_noPassengersOrCrew == g * m_noPassengersOrCrew,

			airframe.m == empty_mass_fraction * MTOM,

			v_loiter   == ((1./3.)**(1./4.)) * v_cruise,    # Approximation for loiter speed.        See derivation in publications.
			L_D_loiter == ((3.**0.5)/2.)     * L_D_cruise,  # Approximation for loiter lift-to-drag. See derivation in publications.
		]

		return constraints


class OnDemandAircraftHoverPerformance(Model):

	def setup(self, aircraft, state):

		MTOM = aircraft.MTOM
		g    = aircraft.g

		tailRotor_power_fraction = aircraft.tailRotor_power_fraction_hover

		L                     = state.L
		T                     = state.T
		P_electric            = state.P_electric
		P_shaft               = state.P_shaft
		P_shaft_liftingRotors = state.P_shaft_liftingRotors
		P_shaft_tailRotor     = state.P_shaft_tailRotor

		# Component performance models
		self.battery_perf    = battery_perf    = aircraft.battery.performance()
		self.rotors_perf     = rotors_perf     = aircraft.rotors.performance(state)
		self.electrical_perf = electrical_perf = aircraft.electrical_system.performance()
		self.perf_models     = perf_models     = [battery_perf, rotors_perf, electrical_perf]

		self.m = m = Variable("m", "kg", "Aircraft mass during segment")
		self.W = W = Variable("W", "N",  "Aircraft weight during segment")

		constraints = [perf_models]

		constraints += [
			W == m * g,
			m <= MTOM,		

			P_electric == battery_perf.P,
			P_electric == electrical_perf.P_in,

			P_shaft               == electrical_perf.P_out,
			P_shaft               >= P_shaft_liftingRotors + P_shaft_tailRotor,
			P_shaft_tailRotor     == tailRotor_power_fraction * P_shaft,
			P_shaft_liftingRotors == rotors_perf.P,

			T == rotors_perf.T,
		]

		return constraints


class OnDemandAircraftLevelFlightFlightPerformance(Model):

	def setup(self, aircraft, state):

		MTOM = aircraft.MTOM
		g    = aircraft.g

		eta_levelFlight          = aircraft.eta_levelFlight
		tailRotor_power_fraction = aircraft.tailRotor_power_fraction_levelFlight

		v                 = state.v
		L                 = state.L
		T                 = state.T
		D                 = state.D
		L_D               = state.L_D
		P_electric        = state.P_electric
		P_shaft           = state.P_shaft
		P_shaft_thrust    = state.P_shaft_thrust
		P_shaft_tailRotor = state.P_shaft_tailRotor

		# Component performance models
		self.battery_perf    = battery_perf    = aircraft.battery.performance()
		self.electrical_perf = electrical_perf = aircraft.electrical_system.performance()
		self.perf_models     = perf_models     = [battery_perf, electrical_perf]

		self.m = m = Variable("m", "kg", "Aircraft mass during segment")
		self.W = W = Variable("W", "N",  "Aircraft weight during segment")

		constraints = [perf_models]

		constraints += [
			W == m * g,
			m <= MTOM,

			P_electric == battery_perf.P,
			P_electric == electrical_perf.P_in,

			P_shaft               == electrical_perf.P_out,
			P_shaft               >= P_shaft_thrust + P_shaft_tailRotor,
			P_shaft_tailRotor     == tailRotor_power_fraction * P_shaft,

			T * v == eta_levelFlight * P_shaft_thrust,

		]

		return constraints


class Airframe(Model):
	
	def setup(self):
		
		self.m = m = Variable("m", "kg", "Airframe mass")
		self.W = W = Variable("W", "N",  "Airframe weight")
		
		self.cost_per_mass   = cost_per_mass   = Variable("cost_per_mass",   "kg^-1", "Cost per unit airframe mass")
		self.cost_per_weight = cost_per_weight = Variable("cost_per_weight", "N^-1",  "Cost per unit airframe weight")

		self.purchase_price = purchase_price = Variable("purchase_price", "-",     "Airframe purchase price")
		self.lifetime       = lifetime       = Variable("lifetime",       "hours", "Airframe lifetime")

		constraints = [
			purchase_price == cost_per_mass   * m,
			purchase_price == cost_per_weight * W,

		]

		return constraints


class Avionics(Model):
	
	def setup(self):

		self.purchase_price = purchase_price = Variable("purchase_price", "-",     "Avionics purchase price")
		self.lifetime       = lifetime       = Variable("lifetime",       "hours", "Avionics lifetime")
		
		return []


class Battery(Model):

	def performance(self):
		return BatteryPerformance(self)

	def setup(self):
				
		self.E      = E      = Variable("E",        "kWh",   "Battery energy capacity")
		self.E_eff  = E_eff  = Variable("E_{eff}",  "kWh",   "Effective battery energy capacity")
		self.E_frac = E_frac = Variable("E_{frac}", "-",     "Percentage of the battery energy that can be used without damaging battery")
		self.P_max  = P_max  = Variable("P_{max}",  "kW",    "Battery maximum power")
		self.e      = e      = Variable("e",        "Wh/kg", "Battery specific energy (energy density)")
		self.p      = p      = Variable("p",        "W/kg",  "Battery specific power (power density)")

		self.cost_per_E     = cost_per_E     = Variable("cost_per_E",     "kWh**-1", "Battery cost per unit energy stored")
		self.cost_per_m     = cost_per_m     = Variable("cost_per_m",     "kg**-1",  "Battery cost per unit mass")
		self.cost_per_W     = cost_per_W     = Variable("cost_per_W",     "N**-1",   "Battery cost per unit weight")
		self.purchase_price = purchase_price = Variable("purchase_price", "-",       "Purchase price of the battery")
		self.cycle_life     = cycle_life     = Variable("cycle_life",     "-",       "Number of cycles before battery needs replacement")

		self.m = m = Variable("m", "kg", "Battery mass")
		self.W = W = Variable("W", "N",  "Battery weight")

		constraints = [
			E     == m * e,
			P_max == m * p,
			E_eff == E_frac * E,

			purchase_price == cost_per_E * E,
			purchase_price == cost_per_m * m,
			purchase_price == cost_per_W * W,
		]

		return constraints


class BatteryPerformance(Model):
	
	def setup(self, battery):

		E_eff = battery.E_eff
		P_max = battery.P_max
		
		self.E = E = Variable("E", "kWh", "Segment electrical energy used")
		self.P = P = Variable("P", "kW",  "Segment electrical power draw")
		self.t = t = Variable("t", "s",   "Segment time")

		constraints = [
			E == P * t, 
			E <= E_eff,
			P <= P_max,
		]
		
		return constraints


class Rotors(Model):

	def performance(self, state):
		return RotorsPerformance(self, state)

	def setup(self):

		self.R       = R       = Variable("R",            "m",   "Rotor radius")
		self.D       = D       = Variable("D",            "m",   "Rotor diameter")
		self.c_avg   = c_avg   = Variable("\overline{c}", "m",   "Rotor average blade chord")
		self.A       = A       = Variable("A",            "m^2", "Area of 1 rotor disk")
		self.A_blade = A_blade = Variable("A_{blade}",    "m^2", "Area of 1 set of rotor blades")
		self.A_total = A_total = Variable("A_{total}",    "m^2", "Combined area of all rotor disks")
		self.N       = N       = Variable("N",            "-",   "Number of rotors")
		self.B       = B       = Variable("B",            "-",   "Number of rotor blades")
		self.AR      = AR      = Variable("AR",           "-",   "Rotor blade aspect ratio")
		self.s       = s       = Variable("s",            "-",   "Rotor solidity")
		
		self.ki          = ki          = Variable("ki",            "-",     "Rotor induced power factor")
		self.Cd0         = Cd0         = Variable("Cd0",           "-",     "Rotor blade two-dimensional zero-lift drag coefficient")
		self.T_A_max     = T_A_max     = Variable("(T/A)_{max}",   "N/m^2", "Rotor maximum allowed disk loading")
		self.M_tip_max   = M_tip_max   = Variable("M_{tip,max}",   "-",     "Rotor maximum allowed tip Mach number")
		self.Cl_mean_max = Cl_mean_max = Variable("Cl_{mean,max}", "-",     "Rotor maximum allowed mean lift coefficient")

		constraints = [
			A       == pi * R**2., 
			A_blade == B  * c_avg * R,
			A_blade == s  * A,
			D       == 2. * R,
			AR      == R  / c_avg,
			A_total == N  * A,
		]

		return constraints


class RotorsPerformance(Model):
	
	def setup(self, rotors, state):

		R       = rotors.R
		A       = rotors.A
		A_total = rotors.A_total
		N       = rotors.N
		s       = rotors.s
		
		ki          = rotors.ki
		Cd0         = rotors.Cd0
		T_A_max     = rotors.T_A_max
		M_tip_max   = rotors.M_tip_max
		Cl_mean_max = rotors.Cl_mean_max

		rho = state.atmosphere.rho
		a   = state.atmosphere.a
		
		self.T          = T          = Variable("T",          "N",     "Total thrust")
		self.T_perRotor = T_perRotor = Variable("T_perRotor", "N",     "Thrust per rotor")
		self.T_A        = T_A        = Variable("T/A",        "N/m^2", "Disk loading")
		self.P          = P          = Variable("P",          "kW",    "Total power")
		self.P_perRotor = P_perRotor = Variable("P_perRotor", "kW",    "Power per rotor")
		self.Q_perRotor = Q_perRotor = Variable("Q_perRotor", "N*m",   "Torque per rotor")
		self.V_tip      = V_tip      = Variable("V_{tip}",    "m/s",   "Rotor tip speed")
		self.omega      = omega      = Variable("\\omega",    "rpm",   "Rotor angular velocity")
		self.M_tip      = M_tip      = Variable("M_{tip}",    "-",     "Rotor tip Mach number")

		self.CT      = CT      = Variable("CT",        "-", "Thrust coefficient")
		self.CQ      = CQ      = Variable("CQ",        "-", "Torque coefficient")
		self.CP      = CP      = Variable("CP",        "-", "Power coefficient")
		self.CPi     = CPi     = Variable("CPi",       "-", "Induced (ideal) power coefficient")
		self.CPp     = CPp     = Variable("CPp",       "-", "Profile power coefficient")
		self.Cl_mean = Cl_mean = Variable("Cl_{mean}", "-", "Mean lift coefficient")
		self.FOM     = FOM     = Variable("FOM",       "-", "Figure of merit")

		constraints = [state]

		constraints += [
			T == N * T_perRotor,
			P == N * P_perRotor,

			T_perRotor == 0.5 * rho * (V_tip**2.) * A * CT,
			P_perRotor == 0.5 * rho * (V_tip**3.) * A * CP,
			Q_perRotor == 0.5 * rho * (V_tip**2.) * A * R * CQ,

			CPi == 0.5  * CT**1.5,
			CPp == 0.25 * s * Cd0,
			CP  >= ki*CPi + CPp,
			FOM == CPi / CP,
			CQ  == CP,

			V_tip == omega * R,
			V_tip == M_tip * a,
			M_tip <= M_tip_max,

			T_A == T_perRotor / A,
			T_A <= T_A_max,

			Cl_mean == 3. * CT / s,
			Cl_mean <= Cl_mean_max,
		]

		return constraints


class ElectricalSystem(Model):
	
	def performance(self):
		return ElectricalSystemPerformance(self)

	def setup(self):
		
		self.eta = eta = Variable("\eta", "-", "Electrical efficiency")
		return []


class ElectricalSystemPerformance(Model):
	
	def setup(self, electrical_system):

		eta = electrical_system.eta
		
		self.P_in  = P_in  = Variable("P_{in}",  "kW", "Input electrical power (from the battery)")
		self.P_out = P_out = Variable("P_{out}", "kW", "Output shaft power (to the rotors)")
		
		constraints = [P_out == eta * P_in]
		
		return constraints


if __name__=="__main__":

	test_aircraft = OnDemandAircraft()
	test_aircraft = test_aircraft.standard_substitutions(config="Lift + cruise", autonomousEnabled=True)