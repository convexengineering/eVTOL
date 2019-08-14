# Mission models.
import numpy as np
import math
pi = math.pi

from gpkit                  import Variable, Model, Vectorize, ureg
from aircraft_models        import OnDemandAircraft
from standard_atmosphere    import stdatmo
from standard_substitutions import on_demand_sizing_mission_substitutions, on_demand_revenue_mission_substitutions, on_demand_deadhead_mission_substitutions

# from noise_models import rotational_noise, vortex_noise, noise_weighting

class FixedStandardAtmosphere(Model):
	
	def setup(self, h):
		
		atmospheric_data = stdatmo(h)
		
		rho = atmospheric_data["\rho"].to(ureg.kg / ureg.m**3.).magnitude
		a   = atmospheric_data["a"].to(ureg.m / ureg.s).magnitude
		
		self.rho = rho = Variable("\\rho", rho, "kg/m^3", "Air density")
		self.a   = a   = Variable("a",     a,   "m/s",    "Speed of sound")

		constraints = []
		
		return constraints


class HoverFlightState(Model):
	
	def setup(self, h=0*ureg.m):

		self.L                     = L                     = Variable("L",                       "N",  "Lift")
		self.W                     = W                     = Variable("W",                       "N",  "Vehicle weight")
		self.T                     = T                     = Variable("T",                       "N",  "Thrust")
		self.P_electric            = P_electric            = Variable("P_{electric}",            "kW", "Electrical power draw (from the battery)")
		self.P_shaft               = P_shaft               = Variable("P_{shaft}",               "kW", "Shaft power (to all rotors)")
		self.P_shaft_liftingRotors = P_shaft_liftingRotors = Variable("P_{shaft,liftingRotors}", "kW", "Shaft power (to the lifting rotors)")
		self.P_shaft_tailRotor     = P_shaft_tailRotor     = Variable("P_{shaft,tailRotor}",     "kW", "Shaft power (to the tail rotor)")
		
		self.atmosphere = atmosphere = FixedStandardAtmosphere(h)

		constraints = [atmosphere]
		
		return constraints


class LevelFlightState(Model):
	
	def setup(self, h=0*ureg.m):

		self.v                 = v                 = Variable("v",                   "m/s", "Flight speed")
		self.L                 = L                 = Variable("L",                   "N",   "Lift")
		self.T                 = T                 = Variable("T",                   "N",   "Thrust")
		self.D                 = D                 = Variable("D",                   "N",   "Drag")
		self.L_D               = L_D               = Variable("L/D",                 "-",   "Lift-to-drag ratio")
		self.P_electric        = P_electric        = Variable("P_{electric}",        "kW",  "Electrical power draw (from the battery)")
		self.P_shaft           = P_shaft           = Variable("P_{shaft}",           "kW",  "Shaft power (to all rotors)")
		self.P_shaft_thrust    = P_shaft_thrust    = Variable("P_{shaft,thrust}",    "kW",  "Shaft power (to generate thrust)")
		self.P_shaft_tailRotor = P_shaft_tailRotor = Variable("P_{shaft,tailRotor}", "kW",  "Shaft power (to the tail rotor)")
		
		self.atmosphere = atmosphere = FixedStandardAtmosphere(h)

		constraints =  [atmosphere]
		constraints += [L_D == L / D] 

		return constraints


class HoverTakeoff(Model):
	
	# All hover segments use exactly the same code. Duplication simplifies data output.
	def setup(self, aircraft, h=0.*ureg.m):

		self.state       = state       = HoverFlightState(h=h)
		self.performance = performance = aircraft.hover_performance(state)
		
		L = state.L
		T = state.T

		m = performance.m
		W = performance.W

		self.t_segment = t_segment = Variable("t_{segment}", "min", "Segment time")

		constraints =  [state, performance]
		constraints += [
			L         == W,
			L         == T,
			t_segment == performance.battery_perf.t,
		]
		
		return constraints


class HoverLanding(Model):
	
	# All hover segments use exactly the same code. Duplication simplifies data output.
	def setup(self, aircraft, h=0.*ureg.m):

		self.state       = state       = HoverFlightState(h=h)
		self.performance = performance = aircraft.hover_performance(state)
		
		L = state.L
		T = state.T

		m = performance.m
		W = performance.W

		self.t_segment = t_segment = Variable("t_{segment}", "min", "Segment time")

		constraints =  [state, performance]
		constraints += [
			L         == W,
			L         == T,
			t_segment == performance.battery_perf.t,
		]
		
		return constraints


class Cruise(Model):

	# Cruise and Reserve segments use exactly the same code. Duplication simplifies data output.
	def setup(self, aircraft, h=0*ureg.m):

		self.state       = state       = LevelFlightState(h=h)
		self.performance = performance = aircraft.level_flight_performance(state)

		v   = state.v
		L   = state.L
		T   = state.T
		D   = state.D
		L_D = state.L_D

		m = performance.m
		W = performance.W

		self.t_segment = t_segment = Variable("t_{segment}", "min", "Segment time")
		self.d_segment = d_segment = Variable("d_{segment}", "km",  "Segment distance travelled")

		constraints =  [state, performance]
		constraints += [
			L == W,
			T == D,
			
			d_segment == v * t_segment,
			t_segment == performance.battery_perf.t,
		]
		
		return constraints


class Reserve(Model):

	# Cruise and Reserve segments use exactly the same code. Duplication simplifies data output.
	def setup(self, aircraft, h=0.*ureg.m):

		self.state       = state       = LevelFlightState(h=h)
		self.performance = performance = aircraft.level_flight_performance(state)

		v   = state.v
		L   = state.L
		T   = state.T
		D   = state.D
		L_D = state.L_D

		m = performance.m
		W = performance.W

		self.t_segment = t_segment = Variable("t_{segment}", "min", "Segment time")
		self.d_segment = d_segment = Variable("d_{segment}", "km",  "Segment distance travelled")

		constraints =  [state, performance]
		constraints += [
			L == W,
			T == D,  
			
			d_segment == v * t_segment,
			t_segment == performance.battery_perf.t,
		]
		
		return constraints


class OnDemandSizingMission(Model):

	def standard_substitutions(self, piloted=True, reserve="20-minute loiter"):
		return on_demand_sizing_mission_substitutions(mission=self, piloted=piloted, reserve=reserve)

	# Mission the aircraft must be able to fly. No economic analysis.
	def setup(self, aircraft, h=0.*ureg.m):

		W_noPassengersOrCrew = aircraft.W_noPassengersOrCrew
		MTOW                 = aircraft.MTOW
		g                    = aircraft.g
		
		E_eff   = aircraft.battery.E_eff
		T_A_max = aircraft.rotors.T_A_max

		self.crew       = crew       = Crew()
		self.passengers = passengers = Passengers()
		self.payload    = payload    = [crew, passengers]

		self.m_mission = m_mission = Variable("m_{mission}", "kg",  "Mission aircraft mass (mass of the aircraft during the mission)")
		self.W_mission = W_mission = Variable("W_{mission}", "N",   "Mission aircraft weight (weight of the aircraft during the mission)")
		self.E_mission = E_mission = Variable("E_{mission}", "kWh", "Mission electrical energy used")

		self.v_reserve_nondim   = v_reserve_nondim   = Variable("v_{reserve,nondim}",     "-", "Reserve flight speed / cruise flight speed")
		self.L_D_reserve_nondim = L_D_reserve_nondim = Variable("(L/D)_{reserve,nondim}", "-", "Reserve lift-to-drag / cruise lift-to-drag")
		
		self.takeoff_segment = takeoff_segment = HoverTakeoff(aircraft=aircraft, h=h)
		self.cruise_segment  = cruise_segment  = Cruise(      aircraft=aircraft, h=h)
		self.reserve_segment = reserve_segment = Reserve(     aircraft=aircraft, h=h)
		self.landing_segment = landing_segment = HoverLanding(aircraft=aircraft, h=h)

		segments             = [takeoff_segment, cruise_segment, reserve_segment, landing_segment]
		flight_segments      = [takeoff_segment, cruise_segment, reserve_segment, landing_segment]
		hover_segments       = [takeoff_segment,                                  landing_segment]
		levelFlight_segments = [                 cruise_segment, reserve_segment                 ]

		constraints =  [payload, segments]

		constraints += [cruise_segment.state.v   == aircraft.v_cruise]
		constraints += [cruise_segment.state.L_D == aircraft.L_D_cruise]
		
		constraints += [reserve_segment.state.v   == cruise_segment.state.v   * v_reserve_nondim]
		constraints += [reserve_segment.state.L_D == cruise_segment.state.L_D * L_D_reserve_nondim]
		
		constraints += [W_mission == g * m_mission]
		constraints += [c.W       == g * c.m for c in payload]
		constraints += [W_mission == c.performance.W for c in flight_segments]
		constraints += [W_mission >= W_noPassengersOrCrew + sum(c.W for c in payload)]
		constraints += [MTOW      >= W_mission]
		
		constraints += [E_mission >= sum(c.performance.battery_perf.E for c in flight_segments)]
		constraints += [E_eff     >= E_mission]

		constraints += [T_A_max == c.performance.rotors_perf.T_A for c in hover_segments]  # Prevents disk area from going to infinity. TODO: check.

		return constraints


class Crew(Model):
	
	def setup(self):

		self.m_unit = m_unit = Variable("m_{unit}", "kg", "Crew unit mass")
		self.W_unit = W_unit = Variable("W_{unit}", "N",  "Crew unit weight")
		
		self.N = N = Variable("N", "-", "Number of crew members")

		self.m = m = Variable("m", "kg", "Crew total mass")
		self.W = W = Variable("W", "N",  "Crew weight")

		constraints = [
			m == N * m_unit,
			W == N * W_unit,
		]

		return constraints

class Passengers(Model):
	
	def setup(self):

		self.m_unit = m_unit = Variable("m_{unit}", "kg", "Passenger unit mass")
		self.W_unit = W_unit = Variable("W_{unit}", "N",  "Passenger unit weight")
		
		self.N = N = Variable("N", "-", "Number of passengers")

		self.m = m = Variable("m", "kg", "Passenger total mass")
		self.W = W = Variable("W", "N",  "Passenger total weight")

		constraints = [
			m == N * m_unit,
			W == N * W_unit,
		]

		return constraints


class Charger(Model):

	# Battery charger
	def setup(self):

		self.P   = P   = Variable("P",    "kW", "Charger power")
		self.eta = eta = Variable("\eta", "-",  "Charging efficiency")

		return []


class TimeOnGround(Model):
	
	# Mission segment for charging and passenger drop-off/pick-up
	def setup(self, aircraft):

		self.charger = charger = Charger()

		P   = charger.P
		eta = charger.eta

		self.t_segment   = t_segment   = Variable("t_{segment}",   "min", "Segment time")
		self.t_passenger = t_passenger = Variable("t_{passenger}", "min", "Time required to load/unload passengers and conduct safety checks")
		self.t_charge    = t_charge    = Variable("t_{charge}",    "min", "Time required to fully charge the battery")
		self.E_charger   = E_charger   = Variable("E_{charger}",   "kWh", "Energy supplied by charger")
		self.E_added     = E_added     = Variable("E_{added}",     "kWh", "Energy added to battery")
		
		constraints = [
			t_segment >= t_passenger, 
			t_segment >= t_charge,

			E_charger == P   * t_charge,
			E_added   == eta * E_charger,
		]

		return constraints


class OnDemandRevenueMission(Model):

	def standard_substitutions(self, piloted=True):
		return on_demand_revenue_mission_substitutions(mission=self, piloted=piloted)

	# Revenue and Deadhead missions have exactly the same code, except for the substitutions function. Simplifies data output.
	def setup(self, aircraft, h=0.*ureg.m):

		W_noPassengersOrCrew = aircraft.W_noPassengersOrCrew
		MTOW                 = aircraft.MTOW
		g                    = aircraft.g

		E_eff   = aircraft.battery.E_eff
		T_A_max = aircraft.rotors.T_A_max

		self.crew       = crew       = Crew()
		self.passengers = passengers = Passengers()
		self.payload    = payload    = [crew, passengers]

		self.m_mission = m_mission = Variable("m_{mission}", "kg",  "Mission aircraft mass (mass of the aircraft during the mission)")
		self.W_mission = W_mission = Variable("W_{mission}", "N",   "Mission aircraft weight (weight of the aircraft during the mission)")
		self.E_mission = E_mission = Variable("E_{mission}", "kWh", "Mission electrical energy used")

		self.t_mission = t_mission = Variable("t_{mission}", "min", "Time to complete mission (including charging)")
		self.t_flight  = t_flight  = Variable("t_{flight}",  "min", "Time in flight")

		self.takeoff_segment = takeoff_segment = HoverTakeoff(aircraft=aircraft, h=h)
		self.cruise_segment  = cruise_segment  = Cruise(      aircraft=aircraft, h=h)
		self.landing_segment = landing_segment = HoverLanding(aircraft=aircraft, h=h)
		self.ground_segment  = ground_segment  = TimeOnGround(aircraft=aircraft)

		segments             = [takeoff_segment, cruise_segment, landing_segment, ground_segment]
		flight_segments      = [takeoff_segment, cruise_segment, landing_segment,               ]
		hover_segments       = [takeoff_segment,                 landing_segment,               ]
		levelFlight_segments = [                 cruise_segment,                                ]

		constraints =  [payload, segments]

		constraints += [cruise_segment.state.v   == aircraft.v_cruise]
		constraints += [cruise_segment.state.L_D == aircraft.L_D_cruise]

		constraints += [t_flight  >= sum(c.t_segment for c in flight_segments)]
		constraints += [t_mission >= t_flight + ground_segment.t_segment]
		constraints += [E_mission == ground_segment.E_added]
		constraints += [W_mission == g * m_mission]

		constraints += [c.W       == g * c.m for c in payload]
		constraints += [W_mission == c.performance.W for c in flight_segments]
		constraints += [W_mission >= W_noPassengersOrCrew + sum(c.W for c in payload)]
		constraints += [MTOW      >= W_mission]

		constraints += [E_mission >= sum(c.performance.battery_perf.E for c in flight_segments)]
		constraints += [E_eff     >= E_mission]

		return constraints


class OnDemandDeadheadMission(Model):

	def standard_substitutions(self, piloted=True):
		return on_demand_deadhead_mission_substitutions(mission=self, piloted=piloted)

	# Revenue and Deadhead missions have exactly the same code, except for the substitutions function. Simplifies data output.
	def setup(self, aircraft, h=0.*ureg.m):

		W_noPassengersOrCrew = aircraft.W_noPassengersOrCrew
		MTOW                 = aircraft.MTOW
		g                    = aircraft.g

		E_eff   = aircraft.battery.E_eff
		T_A_max = aircraft.rotors.T_A_max

		self.crew       = crew       = Crew()
		self.passengers = passengers = Passengers()
		self.payload    = payload    = [crew, passengers]

		self.m_mission = m_mission = Variable("m_{mission}", "kg",  "Mission aircraft mass (mass of the aircraft during the mission)")
		self.W_mission = W_mission = Variable("W_{mission}", "N",   "Mission aircraft weight (weight of the aircraft during the mission)")
		self.E_mission = E_mission = Variable("E_{mission}", "kWh", "Mission electrical energy used")

		self.t_mission = t_mission = Variable("t_{mission}", "min", "Time to complete mission (including charging)")
		self.t_flight  = t_flight  = Variable("t_{flight}",  "min", "Time in flight")

		self.takeoff_segment = takeoff_segment = HoverTakeoff(aircraft=aircraft, h=h)
		self.cruise_segment  = cruise_segment  = Cruise(aircraft=aircraft, h=h)
		self.landing_segment = landing_segment = HoverLanding(aircraft=aircraft, h=h)
		self.ground_segment  = ground_segment  = TimeOnGround(aircraft=aircraft)

		segments             = [takeoff_segment, cruise_segment, landing_segment, ground_segment]
		flight_segments      = [takeoff_segment, cruise_segment, landing_segment,               ]
		hover_segments       = [takeoff_segment,                 landing_segment,               ]
		levelFlight_segments = [                 cruise_segment,                                ]

		constraints =  [payload, segments]

		constraints += [cruise_segment.state.v   == aircraft.v_cruise]
		constraints += [cruise_segment.state.L_D == aircraft.L_D_cruise]

		constraints += [t_flight  >= sum(c.t_segment for c in flight_segments)]
		constraints += [t_mission >= t_flight + ground_segment.t_segment]
		constraints += [E_mission == ground_segment.E_added]
		constraints += [W_mission == g * m_mission]

		constraints += [c.W       == g * c.m for c in payload]
		constraints += [W_mission == c.performance.W for c in flight_segments]
		constraints += [W_mission >= W_noPassengersOrCrew + sum(c.W for c in payload)]
		constraints += [MTOW      >= W_mission]

		constraints += [E_mission >= sum(c.performance.battery_perf.E for c in flight_segments)]
		constraints += [E_eff     >= E_mission]

		return constraints

if __name__ == "__main__":

	aircraft = OnDemandAircraft()
	aircraft = aircraft.standard_substitutions(config="Lift + cruise", autonomousEnabled=True)

	sizing_mission = OnDemandSizingMission(aircraft=aircraft)
	sizing_mission = sizing_mission.standard_substitutions(piloted=True, reserve="20-minute loiter")

	objective_function = aircraft.MTOM
	problem            = Model(objective_function, [aircraft, sizing_mission])
	solution           = problem.solve(verbosity=0)

	# print solution.table()
	
