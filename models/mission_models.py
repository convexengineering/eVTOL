# Mission models.
import numpy as np
import math
pi = math.pi

from gpkit                  import Variable, Model, Vectorize, ureg
from aircraft_models        import OnDemandAircraft
from standard_atmosphere    import stdatmo
from standard_substitutions import on_demand_sizing_mission_substitutions

# from noise_models import rotational_noise, vortex_noise, noise_weighting

class FixedStandardAtmosphere(Model):
	
	def setup(self, h):
		
		atmospheric_data = stdatmo(h)
		
		rho = atmospheric_data["\rho"].to(ureg.kg/ureg.m**3).magnitude
		a   = atmospheric_data["a"].to(ureg.m/ureg.s).magnitude
		
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
	def setup(self, aircraft, h=0*ureg.m):

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
	def setup(self, aircraft, h=0*ureg.m):

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
			L   == W,
			T   == D,
			v   == aircraft.v_cruise,    # Cruise value. 
			L_D == aircraft.L_D_cruise,  # Cruise value.   
			
			d_segment == v * t_segment,
			t_segment == performance.battery_perf.t,
		]
		
		return constraints


class Loiter(Model):

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
			L   == W,
			T   == D,
			v   == aircraft.v_loiter,    # Loiter value. 
			L_D == aircraft.L_D_loiter,  # Loiter value.   
			
			d_segment == v * t_segment,
			t_segment == performance.battery_perf.t,
		]
		
		return constraints


class OnDemandSizingMission(Model):

	def standard_substitutions(self, autonomous=False, reserve="20-minute loiter"):
		return on_demand_sizing_mission_substitutions(mission=self, autonomous=autonomous, reserve=reserve)

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
		
		self.takeoff_segment = takeoff_segment = HoverTakeoff(aircraft=aircraft, h=h)
		self.cruise_segment  = cruise_segment  = Cruise(aircraft=aircraft, h=h)
		self.loiter_segment  = loiter_segment  = Loiter(aircraft=aircraft, h=h)
		self.landing_segment = landing_segment = HoverLanding(aircraft=aircraft, h=h)

		segments             = [takeoff_segment, cruise_segment, loiter_segment, landing_segment]
		flight_segments      = [takeoff_segment, cruise_segment, loiter_segment, landing_segment]
		hover_segments       = [takeoff_segment,                                 landing_segment]
		levelFlight_segments = [                 cruise_segment, loiter_segment                 ]

		constraints =  [payload, segments]
		
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




class Hover(Model):
	#t should be set via substitution
	def setup(self,mission,aircraft,state):
		E = Variable("E","kWh","Electrical energy used during hover segment")
		P_battery = Variable("P_{battery}","kW","Power drawn (from batteries) during hover segment")
		P_rotors  = Variable("P_{rotors}","kW","Power used (by lifting rotors) during hover segment")
		P_tailRotor = Variable("P_{tailRotor}","kW","Power used (by tail rotor) during hover segment")
		tailRotor_power_fraction = Variable("tailRotor_power_fraction",
			"-","Tail-rotor power as a fraction of lifting-rotors power")
		T = Variable("T","N","Total thrust (from rotors) during hover segment")
		T_A = Variable("T/A","N/m**2","Disk loading during hover segment")
		t = Variable("t","s","Time in hover segment")
		W = mission.W

		self.E = E
		self.P_battery = P_battery
		self.P_rotors = P_rotors
		self.P_tailRotor = P_tailRotor
		self.tailRotor_power_fraction = tailRotor_power_fraction
		self.T = T
		self.T_A = T_A
		self.t = t
		self.W = W

		self.rotorPerf = aircraft.rotors.performance(state)
		self.batteryPerf = aircraft.battery.performance()
		self.electricalSystemPerf = aircraft.electricalSystem.performance()

		constraints = []
		constraints += [self.rotorPerf, self.batteryPerf, self.electricalSystemPerf]
		
		constraints += [P_rotors==self.rotorPerf.P,T==self.rotorPerf.T,
			T_A==self.rotorPerf.T_A]
		constraints += [self.electricalSystemPerf.P_in == P_battery,
			self.electricalSystemPerf.P_out >= P_rotors + P_tailRotor]
		constraints += [E==self.batteryPerf.E,t==self.batteryPerf.t,
			P_battery==self.batteryPerf.P]
		constraints += [T == W]

		constraints += [P_tailRotor == tailRotor_power_fraction*P_rotors]
		
		return constraints

class LevelFlight(Model):
	#Substitution required for either segment_range  or t (loiter time).
	def setup(self,mission,aircraft):
		E = Variable("E","kWh","Electrical energy used during level-flight segment")
		P_battery = Variable("P_{battery}","kW","Power drawn (from batteries) during segment")
		P_cruise  = Variable("P_{cruise}","kW","Power used (by propulsion system) during cruise segment")
		P_tailRotor = Variable("P_{tailRotor}","kW","Power used (by tail rotor) during hover segment")
		tailRotor_power_fraction = Variable("tailRotor_power_fraction",
			"-","Tail-rotor power as a fraction of cruise power")
		T = Variable("T","N","Thrust during level-flight  segment")
		D = Variable("D","N","Drag during level-flight segment")
		t = Variable("t","s","Time in level-flight segment")
		segment_range = Variable("segment_range","nautical_mile",
			"Distance travelled during segment")
		V = Variable("V","mph","Velocity during segment")
		L_D = Variable("L_D","-","Segment lift-to-drag ratio")
		
		W = mission.W
		eta_cruise = aircraft.eta_cruise
		
		self.E = E
		self.P_battery = P_battery
		self.P_cruise = P_cruise
		self.P_tailRotor = P_tailRotor
		self.tailRotor_power_fraction = tailRotor_power_fraction
		self.T = T
		self.D = D
		self.t = t
		self.segment_range = segment_range
		self.V = V
		self.L_D = L_D

		constraints = []
		
		self.batteryPerf = aircraft.battery.performance()
		self.electricalSystemPerf = aircraft.electricalSystem.performance()

		constraints += [self.batteryPerf, self.electricalSystemPerf]

		constraints += [E==self.batteryPerf.E, P_battery==self.batteryPerf.P,
			t==self.batteryPerf.t]
		constraints += [self.electricalSystemPerf.P_in == P_battery,
			self.electricalSystemPerf.P_out >= P_cruise + P_tailRotor]
		constraints += [segment_range==V*t,eta_cruise*P_cruise==T*V,T==D,W==L_D*D]

		constraints += [P_tailRotor == tailRotor_power_fraction*P_cruise]

		return constraints

class TimeOnGround(Model):
	#Mission segment for charging and passenger drop-off/pick-up
	def setup(self,mission):

		E_mission = mission.E_mission

		t = Variable("t","s","Time spent on ground")
		t_passenger = Variable("t_{passenger}",5,"minute",
			"Time required to load/unload passengers and conduct safety checks")
		t_charge = Variable("t_{charge}","s","Time required to fully charge the battery")
		charger_power = Variable("charger_power","kW","Charger power")
		eta_charger = Variable("\eta_{charger}",0.9,"-","Charging efficiency")
		E_charger = Variable("E_{charger}","kWh","Energy supplied by charger")

		self.t = t
		self.t_passenger = t_passenger
		self.t_charge = t_charge
		self.charger_power = charger_power
		self.eta_charger = eta_charger
		self.E_charger = E_charger

		constraints = []
		
		constraints += [t >= t_passenger, t >= t_charge]
		constraints += [E_mission == eta_charger*E_charger]
		constraints += [E_charger == charger_power*t_charge]

		return constraints



class OnDemandRevenueMission(Model):
	#Revenue-generating mission. Exactly the same code as OnDemandDeadheadMission.
    def setup(self,aircraft,mission_type="piloted"):

    	if not(aircraft.autonomousEnabled) and (mission_type != "piloted"):
    		raise ValueError("Autonomy is not enabled for Aircraft() model.")

    	W = Variable("W_{mission}","N","Weight of the aircraft during the mission")
    	mission_range = Variable("mission_range","nautical_mile","Mission range")
    	t_hover = Variable("t_{hover}","s","Time in hover")
    	V_cruise = Variable("V_{cruise}","mph","Aircraft cruising speed")
    	T_A = Variable("T/A","N/m**2","Disk loading")
    	
        C_eff = aircraft.battery.C_eff #effective battery capacity
        
        t_mission = Variable("t_{mission}","minutes","Time to complete mission (including charging)")
        t_flight = Variable("t_{flight}","minutes","Time in flight")
        E_mission = Variable("E_{mission}","kWh","Electrical energy used during mission")

        self.W = W
        self.mission_range = mission_range
        self.t_hover = t_hover
        self.V_cruise = V_cruise
        self.T_A = T_A
        self.C_eff = C_eff
        self.t_mission = t_mission
        self.t_flight = t_flight
        self.E_mission = E_mission

        self.mission_type = mission_type

        self.crew = Crew(mission_type=mission_type)
        self.passengers = Passengers()
        
        hoverState = FlightState(h=0*ureg.ft)

        self.fs0 = Hover(self,aircraft,hoverState)#takeoff
        self.fs1 = LevelFlight(self,aircraft)#fly to destination
        self.fs2 = Hover(self,aircraft,hoverState)#landing
        self.time_on_ground = TimeOnGround(self)

        self.segments = [self.fs0, self.fs1, self.fs2, self.time_on_ground]
        self.flight_segments = [self.fs0, self.fs1, self.fs2]
        self.levelFlight_segments = [self.fs1]
        self.hover_segments = [self.fs0, self.fs2]

        #Power and energy consumption by mission segment
        with Vectorize(len(self.flight_segments)):
        	P_battery = Variable("P_{battery}","kW","Segment power draw")
        	E = Variable("E","kWh","Segment energy use")

        #Data from hover segments
        numHoverSegments = len(self.hover_segments)
        with Vectorize(numHoverSegments):
        	CT = Variable("CT","-","Thrust coefficient")
        	CP = Variable("CP","-","Power coefficient")
        	Q_perRotor = Variable("Q_perRotor","N*m","Torque per lifting rotor")
        	T_perRotor = Variable("T_perRotor","N","Thrust per lifting rotor")
        	P = Variable("P","kW","Total power supplied to all lifting rotors")
        	P_perRotor = Variable("P_perRotor","kW","Power per lifting rotor")
        	VT = Variable("VT","ft/s","Rotor tip speed")
        	omega = Variable("\omega","rpm","Rotor angular velocity")
        	MT = Variable("MT","-","Rotor tip Mach number")
        	FOM = Variable("FOM","-","Figure of merit")
        	p_ratio = Variable("p_{ratio}","-","Sound pressure ratio in hover")

        constraints = []

        constraints += [self.fs0.T_A == T_A]
        constraints += [self.fs1.L_D == aircraft.L_D_cruise]
        constraints += [self.fs1.V == V_cruise]

        constraints += [self.segments]
        constraints += [self.crew,self.passengers]

        constraints += [W >= aircraft.W_empty + self.passengers.W \
        	+ self.crew.W]
        constraints += [aircraft.TOGW >= W]
        
        constraints += [mission_range == self.fs1.segment_range]
        constraints += [p_ratio == self.fs0.rotorPerf.p_ratio]
        constraints += hoverState

        constraints += [E_mission >= sum(c.E for c in self.flight_segments)]
        constraints += [C_eff >= E_mission]

        constraints += [aircraft.tailRotor_power_fraction_levelFlight == segment.tailRotor_power_fraction \
        	for i,segment in enumerate(self.levelFlight_segments)]
        constraints += [aircraft.tailRotor_power_fraction_hover == segment.tailRotor_power_fraction \
        	for i,segment in enumerate(self.hover_segments)]
        constraints += [t_hover == segment.t for i,segment in enumerate(self.hover_segments)]

        constraints += [t_flight >= sum(c.t for c in self.flight_segments)]
        constraints += [t_mission >= t_flight + self.time_on_ground.t]

        constraints += [P_battery[i] == segment.P_battery for i,segment in enumerate(self.flight_segments)]
        constraints += [E[i] == segment.E for i,segment in enumerate(self.flight_segments)]

        constraints += [CT[i] == segment.rotorPerf.CT for i,segment in enumerate(self.hover_segments)]
        constraints += [CP[i] == segment.rotorPerf.CP for i,segment in enumerate(self.hover_segments)]
        constraints += [Q_perRotor[i] == segment.rotorPerf.Q_perRotor for i,segment in enumerate(self.hover_segments)]
        constraints += [T_perRotor[i] == segment.rotorPerf.T_perRotor for i,segment in enumerate(self.hover_segments)]
        constraints += [P[i] == segment.rotorPerf.P for i,segment in enumerate(self.hover_segments)]
        constraints += [P_perRotor[i] == segment.rotorPerf.P_perRotor for i,segment in enumerate(self.hover_segments)]
        constraints += [VT[i] == segment.rotorPerf.VT for i,segment in enumerate(self.hover_segments)]
        constraints += [omega[i] == segment.rotorPerf.omega for i,segment in enumerate(self.hover_segments)]
        constraints += [MT[i] == segment.rotorPerf.MT for i,segment in enumerate(self.hover_segments)]
        constraints += [FOM[i] == segment.rotorPerf.FOM for i,segment in enumerate(self.hover_segments)]
        constraints += [p_ratio[i] == segment.rotorPerf.p_ratio for i,segment in enumerate(self.hover_segments)]

        return constraints

class OnDemandDeadheadMission(Model):
	#Deadhead mission. Exactly the same code as OnDemandRevenueMission.
    def setup(self,aircraft,mission_type="piloted"):

    	if not(aircraft.autonomousEnabled) and (mission_type != "piloted"):
    		raise ValueError("Autonomy is not enabled for Aircraft() model.")

    	W = Variable("W_{mission}","N","Weight of the aircraft during the mission")
    	mission_range = Variable("mission_range","nautical_mile","Mission range")
    	t_hover = Variable("t_{hover}","s","Time in hover")
    	V_cruise = Variable("V_{cruise}","mph","Aircraft cruising speed")
    	T_A = Variable("T/A","N/m**2","Disk loading")
    	
        C_eff = aircraft.battery.C_eff #effective battery capacity
        
        t_mission = Variable("t_{mission}","minutes","Time to complete mission (including charging)")
        t_flight = Variable("t_{flight}","minutes","Time in flight")
        E_mission = Variable("E_{mission}","kWh","Electrical energy used during mission")

        self.W = W
        self.mission_range = mission_range
        self.t_hover = t_hover
        self.V_cruise = V_cruise
        self.T_A = T_A
        self.C_eff = C_eff
        self.t_mission = t_mission
        self.t_flight = t_flight
        self.E_mission = E_mission

        self.mission_type = mission_type

        self.crew = Crew(mission_type=mission_type)
        self.passengers = Passengers()
        
        hoverState = FlightState(h=0*ureg.ft)

        self.fs0 = Hover(self,aircraft,hoverState)#takeoff
        self.fs1 = LevelFlight(self,aircraft)#fly to destination
        self.fs2 = Hover(self,aircraft,hoverState)#landing
        self.time_on_ground = TimeOnGround(self)

        self.segments = [self.fs0, self.fs1, self.fs2, self.time_on_ground]
        self.flight_segments = [self.fs0, self.fs1, self.fs2]
        self.levelFlight_segments = [self.fs1]
        self.hover_segments = [self.fs0, self.fs2]

        #Power and energy consumption by mission segment
        with Vectorize(len(self.flight_segments)):
        	P_battery = Variable("P_{battery}","kW","Segment power draw")
        	E = Variable("E","kWh","Segment energy use")

        #Data from hover segments
        numHoverSegments = len(self.hover_segments)
        with Vectorize(numHoverSegments):
        	CT = Variable("CT","-","Thrust coefficient")
        	CP = Variable("CP","-","Power coefficient")
        	Q_perRotor = Variable("Q_perRotor","N*m","Torque per lifting rotor")
        	T_perRotor = Variable("T_perRotor","N","Thrust per lifting rotor")
        	P = Variable("P","kW","Total power supplied to all lifting rotors")
        	P_perRotor = Variable("P_perRotor","kW","Power per lifting rotor")
        	VT = Variable("VT","ft/s","Rotor tip speed")
        	omega = Variable("\omega","rpm","Rotor angular velocity")
        	MT = Variable("MT","-","Rotor tip Mach number")
        	FOM = Variable("FOM","-","Figure of merit")
        	p_ratio = Variable("p_{ratio}","-","Sound pressure ratio in hover")

        constraints = []

        constraints += [self.fs0.T_A == T_A]
        constraints += [self.fs1.L_D == aircraft.L_D_cruise]
        constraints += [self.fs1.V == V_cruise]

        constraints += [self.segments]
        constraints += [self.crew,self.passengers]

        constraints += [W >= aircraft.W_empty + self.passengers.W \
        	+ self.crew.W]
        constraints += [aircraft.TOGW >= W]
        
        constraints += [mission_range == self.fs1.segment_range]
        constraints += [p_ratio == self.fs0.rotorPerf.p_ratio]
        constraints += hoverState

        constraints += [E_mission >= sum(c.E for c in self.flight_segments)]
        constraints += [C_eff >= E_mission]

        constraints += [aircraft.tailRotor_power_fraction_levelFlight == segment.tailRotor_power_fraction \
        	for i,segment in enumerate(self.levelFlight_segments)]
        constraints += [aircraft.tailRotor_power_fraction_hover == segment.tailRotor_power_fraction \
        	for i,segment in enumerate(self.hover_segments)]
        constraints += [t_hover == segment.t for i,segment in enumerate(self.hover_segments)]

        constraints += [t_flight >= sum(c.t for c in self.flight_segments)]
        constraints += [t_mission >= t_flight + self.time_on_ground.t]

        constraints += [P_battery[i] == segment.P_battery for i,segment in enumerate(self.flight_segments)]
        constraints += [E[i] == segment.E for i,segment in enumerate(self.flight_segments)]

        constraints += [CT[i] == segment.rotorPerf.CT for i,segment in enumerate(self.hover_segments)]
        constraints += [CP[i] == segment.rotorPerf.CP for i,segment in enumerate(self.hover_segments)]
        constraints += [Q_perRotor[i] == segment.rotorPerf.Q_perRotor for i,segment in enumerate(self.hover_segments)]
        constraints += [T_perRotor[i] == segment.rotorPerf.T_perRotor for i,segment in enumerate(self.hover_segments)]
        constraints += [P[i] == segment.rotorPerf.P for i,segment in enumerate(self.hover_segments)]
        constraints += [P_perRotor[i] == segment.rotorPerf.P_perRotor for i,segment in enumerate(self.hover_segments)]
        constraints += [VT[i] == segment.rotorPerf.VT for i,segment in enumerate(self.hover_segments)]
        constraints += [omega[i] == segment.rotorPerf.omega for i,segment in enumerate(self.hover_segments)]
        constraints += [MT[i] == segment.rotorPerf.MT for i,segment in enumerate(self.hover_segments)]
        constraints += [FOM[i] == segment.rotorPerf.FOM for i,segment in enumerate(self.hover_segments)]
        constraints += [p_ratio[i] == segment.rotorPerf.p_ratio for i,segment in enumerate(self.hover_segments)]

        return constraints


if __name__=="__main__":

	aircraft = OnDemandAircraft()
	aircraft = aircraft.standard_substitutions(autonomousEnabled=True)

	sizing_mission = OnDemandSizingMission(aircraft=aircraft)
	sizing_mission = sizing_mission.standard_substitutions(autonomous=False, reserve="20-minute loiter")

	objective_function = aircraft.MTOM
	problem            = Model(objective_function, [aircraft, sizing_mission])
	solution           = problem.solve(verbosity=0)

	
	
	
	"""
	delta_S = 500*ureg.ft
	noise_weighting = "A"
	B = 5

	SPL_dict = {}
	missions = ["Sizing","Revenue","Deadhead"]

	for mission in missions:
		mission_name = "OnDemand" + mission + "Mission"
		
		T_perRotor = solution("T_perRotor_" + mission_name)[0]
		R = solution("R")
		VT = solution("VT_" + mission_name)[0]
		s = solution("s")
		Cl_mean = solution("Cl_{mean_{max}}")
		N = solution("N")

		f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
			weighting=noise_weighting)

		SPL_dict[mission] = SPL

	if (reserve_type == "FAA_aircram") or (reserve_type == "FAA_heli"):
		num = solution("t_{loiter}_OnDemandSizingMission").to(ureg.minute).magnitude
		reserve_type_string = " (%0.0f-minute loiter time)" % num
	if reserve_type == "Uber":
		num = solution("R_{divert}_OnDemandSizingMission").to(ureg.nautical_mile).magnitude
		reserve_type_string = " (%0.1f-nmi diversion distance)" % num

	
	print
	print "Concept representative analysis"
	print
	print "Battery energy density: %0.0f Wh/kg" \
		% solution("C_m_OnDemandAircraft/Battery").to(ureg.Wh/ureg.kg).magnitude
	print "Empty weight fraction: %0.4f" \
		% solution("weight_fraction_OnDemandAircraft/Structure")
	print "Cruise lift-to-drag ratio: %0.1f" \
		% solution("L_D_cruise_OnDemandAircram")
	print "Hover disk loading: %0.1f lbf/ft^2" \
		% solution("T/A_OnDemandSizingMission").to(ureg("N/m**2")).magnitude
	print "Rotor maximum mean lift coefficient: %0.2f" \
		% solution("Cl_{mean_{max}}_OnDemandAircraft/Rotors")
	print "Cruise propulsive efficiency: %0.2f" \
		% solution("\eta_{cruise}_OnDemandAircram")
	print "Electrical system efficiency: %0.2f" \
		% solution("\eta_OnDemandAircraft/ElectricalSystem")
	print "Observer distance: %0.0f m" % delta_S.to(ureg.ft).magnitude
	print "Noise weighting type: %s" % noise_weighting
	print
	print "Sizing Mission (%s)" % sizing_mission_type
	print "Mission range: %0.0f nmi" % \
		solution("mission_range_OnDemandSizingMission").to(ureg.nautical_mile).magnitude
	print "Number of passengers: %0.1f" % \
		solution("N_{passengers}_OnDemandSizingMission/Passengers")
	print "Reserve type: " + reserve_type + reserve_type_string
	print "Vehicle weight during mission: %0.0f lbf" % \
		solution("W_{mission}_OnDemandSizingMission").to(ureg.lbf).magnitude
	print "SPL in hover: %0.1f dB" % SPL_dict["Sizing"]
	print
	print "Revenue-Generating Mission (%s)" % revenue_mission_type
	print "Mission range: %0.0f nmi" % \
		solution("mission_range_OnDemandRevenueMission").to(ureg.nautical_mile).magnitude
	print "Number of passengers: %0.1f" % \
		solution("N_{passengers}_OnDemandRevenueMission/Passengers")
	print "Vehicle weight during mission: %0.0f lbf" % \
		solution("W_{mission}_OnDemandRevenueMission").to(ureg.lbf).magnitude
	print "Total time: %0.1f minutes" % \
		solution("t_{mission}_OnDemandRevenueMission").to(ureg.minute).magnitude
	print "Flight time: %0.1f minutes" % \
		solution("t_{flight}_OnDemandRevenueMission").to(ureg.minute).magnitude
	print "Time on ground: %0.1f minutes" % \
		solution("t_OnDemandRevenueMission/TimeOnGround").to(ureg.minute).magnitude
	print "SPL in hover: %0.1f dB" % SPL_dict["Revenue"]
	print
	print "Deadhead Mission (%s)" % deadhead_mission_type
	print "Mission range: %0.0f nmi" % \
		solution("mission_range_OnDemandDeadheadMission").to(ureg.nautical_mile).magnitude
	print "Number of passengers: %0.1f" % \
		solution("N_{passengers}_OnDemandDeadheadMission/Passengers")
	print "Vehicle weight during mission: %0.0f lbf" % \
		solution("W_{mission}_OnDemandDeadheadMission").to(ureg.lbf).magnitude
	print "Total time: %0.1f minutes" % \
		solution("t_{mission}_OnDemandDeadheadMission").to(ureg.minute).magnitude
	print "Flight time: %0.1f minutes" % \
		solution("t_{flight}_OnDemandDeadheadMission").to(ureg.minute).magnitude
	print "Time on ground: %0.1f minutes" % \
		solution("t_OnDemandDeadheadMission/TimeOnGround").to(ureg.minute).magnitude
	print "SPL in hover: %0.1f dB" % SPL_dict["Deadhead"]
	print
	print "Takeoff gross weight: %0.0f lbs" % \
		solution("TOGW_OnDemandAircram").to(ureg.lbf).magnitude
	print "Empty weight: %0.0f lbs" % \
		solution("W_OnDemandAircraft/Structure").to(ureg.lbf).magnitude
	print "Battery weight: %0.0f lbs" % \
		solution("W_OnDemandAircraft/Battery").to(ureg.lbf).magnitude
	print "Vehicle purchase price: $%0.0f " % \
		solution("purchase_price_OnDemandAircram")
	print "Avionics purchase price: $%0.0f " % \
		solution("purchase_price_OnDemandAircraft/Avionics")
	print "Battery purchase price:  $%0.0f " % \
		solution("purchase_price_OnDemandAircraft/Battery")
	print
	print "Cost per trip: $%0.2f" % \
		solution("cost_per_trip_OnDemandMissionCost")
	print "Cost per trip, per passenger: $%0.2f" % \
		solution("cost_per_trip_per_passenger_OnDemandMissionCost")
	print "Cost per trip, per seat mile: $%0.2f per mile" % \
		solution("cost_per_seat_mile_OnDemandMissionCost").to(ureg.mile**-1).magnitude
	print "Cost from revenue-generating flight: $%0.2f" % \
		solution("revenue_cost_per_trip_OnDemandMissionCost")
	print "Cost from deadhead flight: $%0.2f" % \
		solution("deadhead_cost_per_trip_OnDemandMissionCost")
	print
	print "Cost Breakdown from Revenue-Generating Flight Only (no deadhead)"
	print
	print "Vehicle capital expenses, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses")
	print "Amortized vehicle acquisition cost, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses/VehicleAcquisitionCost")
	print "Amortized avionics acquisition cost, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses/AvionicsAcquisitionCost")
	print "Amortized battery acquisition cost, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses/BatteryAcquisitionCost")
	print	
	print "Vehicle operating expenses, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses")
	print "Direct operating cost, per trip: $%0.2f" % \
		solution("DOC_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses")
	print "Indirect operating cost, per trip: $%0.2f" % \
		solution("IOC_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses")
	print
	print "Pilot cost, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses/PilotCost")
	print "Amortized maintenance cost, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses/MaintenanceCost")
	print "Energy cost, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses/EnergyCost")
	"""
	
	#print solution.summary()
	
