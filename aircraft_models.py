#Top-level aircraft model.

import math
import numpy as np
from gpkit import Variable, Model, Vectorize, ureg
from standard_atmosphere import stdatmo

class OnDemandAircraft(Model):
	def setup(self,autonomousEnabled=False):
		
		TOGW = Variable("TOGW","lbf","Aircraft takeoff gross weight")
		W_empty = Variable("W_{empty}","lbf","Weight without passengers or crew")
		C_eff = Variable("C_{eff}","kWh","Effective battery capacity")
		g = Variable("g",9.807,"m/s**2","Gravitational acceleration")
		
		L_D_loiter = Variable("L_D_loiter","-","Loiter L/D ratio (approximation)")
		L_D_cruise = Variable("L_D_cruise","-","Cruise L/D ratio")
		eta_cruise = Variable("\eta_{cruise}","-","Cruise propulsive efficiency")
		tailRotor_power_fraction_hover = Variable("tailRotor_power_fraction_hover",
			0.0001,"-","Tail-rotor power as a fraction of lifting-rotors power")
		tailRotor_power_fraction_levelFlight = Variable("tailRotor_power_fraction_levelFlight",
			0.0001,"-","Tail-rotor power as a fraction of lifting-rotors power")

		cost_per_weight = Variable("cost_per_weight","lbf**-1",
			"Cost per unit empty weight of the aircraft")
		purchase_price = Variable("purchase_price","-","Purchase price of the airframe")
		vehicle_life = Variable("vehicle_life",20000*ureg.hour,"hours","Vehicle lifetime")

		self.autonomousEnabled = autonomousEnabled

		self.rotors = Rotors()
		self.battery = Battery()
		self.structure = Structure()
		self.electricalSystem = ElectricalSystem()
		self.avionics = Avionics(autonomousEnabled=autonomousEnabled)
		
		self.TOGW = TOGW
		self.W_empty = W_empty
		self.C_eff = C_eff
		self.g = g
		self.L_D_loiter = L_D_loiter
		self.L_D_cruise = L_D_cruise
		self.eta_cruise = eta_cruise
		self.tailRotor_power_fraction_hover = tailRotor_power_fraction_hover
		self.tailRotor_power_fraction_levelFlight = tailRotor_power_fraction_levelFlight

		self.cost_per_weight = cost_per_weight
		self.purchase_price = purchase_price
		self.vehicle_life = vehicle_life

		self.structure.subinplace({self.structure.topvar("TOGW"):TOGW})

		self.components = [self.rotors,self.battery,self.structure,self.electricalSystem,self.avionics]
		
		constraints = []
		
		constraints += [g == self.battery.topvar("g")]
		constraints += [self.components]#all constraints implemented at component level
		
		constraints += [L_D_loiter == ((3**0.5)/2.)*L_D_cruise]

		constraints += [C_eff == self.battery.topvar("C_{eff}")]#battery-capacity constraint
		constraints += [W_empty >= sum(c.topvar("W") for c in self.components)]#weight constraint
		constraints += [purchase_price == cost_per_weight*self.structure.topvar("W")]

		return constraints

class Structure(Model):
	def setup(self):
		
		TOGW = Variable("TOGW","lbf",
			"Aircraft takeoff gross weight (requires substitution or subinplace)")
		W = Variable("W","lbf","Empty weight")
		weight_fraction = Variable("weight_fraction","-","Empty weight fraction")
		
		self.W = W
		self.weight_fraction = weight_fraction

		return [W == weight_fraction*TOGW]


class Rotors(Model):

	def performance(self,flightState):
		return RotorsAero(self,flightState)

	def setup(self):
		R = Variable("R","ft","Propeller radius")
		D = Variable("D","ft","Propeller diameter")
		A = Variable("A","ft^2","Area of 1 rotor disk")
		A_total = Variable("A_{total}","ft^2","Combined area of all rotor disks")
		N = Variable("N","-","Number of rotors")
		s = Variable("s",0.1,"-","Propeller solidity")
		Cl_mean_max = Variable("Cl_{mean_{max}}","-","Maximum allowed mean lift coefficient")
		W = Variable("W",0,"lbf","Rotor weight") #weight model not implemented yet

		self.R = R
		self.D = D
		self.A = A
		self.A_total = A_total
		self.N = N
		self.s = s
		self.Cl_mean_max = Cl_mean_max
		self.W = W

		constraints = [A == math.pi*R**2, D==2*R, A_total==N*A,
			Cl_mean_max == Cl_mean_max]

		return constraints

class RotorsAero(Model):
	def setup(self,rotors,flightState):
		T = Variable("T","lbf","Total thrust")
		T_perRotor = Variable("T_perRotor","lbf","Thrust per rotor")
		T_A = Variable("T/A","lbf/ft**2","Disk loading")
		Q_perRotor = Variable("Q_perRotor","lbf*ft","Torque per rotor")
		P = Variable("P","kW","Total power")
		P_perRotor = Variable("P_perRotor","kW","Power per rotor")
		VT = Variable("VT","ft/s","Propeller tip speed")
		omega = Variable("\omega","rpm","Propeller angular velocity")
		MT = Variable("MT","-","Propeller tip Mach number")
		MT_max = Variable("MT_max",0.9,"-","Maximum allowed tip Mach number")

		CT = Variable("CT","-","Thrust coefficient")
		CQ = Variable("CQ","-","Torque coefficient")
		CP = Variable("CP","-","Power coefficient")
		CPi = Variable("CPi","-","Induced (ideal) power coefficient")
		CPp = Variable("CPp","-","Profile power coefficient")
		Cl_mean = Variable("Cl_mean","-","Mean lift coefficient")
		FOM = Variable("FOM","-","Figure of merit")

		ki = Variable("ki",1.2,"-","Induced power factor")
		Cd0 = Variable("Cd0",0.01,"-","Blade two-dimensional zero-lift drag coefficient")

		p_ratio = Variable("p_{ratio}","-","Sound pressure ratio (p/p_{ref})")
		x = Variable("x",500,"ft","Distance from source at which to calculate sound")
		k3 = Variable("k3",6.804e-3,"s**3/ft**3","Sound-pressure constant")

		R = rotors.R
		A = rotors.A
		A_total = rotors.A_total
		N = rotors.N
		s = rotors.s
		Cl_mean_max = rotors.Cl_mean_max

		rho = flightState.rho
		a = flightState.a

		self.T = T
		self.T_perRotor = T_perRotor
		self.T_A = T_A
		self.Q_perRotor = Q_perRotor
		self.P = P
		self.P_perRotor = P_perRotor
		self.VT = VT
		self.omega = omega
		self.MT = MT
		self.MT_max = MT_max
		
		self.CT = CT
		self.CQ = CQ
		self.CP = CP
		self.CPi = CPi
		self.CPp = CPp
		self.Cl_mean = Cl_mean
		self.FOM = FOM
		
		self.ki = ki
		self.Cd0 = Cd0
		
		self.p_ratio = p_ratio
		self.x = x
		self.k3 = k3

		constraints = [flightState]

		#Top-level constraints
		constraints += [T == N * T_perRotor,
			P == N * P_perRotor]
		constraints += [T_perRotor == 0.5*rho*(VT**2)*A*CT,
			P_perRotor == 0.5*rho*(VT**3)*A*CP]
		constraints += [T_A == T/A_total]

		#Torque 
		constraints += [CQ == CP]
		constraints += [Q_perRotor == 0.5*rho*(VT**2)*A*R*CQ]

		#Performance model
		constraints += [CPi == 0.5*CT**1.5,
			CPp == 0.25*s*Cd0,
			CP >= ki*CPi + CPp,
			FOM == CPi / CP]

		#Tip-speed constraints (upper limit on VT)
		constraints += [VT == R*omega,
			VT == MT * a,
			MT <= MT_max]

		#Mean lift coefficient constraints (lower limit on VT)
		constraints += [Cl_mean == 3*CT/s,
			Cl_mean <= Cl_mean_max]

		#Noise model
		constraints += [p_ratio == k3*((T*omega)/(rho*x))*(N*s)**-0.5]

		return constraints

class Battery(Model):

	def performance(self):
		return BatteryPerformance(self)

	#Requires a substitution or constraint for g (gravitational acceleration)
	def setup(self):
		
		g = Variable("g","m/s**2","Gravitational acceleration")
		
		C = Variable("C","kWh","Battery capacity")
		C_eff = Variable("C_{eff}","kWh","Effective battery capacity")
		usable_energy_fraction = Variable("usable_energy_fraction",0.8,
			"-","Percentage of the battery energy that can be used without damaging battery")
	
		W = Variable("W","lbf","Battery weight")
		m = Variable("m","kg","Battery mass")
		C_m = Variable("C_m","Wh/kg","Battery energy density")
		P_m = Variable("P_m",3000*ureg.W/ureg.kg,"W/kg","Battery power density")
		P_max = Variable("P_{max}","kW","Battery maximum power")

		cost_per_C = Variable("cost_per_C","kWh**-1","Battery cost per unit energy stored")
		purchase_price = Variable("purchase_price","-","Purchase price of the battery")
		cycle_life = Variable("cycle_life",2000,"-",
			"Number of cycles before battery needs replacement")

		self.g = g
		self.C = C
		self.C_eff = C_eff
		self.usable_energy_fraction = usable_energy_fraction
		self.W = W
		self.m = m
		self.C_m = C_m
		self.P_m = P_m
		self.P_max = P_max
		self.cost_per_C = cost_per_C
		self.purchase_price = purchase_price
		self.cycle_life = cycle_life

		constraints = []

		constraints += [C==m*C_m, W==m*g]
		constraints += [C_eff == usable_energy_fraction*C, P_max==P_m*m]
		constraints += [purchase_price == cost_per_C*C]

		return constraints

class BatteryPerformance(Model):
	def setup(self,battery):
		E = Variable("E","kWh","Electrical energy used during segment")
		P = Variable("P","kW","Power draw during segment")
		t = Variable("t","s","Time over which battery is providing power")

		self.E = E
		self.P = P
		self.t = t

		constraints = [E==P*t, P<=battery.P_max]
		return constraints

class Crew(Model):
	def setup(self,mission_type="piloted"):
		
		W_oneCrew = Variable("W_{oneCrew}",190,"lbf","Weight of 1 crew member")
		N_crew = Variable("N_{crew}",1,"-","Number of crew members (if present)")

		constraints = []

		if mission_type == "autonomous":
			W = Variable("W",0,"lbf","Total weight")
		if mission_type == "piloted":
			W = Variable("W","lbf","Total weight")
			constraints += [W == N_crew*W_oneCrew]

		self.W_oneCrew = W_oneCrew
		self.N_crew = N_crew
		self.W = W

		return constraints

class Passengers(Model):
	def setup(self):
		W_onePassenger = Variable("W_{onePassenger}",200,"lbf","Weight of 1 passenger")
		N_passengers = Variable("N_{passengers}","-","Number of passengers")
		W = Variable("W","lbf","Total weight")

		self.W_onePassenger = W_onePassenger
		self.N_passengers = N_passengers
		self.W = W

		return [W == N_passengers*W_onePassenger]

class ElectricalSystem(Model):
	def performance(self):
		return ElectricalSystemPerformance(self)

	def setup(self):
		W = Variable("W",0,"lbf","Electrical power system weight")
		eta = Variable("\eta","-","Electrical power system efficiency")

		self.W = W
		self.eta = eta

		constraints = []
		return constraints

class ElectricalSystemPerformance(Model):
	def setup(self,electricalSystem):
		P_in = Variable("P_{in}","kW","Input power (from the battery)")
		P_out = Variable("P_{out}","kW","Output power (to the motor or motors)")
		eta = electricalSystem.eta

		self.P_in = P_in
		self.P_out = P_out
		self.eta = eta

		constraints = []
		constraints += [P_out == eta*P_in]
		return constraints

class Avionics(Model):
	def setup(self,autonomousEnabled=False):

		W = Variable("W",0,"lbf","Weight of the avionics")

		if autonomousEnabled:
			purchase_price = Variable("purchase_price",60000,"-",
				"Purchase price of the avionics (Uber estimate)")
		else:
			purchase_price = Variable("purchase_price",1,"-",
				"Purchase price of the avionics (negligibly small)")

		self.purchase_price = purchase_price
		self.W = W

		constraints = []
		return constraints


class FlightState(Model):
	def setup(self,h):
		
		atmospheric_data = stdatmo(h)
		rho = atmospheric_data["\rho"].to(ureg.kg/ureg.m**3)
		a = atmospheric_data["a"].to(ureg.ft/ureg.s)
		rho = Variable("\rho",rho,"kg/m^3","Air density")
		a = Variable("a",a,"ft/s","Speed of sound")

		self.rho = rho
		self.a = a

		constraints = []
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
		T = Variable("T","lbf","Total thrust (from rotors) during hover segment")
		T_A = Variable("T/A","lbf/ft**2","Disk loading during hover segment")
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
		
		constraints += [P_rotors==self.rotorPerf.topvar("P"),T==self.rotorPerf.topvar("T"),
			T_A==self.rotorPerf.topvar("T/A")]
		constraints += [self.electricalSystemPerf.topvar("P_{in}") == P_battery,
			self.electricalSystemPerf.topvar("P_{out}") >= P_rotors + P_tailRotor]
		constraints += [E==self.batteryPerf.topvar("E"),t==self.batteryPerf.topvar("t"),
			P_battery==self.batteryPerf.topvar("P")]
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
		T = Variable("T","lbf","Thrust during level-flight  segment")
		D = Variable("D","lbf","Drag during level-flight segment")
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

		constraints += [E==self.batteryPerf.topvar("E"), P_battery==self.batteryPerf.topvar("P"),
			t==self.batteryPerf.topvar("t")]
		constraints += [self.electricalSystemPerf.topvar("P_{in}") == P_battery,
			self.electricalSystemPerf.topvar("P_{out}") >= P_cruise + P_tailRotor]
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

class OnDemandSizingMission(Model):
	#Mission the aircraft must be able to fly. No economic analysis.
    def setup(self,aircraft,reserve_type="FAA_heli",mission_type="piloted"):
		
		if not(aircraft.autonomousEnabled) and (mission_type != "piloted"):
			raise ValueError("Autonomy is not enabled for Aircraft() model.")
		
		W = Variable("W_{mission}","lbf","Weight of the aircraft during the mission")
		mission_range = Variable("mission_range","nautical_mile","Mission range")
		t_hover = Variable("t_{hover}","s","Time in hover")
		E_mission = Variable("E_{mission}","kWh","Electrical energy used during mission")
		V_cruise = Variable("V_{cruise}","mph","Aircraft cruising speed")
		V_loiter = Variable("V_{loiter}","mph","Aircraft loiter speed")
		T_A = Variable("T/A","lbf/ft**2","Disk loading")

		C_eff = aircraft.battery.C_eff #effective battery capacity

		self.W = W
		self.mission_range = mission_range
		self.t_hover = t_hover
		self.E_mission = E_mission
		self.V_cruise = V_cruise
		self.V_loiter = V_loiter
		self.T_A = T_A

		self.mission_type = mission_type

		self.crew = Crew(mission_type=mission_type)
		self.passengers = Passengers()
		
		hoverState = FlightState(h=0*ureg.ft)
		
		constraints = []
		
		self.fs0 = Hover(self,aircraft,hoverState) #takeoff
		self.fs1 = LevelFlight(self,aircraft) #fly to destination
		self.fs2 = LevelFlight(self,aircraft) #reserve

		constraints += [self.fs0.T_A == T_A]
		constraints += [self.fs1.L_D == aircraft.L_D_cruise]
		constraints += [self.fs1.V == V_cruise]

		#Reserve segment
		if reserve_type == "FAA_aircraft" or reserve_type == "FAA_heli":

			constraints += [self.fs2.L_D == aircraft.L_D_loiter]
			constraints += [self.fs2.V == V_loiter]
			
			if reserve_type == "FAA_aircraft":
				#30-minute loiter time, as per VFR rules for aircraft (daytime only)
				t_loiter = Variable("t_{loiter}",30,"minutes","Loiter time")
			elif reserve_type == "FAA_heli":
				#20-minute loiter time, as per VFR rules for helicopters
				t_loiter = Variable("t_{loiter}",20,"minutes","Loiter time")

			self.t_loiter = t_loiter
			constraints += [t_loiter == self.fs2.topvar("t")]

		if reserve_type == "Uber":#2-nautical-mile diversion distance; used by McDonald & German

			constraints += [self.fs2.L_D == aircraft.L_D_cruise]
			constraints += [self.fs2.V == V_cruise]

			R_divert = Variable("R_{divert}",2,"nautical_mile","Diversion distance")
			self.R_divert = R_divert
			constraints += [R_divert == self.fs2.topvar("segment_range")]
		
		self.fs3 = Hover(self,aircraft,hoverState)#landing again

		self.flight_segments = [self.fs0, self.fs1, self.fs2, self.fs3]
		self.levelFlight_segments = [self.fs1, self.fs2]
		self.hover_segments  = [self.fs0, self.fs3] #not including loiter
		
		#Power and energy consumption by mission segment
		with Vectorize(len(self.flight_segments)):
			P_battery = Variable("P_{battery}","kW","Segment power draw")
			E = Variable("E","kWh","Segment energy use")

		#Data from hover segments
		with Vectorize(len(self.hover_segments)):
			CT = Variable("CT","-","Thrust coefficient")
			CP = Variable("CP","-","Power coefficient")
			Q_perRotor = Variable("Q_perRotor","lbf*ft","Torque per lifting rotor")
			T_perRotor = Variable("T_perRotor","lbf","Thrust per lifting rotor")
			P = Variable("P","kW","Total power supplied to all lifting rotors")
			P_perRotor = Variable("P_perRotor","kW","Power per lifting rotor")
			VT = Variable("VT","ft/s","Propeller tip speed")
			omega = Variable("\omega","rpm","Propeller angular velocity")
			MT = Variable("MT","-","Propeller tip Mach number")
			FOM = Variable("FOM","-","Figure of merit")
			p_ratio = Variable("p_{ratio}","-","Sound pressure ratio in hover")

		constraints += [self.flight_segments]
		constraints += [self.crew, self.passengers]
		constraints += [W >= aircraft.topvar("W_{empty}") + self.passengers.topvar("W") \
			+ self.crew.topvar("W")]
		constraints += [aircraft.topvar("TOGW") >= W]
		
		constraints += [mission_range == self.fs1.topvar("segment_range")]
		constraints += [hoverState]
		
		constraints += [V_loiter == ((1/3.)**(1/4.))*V_cruise]

		constraints += [E_mission >= sum(c.topvar("E") for c in self.flight_segments)]
		constraints += [C_eff >= E_mission]

		constraints += [aircraft.tailRotor_power_fraction_levelFlight == segment.tailRotor_power_fraction \
			for i,segment in enumerate(self.levelFlight_segments)]
		constraints += [aircraft.tailRotor_power_fraction_hover == segment.tailRotor_power_fraction \
			for i,segment in enumerate(self.hover_segments)]
		constraints += [t_hover == segment.topvar("t") for i,segment in enumerate(self.hover_segments)]

		constraints += [P_battery[i] == segment.topvar("P_{battery}") for i,segment in enumerate(self.flight_segments)]
		constraints += [E[i] == segment.topvar("E") for i,segment in enumerate(self.flight_segments)]

		constraints += [CT[i] == segment.rotorPerf.topvar("CT") for i,segment in enumerate(self.hover_segments)]
		constraints += [CP[i] == segment.rotorPerf.topvar("CP") for i,segment in enumerate(self.hover_segments)]
		constraints += [Q_perRotor[i] == segment.rotorPerf.topvar("Q_perRotor") for i,segment in enumerate(self.hover_segments)]
		constraints += [T_perRotor[i] == segment.rotorPerf.topvar("T_perRotor") for i,segment in enumerate(self.hover_segments)]
		constraints += [P[i] == segment.rotorPerf.topvar("P") for i,segment in enumerate(self.hover_segments)]
		constraints += [P_perRotor[i] == segment.rotorPerf.topvar("P_perRotor") for i,segment in enumerate(self.hover_segments)]
		constraints += [VT[i] == segment.rotorPerf.topvar("VT") for i,segment in enumerate(self.hover_segments)]
		constraints += [omega[i] == segment.rotorPerf.topvar("\omega") for i,segment in enumerate(self.hover_segments)]
		constraints += [MT[i] == segment.rotorPerf.topvar("MT") for i,segment in enumerate(self.hover_segments)]
		constraints += [FOM[i] == segment.rotorPerf.topvar("FOM") for i,segment in enumerate(self.hover_segments)]
		constraints += [p_ratio[i] == segment.rotorPerf.topvar("p_{ratio}") for i,segment in enumerate(self.hover_segments)]

		return constraints

class OnDemandRevenueMission(Model):
	#Revenue-generating mission. Exactly the same code as OnDemandDeadheadMission.
    def setup(self,aircraft,mission_type="piloted"):

    	if not(aircraft.autonomousEnabled) and (mission_type != "piloted"):
    		raise ValueError("Autonomy is not enabled for Aircraft() model.")

    	W = Variable("W_{mission}","lbf","Weight of the aircraft during the mission")
    	mission_range = Variable("mission_range","nautical_mile","Mission range")
    	t_hover = Variable("t_{hover}","s","Time in hover")
    	V_cruise = Variable("V_{cruise}","mph","Aircraft cruising speed")
    	T_A = Variable("T/A","lbf/ft**2","Disk loading")
    	
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
        	Q_perRotor = Variable("Q_perRotor","lbf*ft","Torque per lifting rotor")
        	T_perRotor = Variable("T_perRotor","lbf","Thrust per lifting rotor")
        	P = Variable("P","kW","Total power supplied to all lifting rotors")
        	P_perRotor = Variable("P_perRotor","kW","Power per lifting rotor")
        	VT = Variable("VT","ft/s","Propeller tip speed")
        	omega = Variable("\omega","rpm","Propeller angular velocity")
        	MT = Variable("MT","-","Propeller tip Mach number")
        	FOM = Variable("FOM","-","Figure of merit")
        	p_ratio = Variable("p_{ratio}","-","Sound pressure ratio in hover")

        constraints = []

        constraints += [self.fs0.T_A == T_A]
        constraints += [self.fs1.L_D == aircraft.L_D_cruise]
        constraints += [self.fs1.V == V_cruise]

        constraints += [self.segments]
        constraints += [self.crew,self.passengers]

        constraints += [W >= aircraft.topvar("W_{empty}") + self.passengers.topvar("W") \
        	+ self.crew.topvar("W")]
        constraints += [aircraft.topvar("TOGW") >= W]
        
        constraints += [mission_range == self.fs1.topvar("segment_range")]
        constraints += [p_ratio == self.fs0.rotorPerf.topvar("p_{ratio}")]
        constraints += hoverState

        constraints += [E_mission >= sum(c.topvar("E") for c in self.flight_segments)]
        constraints += [C_eff >= E_mission]

        constraints += [aircraft.tailRotor_power_fraction_levelFlight == segment.tailRotor_power_fraction \
        	for i,segment in enumerate(self.levelFlight_segments)]
        constraints += [aircraft.tailRotor_power_fraction_hover == segment.tailRotor_power_fraction \
        	for i,segment in enumerate(self.hover_segments)]
        constraints += [t_hover == segment.topvar("t") for i,segment in enumerate(self.hover_segments)]

        constraints += [t_flight >= sum(c.topvar("t") for c in self.flight_segments)]
        constraints += [t_mission >= t_flight + self.time_on_ground.topvar("t")]

        constraints += [P_battery[i] == segment.topvar("P_{battery}") for i,segment in enumerate(self.flight_segments)]
        constraints += [E[i] == segment.topvar("E") for i,segment in enumerate(self.flight_segments)]

        constraints += [CT[i] == segment.rotorPerf.topvar("CT") for i,segment in enumerate(self.hover_segments)]
        constraints += [CP[i] == segment.rotorPerf.topvar("CP") for i,segment in enumerate(self.hover_segments)]
        constraints += [Q_perRotor[i] == segment.rotorPerf.topvar("Q_perRotor") for i,segment in enumerate(self.hover_segments)]
        constraints += [T_perRotor[i] == segment.rotorPerf.topvar("T_perRotor") for i,segment in enumerate(self.hover_segments)]
        constraints += [P[i] == segment.rotorPerf.topvar("P") for i,segment in enumerate(self.hover_segments)]
        constraints += [P_perRotor[i] == segment.rotorPerf.topvar("P_perRotor") for i,segment in enumerate(self.hover_segments)]
        constraints += [VT[i] == segment.rotorPerf.topvar("VT") for i,segment in enumerate(self.hover_segments)]
        constraints += [omega[i] == segment.rotorPerf.topvar("\omega") for i,segment in enumerate(self.hover_segments)]
        constraints += [MT[i] == segment.rotorPerf.topvar("MT") for i,segment in enumerate(self.hover_segments)]
        constraints += [FOM[i] == segment.rotorPerf.topvar("FOM") for i,segment in enumerate(self.hover_segments)]
        constraints += [p_ratio[i] == segment.rotorPerf.topvar("p_{ratio}") for i,segment in enumerate(self.hover_segments)]

        return constraints

class OnDemandDeadheadMission(Model):
	#Deadhead mission. Exactly the same code as OnDemandRevenueMission.
    def setup(self,aircraft,mission_type="piloted"):

    	if not(aircraft.autonomousEnabled) and (mission_type != "piloted"):
    		raise ValueError("Autonomy is not enabled for Aircraft() model.")

    	W = Variable("W_{mission}","lbf","Weight of the aircraft during the mission")
    	mission_range = Variable("mission_range","nautical_mile","Mission range")
    	t_hover = Variable("t_{hover}","s","Time in hover")
    	V_cruise = Variable("V_{cruise}","mph","Aircraft cruising speed")
    	T_A = Variable("T/A","lbf/ft**2","Disk loading")
    	
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
        	Q_perRotor = Variable("Q_perRotor","lbf*ft","Torque per lifting rotor")
        	T_perRotor = Variable("T_perRotor","lbf","Thrust per lifting rotor")
        	P = Variable("P","kW","Total power supplied to all lifting rotors")
        	P_perRotor = Variable("P_perRotor","kW","Power per lifting rotor")
        	VT = Variable("VT","ft/s","Propeller tip speed")
        	omega = Variable("\omega","rpm","Propeller angular velocity")
        	MT = Variable("MT","-","Propeller tip Mach number")
        	FOM = Variable("FOM","-","Figure of merit")
        	p_ratio = Variable("p_{ratio}","-","Sound pressure ratio in hover")

        constraints = []

        constraints += [self.fs0.T_A == T_A]
        constraints += [self.fs1.L_D == aircraft.L_D_cruise]
        constraints += [self.fs1.V == V_cruise]

        constraints += [self.segments]
        constraints += [self.crew,self.passengers]

        constraints += [W >= aircraft.topvar("W_{empty}") + self.passengers.topvar("W") \
        	+ self.crew.topvar("W")]
        constraints += [aircraft.topvar("TOGW") >= W]
        
        constraints += [mission_range == self.fs1.topvar("segment_range")]
        constraints += [p_ratio == self.fs0.rotorPerf.topvar("p_{ratio}")]
        constraints += hoverState

        constraints += [E_mission >= sum(c.topvar("E") for c in self.flight_segments)]
        constraints += [C_eff >= E_mission]

        constraints += [aircraft.tailRotor_power_fraction_levelFlight == segment.tailRotor_power_fraction \
        	for i,segment in enumerate(self.levelFlight_segments)]
        constraints += [aircraft.tailRotor_power_fraction_hover == segment.tailRotor_power_fraction \
        	for i,segment in enumerate(self.hover_segments)]
        constraints += [t_hover == segment.topvar("t") for i,segment in enumerate(self.hover_segments)]

        constraints += [t_flight >= sum(c.topvar("t") for c in self.flight_segments)]
        constraints += [t_mission >= t_flight + self.time_on_ground.topvar("t")]

        constraints += [P_battery[i] == segment.topvar("P_{battery}") for i,segment in enumerate(self.flight_segments)]
        constraints += [E[i] == segment.topvar("E") for i,segment in enumerate(self.flight_segments)]

        constraints += [CT[i] == segment.rotorPerf.topvar("CT") for i,segment in enumerate(self.hover_segments)]
        constraints += [CP[i] == segment.rotorPerf.topvar("CP") for i,segment in enumerate(self.hover_segments)]
        constraints += [Q_perRotor[i] == segment.rotorPerf.topvar("Q_perRotor") for i,segment in enumerate(self.hover_segments)]
        constraints += [T_perRotor[i] == segment.rotorPerf.topvar("T_perRotor") for i,segment in enumerate(self.hover_segments)]
        constraints += [P[i] == segment.rotorPerf.topvar("P") for i,segment in enumerate(self.hover_segments)]
        constraints += [P_perRotor[i] == segment.rotorPerf.topvar("P_perRotor") for i,segment in enumerate(self.hover_segments)]
        constraints += [VT[i] == segment.rotorPerf.topvar("VT") for i,segment in enumerate(self.hover_segments)]
        constraints += [omega[i] == segment.rotorPerf.topvar("\omega") for i,segment in enumerate(self.hover_segments)]
        constraints += [MT[i] == segment.rotorPerf.topvar("MT") for i,segment in enumerate(self.hover_segments)]
        constraints += [FOM[i] == segment.rotorPerf.topvar("FOM") for i,segment in enumerate(self.hover_segments)]
        constraints += [p_ratio[i] == segment.rotorPerf.topvar("p_{ratio}") for i,segment in enumerate(self.hover_segments)]

        return constraints

class OnDemandMissionCost(Model):
	#Includes both revenue and deadhead missions
	def setup(self,aircraft,revenue_mission,deadhead_mission):

		N_passengers = revenue_mission.passengers.topvar("N_{passengers}")
		trip_distance = revenue_mission.topvar("mission_range")

		cpt = Variable("cost_per_trip","-","Cost (in dollars) for one trip")
		cpt_revenue = Variable("revenue_cost_per_trip","-",
			"Portion of the cost per trip incurred during the revenue-generating flights")
		cpt_deadhead = Variable("deadhead_cost_per_trip","-",
			"Portion of the cost per trip incurred during the deadhead flights")
		cptpp = Variable("cost_per_trip_per_passenger","-",
			"Cost (in dollars) for one trip, per passenger carried on revenue trip")
		cpt_seat_mile = Variable("cost_per_seat_mile","mile**-1",
			"Cost per trip, per seat (passenger) mile")
		deadhead_ratio = Variable("deadhead_ratio","-","Number of deadhead missions per total missions")
		NdNr = Variable("N_{deadhead}/N_{typical}","-",
			"Number of deadhead missions per typical mission")

		revenue_mission_costs = RevenueMissionCost(aircraft,revenue_mission)
		deadhead_mission_costs = DeadheadMissionCost(aircraft,deadhead_mission)

		self.cpt = cpt
		self.cpt_revenue = cpt_revenue
		self.cpt_deadhead = cpt_deadhead
		self.cptpp = cptpp
		self.cpt_seat_mile = cpt_seat_mile
		self.deadhead_ratio = deadhead_ratio
		self.NdNr = NdNr
		self.revenue_mission_costs = revenue_mission_costs
		self.deadhead_mission_costs = deadhead_mission_costs

		constraints = []
		constraints += [revenue_mission_costs, deadhead_mission_costs]

		constraints += [NdNr >= deadhead_ratio*(NdNr+1)]
		
		constraints += [cpt_revenue == revenue_mission_costs.topvar("cost_per_mission")]
		constraints += [cpt_deadhead == NdNr*deadhead_mission_costs.topvar("cost_per_mission")]
		constraints += [cpt >= cpt_revenue + cpt_deadhead]
		
		constraints += [cpt == cptpp*N_passengers]
		constraints += [cpt == cpt_seat_mile*N_passengers*trip_distance]
		
		return constraints

class RevenueMissionCost(Model):
	#Cost for one mission. Exactly the same code as DeadheadMissionCost.
	def setup(self,aircraft,mission):

		t_mission = mission.t_mission

		cost_per_mission = Variable("cost_per_mission","-","Cost per mission")
		cost_per_time = Variable("cost_per_time","hr**-1","Cost per unit mission time")

		capital_expenses = CapitalExpenses(aircraft,mission)
		operating_expenses = OperatingExpenses(aircraft,mission)
		expenses = [capital_expenses, operating_expenses]

		self.cost_per_mission = cost_per_mission
		self.cost_per_time = cost_per_time
		self.capital_expenses = capital_expenses
		self.operating_expenses = operating_expenses

		constraints = []
		
		constraints += [expenses]
		constraints += [cost_per_mission >= sum(c.topvar("cost_per_mission") for c in expenses)]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints

class DeadheadMissionCost(Model):
	#Cost for one mission. Exactly the same code as RevenueMissionCost.
	def setup(self,aircraft,mission):

		t_mission = mission.t_mission

		cost_per_mission = Variable("cost_per_mission","-","Cost per mission")
		cost_per_time = Variable("cost_per_time","hr**-1","Cost per unit mission time")

		capital_expenses = CapitalExpenses(aircraft,mission)
		operating_expenses = OperatingExpenses(aircraft,mission)
		expenses = [capital_expenses, operating_expenses]

		self.cost_per_mission = cost_per_mission
		self.cost_per_time = cost_per_time
		
		self.capital_expenses = capital_expenses
		self.operating_expenses = operating_expenses

		constraints = []
		
		constraints += [expenses]
		constraints += [cost_per_mission >= sum(c.topvar("cost_per_mission") for c in expenses)]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints

class VehicleAcquisitionCost(Model):
	def setup(self,aircraft,mission):
		
		t_mission = mission.t_mission
		purchase_price = aircraft.purchase_price
		vehicle_life = aircraft.vehicle_life
		
		cost_per_time = Variable("cost_per_time","hr**-1",
			"Amortized vehicle purchase price per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-",
			"Amortized vehicle acquisition cost per mission")

		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission
		
		constraints = []

		constraints += [cost_per_time == purchase_price/vehicle_life]
		constraints += [cost_per_mission == t_mission*cost_per_time]
		
		return constraints

class AvionicsAcquisitionCost(Model):
	def setup(self,aircraft,mission):
		
		t_mission = mission.t_mission
		purchase_price = aircraft.avionics.purchase_price
		vehicle_life = aircraft.vehicle_life
		
		cost_per_time = Variable("cost_per_time","hr**-1",
			"Amortized avionics purchase price per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-",
			"Amortized avionics acquisition cost per mission")

		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		constraints = []

		constraints += [cost_per_time == purchase_price/vehicle_life]
		constraints += [cost_per_mission == t_mission*cost_per_time]
		
		return constraints

class BatteryAcquisitionCost(Model):
	def setup(self,battery,mission):
		
		t_mission = mission.t_mission
		purchase_price = battery.purchase_price
		cycle_life = battery.cycle_life
		
		cost_per_time = Variable("cost_per_time","hr**-1",
			"Amortized battery purchase price per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-",
			"Amortized battery cost per mission")

		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		constraints = []

		constraints += [cost_per_mission == purchase_price/cycle_life]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints

class CapitalExpenses(Model):
	def setup(self,aircraft,mission):

		t_mission = mission.t_mission

		cost_per_time = Variable("cost_per_time","hr**-1","Capital expenses per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Capital expenses per mission")

		vehicle_cost = VehicleAcquisitionCost(aircraft,mission)
		avionics_cost = AvionicsAcquisitionCost(aircraft,mission)
		battery_cost = BatteryAcquisitionCost(aircraft.battery,mission)

		self.costs = [vehicle_cost, avionics_cost, battery_cost]

		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		self.vehicle_cost = vehicle_cost
		self.avionics_cost = avionics_cost
		self.battery_cost = battery_cost

		constraints = []
		constraints += [self.costs]
		
		constraints += [cost_per_mission >= sum(c.topvar("cost_per_mission") for c in self.costs)]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints


class PilotCost(Model):
	def setup(self,mission):

		t_mission = mission.t_mission

		wrap_rate = Variable("wrap_rate","hr**-1",
			"Cost per pilot, per unit mission time (including benefits and overhead)")
		cost_per_time = Variable("cost_per_time","hr**-1","Pilot cost per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Pilot cost per mission")

		self.wrap_rate = wrap_rate
		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		constraints = []
		
		if mission.mission_type == "autonomous":
			aircraft_per_bunker_pilot = Variable("aircraft_per_bunker_pilot",8,"-",
				"Number of aircraft controlled by 1 bunker pilot (assuming no crew on board)")
			constraints += [cost_per_time == wrap_rate/aircraft_per_bunker_pilot]

		if mission.mission_type == "piloted":
			pilots_per_aircraft = Variable("pilots_per_aircraft",1.5,"-",
				"Pilots per aircraft (assuming crew on board)")
			constraints += [cost_per_time == wrap_rate*pilots_per_aircraft]

		constraints += [cost_per_mission == t_mission*cost_per_time]
		
		return constraints

class MaintenanceCost(Model):
	def setup(self,mission):

		t_mission = mission.t_mission

		MMH_FH = Variable("MMH_FH","-","Maintenance man-hours per flight hour")
		wrap_rate = Variable("wrap_rate","hr**-1",
			"Cost per mechanic, per unit maintenance time (including benefits and overhead)")

		cost_per_time = Variable("cost_per_time","hr**-1","Maintenance cost per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Maintenance cost per mission")

		self.MMH_FH = MMH_FH
		self.wrap_rate = wrap_rate
		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		constraints = []

		constraints += [cost_per_time == MMH_FH*wrap_rate]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints

class EnergyCost(Model):
	def setup(self,mission):

		t_mission = mission.t_mission
		E_charger = mission.time_on_ground.E_charger

		cost_per_energy = Variable("cost_per_energy",0.12,"kWh**-1","Price of electricity")
		cost_per_time = Variable("cost_per_time","hr**-1","Energy cost per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Energy cost per mission")

		self.cost_per_energy = cost_per_energy
		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		constraints = []

		constraints += [cost_per_mission == E_charger*cost_per_energy]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints

class IndirectOperatingCost(Model):
	def setup(self,operating_expenses):

		IOC_fraction = Variable("IOC_fraction",0.12,"-","IOC as a fraction of DOC")
		cost_per_time = Variable("cost_per_time","hr**-1","IOC per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","IOC per mission")

		self.IOC_fraction = IOC_fraction
		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		constraints = []

		constraints += [cost_per_mission == IOC_fraction*operating_expenses.DOC]
		constraints += [cost_per_time == IOC_fraction*operating_expenses.DOC_per_time]

		return constraints

class OperatingExpenses(Model):
	def setup(self,aircraft,mission):

		t_mission = mission.t_mission

		cost_per_time = Variable("cost_per_time","hr**-1","Operating expenses per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Operating expenses per mission")

		DOC = Variable("DOC","-","Direct operating cost per mission")
		DOC_per_time = Variable("DOC_per_time","hr**-1","Direct operating cost per unit mission time")
		IOC = Variable("IOC","-","Indirect operating cost per mission")
		IOC_per_time = Variable("IOC_per_time","hr**-1","Indirect operating cost per unit mission time")

		self.DOC = DOC
		self.DOC_per_time = DOC_per_time
		self.IOC = IOC
		self.IOC_per_time = IOC_per_time

		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		pilot_cost = PilotCost(mission)
		maintenance_cost = MaintenanceCost(mission)
		energy_cost = EnergyCost(mission)
		indirect_operating_cost = IndirectOperatingCost(self)

		self.pilot_cost = pilot_cost
		self.maintenance_cost = maintenance_cost
		self.energy_cost = energy_cost
		self.indirect_operating_cost = indirect_operating_cost

		constraints = []
		constraints += [pilot_cost, maintenance_cost, energy_cost, indirect_operating_cost]

		constraints += [DOC >= pilot_cost.topvar("cost_per_mission") 
			+ maintenance_cost.topvar("cost_per_mission") + energy_cost.topvar("cost_per_mission")]
		constraints += [DOC_per_time == DOC/t_mission]

		constraints += [IOC == indirect_operating_cost.topvar("cost_per_mission")]
		constraints += [IOC_per_time == indirect_operating_cost.topvar("cost_per_time")]

		constraints += [cost_per_mission >= DOC + IOC]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints


def test():
	#String inputs
	reserve_type="FAA_heli"
	sizing_mission_type="piloted"
	revenue_mission_type="piloted"
	deadhead_mission_type="autonomous"

	Aircraft = OnDemandAircraft(autonomousEnabled=True)
	Aircraft_subDict = {
		Aircraft.L_D_cruise: 14., #estimated L/D in cruise
		Aircraft.eta_cruise: 0.85, #propulsive efficiency in cruise
		Aircraft.cost_per_weight: 350*ureg.lbf**-1, #vehicle cost per unit empty weight
		Aircraft.battery.cost_per_C: 400*ureg.kWh**-1, #battery cost per unit energy capacity
		Aircraft.rotors.N: 12, #number of propellers
		Aircraft.rotors.Cl_mean_max: 1.0, #maximum allowed mean lift coefficient
		Aircraft.battery.C_m: 400*ureg.Wh/ureg.kg, #battery energy density
		Aircraft.structure.weight_fraction: 0.55, #empty weight fraction
		Aircraft.electricalSystem.eta: 0.9, #electrical system efficiency	
	}
	Aircraft.substitutions.update(Aircraft_subDict)

	SizingMission = OnDemandSizingMission(Aircraft,mission_type=sizing_mission_type,
		reserve_type=reserve_type)
	sizingMission_subDict = {
		SizingMission.mission_range: 87*ureg.nautical_mile,#mission range
		SizingMission.V_cruise: 200*ureg.mph,#cruising speed
		SizingMission.t_hover: 120*ureg.s,#hover time
		SizingMission.T_A: 15.*ureg("lbf")/ureg("ft")**2,#disk loading
		SizingMission.passengers.N_passengers: 3,#Number of passengers
	}
	SizingMission.substitutions.update(sizingMission_subDict)

	RevenueMission = OnDemandRevenueMission(Aircraft,mission_type=revenue_mission_type)
	revenueMission_subDict = {
		RevenueMission.mission_range: 30*ureg.nautical_mile,#mission range
		RevenueMission.V_cruise: 200*ureg.mph,#cruising speed
		RevenueMission.t_hover: 30*ureg.s,#hover time
		RevenueMission.passengers.N_passengers: 2,#Number of passengers
		RevenueMission.time_on_ground.charger_power: 200*ureg.kW, #Charger power
	}
	RevenueMission.substitutions.update(revenueMission_subDict)

	DeadheadMission = OnDemandDeadheadMission(Aircraft,mission_type=deadhead_mission_type)
	deadheadMission_subDict = {
		DeadheadMission.mission_range: 30*ureg.nautical_mile,#mission range
		DeadheadMission.V_cruise: 200*ureg.mph,#cruising speed
		DeadheadMission.t_hover: 30*ureg.s,#hover time
		DeadheadMission.passengers.N_passengers: 0.00001,#Number of passengers
		DeadheadMission.time_on_ground.charger_power: 200*ureg.kW, #Charger power
	}
	DeadheadMission.substitutions.update(deadheadMission_subDict)

	MissionCost = OnDemandMissionCost(Aircraft,RevenueMission,DeadheadMission)
	missionCost_subDict = {
		MissionCost.revenue_mission_costs.operating_expenses.pilot_cost.wrap_rate: 70*ureg.hr**-1,#pilot wrap rate
		MissionCost.revenue_mission_costs.operating_expenses.maintenance_cost.wrap_rate: 60*ureg.hr**-1, #mechanic wrap rate
		MissionCost.revenue_mission_costs.operating_expenses.maintenance_cost.MMH_FH: 0.6, #maintenance man-hours per flight hour
		MissionCost.deadhead_mission_costs.operating_expenses.pilot_cost.wrap_rate: 70*ureg.hr**-1,#pilot wrap rate
		MissionCost.deadhead_mission_costs.operating_expenses.maintenance_cost.wrap_rate: 60*ureg.hr**-1, #mechanic wrap rate
		MissionCost.deadhead_mission_costs.operating_expenses.maintenance_cost.MMH_FH: 0.6, #maintenance man-hours per flight hour
		MissionCost.deadhead_ratio: 0.2, #deadhead ratio
	}
	MissionCost.substitutions.update(missionCost_subDict)

	
	problem = Model(MissionCost["cost_per_trip"],
		[Aircraft, SizingMission, RevenueMission, DeadheadMission, MissionCost])
	
	solution = problem.solve(verbosity=0)
	return solution
	


if __name__=="__main__":

	#Concept representative analysis

	from noise_models import rotational_noise, vortex_noise, noise_weighting
	
	#String inputs
	reserve_type="FAA_heli"
	sizing_mission_type="piloted"
	revenue_mission_type="piloted"
	deadhead_mission_type="autonomous"

	Aircraft = OnDemandAircraft(autonomousEnabled=True)
	Aircraft_subDict = {
		Aircraft.L_D_cruise: 14., #estimated L/D in cruise
		Aircraft.eta_cruise: 0.85, #propulsive efficiency in cruise
		Aircraft.tailRotor_power_fraction_hover: 0.0001,
		Aircraft.tailRotor_power_fraction_levelFlight: 0.0001,
		Aircraft.cost_per_weight: 350*ureg.lbf**-1, #vehicle cost per unit empty weight
		Aircraft.battery.cost_per_C: 400*ureg.kWh**-1, #battery cost per unit energy capacity
		Aircraft.rotors.N: 12, #number of propellers
		Aircraft.rotors.Cl_mean_max: 1.0, #maximum allowed mean lift coefficient
		Aircraft.battery.C_m: 400*ureg.Wh/ureg.kg, #battery energy density
		Aircraft.structure.weight_fraction: 0.55, #empty weight fraction
		Aircraft.electricalSystem.eta: 0.9, #electrical system efficiency	
	}
	Aircraft.substitutions.update(Aircraft_subDict)

	SizingMission = OnDemandSizingMission(Aircraft,mission_type=sizing_mission_type,
		reserve_type=reserve_type)
	sizingMission_subDict = {
		SizingMission.mission_range: 87*ureg.nautical_mile,#mission range
		SizingMission.V_cruise: 200*ureg.mph,#cruising speed
		SizingMission.t_hover: 120*ureg.s,#hover time
		SizingMission.T_A: 15.*ureg("lbf")/ureg("ft")**2,#disk loading
		SizingMission.passengers.N_passengers: 3,#Number of passengers
	}
	SizingMission.substitutions.update(sizingMission_subDict)

	RevenueMission = OnDemandRevenueMission(Aircraft,mission_type=revenue_mission_type)
	revenueMission_subDict = {
		RevenueMission.mission_range: 30*ureg.nautical_mile,#mission range
		RevenueMission.V_cruise: 200*ureg.mph,#cruising speed
		RevenueMission.t_hover: 30*ureg.s,#hover time
		RevenueMission.passengers.N_passengers: 2,#Number of passengers
		RevenueMission.time_on_ground.charger_power: 200*ureg.kW, #Charger power
	}
	RevenueMission.substitutions.update(revenueMission_subDict)

	DeadheadMission = OnDemandDeadheadMission(Aircraft,mission_type=deadhead_mission_type)
	deadheadMission_subDict = {
		DeadheadMission.mission_range: 30*ureg.nautical_mile,#mission range
		DeadheadMission.V_cruise: 200*ureg.mph,#cruising speed
		DeadheadMission.t_hover: 30*ureg.s,#hover time
		DeadheadMission.passengers.N_passengers: 0.00001,#Number of passengers
		DeadheadMission.time_on_ground.charger_power: 200*ureg.kW, #Charger power
	}
	DeadheadMission.substitutions.update(deadheadMission_subDict)

	MissionCost = OnDemandMissionCost(Aircraft,RevenueMission,DeadheadMission)
	missionCost_subDict = {
		MissionCost.revenue_mission_costs.operating_expenses.pilot_cost.wrap_rate: 70*ureg.hr**-1,#pilot wrap rate
		MissionCost.revenue_mission_costs.operating_expenses.maintenance_cost.wrap_rate: 60*ureg.hr**-1, #mechanic wrap rate
		MissionCost.revenue_mission_costs.operating_expenses.maintenance_cost.MMH_FH: 0.6, #maintenance man-hours per flight hour
		MissionCost.deadhead_mission_costs.operating_expenses.pilot_cost.wrap_rate: 70*ureg.hr**-1,#pilot wrap rate
		MissionCost.deadhead_mission_costs.operating_expenses.maintenance_cost.wrap_rate: 60*ureg.hr**-1, #mechanic wrap rate
		MissionCost.deadhead_mission_costs.operating_expenses.maintenance_cost.MMH_FH: 0.6, #maintenance man-hours per flight hour
		MissionCost.deadhead_ratio: 0.2, #deadhead ratio
	}
	MissionCost.substitutions.update(missionCost_subDict)

	problem = Model(MissionCost["cost_per_trip"],
		[Aircraft, SizingMission, RevenueMission, DeadheadMission, MissionCost])
	solution = problem.solve(verbosity=0)

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

	if (reserve_type == "FAA_aircraft") or (reserve_type == "FAA_heli"):
		num = solution("t_{loiter}_OnDemandSizingMission").to(ureg.minute).magnitude
		reserve_type_string = " (%0.0f-minute loiter time)" % num
	if reserve_type == "Uber":
		num = solution("R_{divert}_OnDemandSizingMission").to(ureg.nautical_mile).magnitude
		reserve_type_string = " (%0.1f-nm diversion distance)" % num
	
	print
	print "Concept representative analysis"
	print
	print "Battery energy density: %0.0f Wh/kg" \
		% solution("C_m_OnDemandAircraft/Battery").to(ureg.Wh/ureg.kg).magnitude
	print "Empty weight fraction: %0.4f" \
		% solution("weight_fraction_OnDemandAircraft/Structure")
	print "Cruise lift-to-drag ratio: %0.1f" \
		% solution("L_D_cruise_OnDemandAircraft")
	print "Hover disk loading: %0.1f lbf/ft^2" \
		% solution("T/A_OnDemandSizingMission").to(ureg("lbf/ft**2")).magnitude
	print "Rotor maximum mean lift coefficient: %0.2f" \
		% solution("Cl_{mean_{max}}_OnDemandAircraft/Rotors")
	print "Cruise propulsive efficiency: %0.2f" \
		% solution("\eta_{cruise}_OnDemandAircraft")
	print "Electrical system efficiency: %0.2f" \
		% solution("\eta_OnDemandAircraft/ElectricalSystem")
	print "Observer distance: %0.0f ft" % delta_S.to(ureg.ft).magnitude
	print "Noise weighting type: %s" % noise_weighting
	print
	print "Sizing Mission (%s)" % sizing_mission_type
	print "Mission range: %0.0f nm" % \
		solution("mission_range_OnDemandSizingMission").to(ureg.nautical_mile).magnitude
	print "Number of passengers: %0.1f" % \
		solution("N_{passengers}_OnDemandSizingMission/Passengers")
	print "Reserve type: " + reserve_type + reserve_type_string
	print "Vehicle weight during mission: %0.0f lbf" % \
		solution("W_{mission}_OnDemandSizingMission").to(ureg.lbf).magnitude
	print "SPL in hover: %0.1f dB" % SPL_dict["Sizing"]
	print
	print "Revenue-Generating Mission (%s)" % revenue_mission_type
	print "Mission range: %0.0f nm" % \
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
	print "Mission range: %0.0f nm" % \
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
		solution("TOGW_OnDemandAircraft").to(ureg.lbf).magnitude
	print "Empty weight: %0.0f lbs" % \
		solution("W_OnDemandAircraft/Structure").to(ureg.lbf).magnitude
	print "Battery weight: %0.0f lbs" % \
		solution("W_OnDemandAircraft/Battery").to(ureg.lbf).magnitude
	print "Vehicle purchase price: $%0.0f " % \
		solution("purchase_price_OnDemandAircraft")
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
	
	#print solution.summary()
