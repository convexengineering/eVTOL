#Top-level aircraft model.

import math
import numpy as np
from gpkit import Variable, Model, Vectorize, ureg
from standard_atmosphere import stdatmo

class OnDemandAircraft(Model):
	def setup(self,N,L_D,eta_cruise,weight_fraction,C_m,Cl_mean_max,n=1.,eta_electric=0.9,
		cost_per_weight=350*ureg.lbf**-1,vehicle_life=20000*ureg.hour,cost_per_C=400*ureg.kWh**-1,
		autonomousEnabled="No"):
		
		MTOW = Variable("MTOW","lbf","Takeoff weight")
		W_empty = Variable("W_{empty}","lbf","Weight without passengers or crew")
		C_eff = Variable("C_{eff}","kWh","Effective battery capacity")
		g = Variable("g",9.807,"m/s**2","Gravitational acceleration")
		L_D = Variable("L_D",L_D,"-","Cruise L/D ratio")
		eta_cruise = Variable("\eta_{cruise}",eta_cruise,"-","Cruise propulsive efficiency")

		cost_per_weight = Variable("cost_per_weight",cost_per_weight,"lbf**-1",
			"Cost per unit empty weight of the aircraft")
		purchase_price = Variable("purchase_price","-","Purchase price of the aircraft")
		vehicle_life = Variable("vehicle_life",vehicle_life,"hours","Vehicle lifetime")

		self.MTOW = MTOW
		self.autonomousEnabled = autonomousEnabled

		self.rotors = Rotors(N=N,Cl_mean_max=Cl_mean_max)
		self.battery = Battery(C_m=C_m,n=n,cost_per_C=cost_per_C)
		self.structure = Structure(self,weight_fraction)
		self.powerSystem = PowerSystem(eta=eta_electric)
		self.avionics = Avionics(autonomousEnabled=autonomousEnabled)
		
		self.components = [self.rotors,self.battery,self.structure,self.powerSystem,self.avionics]
		
		constraints = []
		
		constraints += [g == self.battery.topvar("g")]
		constraints += [self.components]#all constraints implemented at component level
		constraints += [C_eff == self.battery.topvar("C_{eff}")]#battery-capacity constraint
		constraints += [W_empty >= sum(c.topvar("W") for c in self.components)]#weight constraint
		constraints += [purchase_price == cost_per_weight*self.structure.topvar("W")]

		return constraints

class Structure(Model):
	def setup(self,aircraft,weight_fraction):
		W = Variable("W","lbf","Structural weight")
		weight_fraction = Variable("weight_fraction",weight_fraction,"-","Structural weight fraction")

		return [W == weight_fraction*aircraft.MTOW]


class Rotors(Model):

	def performance(self,flightState,MT_max=0.9,SPL_req=100):
		return RotorsAero(self,flightState,MT_max,SPL_req)

	def setup(self,N=1,s=0.1,Cl_mean_max=1.0):
		R = Variable("R","ft","Propeller radius")
		D = Variable("D","ft","Propeller diameter")
		A = Variable("A","ft^2","Area of 1 rotor disk")
		A_total = Variable("A_{total}","ft^2","Combined area of all rotor disks")
		N = Variable("N",N,"-","Number of rotors")
		s = Variable("s",s,"-","Propeller solidity")
		Cl_mean_max = Variable("Cl_{mean_{max}}",Cl_mean_max,"-",
			"Maximum allowed mean lift coefficient")

		W = Variable("W",0,"lbf","Rotor weight") #weight model not implemented yet

		constraints = [A == math.pi*R**2, D==2*R, N==N, s==s, A_total==N*A, 
			Cl_mean_max == Cl_mean_max]

		return constraints

class RotorsAero(Model):
	def setup(self,rotors,flightState,MT_max=0.9,SPL_req=150):
		T = Variable("T","lbf","Total thrust")
		T_perRotor = Variable("T_perRotor","lbf","Thrust per rotor")
		T_A = Variable("T/A","lbf/ft**2","Disk loading")
		P = Variable("P","kW","Total power")
		P_perRotor = Variable("P_perRotor","kW","Power per rotor")
		VT = Variable("VT","ft/s","Propeller tip speed")
		omega = Variable("\omega","rpm","Propeller angular velocity")
		MT = Variable("MT","-","Propeller tip Mach number")
		MT_max = Variable("MT_max",MT_max,"-","Maximum allowed tip Mach number")

		CT = Variable("CT","-","Thrust coefficient")
		CP = Variable("CP","-","Power coefficient")
		CPi = Variable("CPi","-","Induced (ideal) power coefficient")
		CPp = Variable("CP","-","Profile power coefficient")
		Cl_mean = Variable("Cl_mean","-","Mean lift coefficient")
		FOM = Variable("FOM","-","Figure of merit")

		ki = Variable("ki",1.1,"-","Induced power factor")
		Cd0 = Variable("Cd0",0.01,"-","Blade two-dimensional zero-lift drag coefficient")

		p_ratio = Variable("p_{ratio}","-","Sound pressure ratio (p/p_{ref})")
		p_ratio_max = Variable("p_{ratio_max}",10**(SPL_req/20.),"-","Max allowed sound pressure ratio")
		x = Variable("x",500,"ft","Distance from source at which to calculate sound")
		k3 = Variable("k3",6.804e-3,"s**3/ft**3","Sound-pressure constant")

		R = rotors.topvar("R")
		A = rotors.topvar("A")
		A_total = rotors.topvar("A_{total}")
		N = rotors.topvar("N")
		s = rotors.topvar("s")
		Cl_mean_max = rotors.topvar("Cl_{mean_{max}}")

		rho = flightState.topvar("\rho")
		a = flightState.topvar("a")

		constraints = [flightState]

		#Top-level constraints
		constraints += [T == N * T_perRotor,
			P == N * P_perRotor]
		constraints += [T_perRotor == 0.5*rho*(VT**2)*A*CT,
			P_perRotor == 0.5*rho*(VT**3)*A*CP]
		constraints += [T_A == T/A_total]

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
		constraints += [p_ratio == k3*((T*omega)/(rho*x))*(N*s)**-0.5,
			p_ratio <= p_ratio_max]

		return constraints

class Battery(Model):

	def performance(self):
		return BatteryPerformance(self)

	#Requires a substitution or constraint for g (gravitational acceleration)
	def setup(self,C_m=350*ureg.Wh/ureg.kg,usable_energy_fraction=0.8,P_m=3000*ureg.W/ureg.kg,
		n=1.,cost_per_C=400*ureg.kWh**-1):
		
		g = Variable("g","m/s**2","Gravitational acceleration")
		
		C = Variable("C","kWh","Battery capacity")
		C_eff = Variable("C_{eff}","kWh","Effective battery capacity")
		usable_energy_fraction = Variable("usable_energy_fraction",usable_energy_fraction,
			"-","Percentage of the battery energy that can be used (without damaging battery)")
	
		W = Variable("W","lbf","Battery weight")
		m = Variable("m","kg","Battery mass")
		C_m = Variable("C_m",C_m,"Wh/kg","Battery energy density")
		P_m = Variable("P_m",P_m,"W/kg","Battery power density")
		P_max = Variable("P_{max}","kW","Battery maximum power")

		cost_per_C = Variable("cost_per_C",cost_per_C,"kWh**-1",
			"Battery cost per unit energy stored")
		purchase_price = Variable("purchase_price","-","Purchase price of the battery")
		cycle_life = Variable("cycle_life",2000,"-",
			"Number of cycles before battery needs replacement")

		self.P_max = P_max
		self.n = n #battery discharge parameter (needed for Peukert effect)

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
		Rt = Variable("Rt",1.,"hr","Battery hour rating")

		self.t = t

		constraints = [E==P*Rt*((t/Rt)**(1/battery.n)), P<=battery.P_max]
		return constraints

class Crew(Model):
	def setup(self,mission_type="piloted",W_oneCrew=190*ureg.lbf):
		
		W_oneCrew = Variable("W_{oneCrew}",W_oneCrew,"lbf","Weight of 1 crew member")
		N_crew = Variable("N_{crew}",1,"-","Number of crew members (if present)")

		constraints = []

		if mission_type == "autonomous":
			W = Variable("W",0,"lbf","Total weight")
		if mission_type == "piloted":
			W = Variable("W","lbf","Total weight")
			constraints += [W == N_crew*W_oneCrew]

		return constraints

class Passengers(Model):
	def setup(self,W_onePassenger=200*ureg.lbf,N_passengers=1):
		W_onePassenger = Variable("W_{onePassenger}",W_onePassenger,
			"lbf","Weight of 1 passenger")
		N_passengers = Variable("N_{passengers}",N_passengers,"-","Number of passengers")
		W = Variable("W","lbf","Total weight")

		return [W == N_passengers*W_onePassenger]

class PowerSystem(Model):
	def performance(self):
		return PowerSystemPerformance(self)

	def setup(self,eta=0.9):
		W = Variable("W",0,"lbf","Electrical power system weight")
		eta = Variable("eta",eta,"-","Electrical power system efficiency")

		self.eta = eta

		constraints = []
		constraints += [W==W, eta==eta]
		return constraints

class PowerSystemPerformance(Model):
	def setup(self,powerSystem):
		P_in = Variable("P_{in}","kW","Input power (from the battery)")
		P_out = Variable("P_{out}","kW","Output power (to the motor or motors)")

		constraints = []
		constraints += [P_out == powerSystem.eta*P_in]
		return constraints

class Avionics(Model):
	def setup(self,autonomousEnabled="No"):

		W = Variable("W",0,"lbf","Weight of the avionics")

		if autonomousEnabled == "Yes":
			purchase_price = Variable("purchase_price",60000,"-",
				"Purchase price of the avionics (Uber estimate)")
		if autonomousEnabled == "No":
			purchase_price = Variable("purchase_price",1,"-",
				"Purchase price of the avionics (negligibly small)")

		constraints = []
		return constraints


class FlightState(Model):
	def setup(self,h):
		
		atmospheric_data = stdatmo(h)
		rho = atmospheric_data["\rho"].to(ureg.kg/ureg.m**3)
		a = atmospheric_data["a"].to(ureg.ft/ureg.s)
		rho = Variable("\rho",rho,"kg/m^3","Air density")
		a = Variable("a",a,"ft/s","Speed of sound")

		constraints = []
		return constraints

class Hover(Model):
	def setup(self,mission,aircraft,state,t=120*ureg.s):
		E = Variable("E","kWh","Electrical energy used during hover segment")
		P_battery = Variable("P_{battery}","kW","Power drawn (from batteries) during hover segment")
		P_rotors  = Variable("P_{rotors}","kW","Power used (by rotors) during hover segment")
		T = Variable("T","lbf","Total thrust (from rotors) during hover segment")
		T_A = Variable("T/A","lbf/ft**2","Disk loading during hover segment")
		t = Variable("t",t,"s","Time in hover segment")
		W = mission.W

		self.rotorPerf = aircraft.rotors.performance(state)
		self.batteryPerf = aircraft.battery.performance()
		self.powerSystemPerf = aircraft.powerSystem.performance()

		constraints = []
		constraints += [self.rotorPerf, self.batteryPerf, self.powerSystemPerf]
		
		constraints += [P_rotors==self.rotorPerf.topvar("P"),T==self.rotorPerf.topvar("T"),
			T_A==self.rotorPerf.topvar("T/A")]
		constraints += [P_battery == self.powerSystemPerf.topvar("P_{in}"),
			P_rotors == self.powerSystemPerf.topvar("P_{out}")]
		constraints += [E==self.batteryPerf.topvar("E"), P_battery==self.batteryPerf.topvar("P"), 
			t==self.batteryPerf.topvar("t")]
		constraints += [T==W]
		
		return constraints

class LevelFlight(Model):
	#Substitution required for either segment_range  or t (loiter time).
	def setup(self,mission,aircraft,V=150*ureg.mph):
		E = Variable("E","kWh","Electrical energy used during level-flight segment")
		P_battery = Variable("P_{battery}","kW","Power drawn (from batteries) during segment")
		P_cruise  = Variable("P_{cruise}","kW","Power used (by propulsion system) during cruise segment")
		T = Variable("T","lbf","Thrust during level-flight  segment")
		D = Variable("D","lbf","Drag during level-flight segment")
		t = Variable("t","s","Time in level-flight segment")
		segment_range = Variable("segment_range","nautical_mile",
			"Distance travelled during segment")
		V = Variable("V",V,"mph","Velocity during segment")
		
		W = mission.W
		L_D = aircraft.topvar("L_D")
		eta_cruise = aircraft.topvar("\eta_{cruise}")
		
		self.batteryPerf = aircraft.battery.performance()
		self.powerSystemPerf = aircraft.powerSystem.performance()

		constraints = []
		constraints += [self.batteryPerf, self.powerSystemPerf]

		constraints += [E==self.batteryPerf.topvar("E"), P_battery==self.batteryPerf.topvar("P"),
			t==self.batteryPerf.topvar("t")]
		constraints += [P_battery == self.powerSystemPerf.topvar("P_{in}"),
			P_cruise == self.powerSystemPerf.topvar("P_{out}")]
		constraints += [segment_range==V*t,eta_cruise*P_cruise==T*V,T==D,W==L_D*D]

		return constraints

class TimeOnGround(Model):
	#Mission segment for charging and passenger drop-off/pick-up
	def setup(self,mission,charger_power=200*ureg.kW):

		E_mission = mission.E_mission

		t = Variable("t","s","Time spent on ground")
		t_passenger = Variable("t_{passenger}",5,"minute",
			"Time required to load/unload passengers and conduct safety checks")
		t_charge = Variable("t_{charge}","s","Time required to fully charge the battery")
		charger_power = Variable("charger_power",charger_power,"kW","Charger power")

		constraints = []
		
		constraints += [t >= t_passenger, t >= t_charge]
		constraints += [E_mission == charger_power*t_charge]

		return constraints

class OnDemandSizingMission(Model):
	#Mission the aircraft must be able to fly. No economic analysis.
    def setup(self,aircraft,mission_range=100*ureg.nautical_mile,V_cruise=150*ureg.mph,
    	V_loiter=100*ureg.mph,N_passengers=1,time_in_hover=120*ureg.s,reserve_type="Uber",
    	mission_type="piloted"):

    	if (aircraft.autonomousEnabled == "No") & (mission_type != "piloted"):
    		raise ValueError("Autonomy is not enabled for Aircraft() model.")

    	W = Variable("W_{mission}","lbf","Weight of the aircraft during the mission")
    	mission_range = Variable("mission_range",mission_range,"nautical_mile","Mission range")
    	p_ratio = Variable("p_{ratio}","-","Sound pressure ratio in hover")
        C_eff = aircraft.battery.topvar("C_{eff}") #effective battery capacity

        E_mission = Variable("E_{mission}","kWh","Electrical energy used during mission")

        self.W = W
        self.mission_type = mission_type
        self.crew = Crew(mission_type=mission_type)
        self.passengers = Passengers(N_passengers=N_passengers)
        
        hoverState = FlightState(h=0*ureg.ft)

        self.fs0 = Hover(self,aircraft,hoverState,t=time_in_hover)#takeoff
        self.fs1 = LevelFlight(self,aircraft,V=V_cruise)#fly to destination
        self.fs2 = Hover(self,aircraft,hoverState,t=time_in_hover)#landing
        self.fs3 = Hover(self,aircraft,hoverState,t=time_in_hover)#take off again
        self.fs4 = LevelFlight(self,aircraft,V=V_loiter)#loiter (reserve)
        self.fs5 = Hover(self,aircraft,hoverState,t=time_in_hover)#landing again

        self.flight_segments = [self.fs0, self.fs1, self.fs2, self.fs3, self.fs4, self.fs5]

        constraints = []
        constraints += [self.flight_segments]
        constraints += [self.crew, self.passengers]
       
        constraints += [W >= aircraft.topvar("W_{empty}") + self.passengers.topvar("W") \
        	+ self.crew.topvar("W")]
        constraints += [aircraft.topvar("MTOW") >= W]
        
        if reserve_type == "FAA":#45-minute loiter time, as per night VFR rules
        	t_loiter = Variable("t_{loiter}",45,"minutes","Loiter time")
        	constraints += [t_loiter == self.fs4.topvar("t")]
        if reserve_type == "Uber":#2-nautical-mile diversion distance; used by McDonald & German
        	R_divert = Variable("R_{divert}",2,"nautical_mile","Diversion distance")
        	constraints += [R_divert == self.fs4.topvar("segment_range")]

        constraints += [mission_range == self.fs1.topvar("segment_range")]
        constraints += [p_ratio == self.fs0.rotorPerf.topvar("p_{ratio}")]
        constraints += hoverState

        constraints += [E_mission >= sum(c.topvar("E") for c in self.flight_segments)]
        constraints += [C_eff >= E_mission]
        
        return constraints

class OnDemandTypicalMission(Model):
	#Typical mission. Economic analysis included.
    def setup(self,aircraft,mission_range=100*ureg.nautical_mile,V_cruise=150*ureg.mph,
    	N_passengers=1,time_in_hover=60*ureg.s,charger_power=200*ureg.kW,
    	mission_type="piloted"):

    	if (aircraft.autonomousEnabled == "No") & (mission_type != "piloted"):
    		raise ValueError("Autonomy is not enabled for Aircraft() model.")

    	W = Variable("W_{mission}","lbf","Weight of the aircraft during the mission")
    	mission_range = Variable("mission_range",mission_range,"nautical_mile",
    		"Mission range (not including reserves)")
    	p_ratio = Variable("p_{ratio}","-","Sound pressure ratio in hover")
        C_eff = aircraft.battery.topvar("C_{eff}") #effective battery capacity
        
        t_mission = Variable("t_{mission}","minutes","Time to complete mission (including charging)")
        t_flight = Variable("t_{flight}","minutes","Time in flight")
        E_mission = Variable("E_{mission}","kWh","Electrical energy used during mission")

        self.W = W
        self.E_mission = E_mission
        self.mission_type = mission_type
        self.crew = Crew(mission_type=mission_type)
        self.passengers = Passengers(N_passengers=N_passengers)
        
        hoverState = FlightState(h=0*ureg.ft)

        self.fs0 = Hover(self,aircraft,hoverState,t=time_in_hover)#takeoff
        self.fs1 = LevelFlight(self,aircraft,V=V_cruise)#fly to destination
        self.fs2 = Hover(self,aircraft,hoverState,t=time_in_hover)#landing
        self.time_on_ground = TimeOnGround(self,charger_power=charger_power)

        self.segments = [self.fs0, self.fs1, self.fs2, self.time_on_ground]
        self.flight_segments = [self.fs0, self.fs1, self.fs2]

        constraints = []
        constraints += [self.segments]
        constraints += [self.crew,self.passengers]

        constraints += [W >= aircraft.topvar("W_{empty}") + self.passengers.topvar("W") \
        	+ self.crew.topvar("W")]
        constraints += [aircraft.topvar("MTOW") >= W]
        
        constraints += [mission_range == self.fs1.topvar("segment_range")]
        constraints += [p_ratio == self.fs0.rotorPerf.topvar("p_{ratio}")]
        constraints += hoverState

        constraints += [E_mission >= sum(c.topvar("E") for c in self.flight_segments)]
        constraints += [C_eff >= E_mission]

        constraints += [t_flight >= sum(c.topvar("t") for c in self.flight_segments)]
        constraints += [t_mission >= t_flight + self.time_on_ground.topvar("t")]
        
        return constraints

class OnDemandMissionCost(Model):
	#Includes both revenue and deadhead missions
	def setup(self,aircraft,revenue_mission,deadhead_mission,pilot_wrap_rate=70*ureg.hr**-1,
		mechanic_wrap_rate=60*ureg.hr**-1,MMH_FH=0.6,deadhead_ratio=0.2):

		N_passengers = revenue_mission.passengers.topvar("N_{passengers}")

		cpt = Variable("cost_per_trip","-","Cost (in dollars) for one trip")
		cptpp = Variable("cost_per_trip_per_passenger","-",
			"Cost (in dollars) for one trip, per passenger carried on revenue trip")

		NdNr = Variable("N_{deadhead}/N_{typical}",deadhead_ratio/(1-deadhead_ratio),"-",
			"Number of deadhead missions per typical mission")

		revenue_mission_costs = OneMissionCost(aircraft,revenue_mission,
			pilot_wrap_rate=pilot_wrap_rate,mechanic_wrap_rate=mechanic_wrap_rate,
			MMH_FH=MMH_FH)
		deadhead_mission_costs = OneMissionCost(aircraft,deadhead_mission,
			pilot_wrap_rate=pilot_wrap_rate,mechanic_wrap_rate=mechanic_wrap_rate,
			MMH_FH=MMH_FH)
		
		constraints = []
		constraints += [revenue_mission_costs, deadhead_mission_costs]
		
		constraints += [cpt >= revenue_mission_costs.topvar("cost_per_mission") + NdNr*deadhead_mission_costs.topvar("cost_per_mission")]
		constraints += [cpt == cptpp*N_passengers]
		
		return constraints

class OneMissionCost(Model):
	#Cost for one mission. Can be either deadhead or typical.
	def setup(self,aircraft,mission,pilot_wrap_rate=70*ureg.hr**-1,
		mechanic_wrap_rate=60*ureg.hr**-1,MMH_FH=0.6):

		t_mission = mission.topvar("t_{mission}")

		cost_per_mission = Variable("cost_per_mission","-","Cost per mission")
		cost_per_time = Variable("cost_per_time","hr**-1","Cost per unit mission time")

		capital_expenses = CapitalExpenses(aircraft,mission)
		operating_expenses = OperatingExpenses(aircraft,mission,
			pilot_wrap_rate=pilot_wrap_rate,mechanic_wrap_rate=mechanic_wrap_rate,
			MMH_FH=MMH_FH)
		expenses = [capital_expenses, operating_expenses]

		constraints = []
		
		constraints += [expenses]
		constraints += [cost_per_mission >= sum(c.topvar("cost_per_mission") for c in expenses)]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints

class VehicleAcquisitionCost(Model):
	def setup(self,aircraft,mission):
		
		t_mission = mission.topvar("t_{mission}")
		purchase_price = aircraft.topvar("purchase_price")
		vehicle_life = aircraft.topvar("vehicle_life")
		
		cost_per_time = Variable("cost_per_time","hr**-1",
			"Amortized vehicle purchase price per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-",
			"Amortized vehicle acquisition cost per mission")

		constraints = []

		constraints += [cost_per_time == purchase_price/vehicle_life]
		constraints += [cost_per_mission == t_mission*cost_per_time]
		
		return constraints

class AvionicsAcquisitionCost(Model):
	def setup(self,aircraft,mission):
		
		t_mission = mission.topvar("t_{mission}")
		purchase_price = aircraft.avionics.topvar("purchase_price")
		vehicle_life = aircraft.topvar("vehicle_life")
		
		cost_per_time = Variable("cost_per_time","hr**-1",
			"Amortized avionics purchase price per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-",
			"Amortized avionics acquisition cost per mission")

		constraints = []

		constraints += [cost_per_time == purchase_price/vehicle_life]
		constraints += [cost_per_mission == t_mission*cost_per_time]
		
		return constraints

class BatteryAcquisitionCost(Model):
	def setup(self,battery,mission):
		
		t_mission = mission.topvar("t_{mission}")

		purchase_price = battery.topvar("purchase_price")
		cycle_life = battery.topvar("cycle_life")
		
		cost_per_time = Variable("cost_per_time","hr**-1",
			"Amortized battery purchase price per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-",
			"Amortized battery cost per mission")

		constraints = []

		constraints += [cost_per_mission == purchase_price/cycle_life]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints

class CapitalExpenses(Model):
	def setup(self,aircraft,mission):

		t_mission = mission.topvar("t_{mission}")

		cost_per_time = Variable("cost_per_time","hr**-1","Capital expenses per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Capital expenses per mission")

		vehicle_cost = VehicleAcquisitionCost(aircraft,mission)
		avionics_cost = AvionicsAcquisitionCost(aircraft,mission)
		battery_cost = BatteryAcquisitionCost(aircraft.battery,mission)

		self.costs = [vehicle_cost, avionics_cost, battery_cost]

		constraints = []
		constraints += [self.costs]
		
		constraints += [cost_per_mission >= sum(c.topvar("cost_per_mission") for c in self.costs)]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints


class PilotCost(Model):
	def setup(self,mission,wrap_rate=70*ureg.hr**-1,pilots_per_aircraft=1.5,
		aircraft_per_bunker_pilot=8.):

		t_mission = mission.topvar("t_{mission}")

		wrap_rate = Variable("wrap_rate",wrap_rate,"hr**-1",
			"Cost per pilot, per unit mission time (including benefits and overhead)")
		
		cost_per_time = Variable("cost_per_time","hr**-1","Pilot cost per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Pilot cost per mission")

		constraints = []
		
		if mission.mission_type == "autonomous":
			aircraft_per_bunker_pilot = Variable("aircraft_per_bunker_pilot",
				aircraft_per_bunker_pilot,"-",
				"Number of aircraft controlled by 1 bunker pilot (assuming no crew on board)")
			constraints += [cost_per_time == wrap_rate/aircraft_per_bunker_pilot]

		if mission.mission_type == "piloted":
			pilots_per_aircraft = Variable("pilots_per_aircraft",pilots_per_aircraft,"-",
				"Pilots per aircraft (assuming crew on board)")
			constraints += [cost_per_time == wrap_rate*pilots_per_aircraft]

		constraints += [cost_per_mission == t_mission*cost_per_time]
		
		return constraints

class MaintenanceCost(Model):
	def setup(self,mission,wrap_rate=60*ureg.hr**-1,MMH_FH=0.6):

		t_mission = mission.topvar("t_{mission}")

		MMH_FH = Variable("MMH_FH",MMH_FH,"-","Maintenance man-hours per flight hour")
		wrap_rate = Variable("wrap_rate",wrap_rate,"hr**-1",
			"Cost per mechanic, per unit maintenance time (including benefits and overhead)")

		cost_per_time = Variable("cost_per_time","hr**-1","Maintenance cost per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Maintenance cost per mission")

		constraints = []

		constraints += [cost_per_time == MMH_FH*wrap_rate]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints

class EnergyCost(Model):
	def setup(self,mission,cost_per_energy=0.12*ureg.kWh**-1):

		t_mission = mission.topvar("t_{mission}")
		E_mission = mission.topvar("E_{mission}")

		cost_per_energy = Variable("cost_per_energy",cost_per_energy,"kWh**-1",
			"Price of electricity")
		eta_charger = Variable("\eta_{charger}",0.9,"-","Charging efficiency")

		cost_per_time = Variable("cost_per_time","hr**-1","Energy cost per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Energy cost per mission")

		constraints = []

		constraints += [cost_per_mission == E_mission*cost_per_energy/eta_charger]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints

class IndirectOperatingCost(Model):
	def setup(self,operating_expenses,IOC_fraction=0.12):

		cost_per_time = Variable("cost_per_time","hr**-1","IOC per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","IOC per mission")

		IOC_fraction = Variable("IOC_fraction",IOC_fraction,"-","IOC as a fraction of DOC")

		constraints = []

		constraints += [cost_per_mission == IOC_fraction*operating_expenses.DOC]
		constraints += [cost_per_time == IOC_fraction*operating_expenses.DOC_per_time]

		return constraints

class OperatingExpenses(Model):
	def setup(self,aircraft,mission,pilot_wrap_rate=70*ureg.hr**-1,
		mechanic_wrap_rate=60*ureg.hr**-1,MMH_FH=0.6):

		t_mission = mission.topvar("t_{mission}")

		cost_per_time = Variable("cost_per_time","hr**-1","Operating expenses per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Operating expenses per mission")

		DOC = Variable("DOC","-","Direct operating cost per mission")
		DOC_per_time = Variable("DOC_per_time","hr**-1","Direct operating cost per unit mission time")
		IOC = Variable("IOC","-","Indirect operating cost per mission")
		IOC_per_time = Variable("IOC_per_time","hr**-1","Indirect operating cost per unit mission time")

		self.DOC = DOC
		self.DOC_per_time = DOC_per_time

		pilot_cost = PilotCost(mission,wrap_rate=pilot_wrap_rate)
		maintenance_cost = MaintenanceCost(mission,wrap_rate=mechanic_wrap_rate,
			MMH_FH=MMH_FH)
		energy_cost = EnergyCost(mission)
		indirect_operating_cost = IndirectOperatingCost(self)

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


if __name__=="__main__":
	
	#Joby S2 representative analysis (applies to tilt-rotors in general)
	
	N = 12 #number of propellers
	T_A = 16.3*ureg("lbf")/ureg("ft")**2
	L_D = 14. #estimated L/D in cruise
	eta_cruise = 0.85 #propulsive efficiency in cruise
	eta_electric = 0.9 #electrical system efficiency
	weight_fraction = 0.3188 #structural mass fraction
	C_m = 400*ureg.Wh/ureg.kg #battery energy density
	Cl_mean_max = 1.0
	n=1.0#battery discharge parameter
	reserve_type = "FAA"

	V_cruise = 200*ureg.mph
	V_loiter=100*ureg.mph

	sizing_mission_range = 200*ureg.nautical_mile
	revenue_mission_range = 100*ureg.nautical_mile
	deadhead_mission_range = 100*ureg.nautical_mile

	sizing_time_in_hover=120*ureg.s
	revenue_time_in_hover=30*ureg.s
	deadhead_time_in_hover=30*ureg.s

	autonomousEnabled = "No"
	sizing_mission_type = "piloted"
	revenue_mission_type = "piloted"
	deadhead_mission_type = "piloted"

	sizing_N_passengers = 1
	revenue_N_passengers = 1
	deadhead_N_passengers = 0.00001

	charger_power=200*ureg.kW

	vehicle_cost_per_weight=350*ureg.lbf**-1
	battery_cost_per_C = 400*ureg.kWh**-1
	pilot_wrap_rate = 70*ureg.hr**-1
	mechanic_wrap_rate = 60*ureg.hr**-1
	MMH_FH = 0.6
	deadhead_ratio = 0.2

	testAircraft = OnDemandAircraft(N=N,L_D=L_D,eta_cruise=eta_cruise,C_m=C_m,
		Cl_mean_max=Cl_mean_max,weight_fraction=weight_fraction,n=n,eta_electric=eta_electric,
		cost_per_weight=vehicle_cost_per_weight,cost_per_C=battery_cost_per_C,
		autonomousEnabled=autonomousEnabled)

	testSizingMission = OnDemandSizingMission(testAircraft,mission_range=sizing_mission_range,
		V_cruise=V_cruise,V_loiter=V_loiter,N_passengers=sizing_N_passengers,
		time_in_hover=sizing_time_in_hover,reserve_type=reserve_type,
		mission_type=sizing_mission_type)
	testSizingMission.substitutions.update({testSizingMission.fs0.topvar("T/A"):T_A})

	testRevenueMission = OnDemandTypicalMission(testAircraft,mission_range=revenue_mission_range,
		V_cruise=V_cruise,N_passengers=revenue_N_passengers,time_in_hover=revenue_time_in_hover,
		charger_power=charger_power,mission_type=revenue_mission_type)

	testDeadheadMission = OnDemandTypicalMission(testAircraft,mission_range=deadhead_mission_range,
		V_cruise=V_cruise,N_passengers=deadhead_N_passengers,time_in_hover=deadhead_time_in_hover,
		charger_power=charger_power,mission_type=deadhead_mission_type)

	testMissionCost = OnDemandMissionCost(testAircraft,testRevenueMission,testDeadheadMission,
		pilot_wrap_rate=pilot_wrap_rate,mechanic_wrap_rate=mechanic_wrap_rate,MMH_FH=MMH_FH,
		deadhead_ratio=deadhead_ratio)
	
	problem = Model(testMissionCost["cost_per_trip"],
		[testAircraft, testSizingMission, testRevenueMission, testDeadheadMission, testMissionCost])
	
	solution = problem.solve(verbosity=0)
	
	
	SPL_sizing  = np.array(20*np.log10(solution["variables"]["p_{ratio}_OnDemandSizingMission"]))
	SPL_revenue = np.array(20*np.log10(solution["variables"]["p_{ratio}_OnDemandTypicalMission"]["p_{ratio}_OnDemandTypicalMission.1"]))
	SPL_deadhead = np.array(20*np.log10(solution["variables"]["p_{ratio}_OnDemandTypicalMission"]["p_{ratio}_OnDemandTypicalMission"]))

	'''
	if reserve_type == "FAA":
		num = solution["constants"]["t_{loiter}_OnDemandSizingMission"].to(ureg.minute).magnitude
		reserve_type_string = " (%0.0f-minute loiter time)" % num
	if reserve_type == "Uber":
		num = solution["constants"]["R_{divert}_OnDemandSizingMission"].to(ureg.nautical_mile).magnitude
		reserve_type_string = " (%0.1f-nm diversion distance)" % num


	print
	print "Concept representative analysis"
	print
	print "Battery energy density: %0.0f Wh/kg" % C_m.to(ureg.Wh/ureg.kg).magnitude
	print "Structural mass fraction: %0.4f" % weight_fraction
	print "Cruise lift-to-drag ratio: %0.1f" % L_D
	print "Hover disk loading: %0.1f lbf/ft^2" % T_A.to(ureg("lbf/ft**2")).magnitude
	print "Rotor maximum mean lift coefficient: %0.2f" % Cl_mean_max
	print "Cruise propulsive efficiency: %0.2f" % eta_cruise
	print "Electrical system efficiency: %0.2f" % eta_electric
	print
	print "Sizing Mission (%s)" % sizing_mission_type
	print "Mission range: %0.0f nm" % \
		solution["variables"]["mission_range_OnDemandSizingMission"].to(ureg.nautical_mile).magnitude
	print "Number of passengers: %0.1f" % \
		solution["constants"]["N_{passengers}_OnDemandSizingMission/Passengers"]
	print "Reserve type: " + reserve_type + reserve_type_string
	print "Vehicle weight during mission: %0.0f lbf" % \
		solution["variables"]["W_{mission}_OnDemandSizingMission"].to(ureg.lbf).magnitude
	print "SPL in hover: %0.1f dB" % SPL_sizing
	print
	print "Typical Mission (%s)" % typical_mission_type
	print "Mission range: %0.0f nm" % \
		solution["variables"]["mission_range_OnDemandTypicalMission"].to(ureg.nautical_mile).magnitude
	print "Number of passengers: %0.1f" % \
		solution["constants"]["N_{passengers}_OnDemandTypicalMission/Passengers"]
	print "Vehicle weight during mission: %0.0f lbf" % \
		solution["variables"]["W_{mission}_OnDemandTypicalMission"].to(ureg.lbf).magnitude
	print "SPL in hover: %0.1f dB" % SPL_typical
	print
	print "Maximum takeoff weight: %0.0f lbs" % \
		solution["variables"]["MTOW_OnDemandAircraft"].to(ureg.lbf).magnitude
	print "Structural weight: %0.0f lbs" % \
		solution["variables"]["W_OnDemandAircraft/Structure"].to(ureg.lbf).magnitude
	print "Battery weight: %0.0f lbs" % \
		solution["variables"]["W_OnDemandAircraft/Battery"].to(ureg.lbf).magnitude
	print "Vehicle purchase price: $%0.0f " % \
		solution["variables"]["purchase_price_OnDemandAircraft"]
	print "Avionics purchase price: $%0.0f " % \
		solution["variables"]["purchase_price_OnDemandAircraft/Avionics"]
	print "Battery purchase price:  $%0.0f " % \
		solution["variables"]["purchase_price_OnDemandAircraft/Battery"]
	print
	print "Typical-mission total time: %0.1f minutes" % \
		solution["variables"]["t_{mission}_OnDemandTypicalMission"].to(ureg.minute).magnitude
	print "Typical-mission flight time: %0.1f minutes" % \
		solution["variables"]["t_{flight}_OnDemandTypicalMission"].to(ureg.minute).magnitude
	print "Typical-mission time on ground: %0.1f minutes" % \
		solution["variables"]["t_OnDemandTypicalMission/TimeOnGround"].to(ureg.minute).magnitude
	print "Cost per trip: $%0.2f" % \
		solution["variables"]["cost_per_trip_OnDemandMissionCost"]
	print "Cost per trip, per passenger: $%0.2f" % \
		solution["variables"]["cost_per_trip_per_passenger_OnDemandMissionCost"]
	print
	print "Vehicle capital expenses, per trip: $%0.2f" % \
		solution["variables"]["cost_per_mission_OnDemandMissionCost/CapitalExpenses"]
	print "Amortized vehicle acquisition cost, per trip: $%0.2f" % \
		solution["variables"]["cost_per_mission_OnDemandMissionCost/CapitalExpenses/VehicleAcquisitionCost"]
	print "Amortized avionics acquisition cost, per trip: $%0.2f" % \
		solution["variables"]["cost_per_mission_OnDemandMissionCost/CapitalExpenses/AvionicsAcquisitionCost"]
	print "Amortized battery acquisition cost, per trip: $%0.2f" % \
		solution["variables"]["cost_per_mission_OnDemandMissionCost/CapitalExpenses/BatteryAcquisitionCost"]
	print	
	print "Vehicle operating expenses, per trip: $%0.2f" % \
		solution["variables"]["cost_per_mission_OnDemandMissionCost/OperatingExpenses"]
	print "Direct operating cost, per trip: $%0.2f" % \
		solution["variables"]["DOC_OnDemandMissionCost/OperatingExpenses"]
	print "Indirect operating cost, per trip: $%0.2f" % \
		solution["variables"]["IOC_OnDemandMissionCost/OperatingExpenses"]
	print
	print "Pilot cost, per trip: $%0.2f" % \
		solution["variables"]["cost_per_mission_OnDemandMissionCost/OperatingExpenses/PilotCost"]
	print "Amortized maintenance cost, per trip: $%0.2f" % \
		solution["variables"]["cost_per_mission_OnDemandMissionCost/OperatingExpenses/MaintenanceCost"]
	print "Energy cost, per trip: $%0.2f" % \
		solution["variables"]["cost_per_mission_OnDemandMissionCost/OperatingExpenses/EnergyCost"]
	
	#print solution.summary()
	'''
