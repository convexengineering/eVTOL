#Top-level aircraft model.

import math
import numpy as np
from gpkit import Variable, Model, Vectorize, ureg
from standard_atmosphere import stdatmo


class SimpleOnDemandAircraft(Model):
	def setup(self,N,L_D,eta,weight_fraction,C_m,N_passengers=1,N_crew=1,n=1.):
		
		MTOW = Variable("MTOW","lbf","Takeoff weight")
		C_eff = Variable("C_{eff}","kWh","Effective battery capacity")
		g = Variable("g",9.807,"m/s**2","Gravitational acceleration")
		L_D = Variable("L_D",L_D,"-","Cruise L/D ratio")
		eta = Variable("\eta",eta,"-","Cruise propulsive efficiency")

		self.MTOW = MTOW
		self.C_eff = C_eff
		self.g = g
		self.L_D = L_D
		self.eta = eta

		self.rotors = Rotors(N=N)
		self.battery = Battery(C_m=C_m,n=n)
		self.crew = Crew(N_crew=N_crew)
		self.passengers = Passengers(N_passengers=N_passengers)
		self.structure = SimpleOnDemandStructure(self,weight_fraction)
		self.components = [self.rotors,self.battery,self.crew,self.passengers,self.structure]
		
		constraints = []
		constraints += [g == self.battery.topvar("g")]
		constraints += [self.components]#all constraints implemented at component level
		constraints += [C_eff == self.battery.topvar("C_{eff}")]#battery-capacity constraint
		constraints += [MTOW >= sum(c.topvar("W") for c in self.components)]#top-level weight constraint
		return constraints

class SimpleOnDemandStructure(Model):
	def setup(self,aircraft,weight_fraction):
		W = Variable("W","lbf","Structural weight")
		weight_fraction = Variable("weight_fraction",weight_fraction,"-","Structural weight fraction")

		return [W==weight_fraction*aircraft.MTOW]


class Rotors(Model):

	def performance(self,flightState,MT_max=0.9,CL_mean_max=1.0,SPL_req=100):
		return RotorsAero(self,flightState,MT_max,CL_mean_max,SPL_req)

	def setup(self,N=1,s=0.1):
		R = Variable("R","ft","Propeller radius")
		D = Variable("D","ft","Propeller diameter")
		A = Variable("A","ft^2","Area of 1 rotor disk")
		A_total = Variable("A_{total}","ft^2","Combined area of all rotor disks")
		N = Variable("N",N,"-","Number of rotors")
		s = Variable("s",s,"-","Propeller solidity")

		W = Variable("W",0,"lbf","Rotor weight") #weight model not implemented yet

		constraints = [A == math.pi*R**2, D==2*R, N==N, s==s, A_total==N*A]

		return constraints

class RotorsAero(Model):
	def setup(self,rotors,flightState,MT_max=0.9,CL_mean_max=1.0,SPL_req=100):
		T = Variable("T","lbf","Total thrust")
		T_perRotor = Variable("T_perRotor","lbf","Thrust per rotor")
		T_A = Variable("T/A","lbf/ft**2","Disk loading")
		P = Variable("P","hp","Total power")
		P_perRotor = Variable("P_perRotor","hp","Power per rotor")
		VT = Variable("VT","ft/s","Propeller tip speed")
		omega = Variable("\omega","rpm","Propeller angular velocity")
		MT = Variable("MT","-","Propeller tip Mach number")
		MT_max = Variable("MT_max",MT_max,"-","Maximum allowed tip Mach number")

		CT = Variable("CT","-","Thrust coefficient")
		CP = Variable("CP","-","Power coefficient")
		CPi = Variable("CPi","-","Induced (ideal) power coefficient")
		CPp = Variable("CP","-","Profile power coefficient")
		CL_mean = Variable("CL_mean","-","Mean lift coefficient")
		CL_mean_max = Variable("CL_mean_max",CL_mean_max,"-","Maximum allowed mean lift coefficient")
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
		constraints += [CL_mean == 3*CT/s,
			CL_mean <= CL_mean_max]

		#Noise model
		constraints += [p_ratio == k3*((T*omega)/(rho*x))*(N*s)**-0.5,
			p_ratio <= p_ratio_max]

		return constraints

class Battery(Model):

	def performance(self):
		return BatteryPerformance(self)

	#Requires a substitution or constraint for g (gravitational acceleration)
	def setup(self,C_m=350*ureg.Wh/ureg.kg,usable_energy_fraction=0.8,P_m=3000*ureg.W/ureg.kg,n=1.):
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

		self.P_max = P_max
		self.n = n #battery discharge parameter (needed for Peukert effect)

		return [C==m*C_m, W==m*g, C_eff == usable_energy_fraction*C, P_max==P_m*m]

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
	def setup(self,W_oneCrew=190*ureg.lbf,N_crew=1):
		W_oneCrew = Variable("W_{oneCrew}",W_oneCrew,"lbf","Weight of 1 crew member")
		N_crew = Variable("N_{crew}",N_crew,"-","Number of crew members")
		W = Variable("W","lbf","Total weight")

		return [W == N_crew*W_oneCrew]

class Passengers(Model):
	def setup(self,W_onePassenger=200*ureg.lbf,N_passengers=1):
		W_onePassenger = Variable("W_{onePassenger}",W_onePassenger,
			"lbf","Weight of 1 passenger")
		N_passengers = Variable("N_{passengers}",N_passengers,"-","Number of passengers")
		W = Variable("W","lbf","Total weight")

		return [W == N_passengers*W_onePassenger]

class FlightState(Model):
	def setup(self,h):
		
		atmospheric_data = stdatmo(h)
		rho = atmospheric_data["\rho"].to(ureg.kg/ureg.m**3)
		a = atmospheric_data["a"].to(ureg.ft/ureg.s)
		rho = Variable("\rho",rho,"kg/m^3","Air density")
		a = Variable("a",a,"ft/s","Speed of sound")

		constraints = []
		constraints += [a == a, rho == rho]
		return constraints

class Hover(Model):
	def setup(self,aircraft,state,t=120*ureg.s):
		E = Variable("E","kWh","Electrical energy used during hover segment")
		P = Variable("P","kW","Power draw (input to rotor) during hover segment")
		T = Variable("T","lbf","Total thrust (from rotors) during hover segment")
		T_A = Variable("T/A","lbf/ft**2","Disk loading during hover segment")
		t = Variable("t",t,"s","Time in hover segment")
		W = aircraft.MTOW
		self.E = E

		rotorPerf = aircraft.rotors.performance(state)
		batteryPerf = aircraft.battery.performance()

		constraints = [rotorPerf, batteryPerf]
		constraints += [P==rotorPerf.topvar("P"),T==rotorPerf.topvar("T"),
			T_A==rotorPerf.topvar("T/A")]
		constraints += [E==batteryPerf.topvar("E"), P==batteryPerf.topvar("P"), 
			t==batteryPerf.topvar("t")]
		constraints += [T==W]
		return constraints

class LevelFlight(Model):
	#Substitution required for either segment_range  or t (loiter time).
	def setup(self,aircraft,V=150*ureg.mph):
		E = Variable("E","kWh","Electrical energy used during level-flight segment")
		P = Variable("P","kW","Power draw during level-flight segment")
		T = Variable("T","lbf","Thrust during level-flight  segment")
		D = Variable("D","lbf","Drag during level-flight segment")
		t = Variable("t","s","Time in level-flight segment")
		segment_range = Variable("segment_range","nautical_mile",
			"Distance travelled during segment")
		V = Variable("V",V,"mph","Velocity during segment")
		
		W = aircraft.MTOW
		L_D = aircraft.L_D
		eta = aircraft.eta

		self.E = E
		
		batteryPerf = aircraft.battery.performance()

		constraints = []
		constraints += [segment_range==V*t,eta*P==T*V,T==D,W==L_D*D]
		constraints += [E==batteryPerf.topvar("E"), P==batteryPerf.topvar("P"),
			t==batteryPerf.topvar("t")]
		constraints += [batteryPerf]

		return constraints

class OnDemandSizingMission(Model):
	#Mission the aircraft must be able to fly. No economic analysis.
    def setup(self,aircraft,mission_range=100*ureg.nautical_mile,V_cruise=150*ureg.mph,
    	V_loiter=100*ureg.mph,time_in_hover=120*ureg.s):

    	mission_range = Variable("mission_range",mission_range,"nautical_mile","Mission range")
    	p_ratio = Variable("p_{ratio}","-","Sound pressure ratio in hover")
        C_eff = aircraft.C_eff

        self.aircraft = aircraft
        
        hoverState = FlightState(h=0*ureg.ft)

        self.fs0 = Hover(aircraft,hoverState,t=time_in_hover)#takeoff
        self.fs1 = LevelFlight(aircraft,V=V_cruise)#fly to destination
        self.fs2 = Hover(aircraft,hoverState,t=time_in_hover)#landing
        self.fs3 = Hover(aircraft,hoverState,t=time_in_hover)#take off again
        self.fs4 = LevelFlight(aircraft,V=V_loiter)#loiter (reserve)
        self.fs5 = Hover(aircraft,hoverState,t=time_in_hover)#landing again

        loiter_time = 45*ureg("minute") #FAA requirement
        #self.fs4.substitutions.update({"t_SimpleOnDemandMission/LevelFlight":loiter_time})
        self.fs4.substitutions.update({self.fs4.topvar("t"):loiter_time})
       		
        constraints = []
        constraints += [mission_range == self.fs1.topvar("segment_range")]
        constraints += [C_eff >= self.fs0.E + self.fs1.E + self.fs2.E + self.fs3.E
        	+ self.fs4.E + self.fs5.E]
        constraints += [self.fs0, self.fs1, self.fs2, self.fs3, self.fs4, self.fs5]
        constraints += [p_ratio == self.fs0["p_{ratio}"]]
        constraints += hoverState
        return constraints

class OnDemandTypicalMission(Model):
	#Typical mission. Economic analysis included.
    def setup(self,aircraft,mission_range=100*ureg.nautical_mile,V_cruise=150*ureg.mph,
    	time_in_hover=60*ureg.s,cost_per_weight=112*ureg.lbf**-1,pilot_salary=40*ureg.hr**-1,
    	mechanic_salary=30*ureg.hr**-1):

    	mission_range = Variable("mission_range",mission_range,"nautical_mile",
    		"Mission range (not including reserves)")
    	p_ratio = Variable("p_{ratio}","-","Sound pressure ratio in hover")
        C_eff = aircraft.C_eff #effective battery capacity

        cpt = Variable("cost_per_trip","-","Cost (in dollars) for one trip")
        cptpp = Variable("cost_per_trip_per_passenger","-",
        	"Cost (in dollars) for one trip, per passenger")
        
        c_vehicle = Variable("c_{vehicle}","-","Vehicle amortized cost (per mission)")
        t_mission = Variable("t_{mission}","minutes","Time to complete mission")
        vehicle_life = Variable("vehicle_life",10,"years","Vehicle lifetime")
        cost_per_weight = Variable("cost_per_weight",cost_per_weight,"lbf**-1",
        	"Cost per unit weight of the aircraft")
        purchase_price = Variable("purchase_price","-","Purchase price of the aircraft")

        c_energy = Variable("c_{energy}","-","Energy cost (per mission)")
        cost_per_energy = Variable("cost_per_energy",0.12,"kWh**-1",
        	"Price of electricity (dollars per kWh)")
        E_mission = Variable("E_{mission}","kWh","Electrical energy used during mission")

        c_pilot = Variable("c_{pilot}","-","Pilot cost (per mission)")
        pilot_salary = Variable("pilot_salary",pilot_salary,"hr**-1","Pilot salary")

        c_maintenance = Variable("c_{maintenance}","-","Maintenance cost per mission")
        overhaul_cost = Variable("overhaul_cost","-","Cost of 1 overhaul")
        overhaul_time = Variable("overhaul_time",2,"hours","Time to complete 1 overhaul")
        N_mechanics = Variable("N_{mechanics}",2,"-",
        	"Number of mechanics required for an overhaul")
        time_between_overhauls = Variable("time_between_overhauls",50,"hours",
        	"Time between overhauls")
        mechanic_salary = Variable("mechanic_salary",mechanic_salary,"hr**-1",
        	"Mechanic salary")

        self.aircraft = aircraft
        
        hoverState = FlightState(h=0*ureg.ft)

        self.fs0 = Hover(aircraft,hoverState,t=time_in_hover)#takeoff
        self.fs1 = LevelFlight(aircraft,V=V_cruise)#fly to destination
        self.fs2 = Hover(aircraft,hoverState,t=time_in_hover)#landing
       		
        constraints = []
        constraints += [mission_range == self.fs1.topvar("segment_range")]
        constraints += [C_eff >= self.fs0.E + self.fs1.E + self.fs2.E]
        constraints += [self.fs0, self.fs1, self.fs2]
        constraints += [p_ratio == self.fs0["p_{ratio}"]]
        constraints += hoverState

        constraints += [cpt == cptpp*aircraft.passengers.topvar("N_{passengers}")]
        constraints += [cpt >= c_vehicle + c_energy + c_pilot + c_maintenance]
        
        constraints += [c_vehicle == purchase_price*t_mission/vehicle_life]
        constraints += [t_mission >= self.fs0.topvar("t") + self.fs1.topvar("t")+ self.fs2.topvar("t")]
        constraints += [purchase_price == cost_per_weight*aircraft.MTOW]

        constraints += [c_energy == E_mission*cost_per_energy]
        constraints += [E_mission >= self.fs0.E + self.fs1.E + self.fs2.E]

        constraints += [c_pilot == pilot_salary*t_mission]

        constraints += [c_maintenance == overhaul_cost*t_mission/time_between_overhauls]
        constraints += [overhaul_cost == N_mechanics*overhaul_time*mechanic_salary]

        return constraints

if __name__=="__main__":
	
	#Joby S2 representative analysis (applies to tilt-rotors in general)
	
	N = 12 #number of propellers
	T_A = 16.3*ureg("lbf")/ureg("ft")**2
	L_D = 14. #estimated L/D in cruise
	eta = 0.8 #estimated propulsive efficiency in cruise
	weight_fraction = 0.3455#structural mass fraction
	#C_m = 400*ureg.Wh/ureg.kg #battery energy density
	C_m = 400*ureg.Wh/ureg.kg #battery energy density
	N_passengers = 1
	N_crew = 1
	n=1.0#battery discharge parameter

	V_cruise = 200*ureg.mph
	V_loiter=100*ureg.mph

	sizing_mission_range = 200*ureg.nautical_mile
	typical_mission_range = 100*ureg.nautical_mile

	sizing_time_in_hover=120*ureg.s
	typical_time_in_hover=60*ureg.s

	cost_per_weight=112*ureg.lbf**-1
	pilot_salary = 40*ureg.hr**-1
	mechanic_salary=30*ureg.hr**-1

	testAircraft = SimpleOnDemandAircraft(N=N,L_D=L_D,eta=eta,C_m=C_m,
		weight_fraction=weight_fraction,N_passengers=N_passengers,
		N_crew=N_crew,n=n)

	testSizingMission = OnDemandSizingMission(testAircraft,mission_range=sizing_mission_range,
		V_cruise=V_cruise,V_loiter=V_loiter,time_in_hover=sizing_time_in_hover)
	testSizingMission.substitutions.update({testSizingMission.fs0.topvar("T/A"):T_A,
		testSizingMission.fs2.topvar("T/A"):T_A,testSizingMission.fs3.topvar("T/A"):T_A,
		testSizingMission.fs5.topvar("T/A"):T_A})

	testTypicalMission = OnDemandTypicalMission(testAircraft,mission_range=typical_mission_range,
		V_cruise=V_cruise,time_in_hover=typical_time_in_hover,cost_per_weight=cost_per_weight,
		pilot_salary=pilot_salary,mechanic_salary=mechanic_salary)
	
	problem = Model(testTypicalMission["cost_per_trip"],
		[testAircraft, testSizingMission, testTypicalMission])
	solution = problem.solve(verbosity=0)

	SPL = np.array(20*np.log10(solution["variables"]["p_{ratio}_OnDemandSizingMission"]))

	print
	print "Concept representative analysis"
	print
	print "Battery energy density: %0.0f Wh/kg" % C_m.to(ureg.Wh/ureg.kg).magnitude
	print "Structural mass fraction: %0.4f" % weight_fraction
	print "Cruise lift-to-drag ratio: %0.1f" % L_D
	print "Hover disk loading: %0.1f lbf/ft^2" % T_A.to(ureg("lbf/ft**2")).magnitude
	print "Cruise propulsive efficiency: %0.2f" % eta
	print
	print "Sizing mission range: %0.0f nm" % \
		solution["variables"]["mission_range_OnDemandSizingMission"].to(ureg.nautical_mile).magnitude
	print "Typical mission range: %0.0f nm" % \
		solution["variables"]["mission_range_OnDemandTypicalMission"].to(ureg.nautical_mile).magnitude
	print "Number of passengers: %0.0f" % \
		solution["constants"]["N_{passengers}_SimpleOnDemandAircraft/Passengers"]
	print
	print "Takeoff weight: %0.0f lbs" % \
		solution["variables"]["MTOW_SimpleOnDemandAircraft"].to(ureg.lbf).magnitude
	print "Battery weight: %0.0f lbs" % \
		solution["variables"]["W_SimpleOnDemandAircraft/Battery"].to(ureg.lbf).magnitude
	print "SPL in hover: %0.1f dB" % SPL
	print
	print "Typical mission time: %0.1f minutes" % \
		solution["variables"]["t_{mission}_OnDemandTypicalMission"].to(ureg.minute).magnitude
	print "Cost per trip: $%0.2f" % \
		solution["variables"]["cost_per_trip_OnDemandTypicalMission"]
	print "Cost per trip, per passenger: $%0.2f" % \
		solution["variables"]["cost_per_trip_per_passenger_OnDemandTypicalMission"]
	print "Vehicle purchase price: $%0.0f " % \
		solution["variables"]["purchase_price_OnDemandTypicalMission"]
	print "Overhaul cost: $%0.2f " % \
		solution["variables"]["overhaul_cost_OnDemandTypicalMission"]
	print
	print "Vehicle amortized cost, per trip: $%0.2f " % \
		solution["variables"]["c_{vehicle}_OnDemandTypicalMission"]
	print "Energy cost, per trip: $%0.2f " % \
		solution["variables"]["c_{energy}_OnDemandTypicalMission"]
	print "Pilot cost, per trip: $%0.2f " % \
		solution["variables"]["c_{pilot}_OnDemandTypicalMission"]
	print "Maintenance cost, per trip: $%0.2f " % \
		solution["variables"]["c_{maintenance}_OnDemandTypicalMission"]

	#print solution.summary()
