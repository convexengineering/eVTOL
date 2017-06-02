#Top-level aircraft model.

import math
import numpy as np
from gpkit import Variable, Model, Vectorize
from standard_atmosphere import stdatmo
import pint 
ureg = pint.UnitRegistry()


class SimpleOnDemandAircraft(Model):
	def setup(self,R,N,L_D,eta,weight_fraction,C_m,N_passengers=1,N_crew=1,n=1.):
		
		W_TO = Variable("W_{TO}","lbf","Takeoff weight")
		C_eff = Variable("C_{eff}","kWh","Effective battery capacity")
		g = Variable("g",9.807,"m/s**2","Gravitational acceleration")

		L_D = Variable("L_D",L_D,"-","Cruise L/D ratio")
		eta = Variable("\eta",eta,"-","Cruise propulsive efficiency")
		
		self.W_TO = W_TO
		self.C_eff = C_eff
		self.g = g
		self.L_D = L_D
		self.eta = eta

		self.rotors = Rotors(N=N)
		self.battery = Battery(C_m=C_m,n=n)
		self.crew = Crew(N_crew=N_crew)
		self.passengers = Passengers(N_passengers=N_passengers)
		self.structure = SimpleOnDemandStructure(self,weight_fraction)

		R_units = self.rotors["R"].units
		self.rotors.substitutions.update({"R":R.to(R_units).magnitude})

		self.components = [self.rotors,self.battery,self.crew,self.passengers,self.structure]
		constraints = []
		constraints += [g == self.battery.topvar("g")]
		constraints += [self.components]#all constraints implemented at component level
		constraints += [C_eff == self.battery.topvar("C_{eff}")]#battery-capacity constraint
		constraints += [W_TO >= sum(c.topvar("W") for c in self.components)]#top-level weight constraint
		return constraints

class SimpleOnDemandStructure(Model):
	def setup(self,aircraft,weight_fraction):
		W = Variable("W","lbf","Structural weight")
		weight_fraction = Variable("weight_fraction",weight_fraction,"-","Structural weight fraction")

		return [W==weight_fraction*aircraft.W_TO]


class Rotors(Model):

	def performance(self,flightState,MT_max=0.9,CL_mean_max=1.0,SPL_req=100):
		return RotorsAero(self,flightState,MT_max,CL_mean_max,SPL_req)

	def setup(self,N=1,s=0.1):
		R = Variable("R","ft","Propeller radius")
		D = Variable("D","ft","Propeller diameter")
		A = Variable("A","ft^2","Rotor disk area")
		N = Variable("N",N,"-","Number of rotors")
		s = Variable("s",s,"-","Propeller solidity")

		W = Variable("W",0,"lbf","Rotor weight") #weight model not implemented yet

		constraints = [A == math.pi*R**2, D==2*R, N==N, s==s]

		return constraints

class RotorsAero(Model):
	def setup(self,rotors,flightState,MT_max=0.9,CL_mean_max=1.0,SPL_req=100):
		T = Variable("T","lbf","Total thrust")
		T_perRotor = Variable("T_perRotor","lbf","Thrust per rotor")
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
		C_m = Variable("C_m",C_m.to(ureg.Wh/ureg.kg).magnitude,
			"Wh/kg","Battery energy density")
		P_m = Variable("P_m",P_m.to(ureg.W/ureg.kg).magnitude,
			"W/kg","Battery power density")
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
		W_oneCrew = Variable("W_{oneCrew}",
			W_oneCrew.to(ureg.lbf).magnitude,"lbf","Weight of 1 crew member")
		N_crew = Variable("N_{crew}",N_crew,"-","Number of crew members")
		W = Variable("W","lbf","Total weight")

		return [W == N_crew*W_oneCrew]

class Passengers(Model):
	def setup(self,W_onePassenger=200*ureg.lbf,N_passengers=1):
		W_onePassenger = Variable("W_{onePassenger}",
			W_onePassenger.to(ureg.lbf).magnitude,"lbf","Weight of 1 passenger")
		N_passengers = Variable("N_{crew}",N_passengers,"-","Number of passengers")
		W = Variable("W","lbf","Total weight")

		return [W == N_passengers*W_onePassenger]

class FlightState(Model):
	def setup(self,h):
		
		atmospheric_data = stdatmo(h)
		rho = atmospheric_data["\rho"]
		a = atmospheric_data["a"]
		rho = Variable("\rho",rho.to(ureg.kg/ureg.m**3).magnitude,"kg/m^3","Air density")
		a = Variable("a",a.to(ureg.ft/ureg.s).magnitude,"ft/s","Speed of sound")

		constraints = []
		constraints += [a == a, rho == rho]
		return constraints

class Hover(Model):
	def setup(self,aircraft,state,t=120*ureg.s):
		E = Variable("E","kWh","Electrical energy used during hover segment")
		P = Variable("P","kW","Power draw (input to rotor) during hover segment")
		T = Variable("T","lbf","Total thrust (from rotors) during hover segment")
		t = Variable("t",t.to(ureg.s).magnitude,"s","Time in hover segment")
		W = aircraft.W_TO
		self.E = E

		rotorPerf = aircraft.rotors.performance(state)
		batteryPerf = aircraft.battery.performance()

		constraints = [rotorPerf, batteryPerf]
		constraints += [P==rotorPerf.topvar("P"),T==rotorPerf.topvar("T")]
		constraints += [E==batteryPerf.topvar("E"), P==batteryPerf.topvar("P"), 
			t==batteryPerf.topvar("t")]
		constraints += [T==W]
		return constraints

class LevelFlight(Model):
	#Substitution required for either R (segment range) or t (loiter time).
	def setup(self,aircraft,V=150*ureg.mph):
		E = Variable("E","kWh","Electrical energy used during level-flight segment")
		P = Variable("P","kW","Power draw during level-flight segment")
		T = Variable("T","lbf","Thrust during level-flight  segment")
		D = Variable("D","lbf","Drag during level-flight segment")
		t = Variable("t","s","Time in level-flight segment")
		R = Variable("R","nautical_mile","Distance travelled during segment")
		V = Variable("V",V.to(ureg.knot).magnitude,"mph","Velocity during segment")
		
		W = aircraft.W_TO
		L_D = aircraft.L_D
		eta = aircraft.eta

		self.E = E
		
		batteryPerf = aircraft.battery.performance()

		constraints = []
		constraints += [R==V*t,eta*P==T*V,T==D,W==L_D*D]
		constraints += [E==batteryPerf.topvar("E"), P==batteryPerf.topvar("P"),
			t==batteryPerf.topvar("t")]
		constraints += [batteryPerf]

		return constraints

class SimpleOnDemandMission(Model):
    def setup(self,aircraft,R=100*ureg.nautical_mile,V_cruise=150*ureg.mph,
    	V_loiter=100*ureg.mph,time_in_hover=120*ureg.s,reserve="Yes"):

    	p_ratio = Variable("p_{ratio}","-","Sound pressure ratio in hover")
        C_eff = aircraft.C_eff

        self.aircraft = aircraft
        
        hoverState = FlightState(h=0*ureg.ft)

        fs0 = Hover(aircraft,hoverState,t=time_in_hover)#takeoff
        fs1 = LevelFlight(aircraft,V=V_cruise)#fly to destination
        fs2 = Hover(aircraft,hoverState,t=time_in_hover)#landing
        fs3 = Hover(aircraft,hoverState,t=time_in_hover)#take off again
        fs4 = LevelFlight(aircraft,V=V_loiter)#loiter (reserve)
        fs5 = Hover(aircraft,hoverState,t=time_in_hover)#landing again

        range_units = fs1.topvar("R").units
        fs1.substitutions.update({"R":R.to(range_units).magnitude})

        loiter_time = 45*ureg("minute") #FAA requirement
        loiter_time_units = fs4.topvar("t").units
        fs4.substitutions.update({"t_SimpleOnDemandMission/LevelFlight":loiter_time.to(loiter_time_units).magnitude})
       		
        constraints = []
        constraints += [C_eff >= fs0.E+fs1.E+fs2.E+fs3.E+fs4.E+fs5.E]
        constraints += [fs0, fs1, fs2, fs3, fs4, fs5]
        constraints += [p_ratio == fs0["p_{ratio}"]]
        constraints += hoverState
        return constraints

if __name__=="__main__":
	
	#Joby S2 representative analysis
	
	N = 12 #number of propellers
	R=1.804*ureg("ft") #propeller radius
	L_D = 14 #estimated L/D in cruise
	eta = 0.8 #estimated propulsive efficiency in cruise
	weight_fraction = 0.358#structural mass fraction
	C_m = 400*ureg.Wh/ureg.kg #battery energy density
	N_passengers = 1
	N_crew = 1
	n=1.#battery discharge parameter

	mission_range = 200*ureg.nautical_mile
	V_cruise = 200*ureg.mph
	V_loiter=100*ureg.mph

	testAircraft = SimpleOnDemandAircraft(R=R,N=N,L_D=L_D,eta=eta,C_m=C_m,
		weight_fraction=weight_fraction,n=n)
	testMission = SimpleOnDemandMission(testAircraft,R=mission_range,V_cruise=V_cruise,
		V_loiter=V_loiter)
	problem = Model(testAircraft["W_{TO}"],[testAircraft,testMission])
	solution = problem.solve(verbosity=0)

	SPL = np.array(20*np.log10(solution["variables"]["p_{ratio}_SimpleOnDemandMission"]))

	print
	print "Joby S2 representative analysis"
	print
	print "Battery energy density: %0.0f Wh/kg" % C_m.to(ureg.Wh/ureg.kg).magnitude
	print "Structural mass fraction: %0.3f" % weight_fraction
	print "Cruise lift-to-drag ratio: %0.1f" % L_D
	print "Cruise propulsive efficiency: %0.2f" % eta
	print
	print "Takeoff weight: %0.0f lbs" % \
		solution["variables"]["W_{TO}_SimpleOnDemandAircraft"].to(ureg.lbf).magnitude
	print "Battery weight: %0.0f lbs" % \
		solution["variables"]["W_SimpleOnDemandAircraft/Battery"].to(ureg.lbf).magnitude
	print "SPL in hover: %0.1f dB" % SPL

	#print solution.summary()
