#GP-compatible models for the lifting rotor(s) of on-demand aviation aircraft

import math
import numpy as np
from gpkit import Variable, Model

class Rotors(Model):

	def dynamic(self,rotorsState):
		return RotorsAero(self,rotorsState)

	def setup(self,N=1,s=0.01):
		R = Variable("R","ft","Propeller radius")
		A = Variable("A","ft^2","Rotor disk area")
		N = Variable("N",N,"-","Number of rotors")
		s = Variable("s",s,"-","Propeller solidity")

		constraints = [A == math.pi*R**2, N==N, s==s]

		return constraints

class RotorsFlightState(Model):
	def setup(self,rho_metric=1.225,a_metric=340.3,MT_max=0.9,CL_mean_max=1.0,SPL_req=100):
		ki = Variable("ki",1.1,"-","Induced power factor")
		Cd0 = Variable("Cd0",0.01,"-","Blade two-dimensional zero-lift drag coefficient")

		rho = Variable("\rho",rho_metric,"kg/m^3","Air density")
		a = Variable("a",a_metric,"m/s","Speed of sound")

		MT_max = Variable("MT_max",MT_max,"-","Maximum allowed tip Mach number")
		CL_mean_max = Variable("CL_mean_max",CL_mean_max,"-","Maximum allowed mean lift coefficient")

		p_ratio_max = Variable("p_{ratio_max}",10**(SPL_req/20.),"-","Max allowed sound pressure ratio")

		constraints = []
		constraints += [ki==ki,Cd0==Cd0,rho==rho,a==a,MT_max==MT_max,
			CL_mean_max==CL_mean_max,p_ratio_max==p_ratio_max]

		return constraints


class RotorsAero(Model):
	def setup(self,rotors,rotorsState):
		T = Variable("T","lbf","Total thrust")
		T_perRotor = Variable("T_perRotor","lbf","Thrust per rotor")
		P = Variable("P","hp","Total power")
		P_perRotor = Variable("P_perRotor","hp","Power per rotor")
		VT = Variable("VT","ft/s","Propeller tip speed")
		omega = Variable("\omega","rpm","Propeller angular velocity")
		MT = Variable("MT","-","Propeller tip Mach number")

		CT = Variable("CT","-","Thrust coefficient")
		CP = Variable("CP","-","Power coefficient")
		CPi = Variable("CPi","-","Induced (ideal) power coefficient")
		CPp = Variable("CP","-","Profile power coefficient")
		CL_mean = Variable("CL_mean","-","Mean lift coefficient")
		FOM = Variable("FOM","-","Figure of merit")

		p_ratio = Variable("p_{ratio}","-","Sound pressure ratio (p/p_{ref})")
		x = Variable("x",500,"ft","Distance from source at which to calculate sound")
		k3 = Variable("k3",6.804e-3,"s**3/ft**3","Sound-pressure constant")

		R = rotors["R"]
		A = rotors["A"]
		N = rotors["N"]
		s = rotors["s"]

		ki = rotorsState["ki"]
		Cd0 = rotorsState["Cd0"]
		rho = rotorsState["\rho"]
		a = rotorsState["a"]
		MT_max = rotorsState["MT_max"]
		CL_mean_max = rotorsState["CL_mean_max"]
		p_ratio_max = rotorsState["p_{ratio_max}"]

		constraints = [rotorsState]

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

if __name__ == "__main__":
	import pint
	ureg = pint.UnitRegistry()

	#Analysis representative of the Joby S2

	N = 12
	s = 0.1
	SPL_requirement = 100 #constraint not really enforced
	T = 2000 #lbf; Joby S2
	R = 1.804 #ft; Joby S2
	CL_mean_max = 1.

	#fint the lowest tip speed that works
	testRotor = Rotors(N=N,s=s)
	testRotor.substitutions.update({"R":R})

	testState = RotorsFlightState(CL_mean_max=CL_mean_max,SPL_req=SPL_requirement)
	testRotor_AeroAnalysis = testRotor.dynamic(testState)
	testRotor_AeroAnalysis.substitutions.update({"T":T})

	testModel = Model(testRotor_AeroAnalysis["P"],[testRotor,testRotor_AeroAnalysis])
	testSolution = testModel.solve(verbosity=0)
	print testSolution.summary()