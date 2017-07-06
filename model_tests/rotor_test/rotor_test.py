#Code for testing the GP-compatible rotor models

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../..'))

import math
import numpy as np
from gpkit import Variable, Model, ureg
from standard_atmosphere import stdatmo
from aircraft_models import Rotors, FlightState, RotorsAero

def rotors_analysis_function(T=2000*ureg("lbf"),VT="unconstrained",h=0*ureg.ft,
	N=12,R=1.804*ureg("ft"),s=0.1,Cl_mean_max=1.4,SPL_requirement=100.,
	print_summary="No"):
	
	#Function uses GPKit models as the backend to analyze a rotor.
	testRotor = Rotors(N=N,s=s,Cl_mean_max=Cl_mean_max)
	testRotor.substitutions.update({"R":R})
	testState = FlightState(h=h)
	testRotor_AeroAnalysis = testRotor.performance(testState,SPL_req=SPL_requirement)
	testRotor_AeroAnalysis.substitutions.update({"T":T.to(ureg.lbf).magnitude})

	if VT != "unconstrained":
		testRotor_AeroAnalysis.substitutions.update({"VT":VT})

	testModel = Model(testRotor_AeroAnalysis["P"],[testRotor,testRotor_AeroAnalysis])
	testSolution = testModel.solve(verbosity=0)

	if print_summary=="Yes":
		print testSolution.summary()

	VT = testSolution["variables"]["VT_RotorsAero"]
	P = testSolution["variables"]["P_RotorsAero"]
	FOM = testSolution["variables"]["FOM_RotorsAero"]
	Cl_mean = testSolution["variables"]["Cl_mean_RotorsAero"]
	SPL = 20*np.log10(testSolution["variables"]["p_{ratio}_RotorsAero"])
	
	return [VT,P,FOM,Cl_mean,SPL]


if __name__ == "__main__":

	#Analysis representative of the Joby S2

	T = 2000*ureg.lbf
	VT = 700*ureg.ft/ureg.s
	N = 12
	R = 1.804*ureg.ft
	s = 0.1
	Cl_mean_max = 1.
	SPL_requirement = 100 #constraint not really enforced

	[VT_computed,P,FOM,CL_mean,SPL] = rotors_analysis_function(T=T,VT=VT,N=N,R=R,s=s,
		Cl_mean_max=Cl_mean_max,SPL_requirement=SPL_requirement)

	print
	print "Analysis representative of the Joby S2"
	print 
	print "T = %0.0f lbf" % T.to(ureg.lbf).magnitude
	print "VT = %0.0f ft/s" % VT.to(ureg.ft/ureg.s).magnitude
	print "N = %0.0f" % N
	print "R = %0.3f ft" % R.to(ureg.ft).magnitude
	print "s = %0.2f" % s
	print
	print "P = %0.0f hp" % P.to(ureg.hp).magnitude
	print "FOM = %0.3f" % FOM
	print "CL_mean = %0.3f" % CL_mean
	print "SPL = %0.1f dB" % SPL



	