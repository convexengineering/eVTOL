#Design-space sweep for the Joby S2

import math
import numpy as np
from gpkit import Variable, VectorVariable, Model, Vectorize
from rotor_models import Rotors, RotorsFlightState, RotorsAero
from matplotlib import pyplot as plt


import pint
ureg = pint.UnitRegistry()

#Analysis representative of the Joby S2

N = 12
s = 0.1
SPL_requirement = 100 #constraint not really enforced
W_hover = 2000 #lbf; Joby S2
R = 1.804 #ft; Joby S2
CL_mean_max = 1.4 #reasonable guess
Mtip_max = 0.9 #maximum tip Mach number

#generic models
testRotor = Rotors(N=N,s=s)
testRotor.substitutions.update({"R":R})
testState = RotorsFlightState(CL_mean_max=CL_mean_max,SPL_req=SPL_requirement)

#find the lowest tip speed that works (limited by CLmean constraint)
lowRotor_AeroAnalysis = testRotor.dynamic(testState)
lowRotor_AeroAnalysis.substitutions.update({"T":W_hover})

lowModel = Model(lowRotor_AeroAnalysis["P"],[testRotor,lowRotor_AeroAnalysis])
lowSolution = lowModel.solve(verbosity=0)
low_VT = lowSolution["variables"]["VT_RotorsAero"] #lowest tip speed for the sweep

#find the highest tip speed that works (limited by maximum tip Mach number)
highRotor_AeroAnalysis = testRotor.dynamic(testState)
highRotor_AeroAnalysis.substitutions.update({"T":W_hover})
highRotor_AeroAnalysis.substitutions.update({"MT":Mtip_max})

highModel = Model(highRotor_AeroAnalysis["P"],[testRotor,highRotor_AeroAnalysis])
highSolution = highModel.solve(verbosity=0)
high_VT = highSolution["variables"]["VT_RotorsAero"] #highest tip speed for the sweep

#sweep from highest to lowest
VT_array = np.linspace(low_VT.magnitude,high_VT.magnitude,10)
FOM_array = np.zeros(np.size(VT_array))
P_array = np.zeros(np.size(VT_array))
CLmean_array = np.zeros(np.size(VT_array))
SPL_array = np.zeros(np.size(VT_array))


for i, VT in enumerate(VT_array):
	sweepRotor_AeroAnalysis = testRotor.dynamic(testState)
	sweepRotor_AeroAnalysis.substitutions.update({"T":W_hover})
	sweepRotor_AeroAnalysis.substitutions.update({"VT":VT})
	sweepModel = Model(sweepRotor_AeroAnalysis["P"],[testRotor,sweepRotor_AeroAnalysis])
	sweepSolution = sweepModel.solve(verbosity=0)

	FOM_array[i] = sweepSolution["variables"]["FOM_RotorsAero"]
	P_array[i] = sweepSolution["variables"]["P_RotorsAero"].magnitude
	CLmean_array[i] = sweepSolution["variables"]["CL_mean_RotorsAero"]
	SPL_array[i] = 20*np.log10(sweepSolution["variables"]["p_{ratio}_RotorsAero"])

# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(11,8), dpi=80)
plt.show()

#FOM vs. VT
plt.subplot(2,2,1)
plt.plot(VT_array,FOM_array,color="black", linewidth=1.5, linestyle="-", marker='s', markersize=8)	 
plt.grid()
plt.xlabel('Propeller tip speed (ft/s)', fontsize = 16)
plt.ylabel('FOM (dimensionless)', fontsize = 16)
plt.title("Figure of Merit",fontsize = 20)
plt.ylim(ymin=0,ymax=0.9)

#Required Power vs. VT
plt.subplot(2,2,2)
plt.plot(VT_array,P_array,color="black", linewidth=1.5, linestyle="-", marker='s', markersize=8)	 
plt.grid()
plt.xlabel('Propeller tip speed (ft/s)', fontsize = 16)
plt.ylabel('Power required (HP)', fontsize = 16)
plt.title("Power Required",fontsize = 20)
plt.ylim(ymin=0)

#Mean Lift Coefficient vs. VT
plt.subplot(2,2,3)
plt.plot(VT_array,CLmean_array,color="black", linewidth=1.5, linestyle="-", marker='s', markersize=8)	 
plt.grid()
plt.xlabel('Propeller tip speed (ft/s)', fontsize = 16)
plt.ylabel('Mean $C_L$ (dimensionless)', fontsize = 16)
plt.title("Mean Lift Coefficient",fontsize = 20)
plt.ylim(ymin=0)

#SPL vs. VT
plt.subplot(2,2,4)
plt.plot(VT_array,SPL_array,color="black", linewidth=1.5, linestyle="-", marker='s', markersize=8)	 
plt.grid()
plt.xlabel('Propeller tip speed (ft/s)', fontsize = 16)
plt.ylabel('Vortex noise (dB)', fontsize = 16)
plt.title("Vortex Noise",fontsize = 20)

title_str = "Propeller Tip-Speed Sweep for the Joby S2\n" \
	+ "$W_{hover} = "+str(W_hover)+"\; lbf$; " \
	+ "$R = "+str(R)+"\; ft$; " \
	+ "$s = "+str(s)+"$"

plt.suptitle(title_str,fontsize = 20)

plt.tight_layout()#makes sure subplots are spaced neatly
plt.subplots_adjust(left=0.125,right=0.9,bottom=0.1,top = 0.85)#adds space at the top for the title