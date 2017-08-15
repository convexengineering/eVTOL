#Validation of rotor model against test data

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/..'))

import numpy as np
from gpkit import Variable, Model, Vectorize, ureg
from matplotlib import pyplot as plt
from aircraft_models import Rotors, FlightState, RotorsAero

#Test data, from Bagai & Leishman (1991)

test_data = {}

test_data["N"] = 1
test_data["s"] = 0.098
test_data["R"] = 32.5*ureg.inch

test_data["CT/s_oldConvention"] = np.array([0.035, 0.04, 0.042, 0.05, 0.06, 0.06, 0.067,\
	0.068, 0.075, 0.077, 0.083, 0.088, 0.095, 0.097, 0.102, 0.103])
test_data["FOM"] = np.array([0.51, 0.59, 0.55, 0.63, 0.6, 0.65, 0.65, 0.63, 0.66, 0.65,\
	0.65, 0.65, 0.66, 0.65, 0.64, 0.65])
test_data["CQ/s_oldConvention"] = np.array([0.0026, 0.003, 0.0034, 0.0037, 0.0046, 0.005,\
	0.0056, 0.006, 0.0068, 0.0071, 0.0081, 0.0086, 0.0095, 0.0101, 0.0112, 0.0112])

test_data["CT"] = 2*test_data["s"]*test_data["CT/s_oldConvention"]
test_data["CP"] = 2*test_data["s"]*test_data["CQ/s_oldConvention"]


#Data from GPKit analysis
gp_model_data = {}

gp_model_data["ki"] = 1.4
gp_model_data["Cd0"] = 0.006

gp_model_data["N"] = test_data["N"]
gp_model_data["s"] = test_data["s"]
gp_model_data["R"] = test_data["R"]

gp_model_data["CT"] = np.linspace(0.005,0.02,16)
gp_model_data["FOM"] = np.zeros(np.size(gp_model_data["CT"]))
gp_model_data["CP"] = np.zeros(np.size(gp_model_data["CT"]))

gp_model_Rotor = Rotors(N=gp_model_data["N"],s=gp_model_data["s"])
gp_model_Rotor.substitutions.update({"R":gp_model_data["R"]})

gp_model_State = FlightState(h=0*ureg.ft)

for i,CT in enumerate(gp_model_data["CT"]):
	gp_model_Rotor_AeroAnalysis = gp_model_Rotor.performance(gp_model_State,
		ki=gp_model_data["ki"],Cd0=gp_model_data["Cd0"])
	gp_model_Rotor_AeroAnalysis.substitutions.update({"CT":CT})
	gp_model_Rotor_AeroAnalysis.substitutions.update({"VT":20*ureg.ft/ureg.s})
	gp_model_Model = Model(gp_model_Rotor_AeroAnalysis["P"],\
		[gp_model_Rotor,gp_model_Rotor_AeroAnalysis,gp_model_State])
	gp_model_Solution = gp_model_Model.solve(verbosity=0)

	gp_model_data["FOM"][i] = gp_model_Solution("FOM")
	gp_model_data["CP"][i] = gp_model_Solution("CP")


#Plotting commands

plt.ion()
fig1 = plt.figure(figsize=(12,6), dpi=80)
plt.show()

style = {}
style["linestyle"] = ["-","-","-","-","--","--","--","--"]
style["marker"] = ["s","o","^","v","s","o","^","v"]
style["fillstyle"] = ["full","full","full","full","none","none","none","none"]
style["markersize"] = 10

#Figure of Merit
plt.subplot(1,2,1)
plt.plot(gp_model_data["CT"],gp_model_data["FOM"],color="black",linestyle="-",
	linewidth=2,label="GP model")
plt.plot(test_data["CT"],test_data["FOM"],color="black",marker='o',linestyle="none",
	fillstyle="full",markersize=style["markersize"],label="Test data")
plt.grid()
plt.xlim(xmin=0)
plt.ylim(ymin=0)
plt.xlabel('Thrust coefficient', fontsize = 16)
plt.ylabel('Figure of merit', fontsize = 16)
plt.title("Figure of Merit",fontsize = 20)
plt.legend(numpoints = 1,loc='lower right', fontsize = 15)

plt.subplot(1,2,2)
plt.plot(gp_model_data["CT"],gp_model_data["CP"],color="black",linestyle="-",
	linewidth=2,label="GP model")
plt.plot(test_data["CT"],test_data["CP"],color="black",marker='o',linestyle="none",
	fillstyle="full",markersize=style["markersize"],label="Test data")
plt.grid()
plt.xlim(xmin=0)
plt.ylim(ymin=0)
plt.xlabel('Thrust coefficient', fontsize = 16)
plt.ylabel('Power coefficient', fontsize = 16)
plt.title("Power Coefficient",fontsize = 20)
plt.legend(numpoints = 1,loc='lower right', fontsize = 15)


title_str = "Rotor Aerodynamic Model Validation ($k_i$ = %0.1f; $C_{d_0}$ = %0.3f)"\
	% (gp_model_data["ki"], gp_model_data["Cd0"])
plt.suptitle(title_str,fontsize = 22)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.98,bottom=0.1,top=0.87)
plt.savefig('rotor_validation_plot_01.pdf')