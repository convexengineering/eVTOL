#Validation of rotor model against test data

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../..'))

import numpy as np
from gpkit import Variable, Model, Vectorize, ureg
from matplotlib import pyplot as plt
from aircraft_models import Rotors, FlightState, RotorsAero

#Test data, from Leishman textbook

test_data = {}

test_data["N"] = 1
test_data["s"] = 0.098
test_data["R"] = 32.5*ureg.inch

test_data["CT_oldConvention"] = np.array([0.0019, 0.0026, 0.0027, 0.0033, 0.0041, 0.0042,\
	0.005, 0.0051, 0.0061, 0.0062, 0.0065, 0.0067, 0.0072, 0.0073, 0.0078, 0.008, 0.0083,\
	0.0084, 0.0088])
test_data["FOM"] = np.array([0.31, 0.42, 0.45, 0.51, 0.57, 0.59, 0.63, 0.64, 0.66, 0.69,\
	0.67, 0.68, 0.7, 0.7, 0.7, 0.71, 0.7, 0.71, 0.71])
test_data["CP_oldConvention"] = np.array([0.00019, 0.00022, 0.00022, 0.00026, 0.00032,\
	0.00032, 0.00039, 0.0004, 0.0005, 0.00049, 0.00055, 0.00056, 0.00063, 0.00063, 0.0007,\
	0.00071, 0.00077, 0.00076, 0.00082])

test_data["CT"] = 2*test_data["CT_oldConvention"]
test_data["CP"] = 2*test_data["CP_oldConvention"]


#Data from GPKit analysis
gp_model_data = {}

gp_model_data["ki"] = np.array([1.15,1.2])
gp_model_data["Cd0"] = 0.01

gp_model_data["N"] = test_data["N"]
gp_model_data["s"] = test_data["s"]
gp_model_data["R"] = test_data["R"]

gp_model_data["CT"] = np.linspace(0.002,0.02,16)
gp_model_data["FOM"] = np.zeros([np.size(gp_model_data["CT"]),np.size(gp_model_data["ki"])])
gp_model_data["CP"] = np.zeros([np.size(gp_model_data["CT"]),np.size(gp_model_data["ki"])])

gp_model_State = FlightState(h=0*ureg.ft)

for i,CT in enumerate(gp_model_data["CT"]):
	for j,ki in enumerate(gp_model_data["ki"]):

		gp_model_subDict = {}

		gp_model_Rotor = Rotors()
		gp_model_subDict.update({
			gp_model_Rotor.N: gp_model_data["N"],
			gp_model_Rotor.s: gp_model_data["s"],
			gp_model_Rotor.R: gp_model_data["R"],
			gp_model_Rotor.Cl_mean_max: 2.0,#not needed

		})

		gp_model_Rotor_AeroAnalysis = gp_model_Rotor.performance(gp_model_State)
		gp_model_subDict.update({
			gp_model_Rotor_AeroAnalysis.ki: ki,
			gp_model_Rotor_AeroAnalysis.Cd0: gp_model_data["Cd0"],
			gp_model_Rotor_AeroAnalysis.CT: CT,
			gp_model_Rotor_AeroAnalysis.VT: 20*ureg.ft/ureg.s,
		})

		gp_model_Model = Model(gp_model_Rotor_AeroAnalysis["P"],\
			[gp_model_Rotor,gp_model_Rotor_AeroAnalysis,gp_model_State])
		gp_model_Model.substitutions.update(gp_model_subDict)
		gp_model_Solution = gp_model_Model.solve(verbosity=0)

		gp_model_data["FOM"][i,j] = gp_model_Solution("FOM")
		gp_model_data["CP"][i,j] = gp_model_Solution("CP")



#Plotting commands

plt.ion()
fig1 = plt.figure(figsize=(12,6), dpi=80)
plt.show()

style = {}
style["linestyle"] = ["-","--"]
style["markersize"] = 10

#Figure of Merit
plt.subplot(1,2,1)
for i,ki in enumerate(gp_model_data["ki"]):
	data_label = "GP model ($k_i$ = %0.2f)" % ki
	plt.plot(gp_model_data["CT"],gp_model_data["FOM"][:,i],color="black",
		linestyle=style["linestyle"][i],linewidth=2,label=data_label)

plt.plot(test_data["CT"],test_data["FOM"],color="black",marker='o',linestyle="none",
	fillstyle="full",markersize=style["markersize"],label="Test data")

plt.grid()
plt.xlim(xmin=0)
plt.ylim(ymin=0)
[xmin,xmax] = plt.gca().get_xlim()
plt.xticks(np.arange(xmin, xmax, 0.005),fontsize=12)
[ymin,ymax] = plt.gca().get_ylim()
plt.yticks(np.arange(ymin, 1.1*ymax, 0.1),fontsize=12)
plt.xlabel('Thrust coefficient', fontsize = 20)
plt.ylabel('Figure of merit', fontsize = 20)
plt.title("Figure of Merit",fontsize = 24)
plt.legend(numpoints = 1,loc='lower right', fontsize = 18,framealpha=1)


plt.subplot(1,2,2)
for i,ki in enumerate(gp_model_data["ki"]):
	data_label = "GP model ($k_i$ = %0.2f)" % ki
	plt.plot(gp_model_data["CT"],gp_model_data["CP"][:,i],color="black",
		linestyle=style["linestyle"][i],linewidth=2,label=data_label)

plt.plot(test_data["CT"],test_data["CP"],color="black",marker='o',linestyle="none",
	fillstyle="full",markersize=style["markersize"],label="Test data")

plt.grid()
plt.xlim(xmin=0)
plt.ylim(ymin=0)
[xmin,xmax] = plt.gca().get_xlim()
plt.xticks(np.arange(xmin, xmax, 0.005),fontsize=12)
[ymin,ymax] = plt.gca().get_ylim()
plt.yticks(np.arange(ymin, 1.1*ymax, 0.0005),fontsize=12)
plt.xlabel('Thrust coefficient', fontsize = 20)
plt.ylabel('Power coefficient', fontsize = 20)
plt.title("Power Coefficient",fontsize = 24)
plt.legend(numpoints = 1,loc='lower right', fontsize = 18,framealpha=1)


title_str = "Rotor Aerodynamic Model Validation (s = %0.3f; $C_{d_0}$ = %0.2f)"\
	% (gp_model_data["s"], gp_model_data["Cd0"])
plt.suptitle(title_str,fontsize = 26)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.98,bottom=0.1,top=0.85)
plt.savefig('rotor_validation_plot_01.pdf')