#Validation of rotor model against test data

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../../models'))

import numpy as np

from gpkit           import Variable, Model, Vectorize, ureg
from matplotlib      import pyplot as plt
from aircraft_models import Rotors
from mission_models  import HoverFlightState

#Test data, from Leishman textbook

test_data = {}

test_data["N"]           = 1.
test_data["s"]           = 0.098
test_data["R"]           = 32.5 * ureg.inch
test_data["B"]           = 4.

# self.T_A_max     = T_A_max     = Variable("",   "N/m^2", "Rotor maximum allowed disk loading")
		
#self.Cl_mean_max = Cl_mean_max = Variable("Cl_{mean,max}", "-",     "Rotor maximum allowed mean lift coefficient")



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

gp_model_data["ki"] = np.array([1.15, 1.2])
gp_model_data["Cd0"] = 0.01

gp_model_data["N"] = test_data["N"]
gp_model_data["s"] = test_data["s"]
gp_model_data["R"] = test_data["R"]
gp_model_data["B"] = test_data["B"]

gp_model_data["t/c"]           = 0.12                         # Irrelevant
gp_model_data["M_{tip,max}"]   = 0.9                          # Irrelevant
gp_model_data["(T/A)_{max}"]   = 7.  * ureg.lbf / ureg.ft**2  # Irrelevant
gp_model_data["Cl_{mean,max}"] = 2.0                          # Irrelevant
gp_model_data["v_{tip}"]       = 20. * ureg.ft/ureg.s         # Irrelevant

gp_model_data["CT"]  = np.linspace(0.002,0.02,16)
gp_model_data["FOM"] = np.zeros([np.size(gp_model_data["CT"]), np.size(gp_model_data["ki"])])
gp_model_data["CP"]  = np.zeros([np.size(gp_model_data["CT"]), np.size(gp_model_data["ki"])])

gp_model_State = HoverFlightState(h=0*ureg.ft)

for i, CT in enumerate(gp_model_data["CT"]):
	
	for j, ki in enumerate(gp_model_data["ki"]):

		gp_model_Rotor = Rotors()
		gp_model_Rotor.substitutions.update({
			gp_model_Rotor.N:   gp_model_data["N"],
			gp_model_Rotor.s:   gp_model_data["s"],
			gp_model_Rotor.R:   gp_model_data["R"],
			gp_model_Rotor.B:   gp_model_data["B"],
			gp_model_Rotor.t_c: gp_model_data["t/c"],

			gp_model_Rotor.ki:          ki,
			gp_model_Rotor.Cd0:         gp_model_data["Cd0"],
			gp_model_Rotor.T_A_max:     gp_model_data["(T/A)_{max}"],
			gp_model_Rotor.M_tip_max:   gp_model_data["M_{tip,max}"],
			gp_model_Rotor.Cl_mean_max: gp_model_data["Cl_{mean,max}"],
		})

		gp_model_Rotor_AeroAnalysis = gp_model_Rotor.performance(gp_model_State)
		
		gp_model_Rotor_AeroAnalysis.substitutions.update({
			gp_model_Rotor_AeroAnalysis.CT:    CT,
			gp_model_Rotor_AeroAnalysis.v_tip: gp_model_data["v_{tip}"]
		})

		objective_function = gp_model_Rotor_AeroAnalysis.P

		problem  = Model(objective_function, [gp_model_Rotor, gp_model_Rotor_AeroAnalysis, gp_model_State])
		solution = problem.solve(verbosity=0)

		gp_model_data["FOM"][i,j] = solution("FOM")
		gp_model_data["CP"][i,j]  = solution("CP")



#Plotting commands

plt.ion()
fig1 = plt.figure(figsize=(5,8), dpi=80)
plt.show()

style = {}
style["linestyle"] = ["-","--"]
style["markersize"] = 10

style["fontsize"] = {}
style["fontsize"]["xticks"]     = 16
style["fontsize"]["yticks"]     = 16
style["fontsize"]["xlabel"]     = 18
style["fontsize"]["ylabel"]     = 18
style["fontsize"]["title"]      = 24
style["fontsize"]["suptitle"]   = 20
style["fontsize"]["legend"]     = 16
style["fontsize"]["text_label"] = 18


# Power Coefficient
ax = plt.subplot(2,1,1)
for i,ki in enumerate(gp_model_data["ki"]):
	data_label = "GP model ($k_i$ = %0.2f)" % ki
	plt.plot(gp_model_data["CT"], gp_model_data["CP"][:,i],color="black",
		linestyle=style["linestyle"][i],linewidth=2,label=data_label)

plt.plot(test_data["CT"], test_data["CP"],color="black",marker='o',linestyle="none",
	fillstyle="full",markersize=style["markersize"],label="Test data")

plt.grid()
plt.xlim(xmin=0)
plt.ylim(ymin=0)
[xmin,xmax] = plt.gca().get_xlim()
plt.xticks(np.arange(xmin, xmax, 0.005),fontsize=style["fontsize"]["xticks"])
[ymin,ymax] = plt.gca().get_ylim()
plt.yticks(np.arange(ymin, 1.1*ymax, 0.0005),fontsize=style["fontsize"]["yticks"])
ax.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.xlabel('Thrust coefficient', fontsize=style["fontsize"]["xlabel"])
plt.ylabel('Power coefficient', fontsize=style["fontsize"]["ylabel"])
plt.title("Power Coefficient",fontsize=style["fontsize"]["title"])

#Figure of Merit
plt.subplot(2,1,2)
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
plt.xticks(np.arange(xmin, xmax, 0.005), fontsize=style["fontsize"]["xticks"])
[ymin,ymax] = plt.gca().get_ylim()
plt.yticks(np.arange(ymin, 1.1*ymax, 0.1),fontsize=style["fontsize"]["yticks"])
plt.xlabel('Thrust coefficient', fontsize=style["fontsize"]["xlabel"])
plt.ylabel('Figure of merit', fontsize=style["fontsize"]["ylabel"])
plt.title("Figure of Merit",fontsize = style["fontsize"]["title"])
plt.legend(numpoints = 1,loc='lower right', fontsize=style["fontsize"]["legend"],framealpha=1)


plt.tight_layout()
plt.subplots_adjust(left=0.18,right=0.95,bottom=0.08,top=0.95)
plt.savefig('rotor_validation_plot_01.pdf')