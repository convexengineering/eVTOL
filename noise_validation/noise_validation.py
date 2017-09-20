#Validation of rotor model against test data

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/..'))

import math
import numpy as np
from gpkit import Variable, Model, Vectorize, ureg
from matplotlib import pyplot as plt
from noise_models import vortex_noise

#Test data, from Leishman textbook

pi = math.pi

generic_inputs = {}
generic_inputs["\rho"] = 1.19956*ureg.kg/ureg.m**3
generic_inputs["delta_S"] = 300*ureg.ft

heli_info = {}

heli_info["CH-3C"] = {}
heli_info["CH-3C"]["D"] = 62*ureg.ft
heli_info["CH-3C"]["R"] = heli_info["CH-3C"]["D"]/2
heli_info["CH-3C"]["A"] = pi*heli_info["CH-3C"]["R"]**2
heli_info["CH-3C"]["B"] = 5
heli_info["CH-3C"]["s"] = 0.078
heli_info["CH-3C"]["t_c"] = 0.12
heli_info["CH-3C"]["St"] = 0.28

heli_info["CH-53A"] = {}
heli_info["CH-53A"]["D"] = 72*ureg.ft
heli_info["CH-53A"]["R"] = heli_info["CH-3C"]["D"]/2
heli_info["CH-53A"]["A"] = pi*heli_info["CH-53A"]["R"]**2
heli_info["CH-53A"]["B"] = 6
heli_info["CH-53A"]["s"] = 0.115
heli_info["CH-53A"]["t_c"] = 0.12
heli_info["CH-53A"]["St"] = 0.28

test_data = {}
test_data["CH-3C"] = {}
test_data["CH-53A"] = {}

test_data["CH-3C"]["183.0 rpm"] = {}
test_data["CH-3C"]["183.0 rpm"]["omega"] = 183*ureg.rpm
test_data["CH-3C"]["183.0 rpm"]["T"] = [13400,16200,18700,20500]*ureg.lbf
test_data["CH-3C"]["183.0 rpm"]["SPL_measured"] = [76,79,81,82]

test_data["CH-3C"]["203.0 rpm"] = {}
test_data["CH-3C"]["203.0 rpm"]["omega"] = 203*ureg.rpm
test_data["CH-3C"]["203.0 rpm"]["T"] = [16300,18100,19900,21400]*ureg.lbf
test_data["CH-3C"]["203.0 rpm"]["SPL_measured"] = [81,81,82,83]

test_data["CH-3C"]["213.0 rpm"] = {}
test_data["CH-3C"]["213.0 rpm"]["omega"] = 213*ureg.rpm
test_data["CH-3C"]["213.0 rpm"]["T"] = [14500,18200,20000]*ureg.lbf
test_data["CH-3C"]["213.0 rpm"]["SPL_measured"] = [81,83,83]

test_data["CH-53A"]["166.0 rpm"] = {}
test_data["CH-53A"]["166.0 rpm"]["omega"] = 166*ureg.rpm
test_data["CH-53A"]["166.0 rpm"]["T"] = [24000,28400,32000,36200,39000]*ureg.lbf
test_data["CH-53A"]["166.0 rpm"]["SPL_measured"] = [80,81,83,83,85]

test_data["CH-53A"]["185.5 rpm"] = {}
test_data["CH-53A"]["185.5 rpm"]["omega"] = 185.5*ureg.rpm
test_data["CH-53A"]["185.5 rpm"]["T"] = [25000,30100,36200,41600]*ureg.lbf
test_data["CH-53A"]["185.5 rpm"]["SPL_measured"] = [83,83,85,87]

test_data["CH-53A"]["215.0 rpm"] = {}
test_data["CH-53A"]["215.0 rpm"]["omega"] = 215*ureg.rpm
test_data["CH-53A"]["215.0 rpm"]["T"] = [23700,29600,37900,43520]*ureg.lbf
test_data["CH-53A"]["215.0 rpm"]["SPL_measured"] = [85,86,89,90]


rho = generic_inputs["\rho"]
delta_S = generic_inputs["delta_S"]

for heli in test_data:
	for omega_data in test_data[heli]:
		
		test_data[heli][omega_data]["SPL_calculated"] = np.zeros(np.size(test_data[heli][omega_data]["T"]))
		
		for i,T in enumerate(test_data[heli][omega_data]["T"]):
			
			R = heli_info[heli]["R"]
			
			omega_rad_s = test_data[heli][omega_data]["omega"].to(ureg.radian/ureg.s)
			omega_rad_s = (omega_rad_s.magnitude)*ureg.s**-1
			VT = omega_rad_s*R
			
			s = heli_info[heli]["s"]
			
			A = heli_info[heli]["A"]
			CT = (T / (0.5*rho*(VT**2)*A)).to(ureg.dimensionless)
			Cl_mean = 3*CT/s
			
			B = heli_info[heli]["B"]
			t_c = heli_info[heli]["t_c"]
			St = heli_info[heli]["St"]
			
			f_peak, SPL, spectrum = vortex_noise(T_perRotor=T,R=R,VT=VT,s=s,Cl_mean=Cl_mean,
					N=1,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=t_c,St=St,weighting="None")
			test_data[heli][omega_data]["SPL_calculated"][i] = SPL



'''
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
plt.xlabel('Thrust coefficient', fontsize = 20)
plt.ylabel('Figure of merit', fontsize = 20)
plt.title("Figure of Merit",fontsize = 24)
plt.legend(numpoints = 1,loc='lower right', fontsize = 18)


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
plt.xlabel('Thrust coefficient', fontsize = 20)
plt.ylabel('Power coefficient', fontsize = 20)
plt.title("Power Coefficient",fontsize = 24)
plt.legend(numpoints = 1,loc='lower right', fontsize = 18)


title_str = "Rotor Aerodynamic Model Validation (s = %0.3f; $C_{d_0}$ = %0.2f)"\
	% (gp_model_data["s"], gp_model_data["Cd0"])
plt.suptitle(title_str,fontsize = 26)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.98,bottom=0.1,top=0.86)
plt.savefig('rotor_validation_plot_01.pdf')
'''