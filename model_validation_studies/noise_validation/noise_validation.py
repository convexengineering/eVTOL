#Validation of rotor model against test data

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../..'))

import math
import numpy as np
from gpkit import Variable, Model, Vectorize, ureg
from matplotlib import pyplot as plt
from matplotlib import ticker as mtick
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
test_data["CH-3C"] = [{} for i in range(3)]
test_data["CH-53A"] = [{} for i in range(3)]

test_data["CH-3C"][0]["omega"] = 183*ureg.rpm
test_data["CH-3C"][0]["T"] = [13400,16200,18700,20500]*ureg.lbf
test_data["CH-3C"][0]["SPL_measured"] = [76,79,81,82]

test_data["CH-3C"][1]["omega"] = 203*ureg.rpm
test_data["CH-3C"][1]["T"] = [16300,18100,19900,21400]*ureg.lbf
test_data["CH-3C"][1]["SPL_measured"] = [81,81,82,83]

test_data["CH-3C"][2]["omega"] = 213*ureg.rpm
test_data["CH-3C"][2]["T"] = [14500,18200,20000]*ureg.lbf
test_data["CH-3C"][2]["SPL_measured"] = [81,83,83]

test_data["CH-53A"][0]["omega"] = 166*ureg.rpm
test_data["CH-53A"][0]["T"] = [24000,28400,32000,36200,39000]*ureg.lbf
test_data["CH-53A"][0]["SPL_measured"] = [80,81,83,83,85]

test_data["CH-53A"][1]["omega"] = 185.5*ureg.rpm
test_data["CH-53A"][1]["T"] = [25000,30100,36200,41600]*ureg.lbf
test_data["CH-53A"][1]["SPL_measured"] = [83,83,85,87]

test_data["CH-53A"][2]["omega"] = 215*ureg.rpm
test_data["CH-53A"][2]["T"] = [23700,29600,37900,43520]*ureg.lbf
test_data["CH-53A"][2]["SPL_measured"] = [85,86,89,90]

rho = generic_inputs["\rho"]
delta_S = generic_inputs["delta_S"]

for heli in test_data:
	
	R = heli_info[heli]["R"]
	s = heli_info[heli]["s"]
	A = heli_info[heli]["A"]
	B = heli_info[heli]["B"]
	t_c = heli_info[heli]["t_c"]
	St = heli_info[heli]["St"]
	
	for i,data in enumerate(test_data[heli]):
	
		test_data[heli][i]["SPL_calculated"] = np.zeros(np.size(test_data[heli][i]["T"]))
		
		omega_rad_s = test_data[heli][i]["omega"].to(ureg.radian/ureg.s)
		omega_rad_s = (omega_rad_s.magnitude)*ureg.s**-1
		VT = omega_rad_s*R
		
		for j,T in enumerate(test_data[heli][i]["T"]):
		
			CT = (T / (0.5*rho*(VT**2)*A)).to(ureg.dimensionless)
			Cl_mean = 3*CT/s
			
			f_peak, SPL, spectrum = vortex_noise(T_perRotor=T,R=R,VT=VT,s=s,Cl_mean=Cl_mean,
					N=1,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=t_c,St=St,weighting="None")
			test_data[heli][i]["SPL_calculated"][j] = SPL


#Plotting commands

plt.ion()
fig1 = plt.figure(figsize=(12,12), dpi=80)
plt.show()

style = {}
style["linestyle"] = ["-","--","-."]
style["markersize"] = 10


for j, heli in enumerate(test_data):
	
	for i,data in enumerate(test_data[heli]):
		
		plt.subplot(3,2,2*i+j+1)
	
		omega = test_data[heli][i]["omega"].to(ureg.rpm).magnitude
		
		T = test_data[heli][i]["T"].to(ureg.lbf).magnitude/1e3
		SPL_calculated = test_data[heli][i]["SPL_calculated"]
		SPL_measured = test_data[heli][i]["SPL_measured"]
		max_SPL_error = np.max(np.abs(SPL_calculated-SPL_measured))
		
		plt.plot(T,SPL_calculated,color="black",linestyle="-",linewidth=2,
			marker="o",markersize=style["markersize"],label="Calculated")
		plt.plot(T,SPL_measured,color="black",linestyle="-",linewidth=2,
			marker="s",markersize=style["markersize"],label="Measured")
		
		[ymin,ymax] = plt.gca().get_ylim()
		plt.ylim(ymin=ymin-1,ymax=ymax+1)
		plt.grid()
		plt.xticks(fontsize=12)
		plt.yticks(fontsize=12)
		plt.xlabel(r'Thrust (lbf $\times \; 10^3$)', fontsize = 16)
		plt.ylabel('SPL (dB)', fontsize = 16)
		
		title_str = heli \
			+ " (%0.0f rpm; max error = %0.1f dB)" \
			% (omega, max_SPL_error)
		plt.title(title_str,fontsize = 18)
		plt.legend(numpoints = 2,loc='lower right', fontsize = 14,framealpha=1)
		

title_str = "Noise Model Validation ($\Delta S$ = %0.0f ft)" \
	% generic_inputs["delta_S"].to(ureg.ft).magnitude
plt.suptitle(title_str,fontsize = 26)
plt.tight_layout()
plt.subplots_adjust(left=0.06,right=0.98,bottom=0.05,top=0.92)
plt.savefig('noise_validation_plot_01.pdf')
