#Design-space sweep for the Joby S2

import math
import numpy as np
from gpkit import Variable, VectorVariable, Model, Vectorize
from rotor_models import rotors_analysis_function
from standard_atmosphere import stdatmo
from matplotlib import pyplot as plt


import pint
ureg = pint.UnitRegistry()

#Analysis representative of the Joby S2

N = 12
s = 0.1
SPL_requirement = 100 #constraint not really enforced
W_hover = 2000*ureg.lbf 
R = 1.804*ureg.ft 
h=0*ureg.ft
CL_mean_max = 1.4 #reasonable guess
Mtip_max = 0.9 #maximum tip Mach number


#find the lowest tip speed that works (limited by CLmean constraint)
low_data = rotors_analysis_function(T=W_hover,VT="unconstrained",h=h,N=N,
	R=R,s=s,CL_mean_max=CL_mean_max,SPL_requirement=SPL_requirement,
	print_summary="No")
VT_min = low_data[0]

#find the highest tip speed that works (limited by tip Mach number)
atmospheric_data = stdatmo(h)
VT_max = Mtip_max*atmospheric_data["a"].to(ureg.ft/ureg.s)

#sweep from lowest to highest
VT_array = np.linspace(VT_min.to(ureg.ft/ureg.s).magnitude,
	VT_max.to(ureg.ft/ureg.s).magnitude,10)*ureg.ft/ureg.s
FOM_array = np.zeros(np.size(VT_array))
P_array = np.zeros(np.size(VT_array))
CL_mean_array = np.zeros(np.size(VT_array))
SPL_array = np.zeros(np.size(VT_array))

for i, VT in enumerate(VT_array):

	[VTo,P,FOM,CL_mean,SPL] = rotors_analysis_function(T=W_hover,VT=VT,
		h=h,N=N,R=R,s=s,CL_mean_max=CL_mean_max,SPL_requirement=SPL_requirement,
		print_summary="No")
	P_array[i] = P.to(ureg.hp).magnitude
	FOM_array[i] = FOM
	CL_mean_array[i] = CL_mean
	SPL_array[i] = SPL

VT_array = VT_array.to(ureg.ft/ureg.s).magnitude

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
plt.plot(VT_array,CL_mean_array,color="black", linewidth=1.5, linestyle="-", marker='s', markersize=8)	 
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
	+ "$W_{hover} = " + str(W_hover.to(ureg.lbf).magnitude) + "\; lbf$; " \
	+ "$R = " + str(R.to(ureg.ft).magnitude) + "\; ft$; " \
	+ "$s = " + str(s) + "$"

plt.suptitle(title_str,fontsize = 20)

plt.tight_layout()#makes sure subplots are spaced neatly
plt.subplots_adjust(left=0.125,right=0.9,bottom=0.1,top = 0.85)#adds space at the top for the title
