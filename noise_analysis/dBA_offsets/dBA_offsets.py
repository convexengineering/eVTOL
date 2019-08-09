#Plot A-weighting response function, and dBA offset as a function of vortex-noise peak frequency

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../../models'))

import numpy      as np
from gpkit        import ureg
from matplotlib   import pyplot as plt
from noise_models import noise_weighting

#Computations
f_peak_array                  = np.logspace(np.log10(100), np.log10(40000),100) * ureg.turn / ureg.s
A_weighting_response_function = np.zeros(np.size(f_peak_array))
vortex_dBA_offset             = np.zeros(np.size(f_peak_array))

A_weighting_response_function = noise_weighting(f=f_peak_array, SPL=A_weighting_response_function, weighting="A")

for i,f_peak in enumerate(f_peak_array):
	
	spectrum = {}
	spectrum["f"]   = f_peak * [0.5,1,2,4,8,16]
	spectrum["SPL"] = -np.array([7.92,4.17,8.33,8.75,12.92,13.33])
	spectrum["SPL"] = noise_weighting(f=spectrum["f"], SPL=spectrum["SPL"], weighting="A")
	
	fr = (spectrum["f"]/f_peak).to(ureg.dimensionless)  # Frequency ratio array

	weighted_p_ratio_squared = 0

	for j in range(0,np.size(fr)-1):
		fr1 = fr[j]
		fr2 = fr[j+1]
		SPL1 = spectrum["SPL"][j]
		SPL2 = spectrum["SPL"][j+1]

		a = (SPL2-SPL1)/(np.log10(fr2)-np.log10(fr1))
		b = SPL2 - a*np.log10(fr2)

		leading_term = (10**(b/10))/((a/10) + 1)
		fr_term      = fr2**((a/10) + 1) - fr1**((a/10) + 1)

		weighted_p_ratio_squared += leading_term*fr_term

	vortex_dBA_offset[i] = 10*np.log10(weighted_p_ratio_squared)


# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(6, 5), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()


plt.plot(f_peak_array.to(ureg.turn/ureg.s).magnitude,A_weighting_response_function,
	'k--',linewidth=3,label="$A(f)$")
plt.plot(f_peak_array.to(ureg.turn/ureg.s).magnitude,vortex_dBA_offset,
	'k-',linewidth=3,label="$A(f_{peak}, vortex)$")

plt.xlim(xmin=np.min(f_peak_array.to(ureg.turn/ureg.s).magnitude),
	xmax=np.max(f_peak_array.to(ureg.turn/ureg.s).magnitude))
plt.xscale("log")
plt.grid()
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)
plt.xlabel('Frequency (Hz)', fontsize = 20)
plt.ylabel('Relative response (dB)', fontsize = 20)
plt.legend(fontsize = 20, loc="lower center", framealpha=1)
plt.title("A-Weighting Response", fontsize = 26)
plt.tight_layout()
plt.savefig('dBA_offsets.pdf')
