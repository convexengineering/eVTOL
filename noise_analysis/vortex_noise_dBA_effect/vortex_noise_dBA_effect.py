#Plot dBA offset as a function of vortex-noise peak frequency

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/' + '..'))

import numpy as np
from gpkit import ureg
from matplotlib import pyplot as plt
from noise_models import noise_weighting

#Computations
f_peak_array = np.logspace(np.log10(10),np.log10(40000),100)*ureg.turn/ureg.s
vortex_dBA_offset = np.zeros(np.size(f_peak_array))


for i,f_peak in enumerate(f_peak_array):
	
	spectrum = {}
	spectrum["f"] = f_peak*[0.5,1,2,4,8,16]
	spectrum["SPL"] = -np.array([7.92,4.17,8.33,8.75,12.92,13.33])

	spectrum["SPL"] = noise_weighting(spectrum["f"],spectrum["SPL"],type="A")
	
	fr = (spectrum["f"]/f_peak).to(ureg.dimensionless) #frequency ratio array

	weighted_p_ratio_squared = 0

	for j in range(0,np.size(fr)-1):
		fr1 = fr[j]
		fr2 = fr[j+1]
		SPL1 = spectrum["SPL"][j]
		SPL2 = spectrum["SPL"][j+1]

		a = (SPL2-SPL1)/(np.log10(fr2)-np.log10(fr1))
		b = SPL2 - a*np.log10(fr2)

		leading_term = (10**(b/10))/((a/10) + 1)
		fr_term = fr2**((a/10) + 1) - fr1**((a/10) + 1)

		weighted_p_ratio_squared += leading_term*fr_term

	vortex_dBA_offset[i] = 10*np.log10(weighted_p_ratio_squared)


# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(12,6), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

plt.plot(f_peak_array.to(ureg.turn/ureg.s).magnitude,vortex_dBA_offset,'k-',linewidth=3)

plt.xlim(xmin=np.min(f_peak_array.to(ureg.turn/ureg.s).magnitude),
	xmax=np.max(f_peak_array.to(ureg.turn/ureg.s).magnitude))
plt.xscale("log")
plt.grid()
plt.xlabel('Peak frequency (Hz)', fontsize = 16)
plt.ylabel('Relative response (dB)', fontsize = 16)
plt.title("Vortex Noise A-Weighting Response", fontsize = 24)
plt.tight_layout()
plt.savefig('vortex_noise_dBA_effect.pdf')