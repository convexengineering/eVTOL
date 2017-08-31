#Plot A-weighting response function

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/' + '..'))

import numpy as np
from gpkit import ureg
from matplotlib import pyplot as plt
from noise_models import noise_weighting

#Computations
f = np.logspace(np.log10(10),np.log10(40000),100)*ureg.turn/ureg.s
SPL = np.zeros(np.size(f))
dBA_offset = noise_weighting(f,SPL,type="A")


# Plotting commands
plt.ion()
fig1 = plt.figure(figsize=(12,6), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

plt.plot(f.to(ureg.turn/ureg.s).magnitude,dBA_offset,'k-',linewidth=3)

plt.xlim(xmin=np.min(f.to(ureg.turn/ureg.s).magnitude),
	xmax=np.max(f.to(ureg.turn/ureg.s).magnitude))
plt.xscale("log")
plt.grid()
plt.xlabel('Frequency (Hz)', fontsize = 16)
plt.ylabel('Relative response (dB)', fontsize = 16)
plt.title("A-Weighting Response Function", fontsize = 24)
plt.tight_layout()
plt.savefig('dBA_response_function.pdf')