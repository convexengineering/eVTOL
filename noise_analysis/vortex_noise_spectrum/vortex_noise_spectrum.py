# Plot the vortex noise spectrum

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + "/../../models"))

import numpy as np
from matplotlib import pyplot as plt


spectrum = {}
spectrum["fr"]       = np.array([0.5,  1,    2,    4,    8,     16   ])
spectrum["offsets"]  = np.array([7.92, 4.17, 8.33, 8.75, 12.92, 13.33])

plt.ion()
fig1 = plt.figure(figsize=(5,3), dpi=80)
plt.rc('axes', axisbelow=True)
plt.show()

style = {}
style["rotation"]         = -45
style["legend_ncols"]     = 2
style["bar_width_wide"]   = 0.7
style["bar_width_medium"] = 0.3
style["bar_width_narrow"] = 0.2
style["offsets"]          = [-0.25, 0, 0.25]
style["colors"]           = ["grey", "w", "k", "lightgrey"]

style["linestyle"]  = ["-","-","-","-","--","--","--","--"]
style["marker"]     = ["s","o","^","v","s","o","^","v"]
style["fillstyle"]  = ["full","full","full","full","none","none","none","none"]
style["markersize"] = 10

style["fontsize"] = {}
style["fontsize"]["xticks"]     = 14
style["fontsize"]["yticks"]     = 14
style["fontsize"]["xlabel"]     = 18
style["fontsize"]["ylabel"]     = 16
style["fontsize"]["title"]      = 16
style["fontsize"]["legend"]     = 14
style["fontsize"]["text_label"] = 18


plt.plot(spectrum["fr"], -spectrum["offsets"], 'k-o', fillstyle="full", markersize=10, linewidth=3, label="SPL offset")

plt.xscale('log')
plt.grid(zorder=0)
plt.xticks(spectrum["fr"], spectrum["fr"], fontsize=style["fontsize"]["xticks"])
plt.yticks(fontsize=style["fontsize"]["yticks"])
plt.xlabel(r'$f/f_{peak}$',                   fontsize=style["fontsize"]["xlabel"])
plt.ylabel('SPL offset\n(dB wrt overall SPL)', fontsize=style["fontsize"]["ylabel"])
plt.legend(loc='upper right',                 fontsize=style["fontsize"]["legend"], framealpha=1, numpoints=1)

plt.ylim(ymin=-15, ymax=0)

plt.tight_layout()
plt.subplots_adjust(left=0.26, right=0.98, bottom=0.22, top=0.96)
plt.savefig('vortex_noise_spectrum_plot_01.pdf')