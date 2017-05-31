#Standard-atmospheric function for Python. Not GP-compatible. Uses a lookup table.
#Pint is used to ensure unit consistency

import numpy as np
import scipy.interpolate as interp
import pint 
ureg = pint.UnitRegistry()

def stdatmo(h):
	
	data_from_file = np.loadtxt("stdatmo_table.txt",skiprows=2)

	#units set manually to those in the lookup table
	data_from_file = {"h":data_from_file[:,0]*ureg.m,#altitude
		"\rho":data_from_file[:,1]*ureg.kg/ureg.m**3,#air density
		"a":data_from_file[:,2]*ureg.m/ureg.s,#speed of sound
		"T":data_from_file[:,3]*ureg.K,#temperature
		"P":data_from_file[:,4]*ureg.Pa,#pressure
		"kvisc":data_from_file[:,5]*ureg.m**2/ureg.s}#kinematic viscosity

	#these functions all use SI units
	interp_fcns = {"\rho":interp.interp1d(data_from_file["h"].magnitude,data_from_file["\rho"].magnitude,kind='cubic'),
		"a":interp.interp1d(data_from_file["h"].magnitude,data_from_file["a"].magnitude,kind='cubic'),
		"T":interp.interp1d(data_from_file["h"].magnitude,data_from_file["T"].magnitude,kind='cubic'),
		"P":interp.interp1d(data_from_file["h"].magnitude,data_from_file["P"].magnitude,kind='cubic'),
		"kvisc":interp.interp1d(data_from_file["h"].magnitude,data_from_file["kvisc"].magnitude,kind='cubic')}

	#output is a dictionary
	h_correctUnits = h.to(data_from_file["h"].units).magnitude
	output = {"\rho":interp_fcns["\rho"](h_correctUnits)*data_from_file["\rho"].units,
		"a":interp_fcns["a"](h_correctUnits)*data_from_file["a"].units,
		"T":interp_fcns["T"](h_correctUnits)*data_from_file["T"].units,
		"P":interp_fcns["P"](h_correctUnits)*data_from_file["P"].units,
		"kvisc":interp_fcns["kvisc"](h_correctUnits)*data_from_file["kvisc"].units,}

	return output


if __name__=="__main__":
	#Small test case
	h = np.linspace(0,40000,2)*ureg.ft
	atmospheric_data = stdatmo(h)
	print atmospheric_data["\rho"]