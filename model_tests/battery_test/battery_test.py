#Code for testing the battery model. No Peukert effect as yet.

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../..'))

import numpy as np
from gpkit import Variable, Model
from aircraft_models import Battery
from gpkit import ureg

if __name__=="__main__":

	C_m = 300*ureg.Wh/ureg.kg
	g = 9.807*ureg.m/ureg.s**2
	W = 400*ureg.lbf
	battery_cost_per_C = 400*ureg.kWh**-1
	
	testBattery = Battery()

	W_units = testBattery["W"].units
	g_units = testBattery["g"].units
	testBattery.substitutions.update({"C_m":C_m,"W":W,"g":g,"cost_per_C":battery_cost_per_C})
	
	testBatteryModel = Model(1/testBattery["C"],testBattery)
	testBatterySolution = testBatteryModel.solve(verbosity=0)
	print testBatterySolution.summary()