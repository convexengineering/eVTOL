import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/' + '..'))

import numpy as np
from gpkit import Model, ureg
from aircraft_models import OnDemandAircraft 
from aircraft_models import OnDemandSizingMission, OnDemandRevenueMission
from aircraft_models import OnDemandDeadheadMission, OnDemandMissionCost
from study_input_data import generic_data, configuration_data

def test():
	#String inputs
	reserve_type="FAA_heli"
	sizing_mission_type="piloted"
	revenue_mission_type="piloted"
	deadhead_mission_type="autonomous"

	problem_subDict = {}
	
	Aircraft = OnDemandAircraft(autonomousEnabled=True)
	problem_subDict.update({
		Aircraft.L_D_cruise: 14., #estimated L/D in cruise
		Aircraft.eta_cruise: 0.85, #propulsive efficiency in cruise
		Aircraft.tailRotor_power_fraction_hover: 0.0001,
		Aircraft.tailRotor_power_fraction_levelFlight: 0.0001,
		Aircraft.cost_per_weight: 350*ureg.lbf**-1, #vehicle cost per unit empty weight
		Aircraft.battery.C_m: 400*ureg.Wh/ureg.kg, #battery energy density
		Aircraft.battery.cost_per_C: 400*ureg.kWh**-1, #battery cost per unit energy capacity
		Aircraft.rotors.N: 12, #number of propellers
		Aircraft.rotors.Cl_mean_max: 1.0, #maximum allowed mean lift coefficient
		Aircraft.structure.weight_fraction: 0.55, #empty weight fraction
		Aircraft.electricalSystem.eta: 0.9, #electrical system efficiency	
	})

	SizingMission = OnDemandSizingMission(Aircraft,mission_type=sizing_mission_type,
		reserve_type=reserve_type)
	problem_subDict.update({
		SizingMission.mission_range: 87*ureg.nautical_mile,#mission range
		SizingMission.V_cruise: 200*ureg.mph,#cruising speed
		SizingMission.t_hover: 120*ureg.s,#hover time
		SizingMission.T_A: 15.*ureg("lbf")/ureg("ft")**2,#disk loading
		SizingMission.passengers.N_passengers: 3,#Number of passengers
	})

	RevenueMission = OnDemandRevenueMission(Aircraft,mission_type=revenue_mission_type)
	problem_subDict.update({
		RevenueMission.mission_range: 30*ureg.nautical_mile,#mission range
		RevenueMission.V_cruise: 200*ureg.mph,#cruising speed
		RevenueMission.t_hover: 30*ureg.s,#hover time
		RevenueMission.passengers.N_passengers: 2,#Number of passengers
		RevenueMission.time_on_ground.charger_power: 200*ureg.kW, #Charger power
	})

	DeadheadMission = OnDemandDeadheadMission(Aircraft,mission_type=deadhead_mission_type)
	problem_subDict.update({
		DeadheadMission.mission_range: 30*ureg.nautical_mile,#mission range
		DeadheadMission.V_cruise: 200*ureg.mph,#cruising speed
		DeadheadMission.t_hover: 30*ureg.s,#hover time
		DeadheadMission.passengers.N_passengers: 0.00001,#Number of passengers
		DeadheadMission.time_on_ground.charger_power: 200*ureg.kW, #Charger power
	})

	MissionCost = OnDemandMissionCost(Aircraft,RevenueMission,DeadheadMission)
	problem_subDict.update({
		MissionCost.revenue_mission_costs.operating_expenses.pilot_cost.wrap_rate: 70*ureg.hr**-1,#pilot wrap rate
		MissionCost.revenue_mission_costs.operating_expenses.maintenance_cost.wrap_rate: 60*ureg.hr**-1, #mechanic wrap rate
		MissionCost.revenue_mission_costs.operating_expenses.maintenance_cost.MMH_FH: 0.6, #maintenance man-hours per flight hour
		MissionCost.deadhead_mission_costs.operating_expenses.pilot_cost.wrap_rate: 70*ureg.hr**-1,#pilot wrap rate
		MissionCost.deadhead_mission_costs.operating_expenses.maintenance_cost.wrap_rate: 60*ureg.hr**-1, #mechanic wrap rate
		MissionCost.deadhead_mission_costs.operating_expenses.maintenance_cost.MMH_FH: 0.6, #maintenance man-hours per flight hour
		MissionCost.deadhead_ratio: 0.2, #deadhead ratio
	})
	
	problem = Model(MissionCost["cost_per_trip"],
		[Aircraft, SizingMission, RevenueMission, DeadheadMission, MissionCost])
	problem.substitutions.update(problem_subDict)
	solution = problem.solve(verbosity=0)
	return solution

for i in range(1,51):
	print "Test run %0.0f" % i
	solution = test()