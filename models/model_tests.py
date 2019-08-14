# Test case

import numpy as np
from gpkit                  import Model, ureg
from copy                   import deepcopy
from matplotlib             import pyplot as plt
from aircraft_models        import OnDemandAircraft
from mission_models         import OnDemandSizingMission, OnDemandRevenueMission, OnDemandDeadheadMission
from cost_models            import OnDemandMissionCost
from noise_models           import vortex_noise
from standard_substitutions import generic_data, configs

def test():

	for config in configs:

		aircraft = OnDemandAircraft()
		aircraft = aircraft.standard_substitutions(config=config, autonomousEnabled=generic_data["autonomousEnabled"])

		sizing_mission = OnDemandSizingMission(aircraft=aircraft)
		sizing_mission = sizing_mission.standard_substitutions(piloted=generic_data["isSizingMissionPiloted"], reserve=generic_data["reserve"])

		revenue_mission = OnDemandRevenueMission(aircraft=aircraft)
		revenue_mission = revenue_mission.standard_substitutions(piloted=generic_data["isRevenueMissionPiloted"])

		deadhead_mission = OnDemandDeadheadMission(aircraft=aircraft)
		deadhead_mission = deadhead_mission.standard_substitutions(piloted=generic_data["isDeadheadMissionPiloted"])

		mission_cost = OnDemandMissionCost(aircraft=aircraft, revenue_mission=revenue_mission, deadhead_mission=deadhead_mission)
		mission_cost = mission_cost.standard_substitutions(isRevenueMissionPiloted=generic_data["isRevenueMissionPiloted"], isDeadheadMissionPiloted=generic_data["isDeadheadMissionPiloted"])

		objective_function = mission_cost.cpt
		problem            = Model(objective_function, [aircraft, sizing_mission, revenue_mission, deadhead_mission, mission_cost])
		solution           = problem.solve(verbosity=0)

if __name__=="__main__":

	test()