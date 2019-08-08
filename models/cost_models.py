# Cost models.

import math
import numpy as np
from gpkit import Variable, Model, Vectorize, ureg

from aircraft_models        import OnDemandAircraft
from mission_models         import OnDemandSizingMission, OnDemandRevenueMission, OnDemandDeadheadMission
from standard_substitutions import on_demand_mission_cost_substitutions

pi = math.pi


class OnDemandMissionCost(Model):

	def standard_substitutions(self, isRevenueMissionPiloted=True, isDeadheadMissionPiloted=False):
		return on_demand_mission_cost_substitutions(mission_cost=self, isRevenueMissionPiloted=isRevenueMissionPiloted, isDeadheadMissionPiloted=isDeadheadMissionPiloted)
	
	# Includes both revenue and deadhead missions
	def setup(self, aircraft, revenue_mission, deadhead_mission):

		N_passengers  = revenue_mission.passengers.N
		trip_distance = revenue_mission.cruise_segment.d_segment

		self.cpt              = cpt              = Variable("cost_per_trip",               "-",      "Cost for one trip")
		self.cptpp            = cptpp            = Variable("cost_per_trip_per_passenger", "-",      "Cost for one trip, per passenger carried on revenue trip")
		self.cpt_passenger_km = cpt_passenger_km = Variable("cost_per_passenger_km",       "km**-1", "Cost per trip, per seat (passenger) kilometer")
		self.cpt_revenue      = cpt_revenue      = Variable("revenue_cost_per_trip",       "-",      "Portion of the cost per trip incurred during the revenue-generating flights")
		self.cpt_deadhead     = cpt_deadhead     = Variable("deadhead_cost_per_trip",      "-",      "Portion of the cost per trip incurred during the deadhead flights")
		self.deadhead_ratio   = deadhead_ratio   = Variable("deadhead_ratio",              "-",      "Number of deadhead missions per total missions")
		self.NdNr             = NdNr             = Variable("N_{deadhead}/N_{revenue}",    "-",      "Number of deadhead missions per revenue mission")

		self.revenue_mission_cost  = revenue_mission_cost  = RevenueMissionCost( aircraft=aircraft, mission=revenue_mission)
		self.deadhead_mission_cost = deadhead_mission_cost = DeadheadMissionCost(aircraft=aircraft, mission=deadhead_mission)
		self.mission_costs         = mission_costs         = [revenue_mission_cost, deadhead_mission_cost]

		constraints = [mission_costs]

		constraints += [1. >= deadhead_ratio * (NdNr**-1 + 1.)]  # Obtained via algebraic manipulation 
		
		constraints += [cpt_revenue  == revenue_mission_cost.cost_per_mission        ]
		constraints += [cpt_deadhead == deadhead_mission_cost.cost_per_mission * NdNr]
		constraints += [cpt >= cpt_revenue + cpt_deadhead]
		
		constraints += [cpt == cptpp            * N_passengers                ]
		constraints += [cpt == cpt_passenger_km * N_passengers * trip_distance]
		
		return constraints

class RevenueMissionCost(Model):
	
	# Cost for one mission. Revenue and Deadhead cost models have exactly the same code. Simplifies data output.
	def setup(self, aircraft, mission):

		t_mission = mission.t_mission

		self.cost_per_mission = cost_per_mission = Variable("cost_per_mission", "-",      "Cost per mission")
		self.cost_per_time    = cost_per_time    = Variable("cost_per_time",    "hr**-1", "Cost per unit mission time")

		self.capital_expenses   = capital_expenses   = CapitalExpenses(aircraft=aircraft, mission=mission)
		self.operating_expenses = operating_expenses = OperatingExpenses(                 mission=mission)
		self.expenses           = expenses           = [capital_expenses, operating_expenses]
		
		constraints =  [expenses]
		constraints += [cost_per_mission   >= sum(c.cost_per_mission      for c in expenses)]
		constraints += [c.cost_per_mission == t_mission * c.cost_per_time for c in expenses]
		constraints += [cost_per_mission   == t_mission * cost_per_time]

		return constraints


class DeadheadMissionCost(Model):
	
	# Cost for one mission. Revenue and Deadhead cost models have exactly the same code. Simplifies data output.
	def setup(self, aircraft, mission):

		t_mission = mission.t_mission

		self.cost_per_mission = cost_per_mission = Variable("cost_per_mission", "-",      "Cost per mission")
		self.cost_per_time    = cost_per_time    = Variable("cost_per_time",    "hr**-1", "Cost per unit mission time")

		self.capital_expenses   = capital_expenses   = CapitalExpenses(aircraft=aircraft, mission=mission)
		self.operating_expenses = operating_expenses = OperatingExpenses(                 mission=mission)
		self.expenses           = expenses           = [capital_expenses, operating_expenses]
		
		constraints =  [expenses]
		constraints += [cost_per_mission   >= sum(c.cost_per_mission      for c in expenses)]
		constraints += [c.cost_per_mission == t_mission * c.cost_per_time for c in expenses]
		constraints += [cost_per_mission   == t_mission * cost_per_time]

		return constraints	


class AirframeAcquisitionCost(Model):
	
	def setup(self, aircraft):
		
		purchase_price = aircraft.airframe.purchase_price
		lifetime       = aircraft.airframe.lifetime
		
		self.cost_per_time    = cost_per_time    = Variable("cost_per_time",    "hr**-1", "Amortized vehicle acquisition cost (purchase price) per unit mission time")
		self.cost_per_mission = cost_per_mission = Variable("cost_per_mission", "-",      "Amortized vehicle acquisition cost (purchase price) per mission")

		constraints = [purchase_price == cost_per_time * lifetime]
		
		return constraints


class AvionicsAcquisitionCost(Model):
	
	def setup(self, aircraft):
		
		purchase_price = aircraft.avionics.purchase_price
		lifetime       = aircraft.avionics.lifetime
		
		self.cost_per_time    = cost_per_time    = Variable("cost_per_time",    "hr**-1", "Amortized avionics acquisition cost (purchase price) per unit mission time")
		self.cost_per_mission = cost_per_mission = Variable("cost_per_mission", "-",      "Amortized avionics acquisition cost (purchase price) per mission")
		
		constraints = [purchase_price == cost_per_time * lifetime]
		
		return constraints


class BatteryAcquisitionCost(Model):
	
	def setup(self, aircraft):
		
		purchase_price = aircraft.battery.purchase_price
		cycle_life     = aircraft.battery.cycle_life
		
		self.cost_per_time    = cost_per_time    = Variable("cost_per_time",    "hr**-1", "Amortized battery acquisition cost (purchase price) per unit mission time")
		self.cost_per_mission = cost_per_mission = Variable("cost_per_mission", "-",      "Amortized battery acquisition cost (purchase price) per mission")
		
		constraints = [cost_per_mission == purchase_price / cycle_life]
		
		return constraints


class CapitalExpenses(Model):
	
	def setup(self, aircraft, mission):

		t_mission = mission.t_mission

		self.airframe_cost = airframe_cost = AirframeAcquisitionCost(aircraft=aircraft)
		self.avionics_cost = avionics_cost = AvionicsAcquisitionCost(aircraft=aircraft)
		self.battery_cost  = battery_cost  = BatteryAcquisitionCost( aircraft=aircraft)
		self.costs         = costs         = [airframe_cost, avionics_cost, battery_cost]

		self.cost_per_time    = cost_per_time    = Variable("cost_per_time",    "hr**-1", "Capital expenses per unit mission time")
		self.cost_per_mission = cost_per_mission = Variable("cost_per_mission", "-",      "Capital expenses per mission")
		
		constraints =  [costs]
		constraints += [cost_per_mission   >= sum(c.cost_per_mission      for c in costs)]
		constraints += [c.cost_per_mission == t_mission * c.cost_per_time for c in costs]

		return constraints


class PilotCost(Model):
	
	def setup(self):

		self.pilots_per_aircraft = pilots_per_aircraft = Variable("pilots_per_aircraft", "-",      "Pilots per aircraft (assuming crew on board)")
		self.wrap_rate           = wrap_rate           = Variable("wrap_rate",           "hr**-1", "Pilot wrap rate (including benefits and overhead)")
		self.cost_per_time       = cost_per_time       = Variable("cost_per_time",       "hr**-1", "Pilot cost per unit mission time")
		self.cost_per_mission    = cost_per_mission    = Variable("cost_per_mission",    "-",      "Pilot cost per mission")

		constraints = [cost_per_time == wrap_rate * pilots_per_aircraft]
		
		return constraints


class MaintenanceCost(Model):
	
	def setup(self):

		self.MMH_FH           = MMH_FH           = Variable("MMH_FH",           "-",      "Maintenance man-hours per flight hour")
		self.wrap_rate        = wrap_rate        = Variable("wrap_rate",        "hr**-1", "Maintenance wrap rate (including benefits and overhead)")
		self.cost_per_time    = cost_per_time    = Variable("cost_per_time",    "hr**-1", "Maintenance cost per unit mission time")
		self.cost_per_mission = cost_per_mission = Variable("cost_per_mission", "-",      "Maintenance cost per mission")

		constraints = [cost_per_time == wrap_rate * MMH_FH]
		
		return constraints


class EnergyCost(Model):
	
	def setup(self, mission):

		E_charger = mission.ground_segment.E_charger

		self.cost_per_energy  = cost_per_energy  = Variable("cost_per_energy",  "kWh**-1", "Price of electricity")
		self.cost_per_time    = cost_per_time    = Variable("cost_per_time",    "hr**-1",  "Energy cost per unit mission time")
		self.cost_per_mission = cost_per_mission = Variable("cost_per_mission", "-",       "Energy cost per mission")

		constraints = [cost_per_mission == E_charger * cost_per_energy]

		return constraints

class IndirectOperatingCost(Model):
	
	def setup(self):

		self.cost_per_time    = cost_per_time    = Variable("cost_per_time",    "hr**-1", "IOC per unit mission time")
		self.cost_per_mission = cost_per_mission = Variable("cost_per_mission", "-",      "IOC per mission")

		constraints = []

		return constraints

class OperatingExpenses(Model):
	
	def setup(self, mission):

		t_mission = mission.t_mission

		self.cost_per_time    = cost_per_time    = Variable("cost_per_time",    "hr**-1", "Operating expenses per unit mission time")
		self.cost_per_mission = cost_per_mission = Variable("cost_per_mission", "-",      "Operating expenses per mission")
		
		self.DOC_per_time    = DOC_per_time    = Variable("DOC_per_time",    "hr**-1", "Direct operating cost per unit mission time")
		self.DOC_per_mission = DOC_per_mission = Variable("DOC_per_mission", "-",      "Direct operating cost per mission")
		self.IOC_fraction    = IOC_fraction    = Variable("IOC_fraction",    "-",      "IOC as a fraction of DOC")

		self.pilot_cost              = pilot_cost              = PilotCost()
		self.maintenance_cost        = maintenance_cost        = MaintenanceCost()
		self.energy_cost             = energy_cost             = EnergyCost(mission=mission)
		self.indirect_operating_cost = indirect_operating_cost = IndirectOperatingCost()
		
		self.costs                   = costs                   = [pilot_cost, maintenance_cost, energy_cost, indirect_operating_cost]
		self.direct_costs            = direct_costs            = [pilot_cost, maintenance_cost, energy_cost                         ]

		constraints = [costs]

		constraints += [cost_per_mission >= sum(c.cost_per_mission for c in costs)]
		constraints += [DOC_per_mission  >= sum(c.cost_per_mission for c in direct_costs)]
		
		constraints += [DOC_per_mission  == t_mission * DOC_per_time]
		constraints += [DOC_per_mission  == indirect_operating_cost.cost_per_mission / IOC_fraction]

		constraints += [c.cost_per_mission == t_mission * c.cost_per_time for c in costs]

		return constraints



if __name__ == "__main__":

	#Concept representative analysis

	# from noise_models import rotational_noise, vortex_noise, noise_weighting
	
	#String inputs
	# reserve_type="FAA_heli"
	# sizing_mission_type="piloted"

	aircraft = OnDemandAircraft()
	aircraft = aircraft.standard_substitutions(config="Compound heli", autonomousEnabled=True)

	sizing_mission = OnDemandSizingMission(aircraft=aircraft)
	sizing_mission = sizing_mission.standard_substitutions(piloted=True, reserve="20-minute loiter")

	revenue_mission = OnDemandRevenueMission(aircraft=aircraft)
	revenue_mission = revenue_mission.standard_substitutions(piloted=True)

	deadhead_mission = OnDemandDeadheadMission(aircraft=aircraft)
	deadhead_mission = deadhead_mission.standard_substitutions(piloted=False)

	mission_cost = OnDemandMissionCost(aircraft=aircraft, revenue_mission=revenue_mission, deadhead_mission=deadhead_mission)
	mission_cost = mission_cost.standard_substitutions(isRevenueMissionPiloted=True, isDeadheadMissionPiloted=False)

	objective_function = mission_cost.cpt
	problem            = Model(objective_function, [aircraft, sizing_mission, revenue_mission, deadhead_mission, mission_cost])
	solution           = problem.solve(verbosity=0)


	"""
	delta_S = 500*ureg.ft
	noise_weighting = "A"
	B = 5

	SPL_dict = {}
	missions = ["Sizing","Revenue","Deadhead"]

	for mission in missions:
		mission_name = "OnDemand" + mission + "Mission"
		
		T_perRotor = solution("T_perRotor_" + mission_name)[0]
		R = solution("R")
		VT = solution("VT_" + mission_name)[0]
		s = solution("s")
		Cl_mean = solution("Cl_{mean_{max}}")
		N = solution("N")

		f_peak, SPL, spectrum = vortex_noise(T_perRotor=T_perRotor,R=R,VT=VT,s=s,
			Cl_mean=Cl_mean,N=N,B=B,delta_S=delta_S,h=0*ureg.ft,t_c=0.12,St=0.28,
			weighting=noise_weighting)

		SPL_dict[mission] = SPL

	if (reserve_type == "FAA_aircram") or (reserve_type == "FAA_heli"):
		num = solution("t_{loiter}_OnDemandSizingMission").to(ureg.minute).magnitude
		reserve_type_string = " (%0.0f-minute loiter time)" % num
	if reserve_type == "Uber":
		num = solution("R_{divert}_OnDemandSizingMission").to(ureg.nautical_mile).magnitude
		reserve_type_string = " (%0.1f-nmi diversion distance)" % num

	
	print
	print "Concept representative analysis"
	print
	print "Battery energy density: %0.0f Wh/kg" \
		% solution("C_m_OnDemandAircraft/Battery").to(ureg.Wh/ureg.kg).magnitude
	print "Empty weight fraction: %0.4f" \
		% solution("weight_fraction_OnDemandAircraft/Structure")
	print "Cruise lift-to-drag ratio: %0.1f" \
		% solution("L_D_cruise_OnDemandAircram")
	print "Hover disk loading: %0.1f lbf/ft^2" \
		% solution("T/A_OnDemandSizingMission").to(ureg("N/m**2")).magnitude
	print "Rotor maximum mean lift coefficient: %0.2f" \
		% solution("Cl_{mean_{max}}_OnDemandAircraft/Rotors")
	print "Cruise propulsive efficiency: %0.2f" \
		% solution("\eta_{cruise}_OnDemandAircram")
	print "Electrical system efficiency: %0.2f" \
		% solution("\eta_OnDemandAircraft/ElectricalSystem")
	print "Observer distance: %0.0f m" % delta_S.to(ureg.ft).magnitude
	print "Noise weighting type: %s" % noise_weighting
	print
	print "Sizing Mission (%s)" % sizing_mission_type
	print "Mission range: %0.0f nmi" % \
		solution("mission_range_OnDemandSizingMission").to(ureg.nautical_mile).magnitude
	print "Number of passengers: %0.1f" % \
		solution("N_{passengers}_OnDemandSizingMission/Passengers")
	print "Reserve type: " + reserve_type + reserve_type_string
	print "Vehicle weight during mission: %0.0f lbf" % \
		solution("W_{mission}_OnDemandSizingMission").to(ureg.lbf).magnitude
	print "SPL in hover: %0.1f dB" % SPL_dict["Sizing"]
	print
	print "Revenue-Generating Mission (%s)" % revenue_mission_type
	print "Mission range: %0.0f nmi" % \
		solution("mission_range_OnDemandRevenueMission").to(ureg.nautical_mile).magnitude
	print "Number of passengers: %0.1f" % \
		solution("N_{passengers}_OnDemandRevenueMission/Passengers")
	print "Vehicle weight during mission: %0.0f lbf" % \
		solution("W_{mission}_OnDemandRevenueMission").to(ureg.lbf).magnitude
	print "Total time: %0.1f minutes" % \
		solution("t_{mission}_OnDemandRevenueMission").to(ureg.minute).magnitude
	print "Flight time: %0.1f minutes" % \
		solution("t_{flight}_OnDemandRevenueMission").to(ureg.minute).magnitude
	print "Time on ground: %0.1f minutes" % \
		solution("t_OnDemandRevenueMission/TimeOnGround").to(ureg.minute).magnitude
	print "SPL in hover: %0.1f dB" % SPL_dict["Revenue"]
	print
	print "Deadhead Mission (%s)" % deadhead_mission_type
	print "Mission range: %0.0f nmi" % \
		solution("mission_range_OnDemandDeadheadMission").to(ureg.nautical_mile).magnitude
	print "Number of passengers: %0.1f" % \
		solution("N_{passengers}_OnDemandDeadheadMission/Passengers")
	print "Vehicle weight during mission: %0.0f lbf" % \
		solution("W_{mission}_OnDemandDeadheadMission").to(ureg.lbf).magnitude
	print "Total time: %0.1f minutes" % \
		solution("t_{mission}_OnDemandDeadheadMission").to(ureg.minute).magnitude
	print "Flight time: %0.1f minutes" % \
		solution("t_{flight}_OnDemandDeadheadMission").to(ureg.minute).magnitude
	print "Time on ground: %0.1f minutes" % \
		solution("t_OnDemandDeadheadMission/TimeOnGround").to(ureg.minute).magnitude
	print "SPL in hover: %0.1f dB" % SPL_dict["Deadhead"]
	print
	print "Takeoff gross weight: %0.0f lbs" % \
		solution("TOGW_OnDemandAircram").to(ureg.lbf).magnitude
	print "Empty weight: %0.0f lbs" % \
		solution("W_OnDemandAircraft/Structure").to(ureg.lbf).magnitude
	print "Battery weight: %0.0f lbs" % \
		solution("W_OnDemandAircraft/Battery").to(ureg.lbf).magnitude
	print "Vehicle purchase price: $%0.0f " % \
		solution("purchase_price_OnDemandAircram")
	print "Avionics purchase price: $%0.0f " % \
		solution("purchase_price_OnDemandAircraft/Avionics")
	print "Battery purchase price:  $%0.0f " % \
		solution("purchase_price_OnDemandAircraft/Battery")
	print
	print "Cost per trip: $%0.2f" % \
		solution("cost_per_trip_OnDemandMissionCost")
	print "Cost per trip, per passenger: $%0.2f" % \
		solution("cost_per_trip_per_passenger_OnDemandMissionCost")
	print "Cost per trip, per seat mile: $%0.2f per mile" % \
		solution("cost_per_seat_mile_OnDemandMissionCost").to(ureg.mile**-1).magnitude
	print "Cost from revenue-generating flight: $%0.2f" % \
		solution("revenue_cost_per_trip_OnDemandMissionCost")
	print "Cost from deadhead flight: $%0.2f" % \
		solution("deadhead_cost_per_trip_OnDemandMissionCost")
	print
	print "Cost Breakdown from Revenue-Generating Flight Only (no deadhead)"
	print
	print "Vehicle capital expenses, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses")
	print "Amortized vehicle acquisition cost, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses/VehicleAcquisitionCost")
	print "Amortized avionics acquisition cost, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses/AvionicsAcquisitionCost")
	print "Amortized battery acquisition cost, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/CapitalExpenses/BatteryAcquisitionCost")
	print	
	print "Vehicle operating expenses, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses")
	print "Direct operating cost, per trip: $%0.2f" % \
		solution("DOC_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses")
	print "Indirect operating cost, per trip: $%0.2f" % \
		solution("IOC_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses")
	print
	print "Pilot cost, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses/PilotCost")
	print "Amortized maintenance cost, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses/MaintenanceCost")
	print "Energy cost, per trip: $%0.2f" % \
		solution("cost_per_mission_OnDemandMissionCost/RevenueMissionCost/OperatingExpenses/EnergyCost")
	"""
	
	#print solution.summary()
	
