#Top-level aircraft model.

import math
import numpy as np
from gpkit import Variable, Model, Vectorize, ureg
from standard_atmosphere import stdatmo

pi = math.pi


class OnDemandMissionCost(Model):
	#Includes both revenue and deadhead missions
	def setup(self,aircraft,revenue_mission,deadhead_mission):

		N_passengers = revenue_mission.passengers.N_passengers
		trip_distance = revenue_mission.mission_range

		cpt = Variable("cost_per_trip","-","Cost (in dollars) for one trip")
		cpt_revenue = Variable("revenue_cost_per_trip","-",
			"Portion of the cost per trip incurred during the revenue-generating flights")
		cpt_deadhead = Variable("deadhead_cost_per_trip","-",
			"Portion of the cost per trip incurred during the deadhead flights")
		cptpp = Variable("cost_per_trip_per_passenger","-",
			"Cost (in dollars) for one trip, per passenger carried on revenue trip")
		cpt_seat_mile = Variable("cost_per_seat_mile","mile**-1",
			"Cost per trip, per seat (passenger) mile")
		deadhead_ratio = Variable("deadhead_ratio","-","Number of deadhead missions per total missions")
		NdNr = Variable("N_{deadhead}/N_{typical}","-",
			"Number of deadhead missions per typical mission")

		revenue_mission_costs = RevenueMissionCost(aircraft,revenue_mission)
		deadhead_mission_costs = DeadheadMissionCost(aircraft,deadhead_mission)

		self.cpt = cpt
		self.cpt_revenue = cpt_revenue
		self.cpt_deadhead = cpt_deadhead
		self.cptpp = cptpp
		self.cpt_seat_mile = cpt_seat_mile
		self.deadhead_ratio = deadhead_ratio
		self.NdNr = NdNr
		self.revenue_mission_costs = revenue_mission_costs
		self.deadhead_mission_costs = deadhead_mission_costs

		constraints = []
		constraints += [revenue_mission_costs, deadhead_mission_costs]

		constraints += [NdNr >= deadhead_ratio*(NdNr+1)]
		
		constraints += [cpt_revenue == revenue_mission_costs.cost_per_mission]
		constraints += [cpt_deadhead == NdNr*deadhead_mission_costs.cost_per_mission]
		constraints += [cpt >= cpt_revenue + cpt_deadhead]
		
		constraints += [cpt == cptpp*N_passengers]
		constraints += [cpt == cpt_seat_mile*N_passengers*trip_distance]
		
		return constraints

class RevenueMissionCost(Model):
	#Cost for one mission. Exactly the same code as DeadheadMissionCost.
	def setup(self,aircraft,mission):

		t_mission = mission.t_mission

		cost_per_mission = Variable("cost_per_mission","-","Cost per mission")
		cost_per_time = Variable("cost_per_time","hr**-1","Cost per unit mission time")

		capital_expenses = CapitalExpenses(aircraft,mission)
		operating_expenses = OperatingExpenses(aircraft,mission)
		expenses = [capital_expenses, operating_expenses]

		self.cost_per_mission = cost_per_mission
		self.cost_per_time = cost_per_time
		self.capital_expenses = capital_expenses
		self.operating_expenses = operating_expenses

		constraints = []
		
		constraints += [expenses]
		constraints += [cost_per_mission >= sum(c.cost_per_mission for c in expenses)]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints

class DeadheadMissionCost(Model):
	#Cost for one mission. Exactly the same code as RevenueMissionCost.
	def setup(self,aircraft,mission):

		t_mission = mission.t_mission

		cost_per_mission = Variable("cost_per_mission","-","Cost per mission")
		cost_per_time = Variable("cost_per_time","hr**-1","Cost per unit mission time")

		capital_expenses = CapitalExpenses(aircraft,mission)
		operating_expenses = OperatingExpenses(aircraft,mission)
		expenses = [capital_expenses, operating_expenses]

		self.cost_per_mission = cost_per_mission
		self.cost_per_time = cost_per_time
		
		self.capital_expenses = capital_expenses
		self.operating_expenses = operating_expenses

		constraints = []
		
		constraints += [expenses]
		constraints += [cost_per_mission >= sum(c.cost_per_mission for c in expenses)]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints

class VehicleAcquisitionCost(Model):
	def setup(self,aircraft,mission):
		
		t_mission = mission.t_mission
		purchase_price = aircraft.purchase_price
		vehicle_life = aircraft.vehicle_life
		
		cost_per_time = Variable("cost_per_time","hr**-1",
			"Amortized vehicle purchase price per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-",
			"Amortized vehicle acquisition cost per mission")

		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission
		
		constraints = []

		constraints += [cost_per_time == purchase_price/vehicle_life]
		constraints += [cost_per_mission == t_mission*cost_per_time]
		
		return constraints

class AvionicsAcquisitionCost(Model):
	def setup(self,aircraft,mission):
		
		t_mission = mission.t_mission
		purchase_price = aircraft.avionics.purchase_price
		vehicle_life = aircraft.vehicle_life
		
		cost_per_time = Variable("cost_per_time","hr**-1",
			"Amortized avionics purchase price per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-",
			"Amortized avionics acquisition cost per mission")

		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		constraints = []

		constraints += [cost_per_time == purchase_price/vehicle_life]
		constraints += [cost_per_mission == t_mission*cost_per_time]
		
		return constraints

class BatteryAcquisitionCost(Model):
	def setup(self,battery,mission):
		
		t_mission = mission.t_mission
		purchase_price = battery.purchase_price
		cycle_life = battery.cycle_life
		
		cost_per_time = Variable("cost_per_time","hr**-1",
			"Amortized battery purchase price per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-",
			"Amortized battery cost per mission")

		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		constraints = []

		constraints += [cost_per_mission == purchase_price/cycle_life]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints

class CapitalExpenses(Model):
	def setup(self,aircraft,mission):

		t_mission = mission.t_mission

		cost_per_time = Variable("cost_per_time","hr**-1","Capital expenses per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Capital expenses per mission")

		vehicle_cost = VehicleAcquisitionCost(aircraft,mission)
		avionics_cost = AvionicsAcquisitionCost(aircraft,mission)
		battery_cost = BatteryAcquisitionCost(aircraft.battery,mission)

		self.costs = [vehicle_cost, avionics_cost, battery_cost]

		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		self.vehicle_cost = vehicle_cost
		self.avionics_cost = avionics_cost
		self.battery_cost = battery_cost

		constraints = []
		constraints += [self.costs]
		
		constraints += [cost_per_mission >= sum(c.cost_per_mission for c in self.costs)]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints


class PilotCost(Model):
	def setup(self,mission):

		t_mission = mission.t_mission

		wrap_rate = Variable("wrap_rate","hr**-1",
			"Cost per pilot, per unit mission time (including benefits and overhead)")
		cost_per_time = Variable("cost_per_time","hr**-1","Pilot cost per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Pilot cost per mission")

		self.wrap_rate = wrap_rate
		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		constraints = []
		
		if mission.mission_type == "autonomous":
			aircraft_per_bunker_pilot = Variable("aircraft_per_bunker_pilot",8,"-",
				"Number of aircraft controlled by 1 bunker pilot (assuming no crew on board)")
			constraints += [cost_per_time == wrap_rate/aircraft_per_bunker_pilot]

		if mission.mission_type == "piloted":
			pilots_per_aircraft = Variable("pilots_per_aircram",1.5,"-",
				"Pilots per aircraft (assuming crew on board)")
			constraints += [cost_per_time == wrap_rate*pilots_per_aircraft]

		constraints += [cost_per_mission == t_mission*cost_per_time]
		
		return constraints

class MaintenanceCost(Model):
	def setup(self,mission):

		t_mission = mission.t_mission

		MMH_FH = Variable("MMH_FH","-","Maintenance man-hours per flight hour")
		wrap_rate = Variable("wrap_rate","hr**-1",
			"Cost per mechanic, per unit maintenance time (including benefits and overhead)")

		cost_per_time = Variable("cost_per_time","hr**-1","Maintenance cost per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Maintenance cost per mission")

		self.MMH_FH = MMH_FH
		self.wrap_rate = wrap_rate
		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		constraints = []

		constraints += [cost_per_time == MMH_FH*wrap_rate]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints

class EnergyCost(Model):
	def setup(self,mission):

		t_mission = mission.t_mission
		E_charger = mission.time_on_ground.E_charger

		cost_per_energy = Variable("cost_per_energy",0.12,"kWh**-1","Price of electricity")
		cost_per_time = Variable("cost_per_time","hr**-1","Energy cost per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Energy cost per mission")

		self.cost_per_energy = cost_per_energy
		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		constraints = []

		constraints += [cost_per_mission == E_charger*cost_per_energy]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints

class IndirectOperatingCost(Model):
	def setup(self,operating_expenses):

		IOC_fraction = Variable("IOC_fraction",0.12,"-","IOC as a fraction of DOC")
		cost_per_time = Variable("cost_per_time","hr**-1","IOC per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","IOC per mission")

		self.IOC_fraction = IOC_fraction
		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		constraints = []

		constraints += [cost_per_mission == IOC_fraction*operating_expenses.DOC]
		constraints += [cost_per_time == IOC_fraction*operating_expenses.DOC_per_time]

		return constraints

class OperatingExpenses(Model):
	def setup(self,aircraft,mission):

		t_mission = mission.t_mission

		cost_per_time = Variable("cost_per_time","hr**-1","Operating expenses per unit mission time")
		cost_per_mission = Variable("cost_per_mission","-","Operating expenses per mission")

		DOC = Variable("DOC","-","Direct operating cost per mission")
		DOC_per_time = Variable("DOC_per_time","hr**-1","Direct operating cost per unit mission time")
		IOC = Variable("IOC","-","Indirect operating cost per mission")
		IOC_per_time = Variable("IOC_per_time","hr**-1","Indirect operating cost per unit mission time")

		self.DOC = DOC
		self.DOC_per_time = DOC_per_time
		self.IOC = IOC
		self.IOC_per_time = IOC_per_time

		self.cost_per_time = cost_per_time
		self.cost_per_mission = cost_per_mission

		pilot_cost = PilotCost(mission)
		maintenance_cost = MaintenanceCost(mission)
		energy_cost = EnergyCost(mission)
		indirect_operating_cost = IndirectOperatingCost(self)

		self.pilot_cost = pilot_cost
		self.maintenance_cost = maintenance_cost
		self.energy_cost = energy_cost
		self.indirect_operating_cost = indirect_operating_cost

		constraints = []
		constraints += [pilot_cost, maintenance_cost, energy_cost, indirect_operating_cost]

		constraints += [DOC >= pilot_cost.cost_per_mission 
			+ maintenance_cost.cost_per_mission + energy_cost.cost_per_mission]
		constraints += [DOC_per_time == DOC/t_mission]

		constraints += [IOC == indirect_operating_cost.cost_per_mission]
		constraints += [IOC_per_time == indirect_operating_cost.cost_per_time]

		constraints += [cost_per_mission >= DOC + IOC]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints




if __name__=="__main__":

	#Concept representative analysis

	# from noise_models import rotational_noise, vortex_noise, noise_weighting
	
	#String inputs
	reserve_type="FAA_heli"
	sizing_mission_type="piloted"

	problem_subDict = {}
	
	Aircraft = OnDemandAircraft()
	problem_subDict.update({
		Aircraft.L_D_cruise: 14., #estimated L/D in cruise
		Aircraft.eta_cruise: 0.85, #propulsive efficiency in cruise
		Aircraft.tailRotor_power_fraction_hover: 0.1, #  TODO: fix.
		Aircraft.tailRotor_power_fraction_levelFlight: 0.1, #  TODO: fix.
		Aircraft.cost_per_weight: 350*ureg.lbf**-1, #vehicle cost per unit empty weight
		Aircraft.battery.cost_per_C: 400*ureg.kWh**-1, #battery cost per unit energy capacity
		Aircraft.rotors.N: 12, #number of propellers
		Aircraft.rotors.Cl_mean_max: 1.0, #maximum allowed mean lift coefficient
		Aircraft.battery.C_m: 400*ureg.Wh/ureg.kg, #battery energy density
		Aircraft.structure.weight_fraction: 0.55, #empty weight fraction
		Aircraft.electricalSystem.eta: 0.9, #electrical system efficiency	
	})

	SizingMission = OnDemandSizingMission(Aircraft,mission_type=sizing_mission_type,
		reserve_type=reserve_type)
	problem_subDict.update({
		SizingMission.mission_range: 87*ureg.nautical_mile,#mission range
		SizingMission.V_cruise: 200*ureg.mph,#cruising speed
		SizingMission.t_hover: 120*ureg.s,#hover time
		SizingMission.T_A: 15.*ureg("N")/ureg("m")**2,#disk loading
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
		DeadheadMission.passengers.N_passengers: 0.1,#Number of passengers TODO: fix.
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

	for i in range(20):
		solution = problem.solve(verbosity=0)

		cpsk = solution("cost_per_seat_mile").to(ureg.km**-1).magnitude

		print "Cost per seat km: $%0.2f" % cpsk


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
	
