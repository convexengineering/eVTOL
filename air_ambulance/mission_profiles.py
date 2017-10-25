# Air-ambulance mission profiles

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/..'))

import numpy as np
from gpkit import Variable, Model, Vectorize, ureg
from matplotlib import pyplot as plt
from aircraft_models import OnDemandAircraft, OnDemandMissionCost
from aircraft_models import Crew, Passengers, FlightState 
from aircraft_models import Hover, LevelFlight, TimeOnGround
from aircraft_models import CapitalExpenses, OperatingExpenses
from study_input_data import generic_data, configuration_data
from noise_models import vortex_noise

class MedicalEquipment(Model):
	def setup(self):
		
		W = Variable("W",150,"lbf","Medical equipment weight")
		
		return []

class AirAmbulanceSizingMission(Model):
	#Mission the aircraft must be able to fly. No economic analysis.
    def setup(self,aircraft,V_cruise=150*ureg.mph,N_crew=3,N_passengers=1,reserve_type="FAA_heli",
		mission_type="piloted",loiter_type="level_flight",tailRotor_power_fraction_hover=0.0001,
		tailRotor_power_fraction_levelFlight=0.0001):
		
		if not(aircraft.autonomousEnabled) and (mission_type != "piloted"):
			raise ValueError("Autonomy is not enabled for Aircraft() model.")
		
		W = Variable("W_{mission}","lbf","Weight of the aircraft during the mission")
		
		C_eff = aircraft.battery.topvar("C_{eff}") #effective battery capacity
		E_mission = Variable("E_{mission}","kWh","Electrical energy used during mission")

		self.W = W
		self.E_mission = E_mission
		self.mission_type = mission_type
		self.crew = Crew(mission_type=mission_type,N_crew=N_crew,W_oneCrew=225*ureg.lbf)
		self.passengers = Passengers(N_passengers=N_passengers,W_onePassenger=225*ureg.lbf)
		self.medical_equipment = MedicalEquipment()
		
		hoverState = FlightState(h=0*ureg.ft)
		
		constraints = []
		
		self.fs0 = Hover(self,aircraft,hoverState,
			tailRotor_power_fraction=tailRotor_power_fraction_hover)#takeoff from base
		self.fs1 = LevelFlight(self,aircraft,V=V_cruise,
			tailRotor_power_fraction=tailRotor_power_fraction_levelFlight)#fly to patient location
		self.fs2 = Hover(self,aircraft,hoverState,
			tailRotor_power_fraction=tailRotor_power_fraction_hover)#Hover/land at patient location
		self.fs3 = Hover(self,aircraft,hoverState,
			tailRotor_power_fraction=tailRotor_power_fraction_hover)#Takeoff from patient location
		self.fs4 = LevelFlight(self,aircraft,V=V_cruise,
			tailRotor_power_fraction=tailRotor_power_fraction_levelFlight)#Fly to hospital
		self.fs5 = Hover(self,aircraft,hoverState,
			tailRotor_power_fraction=tailRotor_power_fraction_hover)#Hover/land at hospital
		self.fs6 = Hover(self,aircraft,hoverState,
			tailRotor_power_fraction=tailRotor_power_fraction_hover)#Takeoff from hospital
		self.fs7 = LevelFlight(self,aircraft,V=V_cruise,
			tailRotor_power_fraction=tailRotor_power_fraction_levelFlight)#Fly to base
		
		#Reserve segment
		if reserve_type == "FAA_aircraft" or reserve_type == "FAA_heli":
			V_reserve = ((1/3.)**(1/4.))*V_cruise #Approximation for max-endurance speed
			
			if reserve_type == "FAA_aircraft":
				#30-minute loiter time, as per VFR rules for aircraft (daytime only)
				t_loiter = Variable("t_{loiter}",30,"minutes","Loiter time")
			elif reserve_type == "FAA_heli":
				#20-minute loiter time, as per VFR rules for helicopters
				t_loiter = Variable("t_{loiter}",20,"minutes","Loiter time")

			if loiter_type == "level_flight":#loiter segment is a level-flight segment
				self.fs8 = LevelFlight(self,aircraft,V=V_reserve,segment_type="loiter",
					tailRotor_power_fraction=tailRotor_power_fraction_levelFlight)
			elif loiter_type == "hover":#loiter segment is a hover segment
				self.fs8 = Hover(self,aircraft,hoverState,
					tailRotor_power_fraction=tailRotor_power_fraction_hover)

			constraints += [t_loiter == self.fs8.topvar("t")]

		if reserve_type == "Uber":#2-nautical-mile diversion distance; used by McDonald & German
			V_reserve = V_cruise
			R_divert = Variable("R_{divert}",2,"nautical_mile","Diversion distance")
			self.fs8 = LevelFlight(self,aircraft,V=V_reserve,segment_type="cruise",
				tailRotor_power_fraction=tailRotor_power_fraction_levelFlight)#reserve segment
			constraints += [R_divert == self.fs8.topvar("segment_range")]
		
		self.fs9 = Hover(self,aircraft,hoverState,
			tailRotor_power_fraction=tailRotor_power_fraction_hover)#Land at base
		self.time_on_ground = TimeOnGround(self)#Charging

		self.flight_segments = [self.fs0, self.fs1, self.fs2, self.fs3, self.fs4,\
			self.fs5, self.fs6, self.fs7, self.fs8, self.fs9]#all segments included
		self.cruise_segments = [self.fs1, self.fs4, self.fs7]#loiter not included
		self.hover_segments  = [self.fs0, self.fs2, self.fs3, self.fs5, self.fs6,\
			self.fs9]
		
		#Power and energy consumption by mission segment
		with Vectorize(len(self.flight_segments)):
			P_battery = Variable("P_{battery}","kW","Segment power draw")
			E = Variable("E","kWh","Segment energy use")

		#Segment range constraints
		with Vectorize(len(self.cruise_segments)):
			segment_range = Variable("segment_range","nautical_mile","Segment range")
		
		#Data from hover segments
		with Vectorize(len(self.hover_segments)):
			t_hover = Variable("t_{hover}","s","Segment hover time")
			CT = Variable("CT","-","Thrust coefficient")
			CP = Variable("CP","-","Power coefficient")
			Q_perRotor = Variable("Q_perRotor","lbf*ft","Torque per lifting rotor")
			T_perRotor = Variable("T_perRotor","lbf","Thrust per lifting rotor")
			P = Variable("P","kW","Total power supplied to all lifting rotors")
			P_perRotor = Variable("P_perRotor","kW","Power per lifting rotor")
			VT = Variable("VT","ft/s","Propeller tip speed")
			omega = Variable("\omega","rpm","Propeller angular velocity")
			MT = Variable("MT","-","Propeller tip Mach number")
			FOM = Variable("FOM","-","Figure of merit")
			p_ratio = Variable("p_{ratio}","-","Sound pressure ratio in hover")

		constraints += [self.flight_segments, self.time_on_ground]
		constraints += [self.crew, self.passengers]
		constraints += [W >= aircraft.topvar("W_{empty}") + self.passengers.topvar("W") \
			+ self.crew.topvar("W") + self.medical_equipment.topvar("W")]
		constraints += [aircraft.topvar("MTOW") >= W]
		
		#Mission-segment constraints
		constraints += [segment_range[i] == segment.topvar("segment_range") for i,segment in enumerate(self.cruise_segments)]
		constraints += [t_hover[i] == segment.topvar("t") for i,segment in enumerate(self.hover_segments)]

		constraints += [hoverState]
		
		constraints += [E_mission >= sum(c.topvar("E") for c in self.flight_segments)]
		constraints += [C_eff >= E_mission]

		constraints += [P_battery[i] == segment.topvar("P_{battery}") for i,segment in enumerate(self.flight_segments)]
		constraints += [E[i] == segment.topvar("E") for i,segment in enumerate(self.flight_segments)]

		constraints += [CT[i] == segment.rotorPerf.topvar("CT") for i,segment in enumerate(self.hover_segments)]
		constraints += [CP[i] == segment.rotorPerf.topvar("CP") for i,segment in enumerate(self.hover_segments)]
		constraints += [Q_perRotor[i] == segment.rotorPerf.topvar("Q_perRotor") for i,segment in enumerate(self.hover_segments)]
		constraints += [T_perRotor[i] == segment.rotorPerf.topvar("T_perRotor") for i,segment in enumerate(self.hover_segments)]
		constraints += [P[i] == segment.rotorPerf.topvar("P") for i,segment in enumerate(self.hover_segments)]
		constraints += [P_perRotor[i] == segment.rotorPerf.topvar("P_perRotor") for i,segment in enumerate(self.hover_segments)]
		constraints += [VT[i] == segment.rotorPerf.topvar("VT") for i,segment in enumerate(self.hover_segments)]
		constraints += [omega[i] == segment.rotorPerf.topvar("\omega") for i,segment in enumerate(self.hover_segments)]
		constraints += [MT[i] == segment.rotorPerf.topvar("MT") for i,segment in enumerate(self.hover_segments)]
		constraints += [FOM[i] == segment.rotorPerf.topvar("FOM") for i,segment in enumerate(self.hover_segments)]
		constraints += [p_ratio[i] == segment.rotorPerf.topvar("p_{ratio}") for i,segment in enumerate(self.hover_segments)]

		#Mission time 
		t_flight = Variable("t_{flight}","s","Time in flight")
		t_mission = Variable("t_{mission}","s","Mission time (including charging)")
		
		constraints += [t_flight >= sum(c.topvar("t") for c in self.flight_segments)]
		constraints += [t_mission >= t_flight + self.time_on_ground.topvar("t")]
		
		return constraints

class AirAmbulanceMissionCost(Model):
	#Cost for one mission. Exactly the same code as DeadheadMissionCost.
	def setup(self,aircraft,mission,pilot_wrap_rate=70*ureg.hr**-1,
		mechanic_wrap_rate=60*ureg.hr**-1,MMH_FH=0.6):
		
		t_mission = mission.topvar("t_{mission}")
		
		cost_per_mission = Variable("cost_per_mission","-","Cost per mission")
		cost_per_time = Variable("cost_per_time","hr**-1","Cost per unit mission time")

		capital_expenses = CapitalExpenses(aircraft,mission)
		operating_expenses = OperatingExpenses(aircraft,mission,
			pilot_wrap_rate=pilot_wrap_rate,mechanic_wrap_rate=mechanic_wrap_rate,
			MMH_FH=MMH_FH)
		expenses = [capital_expenses, operating_expenses]

		constraints = []
		
		constraints += [expenses]
		constraints += [cost_per_mission >= sum(c.topvar("cost_per_mission") for c in expenses)]
		constraints += [cost_per_mission == t_mission*cost_per_time]

		return constraints
	
if __name__=="__main__":
	
	eta_cruise = generic_data["\eta_{cruise}"] 
	eta_electric = generic_data["\eta_{electric}"]
	C_m = generic_data["C_m"]
	n = generic_data["n"]
	B = generic_data["B"]
	
	reserve_type = generic_data["reserve_type"]
	autonomousEnabled = generic_data["autonomousEnabled"]
	charger_power = generic_data["charger_power"]
	
	vehicle_cost_per_weight = generic_data["vehicle_cost_per_weight"]
	battery_cost_per_C = generic_data["battery_cost_per_C"]
	pilot_wrap_rate = generic_data["pilot_wrap_rate"]
	mechanic_wrap_rate = generic_data["mechanic_wrap_rate"]
	MMH_FH = generic_data["MMH_FH"]
	deadhead_ratio = generic_data["deadhead_ratio"]
	
	delta_S = generic_data["delta_S"]
	
	config = "Tilt rotor"
	print "Solving configuration: " + config
	configs = configuration_data.copy()
	c = configs[config]
	
	V_cruise = c["V_{cruise}"]
	L_D_cruise = c["L/D"]
	T_A = c["T/A"]
	Cl_mean_max = c["Cl_{mean_{max}}"]
	N = c["N"]
	loiter_type = c["loiter_type"]
	tailRotor_power_fraction_hover = c["tailRotor_power_fraction_hover"]
	tailRotor_power_fraction_levelFlight = c["tailRotor_power_fraction_levelFlight"]
	weight_fraction = c["weight_fraction"]
	
	Aircraft = OnDemandAircraft(N=N,L_D_cruise=L_D_cruise,eta_cruise=eta_cruise,C_m=C_m,
		Cl_mean_max=Cl_mean_max,weight_fraction=weight_fraction,n=n,eta_electric=eta_electric,
		cost_per_weight=vehicle_cost_per_weight,cost_per_C=battery_cost_per_C,
		autonomousEnabled=autonomousEnabled)
	
	SizingMission = AirAmbulanceSizingMission(Aircraft,V_cruise=V_cruise,N_crew=3,
		N_passengers=1,reserve_type=reserve_type,loiter_type=loiter_type,
		tailRotor_power_fraction_hover=tailRotor_power_fraction_hover,
		tailRotor_power_fraction_levelFlight=tailRotor_power_fraction_levelFlight)
	SizingMission.substitutions.update({SizingMission.fs0.topvar("T/A"):T_A})
	
	segment_range_array = [40,60,40]*ureg.nautical_mile
	t_hover_array = [30,120,30,30,30,30]*ureg.second
	
	for i,segment_range in enumerate(segment_range_array):
		SizingMission.substitutions.update({SizingMission.topvar("segment_range")[i]:segment_range})
	for i,t_hover in enumerate(t_hover_array):
		SizingMission.substitutions.update({SizingMission.topvar("t_{hover}")[i]:t_hover})
	
	MissionCost = AirAmbulanceMissionCost(Aircraft,SizingMission)
	
	problem = Model(MissionCost.topvar("cost_per_mission"),[Aircraft, SizingMission, MissionCost])
	solution = problem.solve(verbosity=0)
	
	
	
	
	
