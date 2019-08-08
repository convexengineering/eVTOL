# Contains configuration-specific data
from gpkit import ureg
from collections import OrderedDict
from copy import deepcopy

generic_data = {} #generic data (i.e. for all configs)


generic_data["delta_S"] = 500 * ureg.ft

generic_data["sizing_mission_type"]   = "piloted"
generic_data["revenue_mission_type"]  = "piloted"
generic_data["deadhead_mission_type"] = "autonomous"