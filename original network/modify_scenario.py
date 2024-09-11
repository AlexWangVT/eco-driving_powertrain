print("##### This is the start of script_Modify_Experiment_Parameters #####")

random_seed_list = [40404, 40410] # the number of random seeds equals number of replications
#vehicle_types = ["ICE", "BEV", "PHEV", "HFCV"]
vehicle_types = ["ICEV"]
# powertrain_distribution={
	# "2023": [94, 3, 2, 1],
	#"2030": [75, 12, 12, 1],
	#"2040": [37, 45, 17, 1],
	#"Average": [25, 25, 25, 25],
	#"ICE": [100, 0, 0, 0]}
powertrain_ratios = [100]
demand = [10, 20]
CAV_penetration_list = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]

# Duplicate demand and CAV_penetration_list to make their index consistent with expriment ID, so that we can easily set name for experiments in line 35
length = len(demand)
demand = demand * len(CAV_penetration_list)
demand.sort()
CAV_penetration_list = CAV_penetration_list * length

if target == None:
	scenario = model.getCatalog().find(5373) # 5373 refers to dynamic scenario No_control_demand_50_CAV_50
else:
	scenario = model.getCatalog().find(target.getId())


# get attributes in the experiments
attribute_demand = model.getColumn("GKExperiment::demand_percentage")
attribute_cav_penetration = model.getColumn("GKExperiment::cav_penetration")

for idx, exp in enumerate(scenario.getExperiments()):
	demand_percentage = demand[idx]
	CAV_penetration = CAV_penetration_list[idx]
	exp.setName("Experiment_D{}_CAV{}".format(demand_percentage, CAV_penetration))
	
	# change ratio variable for each vehicle type in the experiments
	cav_ratio = CAV_penetration / 100.0
	noncav_ratio = (100 - CAV_penetration) / 100.0
	for idx in range(len(vehicle_types)):
		exp.setValueForVariable("$"+vehicle_types[idx]+"_CAV_percentage", str(cav_ratio * powertrain_ratios[idx]))
		exp.setValueForVariable("$"+vehicle_types[idx]+"_NONCAV_percentage", str(noncav_ratio * powertrain_ratios[idx]))
	# set attributes in the experiments for correct logging in the API
	exp.setDataValueDouble(attribute_demand, demand_percentage)
	exp.setDataValueDouble(attribute_cav_penetration, CAV_penetration)
	
	for re_idx, repli in enumerate(exp.getReplications()):
		repli.setName("Replication_D{}_CAV{}_{}".format(demand_percentage, CAV_penetration, re_idx))
		repli.setRandomSeed(random_seed_list[re_idx % len(random_seed_list)])


print("##### End of script_Modify_CAV_percentage_all_scenario #####")

model.getCommander().addCommand( None )