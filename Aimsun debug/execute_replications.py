all_scenarios = []
num_demand_level = 2
num_cav_penetration = 11
num_repli = 2
experiment_list = []
if target != None:
	scenario = model.getCatalog().find(target.getId())
	all_scenarios.append(scenario)

	max_per_scenario = 3
	all_replications = []
	cnt = 0
	# collect replications into corresponding dictionaries
	for scenario in all_scenarios:
		for exp in scenario.getExperiments():
			#experiment_list.append(exp)
			for repli in exp.getReplications():
				if not repli.isA("GKExperimentResult") and cnt < max_per_scenario:
					all_replications.append(repli)
					cnt+=1

	print("Total replications to run: {}".format(len(all_replications)))
	for repli in all_replications:
		GKSystem.getSystem().executeAction( "execute", repli, [], "")
	#GKSystem.getSystem().executeAction( "execute", average, all_replications, "" )
	#GKSystem.getSystem().executeAction( "retrieve", all_replications[0], all_replications, "" )