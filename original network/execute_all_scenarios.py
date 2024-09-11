all_scenarios = []
num_demand_level = 2
num_cav_penetration = 11
num_repli = 2

if target != None:
	scenario = model.getCatalog().find(target.getId())
	all_scenarios.append(scenario)

	max_per_scenario = num_demand_level * num_cav_penetration * num_repli
	all_replications = []

	# collect replications into corresponding dictionaries
	for scenario in all_scenarios:
		cnt = 0
		for exp in scenario.getExperiments():
			for repli in exp.getReplications():
				if not repli.isA("GKExperimentResult") and cnt < max_per_scenario:
					all_replications.append(repli)
					cnt+=1

	print("Total replications to run: {}".format(len(all_replications)))
	#GKSystem.getSystem().executeAction( "execute", all_replications[0], all_replications, "" )
	#GKSystem.getSystem().executeAction( "execute", average, all_replications, "" )
	#GKSystem.getSystem().executeAction( "retrieve", all_replications[0], all_replications, "" )