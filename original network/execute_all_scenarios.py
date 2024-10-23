all_scenarios = []

if target != None:
	scenario = model.getCatalog().find(target.getId())
	all_scenarios.append(scenario)

	max_per_scenario = 66
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