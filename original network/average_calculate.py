# This script calculates the average of several simulated replications and then
# load the average in memory

if target != None:
	
	scenario = model.getCatalog().find(target.getId())
	system = GKSystem.getSystem()

	#Create new GKExperimentResult object (Average object)
	average = system.newObject( "GKExperimentResult",model)
	for exp in scenario.getExperiments():
		for replication in exp.getReplications():
			average.addReplication( replication )
		exp.addReplication( average )

		#Calculate and store in the database the average of all the simulated
		#replications in a given experiment result.
		if average != None and average.isA( "GKExperimentResult" ):
			system.executeAction("execute", average, [], "")

		# Be sure that you reset the UNDO buffer after a modification that cannot be undone
		model.getCommander().addCommand( None )
		print("Total replications to run: {}".format(len(exp.getReplications())))

else:
	model.reportError("Average Calculator", "The script must be launched from a Dynamic Experiment context menu")