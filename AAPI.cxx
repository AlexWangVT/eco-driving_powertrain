
#include "AKIProxie.h"
#include "CIProxie.h"
#include "ANGConProxie.h"
#include "AAPI.h"
#include <stdio.h>
// Procedures could be modified by the user

int AAPILoad()
{
	AKIPrintString("LOAD");
	return 0;
}

int AAPIInit()
{	
	AKIPrintString("\tInit");
	ANGConnEnableVehiclesInBatch(true);
	return 0;
}

int AAPISimulationReady()
{	
	AKIPrintString("\tAAPISimulationReady");
	return 0;
}

int AAPIManage(double time, double timeSta, double timTrans, double acicle)
{
	AKIPrintString("\tManage");
	return 0;
}

int AAPIPostManage(double time, double timeSta, double timTrans, double acicle)
{
	AKIPrintString("\tPostManage");
	return 0;
}

int AAPIFinish()
{
	AKIPrintString("\tFinish");
	return 0;
}

int AAPIUnLoad()
{
	AKIPrintString("UNLOAD");
	return 0;
}

int AAPIPreRouteChoiceCalculation(double time, double timeSta)
{
	AKIPrintString("\tPreRouteChoice Calculation");
	return 0;
}

int AAPIEnterVehicle(int idveh, int idsection)
{
	return 0;
}

int AAPIExitVehicle(int idveh, int idsection)
{
	return 0;
}

int AAPIEnterVehicleSection(int idveh, int idsection, double atime)
{
	return 0;
}

int AAPIExitVehicleSection(int idveh, int idsection, double time)
{
	return 0;
}

int AAPIEnterPedestrian(int idPedestrian, int originCentroid)
{
	AKIPrintString("A Legion Pedestrian has entered the network");
	return 0;
}

int AAPIExitPedestrian(int idPedestrian, int destinationCentroid)
{
	AKIPrintString("A Legion Pedestrian has exited the network");
	return 0;
}

int AAPIVehicleStartParking(int idveh, int idsection, double time)
{
	return 0;
}

