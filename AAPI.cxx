  
#include "AKIProxie.h"
#include "CIProxie.h"
#include "ANGConProxie.h"
#include "AAPI.h"
#include <stdio.h>
#include <unordered_map>
#include <map>
#include <string>
#include <deque>
#include <queue>
#include <time.h>
#include <set>
#include <random>
#include <cmath>
#include <fstream>
// Procedures could be modified by the user

using namespace std;

double simtime=0; //current simulation time, in second
unordered_map<int, int> optimal_lane_set; //secid-laneid
unordered_map<int, double> link_flw;
unordered_map<int, int> link_list;
unordered_map<int, int> from_turn;
unordered_map<int, int> to_turn;
unordered_map<int, double> turn_pert;

// set variables for energy consumptions
unordered_map<int, double> ice_base;
unordered_map<int, double> bev_base;
unordered_map<int, double> phev_base1;
unordered_map<int, double> phev_base2;
unordered_map<int, double> hfcv_base;

unordered_map<int, int> eco_flg;

ifstream ecodrive;
int lnk_eco, lnk_ctl[100][4];


int AAPILoad()
{
	srand((uint32_t)time(NULL));
	return 0;
}

void Emission(double spd, double grade, double acc, double& E_i, double& E_e, double& E_p1, double& E_p2, double& E_f)
{
	double m = 1928, g = 9.8066, cr = 1.75, c1 = 0.0328, c2 = 4.575, rhoa = 1.2256, etad = 0.80;
	double Af = 2.73, cd = 0.34, beta = 0.93, alpha = 0.2, ch = 0.95;
	double va = 32, Pa = 2.5, Pb = 5, P_aux = 0.7, P_idle = 1.0;
	double a0 = 0.000341, a1 = 0.0000583, a2 = 0.000001;
	double eng[5]; // ICE, PEV, HPEV1, HPEV2, HFEV
	double rst, P_i, P_w, P_b, P_f, P_h;
	double P_batt, P_fuel;
	spd = spd / 3.6;

	// VT-CPFM model for gasoline vehicles  %%% some issue about the fuel consumption calculation, shall be around 1.5 L
	rst = rhoa * cd * ch * Af * pow(spd, 2) / 25.92 + g * m * cr * (c1 * spd + c2) / 1000 + g * m * grade;
	P_i = ((rst + 1.04 * m * acc) / (3600 * etad)) * spd;
	eng[0] = a0;
	if (P_i > 0) eng[0] = a0 + a1 * P_i + a2 * pow(P_i, 2);

	// CPEM model for EVs
	double gs, gc;
	double eta_dl = 0.92, eta_em = 0.91, eta_rb = 0;
	gc = sqrt(1 / (1 + pow(grade, 2)));
	gs = sqrt(1 - pow(gc, 2));
	P_w = (m * acc + m * g * gc * cr * (c1 * spd*3.6 + c2) / 1000 + rhoa * Af * cd * pow(spd, 2) / 2 + m * g * gs) * spd;
	eng[1] = P_w / (eta_dl * eta_em);
	if (eng[1] < 0) {
		if (acc < 0) eta_rb = 1 / exp(0.0411 / abs(acc));
		eng[1] = eng[1] * eta_rb;
	}
	eng[1] = eng[1] / 3600;

	// another try for VT-CPFM model
	eng[0] = a0;
	if (P_w > 0) eng[0] = a0 + a1 * P_w / 1000 + a2 * pow(P_w / 1000, 2);


	// PHEV energy model
	double P_max = 100 * 0.3;			//max engine motor power
	if (eng[1] > P_max) {
		P_h = (eng[1] - P_max);
		eng[2] = P_max;								// electric usage for PHEV
		eng[3] = a0 + a1 * P_h + a2 * pow(P_h, 2);  // fuel usage for PHEV
	}
	else {
		eng[2] = eng[1];
		eng[3] = 0;
	}

	// HFCV energy model
	if (P_w < 0) P_w = 0;
	P_w = P_w / 3600;
	if (P_w <= Pa) {
		P_batt = P_w + P_aux;
		P_fuel = P_idle;
	}
	else {
		if (P_w > Pa && P_w <= Pb) {
			if (spd * 3.6 <= va) {
				P_batt = P_w + P_aux;
				P_fuel = P_idle;
			}
			else {
				P_batt = P_idle + P_aux;
				P_fuel = P_w * beta;
			}
		}
		else {
			P_batt = P_w * alpha + P_aux;
			P_fuel = P_w * beta;
		}
	}
	eng[4] = P_batt + P_fuel;

	E_i = eng[0];
	E_e = eng[1];
	E_p1 = eng[2];
	E_p2 = eng[3];
	E_f = eng[4];
	//AKIPrintString(("Energy Temp: " + to_string(P_w) + ", Speed = " + to_string(spd) + ", EV = " + to_string(E_e) + ", HFCV = " + to_string(E_f)).c_str());

	//AKIPrintString(("Energy 00000000: " + to_string(grade) + ", ICE = " + to_string(E_e) + ", PHEV = " + to_string(E_p1) + ", PHEV 2 = " + to_string(E_p2) + ", HFCV = " + to_string(E_f)).c_str());
}

int AAPIInit()
{	

	ANGConnEnableVehiclesInBatch(true);

	lnk_eco = 0;
	int tmp;
	ecodrive.open("C:\\Users\\yhhar\\Desktop\\AIMSUN\\Eco-Driving\\ecodrive.txt");
	for (int i = 0; i < 4; i++) {
		ecodrive >> lnk_ctl[lnk_eco][0];
		ecodrive >> lnk_ctl[lnk_eco][1];
		ecodrive >> lnk_ctl[lnk_eco][2];
		ecodrive >> lnk_ctl[lnk_eco][3];
		eco_flg[lnk_ctl[lnk_eco][0]] = 1;
		lnk_eco++;
		AKIPrintString(("Control Input: " + to_string(lnk_ctl[lnk_eco][0]) + ", Speed = " + to_string(lnk_ctl[lnk_eco][0])).c_str());
	}
	/*
	while (ecodrive >> tmp[0] >> tmp[1] >> tmp[2] >> tmp[3]) {
		lnk_ctl[lnk_eco][0] = tmp[0];
		lnk_ctl[lnk_eco][1] = tmp[1];
		lnk_ctl[lnk_eco][2] = tmp[2];
		eco_flg[tmp[0]] = 1;
		lnk_eco++;
		AKIPrintString(("Control Indicator: " + to_string(eco_flg[tmp[0]]) + ", Speed = " + to_string(tmp[0])).c_str());
	}
	*/
	
	ecodrive.close();

	return 0;
}

int EcoDriveFunc(double dist, double ttg, double spd_cur, double spd_limit, double& spd_opt, double& acc_opt) {
	double acc, spd_cruise = 0;
	double ta, dd, a0, v0;
	double eng_ice = 10000000, eng_bev, eng_phev1, eng_phev2, eng_hydr;
	int flag1, flag2;
	double eng_opt, eng_tot = eng_ice;

	flag2 = 0;
	for (int i = 0; i<int(spd_limit); i++) {
		flag1 = 0;
		if (i < spd_cur) {
			dd = dist - i * ttg / 3.6;
			if (dd > 0) {
				ta = dd / ((spd_cur - i) / 3.6 / 2);	// deceleration time (s)
				acc = (spd_cur - i) / ta / 3.6;			// deceleration rate m/s^2
				spd_cruise = i;
				flag1 = 1;
			}
		}
		if (flag1 == 1) {
			// update speed profile and enery consumption
			eng_tot = 0;
			for (int t = 0; t<int(ttg); t++) {
				if (t < ta) {
					v0 = spd_cur - acc * t * 3.6;	// in km/hr
					a0 = acc;
				}
				else {
					v0 = spd_cruise;				// in km/hr
					a0 = 0.0;
				}
				Emission(v0, 0.0, a0, eng_ice, eng_bev, eng_phev1, eng_phev2, eng_hydr);
				eng_tot += eng_ice;
			}
			// search for the optimal speed and acceleration
			if (flag2 == 0) {
				eng_opt = eng_tot;
				spd_opt = spd_cruise;			// in km/hr
				acc_opt = acc;			// in m/s^2
				flag2 = 1;
			}
			else {
				if (eng_ice < eng_tot) {
					eng_opt = eng_tot;
					spd_opt = spd_cruise;
					acc_opt = acc;
				}
			}
		}
	}
	//AKIPrintString(("Control: " + to_string(spd_cur) + ", Speed = " + to_string(spd_opt) + ", Acc = " + to_string(acc_opt) + ", distance = " + to_string(dist) + ", TTG = " + to_string(ttg)).c_str());

	if (flag2 == 1) {
		return 0;
	}
	else {
		return -1;			// error, no optimal speed
	}
}

int AAPIManage(double time, double timeSta, double timTrans, double acicle)
{
	int offset, cycle, ph_cur, ph_veh = 2, sig_id = 5287;
	int cav_typ = 5380, vn_lan[6];
	double ttg, ttr, ph_start, tstep = 0.8;
	double max, min, dur;
	double spd_opt = 100, acc_opt = 0;
	double vlen = 5.0, d2t;
	double simtime = AKIGetCurrentSimulationTime();
	int secnb = AKIInfNetNbSectionsANG();
	for (int i = 0; i < secnb; i++) {
		int secid = AKIInfNetGetSectionANGId(i);
		A2KSectionInf secinf = AKIInfNetGetSectionANGInf(secid);
		StructAkiEstadSection sec_stat = AKIEstGetCurrentStatisticsSection(secid, 0);
		if (eco_flg[secid] == 1) {
			for (int l = 0; l < 6; l++) {
				vn_lan[l] = 0;		// initialize the number of vehicles in each lane as 0
			}
			int nbveh = AKIVehStateGetNbVehiclesSection(secid, true);
			for (int k = 0; k < nbveh; k++){
				InfVeh vehinf = AKIVehStateGetVehicleInfSection(secid, k);
				vn_lan[vehinf.numberLane - 1]++;
				int type_id = AKIVehTypeGetIdVehTypeANG(vehinf.type);

				if (type_id == cav_typ) {		// only applied for CAVs
					int sec_to = AKIVehInfPathGetNextSection(vehinf.idVeh, secid);
					for (int j = 0; j < lnk_eco; j++) {		// identify the phase of the vehicle
						if (secid == lnk_ctl[j][0] && sec_to == lnk_ctl[j][1]) {
							ph_veh = lnk_ctl[j][2];
							sig_id = lnk_ctl[j][3];
							break;
						}
					}
					// find the current status and the time to green or time to red
					offset = ECIGetOffset(sig_id);
					cycle = ECIGetControlCycleofJunction(0, sig_id);
					double ph_dur[8];
					for (int p = 0; p < ECIGetNbPhasesofJunction(0, sig_id); p++) {
						int flg = ECIGetDurationsPhase(sig_id, p+1, simtime, &dur, &max, &min);
						ph_dur[p] = dur;
					}
					ph_cur = ECIGetCurrentPhase(sig_id);	// current phase
					ph_start = ECIGetStartingTimePhase(sig_id);		// start time of the current phase
					if (ph_veh == ph_cur) {
						ttr = ph_dur[ph_veh-1] - (simtime - ph_start);
						ttg = ttr + cycle - ph_dur[ph_veh - 1];
					}
					else {
						if (ph_veh > ph_cur) {
							ttg = 0;
							for (int ph = ph_cur; ph < ph_veh; ph++) {
								ttg += ph_dur[ph-1];
							}
							ttg = ttg - (simtime - ph_start);
							ttr = ttg + ph_dur[ph_veh-1];
						}
						else {
							ttg = 0;
							for (int ph = ph_cur; ph <= ECIGetNbPhasesofJunction(0, sig_id); ph++) {
								ttg += ph_dur[ph-1];
							}
							ttg = ttg - (simtime - ph_start);
							for (int ph = 1; ph < ph_veh; ph++) {
								ttg += ph_dur[ph-1];
							}
							ttr = ttg + ph_dur[ph_veh - 1];
						}
					}

					// apply eco-driving for the CAV
					// only apply it when the current phase is red

					if (ph_veh != ph_cur) {
						AKIVehSetAsTracked(vehinf.idVeh);
						// calculate queue tail

						// control the speed of 
						spd_opt = vehinf.CurrentSpeed;
						d2t = vehinf.distance2End - (vn_lan[vehinf.numberLane - 1] * vlen);
						if (d2t > 0) {
							int flag = EcoDriveFunc(d2t, ttg, vehinf.CurrentSpeed, secinf.speedLimit, spd_opt, acc_opt);
							if (flag >= 0) {
								//if (spd_opt < vehinf.CurrentSpeed) AKIVehTrackedForceSpeed(vehinf.idVeh, vehinf.CurrentSpeed - acc_opt * tstep * 3.6);
								if (spd_opt < vehinf.CurrentSpeed - acc_opt * tstep * 3.6) spd_opt = vehinf.CurrentSpeed - acc_opt * tstep * 3.6;
								AKIVehTrackedModifySpeed(vehinf.idVeh, spd_opt);
							}
							if (secid == 1249 && ph_veh == 2) AKIPrintString(("Control Indicator: " + to_string(vehinf.CurrentSpeed) + ", phase = " + to_string(flag) + ", Speed = " + to_string(spd_opt) + ", Acc = " + to_string(acc_opt) + ", distance = " + to_string(vehinf.distance2End) + ", TTG = " + to_string(ttg)).c_str());
						}
					}
				}
			}
		}
	}

	return 0;
}

int AAPIPostManage(double time, double timeSta, double timTrans, double acicle)
{
	
	return 0;
}

int AAPIFinish()
{
	//AKIPrintString("\tFinish");

	return 0;
}

int AAPIUnLoad()
{
	//AKIPrintString("UNLOAD");
	return 0;
}

int AAPIPreRouteChoiceCalculation(double time, double timeSta)
{
	//AKIPrintString("\tPreRouteChoice Calculation");
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