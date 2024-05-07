
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

double simtime = 0; //current simulation time, in second

unordered_map<int, int> optimal_lane_set; //secid-laneid, unused
unordered_map<int, double> link_flw; // unused
unordered_map<int, int> link_list; // link ID? unused?
unordered_map<int, int> from_turn; // unused
unordered_map<int, int> to_turn; // unused
unordered_map<int, double> turn_pert; // unused

// set variables for energy consumptions
unordered_map<int, double> ice_base; // unused
unordered_map<int, double> bev_base; // unused
unordered_map<int, double> phev_base1; // unused
unordered_map<int, double> phev_base2; // unused
unordered_map<int, double> hfcv_base; // unused

unordered_map<int, int> eco_flg; // what is this used for? flag for eco-driving?

ifstream ecodrive;
ifstream sigopt;
int lnk_eco, lnk_ctl[100][4]; // lnk_ctl why [100][4]?
int nsig, sig_ctl[100][2], ph_lnk[200][8], sig_sec[100][4];
double sigtim[4], sigmin[4];
unordered_map<int, int> sig_lnk; // identify the links associated with the signal optimizaiton system
unordered_map<int, int> sig_flg;

int AAPILoad()
{
	srand((uint32_t)time(NULL));
	return 0;
}

void Emission(double spd, double grade, double acc, double& E_i, double& E_e, double& E_p1, double& E_p2, double& E_f)
{
	double m = 1928, g = 9.8066, cr = 1.75, c1 = 0.0328, c2 = 4.575, rhoa = 1.2256, etad = 0.80;
	double Af = 2.73, cd = 0.34, beta = 0.93, alpha = 0.2, ch = 0.95;
	double va = 32, Pa = 2.5, Pb = 5, P_aux = 0.7, P_idle = 1.0, P_hev=10, FC_hev;

	double a0 = 0.000341, a1 = 0.0000583, a2 = 0.000001;
	double eng[6]; // ICE, PEV, HPEV1, HPEV2, HFEV, Conventioal HEV (second-by-second energy consumption), eng[0] is ICEV fuel rate, eng[1] is BEV energy consumption, and so forth
	double rst, P_i, P_w, P_b, P_f, P_h;
	double P_batt, P_fuel;
	spd = spd / 3.6;

	// VT-CPFM model for gasoline vehicles  %%% some issue about the fuel consumption calculation, shall be around 1.5 L // what is the issue here?
	rst = rhoa * cd * ch * Af * pow(spd, 2) / 25.92 + g * m * cr * (c1 * spd + c2) / 1000 + g * m * grade;
	P_i = ((rst + 1.04 * m * acc) / (3600 * etad)) * spd;
	eng[0] = a0;
	if (P_i > 0) eng[0] = a0 + a1 * P_i + a2 * pow(P_i, 2);

	// CPEM model for EVs
	double gs, gc;
	double eta_dl = 0.92, eta_em = 0.91, eta_rb = 0;
	gc = sqrt(1 / (1 + pow(grade, 2)));
	gs = sqrt(1 - pow(gc, 2));
	P_w = (m * acc + m * g * gc * cr * (c1 * spd * 3.6 + c2) / 1000 + rhoa * Af * cd * pow(spd, 2) / 2 + m * g * gs) * spd;
	eng[1] = P_w / (eta_dl * eta_em);
	if (eng[1] < 0) {
		if (acc < 0) eta_rb = 1 / exp(0.0411 / abs(acc));
		eng[1] = eng[1] * eta_rb;
	}
	eng[1] = eng[1] / 3600; // unit conversion 

	// another try for VT-CPFM model
	//eng[0] = a0;
	//if (P_w > 0) eng[0] = a0 + a1 * P_w / 1000 + a2 * pow(P_w / 1000, 2); // why another try?


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
	eng[4] = P_batt + P_fuel; // what are the units? conventional fuel converted to kW

	// Conventional hybrid vehicle
	eng[5] = 0;
	if ((P_i < 0) || (spd < va / 3.6 && P_i < P_hev)) {
		eng[5] = 0.025 * spd; // FC_ev = 2.5 L/100 km, convert it to ml/s. If necessary, change it to a contant value instead of the function of speed
	}
	else {
		if ((P_i > 0 && spd > va / 3.6) || (spd < va / 3.6 && P_i >= P_hev)) {
			eng[5] = 0.06 + 0.003998 * spd * 3.6 + 0.077092 * P_i - 0.00009155 * pow(P_i, 2);
		}
	}

	E_i = eng[0];
	E_e = eng[1];
	E_p1 = eng[2];
	E_p2 = eng[3];
	E_f = eng[4];
	//AKIPrintString(("Energy Temp: " + to_string(P_w) + ", Speed = " + to_string(spd) + ", EV = " + to_string(E_e) + ", HFCV = " + to_string(E_f)).c_str());

	//AKIPrintString(("Energy 00000000: " + to_string(grade) + ", ICE = " + to_string(E_e) + ", PHEV = " + to_string(E_p1) + ", PHEV 2 = " + to_string(E_p2) + ", HFCV = " + to_string(E_f)).c_str());
}

void SigPhLnk(double ctim) {
	int sigid, nph;
	int ngrp, nturn;
	int grp_id, grp_trn, turn_org, turn_dst;
	int lnk_order = 0;
	int sec_fm, sec_to;
	for (int i = 0; i < nsig; i++) {
		sigid = sig_ctl[i][0];
		nph = sig_ctl[i][1];			//temporarily, set the group be the same to the phase
		nturn = AKIInfNetGetNbTurnsInNode(sigid);
		for (int j = 0; j < nph; j++) {		// for all nph phases/signal group
			grp_trn = ECIGetNumberTurningsofSignalGroup(sigid, j + 1);
			for (int k = 0; k < grp_trn; k++) {
				ECIGetFromToofTurningofSignalGroup(nsig, j + 1, k, &sec_fm, &sec_to);
				for (int m = 0; m < nturn; m++) {
					A2KTurnInf turn_inf = AKIInfNetGetTurnInfo(sigid, j);
					turn_org = turn_inf.originSectionId;
					turn_dst = turn_inf.destinationSectionId;
					if (turn_org == sec_fm && turn_dst == sec_to) {
						if (sig_lnk[turn_org] == 0) {
							sig_lnk[turn_org] = lnk_order;
							for (int k = turn_inf.originFromLane; k <= turn_inf.originToLane; k++) {
								lnk_ctl[lnk_order][k] = j;
							}
							lnk_order++;
						}
					}
				}
			}
		}
	}
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
		eco_flg[lnk_ctl[lnk_eco][0]] = 1; // a flag that indicates if the section is a control section
		lnk_eco++;
		AKIPrintString(("Eco-driving Input: " + to_string(lnk_ctl[lnk_eco][0]) + ", Speed = " + to_string(lnk_ctl[lnk_eco][0])).c_str());
	}
	ecodrive.close();

	sigopt.open("C:\\Users\\yhhar\\Desktop\\AIMSUN\\Eco-Driving\\sigopt.txt");
	sigopt >> nsig;						// read number of signal controlled with optimization
	for (int i = 0; i < nsig; i++) {
		sigopt >> sig_ctl[i][0]>> sig_ctl[i][1];  // intersection node id, number of phases
		sigopt >> sig_sec[i][0] >> sig_sec[i][1] >> sig_sec[i][2] >> sig_sec[i][3]; // links entering to the intersection (, up to 4, if no, then enter 0)
		sig_flg[sig_ctl[i][0]] = i;
		AKIPrintString(("Signal optimization input: " + to_string(sig_ctl[i][0]) + ", number of phases = " + to_string(sig_ctl[i][1])).c_str());
	}
	sigopt.close();

	// update the link and phase information
	SigPhLnk(0);

	return 0;
}

int EcoDriveFunc(double d2t, double d2a, double ttg, double spd_cur, double spd_limit, double& spd_opt, double& acc_opt1, double& acc_opt2) {
	double acc, acc2, spd_cruise = 0;
	double ta, dd, a0, v0;
	double eng_ice = 10000000, eng_bev, eng_phev1, eng_phev2, eng_hydr;
	int flag1, flag2;
	double eng_opt, eng_tot = eng_ice, eng_tot2 = eng_tot;
	double ta2, ta3, dd2;

	flag2 = 0;
	for (int i = 0; i<int(spd_limit); i++) { // how do you determine when to control? ttg?
		flag1 = 0;
		if (i < spd_cur) {
			dd = d2a - i * ttg / 3.6;  // deceleration distance, why i * ttg??
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
			
			for (int k = 0; k < 30; k++) {
				
				acc2 = k * 0.1;									// acceleration rate
				ta2 = (spd_limit - spd_cruise) / 3.6 / acc2;	// acceleration time to speed limit
				dd2 = spd_cruise * ta + 0.5 * acc2 * pow(ta2, 2);
				if (dd2 < d2a) {
					ta3 = (d2a - dd2) / (spd_limit / 3.6);
					eng_tot2 = eng_tot;
					for (int t = 0; t<int(ta2 + ta3); t++) {
						if (t < ta2) {
							v0 = spd_cruise + t * acc2 * 3.6;
							a0 = acc2;
						}
						else {
							v0 = spd_limit;
							a0 = 0;
						}
						Emission(v0, 0.0, a0, eng_ice, eng_bev, eng_phev1, eng_phev2, eng_hydr);
						eng_tot2 += eng_ice;
					}

					// search for the optimal speed and acceleration
					if (flag2 == 0) {
						eng_opt = eng_tot2;
						spd_opt = spd_cruise;
						acc_opt1 = acc;
						acc_opt2 = acc2;
						flag2 = 1;
					}
					else {
						if (eng_tot2 < eng_opt) {
							eng_opt = eng_tot2;
							spd_opt = spd_cruise;	// in km/hr
							acc_opt1 = acc;			// in m/s^2
							acc_opt2 = acc2;		// in m/s^2
							flag2 = 1;
						}
					}
				}
				
			}
		}
	}
	//AKIPrintString(("Control: " + to_string(spd_cur) + ", Speed = " + to_string(spd_opt) + ", Acc = " + to_string(acc_opt) + ", distance = " + to_string(dist) + ", TTG = " + to_string(ttg)).c_str());

	return flag2;
}


double SigDelay(int sigid, int Step, double dt) {	// estimate the total delay of each possible timing plan in the next Step*dt seconds
	int link_id, vlan, ph;
	int n = sig_flg[sigid];
	double tg, tr, flw_in, flw_out;
	int nph = ECIGetNumberPhases(sigid);
	double flw_lan, del_tot = 0;
	double ctim = AKIGetCurrentSimulationTime();

	for (int i = 0; i < 4; i++) {
		link_id = sig_sec[n][i];
		A2KSectionInf secinf = AKIInfNetGetSectionANGInf(link_id);
		if (link_id > 0) {

			// get the existing delay of all vehicles
			int nbveh = AKIVehStateGetNbVehiclesSection(link_id, true);
			for (int k = 0; k < nbveh; k++) {
				InfVeh vehinf = AKIVehStateGetVehicleInfSection(link_id, k);
				vlan = vehinf.numberLane;
				ph = lnk_ctl[link_id][vlan];
				del_tot += (ctim - vehinf.SectionEntranceT) - (secinf.length - vehinf.distance2End) / (secinf.speedLimit / 3.6);		// update the delay of each vehicle, summarized in each lane
			}

			// find the perspective delay of all phases
			for (int j = 0; j < secinf.nbCentralLanes; j++) {
				StructAkiEstadSectionLane secstat = AKIEstGetCurrentStatisticsSectionLane(link_id, j, 0);
				flw_lan = secstat.Flow;
				ph = lnk_ctl[link_id][j];
				if (ph == 0) {
					tg = 0; 
					tr = sigtim[ph];
				}
				else {
					tg = sigtim[ph - 1];
					tr = sigtim[ph];
				}
				flw_in = 0; flw_out = 0;
				for (int t = 0; t<int(Step * dt); t++) {
					flw_in += flw_lan / 3600;		//cumulative in-flow rate in veh
					if (t > tg && t < tr) {
						flw_out += secinf.capacity / secinf.nbCentralLanes / 3600;
						if (flw_out >= flw_in) flw_out = flw_in;
					}
					del_tot += flw_in - flw_out;	// find the total delay of the phase ph
				}
			}
		}
	}
	return del_tot;
}

void SigOptFunc(double timeSta, int Step, double dt) {	// search for the optimal timing plan
	int sigid, nph;
	double tdel, topt, ph_dur;
	int ngrp, nturn, flag;
	for (int i = 0; i < nsig; i++) {
		sigid = sig_ctl[i][0];
		nph = sig_ctl[i][1];

		flag = 0;
		if (nph == 2) {
			for (int p1 = 0; p1 <= Step; p1++) {
				sigtim[0] = p1 * dt;
				sigtim[1] = Step * dt;
				sigtim[2] = 0;
				sigtim[3] = 0;

				// update the optimal timing plan
				tdel = SigDelay(sigid, Step, dt);
				if (flag == 0) {
					topt = tdel;
					for (int i = 0; i < 4; i++) {
						sigmin[i] = sigtim[i];
					}
				}
				else {
					if (tdel < topt) {
						topt = tdel;
						for (int i = 0; i < 4; i++) {
							sigmin[i] = sigtim[i];
						}
					}
				}
			}
			
		}
		else {
			if (nph == 3) {
				for (int p1 = 0; p1 <= Step; p1++) {
					sigtim[0] = p1 * dt;
					for (int p2 = p1; p2 <= Step; p2++) {
						sigtim[1] = p2 * dt;
						sigtim[2] = Step * dt;
						sigtim[3] = 0;

						// update the optimal timing plan
						tdel = SigDelay(sigid, Step, dt);
						if (flag == 0) {
							topt = tdel;
							for (int i = 0; i < 4; i++) {
								sigmin[i] = sigtim[i];
							}
						}
						else {
							if (tdel < topt) {
								topt = tdel;
								for (int i = 0; i < 4; i++) {
									sigmin[i] = sigtim[i];
								}
							}
						}
					}
				}
			}
			else {
				for (int p1 = 0; p1 <= Step; p1++) {
					sigtim[0] = p1 * dt;
					for (int p2 = p1; p2 <= Step; p2++) {
						sigtim[1] = p2 * dt;
						for (int p3 = p2; p3 <= Step; p3++) {
							sigtim[2] = p3 * dt;
							sigtim[3] = Step * dt;

							// update the optimal timing plan
							tdel = SigDelay(sigid, Step, dt);
							if (flag == 0) {
								topt = tdel;
								for (int i = 0; i < 4; i++) {
									sigmin[i] = sigtim[i];
								}
							}
							else {
								if (tdel < topt) {
									topt = tdel;
									for (int i = 0; i < 4; i++) {
										sigmin[i] = sigtim[i];
									}
								}
							}
						}
					}
				}
			}
		}

		for (int p = 0; p < nph; p++) {
			if (p == 0){
				ph_dur = sigmin[p];
			}
			else {
				ph_dur = sigmin[p] - sigmin[p - 1];
			}
			int Report = ECIChangeTimingPhase(sigid, p+1, ph_dur, timeSta);		// set the new timing plan
		}
	}
}

int AAPIManage(double time, double timeSta, double timTrans, double cycle) // 
{
	int offset, sig_cycle, ph_cur, ph_veh = 2, sig_id = 5287; // ph_veh=2 means only control phase 2? but why the ecodrive file has two phases? may have error without initialization
	int cav_typ = 5380, vn_lan[6]; // vn_lan represents the total number of vehicles in each lane of the controlled section
	double ttg, ttg0, ttr, ph_start, tstep = 0.8; // why tstep = 0.8 not 1.0?
	double max, min, dur;
	double spd_opt = 100, acc_opt1 = 0, acc_opt2 = 3.0, spd_wave, spd_pre[7];
	double vlen = 5.0, d2t, d2a; // what is vlen, d2t: distance to the end of the queue, d2a: distance to accelerate to the end of the acceleration zone
	double simtime = AKIGetCurrentSimulationTime();

	int Step = 12;
	double dt = 5.0;
	// update the optimal timing plan at every Step*dt seconds
	if (int(simtime*10) % int(Step * dt * 10) == 0) {
		SigOptFunc(timeSta, Step, dt);
	}

	//update for eco-driving
	int secnb = AKIInfNetNbSectionsANG();
	for (int i = 0; i < secnb; i++) {
		int secid = AKIInfNetGetSectionANGId(i);
		A2KSectionInf secinf = AKIInfNetGetSectionANGInf(secid);
		spd_wave = secinf.capacity / (200 - secinf.capacity / secinf.speedLimit);		// calculate the rarefaction wave speed
		StructAkiEstadSection sec_stat = AKIEstGetCurrentStatisticsSection(secid, 0);
		if (eco_flg[secid] == 1) {
			for (int l = 0; l < 6; l++) {
				vn_lan[l] = 0;		// initialize the number of vehicles in each lane as 0
									// the number of lanes at each control section is 3, there are two control sections
			}
			int nbveh = AKIVehStateGetNbVehiclesSection(secid, true);
			for (int k = 0; k < nbveh; k++) {
				InfVeh vehinf = AKIVehStateGetVehicleInfSection(secid, k);
				if (vn_lan[vehinf.numberLane - 1] == 0) spd_pre[vehinf.numberLane - 1] = secinf.speedLimit;
				vn_lan[vehinf.numberLane - 1]++; // indicate which lane the vehicle is located and accumulate the number of vehicles in that lane
				int type_id = AKIVehTypeGetIdVehTypeANG(vehinf.type);

				if (type_id == cav_typ) {		// eco-driving only applied to CAVs
					int sec_to = AKIVehInfPathGetNextSection(vehinf.idVeh, secid);
					for (int j = 0; j < lnk_eco; j++) {		// identify the phase of the vehicle
						if (secid == lnk_ctl[j][0] && sec_to == lnk_ctl[j][1]) {
							ph_veh = lnk_ctl[j][2];  // the phase that would be controlled
							sig_id = lnk_ctl[j][3];  // node ID
							break;
						}
					}
					// find the current status and the time to green or time to red
					offset = ECIGetOffset(sig_id); // not used?
					sig_cycle = ECIGetControlCycleofJunction(0, sig_id);
					double ph_dur[8]; // there are a total of 4 phases, why use 8? ph_dur refers to phase duration
					for (int p = 0; p < ECIGetNbPhasesofJunction(0, sig_id); p++) {
						int flg = ECIGetDurationsPhase(sig_id, p + 1, simtime, &dur, &max, &min); // p is phase ID
						ph_dur[p] = dur;
					}
					ph_cur = ECIGetCurrentPhase(sig_id);	// current phase, return current phase ID
					ph_start = ECIGetStartingTimePhase(sig_id);		// start time of the current phase
					if (ph_veh == ph_cur) {
						ttr = ph_dur[ph_veh - 1] - (simtime - ph_start); // time to red
						ttg = ttr + sig_cycle - ph_dur[ph_veh - 1]; // time to green refers to the time to next green from the current green
					}
					else {
						if (ph_veh > ph_cur) {
							ttg = 0;
							for (int ph = ph_cur; ph < ph_veh; ph++) {
								ttg += ph_dur[ph - 1];
							}
							ttg = ttg - (simtime - ph_start);
							ttr = ttg + ph_dur[ph_veh - 1];
						}
						else {
							ttg = 0;
							for (int ph = ph_cur; ph <= ECIGetNbPhasesofJunction(0, sig_id); ph++) {
								ttg += ph_dur[ph - 1];
							}
							ttg = ttg - (simtime - ph_start);
							for (int ph = 1; ph < ph_veh; ph++) {
								ttg += ph_dur[ph - 1];
							}
							ttr = ttg + ph_dur[ph_veh - 1];
						}
					}

					// apply eco-driving for the CAV
					// only apply it when the current phase is red
					// when the current phase is green but vehicle cannot get through the junction if not speeding?
					if (ph_veh != ph_cur) {
						AKIVehSetAsTracked(vehinf.idVeh);
						// calculate queue tail
						// quene estimation is missing

						// control the speed of CVs
						spd_opt = vehinf.CurrentSpeed;
						d2t = vehinf.distance2End - (vn_lan[vehinf.numberLane - 1] - 1) * vlen;		// what does this mean? why (vn_lan[vehinf.numberLane - 1] * vlen)? ((vn_lan[vehinf.numberLane - 1]-1) might get negative values.
						ttg0 = ttg + (vn_lan[vehinf.numberLane - 1] - 1) * vlen / (spd_wave / 3.6);		// time until the queue is released
						d2a = (vn_lan[vehinf.numberLane - 1] * vlen) + 100;
						if (d2t > 0) {
							int flag = EcoDriveFunc(d2t, d2a, ttg0, vehinf.CurrentSpeed, secinf.speedLimit, spd_opt, acc_opt1, acc_opt2);
							if (flag > 0) {
								//if (spd_opt < vehinf.CurrentSpeed) AKIVehTrackedForceSpeed(vehinf.idVeh, vehinf.CurrentSpeed - acc_opt * tstep * 3.6);
								if (spd_opt < vehinf.CurrentSpeed - acc_opt1 * tstep * 3.6) spd_opt = vehinf.CurrentSpeed - acc_opt1 * tstep * 3.6; // why >= 
								AKIVehTrackedModifySpeed(vehinf.idVeh, spd_opt);
							}
							if (secid == 1249 && ph_veh == 2) AKIPrintString(("Control Indicator: " + to_string(vehinf.CurrentSpeed) + ", phase = " + to_string(flag) + ", Speed = " + to_string(spd_opt) + ", Acc = " + to_string(acc_opt1) + ", distance = " + to_string(vehinf.distance2End) + ", TTG = " + to_string(ttg)).c_str());
						}
					}
					else { // deal with the situation with queue at green light
						if (spd_pre[vehinf.numberLane - 1] < 1.0) { // the previous vehicle is in the queue
							ttg0 = (vn_lan[vehinf.numberLane - 1] - 1) * vlen / (spd_wave / 3.6);
							ttg0 = ttg0 - (ph_dur[ph_cur - 1] - ttr);
							d2t = vehinf.distance2End - (vn_lan[vehinf.numberLane - 1] - 1) * vlen;
							d2a = (vn_lan[vehinf.numberLane - 1] * vlen) + 100;
							if (d2t > 0) {
								int flag = EcoDriveFunc(d2t, d2a, ttg0, vehinf.CurrentSpeed, secinf.speedLimit, spd_opt, acc_opt1, acc_opt2);
								if (flag > 0) {
									//if (spd_opt < vehinf.CurrentSpeed) AKIVehTrackedForceSpeed(vehinf.idVeh, vehinf.CurrentSpeed - acc_opt * tstep * 3.6);
									if (spd_opt < vehinf.CurrentSpeed - acc_opt1 * tstep * 3.6) spd_opt = vehinf.CurrentSpeed - acc_opt1 * tstep * 3.6; // why >= 
									AKIVehTrackedModifySpeed(vehinf.idVeh, spd_opt);
								}
							}
						}
					}
				}

				spd_pre[vehinf.numberLane - 1] = vehinf.CurrentSpeed;
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