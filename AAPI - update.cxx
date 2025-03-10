
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
#include <iostream>
#include <sstream>
#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
// Procedures could be modified by the user

using namespace std;

double simtime = 0; //current simulation time, in second

//unordered_map<int, int> optimal_lane_set; //secid-laneid, unused
//unordered_map<int, double> link_flw; // unused
//unordered_map<int, int> link_list; // link ID? unused?
//unordered_map<int, int> from_turn; // unused
//unordered_map<int, int> to_turn; // unused
//unordered_map<int, double> turn_pert; // unused
//
//// set variables for energy consumptions
//unordered_map<int, double> ice_base; // unused
//unordered_map<int, double> bev_base; // unused
//unordered_map<int, double> phev_base1; // unused
//unordered_map<int, double> phev_base2; // unused
//unordered_map<int, double> hfcv_base; // unused

unordered_map<int, int> eco_flg; // <section_id, 0/1> indicate if a section is a control section

ifstream ecodrive;
ifstream sigopt;
ofstream fecotraj_output;
int lnk_eco, lnk_ctl[100][4]; // lnk_ctl why [100][4]? because the maximum number of lanes in the network is 3
int nsig, sig_ctl[100][2], ph_lnk[200][8], sig_sec[100][4], lan_ctl[100][4];
double sigtim[4], sigmin[4];
double sigph_dur[100][4], sigph_time[100][5], timeLastChange[100];
unordered_map<int, int> sig_lnk; // identify the links associated with the signal optimizaiton system
unordered_map<int, int> sig_flg;
string control_strategy[4] = { "No_control", "Signal_optimization_only", "Eco-Driving_only", "Proposed_control" };

// scenario experiment logging definition
int scenario_id = ANGConnGetScenarioId();
int experiment_id = ANGConnGetExperimentId();
int repli_id = ANGConnGetReplicationId();
void* control_strategy_pointer = ANGConnGetAttribute(AKIConvertFromAsciiString("GKScenario::driveControlStrategy"));
void* experiment_demand_pointer = ANGConnGetAttribute(AKIConvertFromAsciiString("GKExperiment::demand_percentage"));
void* experiment_CAV_MPR_pointer = ANGConnGetAttribute(AKIConvertFromAsciiString("GKExperiment::cav_penetration"));
bool anyNonAsciiChar;
string control_method = string(AKIConvertToAsciiString(ANGConnGetAttributeValueString(control_strategy_pointer, scenario_id), false, &anyNonAsciiChar));
int demand_percentage = ANGConnGetAttributeValueInt(experiment_demand_pointer, experiment_id);
int cav_penetration = ANGConnGetAttributeValueInt(experiment_CAV_MPR_pointer, experiment_id);

int ctl_flg_coop = 0;  //updated on March 05, 2025

enum VehicleType {
	UNKNOWN = 0,
	ICEV_CAV = 5428,
	ICEV_NON_CAV = 5429,
	BEV_CAV = 5591,
	BEV_NON_CAV = 5590,
	HEV_CAV = 5596,
	HEV_NON_CAV = 5597,
	PHEV_CAV = 5592,
	PHEV_NON_CAV = 5593,
	HFCV_CAV = 5594,
	HFCV_NON_CAV = 5595
};

const VehicleType VEHICLE_TYPE_ICEV_CAV = VehicleType::ICEV_CAV;
const VehicleType VEHICLE_TYPE_ICEV_NON_CAV = VehicleType::ICEV_NON_CAV;
const VehicleType VEHICLE_TYPE_BEV_CAV = VehicleType::BEV_CAV;
const VehicleType VEHICLE_TYPE_BEV_NON_CAV = VehicleType::BEV_NON_CAV;
const VehicleType VEHICLE_TYPE_HEV_CAV = VehicleType::HEV_CAV;
const VehicleType VEHICLE_TYPE_HEV_NON_CAV = VehicleType::HEV_NON_CAV;
const VehicleType VEHICLE_TYPE_PHEV_CAV = VehicleType::PHEV_CAV;
const VehicleType VEHICLE_TYPE_PHEV_NON_CAV = VehicleType::PHEV_NON_CAV;
const VehicleType VEHICLE_TYPE_HFCV_CAV = VehicleType::HFCV_CAV;
const VehicleType VEHICLE_TYPE_HFCV_NON_CAV = VehicleType::HFCV_NON_CAV;


const vector<VehicleType>valid_vehicle_type_list{ VEHICLE_TYPE_ICEV_CAV, VEHICLE_TYPE_ICEV_NON_CAV, VEHICLE_TYPE_BEV_CAV, VEHICLE_TYPE_BEV_NON_CAV,
VEHICLE_TYPE_HEV_CAV, VEHICLE_TYPE_HEV_NON_CAV, VEHICLE_TYPE_PHEV_CAV, VEHICLE_TYPE_PHEV_NON_CAV, VEHICLE_TYPE_HFCV_CAV, VEHICLE_TYPE_HFCV_NON_CAV };

const vector<VehicleType>cav_vehicle_list{ VEHICLE_TYPE_ICEV_CAV, VEHICLE_TYPE_BEV_CAV, VEHICLE_TYPE_HEV_CAV, VEHICLE_TYPE_PHEV_CAV, VEHICLE_TYPE_HFCV_CAV };

map<VehicleType, string> typeToString = { { ICEV_CAV, "ICEV_CAV" },
									 { ICEV_NON_CAV, "ICEV_NON_CAV" },
									 { BEV_CAV, "BEV_CAV" },
									 { BEV_NON_CAV, "BEV_NON_CAV" },
									 { HEV_CAV, "HEV_CAV" },
									 { HEV_NON_CAV, "HEV_NON_CAV" },
									 { PHEV_CAV, "PHEV_CAV" },
									 { PHEV_NON_CAV, "PHEV_NON_CAV" },
									 { HFCV_CAV, "HFCV_CAV" },
									 { HFCV_NON_CAV, "HFCV_NON_CAV" }
};

string enumToString(VehicleType type)
{
	return typeToString[type];
}


void printDebugLog(string s) {
	AKIPrintString(("## Debug log ##: " + s).c_str());
}

int AAPILoad()
{
	srand((uint32_t)time(NULL));
	return 0;
}

void Emission(double spd, double grade, double acc, VehicleType vehicle_type, double& E_i, double& E_e, double& E_p1, double& E_p2, double& E_f, double& E_g)
{
	double m = 1928, g = 9.8066, cr = 1.75, c1 = 0.0328, c2 = 4.575, rhoa = 1.2256, etad = 0.80;
	double Af = 2.73, cd = 0.34, beta = 0.93, alpha = 0.2, ch = 0.95;
	double va = 32, Pa = 2.5, Pb = 5, P_aux = 0.7, P_idle = 1.0, P_hev = 10, FC_hev;
	double a0 = 0.000341, a1 = 0.0000583, a2 = 0.000001;
	double eng[6]; // ICE, PEV, HPEV1, HPEV2, HFEV, Conventioal HEV (second-by-second energy consumption), eng[0] is ICEV fuel rate, eng[1] is BEV energy consumption, and so forth
	double rst, P_i, P_w, P_b, P_f, P_h;
	double P_batt, P_fuel;
	double gs, gc;
	double eta_dl = 0.92, eta_em = 0.91, eta_rb = 0;
	double P_max = 100 * 0.3;			//max engine motor power
	spd = spd / 3.6;
	string type_test = enumToString(vehicle_type);

	switch (vehicle_type) {
	case VehicleType::ICEV_CAV:
	case VehicleType::ICEV_NON_CAV:
		// VT-CPFM model for gasoline vehicles  %%% some issue about the fuel consumption calculation, shall be around 1.5 L // what is the issue here?
		rst = rhoa * cd * ch * Af * pow(spd, 2) / 25.92 + g * m * cr * (c1 * spd + c2) / 1000 + g * m * grade;
		P_i = ((rst + 1.04 * m * acc) / (3600 * etad)) * spd;
		eng[0] = a0;
		if (P_i > 0) eng[0] = a0 + a1 * P_i + a2 * pow(P_i, 2);

		break;

	case VehicleType::BEV_CAV:
	case VehicleType::BEV_NON_CAV:
		// CPEM model for EVs
		gc = sqrt(1 / (1 + pow(grade, 2)));
		gs = sqrt(1 - pow(gc, 2));
		P_w = (m * acc + m * g * gc * cr * (c1 * spd * 3.6 + c2) / 1000 + rhoa * Af * cd * pow(spd, 2) / 2 + m * g * gs) * spd;
		eng[1] = P_w / (eta_dl * eta_em);
		if (eng[1] < 0) {
			if (acc < 0) eta_rb = 1 / exp(0.0411 / abs(acc));
			eng[1] = eng[1] * eta_rb;
		}
		eng[1] = eng[1] / 3600; // unit conversion 
		AKIPrintString(("vehicle type is: " + type_test).c_str());
		break;

	case VehicleType::PHEV_CAV:
	case VehicleType::PHEV_NON_CAV:
		// PHEV energy model
		gc = sqrt(1 / (1 + pow(grade, 2)));
		gs = sqrt(1 - pow(gc, 2));
		P_w = (m * acc + m * g * gc * cr * (c1 * spd * 3.6 + c2) / 1000 + rhoa * Af * cd * pow(spd, 2) / 2 + m * g * gs) * spd;
		eng[1] = P_w / (eta_dl * eta_em);
		if (eng[1] < 0) {
			if (acc < 0) eta_rb = 1 / exp(0.0411 / abs(acc));
			eng[1] = eng[1] * eta_rb;
		}
		eng[1] = eng[1] / 3600; // unit conversion 

		if (eng[1] > P_max) {
			P_h = (eng[1] - P_max);
			eng[2] = P_max;								// electric usage for PHEV
			eng[3] = a0 + a1 * P_h + a2 * pow(P_h, 2);  // fuel usage for PHEV
		}
		else {
			eng[2] = eng[1];
			eng[3] = 0;
		}
		AKIPrintString(("vehicle type is: " + type_test).c_str());
		break;

	case VehicleType::HFCV_CAV:
	case VehicleType::HFCV_NON_CAV:
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
		AKIPrintString(("vehicle type is: " + type_test).c_str());
		break;

	case VehicleType::HEV_CAV:
	case VehicleType::HEV_NON_CAV:
		// Conventional hybrid vehicle
		eng[5] = 0;
		if ((P_i < 0) || (spd < va / 3.6 && P_i < P_hev)) {
			eng[5] = 0.025 * spd; // FC_ev = 2.5 L/100 km, convert it to ml/s. If necessary, change it to a contant value instead of the function of speed
			eng[5] = 0;
		}
		else {
			if ((P_i > 0 && spd > va / 3.6) || (spd < va / 3.6 && P_i >= P_hev)) {
				eng[5] = 0.06 + 0.003998 * spd * 3.6 + 0.077092 * P_i - 0.00009155 * pow(P_i, 2);
			}
		}

		AKIPrintString(("vehicle type is: " + type_test).c_str());
		break;
	}

	E_i = eng[0];
	E_e = eng[1];
	E_p1 = eng[2];
	E_p2 = eng[3];
	E_f = eng[4];
	E_g = eng[5];
	//AKIPrintString(("Energy Temp: " + to_string(P_w) + ", Speed = " + to_string(spd) + ", EV = " + to_string(E_e) + ", HFCV = " + to_string(E_f)).c_str());
	/*AKIPrintString(("icev fuel is: " + to_string(eng[0]) + " bev is: " + to_string(eng[1]) + " phev_1 is: " + to_string(eng[2]) +
		" phev_2 is: " + to_string(eng[3]) + " hfcv is: " + to_string(eng[4]) + " hev is: " + to_string(eng[5])).c_str());*/
		//AKIPrintString(("Energy 00000000: " + to_string(grade) + ", ICE = " + to_string(E_e) + ", PHEV = " + to_string(E_p1) + ", PHEV 2 = " + to_string(E_p2) + ", HFCV = " + to_string(E_f)).c_str());
}

void SigPhLnk(double ctim) { // SigPhLnk function used to find which lane controlled by which signal phase
	int sigid, nph;
	int ngrp, nturn;
	int grp_id, grp_trn, n_trn_grp, turn_org, turn_dst, tmp_turn_org = 0;
	int lnk_order = 1;
	int sec_fm, sec_to;
	int n_grp, grp_id0;
	for (int i = 0; i < nsig; i++) {
		sigid = sig_ctl[i][0];
		nph = sig_ctl[i][1];			//temporarily, set the group be the same to the phase
		nturn = AKIInfNetGetNbTurnsInNode(sigid);
		//AKIPrintString(("signal ID is: " + to_string(sigid) + ", Phase #: " + to_string(nph) + ", N Turn: " + to_string(nturn)).c_str());
		
		for (int j = 1; j <= nph; j++) {		// for all nph phases/signal group
			n_grp = ECIGetNbSignalGroupsPhaseofJunction(sigid, j, ctim);		// get the number of groups in the phase j
			for (int k = 0; k < n_grp; k++) {
				grp_id0 = ECIGetSignalGroupPhaseofJunction(sigid, j, k, ctim);
				n_trn_grp = ECIGetNumberTurningsofSignalGroup(sigid, grp_id0);
				//if (sigid == 3014) AKIPrintString(("Number of Groups: " + to_string(n_grp) + ", Group ID: " + to_string(grp_id0) + ", N Turn: " + to_string(n_trn_grp)).c_str());
				
				for (int m = 0; m < n_trn_grp; m++) {    // search all turns in a signal group
					ECIGetFromToofTurningofSignalGroup(sigid, grp_id0, m, &sec_fm, &sec_to);
					//if (sigid == 5287) AKIPrintString(("Phase #: " + to_string(j) + ", Group #: " + to_string(grp_id0) + ", turn #: " + to_string(m) + " , From: " + to_string(sec_fm) + " , To: " + to_string(sec_to)).c_str());
					
					for (int m = 0; m < nturn; m++) {
						A2KTurnInf turn_inf = AKIInfNetGetTurnInfo(sigid, m);
						turn_org = turn_inf.originSectionId;
						turn_dst = turn_inf.destinationSectionId;

						if (turn_org == sec_fm && turn_dst == sec_to) { // match turn with signal phase
							/*
							if (turn_org == tmp_turn_org) {
								sig_lnk[turn_org] = sig_lnk[tmp_turn_org];
								for (int origin_lane_id = turn_inf.originFromLane; origin_lane_id <= turn_inf.originToLane; origin_lane_id++) { // iterate all lanes of that turn in the origin section
									lan_ctl[sig_lnk[turn_org]][origin_lane_id] = j; // lnk_order index origin section and k index lane
								}
							}
							else {
								sig_lnk[turn_org] = lnk_order;
								for (int origin_lane_id = turn_inf.originFromLane; origin_lane_id <= turn_inf.originToLane; origin_lane_id++) { // iterate all lanes of that turn in the origin section
									lan_ctl[sig_lnk[turn_org]][origin_lane_id] = j; // lnk_order index origin section and k index lane
								}
								lnk_order++;
							}
							*/

							if (sig_lnk[turn_org] == 0) {
								sig_lnk[turn_org] = lnk_order;
								lnk_order++;
							}
							for (int kk = turn_inf.originFromLane; kk <= turn_inf.originToLane; kk++) {
								lan_ctl[sig_lnk[turn_org]][kk] = j;
							}
							tmp_turn_org = turn_org;
							//if (sigid == 5287) AKIPrintString(("sig_lnk value is: " + to_string(sig_lnk[turn_org]) + ", and turn origin section is: " + to_string(turn_org) + ", lane: " + to_string(k) + ", phase: " + to_string(j)).c_str());
						}
					}
				}
			}
		}
	}

	//AKIPrintString(("Signal Phase Info: " + to_string(lan_ctl[sig_lnk[1246]][1]) + ",  " + to_string(lan_ctl[sig_lnk[1246]][2]) + ",  " + to_string(lan_ctl[sig_lnk[1269]][1]) + ",  " + to_string(lan_ctl[sig_lnk[1269]][2]) + ",  " + to_string(lan_ctl[sig_lnk[1269]][3]) + ",  " + to_string(lan_ctl[sig_lnk[747]][1])).c_str());

}

int AAPIInit()
{

	ANGConnEnableVehiclesInBatch(true);

	ecodrive.open("D:\\projects\\eco-driving_powertrainOpt\\ecodrive.txt");

	string line, word;
	int word_int, row = 0;

	while (getline(ecodrive, line)) {
		stringstream ss(line);
		int col = 0;
		while (getline(ss, word, '	')) {
			word_int = stoi(word);
			lnk_ctl[row][col] = word_int;
			col++;
		}
		eco_flg[lnk_ctl[row][0]] = 1;
		AKIPrintString(("origin_section is: " + to_string(lnk_ctl[row][0]) + ", destination_section is: " + to_string(lnk_ctl[row][1])).c_str());
		//AKIPrintString(("eco_flag is: " + to_string(eco_flg[lnk_ctl[row][0]])).c_str());
		row++;
	}
	lnk_eco = row;
	ecodrive.close();

	sigopt.open("D:\\projects\\eco-driving_powertrainOpt\\sigopt.txt");
	sigopt >> nsig;						// read number of signal controlled with optimization

	for (int i = 0; i < nsig; i++) {
		sigopt >> sig_ctl[i][0] >> sig_ctl[i][1];  // intersection node id, number of phases
		sigopt >> sig_sec[i][0] >> sig_sec[i][1] >> sig_sec[i][2] >> sig_sec[i][3]; // links entering to the intersection (, up to 4, if no, then enter 0)
		sig_flg[sig_ctl[i][0]] = i;
		timeLastChange[i] = -10;
		//AKIPrintString(("Signal optimization input: " + to_string(sig_ctl[i][0]) + ", number of phases = " + to_string(sig_ctl[i][1]) + ", origin section = " + to_string(sig_sec[i][0])).c_str());
	}
	sigopt.close();

	// update the link and phase information
	SigPhLnk(0);

	//// output lan_ctl and sig_lnk results
	//string debug_output_path = "results\\debug.txt";
	//ofstream lane_phase_match;
	//int size_lan_ctl = 100, row_lan_ctl = 4;
	//lane_phase_match.open(debug_output_path, fstream::out);
	//for (int i = 0; i < size_lan_ctl; i++) {
	//	for (int j = 0; j < row_lan_ctl; j++)
	//	{
	//		lane_phase_match << lan_ctl[i][j] << '\t';
	//	}
	//	lane_phase_match << std::endl;
	//}
	//lane_phase_match.close();


	//AKIPrintString(("sig_lnk size is: " +  to_string(sig_lnk.size())).c_str());

	// save output to specified path
	AKIPrintString((to_string(experiment_id)).c_str());

	string output_path;
	output_path = "D:\\projects\\eco-driving_powertrainOpt\\original network\\results\\" + control_method + "_" + "demand" + to_string(demand_percentage) + "_" + "CAV" +
		to_string(cav_penetration) + "_" + to_string(repli_id) + ".csv";
	fecotraj_output.open(output_path, fstream::out);
	fecotraj_output << "simulation_time" << "\t" << "section_id" << "\t" << "veh_type_id" << "\t" << "numberLane" << "\t"
		<< "vehicle_id" << "\t" << "distance2End" << "\t" << "currentPosition" << "\t" << "CurrentSpeed" << "\t" << "acceleration" << "\t" << "driving_distance" <<
		"\t" << "ICEV_fuel" << "\t" << "BEV_energy" << "\t" << "PHEV_electric_power" << "\t" << "PHEV_fuel" << "\t" << "HFCV_energy" << "\t" << "hev_fuel" << std::endl;

	return 0;
}

int AAPISimulationReady()
{
	AKIPrintString("\tAAPISimulationReady");
	return 0;
}

int EcoDriveFunc(VehicleType vehicle_type, double d2t, double d2a, double ttg, double spd_cur, double spd_limit, double& spd_opt, double& acc_opt1, double& acc_opt2) {
	double acc, acc2, spd_cruise = 0;
	double ta, dd, a0, v0;
	double eng_ice = 10000000, eng_bev, eng_phev1, eng_phev2, eng_hydr, eng_hev;
	int flag1, flag2;
	double eng_opt, eng_tot = eng_ice, eng_tot2 = eng_tot;
	double ta2, ta3, dd2;
	double grade = 0.0;

	flag2 = 0; // indicate if eco-drive is successfully applied to control the vehicle, 1 is controled, 0 is not, innitial value is 0
	for (int i = 0; i<int(spd_limit); i++) {
		flag1 = 0;
		if (i < spd_cur) {
			dd = d2t - i * ttg / 3.6;  // given d2t and ttg and find cruise speed that ensures there are both deceleration and cruise distances within ttg. 
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
				Emission(v0, grade, a0, vehicle_type, eng_ice, eng_bev, eng_phev1, eng_phev2, eng_hydr, eng_hev);
				eng_tot += eng_ice;
			}

			for (int k = 0; k < 30; k++) {
				acc2 = k * 0.1;									// acceleration rate
				ta2 = (spd_limit - spd_cruise) / 3.6 / acc2;	// acceleration time to speed limit
				dd2 = spd_cruise * ta2 + 0.5 * acc2 * pow(ta2, 2);
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
						Emission(v0, grade, a0, vehicle_type, eng_ice, eng_bev, eng_phev1, eng_phev2, eng_hydr, eng_hev);
						eng_tot2 += eng_ice;
					}

					// search for the optimal speed and acceleration
					if (flag2 == 0) { // initialize variables with the first solutions in response to i = 0
						eng_opt = eng_tot2;
						spd_opt = spd_cruise;
						acc_opt1 = acc;
						acc_opt2 = acc2;
						flag2 = 1;
					}
					else {
						if (eng_tot2 < eng_opt) { // when iterate from the second value of "i", we update trajectory and energy only when the new solution has lower energy consumption
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
	int link_id, vlan, ph, lnk_order, vlan0, lan_cnt[7];
	int n = sig_flg[sigid], jj;
	double tg, tr, flw_in, flw_out;
	int nph = ECIGetNumberPhases(sigid);
	double flw_lan, del_tot = 0, vol_ln[7], veh_cnt[7], del_tmp, del_tmp0;
	double loc_lan[7], spd_lan[7];
	double ctim = AKIGetCurrentSimulationTime();

	for (int i = 0; i < 4; i++) {
		link_id = sig_sec[n][i]; // entry section ID of each signal
		A2KSectionInf secinf = AKIInfNetGetSectionANGInf(link_id);
		if (link_id > 0) {

			// get the existing delay of all vehicles (VT uses existing delay to optimize signal plan)
			for (int k = 0; k < 7; k++) {
				vol_ln[k] = 0;
				veh_cnt[k] = 0;
				loc_lan[k] = 0;
				spd_lan[k] = secinf.speedLimit;
			}
			int nbveh = AKIVehStateGetNbVehiclesSection(link_id, true);
			for (int k = 0; k < nbveh; k++) {
				InfVeh vehinf = AKIVehStateGetVehicleInfSection(link_id, k);
				vlan = vehinf.numberLane;
				loc_lan[vlan - 1] = vehinf.distance2End;
				spd_lan[vlan - 1] = vehinf.CurrentSpeed;

				if (ctim - vehinf.SectionEntranceT < 15) vol_ln[vlan-1]++;
				veh_cnt[vlan - 1]++;
				// ph = lan_ctl[link_id][vlan];
				del_tmp = (ctim - vehinf.SectionEntranceT) - (secinf.length - vehinf.distance2End) / (secinf.speedLimit / 3.6);		// update the delay of each vehicle, summarized in each lane
				if (del_tmp > 0) del_tot += del_tmp;
			} // This "for" loop calculates total delay at the current simulation step (the actual travel time - free flow travel time)
			for (int j = 1; j <= secinf.nbCentralLanes + secinf.nbSideLanes; j++) { // update the inflow rate, set it to capacity is the queue exceed the lane
				if (vol_ln[j - 1] == 0 && secinf.length - loc_lan[j - 1] < 10 && spd_lan[j - 1] < 3.0) vol_ln[j - 1] = (secinf.capacity / secinf.nbCentralLanes) * 15 / 3600;
			}
			// find the perspective delay of all phases
			//if (sigid == 3014) AKIPrintString(("Preliminary Delay: " + to_string(del_tot) + ", link id: " + to_string(link_id) + ", Capacity: " + to_string(secinf.capacity / (secinf.nbCentralLanes + secinf.nbSideLanes))).c_str());
			//if (sigid == 3014) AKIPrintString(("Preliminary plan 1: " + to_string(sigtim[0]) + ", 2: " + to_string(sigtim[1]) + ", 3: " + to_string(sigtim[2])).c_str());

			for (int j = 1; j <= secinf.nbCentralLanes + secinf.nbSideLanes; j++) {
				//StructAkiEstadSectionLane secstat = AKIEstGetCurrentStatisticsSectionLane(link_id, j, 0);
				// flw_lan = secstat.Flow;
				flw_lan = vol_ln[j-1] * 3600 / 15;  // volume in vph aggregated in 15 seconds
				
				lnk_order = sig_lnk[link_id];
				ph = lan_ctl[lnk_order][j] - 1; // must know which lane controlled by which phase
				if (ph == 0) { // tg/tr is the start/end of a phase
					tg = 0;
					tr = sigtim[ph];
				}
				else {
					tg = sigtim[ph - 1];
					tr = sigtim[ph];
				}
				flw_in = veh_cnt[j-1]; flw_out = 0;
				for (int t = 0; t<int(Step * dt); t++) {
					flw_in += flw_lan / 3600;		//cumulative in-flow rate in veh
					if (t > tg && t < tr) {
						//flw_out += secinf.capacity / (secinf.nbCentralLanes + secinf.nbSideLanes) / 3600;
						flw_out += secinf.capacity / 3600;
						if (flw_out >= flw_in) flw_out = flw_in; // cumulative flow-out rate in veh
					}
					del_tot += flw_in - flw_out;	// total delay of each lane equals the area of in-flow rate minus out-flow rate
					//if (sigid == 3014) AKIPrintString(("Temp flow Information: " + to_string(t) + ", in flow: " + to_string(flw_in) + ", out flow: " + to_string(flw_out) + ", total Delay: " + to_string(del_tot)).c_str());
				}

				// remove the delay of the vehicles already in the queue and will be released in the green light
				del_tmp0 = 0; jj = 0;
				lan_cnt[j - 1] = ceil(flw_out);
				if (lan_cnt[j - 1] > veh_cnt[j - 1]) lan_cnt[j - 1] = veh_cnt[j - 1];
				for (int kk = 0; kk < nbveh; kk++) {
					InfVeh vehinf0 = AKIVehStateGetVehicleInfSection(link_id, kk);
					vlan0 = vehinf0.numberLane;
					if (vlan0 == j && jj < lan_cnt[j - 1]) {
						del_tmp0 += (ctim - vehinf0.SectionEntranceT) - (secinf.length - vehinf0.distance2End) / (secinf.speedLimit / 3.6);
						jj++;
						//if (sigid == 3014 && link_id == 1246 && j == 1) AKIPrintString(("Link Lane Delay, Ctime: " + to_string(ctim) + ", entry: " + to_string(vehinf0.SectionEntranceT) + ", temp delay: " + to_string(lan_cnt[j - 1])).c_str());
					}
				}
				del_tot = del_tot - del_tmp0;
				if (del_tot < 0) del_tot = 0;
			}
		}
	}
	//if (sigid == 3014) AKIPrintString(("Delay: " + to_string(del_tot) + ", in-flow: " + to_string(sigtim[0]) + ", out-flow: " + to_string(sigtim[1]) + ", out-flow 2: " + to_string(sigtim[2])).c_str());

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
				tdel = 0;
				tdel = SigDelay(sigid, Step, dt);
				if (flag == 0) {
					topt = tdel;
					for (int i = 0; i < 4; i++) {
						sigmin[i] = sigtim[i];
					}
					flag = 1;
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
						tdel = 0;
						tdel = SigDelay(sigid, Step, dt);
						if (flag == 0) {
							topt = tdel;
							for (int i = 0; i < 4; i++) {
								sigmin[i] = sigtim[i];
							}
							flag = 1;
						}
						else {
							if (tdel < topt) {
								topt = tdel;
								for (int i = 0; i < 4; i++) {
									sigmin[i] = sigtim[i];
								}
							}
						}
						//AKIPrintString(("Signal ID: " + to_string(sigid) + ", total delay: " + to_string(i) + ", optimal: " + to_string(topt) + ", Phase 1: " + to_string(p1)).c_str());
					}
				}
				//AKIPrintString(("Optimal Plan 1: " + to_string(sigmin[0]) + ", 2: " + to_string(sigmin[1]) + ", 3: " + to_string(sigmin[2]) + ", 4: " + to_string(sigmin[3])).c_str());
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
							tdel = 0;
							tdel = SigDelay(sigid, Step, dt);
							if (flag == 0) {
								topt = tdel;
								for (int i = 0; i < 4; i++) {
									sigmin[i] = sigtim[i];
								}
								flag = 1;
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


		ECIDisableEvents(sigid);
		
		/*
		if (sigid == 3014) {
			sigmin[0] = 5;
			sigmin[1] = 85;
			sigmin[2] = 90;
		}
		*/
		
		sigph_time[i][0] = 0;
		for (int p = 0; p < nph; p++) {
			if (p == 0) {
				ph_dur = sigmin[p];
			}
			else {
				ph_dur = sigmin[p] - sigmin[p - 1];
			}
			sigph_dur[i][p] = ph_dur;			// update the phase duration of all controlled signals
			sigph_time[i][p+1] = sigmin[p];
			//int Report = ECIChangeTimingPhase(sigid, p + 1, ph_dur, timeSta);		// set the new timing plan
			
			//if (sigid == 3014) AKIPrintString(("Temp sig ID: " + to_string(Report) + ", the phase id is: " + to_string(p+1) + ", new phase duration is: " + to_string(ph_dur)).c_str());
		}
	}
}

void withoutEcodrive_trajectory_output(double simtime, string control_method, int sec_from) {

	double tstep = 0.2, grade = 0.0;
	double ice_energy, bev_energy, phev1_energy, phev2_energy, hfcv_energy, hev_energy;
	int sec_from_previous_line;

	for (int j = 0; j < lnk_eco; j++) {
		if (control_method == control_strategy[0] || control_method == control_strategy[1]) {

			if (j == 0) {
				sec_from_previous_line = lnk_ctl[j][0];
				sec_from = sec_from_previous_line;
			}
			else {
				sec_from_previous_line = lnk_ctl[j - 1][0];
				sec_from = lnk_ctl[j][0];
			}

			if ((j == 0) || (j > 0 && sec_from != sec_from_previous_line)) {
				/*AKIPrintString(("control section is: " + to_string(sec_from)).c_str());*/
				int nbveh_upstream_section = AKIVehStateGetNbVehiclesSection(sec_from, true);
				for (int veh_num = 0; veh_num < nbveh_upstream_section; veh_num++) {
					InfVeh vehinf_upstream = AKIVehStateGetVehicleInfSection(sec_from, veh_num);
					VehicleType type_id_upstream = static_cast<VehicleType>(AKIVehTypeGetIdVehTypeANG(vehinf_upstream.type));
					double vehicle_acc = (vehinf_upstream.CurrentSpeed - vehinf_upstream.PreviousSpeed) / (3.6 * tstep);
					double distance_by_tstep = vehinf_upstream.PreviousSpeed / 3.6 * tstep / 1000; // in unit of km
					Emission(vehinf_upstream.CurrentSpeed, grade, vehicle_acc, type_id_upstream, ice_energy, bev_energy, phev1_energy, phev2_energy, hfcv_energy, hev_energy);
					int type_id_output = AKIVehTypeGetIdVehTypeANG(vehinf_upstream.type); // VehicleType class cannot be output, so should convert to int
					fecotraj_output << simtime << "\t" << sec_from << "\t" << type_id_output << "\t" << vehinf_upstream.numberLane << "\t" << vehinf_upstream.idVeh << "\t" << vehinf_upstream.distance2End << "\t" << vehinf_upstream.CurrentPos
						<< "\t" << vehinf_upstream.CurrentSpeed << "\t" << vehicle_acc << "\t" << distance_by_tstep << "\t" << ice_energy << "\t" << bev_energy << "\t" << phev1_energy << "\t" << phev2_energy << "\t"
						<< hfcv_energy << "\t" << hev_energy << std::endl;
				}
			}
		}
		int sec_to = lnk_ctl[j][1];
		int nbveh_downstream_section = AKIVehStateGetNbVehiclesSection(sec_to, true);
		//AKIPrintString(("downstream section is: " + to_string(sec_to)).c_str());
		for (int veh_num = 0; veh_num < nbveh_downstream_section; veh_num++) {
			InfVeh vehinf_downstream = AKIVehStateGetVehicleInfSection(sec_to, veh_num);
			VehicleType type_id_downstream = static_cast<VehicleType>(AKIVehTypeGetIdVehTypeANG(vehinf_downstream.type));
			double vehicle_acc = (vehinf_downstream.CurrentSpeed - vehinf_downstream.PreviousSpeed) / (3.6 * tstep);
			double distance_by_tstep = vehinf_downstream.PreviousSpeed / 3.6 * tstep; // in unit of m
			Emission(vehinf_downstream.CurrentSpeed, grade, vehicle_acc, type_id_downstream, ice_energy, bev_energy, phev1_energy, phev2_energy, hfcv_energy, hev_energy);
			int type_id_output = AKIVehTypeGetIdVehTypeANG(vehinf_downstream.type); // VehicleType class cannot be output, so should convert to int
			fecotraj_output << simtime << "\t" << sec_to << "\t" << type_id_output << "\t" << vehinf_downstream.numberLane << "\t" << vehinf_downstream.idVeh << "\t" << vehinf_downstream.distance2End << "\t" << vehinf_downstream.CurrentPos
				<< "\t" << vehinf_downstream.CurrentSpeed << "\t" << vehicle_acc << "\t" << distance_by_tstep << "\t" << ice_energy << "\t" << bev_energy << "\t" << phev1_energy << "\t" << phev2_energy << "\t"
				<< hfcv_energy << "\t" << hev_energy << std::endl;
		}
	}
} // this func used to output trajectory for control strategy "no control" and "signal opt", and trajectory of downstream section for strategies with ecodrive

void ecoDriveControlSectionOutput()  {
	int offset_ecoDrive, sig_cycle_ecoDrive, ph_cur_ecoDrive, ph_veh_ecoDrive = 2, sig_id_ecoDrive = 5287; // ph_veh=2 means only control phase 2? but why the ecodrive file has two phases? may have error without initialization
	int vn_lan_ecoDrive[6]; // vn_lan represents the total number of vehicles in each lane of the controlled section
	double ttg_ecoDrive, ttg0_ecoDrive, ttg1_ecoDrive, dtg1_ecoDrive, ttr_ecoDrive, ph_start_ecoDrive, tstep_ecoDrive = 0.2;
	double max_ecoDrive, min_ecoDrive, dur_ecoDrive;
	double spd_opt_ecoDrive = 100.0, acc_opt1_ecoDrive = 0, acc_opt2_ecoDrive = 3.0, spd_wave_ecoDrive, spd_pre_ecoDrive[7];
	double ta_lan_ecoDrive[6], t0_lan_ecoDrive[6], flw_lan_ecoDrive[6]; // ta_lan/t0_lan is absolute time of entering the current section
	double vlen_ecoDrive = 5.0, d2t_ecoDrive, d2a_ecoDrive; // d2t: distance to the end of the queue, d2a: distance to accelerate to speed limit
	double simtime_ecoDrive = AKIGetCurrentSimulationTime();
	int flag_queue_estimate_method_ecoDrive = 0;		// 0 for accurate queue length estimation, 1 for traffic flow-based queue length estimation ??
	int lnk_order_ecoDrive = 0, flg_lane_ecoDrive = 0;		// flag to indicate whether a vehicle is staying on a wrong lane
	int Step_ecoDrive = 18;
	double dt_ecoDrive = 5.0;
	double ph_dur_ecoDrive[8]; // there are a total of 4 phases, why use 8? ph_dur refers to phase duration
	double grade_ecoDrive = 0.0;
	double ice_energy, bev_energy, phev1_energy, phev2_energy, hfcv_energy, hev_energy;

	int secnb = AKIInfNetNbSectionsANG();
	for (int i = 0; i < secnb; i++) {
		int secid = AKIInfNetGetSectionANGId(i);
		A2KSectionInf secinf = AKIInfNetGetSectionANGInf(secid);
		spd_wave_ecoDrive = 1.2 * secinf.capacity / (200 - secinf.capacity / secinf.speedLimit);		// calculate the rarefaction wave speed	// increase the wave speed by 20%, updated on March 05, 2025
		// StructAkiEstadSection sec_stat = AKIEstGetCurrentStatisticsSection(secid, 0);   // not used?

		if (eco_flg[secid] == 1) {
			for (int l = 0; l < 6; l++) {
				vn_lan_ecoDrive[l] = 0;		// initialize the number of vehicles in each lane as 0
				// the number of lanes at each control section is 3, there are two control sections
				ta_lan_ecoDrive[l] = 0;
				t0_lan_ecoDrive[l] = 0;
				flw_lan_ecoDrive[l] = 0;
			}

			lnk_order_ecoDrive = sig_lnk[secid];
			int nbveh = AKIVehStateGetNbVehiclesSection(secid, true);

			for (int k = 0; k < nbveh; k++) {
				InfVeh vehinf = AKIVehStateGetVehicleInfSection(secid, k);
				if (vn_lan_ecoDrive[vehinf.numberLane - 1] == 0) {
					spd_pre_ecoDrive[vehinf.numberLane - 1] = secinf.speedLimit;
					t0_lan_ecoDrive[vehinf.numberLane - 1] = vehinf.SectionEntranceT;
				}

				ta_lan_ecoDrive[vehinf.numberLane - 1] = vehinf.SectionEntranceT;
				vn_lan_ecoDrive[vehinf.numberLane - 1]++; // indicate which lane the vehicle is located and accumulate the number of vehicles in that lane
				if (vn_lan_ecoDrive[vehinf.numberLane - 1] > 1) {
					flw_lan_ecoDrive[vehinf.numberLane - 1] = (double(vn_lan_ecoDrive[vehinf.numberLane - 1]) - 1) / (ta_lan_ecoDrive[vehinf.numberLane - 1] - t0_lan_ecoDrive[vehinf.numberLane - 1]) * 3600;    // in flow rate at vph
				}

				//AKIVehTrackedModifyLane(vehinf.idVeh, 0);			// does not allow lane changing in the control section
				//int type_id = AKIVehTypeGetIdVehTypeANG(vehinf.type);
				VehicleType type_id = static_cast<VehicleType>(AKIVehTypeGetIdVehTypeANG(vehinf.type));

				// Calculate the number of occurance of the CAV ID in the cav vehicle list, if >0, that means the vehicle is CAV
				int count_occurrance_of_target_in_vector = (int)count(cav_vehicle_list.begin(), cav_vehicle_list.end(), type_id);
				//AKIPrintString(("the number of count_occurrance_of_target_in_vector is: " + to_string(count_occurrance_of_target_in_vector)).c_str());

				if (count_occurrance_of_target_in_vector > 0) {		// eco-driving only applied to CAVs
					int sec_to = AKIVehInfPathGetNextSection(vehinf.idVeh, secid);
					for (int j = 0; j < lnk_eco; j++) {		// identify the phase of the vehicle
						if (secid == lnk_ctl[j][0] && sec_to == lnk_ctl[j][1]) {
							ph_veh_ecoDrive = lnk_ctl[j][2];  // the phase that would be controlled
							sig_id_ecoDrive = lnk_ctl[j][3];  // control signal ID
							//AKIPrintString(("section_id is: " + to_string(lnk_ctl[j][0]) + " ph_veh is: " + to_string(lnk_ctl[j][2]) + " eco_flg is: " + to_string(eco_flg[secid])).c_str());
							break;
						}
					}


					//AKIPrintString(("section id is: " + to_string(secid) + "section order is: " + to_string(lnk_order_ecoDrive) + "numberLane is: " + to_string(vehinf.numberLane)).c_str());

					int ph_lan = lan_ctl[lnk_order_ecoDrive][vehinf.numberLane];			// phase of current lane
					//AKIPrintString(("ph_lane is: " + to_string(ph_lan) + " and lnk_ctl is: " + to_string(ph_veh_ecoDrive)).c_str());
					if (ph_lan == ph_veh_ecoDrive) {
						flg_lane_ecoDrive = 0;
					}
					else {
						flg_lane_ecoDrive = 1;
					}// the vehicle stay at a wrong lane of its expected phase, we don't control the vehicle at the wrong lane
					//AKIPrintString(("wrong lane flag is: " + to_string(flg_lane_ecoDrive)).c_str());

					// find the current status and the time to green or time to red
					offset_ecoDrive = ECIGetOffset(sig_id_ecoDrive); // not used?


					
					/*
					sig_cycle_ecoDrive = ECIGetControlCycleofJunction(0, sig_id_ecoDrive);
					for (int p = 0; p < ECIGetNbPhasesofJunction(0, sig_id_ecoDrive); p++) {
						int flg = ECIGetDurationsPhase(sig_id_ecoDrive, p + 1, simtime_ecoDrive, &dur_ecoDrive, &max_ecoDrive, &min_ecoDrive); // p is phase ID
						ph_dur_ecoDrive[p] = dur_ecoDrive;
					}
					ph_cur_ecoDrive = ECIGetCurrentPhase(sig_id_ecoDrive);	// current phase, return current phase ID
					ph_start_ecoDrive = ECIGetStartingTimePhase(sig_id_ecoDrive);		// start time of the current phase
					*/
					//******************************updated for Integrated eco-driving and signal optimization*****************//updated on March 05, 2025
					if (ctl_flg_coop == 0) {
						sig_cycle_ecoDrive = ECIGetControlCycleofJunction(0, sig_id_ecoDrive);
						//AKIPrintString(("signal cycle is: " + to_string(sig_cycle_ecoDrive)).c_str());

						for (int p = 0; p < ECIGetNbPhasesofJunction(0, sig_id_ecoDrive); p++) {
							int flg = ECIGetDurationsPhase(sig_id_ecoDrive, p + 1, simtime_ecoDrive, &dur_ecoDrive, &max_ecoDrive, &min_ecoDrive); // p is phase ID
							ph_dur_ecoDrive[p] = dur_ecoDrive;
						}
						ph_cur_ecoDrive = ECIGetCurrentPhase(sig_id_ecoDrive);	// current phase, return current phase ID
						ph_start_ecoDrive = ECIGetStartingTimePhase(sig_id_ecoDrive);		// start time of the current phase
					}
					else {
						sig_cycle_ecoDrive = 90;
						sig_idx = sig_flg[sig_id_ecoDrive];
						t10 = int(simtime_ecoDrive * 10) % (sig_cycle_ecoDrive * 10);

						for (int p = 0; p < sig_ctl[sig_idx][1]; p++) {
							ph_dur_ecoDrive[p] = sigph_dur[sig_idx][p];
							if (t10 >= sigph_time[sig_idx][p] * 10 && t10 < sigph_time[sig_idx][p + 1] * 10) {
								ph_cur_ecoDrive = p + 1;
								ph_start_ecoDrive = sigph_time[sig_idx][p] + (simtime_ecoDrive - t10 / 10);
							}
						}
					}
					//*********************************************************************************************************************//
					
					
					if (ph_veh_ecoDrive == ph_cur_ecoDrive) {
						ttr_ecoDrive = ph_dur_ecoDrive[ph_veh_ecoDrive - 1] - (simtime_ecoDrive - ph_start_ecoDrive); // time to red
						ttg_ecoDrive = ttr_ecoDrive + sig_cycle_ecoDrive - ph_dur_ecoDrive[ph_veh_ecoDrive - 1]; // time to green refers to the time to next green from the current green
					}
					else {
						if (ph_veh_ecoDrive > ph_cur_ecoDrive) {
							ttg_ecoDrive = 0;
							for (int ph = ph_cur_ecoDrive; ph < ph_veh_ecoDrive; ph++) {
								ttg_ecoDrive += ph_dur_ecoDrive[ph - 1];
							}
							ttg_ecoDrive = ttg_ecoDrive - (simtime_ecoDrive - ph_start_ecoDrive);
							ttr_ecoDrive = ttg_ecoDrive + ph_dur_ecoDrive[ph_veh_ecoDrive - 1];
						}
						else {
							ttg_ecoDrive = 0;
							for (int ph = ph_cur_ecoDrive; ph <= ECIGetNbPhasesofJunction(0, sig_id_ecoDrive); ph++) {
								ttg_ecoDrive += ph_dur_ecoDrive[ph - 1];
							}
							ttg_ecoDrive = ttg_ecoDrive - (simtime_ecoDrive - ph_start_ecoDrive);
							for (int ph = 1; ph < ph_veh_ecoDrive; ph++) {
								ttg_ecoDrive += ph_dur_ecoDrive[ph - 1];
							}
							ttr_ecoDrive = ttg_ecoDrive + ph_dur_ecoDrive[ph_veh_ecoDrive - 1];
						}
					}

					// apply eco-driving for the CAV
					// apply it when the current phase is red first
					// when the current phase is green but vehicle cannot get through the junction if not speeding?
					if (ph_veh_ecoDrive != ph_cur_ecoDrive) {
						AKIVehSetAsTracked(vehinf.idVeh);

						// control the speed of CVs
						spd_opt_ecoDrive = vehinf.CurrentSpeed;
						if (flag_queue_estimate_method_ecoDrive == 0) { // indicate which queue estimation method is used
							d2t_ecoDrive = vehinf.distance2End - (vn_lan_ecoDrive[vehinf.numberLane - 1] - 1) * vlen_ecoDrive;		// what does this mean? why (vn_lan[vehinf.numberLane - 1] * vlen)? ((vn_lan[vehinf.numberLane - 1]-1) might get negative values.
							ttg0_ecoDrive = ttg_ecoDrive + (vn_lan_ecoDrive[vehinf.numberLane - 1] - 1) * vlen_ecoDrive / (spd_wave_ecoDrive / 3.6);		// time until the queue is released
							d2a_ecoDrive = (vn_lan_ecoDrive[vehinf.numberLane - 1] * vlen_ecoDrive) + 100; // 100 is arbitrarily set, assuming vehicle would be able to accelerate back to speed limit within 100m
						}
						else { // update the input of eco-driving for traffic flow-based queue
							ttg1_ecoDrive = (sig_cycle_ecoDrive - ph_dur_ecoDrive[ph_cur_ecoDrive - 1]) - ttg_ecoDrive; // ttg1 refers to how much red time has been through for the control phase
							ttg1_ecoDrive += vehinf.distance2End / (secinf.speedLimit / 3.6); // ?? queue formation (wait for manual figure)
							d2t_ecoDrive = flw_lan_ecoDrive[vehinf.numberLane - 1] * ttg1_ecoDrive / 3600 * vlen_ecoDrive; // d2t here refers to queue length during red
							ttg0_ecoDrive = ttg_ecoDrive + d2t_ecoDrive / (spd_wave_ecoDrive / 3.6);
							d2a_ecoDrive = d2t_ecoDrive + 100;
							d2t_ecoDrive = vehinf.distance2End - d2t_ecoDrive; // d2t here refers to the distance to the end of queue
						}

						if (d2t_ecoDrive > 0) {
							int flag = EcoDriveFunc(type_id, d2t_ecoDrive, d2a_ecoDrive, ttg0_ecoDrive, vehinf.CurrentSpeed, secinf.speedLimit, spd_opt_ecoDrive, acc_opt1_ecoDrive, acc_opt2_ecoDrive);
							
							if (flag > 0) {
								if (spd_opt_ecoDrive < vehinf.CurrentSpeed - acc_opt1_ecoDrive * tstep_ecoDrive * 3.6) spd_opt_ecoDrive = vehinf.CurrentSpeed - acc_opt1_ecoDrive * tstep_ecoDrive * 3.6;
								//if (secid == 1246) AKIPrintString(("Control Indicator: " + to_string(vehinf.CurrentSpeed) + ", veh ID = " + to_string(vehinf.idVeh) + ", flag = " + to_string(vehinf.CurrentSpeed - acc_opt1_ecoDrive * tstep_ecoDrive * 3.6) + ", Speed = " + to_string(spd_opt_ecoDrive) + ", Acc = " + to_string(acc_opt1_ecoDrive) + ", distance = " + to_string(vehinf.distance2End) + ", TTG = " + to_string(ttg_ecoDrive)).c_str());

								if (flg_lane_ecoDrive >= 0) {
									AKIVehTrackedModifySpeed(vehinf.idVeh, spd_opt_ecoDrive);			// if a CAV is on a wrong lane, then we do not apply eco-driving. We do not control anything to this vehicle 
								}
							}
							
							// the code is just used to verify if results are correct
							/*if (secid == 1249 && ph_veh_ecoDrive == 2)
								AKIPrintString(("Control Indicator: " + to_string(vehinf.CurrentSpeed) + ", phase = " + to_string(flag) + ", Speed = " + to_string(spd_opt_ecoDrive) + ", Acc = " + to_string(acc_opt1_ecoDrive) + ", distance = " + to_string(vehinf.distance2End) + ", TTG = " + to_string(ttg_ecoDrive)).c_str());*/
								// print out data to check if it is correct
						}
					}
					else { // deal with the situation with queue at green light

						if (spd_pre_ecoDrive[vehinf.numberLane - 1] < 1.0) { // the previous vehicle is in the queue
							ttg1_ecoDrive = ph_dur_ecoDrive[ph_cur_ecoDrive - 1] - ttr_ecoDrive;
							dtg1_ecoDrive = secinf.capacity / secinf.nbCentralLanes * ttg1_ecoDrive / 3600; // In most cases, nbcentrallane determine section capacity in Aimsun, adding side lane won't significantly impact capacity
							dtg1_ecoDrive += (vn_lan_ecoDrive[vehinf.numberLane - 1] - 1);		// number of vehicles before the ego-car, i.e., in the queue

							if (flag_queue_estimate_method_ecoDrive == 0) {
								ttg0_ecoDrive = dtg1_ecoDrive * vlen_ecoDrive / (spd_wave_ecoDrive / 3.6);
								ttg0_ecoDrive = ttg0_ecoDrive - (ph_dur_ecoDrive[ph_cur_ecoDrive - 1] - ttr_ecoDrive);
								d2t_ecoDrive = vehinf.distance2End - dtg1_ecoDrive * vlen_ecoDrive;
								d2a_ecoDrive = (dtg1_ecoDrive * vlen_ecoDrive) + 100;
							}
							else {
								ttg1_ecoDrive = sig_cycle_ecoDrive - ttg_ecoDrive;
								ttg1_ecoDrive += vehinf.distance2End / (secinf.speedLimit / 3.6);
								d2t_ecoDrive = flw_lan_ecoDrive[vehinf.numberLane - 1] * ttg1_ecoDrive / 3600 * vlen_ecoDrive;
								ttg0_ecoDrive = d2t_ecoDrive / (spd_wave_ecoDrive / 3.6);
								ttg0_ecoDrive = ttg0_ecoDrive - (ph_dur_ecoDrive[ph_cur_ecoDrive - 1] - ttr_ecoDrive);
								d2a_ecoDrive = d2t_ecoDrive + 100;
								d2t_ecoDrive = vehinf.distance2End - d2t_ecoDrive;
							}
							if (d2t_ecoDrive > 0) {
								int flag = EcoDriveFunc(type_id, d2t_ecoDrive, d2a_ecoDrive, ttg0_ecoDrive, vehinf.CurrentSpeed, secinf.speedLimit, spd_opt_ecoDrive, acc_opt1_ecoDrive, acc_opt2_ecoDrive);
								if (flag > 0) {
									if (spd_opt_ecoDrive < vehinf.CurrentSpeed - acc_opt1_ecoDrive * tstep_ecoDrive * 3.6) spd_opt_ecoDrive = vehinf.CurrentSpeed - acc_opt1_ecoDrive * tstep_ecoDrive * 3.6; // Ensure the speed to which we decelerate next second greater than the final cruise speed, otherwise cruise instead of decelerating
									if (flg_lane_ecoDrive >= 0) {
										AKIVehTrackedModifySpeed(vehinf.idVeh, spd_opt_ecoDrive);
									}

								}
								//if (secid == 1249 && ph_veh_ecoDrive == 2) AKIPrintString(("Control Indicator: " + to_string(vehinf.CurrentSpeed) + ", phase = " + to_string(flag) + ", Speed = " + to_string(spd_opt_ecoDrive) + ", Acc = " + to_string(acc_opt1_ecoDrive) + ", distance = " + to_string(vehinf.distance2End) + ", TTG = " + to_string(ttg_ecoDrive)).c_str());
							}
						}
					}
				}

				spd_pre_ecoDrive[vehinf.numberLane - 1] = vehinf.CurrentSpeed; // calculate the speed of the previous vehicle, the current vehicle would be the leader of the immediate following vehicle at the next iteration

				// calculate second_by_second distance and energy 
				double vehicle_acc = (vehinf.CurrentSpeed - vehinf.PreviousSpeed) / (3.6 * tstep_ecoDrive);
				double distance = vehinf.PreviousSpeed / 3.6 * tstep_ecoDrive / 1000; // in unit of km
				Emission(vehinf.CurrentSpeed, grade_ecoDrive, vehicle_acc, type_id, ice_energy, bev_energy, phev1_energy, phev2_energy, hfcv_energy, hev_energy);
				int vehicle_type_output = AKIVehTypeGetIdVehTypeANG(vehinf.type);
				// save eco-driving vehicle trajectory: time, section ID, vehicle type, lane ID, vehicle ID, distance to signal, current speed
				fecotraj_output << simtime_ecoDrive << "\t" << secid << "\t" << vehicle_type_output << "\t" << vehinf.numberLane << "\t" << vehinf.idVeh << "\t" << vehinf.distance2End << "\t" << vehinf.CurrentPos << "\t" << vehinf.CurrentSpeed << "\t" <<
					vehicle_acc << "\t" << distance << "\t" << ice_energy << "\t" << bev_energy << "\t" << phev1_energy << "\t" << phev2_energy << "\t" << hfcv_energy << "\t" << hev_energy << std::endl;
				//AKIPrintString(("type_id is: " + to_string(type_id) + "\n" + "vehicle_id is: " + to_string(vehinf.idVeh) + "\n" + "speed is: " + to_string(vehinf.CurrentSpeed)).c_str());
			}
		}
	}
}// this func used to develop eco-drive control and output trajectroy on control sections

int AAPIManage(double time, double timeSta, double timTrans, double cycle) // AAPI is the main function 
{
	int offset, sig_cycle, ph_cur, ph_veh = 2, sig_id = 5287; // ph_veh=2 means only control phase 2? but why the ecodrive file has two phases? may have error without initialization
	int vn_lan[6]; // vn_lan represents the total number of vehicles in each lane of the controlled section
	double ttg, ttg0, ttg1, dtg1, ttr, ph_start, tstep = 0.2;
	double max, min, dur;
	double spd_opt = 100.0, acc_opt1 = 0, acc_opt2 = 3.0, spd_wave, spd_pre[7];
	double ta_lan[6], t0_lan[6], flw_lan[6]; // ta_lan/t0_lan is absolute time of entering the current section
	double vlen = 5.0, d2t, d2a; // d2t: distance to the end of the queue, d2a: distance to accelerate to speed limit
	double simtime = AKIGetCurrentSimulationTime();
	int flag_queue_estimate_method = 1;		// 0 for accurate queue length estimation, 1 for traffic flow-based queue length estimation ??
	int lnk_order = 0, flg_lane = 0;		// flag to indicate whether a vehicle is staying on a wrong lane
	int Step = 18;
	double dt = 5.0, t10;
	int sec_from = 0; // which section the turn makes from (control section)
	int sigid, nph, ActReport, EEReport, ph_ch;

	/*
	if (control_method == control_strategy[0]) {  // no control
		withoutEcodrive_trajectory_output(simtime, control_method, sec_from); 
	}

	if (control_method == control_strategy[1]) {  // signal optimization only

		if (simtime > 0) {
			t10 = (int(simtime * 10) % int(Step * dt * 10));
			if (t10 == 0) {
				SigOptFunc(timeSta, Step, dt);
			}

			for (int i = 0; i < nsig; i++) {
				sigid = sig_ctl[i][0];
				nph = sig_ctl[i][1];
				ActReport = -100;

				for (int p = 0; p < nph; p++) {

					if (t10 == (sigph_time[i][p]) * 10) { // sigph_time is the start time of each phase within each update cycle

						ph_ch = p + 1;
						if (p == 0) {
							ph_ch = 1;
						}
						ECIDisableEvents(sigid);
						ActReport = ECIChangeTimingPhase(sigid, ph_ch, sigph_dur[i][p], timeSta);
						EEReport = ECIChangeDirectPhase(sigid, ph_ch, timeSta, time, cycle, sigph_dur[i][p]);
					}
				}
			}
		}

		withoutEcodrive_trajectory_output(simtime, control_method, sec_from);
	}

	if (control_method == control_strategy[2]) {  // eco-driving only
		// output vehicle trajectory in downwtream sections of the control section
		//AKIPrintString("eco-driving is applied.");

		// eco-driving 
		ecoDriveControlSectionOutput();
		withoutEcodrive_trajectory_output(simtime, control_method, sec_from); // For this control strategy, this func only outputs downstream trajectory, upstream trajecotry is output later
	}
	
	if (control_method == control_strategy[3]) {  // proposed cooperative control
		if (simtime > 0) {
			t10 = (int(simtime * 10) % int(Step * dt * 10));
			if (t10 == 0) {
				SigOptFunc(timeSta, Step, dt);
			}

			for (int i = 0; i < nsig; i++) {
				sigid = sig_ctl[i][0];
				nph = sig_ctl[i][1];
				ActReport = -100;

				for (int p = 0; p < nph; p++) {

					if (t10 == (sigph_time[i][p]) * 10) { // sigph_time is the start time of each phase within each update cycle

						ph_ch = p + 1;
						if (p == 0) {
							ph_ch = 1;
						}
						ECIDisableEvents(sigid);
						ActReport = ECIChangeTimingPhase(sigid, ph_ch, sigph_dur[i][p], timeSta);
						EEReport = ECIChangeDirectPhase(sigid, ph_ch, timeSta, time, cycle, sigph_dur[i][p]);
					}
				}
			}
		}

		// eco-driving 
		ecoDriveControlSectionOutput();

		// output vehicle trajectory in downwtream sections of the control section
		withoutEcodrive_trajectory_output(simtime, control_method, sec_from); // For this control strategy, this func only outputs downstream trajectory, upstream trajecotry is output later
	}
	*/

	if (control_method == control_strategy[3]) {  // proposed cooperative control
		if (simtime > 0) {
			t10 = (int(simtime * 10) % int(Step * dt * 10));
			if (t10 == 0) {
				SigOptFunc(timeSta, Step, dt);
				ctl_flg_coop = 1;		//updated on March 05, 2025
			}

			for (int i = 0; i < nsig; i++) {
				sigid = sig_ctl[i][0];
				nph = sig_ctl[i][1];
				ActReport = -100;

				for (int p = 0; p < nph; p++) {
					if (t10 == (sigph_time[i][p]) * 10) { // sigph_time is the start time of each phase within each update cycle
						ph_ch = p + 1;
						if (p == 0) {
							ph_ch = 1;
						}
						ECIDisableEvents(sigid);
						ActReport = ECIChangeTimingPhase(sigid, ph_ch, sigph_dur[i][p], timeSta);
						EEReport = ECIChangeDirectPhase(sigid, ph_ch, timeSta, time, cycle, sigph_dur[i][p]);
					}
				}
			}
		}

		// eco-driving 
		ecoDriveControlSectionOutput();

		// output vehicle trajectory in downwtream sections of the control section
		//withoutEcodrive_trajectory_output(simtime, control_method, sec_from); // For this control strategy, this func only outputs downstream trajectory, upstream trajecotry is output later
	}

	return 0;
}

int AAPIPostManage(double time, double timeSta, double timTrans, double acicle)
{
	//int Step = 18;
	//double dt = 5.0, t10, tstep = 0.2;
	//int sigid, nph;
	//double simtime = AKIGetCurrentSimulationTime();
	//int ActReport, EEReport, ph_ch;

	////signal optimization system;
	//if (simtime > 0) {
	//	t10 = (int(simtime * 10) % int(Step * dt * 10));
	//	if (t10 == 0) {
	//		SigOptFunc(timeSta, Step, dt);
	//	}
	//	
	//	for (int i = 0; i < nsig; i++) {
	//		sigid = sig_ctl[i][0];
	//		nph = sig_ctl[i][1];
	//		ActReport = -100;
	//		
	//		for (int p = 0; p < nph; p++) {
	//			/*
	//			if (t10 == sigph_time[i][p] * 10) {
	//				ECIDisableEvents(sigid);
	//				if (p + 1 == nph) {
	//					ECIEnableEventsActivatingPhase(sigid, 1, 0.0, time);
	//				}
	//				else {
	//					ECIEnableEventsActivatingPhase(sigid, p + 2, 0.0, time);
	//				}
	//			}
	//			*/
	//			//if (sigid == 3014 && t10 + 2 == sigph_time[i][p] * 10) AKIPrintString(("Simulation Time: " + to_string(t10/10) + ", phase: " + to_string(Step * dt) + ", duration: " + to_string(sigph_time[i][p])).c_str());
	//			
	//			if (t10 == (sigph_time[i][p])*10) { // sigph_time is the start time of each phase within each update cycle
	//				
	//				ph_ch = p + 1;
	//				if (p == 0) {
	//					ph_ch = 1;
	//				}

	//				ECIDisableEvents(sigid);
	//				ActReport = ECIChangeTimingPhase(sigid, ph_ch, sigph_dur[i][p], timeSta);
	//				//EEReport = ECIEnableEventsActivatingPhase(sigid, ph_ch, sigph_dur[i][p], timeSta);
	//				EEReport = ECIChangeDirectPhase(sigid, ph_ch, timeSta, time, acicle, sigph_dur[i][p]);

	//				/*
	//				if (sigid == 3014) {
	//					AKIPrintString(("Control Output: Report 1: " + to_string(ECIGetCurrentPhase(sigid)) + ", report 2: " + to_string(ActReport) + ", phase id: " + to_string(ph_ch) + ", time: " + to_string(nph)).c_str());
	//					AKIPrintString(("Phase duration: 0: " + to_string(sigph_dur[i][0]) + ", 1: " + to_string(sigph_dur[i][1]) + ", 2: " + to_string(sigph_dur[i][2]) + ", 3: " + to_string(sigph_dur[i][3])).c_str());
	//				}
	//				*/
	//			}
	//		}
	//		//if (sigid == 3014) AKIPrintString(("Simulation Time: " + to_string(t10 / 10) + ", phase: " + to_string(ECIGetCurrentPhase(sigid))).c_str());
	//	}
	//}
	

	return 0;
}

int AAPIFinish()
{
	fecotraj_output.close();
	AKIPrintString("\tFinish");

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