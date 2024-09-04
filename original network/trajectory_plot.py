import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

trajectory_results = pd.read_csv('D:\projects\eco-driving_powertrainOpt\original network\Eco_Traj.txt', delimiter='\t')
filter_trajctory_by_time = trajectory_results[trajectory_results['simulation_time'].between(4100, 4500)]
vehicle_id = filter_trajctory_by_time.vehicle_id.unique()

control_vehicle_id = 5380
non_control_vehicle_id = 154
section_id = [1249]

plt.figure(figsize=(20,8))
for veh_id in vehicle_id:
    veh_id_df = filter_trajctory_by_time[filter_trajctory_by_time.vehicle_id == veh_id].sort_values('simulation_time')
    veh_id_df = veh_id_df[veh_id_df['section_id'].isin(section_id)]
    veh_id_df = veh_id_df[veh_id_df['numberLane'].isin([1,2])]

    # veh_id_trajectory = veh_id_df.CurrentSpeed
    distance2End = veh_id_df['distance2End']
    time = veh_id_df['simulation_time']

    if control_vehicle_id in veh_id_df['veh_type_id'].unique():
        color = 'g'
    else:
        color = 'k--'

    plt.plot(time, distance2End, color)
    plt.ylim((0, 250))
    plt.xlabel('Time (s)', fontdict={'size':18, 'color':'black'})
    plt.ylabel('Distance to the stop line (m)', fontdict={'size': 18})
    plt.xticks(fontsize = 18)
    plt.yticks(fontsize = 18)

plt.show()