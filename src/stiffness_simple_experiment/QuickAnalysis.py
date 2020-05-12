#!/usr/bin/env python

import pandas as pd
import matplotlib.pyplot as plt

# filePath = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/simple_experiment_data.csv"
filePath = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/part_1/csvs_1_real/simple_experiment_data.csv"

df = []
df = pd.read_csv(filePath)

# print(df.head(5))
# print(df.index.values)
# print(df.columns.values[0])
# print(df.columns.values)
# print(df.columns.values[0])


important_names = ['field.trial_nr','field.trial_time','field.shape_acc',
					'field.orientation_acc','field.shape','field.absolute_angle']


means = df.mean(axis = 0)
stds = df.std(axis = 0)

print(means[important_names])
print(stds[important_names])


ax_time = df.plot(x='field.trial_nr',y='field.trial_time',ylim=(0,8),linewidth=2, marker='o')
ax_time.hlines(means['field.trial_time'],0,20)
ax_time.set_xlabel("trial number",fontsize=18)
ax_time.set_ylabel("time [s]",fontsize=18)
ax_time.legend(["time","average"],loc='best')
# ax_time.legend(loc='best')

ax_shapeAcc = df.plot(x='field.trial_nr',y='field.shape_acc',ylim=(0,102),linewidth=2,marker='o')
ax_shapeAcc.hlines(means['field.shape_acc'],0,20)
ax_shapeAcc.set_xlabel("trial number",fontsize=18)
ax_shapeAcc.set_ylabel("accuracy [%]",fontsize=18)
ax_shapeAcc.legend(["shape accuracy","average"],loc='best')

ax_angleAcc =df.plot(x='field.trial_nr',y='field.orientation_acc',ylim=(0,102),linewidth=2,marker='o')
ax_angleAcc.hlines(means['field.orientation_acc'],0,20)
ax_angleAcc.set_xlabel("trial number",fontsize=18)
ax_angleAcc.set_ylabel("accuracy [%]",fontsize=18)
ax_angleAcc.legend(["orientation accuracy","average"],loc='best')

ax_angle =df.plot(x='field.trial_nr',y='field.absolute_angle',ylim=(0,20),linewidth=2,marker='o')
ax_angle.hlines(means['field.absolute_angle'],0,20)
ax_angle.set_xlabel("trial number",fontsize=18)
ax_angle.set_ylabel("angle [deg]",fontsize=18)
ax_angle.legend(["absolute angle error","average"],loc='best')

# ax_angle =df.plot(x='field.trial_nr',y='field.shape',ylim=(0,0.5),linewidth=2,marker='o')
# ax_angle.hlines(means['field.shape'],0,20)
# ax_angle.set_xlabel("trial number",fontsize=18)
# ax_angle.set_ylabel("angle [deg]",fontsize=18)
# ax_angle.legend(["average principle axis error","average"],loc='best')

plt.show()

print(means['field.trial_time'])
print(stds['field.trial_time'])

# print(df.index.values[0])
# print(df.index.values[-1])
# print(df.shape)
# print(df.index.values[0])
# trials 