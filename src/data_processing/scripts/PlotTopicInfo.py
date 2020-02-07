#!/usr/bin/env python

# For each topic the relevant plot info is shown

class PlotTopicInfo(object):
	def __init__(self,csvName='empty',plotSpecification='empty'):

		# self.info = []
		if csvName != 'empty':
			self.info = self.getInfo(csvName)


	def getInfo(self,csvName,plotSpecification='empty'):
	    switcher = {
	        'omni1_button.csv': self.omni1_button,
	        'omni1_lock_state.csv': self.omni1_lock_state,
	        'omni1_force_feedback.csv': self.omni1_force_feedback,
	        'omni1_joint_states.csv': self.omni1_joint_states,
	        'eigen_pair.csv': self.eigen_pair,
	        'covariance_matrix.csv': self.covariance_matrix,
	        'stiffness_command.csv': self.stiffness_command,
	        'omni_stiffness.csv': self.omni_stiffness,
	        'virtual_robot_force.csv': self.virtual_robot_force,
	        'omni_rotation_tf.csv': self.omni_rotation_tf,
	        'wrist_ft_tool_link_tf.csv': self.wrist_ft_tool_link_tf,
	        'virtual_marker_tf.csv': self.virtual_marker_tf,
	        'marker_visualization.csv': self.marker_visualization,
	        'draw_ellipsoidellipsoid_visualization.csv': self.draw_ellipsoidellipsoid_visualization,
	    }		
	    if csvName in switcher:
	    	func = switcher.get(csvName, lambda: "invalid csv name: {}".format(csvName))
	    	if plotSpecification == 'empty':
	    		self.info = func()
	    		return func()
	    	else:
	    		self.info = func(plotSpecification)
	    		return func(plotSpecification)

	    else: 
	    	print("invalid csv name: {}".format(csvName))


	def omni1_button(self):
		plotInfoDict = {}
		return plotInfoDict

	def omni1_lock_state(self, plotType='curent_position'):

		if plotType == 'current_position':
			plotInfoDict = {'title': "Omni position",
						'xLabel': "Time [s]",
						'yLabel': "Postion [m]",
						'xAxis': "timeVec",
						'yAxis': ["field.current_position.x","field.current_position.y","field.current_position.z"],
						'events': ['field.lock_grey','field.lock_white'],
						'legend': ["x","y","z","Force on","stiffness locked"]
			}
			return plotInfoDict

		elif plotType == 'lock_position':
			plotInfoDict = {'title': "Omni lock position",
						'xLabel': "Time [s]",
						'yLabel': "Postion [m]",
						'xAxis': "timeVec",
						'yAxis': ["field.lock_position.x","field.lock_position.y","field.lock_position.z"],
						'events': ['field.lock_grey','field.lock_white'],
						'legend': ["x","y","z","Force on","stiffness locked"]
			}
			return plotInfoDict

	def omni1_force_feedback(self):
		plotInfoDict = {'title': "Omni force feedback",
					'xLabel': "Time [s]",
					'yLabel': "Force [N]",
					'xAxis': "timeVec",
					'yAxis': ["field.force.x","field.force.y","field.force.z"],
					'legend': ["F_x","F_y","F_z"]
		}
		return plotInfoDict

	def omni1_joint_states(self):
		plotInfoDict = {'title': "Joint States Omni",
					'xLabel': "Time [s]",
					'yLabel': "Jointangle [rad]",
					'xAxis': "timeVec",
					'yAxis': ["field.position0","field.position1","field.position2",
							"field.position3","field.position4","field.position5"],
					'legend': ["waist","shoulder","elbow",
							"wrist1","wrist2","wrist3"]
		}
		return plotInfoDict   

	def eigen_pair(self):
		plotInfoDict = {}
		return plotInfoDict
		
	def covariance_matrix(self):
		plotInfoDict = {'title': "Covariance matrix diagonal",
					'xLabel': "Time [s]",
					'yLabel': "Variance [m]",
					'xAxis': "timeVec",
					'yAxis': ["field.F32MA.data0","field.F32MA.data4","field.F32MA.data8"],
					'legend': ["C_xx","C_yy","C_zz"]
		}
		return plotInfoDict  

	def stiffness_command(self):
		plotInfoDict = {'title': "Stiffness command diagonal",
					'xLabel': "Time [s]",
					'yLabel': "Stiffness [N/m]",
					'xAxis': "timeVec",
					'yAxis': ["field.F32MA.data0","field.F32MA.data4","field.F32MA.data8"],
					'legend': ["K_xx","K_yy","K_zz"]
		}
		return plotInfoDict  
		
	def omni_stiffness(self):
		plotInfoDict = {'title': "Omni stiffness diagonal",
					'xLabel': "Time [s]",
					'yLabel': "Stiffness [N/m]",
					'xAxis': "timeVec",
					'yAxis': ["field.F32MA.data0","field.F32MA.data4","field.F32MA.data8"],
					'legend': ["K_xx","K_yy","K_zz"]
		}
		return plotInfoDict  

	def virtual_robot_force(self):
		plotInfoDict = {'title': "Virtual force on end-effector",
					'xLabel': "Time [s]",
					'yLabel': "Force [N]",
					'xAxis': "timeVec",
					'yAxis': ["field.vector.x","field.vector.y","field.vector.z"],
					'legend': ["F_x","F_y","F_z"]
		}
		return plotInfoDict

	def omni_rotation_tf(self):
		plotInfoDict = {'title': "Omni rotation wrt base",
					'xLabel': "Time [s]",
					'yLabel': "Orientation [rad]",
					'xAxis': "timeVec",
					'yAxis': ["field.transforms0.transform.rotation.x","field.transforms0.transform.rotation.y",
					"field.transforms0.transform.rotation.z","field.transforms0.transform.rotation.w"],
					'legend': ["q_x","q_y","q_z","q_w"]
		}
		return plotInfoDict 

	def wrist_ft_tool_link_tf(self,plotType='translation'):
		if plotType == 'translation':
			plotInfoDict = {'title': "End-effector translation",
						'xLabel': "Time [s]",
						'yLabel': "Position [m]",
						'xAxis': "timeVec",
						'yAxis': ["field.transforms0.transform.translation.x","field.transforms0.transform.translation.y"
						,"field.transforms0.transform.translation.z"],
						'legend': ["x","y","z"]
			}
			return plotInfoDict 
		elif plotType == 'rotation':
			plotInfoDict = {'title': "End-effector transform",
						'xLabel': "Time [s]",
						'yLabel': "Orientation [rad]",
						'xAxis': "timeVec",
						'yAxis': ["field.transforms0.transform.rotation.x","field.transforms0.transform.rotation.y"
						,"field.transforms0.transform.rotation.z","field.transforms0.transform.rotation.w"],
						'legend': ["q_x","q_y","q_z","q_w"]
			}
		else:
			print("{} is not a valid type. Need specification of 'translation' or 'rotation'".format(plotType))

		
	def virtual_marker_tf(self,plotType='translation'):
		if plotType == 'translation':
			plotInfoDict = {'title': "Virtual marker translation",
						'xLabel': "Time [s]",
						'yLabel': "Position [m]",
						'xAxis': "timeVec",
						'yAxis': ["field.transforms0.transform.translation.x","field.transforms0.transform.translation.y"
						,"field.transforms0.transform.translation.z"],
						'legend': ["x","y","z"]
			}
			return plotInfoDict 
		elif plotType == 'rotation':
			plotInfoDict = {'title': "Virtual marker rotation",
						'xLabel': "Time [s]",
						'yLabel': "Orientation [rad]",
						'xAxis': "timeVec",
						'yAxis': ["field.transforms0.transform.rotation.x","field.transforms0.transform.rotation.y"
						,"field.transforms0.transform.rotation.z","field.transforms0.transform.rotation.w"],
						'legend': ["q_x","q_y","q_z","q_w"]
			}
		else:
			print("{} is not a valid type. Need specification of 'translation' or 'rotation'".format(plotType))


	def marker_visualization(self):
		plotInfoDict = {}
		return plotInfoDict
		
	def draw_ellipsoidellipsoid_visualization(self):
		plotInfoDict = {}
		return plotInfoDict  



if __name__== "__main__":

	topics = ['omni1_joint_states.csv', 'omni1_force_feedback.csv', 'marker_visualization.csv', 
	'stiffness_command.csv', 'virtual_robot_force.csv', 'eigen_pair.csv', 'omni_stiffness.csv', 
	'omni1_lock_state.csv', 'draw_ellipsoidellipsoid_visualization.csv', 
	'covariance_matrix.csv', 'omni1_button.csv', 'omni_rotation_tf.csv', 'wrist_ft_tool_link_tf.csv', 
	'virtual_marker_tf.csv']

	PI = PlotTopicInfo()

	for topic in topics:
		PI = PlotTopicInfo(topic)
		print(PI.info)
		print(10*"-----")