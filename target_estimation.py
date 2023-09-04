#!/usr/bin/env python3

# ros includes
import rospy
import actionlib
from multiexpl_msgs.msg import MultiExplAction
from multiexpl_msgs.msg import MultiExplGoal

from multiexpl_msgs.msg import Target



from nav_msgs.msg import OccupancyGrid, Odometry
from std_srvs.srv import Empty, EmptyResponse

# other
import sys


import numpy as np
np.set_printoptions(threshold=sys.maxsize)
from typing import final


class targetFinder:
	"""docstring for targetFinder"""
	def __init__(self, resolution=0.05, target_window=8, robot_dimensions=[0.2, 0.2]):

		# assert isinstance(target_window, (int)), "Target window must be integer, Recieved type %r" % type(target_window).__name__

		assert round(robot_dimensions[0] * robot_dimensions[1], 2) <= round((resolution * target_window) ** 2, 2), "Robot size(area) must not be bigger than target window(area), given robot size is %r m*m, while target window is %r m*m" %(robot_dimensions[0] * robot_dimensions[1], (resolution * target_window) ** 2)
		assert target_window >= 8, "Target window must be greater than or equal to 9 (0.4*0.4 m*m), Recieved %r" % target_window
		# assert target_window % 2 != 0, "Target window must be an odd number, Recieved %r" % target_window

		self.resolution = resolution					# Size of each grid in meters
		self.robot_max_length = robot_dimensions[0]		# Minimum length of the robot in meters
		self.robot_max_width = robot_dimensions[1]		# Maximum width of the robot in meters
		self.target_window = target_window				# size of the target window in pixel (to convert meters => target_window * resolution), note that it is a square and must always be an odd number
		self.origin_x = 0
		self.origin_y = 0

		rospy.Subscriber("map", OccupancyGrid, self.map_data_callback)
		self.pos_0 = np.zeros([2])
		self.pos_1 = np.zeros([2])
		self.pos_2 = np.zeros([2])
		rospy.Subscriber("tb3_0/odom", Odometry, self.PositionCallback_0)
		rospy.Subscriber("tb3_1/odom", Odometry, self.PositionCallback_1)
		rospy.Subscriber("tb3_2/odom", Odometry, self.PositionCallback_2)


	def map_data_callback(self, msg):
		self.origin_x = msg.info.origin.position.x
		self.origin_y = msg.info.origin.position.y

	def PositionCallback_0(self,msg):
		self.pos_0[0] = msg.pose.pose.position.x
		self.pos_0[1] = msg.pose.pose.position.y

	def PositionCallback_1(self,msg):
		self.pos_1[0] = msg.pose.pose.position.x
		self.pos_1[1] = msg.pose.pose.position.y

	def PositionCallback_2(self,msg):
		self.pos_2[0] = msg.pose.pose.position.x
		self.pos_2[1] = msg.pose.pose.position.y

	def get_targets(self, clearance, frontier_clearance, maps, width, height):
		"""
		This function finds out all potential targets in the map
		If a point is near to a frontier while also being far away enough from a wall, then a region around the point that is far enough from the frontier itself
		is chosen at the potential target

		Parameters
			The function only needs a map

		Returns
			It returns the start and end points of all the regions that can be chosen as targets
		"""

		assert isinstance(frontier_clearance, (int)), "Frontier clearance must be integer, Recieved type %r" % type(frontier_clearance).__name__
		assert frontier_clearance >= 2, "Frontier Clearance must be greater than or equal to 2, Recieved frontier clearance is %r" % frontier_clearance
		# assert clearance > self.resolution, "Clearance must not be less than resolution, Recieved clearance is %r, and resolution is %r" % (clearance, self.resolution)

		maps = maps[0]
		# print(np.shape(maps),type(maps))
		potential_targets = []

		clearance = clearance / self.resolution
		cut_off = clearance // 2
		frontier_window = self.target_window + frontier_clearance + clearance

		for i in range(width):
			for j in range(height):
				if maps[i][j] == 0:
					[neighbor_map, x_s, y_s, x_e, y_e] = self.neighbors(frontier_window, i, j, maps, width, height)
					#print("neighbor_map",neighbor_map)
					if not any(1 in n for n in neighbor_map):  # Check if there are any obstacles nearby
						if any(-1 in n for n in neighbor_map):  # Check if there are any frontiers nearby
							[target_map, x_start, y_start, x_end, y_end] = self.neighbors(self.target_window + clearance, i, j, maps, width ,height)
							if not any(-1 in n for n in target_map):  # Make sure there are no frontiers in the target
								potential_targets.append([[x_start + cut_off, y_start + cut_off], [x_end - cut_off, y_end - cut_off]])
		# print(np.array(potential_targets),np.shape(potential_targets))

		return potential_targets


	def neighbors(self, radius, row_number, column_number, submaps, width, height):
		"""
		This function finds (radius/2) number of neighbors of given point on all four sides

		Parameters
			radius: how many neighbors do you want to obtain either horizontally or vertically
			row_number: x coordinate of the point
			column_number: y coordinate of the point
			submaps: The part of the map within which we find the neighbors

		Returns
			The function returns the neighbors and the x and y coordinates of the starting and ending element of the neighbors array with respect to the submap coordinate
		"""

		radius = int(radius // 2)
		row_number = int(row_number)
		column_number = int(column_number)

		# If the selected has neighbors all around it within the map
		if row_number < width - radius and column_number < height - radius and row_number > radius and column_number > radius:
			return submaps[row_number - radius : row_number + radius, column_number - radius : column_number + radius], row_number - radius, column_number - radius, row_number + radius, column_number + radius
		else:
			return [[]], row_number - radius, column_number - radius, row_number + radius, column_number + radius





	def rank_targets(self,potential_targets,map_array):
		unexpl_array = []
		robot_radius = 0.22/0.05
		for i in range(len(potential_targets)):
			point1 = potential_targets[i][0]
			point2 = potential_targets[i][1]
			# print(point1[0],point2[0])
			num_unexpl = 0
			for a in range(int(point1[0]+robot_radius),int(point2[0]+robot_radius)):
				for b in range(int(point1[1]+robot_radius),int(point2[1]+robot_radius)):
					# print(map_array[a][b])
					if map_array[a][b]==-1:
						# print("here")

						num_unexpl+=1
			unexpl_array.append(num_unexpl)
		# print(unexpl_array)

		sorted_list = list(unexpl_array)
		sorted_list.sort()
		rank_targets = []

		for i in range(len(potential_targets)):
			max_unexpl = sorted_list[i]
			index= unexpl_array.index(max_unexpl)
			rank_targets.append((potential_targets[index]))

		return rank_targets

	def best_targets(self,rank_targets,safety_net,maps,width,height):
		best_targets = []


		maps = maps[0]
		for i in range(len(rank_targets)):
				[check_neighbor, x_s, y_s, x_e, y_e] = self.neighbors(self.target_window+safety_net, rank_targets[i][0][0]+self.target_window//2, rank_targets[i][0][1]+self.target_window//2, maps, width, height)

				best_targets.append([[x_s+safety_net//2, y_s+safety_net//2], [x_e-safety_net//2, y_e-safety_net//2]])
				# break

		
		return best_targets

	def split_targets(self, safe_targets):
		first_bot_targets = []
		second_bot_targets = []
		third_bot_targets = []

		# print(self.origin_x,self.origin_y)
		self.pos_0=[(int((self.pos_0[0]-self.origin_x)/0.05)),(int((self.pos_0[1]-self.origin_y)/0.05))]
		self.pos_1=[(int((self.pos_1[0]-self.origin_x)/0.05)),(int((self.pos_1[1]-self.origin_y)/0.05))]
		self.pos_2=[(int((self.pos_2[0]-self.origin_x)/0.05)),(int((self.pos_2[1]-self.origin_y)/0.05))]
		robot_location=np.array([self.pos_0,self.pos_1,self.pos_2])
		for i in range(int((len(safe_targets)))):
			mid_point = [(safe_targets[i][0][0] + safe_targets[i][1][0])/2,(safe_targets[i][0][1] + safe_targets[i][1][1])/2]
			dist_1 = np.linalg.norm(robot_location[0]-mid_point)
			dist_2 = np.linalg.norm(robot_location[1]-mid_point)
			dist_3 = np.linalg.norm(robot_location[2]-mid_point)
			dist = min(dist_1,dist_2,dist_3)
			if dist==dist_1:
				first_bot_targets.append(safe_targets[i])
			elif dist == dist_2:
				second_bot_targets.append(safe_targets[i])
			elif dist==dist_3:
				third_bot_targets.append(safe_targets[i])
		if len(first_bot_targets)==0:
			if len(second_bot_targets)>len(third_bot_targets):
				first_bot_targets = second_bot_targets[3:6]
				del second_bot_targets[3:6]
			else:
				first_bot_targets=third_bot_targets[3:6]
				del first_bot_targets[3:6]
		elif len(second_bot_targets)==0:
			if len(first_bot_targets)>len(third_bot_targets):
				second_bot_targets=first_bot_targets[3:6]
				del first_bot_targets[3:6]
			else:
				second_bot_targets=third_bot_targets[3:6]
				del third_bot_targets[3:6]
		elif len(third_bot_targets)==0:
			if len(second_bot_targets)>len(first_bot_targets):
				third_bot_targets=second_bot_targets[3:6]
				del second_bot_targets[3:6]
			else:
				third_bot_targets=first_bot_targets[3:6]
				del third_bot_targets[3:6]

		# print(first_bot_targets,len(first_bot_targets))
		return first_bot_targets,second_bot_targets,third_bot_targets





class scotsActionClient:
	"""docstring for scotsActionClient"""
	def __init__(self):

		self.total_systhessis_time = 0
		self.total_completion_time = 0

		self._ac = actionlib.SimpleActionClient("/scots", MultiExplAction)

		self._ac.wait_for_server()

		rospy.loginfo("Action Server is Up, starting to send goals.")

	# Function to send Goals to Action Servers
	def send_goal(self, targets):

		# Create Goal message for Simple Action Server
		goal = MultiExplGoal(targets = targets)

		'''object has no attribute 'targets'
			* feedback_cb is set to the function pointer of the function which should be called while
				the goal is being processed by the Simple Action Server.
		'''
		self._ac.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)

		rospy.loginfo("Goal has been sent.")



	def get_time(self):
		return self.total_systhessis_time, self.total_completion_time

	def done_callback(self, status, result):
		self.total_systhessis_time += result.synthesis_time
		self.total_completion_time += result.completion_time

		rospy.loginfo("Target id, {}".format(result.target_id))
		rospy.loginfo("Synthesis Time, {}".format(result.synthesis_time))
		rospy.loginfo("Completion Time,  {}".format(result.completion_time))


	def feedback_callback(self, feedback):
		rospy.loginfo("Current Pose: ({}, {}, {})".format(round(feedback.curr_pose_1.x, 2), round(feedback.curr_pose_1.y, 2), round(feedback.curr_pose_1.theta, 2)))
		rospy.loginfo("Current Pose: ({}, {}, {})".format(round(feedback.curr_pose_2.x, 2), round(feedback.curr_pose_2.y, 2), round(feedback.curr_pose_2.theta, 2)))
		rospy.loginfo("Current Pose: ({}, {}, {})".format(round(feedback.curr_pose_3.x, 2), round(feedback.curr_pose_3.y, 2), round(feedback.curr_pose_3.theta, 2)))



class mapData:
	"""docstring for mapData"""
	def __init__(self, action_client):
		self.width = 0
		self.height = 0
		self.resolution = 0
		self.maps = list()

		self.action_client = action_client

		self.map_topic_name = "/map"
		self.map_sub_handle = rospy.Subscriber(self.map_topic_name, OccupancyGrid, self.map_data_callback)


		rospy.loginfo("Waiting for data on /map topic..")
		rospy.wait_for_message("map", OccupancyGrid, timeout=10)
		self.if_send_new_goal = False


		self.new_goal_service_name = "/new_goal"
		self.new_goal_service = rospy.Service(self.new_goal_service_name, Empty, self.new_goal_callback)


	def map_data_callback(self, msg):
		self.origin_x = msg.info.origin.position.x
		self.origin_y = msg.info.origin.position.y
		self.width = msg.info.width
		self.height = msg.info.height
		self.resolution = msg.info.resolution

		map_image = np.zeros((self.height, self.width, 1), dtype="int8")
		for i in range(0, self.height):
			for j in range(0, self.width):
				if msg.data[i * self.width + j] > 0 :
					map_image[i][j] = 1
				else:
					map_image[i][j] = int(msg.data[i * self.width + j])
		# print(np.array(map_image),np.shape(map_image))
		self.map_array = np.squeeze(map_image)
		self.map_array = np.transpose(self.map_array)

		self.maps = list(map_image.T)

	def new_goal_callback(self, req):
		rospy.loginfo("Got new goal request.")
		self.if_send_new_goal = True
		return EmptyResponse()


	def send_new_goal(self, targets):
		if(self.if_send_new_goal):
			rospy.loginfo("Sending new goal to action server.")
			self.action_client.send_goal(targets)
			self.if_send_new_goal = False
		else:
			rospy.loginfo("send_new_goal flag is false, goal not sent.")

	def get_map(self):
		return self.maps,self.map_array

	def get_map_dimensions(self):
		return self.width, self.height, self.resolution

	def get_map_origin(self):
		return self.origin_x, self.origin_y



def get_safe_targets(target_finder, clearance, frontier_clearance, safety_net, maps, width, height):
	all_targets = target_finder.get_targets(clearance, frontier_clearance, maps, width, height)
	ranked_targets = target_finder.rank_targets(all_targets,map_array)
	# max_indices = target_finder.rank_targets(maps,all_targets, width, height)
	safe_targets = target_finder.best_targets(all_targets, safety_net, maps, width, height)
	bot1_target,bot2_target,bot3_target = target_finder.split_targets(safe_targets)

	return bot1_target,bot2_target,bot3_target


if __name__ == '__main__':

	rospy.init_node("target_estimation")

	action_client = scotsActionClient()
	mapdata = mapData(action_client)

	clearance = 0.2                   # clearance / grid size must be lower than frontier_clearance
	frontier_clearance = 4
	target_window = 10
	safety_net = 2                    # The distance to be taken from the inflated frontier edge, this too must be even

	_w, _h, resolution = mapdata.get_map_dimensions()

	target_finder = targetFinder(resolution=resolution, target_window=target_window, robot_dimensions=[0.2, 0.2])

	rate = rospy.Rate(1)

	try:
		while not rospy.is_shutdown():
			maps,map_array = mapdata.get_map()
			width, height, resolution = mapdata.get_map_dimensions()

			if(clearance < 0 or frontier_clearance < 2):
				rospy.loginfo("Exploration is done.")
				total_systhessis_time, total_completion_time = action_client.get_time()
				rospy.loginfo("Total time spent on Synthesis. %r" % total_systhessis_time)
				rospy.loginfo("Total time spent on Completion. %r" % total_completion_time)
				break

			targets = []

			start = rospy.Time.now()
			first_bot,second_bot,third_bot = get_safe_targets(target_finder, clearance, frontier_clearance, safety_net, maps, width, height)

			# print("first_bot",first_bot,"third_bot",third_bot,"second_bot",second_bot)
			print(len(first_bot),len(second_bot),len(third_bot))
			end = rospy.Time.now()

			print("Total Time. {}".format(end - start))
			# print(first_bot,len(first_bot))

			if(max(len(first_bot),len(second_bot),len(third_bot)) > 0) :

				for i in range(min(len(first_bot),len(second_bot),len(third_bot))):
						print(i)
						if (i%3==0):
							tr_1 = Target()
							tr_1.id = i
							tr_1.window = round((target_window - 1) * resolution, 2)
							tr_1.clearance = round((frontier_clearance + target_window - 1) * resolution, 2)

							for j in range(2):
									tr_1.points.append(round(first_bot[i][j][0] * resolution, 2))
							for j in range(2):
									tr_1.points.append(round(first_bot[i][j][1] * resolution, 2))

							# print(type(tr_1))
							targets.append(tr_1)
							# print("here1")



						if (i%3==1):
							tr_2 = Target()
							tr_2.id = i
							tr_2.window = round((target_window - 1) * resolution, 2)
							tr_2.clearance = round((frontier_clearance + target_window - 1) * resolution, 2)

							for j in range(2):
									tr_2.points.append(round(second_bot[i][j][0] * resolution, 2))
							for j in range(2):
									tr_2.points.append(round(second_bot[i][j][1] * resolution, 2))

							targets.append(tr_2)
							# print("here2")



						if (i%3==2):
							tr_3 = Target()
							tr_3.id = i
							tr_3.window = round((target_window - 1) * resolution, 2)
							tr_3.clearance = round((frontier_clearance + target_window - 1) * resolution, 2)


							for j in range(2):
									tr_3.points.append(round(third_bot[i][j][0] * resolution, 2))
							for j in range(2):
									tr_3.points.append(round(third_bot[i][j][1] * resolution, 2))
							targets.append(tr_3)
					# print("here3")

			# print("multi_targets",multi_targets_msg.targets[0],"len",len(multi_targets_msg.targets))

			# print(type(multi_targets_msg.targets))
			# print(list(multi_targets_msg.targets))


			if len(targets)>0 and mapdata.if_send_new_goal:
					print("here")
					print("Goal targets, %r" % targets)
					mapdata.send_new_goal(targets)

					# resetting the parameters
					clearance = 0.2
					frontier_clearance = 6
					action_client._ac.wait_for_result()


			else:
				print("No targets.. reducing clearance, frontier_clearance and safety net.")
				clearance -= 0.1
				frontier_clearance -= 2
				safety_net -= 2

			rate.sleep()
	except KeyboardInterrupt:
		pass
