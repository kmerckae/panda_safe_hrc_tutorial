import math
import sys
import json
import numpy as np
from geometry_msgs import msg as gmsg
import tf2_geometry_msgs
from collections import OrderedDict
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def toDeg(rot):
	return rot*180/math.pi

def toRad(rot):
	return rot*math.pi/180

class point:

	def __init__(self, x, y, z):

		self.p = np.array([[x],[y],[z]])

	def get_p(self):
		return self.p

	def get_x(self):
		return self.p[0][0]

	def get_y(self):
		return self.p[1][0]

	def get_z(self):
		return self.p[2][0]

	def __iadd__(self, p):
		self.p[0][0] += p.get_x()
		self.p[1][0] += p.get_y()
		self.p[2][0] += p.get_z()
		return self

	def __add__(self, p):
		return point(self.get_x() + p.get_x(),
		             self.get_y() + p.get_y(),
		             self.get_z() + p.get_z())

	def __neg__(self):
		return point(-self.get_x(),
		             -self.get_y(),
		             -self.get_z())

	def __truediv__(self, n):
		return point(self.get_x() / n, self.get_y() / n, self.get_z() / n)

	def __str__(self):
		return str(self.get_x()) + ' ' + str(self.get_y()) + ' ' + str(self.get_z())

	def rotate(self, center, x_rot, y_rot, z_rot):
		self.translate(-center)
		self.p = np.dot(np.dot(np.dot(matrice('z', z_rot).get_mat() ,
		                              matrice('y', y_rot).get_mat()),
		                              matrice('x', x_rot).get_mat()),
		                              self.p)
		self.translate(center)


	def translate(self, p):
		self += p


class object:

	def __init__(self, name, p, l, w, h, x_rot, y_rot, z_rot):

		self.name = name
		self.p = p
		self.l = l
		self.w = w
		self.h = h
		self.x_rot = toRad(x_rot)
		self.y_rot = toRad(y_rot)
		self.z_rot = toRad(z_rot)

	def get_p(self):
		return self.p

	def get_elements(self):

		return (self.name,
		        self.p.get_x(),
		        self.p.get_y(),
		        self.p.get_z(),
		        self.l,
		        self.w,
		        self.h,
		        toDeg(self.x_rot),
		        toDeg(self.y_rot),
		        toDeg(self.z_rot))

	def transform(self, tf):

		pose_stamped = gmsg.PoseStamped()
		q = quaternion_from_euler(self.x_rot, self.y_rot, self.z_rot)
		pose_stamped.pose.orientation.x= q[0]
		pose_stamped.pose.orientation.y= q[1]
		pose_stamped.pose.orientation.z= q[2]
		pose_stamped.pose.orientation.w= q[3]
		pose_stamped.pose.position.x = self.p.get_x()
		pose_stamped.pose.position.y = self.p.get_y()
		pose_stamped.pose.position.z = self.p.get_z()

		pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, tf)
		self.p = point(pose_transformed.pose.position.x, pose_transformed.pose.position.y, pose_transformed.pose.position.z)
		angle = euler_from_quaternion([pose_transformed.pose.orientation.x, pose_transformed.pose.orientation.y, pose_transformed.pose.orientation.z, pose_transformed.pose.orientation.w])
		self.x_rot = angle[0]
		self.y_rot = angle[1]
		self.z_rot = angle[2]

def read_from(name_file):
	objects = []
	with open(name_file) as json_file:
		data = json.load(json_file)
		for o in data['objects']:
			objects.append(object(o['name'],
			                point(o['centroid']['x'],
			                      o['centroid']['y'],
			                      o['centroid']['z']),
			                      o['dimensions']['length'],
			                      o['dimensions']['width'],
			                      o['dimensions']['height'],
			                      o['rotations']['x'],
			                      o['rotations']['y'],
			                      o['rotations']['z']))
	return objects

def transformation(objects, transformation_file):
	new_objects = []
	tf = gmsg.TransformStamped()
	with open(transformation_file) as json_file:
		data = json.load(json_file)
		tf.transform.translation.x = data['translation']['x']
		tf.transform.translation.y = data['translation']['y']
		tf.transform.translation.z = data['translation']['z']
		tf.transform.rotation.x = data['rotations']['x']
		tf.transform.rotation.y = data['rotations']['y']
		tf.transform.rotation.z = data['rotations']['z']
		tf.transform.rotation.w = data['rotations']['w']
		for object in objects:
			object.transform(tf)
			new_objects.append(object)
	return new_objects

def write_to(objects, label_file, num_file):

	data = []
	for object in objects:
		(name, x, y, z, l, w, h, x_rot, y_rot, z_rot) = object.get_elements()
		print(x, y, z)
		dict = OrderedDict([
		        	("name", name),
		        	("centroid", OrderedDict([
		        		("x", x),
		        		("y", y),
		        		("z", z)
					])),
					("dimensions", OrderedDict([
		        		("length", l),
		        		("width", w),
		        		("height", h)
					])),
					("rotations", OrderedDict([
		        		("x", x_rot),
		        		("y", y_rot),
		        		("z", z_rot)
					]))
				])
		data.append(dict)

	dict = OrderedDict([("folder", "pointclouds"),
	        ("filename", num_file + ".pcd"),
	        ("path", "pointclouds/" + num_file + ".pcd"),
	        ("objects", data)])

	print(dict)



	# Writing to label_file
	with open(label_file, "w") as json_file:
		json.dump(dict, json_file, indent = 4)



if __name__ == "__main__":
	if (len(sys.argv) != 2):
		print("Usage : deduce_labels.py number")
		exit()
	num_file = '%06d'%(int(sys.argv[1]))
	num_next_file = '%06d'%(int(sys.argv[1])+1)
	label_file = 'labels/' + num_file + '.json'
	next_label_file = 'labels/' + num_next_file + '.json'
	transformation_file = 'transformation/' + num_file + '.json'

	objects = read_from(label_file)
	new_objects = transformation(objects, transformation_file)
	write_to(new_objects, next_label_file, num_next_file)
