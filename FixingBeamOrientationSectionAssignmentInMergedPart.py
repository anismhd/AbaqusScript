from abaqus import *
from abaqusConstants import *
import sys
import regionToolset
from numpy import zeros, cos, sin, radians, dot, array
SFX = 'amvAutoGenSet_from_'
class TransformationLinearRotation():
	def __init__(self,X0,unit_vector,theta):
		self.RotationMatrix(unit_vector,theta)
		self.X0 = X0
	def __call__(self,x,y,z):
		X = array([x,y,z])
		return tuple(self.X0 + dot(self.R,X.T))
	def rotateOnly(self,x,y,z):
		X = array([x,y,z])
		return tuple(dot(self.R,X.T))
	def RotationMatrix(self,v, t):
		R = zeros([3,3])
		R[0,0] = cos(radians(t)) + (1-cos(radians(t)))*v[0]**2
		R[1,1] = cos(radians(t)) + (1-cos(radians(t)))*v[1]**2
		R[2,2] = cos(radians(t)) + (1-cos(radians(t)))*v[2]**2
		R[0,1] = (1-cos(radians(t)))*v[0]*v[1] - sin(radians(t))*v[2]
		R[1,0] = (1-cos(radians(t)))*v[0]*v[1] + sin(radians(t))*v[2]
		R[0,2] = (1-cos(radians(t)))*v[0]*v[2] + sin(radians(t))*v[1]
		R[2,0] = (1-cos(radians(t)))*v[0]*v[2] - sin(radians(t))*v[1]
		R[1,2] = (1-cos(radians(t)))*v[1]*v[2] - sin(radians(t))*v[0]
		R[2,1] = (1-cos(radians(t)))*v[1]*v[2] + sin(radians(t))*v[0]
		self.R = R
	def edgesPointsOnCordinateTransform(self, pointsOn):
		TraPointsOn = []
		for i,point in enumerate(pointsOn):
			temp = self(point[0][0],point[0][1],point[0][2])
			TraPointsOn.append((temp,))
		return tuple(TraPointsOn)
def assign_beam_orientations(part,edges,TN1):
	TraPointsOn = transformation_class[part].edgesPointsOnCordinateTransform(edges.pointsOn)
	for point in TraPointsOn:
		myRegion = regionToolset.Region(edges=\
			mymodel.parts[merged_part_name].edges.findAt(point))
		mymodel.parts[merged_part_name].assignBeamSectionOrientation(\
				region=myRegion,\
				method=N1_COSINES,\
				n1=(TN1[0], TN1[1], TN1[2]))
def assign_beam_sections(part,edges,sectionName1):
	TraPointsOn = transformation_class[part].edgesPointsOnCordinateTransform(edges.pointsOn)
	for point in TraPointsOn:
		myRegion = regionToolset.Region(edges=\
			mymodel.parts[merged_part_name].edges.findAt(point))
		mymodel.parts[merged_part_name].SectionAssignment(\
				region=myRegion,\
				sectionName=sectionName1)
model_name = 'Model-1'
merged_part_name = 'MERGED_ALL'
merging_parts = ['RAFT-1','building wall-slab-1', 'truss and col-1']
# Opening model
mymodel = mdb.models[model_name]
# Checking all the parts
if not(merged_part_name in mymodel.parts.keys()):
	print('No part with name {0:s} exists in the model data base. Check and renter the correct part number.'.format(merged_part_name))
#	sys.exit()
for part in merging_parts:
	if not(part in mymodel.rootAssembly.instances.keys()):
		print('No part with name {0:s} exists in the model data base. Check and renter the correct part number.'.format(part))
#		sys.exit()
print('All the part names seems to be OK....')
# Identfy all Beam Sections in the model
all_sections = mymodel.sections.keys()
beam_sections_keys = []
for sec in all_sections:
	if type(mymodel.sections[sec]).__name__ == 'BeamSection':
		beam_sections_keys.append(sec)
# Listing all the Beam Sections in individual parts
# for part in merging_parts:
	# for sectionAssign in mymodel.parts[part].sectionAssignments:
		# print(sectionAssign.sectionName,sectionAssign.region)

# for sectionAssign in mymodel.parts[merged_part_name].sectionAssignments:
	# if sectionAssign.sectionName in beam_sections_keys:
		# setname = sectionAssign.region[0]
		# mymodel.parts[merged_part_name].assignBeamSectionOrientation(\
			# region=mymodel.parts[merged_part_name].allInternalSets[setname],\
			# method=N1_COSINES,\
			# n1=(1.0, 1.0, 1.0))
# Transformation matrix for all instances
transformation_class = {}
for part in merging_parts:
	T = mymodel.rootAssembly.instances[part].getTranslation()
	R = mymodel.rootAssembly.instances[part].getRotation()
	transformation_class[part] = TransformationLinearRotation(T,R[1],R[2])
# mapping the edges of meging parts to merged part
'''
# This no longer required as an faster alternative is found...
edge_map ={}
for part in merging_parts:
	partName = mymodel.rootAssembly.instances[part].partName
	edge_map[part] = {}
	for edge in mymodel.parts[partName].edges:
		A = mymodel.parts[merged_part_name].edges.findAt(transformation_class[part](edge.pointOn[0][0],\
																					edge.pointOn[0][1],\
																					edge.pointOn[0][2]))
		if A:
			edge_map[part][edge.index] = A.index
		else:
			edge_map[part][edge.index] = None
'''
# Assigning Beam Section
for part in merging_parts:
	partName = mymodel.rootAssembly.instances[part].partName
	for sectionAssign in mymodel.parts[partName].sectionAssignments:
		if sectionAssign.sectionName in beam_sections_keys:
			setName = sectionAssign.region[0]
			if setName in mymodel.parts[partName].allInternalSets.keys():
				assign_beam_sections(part,\
					mymodel.parts[partName].allInternalSets[setName].edges,sectionAssign.sectionName)
			elif setName in mymodel.parts[partName].alllSets.keys():
				assign_beam_sections(part,\
					mymodel.parts[partName].allSets[setName].edges,sectionAssign.sectionName1)
			else:
				print('Region/sets not found..')
# Finding beam orientation assignment sets
for part in merging_parts:
	partName = mymodel.rootAssembly.instances[part].partName
	for beamOrnt in mymodel.parts[partName].beamSectionOrientations:
		setName = beamOrnt.region[0]
		TN1 = transformation_class[part].rotateOnly(beamOrnt.n1[0],beamOrnt.n1[1],beamOrnt.n1[2])
		if setName in mymodel.parts[partName].allInternalSets.keys():
			assign_beam_orientations(part,\
				mymodel.parts[partName].allInternalSets[setName].edges,TN1)
		elif setName in mymodel.parts[partName].allSets.keys():
			assign_beam_orientations(part,\
				mymodel.parts[partName].allSets[setName].edges,TN1)
		else:
			print('Region/sets not found..')