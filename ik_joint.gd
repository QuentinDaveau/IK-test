extends Resource
class_name IkJoint

"""
Simple data structure to hold properties of an IK joint
"""


export(NodePath) var target_transform

# Bend angle constraints in euler angles (relative to the forward of the joint)
export(Vector3) var constraints = Vector3.ZERO
