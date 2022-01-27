extends Resource
class_name IkJoint

"""
Simple data structure to hold properties of an IK joint
"""


export(NodePath) var target_transform

# A weight of 1 means that the joint will fully bend to reach its target per resolve
# A weight of 0 means that the joint will not bend (no movement at all)
export(float, 0, 1) var weight = 1

# Bend angle constraints in euler angles (relative to the forward of the joint)
export(Vector3) var constraints = Vector3.ZERO
