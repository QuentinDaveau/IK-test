extends Node


"""
Cleaner IK class
"""


export(int) var _max_resolves: int = 4 # A resolve is a forward and backward pass
export(float) var _min_accepted_dist: float = 0.1
export(NodePath) var _target_path

# Array of IkJoint
export(Array, Resource) var _joints

var _target: Spatial

var _joints_resolvers := []
var _joints_distances := []



func _ready() -> void:
	_setup_joints()



func _process(delta: float) -> void:
	_resolve_ik()



func _resolve_ik() -> void:
	_joints_resolvers[0].position = _joints_resolvers[1].position
	_refresh_resolvers()
	# Resolving the IK
	for i in range(_max_resolves):
		if _ik_pass():
			break
	_apply_to_joints()



func _refresh_resolvers() -> void:
	for resolver in _joints_resolvers:
		resolver.refresh_position()


# TODO: Could get rid of distance array for the sake of clarity (can be integrated in resolvers)
func _ik_pass() -> bool:
	# Forward pass
	for j in range(_joints_resolvers.size() - 2, 0, -1):
		_joints_resolvers[j].resolve_position(_joints_resolvers[j + 1], _joints_distances[j + 1])
	# Backward pass
	for j in range(1, _joints_resolvers.size() - 1):
		_joints_resolvers[j].resolve_position(_joints_resolvers[j - 1], _joints_distances[j])
	# Return wether we are close enough to the target or not
	return _joints_resolvers[-2].position.distance_to(_joints_resolvers[-1].position) <= _min_accepted_dist



func _apply_to_joints() -> void:
	for r in range(1, _joints_resolvers.size() - 2):
		_joints_resolvers[r].set_transform(_joints_resolvers[r + 1].position)





func _setup_joints() -> void:
	_target = get_node(_target_path)
	
	# Getting the joints positions, and adding the root and target positions as well
	# So the only 'true' values that we will use are from _joints_resolvers[1] to _joints_resolvers[-2]
	for joint in _joints:
		_joints_resolvers.append(JointResolver.new(joint, get_node(joint.target_transform)))
	
	_joints_distances.clear()
	for i in range(1, _joints_resolvers.size()):
		_joints_distances.append(_joints_resolvers[i].position.distance_to(_joints_resolvers[i - 1].position))
	
	# Adding extra distances for respectively target (x2) and root bones
	_joints_distances.push_front(0)
	_joints_distances.push_front(0)
	_joints_distances.append(0)
	
	# Adding dummy root and target joints
	_joints_resolvers.push_front(JointResolver.new(_joints[0], get_node(_joints[0].target_transform)))
	var dummy_target_joint = IkJoint.new()
	dummy_target_joint.target_transform = _target_path
	_joints_resolvers.append(JointResolver.new(dummy_target_joint, _target))



class JointResolver:
	var _data: IkJoint
	var _joint: Spatial
	var _constraint: Vector3
	
	# Accessing directly position instead of passing by a func helps a lot to gain ms
	var position: Vector3
	var _forward: Vector3
	
	
	func _init(data: IkJoint, joint: Spatial) -> void:
		_data = data
		_joint = joint
		refresh_position()
		_set_constraint()
	
	
	func refresh_position() -> void:
		position = _joint.global_transform.origin
	
	
	func resolve_position(parent_joint: JointResolver, distance: float) -> void:
		_forward = parent_joint.resolve_constraint(position)
		position = parent_joint.position + _forward * distance
	
	
	# TODO: Find a cleaner way for the basis
	func set_transform(lookat_target: Vector3) -> void:
		_joint.global_transform.origin = position
		_joint.global_transform.basis = _joint.global_transform.looking_at(lookat_target, Vector3.UP).basis
	
	
	func resolve_constraint(source_position: Vector3) -> Vector3:
		var direction := position.direction_to(source_position)
		if not _forward or not direction or not _constraint:
			return direction
		else:
			return _clamp_direction(direction)
	
	
	func _clamp_direction(direction: Vector3) -> Vector3:
#		var rot_to_apply := Basis(-_forward.cross(direction).normalized(), -_forward.angle_to(direction))
#		var applied_vector := Vector3.BACK
#		applied_vector.x = rot_to_apply.z.x if abs(rot_to_apply.z.x) < _constraint.x else _constraint.x * sign(rot_to_apply.z.x)
#		applied_vector.y = rot_to_apply.z.y if abs(rot_to_apply.z.y) < _constraint.y else _constraint.y * sign(rot_to_apply.z.y)
#		var right := Vector3.UP.cross(applied_vector).normalized()
#		return Basis(right, applied_vector.cross(right), applied_vector).xform(_forward)
		if _forward.dot(direction) > cos(_constraint.x):
			return direction
		return _forward.rotated(_forward.cross(direction).normalized(), _constraint.x)
	
	
	func _set_constraint() -> void:
		if _data.constraints:
			_constraint.x = PI * _data.constraints.x / 180
#			_constraint = Basis(PI * _data.constraints / 180).z
#			_constraint.x = abs(_constraint.x)
#			_constraint.y = abs(_constraint.y)
#			_constraint.z = abs(_constraint.z)
		

