extends Node


"""
Very basic IK class with simple, one-axis constraints.
"""


export(int) var _max_resolves: int = 4 # A resolve is a forward and backward pass
export(float) var _min_accepted_dist: float = 0.1
export(NodePath) var _target_path

# Array of IkJoint
export(Array, Resource) var _joints

var _joints_resolvers := []



func _ready() -> void:
	_setup_joints()



func _process(delta: float) -> void:
	_resolve_ik()



func _resolve_ik() -> void:
	_refresh_resolvers()
	# Resolving the IK
	for i in range(_max_resolves):
		if _ik_pass():
			break
	_apply_to_joints()



func _refresh_resolvers() -> void:
	for resolver in _joints_resolvers:
		resolver.refresh_position()


# Resolves the IK. Returns true if close enough to target
func _ik_pass() -> bool:
	# Forward pass
	for j in range(_joints_resolvers.size() - 2, 0, -1):
		_joints_resolvers[j].resolve_position(_joints_resolvers[j + 1], true)
	# Backward pass
	for j in range(1, _joints_resolvers.size() ):
		_joints_resolvers[j].resolve_position(_joints_resolvers[j - 1], false)
	# Return wether we are close enough to the target or not
	return _joints_resolvers[-2].position.distance_to(_joints_resolvers[-1].position) <= _min_accepted_dist



func _apply_to_joints() -> void:
	for r in range(1, _joints_resolvers.size() - 1):
		_joints_resolvers[r].set_transform(_joints_resolvers[r + 1].forward)



func _setup_joints() -> void:
	var target: Spatial = get_node(_target_path)
	
	# Creating the joint resolvers, and setting their bone dist
	for i in range(_joints.size()):
		var joint := get_node(_joints[i].target_transform)
		var dist := 0.0
		if i > 0:
			var parent_pos: Vector3 = get_node(_joints[i - 1].target_transform).global_transform.origin
			dist = joint.global_transform.origin.distance_to(parent_pos)
		_joints_resolvers.append(JointResolver.new(joint, dist, _get_rads(_joints[i].constraint)))
	
	# Adding dummy root and target joints. Having a quasi-null constraint allows to correctly apply constraint on the first and last bone
	_joints_resolvers.push_front(JointResolver.new(get_node(_joints[0].target_transform), 0.0, 0.1))
	_joints_resolvers.append(JointResolver.new(target, 0.0, 0.1))



func _get_rads(degrees: float) -> float:
	return PI * degrees / 180.0




# Class dedicated to do the whole joint resolvine part. There is one per joint, plus two dummies
class JointResolver:
	var _joint: Spatial
	
	# Accessing directly position and distance instead of passing by a func helps a lot to gain ms
	var position: Vector3
	var forward: Vector3
	var bone_distance: float
	
	var _constraint: float
	
	
	func _init(joint: Spatial, distance: float, constraint: float) -> void:
		_joint = joint
		bone_distance = distance
		_constraint = constraint
		forward = -joint.global_transform.basis.z
		refresh_position()
	
	
	func refresh_position() -> void:
		position = _joint.global_transform.origin
	
	
	func resolve_position(parent_joint: JointResolver, use_parent_bone: bool) -> void:
		var distance := parent_joint.bone_distance if use_parent_bone else bone_distance
		forward = parent_joint.resolve_constraint(position)
		position = parent_joint.position + forward * distance
	
	
	func set_transform(forward: Vector3) -> void:
		_joint.global_transform.origin = position
		_joint.global_transform.basis = _joint.global_transform.looking_at(position + forward, Vector3.UP).basis
	
	
	func resolve_constraint(source_position: Vector3) -> Vector3:
		var direction := position.direction_to(source_position)
		if not _constraint or not direction:
			return direction
		else:
			return _clamp_direction(direction)
	
	
	func _clamp_direction(direction: Vector3) -> Vector3:
		if forward.dot(direction) > cos(_constraint):
			return direction
		return forward.rotated(forward.cross(direction).normalized(), _constraint)

