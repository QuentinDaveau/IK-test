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

var _joints_spatials := []
var _joints_distances := []



func _ready() -> void:
	_setup_joints()



func _process(delta: float) -> void:
	_resolve_ik()



func _resolve_ik() -> void:
	# Getting the joints positions, and adding the root and target positions as well
	# So the only 'true' values that we will use are from joints[1] to joints[-2]
	var joints_pos := [_joints_spatials[0].global_transform.origin]
	for joint in _joints_spatials:
		joints_pos.append(joint.global_transform.origin)
	joints_pos.append(_target.global_transform.origin)
	
	# Resolving the IK
	for i in range(_max_resolves):
		_forward_pass(joints_pos)
		_backward_pass(joints_pos)
		if _is_close_to_target(joints_pos):
			break
	_apply_to_joints(joints_pos)



func _forward_pass(var positions: Array) -> void:
	for j in range(positions.size() - 2, 0, -1):
		var dir: Vector3 = positions[j + 1].direction_to(positions[j])
		positions[j] = positions[j + 1] + dir * _joints_distances[j + 1]



func _backward_pass(var positions: Array) -> void:
	for j in range(1, positions.size() - 1):
		var dir: Vector3 = positions[j - 1].direction_to(positions[j])
		positions[j] = positions[j - 1] + dir * _joints_distances[j]



func _is_close_to_target(var positions: Array) -> bool:
	return positions[-2].distance_to(positions[-1]) <= _min_accepted_dist



func _apply_to_joints(var positions: Array) -> void:
	for r in range(1, positions.size() - 2):
		_joints_spatials[r - 1].global_transform.origin = positions[r]
		_joints_spatials[r - 1].global_transform.basis = _joints_spatials[r - 1].global_transform.looking_at(positions[r + 1], Vector3.UP).basis




func _setup_joints() -> void:
	_target = get_node(_target_path)
	
	for joint in _joints:
		_joints_spatials.append(get_node(joint.target_transform))
	
	_joints_distances.clear()
	for i in range(1, _joints_spatials.size()):
		_joints_distances.append(_joints_spatials[i].global_transform.origin.distance_to(_joints_spatials[i - 1].global_transform.origin))
	
	# Adding extra distances for respectively target (x2) and root bones
	_joints_distances.push_front(0)
	_joints_distances.push_front(0)
	_joints_distances.append(0)



class JointData:
	var _joint: Spatial
	var _data: IkJoint
	
	var _position
	var _previous_position
	
	
	func refresh_position() -> void:
		_position = _joint.global_transform.origin
	
	
	func set_position(var position: Vector3) -> void:
		_previous_position = _position
		_position = position
	


