extends Node

"""
Simple and ugly 3D IK script based on the FABRIK algorithm
"""
# TODO: Add weight and joint constraints



export(int) var _max_resolves: int = 6
export(float) var _min_accepted_dist: float = 0.1

export(Array, NodePath) var _joints_paths: Array = []
export(NodePath) var _target_path

# Those variables are exported to allow to serialize them and joints_pos the IK in the inspector
var _joints: Array = []
var _joints_dists: Array = []
var _target: Spatial



func _ready() -> void:
	_setup_joints()



func _process(delta):
	_process_ik()



func _process_ik() -> void:
	var joints_pos := [_joints[0].global_transform.origin]
	for joint in _joints:
		joints_pos.append(joint.global_transform.origin)
	
	for i in range(_max_resolves):
		var return_pass := i % 2 == 0
		var coeff := 1 if return_pass else -1
		var dist_coeff := 1 if return_pass else 0
		for j in range(joints_pos.size() - 2 if return_pass else 1, 0 if return_pass else joints_pos.size() - 1, -1 if return_pass else 1):
			var dir: Vector3 = joints_pos[j + coeff].direction_to(joints_pos[j])
			joints_pos[j] = joints_pos[j + coeff] + dir * _joints_dists[j + dist_coeff]
		
		if not return_pass and joints_pos[-2].distance_to(_target.global_transform.origin) <= _min_accepted_dist:
			break
		
	_apply_to_transforms(joints_pos)



func _apply_to_transforms(var positions: Array) -> void:
	for r in range(1, positions.size() - 2):
		_joints[r - 1].global_transform.origin = positions[r]
		_joints[r - 1].global_transform.basis = _joints[r - 1].global_transform.looking_at(positions[r + 1], Vector3.UP).basis



# Fetching the joints
func _setup_joints() -> void:
	_joints = []
	for path in _joints_paths:
		_joints.append(get_node(path))
	
	_joints_dists.append(0)
	_joints_dists.append(0)
	for i in range(1, _joints.size()):
		var dist: float = _joints[i - 1].global_transform.origin.distance_to(_joints[i].global_transform.origin)
		_joints_dists.append(dist)
	
	
	# Adding the target to the joints list for the computation
	_target = get_node(_target_path)
	_joints.append(_target)
	_joints_dists.append(0) # For the target





