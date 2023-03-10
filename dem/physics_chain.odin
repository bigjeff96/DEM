package dem

import "core:math/rand"
import "core:fmt"
import "core:os"
import "core:math"
import "core:slice"
import "core:math/linalg"
import "core:intrinsics"

Chain :: struct {
    spheres:           []Sphere,
    chain_direction_q: linalg.Quaternionf64,
}

init_chain :: proc(
    walls: []Wall,
    radius_for_spheres, density: f64,
    spheres: []Sphere,
    chains_to_check_collision: []Chain,
    bending_exp: bool = false,
) -> Chain {
    chains_to_check_collision := chains_to_check_collision
    chain: Chain
    outer_loop: for true {
	position_seed, chain_direction: vec3

	if !bending_exp {
	    chain_direction = walls[SIDE_WALL_ID].normal
	    position_seed = 0
	} else {
	    position_seed, chain_direction = seed_position_and_orientation(walls, radius_for_spheres, len(spheres))
	}

	chain_direction_q: linalg.Quaternionf64 = quaternion(0., chain_direction.x, chain_direction.y, chain_direction.z)
	// physical constants
	spheres := spheres
	for sphere in &spheres {
	    using sphere
	    radius = radius_for_spheres
	    mass = density * (4. / 3.) * math.PI * math.pow(radius, 3)
	    moment_of_inertia = mass * radius * radius * (2. / 5.)
	    orientation = 1 // identity quaternion
	    velocity = 0
	    position = 0
	    angular_velocity = 0
	}
	chain.spheres = spheres
	chain.chain_direction_q = chain_direction_q

	for sphere, id in &chain.spheres {
	    using sphere
	    if id == 0 do position = position_seed
	    else do position = chain.spheres[id - 1].position + chain_direction * (2.0 * radius)
	}

	is_colliding := false
	for other_chain in &chains_to_check_collision {
	    if collision_chains(&chain, &other_chain) {
		is_colliding = true
		break
	    }
	}

	if !is_colliding do break outer_loop
    }

    return chain
}

chain_internal_forces :: proc(chain: Chain, spheres_in_chain: []Sphere, params: Params) {
    spheres := spheres_in_chain
    alignment_axis_q := chain.chain_direction_q
    assert(alignment_axis_q != 1)

    for i: int = 0; i < len(spheres) - 1; i += 1 {
	sphere_a := &spheres[i]
	sphere_b := &spheres[i + 1]
	chain_axis_a_q := sphere_a.orientation * alignment_axis_q * conj(sphere_a.orientation)
	chain_axis_b_q := sphere_b.orientation * alignment_axis_q * conj(sphere_b.orientation)
	chain_axis_a := vec3_from_quat(chain_axis_a_q)
	chain_axis_b := vec3_from_quat(chain_axis_b_q)

	surface_to_surface_vec :=
	    sphere_a.position + chain_axis_a * sphere_a.radius - (sphere_b.position - chain_axis_b * sphere_b.radius)
	// normal forces in chain
	normal: vec3
	if length_squared(surface_to_surface_vec) > 0. do normal = normalize(surface_to_surface_vec)
	else do normal = -chain_axis_a
	
	surface_slip_velocity :=
	    sphere_a.velocity +
	    cross(sphere_a.angular_velocity, sphere_a.radius * chain_axis_a) -
	    (sphere_b.velocity + cross(sphere_b.angular_velocity, -chain_axis_b * sphere_b.radius))

	normal_force := -100 * surface_to_surface_vec - 0.1 * dot(surface_slip_velocity, normal) * normal
	torque_normal_force_a := cross(chain_axis_a * sphere_a.radius, normal_force)
	torque_normal_force_b := cross(-chain_axis_b * sphere_b.radius, -normal_force)
	sphere_a.force += normal_force
	sphere_b.force -= normal_force
	sphere_a.torque += torque_normal_force_a
	sphere_b.torque += torque_normal_force_b

	// shear forces
	quat_a_to_b: linalg.Quaternionf64 = sphere_b.orientation * linalg.quaternion_inverse(sphere_a.orientation)
	if abs(quat_a_to_b) > 0. {
	    angle, axis, ok := angle_axis_from_quat(quat_a_to_b)
	    shear_torque: vec3
	    if ok {
		// this seems to contribute to the bending of the chain
		viscosity_term := 0.00000025 * dot(sphere_a.angular_velocity - sphere_b.angular_velocity, auto_cast axis)
		shear_torque = auto_cast 0.005 * angle * (auto_cast axis) - viscosity_term * axis
	    } else {
		// what axis to use in this case ?
		viscosity : vec3 = 0.00000025 * (sphere_a.angular_velocity - sphere_b.angular_velocity)
		shear_torque =  - viscosity
	    }
	    sphere_a.torque += shear_torque
	    sphere_b.torque -= shear_torque
	}
    }
}

seed_position_and_orientation :: proc(
    walls: []Wall,
    radius_sphere: f64,
    total_spheres_in_chain: int,
) -> (
    seed_position, chain_orientation: vec3,
) {

    length_box := get_length_box(walls)
    collision_wall := true

    for collision_wall {
	seed_position = vec3{
	    rand.float64_range(-length_box.x / 2 + radius_sphere, length_box.x / 2 - radius_sphere),
	    rand.float64_range(-length_box.y / 2 + radius_sphere, length_box.y / 2 - radius_sphere),
	    rand.float64_range(-length_box.z / 2 + radius_sphere, length_box.z / 2 - radius_sphere),
	}
	chain_orientation = random_unit_vector()
	last_sphere_position := seed_position + f64(total_spheres_in_chain - 1) * (2 * radius_sphere) * chain_orientation

	check_collision := false
	for wall in walls {
	    delta := dot(last_sphere_position - wall.center_position, wall.normal) - radius_sphere
	    if delta < 0 {
		check_collision = true
		break
	    }
	}
	collision_wall = true if check_collision else false
    }

    return seed_position, chain_orientation
}

collision_chains :: proc(a, b: ^Chain) -> bool {
    spheres_a := a.spheres
    spheres_b := b.spheres

    result: bool = false
    outer_loop: for sphere_a in &spheres_a {
	for sphere_b in &spheres_b {
	    delta := measure_delta(&sphere_a, &sphere_b)
	    if delta < 0. {
		result = true
		break outer_loop
	    }
	}
    }

    return result
}

vec_is_nan :: #force_inline proc(vec: $T/[3]$E) -> bool where intrinsics.type_is_float(E) {
    result := false
    for element in vec {
	if math.is_nan(element) {
	    result = true
	    break
	}
    }
    return result
}

vec3_from_quat :: #force_inline proc(q: linalg.Quaternionf64) -> vec3 {
    result := vec3{imag(q), jmag(q), kmag(q)}
    return result
}


angle_axis_from_quat :: proc(q: linalg.Quaternionf64) -> (angle: f64, axis: vec3, ok: bool) {
    if 1 - q.w * q.w <= 0 {
	ok = false
	return
    } else {
    angle = linalg.angle_from_quaternion_f64(q)
    s := math.sqrt_f64(1 - q.w * q.w)
	axis.x = q.x / s
	axis.y = q.y / s
	axis.z = q.z / s
	ok = true
    }

    return
}
