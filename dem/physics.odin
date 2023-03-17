package dem

import rl "vendor:raylib"
import "core:math/rand"
import "core:fmt"
import "core:math"
import "core:slice"
import "core:log"
import "core:math/linalg"

G: f64 : 9.81

vec3 :: distinct [3]f64
Sphere :: struct {
    position:          vec3,
    velocity:          vec3,
    angular_velocity:  vec3,
    force:             vec3,
    torque:            vec3,
    orientation:       linalg.Quaternionf64,
    mass:              f64,
    moment_of_inertia: f64,
    radius:            f64,
}

// NOTE: this needs to represent both particle-particle collisions and particle-wall collisions
Contact :: struct {
    time_last_update:    f64,
    particle_id:         i32,
    particle_or_wall_id: i32,
    delta_normal:        f64,
    friction_coeff:      f64,
    tangent_spring:      vec3,
    other_is_wall:       bool,
}

Wall :: struct {
    center_position: vec3,
    normal:          vec3,
    velocity:        vec3,
    id:              i32,
}

TOP_WALL_ID :: 3
BOTTOM_WALL_ID :: 2
SIDE_WALL_ID :: 4

physics_update :: proc(
    spheres: []Sphere,
    cell_context: ^Cell_context,
    walls: []Wall,
    contacts: ^map[int]Contact,
    is_periodic: bool,
    params: Params,
) {
    using rl, params
    using cell_context
    // for looping over slices
    spheres := spheres
    walls := walls

    length_box := get_length_box(walls)

    @(static)
    current_time: f64 = 0
    defer current_time += dt

    // clear particles in cells
    for cell in &cells {
        clear(&cell.particle_ids)
    }

    // update position by half a timestep and find cell of particle
    for sphere, sphere_id in &spheres {
        using sphere
        position += velocity * dt / 2
        if is_periodic {
            if position.x < -length_box.x / 2 do position.x = position.x + length_box.x
            if position.x > length_box.x / 2 do position.x = position.x - length_box.x
        }
        e_y := vec3{0., 1., 0.}
        force = -e_y * mass * G
        torque = 0

        id_cell_of_particle := position_to_cell_id(position, cell_context, walls)
        append(&cells[id_cell_of_particle].particle_ids, auto_cast sphere_id)
    }

    // find out new forces based on the new positions
    for sphere, sphere_id in &spheres {
        using sphere

        id_cell_of_particle := position_to_cell_id(position, cell_context, walls)
        cell_of_particle := cells[id_cell_of_particle]

        // For it to work like the linked list ( like a stack, last in first out)
        slice.reverse(cell_of_particle.particle_ids[:])

        // in the same cell
        for id_other in cell_of_particle.particle_ids do if i32(sphere_id) < id_other {
                delta: f64
            if is_periodic do delta = measure_delta_spheres_periodic(&sphere, &spheres[id_other], length_box.x)
	    else do delta = measure_delta(&sphere, &spheres[id_other])
                if delta < 0 {
                    index := generate_hash(sphere_id = auto_cast sphere_id, other_id = id_other, other_is_wall = false)
                    update_contact(index, contacts, delta, current_time, params)
                }
            }

        // in neighbor cell
        using cell_of_particle
        for neighbor_cell in neighbor_cells_iterator(cells, neighbors[:total_neighbors]) {
            for id_other in neighbor_cell.particle_ids {
                delta: f64

                if is_periodic do delta = measure_delta_spheres_periodic(&sphere, &spheres[id_other], length_box.x)
		else do delta = measure_delta(&sphere, &spheres[id_other])

                if delta < 0 {
                    index := generate_hash(sphere_id = auto_cast sphere_id, other_id = id_other, other_is_wall = false)
                    update_contact(index, contacts, delta, current_time, params)
                }
            }
        }

        for wall in &walls {
            delta := measure_delta(&sphere, &wall)
            if is_periodic && delta < 0 && wall.center_position.x == 0 {
                index := generate_hash(sphere_id = auto_cast sphere_id, other_id = wall.id, other_is_wall = true)
                update_contact(index, contacts, delta, current_time, params)
            } else if delta < 0 {
                index := generate_hash(sphere_id = auto_cast sphere_id, other_id = wall.id, other_is_wall = true)
                update_contact(index, contacts, delta, current_time, params)
            }
        }
    }

    update_contact_forces(contacts, current_time, spheres, walls, length_box, params, false)

    // update the velocities and angular velocities
    for sphere in &spheres {
        using sphere
        velocity += force * (dt / mass)
        angular_velocity += torque * (dt / moment_of_inertia)
    }

    // another half timestep update to finish it off
    for sphere in &spheres {
        using sphere
        position += velocity * dt / 2
        if is_periodic {
            if position.x < -length_box.x / 2 do position.x = position.x + length_box.x
            if position.x > length_box.x / 2 do position.x = position.x - length_box.x
        }

        // update the orientation quaternion
        if length_squared(angular_velocity) > 0 {
            new_rotation := linalg.quaternion_angle_axis_f64(
                length(angular_velocity) * dt,
                auto_cast normalize(angular_velocity),
            )
            orientation = new_rotation * orientation
        }
    }
}

physics_update_chain :: proc(
    chains: []Chain,
    spheres: []Sphere,
    cell_context: ^Cell_context,
    walls: []Wall,
    contacts: ^map[int]Contact,
    params: Params,
    is_periodic: bool = false,
) {
    using rl, params
    using cell_context
    chains := chains
    spheres := spheres
    walls := walls

    length_box := get_length_box(walls)

    @(static)
    current_time: f64 = 0
    context.user_ptr = &current_time
    defer current_time += dt

    // clear particles in cells
    for cell in &cells {
        clear(&cell.particle_ids)
    }

    // update position by half a timestep and find cell of particle
    for sphere, sphere_id in &spheres {
        using sphere
        position += velocity * dt / 2
        if is_periodic {
            if position.x < -length_box.x / 2 do position.x = position.x + length_box.x
            if position.x > length_box.x / 2 do position.x = position.x - length_box.x
        }
        force = -E_y * mass * G
        torque = 0

        id_cell_of_particle := position_to_cell_id(position, cell_context, walls)
        append(&cells[id_cell_of_particle].particle_ids, auto_cast sphere_id)
    }


    for sphere, sphere_id in &spheres {
        using sphere

        id_cell_of_particle := position_to_cell_id(position, cell_context, walls)
        cell_of_particle := cells[id_cell_of_particle]

        // For it to work like the linked list ( like a stack, last in first out)
        slice.reverse(cell_of_particle.particle_ids[:])

        // in the same cell
        for id_other in cell_of_particle.particle_ids do if auto_cast sphere_id < id_other {
                delta: f64
            if is_periodic do delta = measure_delta_spheres_periodic(&sphere, &spheres[id_other], length_box.x)
	    else do delta = measure_delta(&sphere, &spheres[id_other])

                if delta < 0 {
                    index := generate_hash(sphere_id = auto_cast sphere_id, other_id = id_other, other_is_wall = false)
                    update_contact(index, contacts, delta, current_time, params)
                }
            }

        // in neighbor cell
        using cell_of_particle
        for neighbor_cell in neighbor_cells_iterator(cells, neighbors[:total_neighbors]) {
            for id_other in neighbor_cell.particle_ids {
                delta: f64

                if is_periodic do delta = measure_delta_spheres_periodic(&sphere, &spheres[id_other], length_box.x)
		else do delta = measure_delta(&sphere, &spheres[id_other])

                if delta < 0 {
                    index := generate_hash(sphere_id = auto_cast sphere_id, other_id = id_other, other_is_wall = false)
                    update_contact(index, contacts, delta, current_time, params)
                }
            }
        }

        for wall in &walls {
            delta := measure_delta(&sphere, &wall)
            if is_periodic {
                if delta < 0 && wall.center_position.x == 0 {
                    index := generate_hash(sphere_id = auto_cast sphere_id, other_id = wall.id, other_is_wall = true)
                    update_contact(index, contacts, delta, current_time, params)
                }
            } else {
                if delta < 0 {
                    index := generate_hash(sphere_id = auto_cast sphere_id, other_id = wall.id, other_is_wall = true)
                    update_contact(index, contacts, delta, current_time, params)
                }
            }
        }
    }

    update_contact_forces(contacts, current_time, spheres, walls, length_box, params, false)

    for chain in &chains do chain_internal_forces(chain, chain.spheres, params)

    for sphere, id in &spheres {
        /* if id == 0 do continue */
        using sphere
        velocity += force * (dt / mass)
        angular_velocity += torque * (dt / moment_of_inertia)
    }

    // another half timestep update to finish it off
    for sphere, _ in &spheres {
        using sphere
        position += velocity * dt / 2
        if is_periodic {
            if position.x < -length_box.x / 2 do position.x = position.x + length_box.x
            if position.x > length_box.x / 2 do position.x = position.x - length_box.x
        }

        // update the orientation
        if length_squared(angular_velocity) > 0 {
            new_rotation := linalg.quaternion_angle_axis_f64(
                length(angular_velocity) * dt,
                auto_cast normalize(angular_velocity),
            )
            orientation = new_rotation * orientation
        }
    }
}

init_sphere :: proc(spheres: []Sphere, length_box: [3]f64, min_radius, max_radius, density: f64) -> Sphere {
    spheres := spheres
    using sphere: Sphere
    radius = rand.float64_uniform(min_radius, max_radius)
    mass = density * (4. / 3.) * math.PI * math.pow(radius, 3)
    moment_of_inertia = mass * radius * radius * (2. / 5.)
    orientation = 1 // identity quaternion
    collision := true
    for collision {
        collision = false
        sphere.position = vec3{
            rand.float64_range(-length_box.x / 2 + radius, length_box.x / 2 - radius),
            rand.float64_range(-length_box.y / 2 + radius, length_box.y / 2 - radius),
            rand.float64_range(-length_box.z / 2 + radius, length_box.z / 2 - radius),
        }
        for other_sphere in &spheres {
            if delta := measure_delta(&sphere, &other_sphere); delta < 0 do collision = true
        }
    }
    return sphere
}

init_walls :: proc(length_box: [3]f64, diameter: f64) -> []Wall {

    walls := make([]Wall, 6)

    for i in 0 ..< 3 do assert(math.remainder(length_box[i], diameter) == 0)

    first_walls := [?]Wall{
        {center_position = vec3{-length_box.x / 2, 0, 0}, normal = vec3{1, 0, 0}, id = 0},
        {center_position = vec3{length_box.x / 2, 0, 0}, normal = vec3{-1, 0, 0}, id = 1},
        {center_position = vec3{0, -length_box.y / 2, 0}, normal = vec3{0, 1, 0}, id = 2},
        {center_position = vec3{0, length_box.y / 2, 0}, normal = vec3{0, -1, 0}, id = 3},
        {center_position = vec3{0, 0, -length_box.z / 2}, normal = vec3{0, 0, 1}, id = 4},
        {center_position = vec3{0, 0, length_box.z / 2}, normal = vec3{0, 0, -1}, id = 5},
    }

    copy(walls, first_walls[:])

    return walls
}

measure_delta_spheres :: #force_inline proc(sphere, other_sphere: ^Sphere) -> (delta: f64) {
    delta = length(sphere.position - other_sphere.position) - sphere.radius - other_sphere.radius
    return
}

measure_delta_spheres_periodic :: #force_inline proc(
    sphere, other_sphere: ^Sphere,
    length_box_periodic: f64,
) -> (
    delta: f64,
) {
    sphere_to_sphere_vec := sphere.position - other_sphere.position
    sign_x := 1. if sphere_to_sphere_vec.x > 0 else -1

    if abs(sphere_to_sphere_vec.x) > length_box_periodic / 2 {
        new_sphere_pos := sphere.position - sign_x * E_x * length_box_periodic
        sphere_to_sphere_vec = new_sphere_pos - other_sphere.position
    }
    delta = length(sphere_to_sphere_vec) - sphere.radius - other_sphere.radius

    return
}

measure_delta_sphere_wall :: #force_inline proc(sphere: ^Sphere, wall: ^Wall) -> (delta: f64) {
    using sphere
    delta = dot(position - wall.center_position, wall.normal) - radius
    return
}

measure_delta :: proc {
    measure_delta_spheres,
    measure_delta_sphere_wall,
}

INDEX_POSITION :: 10_000_000
WALL_CHECK :: 10_000

generate_hash :: proc(sphere_id, other_id: i32, other_is_wall: bool) -> int {
    // NOTE: if we have 5000 particles max
    assert(sphere_id < 5000 && other_id < 5000)

    first_index: int
    second_index: int

    if other_is_wall {
        first_index = auto_cast sphere_id
        second_index = auto_cast other_id
    } else {
        first_index = auto_cast sphere_id if sphere_id < other_id else auto_cast other_id
        second_index = auto_cast other_id if first_index == auto_cast sphere_id else auto_cast sphere_id
    }
    if !other_is_wall do return first_index * INDEX_POSITION + second_index
    else do return first_index * INDEX_POSITION + 1 * WALL_CHECK + second_index
}

get_indices_from_hash :: #force_inline proc(hash: int) -> (sphere_index, other_index: int, other_is_wall: bool) {
    sphere_index = hash / INDEX_POSITION

    rest_hash := hash - sphere_index * INDEX_POSITION
    if rest_hash >= WALL_CHECK {
        other_is_wall = true
        other_index = rest_hash - WALL_CHECK
    } else {
        other_index = rest_hash
    }
    return
}

update_contact :: #force_inline proc(
    index: int,
    contacts: ^map[int]Contact,
    delta, current_time: f64,
    params: Params,
) {
    using params

    old_contact, exists := &contacts[index]

    if !exists {
        contact: Contact
        contact.delta_normal = delta
        contact.time_last_update = current_time
        contact.friction_coeff = mu_static
        sphere_id, other_id, other_is_wall := get_indices_from_hash(index)
        contact.particle_id = auto_cast sphere_id
        contact.particle_or_wall_id = auto_cast other_id
        contact.other_is_wall = other_is_wall
        contacts[index] = contact
        return
    } else {
        old_contact.delta_normal = delta
        old_contact.time_last_update = current_time
        return
    }
}

update_contact_forces :: proc(
    contacts: ^map[int]Contact,
    current_time: f64,
    spheres: []Sphere,
    walls: []Wall,
    length_box: [3]f64,
    params: Params,
    is_periodic: bool,
) {
    using params
    for map_key, contact in contacts do if contact.time_last_update == current_time {

            slip_velocity: vec3
            normal: vec3
            contact_viscosity_normal, contact_viscosity_tangent: f64
            sphere := &spheres[contact.particle_id]
            {     // determine what info we need for the sim
                if contact.other_is_wall {
                    using contact
                    using sphere
                    normal = walls[particle_or_wall_id].normal
                    wall := &walls[particle_or_wall_id]
                    slip_velocity = velocity + cross(angular_velocity, -normal * radius) - wall.velocity
                    contact_viscosity_wall_normal := -2. * math.ln(restitution_coeff) *
			math.sqrt(mass * k / (math.ln(restitution_coeff) * math.ln(restitution_coeff) + math.PI * math.PI))

                    contact_viscosity_normal = contact_viscosity_wall_normal
                    contact_viscosity_tangent = contact_viscosity_wall_normal * 0.2
                } else {
                    using contact
                    inspect_contact := contact
                    other_sphere := &spheres[particle_or_wall_id]

                    sphere_to_sphere_vec := sphere.position - other_sphere.position
                    sign_x := 1. if sphere_to_sphere_vec.x > 0 else -1.
                    if is_periodic && abs(sphere_to_sphere_vec.x) > length_box.x / 2 {
                        new_sphere_pos := sphere.position - sign_x * E_x * length_box.x
                        sphere_to_sphere_vec = new_sphere_pos - other_sphere.position
                    }
                    normal = normalize(sphere_to_sphere_vec)
                    slip_velocity = sphere.velocity + cross(sphere.angular_velocity, -sphere.radius * normal) -
			(other_sphere.velocity + cross(other_sphere.angular_velocity, other_sphere.radius * normal))

                    effective_mass := (sphere.mass * other_sphere.mass) / (sphere.mass + other_sphere.mass)
                    contact_viscosity_particles_normal := -2. * math.ln(restitution_coeff) *
			math.sqrt((effective_mass / 2) * k / (math.ln(restitution_coeff) * math.ln(restitution_coeff) + math.PI * math.PI))

                    contact_viscosity_normal = contact_viscosity_particles_normal
                    contact_viscosity_tangent = contact_viscosity_particles_normal * 0.2
                }
            }

            slip_normal := dot(slip_velocity, normal) * normal
            slip_tangent := slip_velocity - slip_normal

            using contact
            delta_tangent_squared := length_squared(tangent_spring)
            tangent_spring = tangent_spring - dot(tangent_spring, normal) * normal
            if delta_tangent_squared > 0 do tangent_spring = math.sqrt(delta_tangent_squared) * normalize(tangent_spring)

            normal_force := -delta_normal * k * normal - contact_viscosity_normal * slip_normal
            coulomb_force_length_squared := length_squared(friction_coeff * normal_force)

            tangent_force := -k_tangent * tangent_spring - contact_viscosity_tangent * slip_tangent

            if length_squared(tangent_force) < coulomb_force_length_squared {
                // static friction
                friction_coeff = mu_static
                tangent_spring = tangent_spring + slip_tangent * dt
            } else {
                // dynamic friction (sliding)
                coulomb_force_length := math.sqrt(coulomb_force_length_squared)
                friction_coeff = mu_dynamic
                coulomb_force_length = length(friction_coeff * normal_force)
                tangent_spring = -(coulomb_force_length * normalize(tangent_force) + contact_viscosity_tangent * slip_tangent) / k_tangent
            }

            // apply the forces on the particles
            torque_from_tangent_force := cross(-normal * sphere.radius, tangent_force)

            sphere.force += normal_force
            sphere.force += tangent_force
            sphere.torque += torque_from_tangent_force

            if !other_is_wall {
                other_sphere := &spheres[particle_or_wall_id]
                other_sphere.force -= normal_force
                other_sphere.force -= tangent_force
                other_sphere.torque += torque_from_tangent_force
            }
        } else {
            // no more contact, can delete the element from the map
            delete_key(contacts, map_key)
        }
}

E_x := vec3{1, 0, 0}
E_y := vec3{0, 1, 0}
E_z := vec3{0, 0, 1}

dot :: #force_inline proc(x, y: vec3) -> f64 #no_bounds_check {
    result: f64
    for i in 0 ..< 3 {
        result += x[i] * y[i]
    }
    return result
}

cross :: #force_inline proc(a, b: vec3) -> vec3 {
    result: vec3
    result.x = a.y * b.z - b.y * a.z
    result.y = b.x * a.z - a.x * b.z
    result.z = a.x * b.y - a.y * b.x
    return result
}

length :: #force_inline proc(a: vec3) -> f64 {
    result := math.sqrt(dot(a, a))
    return result
}

length_squared :: #force_inline proc(a: vec3) -> f64 {
    return dot(a, a)
}

normalize :: #force_inline proc(a: vec3) -> vec3 {
    result := a / length(a)
    return result
}

vec3_to_rl :: #force_inline proc(vec: vec3) -> rl.Vector3 #no_bounds_check {
    result: rl.Vector3
    for element, id in vec do result[id] = auto_cast element
    return result
}

get_length_box :: #force_inline proc(walls: []Wall) -> [3]f64 {
    length_x := length(walls[1].center_position - walls[0].center_position)
    length_y := length(walls[3].center_position - walls[2].center_position)
    length_z := length(walls[5].center_position - walls[4].center_position)

    length_box := [3]f64{length_x, length_y, length_z}
    return length_box
}
