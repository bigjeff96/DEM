package dem

import "core:math"

Cell :: struct {
    particle_ids:      [dynamic]int, // holding the indices of the particles
    neighbors:         [13]int, // also holding indices of neighbor cells
    total_neighbors:   int,
    id:                int,
}

Cell_info :: struct {
    total_cells_along: [3]int, // NOTE: number of cells along each axis of the box
    cell_length:       f64,    
}

Cell_context :: struct {
    info: Cell_info,
    cells: []Cell,
}

init_cell_context :: proc(largest_radius: f64, walls: []Wall) -> (cell_context: ^Cell_context) {
    // divide up the box in cells, each one the size of a particle
    using math
    length_box := get_length_box(walls)
    box_volume: f64 = length_box.x * length_box.y * length_box.z
    total_cells_along: [3]int
    total_cells_along.x = int(length_box.x / (2 * largest_radius))
    total_cells_along.y = int(length_box.y / (2 * largest_radius))
    total_cells_along.z = int(length_box.z / (2 * largest_radius))
    cell_volume: f64 = pow(2 * largest_radius, 3)
    total_cells := int(math.round(box_volume / cell_volume))
    
    cell_context = new(Cell_context)
    cell_context.info.total_cells_along = total_cells_along
    cell_context.info.cell_length = 2 * largest_radius
    
    cells := make([]Cell, total_cells)
    cell_context.cells = cells

    total_cells_level_xy := total_cells_along.x * total_cells_along.y

    //determine cell neighbors for each cell (only want half, so to visit each interaction pair only once)
    for cell, id in &cells {
	using cell_context.info
	cell.id = id
	i, j, k := indices_from_unique_id(id, cell_context)
	tmp_i, tmp_j, tmp_k: int

	// (-1,0,+1: -1,0,+1: +1)
	tmp_k = k + 1
	if tmp_k * total_cells_level_xy < total_cells {
	    for delta_i := -1; delta_i <= 1; delta_i += 1 {
		for delta_j := -1; delta_j <= 1; delta_j += 1 {
		    tmp_i = i + delta_i
		    tmp_j = j + delta_j

		    correct_i := tmp_i >= 0 && tmp_i < total_cells_along.x
		    correct_j := tmp_j >= 0 && tmp_j < total_cells_along.y

		    if correct_i && correct_j {
			using cell
			neighbors[total_neighbors] = unique_id_from_indices(tmp_i, tmp_j, tmp_k, cell_context)
			total_neighbors += 1
		    }
		}
	    }
	}

	// (+1:0:0)
	tmp_k = k
	tmp_j = j
	tmp_i = i + 1
	if tmp_i >= 0 && tmp_i < total_cells_along.x {
	    using cell
	    neighbors[total_neighbors] = unique_id_from_indices(tmp_i, tmp_j, tmp_k, cell_context)
	    total_neighbors += 1
	}

	// (-1,0,+1: +1: 0)
	tmp_k = k
	tmp_j = j + 1
	if tmp_j >= 0 && tmp_j * total_cells_along.x < total_cells_level_xy {
	    for delta_i := -1; delta_i <= 1; delta_i += 1 {
		tmp_i = i + delta_i

		if tmp_i >= 0 && tmp_i < total_cells_along.x {
		    using cell
		    neighbors[total_neighbors] = unique_id_from_indices(tmp_i, tmp_j, tmp_k, cell_context)
		    total_neighbors += 1
		}
	    }
	}
    }

    return cell_context
}

// cells with periodic boundary conditions (along x only)
init_cell_context_PBC :: proc(radius: f64, walls: []Wall) -> (cell_context: ^Cell_context) {
    // divide up the box in cells, each one the size of a particle
    using math
    length_box := get_length_box(walls)
    box_volume: f64 = length_box.x * length_box.y * length_box.z
    total_cells_along: [3]int
    total_cells_along.x = int(length_box.x / (2 * radius))
    total_cells_along.y = int(length_box.y / (2 * radius))
    total_cells_along.z = int(length_box.z / (2 * radius))
    cell_volume := pow(2 * radius, 3)

    cell_context = new(Cell_context)
    cell_context.info.cell_length = 2 * radius
    cell_context.info.total_cells_along = total_cells_along
    
    total_cells := total_cells_along.x * total_cells_along.y * total_cells_along.z
    cells := make([]Cell, total_cells)
    cell_context.cells = cells

    total_cells_level_xy := total_cells_along.x * total_cells_along.y

    //determine cell neighbors for each cell (only want half, so to visit each interaction pair only once)
    for cell, id in &cells {
	using cell_context.info
	cell.id = id
	i, j, k := indices_from_unique_id(id, cell_context)
	tmp_i, tmp_j, tmp_k: int

	// (-1,0,+1: -1,0,+1: +1)
	tmp_k = k + 1
	if tmp_k * total_cells_level_xy < total_cells {
	    for delta_i := -1; delta_i <= 1; delta_i += 1 {
		for delta_j := -1; delta_j <= 1; delta_j += 1 {
		    tmp_i = i + delta_i
		    tmp_j = j + delta_j

		    correct_j := tmp_j >= 0 && tmp_j < total_cells_along.y

		    // periodic along x
		    if tmp_i < 0 do tmp_i = total_cells_along.x + tmp_i
		    if tmp_i == total_cells_along.x do tmp_i = 0

		    if correct_j {
			using cell
			neighbors[total_neighbors] = unique_id_from_indices(tmp_i, tmp_j, tmp_k, cell_context)
			total_neighbors += 1
		    }
		}
	    }
	}

	// (+1:0:0)
	tmp_k = k
	tmp_j = j
	tmp_i = i + 1
	if tmp_i < 0 do tmp_i = total_cells_along.x + tmp_i
	if tmp_i == total_cells_along.x do tmp_i = 0
	{
	    using cell
	    neighbors[total_neighbors] = unique_id_from_indices(tmp_i, tmp_j, tmp_k, cell_context)
	    total_neighbors += 1
	}

	// (-1,0,+1: +1: 0)
	tmp_k = k
	tmp_j = j + 1
	if tmp_j >= 0 && tmp_j * total_cells_along.x < total_cells_level_xy {
	    for delta_i := -1; delta_i <= 1; delta_i += 1 {
		tmp_i = i + delta_i

		// periodic along x
		if tmp_i < 0 do tmp_i = total_cells_along.x + tmp_i
		if tmp_i == total_cells_along.x do tmp_i = 0
		{
		    using cell
		    neighbors[total_neighbors] = unique_id_from_indices(tmp_i, tmp_j, tmp_k, cell_context)
		    total_neighbors += 1
		}
	    }
	}
    }

    return cell_context
}

unique_id_from_indices :: proc(i, j, k: int, cell_context: ^Cell_context) -> (unique_id: int) {
    using cell_context.info
    total_cells_level := total_cells_along.x * total_cells_along.y

    unique_id = i + j * total_cells_along.x + k * total_cells_level
    return
}

indices_from_unique_id :: proc(unique_id: int, cell_context: ^Cell_context) -> (int, int, int) {
    using cell_context.info
    using math
    total_cells_level := total_cells_along.x * total_cells_along.y

    k := floor_div(unique_id, total_cells_level)
    j := floor_div(unique_id % total_cells_level, total_cells_along.x)
    i := (unique_id % total_cells_level) % total_cells_along.x
    return i, j, k
}

cell_coords_unique_id :: proc(unique_id: int, cell_context: ^Cell_context, walls: []Wall) -> vec3 {
    i, j, k := indices_from_unique_id(unique_id, cell_context)
    using cell_context.info
    position: vec3
    position.x = walls[0].center_position.x + cell_length/2. + auto_cast i * cell_length
    position.y = walls[2].center_position.y + cell_length/2. + auto_cast j * cell_length
    position.z = walls[4].center_position.z + cell_length/2. + auto_cast k * cell_length

    return position
}

// NOTE: Can produce a incorrect index
position_to_cell_id :: proc(
    position: vec3,
    cell_context: ^Cell_context,
    walls: []Wall,
) -> (
    cell_id: int,
) {
    using cell_context.info
    i := clamp(int(((position.x - walls[0].center_position.x) / (cell_length)) + 0.5), 0, total_cells_along.x - 1)
    j := clamp(int(((position.y - walls[2].center_position.y) / (cell_length)) + 0.5), 0, total_cells_along.y - 1)
    k := clamp(int(((position.z - walls[4].center_position.z) / (cell_length)) + 0.5), 0, total_cells_along.z - 1)

    cell_id = unique_id_from_indices(i, j, k, cell_context)
    return
}

cell_next_to_wall :: proc(cell: ^Cell, cell_context: ^Cell_context) -> bool {
    using cell_context.info
    i, j, k := indices_from_unique_id(cell.id, cell_context)
    using cell

    if i == 0 || i == total_cells_along.x - 1 do return true
    if j == 0 || j == total_cells_along.y - 1 do return true
    if k == 0 || k == total_cells_along.z - 1 do return true

    return false
}

get_real_neighbor_cell_ids :: proc(cell_context: ^Cell_context, id_of_cell: int, experiment_type: Experiment_type) -> [dynamic]int {
    using cell_context
    using info
    real_neighbor_cell_ids := make([dynamic]int, context.temp_allocator)


    i, j, k := indices_from_unique_id(id_of_cell, cell_context)
    tmp_i, tmp_j, tmp_k: int

    for delta_i := -1; delta_i <= 1; delta_i += 1 {
	for delta_j := -1; delta_j <= 1; delta_j += 1 {
	    for delta_k := -1; delta_k <= 1; delta_k += 1 {
		if delta_i == 0 && delta_j == 0 && delta_k == 0 do continue

		tmp_i = i + delta_i
		tmp_j = j + delta_j
		tmp_k = k + delta_k

		correct_j := tmp_j >= 0 && tmp_j < total_cells_along.y
		correct_k := tmp_k >= 0 && tmp_k < total_cells_along.z
		correct_i := tmp_i >= 0 && tmp_i < total_cells_along.x

		if experiment_type == .sliding {
		    if tmp_i < 0 do tmp_i = total_cells_along.x + tmp_i
		    if tmp_i == total_cells_along.x do tmp_i = 0
		    correct_i = true
		}

		if correct_i && correct_j && correct_k {
		    neighbor_id := unique_id_from_indices(tmp_i, tmp_j, tmp_k, cell_context)
		    append(&real_neighbor_cell_ids, neighbor_id)
		}
	    }
	}
    }

    return real_neighbor_cell_ids
}

neighbor_cells_iterator :: proc(cells: []Cell, neighbor_ids: []int) -> (cell: ^Cell, ok: bool) {
    @(static)
    id: int = 0
    if id < len(neighbor_ids) {
	ok = true
	cell = &cells[neighbor_ids[id]]
	id += 1
	return
    }

    id = 0
    return
}
