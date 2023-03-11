package dem

import "core:slice"
import "core:encoding/json"
import "core:fmt"
import "core:os"
import "core:math"
import "core:log"

Experiment_type :: enum {
    shaker,
    sliding,
    nothing,
}

Experiment :: struct {
    data:            []Simulation_state,
    cell_context:    ^Cell_context,
    experiment_type: Experiment_type,
}

Simulation_state :: struct {
    spheres:      []Sphere,
    walls:        []Wall, // NOTE: since walls will be able to move (shaker)
    current_time: f64,
}

Params :: struct {
    k:                 f64,
    k_tangent:         f64,
    dt:                f64,
    mu_static:         f64,
    mu_dynamic:        f64,
    restitution_coeff: f64,
}

init_params :: proc(radius, density_particle, restitution_coeff, k: f64, length_box: [3]f64) -> Params {
    /* mass := (4. / 3.) * math.PI * math.pow(radius, 3) * density_particle */
    k_tangent := k / 5.0
    /* dt := 0.05 * math.PI * math.sqrt(mass / k) */
    dt := 1e-6

    params := Params {
	k                 = k,
	k_tangent         = k_tangent,
	dt                = dt,
	mu_static         = 0.6,
	mu_dynamic        = 0.2,
	restitution_coeff = restitution_coeff,
    }

    return params
}

dump_experiment_results_json :: proc(results: ^Experiment, file: string) {
    fmt.println("Start of marshal")
    file_data, ok_marchal := json.marshal(results^, {})
    defer delete(file_data)
    fmt.println("Marshaling done")

    if ok_marchal != nil {
	panic("epics marshal fail")
    }
    fmt.println("Start writing file")
    ok := os.write_entire_file(file, file_data)
    fmt.println("Done writing file")
    if !ok {
	panic("epics writing to file fail")
    }
}

read_experiment_results_json :: proc(file: string) -> ^Experiment {
    fmt.println("Start reading file")
    file_data, ok := os.read_entire_file(file)
    if !ok {
	panic("epics file read fail")
    }
    fmt.println("Reading file done")

    results := new(Experiment)
    fmt.println("Start unmarshaling")
    ok_unmarchal := json.unmarshal(file_data[:], results)
    if ok_unmarchal != nil {
	fmt.println(ok_unmarchal)
	panic("epics unmarshal fail")
    }
    fmt.println("Done unmarshaling")
    return results
}

read_experiment :: proc(directory: string, results: ^Experiment) {
    directory_hd, err := os.open(directory)
    if err != os.ERROR_NONE {
	os.exit(1)
    }
    defer os.close(directory_hd)
    file_infos, _ := os.read_dir(directory_hd, 0)
    defer os.file_info_slice_delete(file_infos)

    sort_fileinfos_by_name :: proc(i, j: os.File_Info) -> bool {
	min_lenght_name := min(len(i.name), len(j.name))
	which_is_shorter := 0 if min_lenght_name == len(i.name) else 1

	for id in 0 ..< min_lenght_name {
	    char_i := i.name[id]
	    char_j := j.name[id]

	    if char_i < char_j do return true
	    if char_i > char_j do return false
	}

	if which_is_shorter == 0 do return true
	else do return false
    }
    slice.sort_by(file_infos, sort_fileinfos_by_name)

    index_cells: int
    total_sim_state_files: int
    for file_info, id in file_infos {
	if file_info.name == "cells.dat" do index_cells = id
	// NOTE: Don't have too many output files for the simulation
	if file_info.name[0] == '0' do total_sim_state_files += 1
    }
    // NOTE: shaker stuff is the last file and cells is the one before last
    results.cell_context = read_cell_context(file_infos[index_cells].fullpath)

    data: [dynamic]Simulation_state
    for i in 0 ..< total_sim_state_files {
	current_sim_state: Simulation_state
	current_sim_state = read_sim_state(file_infos[i].fullpath)^
	    append(&data, current_sim_state)
    }
    fmt.printf("\nDone Reading File\n")

    results.data = data[:]
}

dump_cell_context :: proc(cell_context: ^Cell_context, directory: string) {
    log.warn("function not tested")
    using cell_context
    file, ok := os.open(fmt.tprintf("%s%s", directory, "cells.dat"), os.O_RDWR | os.O_CREATE)
    defer os.close(file)
    assert(ok == os.ERROR_NONE)

    // write cell info first
    os.write_ptr(file, &cell_context.info, size_of(Cell_info))

    total_cells := len(cells)
    os.write_ptr(file, &total_cells, size_of(total_cells))

    Cell :: struct {
	particle_ids:    [dynamic]int, // holding the indices of the particles
	neighbors:       [13]int, // also holding indices of neighbor cells
	total_neighbors: int,
	id:              int,
    }

    Cell_context :: struct {
	info:  Cell_info,
	cells: []Cell,
    }

    Cell_info :: struct {
	total_cells_along: [3]int, // NOTE: number of cells along each axis of the box
	cell_length:       f64,
    }

    for cell in &cells {
	total_particle_ids := len(cell.particle_ids)
	os.write_ptr(file, &total_particle_ids, size_of(total_particle_ids))
	for i in 0 ..< total_particle_ids {
	    os.write_ptr(file, &cell.particle_ids[i], size_of(int))
	}
	os.write_ptr(file, &cell.neighbors, size_of(cell.neighbors))
	os.write_ptr(file, &cell.total_neighbors, size_of(cell.total_neighbors))
	os.write_ptr(file, &cell.id, size_of(cell.id))
    }
}

read_cell_context :: proc(filename: string) -> ^Cell_context {
    log.warn("function not tested")
    cells: [dynamic]Cell
    file, ok := os.open(filename, os.O_RDWR)
    defer os.close(file)
    assert(ok == os.ERROR_NONE)

    cell_info: Cell_info
    os.read_ptr(file, &cell_info, size_of(Cell_info))

    total_cells: int
    os.read_ptr(file, &total_cells, size_of(total_cells))

    err: os.Errno
    for _ in 0 ..< total_cells {
	cell: Cell
	total_particle_ids: int
	os.read_ptr(file, &total_particle_ids, size_of(total_particle_ids))
	for _ in 0 ..< total_particle_ids {
	    particle_id: i32
	    _, err = os.read_ptr(file, &particle_id, size_of(int))
	    assert(err == os.ERROR_NONE)
	    append(&cell.particle_ids, particle_id)
	}
	_, err = os.read_ptr(file, &cell.neighbors, size_of(cell.neighbors))
	assert(err == os.ERROR_NONE)
	_, err = os.read_ptr(file, &cell.total_neighbors, size_of(cell.total_neighbors))
	assert(err == os.ERROR_NONE)
	_, err = os.read_ptr(file, &cell.id, size_of(cell.id))
	assert(err == os.ERROR_NONE)

	append(&cells, cell)
    }

    cell_context := new(Cell_context)
    cell_context.info = cell_info
    cell_context.cells = cells[:]
    
    return cell_context
}

dump_sim_state :: proc(sim_state: ^Simulation_state, directory: string, id: int) {
    file, ok := os.open(fmt.tprintf("%s%07d%s", directory, id, ".dat"), os.O_RDWR | os.O_CREATE)
    defer os.close(file)
    assert(ok == os.ERROR_NONE)

    spheres := sim_state.spheres
    current_time := sim_state.current_time
    walls := sim_state.walls

    os.write_ptr(file, &current_time, size_of(current_time))
    total_walls := len(walls)
    total_spheres := len(spheres)
    os.write_ptr(file, &total_walls, size_of(total_walls))
    os.write_ptr(file, &total_spheres, size_of(total_spheres))

    // walls
    for wall in &walls do os.write_ptr(file, &wall, size_of(wall))

    // spheres
    for sphere in &spheres do os.write_ptr(file, &sphere, size_of(sphere))
}

read_sim_state :: proc(filename: string) -> ^Simulation_state {
    spheres: [dynamic]Sphere
    walls: [dynamic]Wall
    current_time: f64
    total_spheres: int
    total_walls: int
    file, _ := os.open(filename, os.O_RDWR)
    defer os.close(file)

    os.read_ptr(file, &current_time, size_of(current_time))
    os.read_ptr(file, &total_walls, size_of(total_walls))
    os.read_ptr(file, &total_spheres, size_of(total_spheres))

    for _ in 0 ..< total_walls {
	wall: Wall
	os.read_ptr(file, &wall, size_of(wall))
	append(&walls, wall)
    }

    for _ in 0 ..< total_spheres {
	sphere: Sphere
	os.read_ptr(file, &sphere, size_of(sphere))
	append(&spheres, sphere)
    }

    result_sim_state := new(Simulation_state)
    result_sim_state.current_time = current_time
    result_sim_state.spheres = spheres[:]
    result_sim_state.walls = walls[:]
    return result_sim_state
}
