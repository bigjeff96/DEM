package compaction

import "core:slice"
import "core:strings"
import "core:fmt"
import "core:os"
import "core:math"
import "core:time"
import "core:log"
import "dem"

main :: proc() {
    logger := log.create_console_logger()
    defer log.destroy_console_logger(logger)
    context.logger = logger

    dem.debug_sim_code()
}

delta_study :: proc(sim_directory: string) {
    using dem
    results := new(dem.Experiment)
    defer free(results)
    read_experiment(sim_directory, results)

    data := results.data
    cell_context := results.cell_context
    deltas := new([dynamic]f64)
    using cell_context

    for sim_state, sim_state_id in data {
        walls := sim_state.walls
        spheres := sim_state.spheres

        length_box := get_length_box(walls)

        for cell in &cells {
            clear(&cell.particle_ids)
        }
        for sphere, sphere_id in spheres {
            using sphere
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
            for id_other in cell_of_particle.particle_ids do if cast(i32)sphere_id < id_other {
                    delta := measure_delta(&sphere, &spheres[id_other])
                    if delta < 0 do append(deltas, delta)

                }
            // in neighbor cell
            using cell_of_particle
            for neighbor_cell in neighbor_cells_iterator(cells, neighbors[:total_neighbors]) {
                for id_other in neighbor_cell.particle_ids {
                    delta := measure_delta(&sphere, &spheres[id_other])
                    if delta < 0 do append(deltas, delta)
                }
            }
        }

        fmt.printf("\rTotal work done: %d with id %d", 100 * sim_state_id / (len(data)), sim_state_id)
    }

    output_file_delta, err := os.open(fmt.tprintf("%s%s", sim_directory, "delta.txt"), os.O_RDWR | os.O_CREATE)
    defer os.close(output_file_delta)
    assert(err == os.ERROR_NONE)

    for delta, id in deltas {
        fmt.fprintf(output_file_delta, "%.10f\n", delta)
        fmt.printf("\rTotal work done: %d", 100 * id / (len(deltas)))
    }
}

compaction_experiment_and_dump :: proc(shaking_parameter: f64, directory: string, dump_results: bool) {
    using dem
    // physical parameters
    total_particles := 1000
    radius: f64 = 1e-3
    radius_for_spheres := 0.8 * radius
    density_glass_beads: f64 = 2500.0
    box_length_x := 7 * 2 * radius
    box_length_z := 7 * 2 * radius
    box_length_y := 80 * 2 * radius
    usual_restitution_coeff := 0.8

    length_box := [3]f64{box_length_x, box_length_y, box_length_z}
    k_pile :=
        10. *
        (f64(total_particles) *
                math.pow(f64(math.PI), 2) *
                (4. / 3.) *
                math.pow(radius_for_spheres, 5) *
                density_glass_beads *
                G) /
        ((radius_for_spheres / 100.) * box_length_x * box_length_z)

    fmt.printf("k_pile is %f\n", k_pile)
    params := init_params(radius_for_spheres, density_glass_beads, usual_restitution_coeff, k_pile, length_box)
    using params

    fmt.printf("dt = %.10f\n", dt)

    walls := init_walls(length_box, 2 * radius)
    cell_context := init_cell_context(radius, walls)
    spheres: [dynamic]Sphere
    contacts: map[int]Contact

    // shaker parameters
    shaker_frequency: f64 = 30.0
    shaker_amplitude := G * shaking_parameter / (4 * math.pow_f64(math.PI, 2.) * math.pow(shaker_frequency, 2))
    total_time := 10. //535.
    waiting_time: f64 = 0.5
    fps := 30.0

    frame_duration := 1. / fps
    frame_duration_steps := int(frame_duration / dt)
    fmt.printf("fps of %v gives a frame duration of %v * dt\n", fps, frame_duration_steps)

    initial_center_positions: [6]vec3

    for wall, id in walls do initial_center_positions[id] = wall.center_position
    shaking_func :: proc(
        walls: []dem.Wall,
        initial_center_positions: [6]dem.vec3,
        shaker_amplitude, shaker_frequency, time: f64,
    ) {
        walls := walls
        for wall in &walls {
            using wall
            velocity =
                E_y *
                shaker_amplitude *
                2 *
                math.PI *
                shaker_frequency *
                math.cos(2 * math.PI * shaker_frequency * time)
            center_position =
                initial_center_positions[id] + E_y * shaker_amplitude * math.sin(2 * math.PI * shaker_frequency * time)
        }
    }
    for _ in 0 ..< total_particles {
        append(&spheres, init_sphere(spheres[:], length_box, radius * 0.80, radius, density_glass_beads))
    }

    shaker_starts: [dynamic]f64
    file_id: int
    if dump_results {
        os.make_directory(directory)
        dump_cell_context(cell_context, directory)
    }

    period_shaker_steps := (1. / shaker_frequency) / dt
    flip_flop := false
    current_shaker_step: f64
    start := time.now()
    for current_time: f64 = 0; current_time <= total_time; current_time += dt * f64(frame_duration_steps) {

        if dump_results {
            dump_sim_state(
                &Simulation_state{spheres = spheres[:], walls = walls, current_time = current_time},
                directory,
                file_id,
            )
        }
        file_id += 1

        for i in 0 ..< frame_duration_steps {
            physics_update(spheres[:], cell_context, walls[:], &contacts, false, params)
            thing_to_compare := period_shaker_steps if flip_flop else waiting_time / dt

            if current_shaker_step < thing_to_compare {
                current_shaker_step += 1
                if flip_flop {
                    shaking_func(
                        walls,
                        initial_center_positions,
                        shaker_amplitude,
                        shaker_frequency,
                        dt * current_shaker_step,
                    )
                } else {
                    if walls[BOTTOM_WALL_ID].velocity != 0 do for wall in &walls do wall.velocity = 0
                }
            } else {
                current_shaker_step = 0
                flip_flop = !flip_flop
                if flip_flop == true do append(&shaker_starts, current_time + f64(i) * dt)
            }
        }

        @(static)
        previous_work_done := 0
        work_done: int = int(current_time * 100 / total_time)

        if work_done > previous_work_done {
            previous_work_done = work_done
            fmt.printf("\rTotal work done: %d%%", previous_work_done)
        } else if previous_work_done == 0 do fmt.printf("\rTotal work done: %d%%", previous_work_done)
    }
    end := time.now()
    simulation_time := time.duration_seconds(time.diff(start, end))

    fmt.printf(" for %f secs, it takes %f secs of compute time\n", total_time, simulation_time)

    if dump_results {
        output_filename_shaker_starts_builder: strings.Builder
        fmt.sbprintf(&output_filename_shaker_starts_builder, "%s%s", directory, "shaker_starts.txt")
        output_filename_shaker_starts := strings.to_string(output_filename_shaker_starts_builder)

        output_file_shaker_starts, err := os.open(output_filename_shaker_starts, os.O_RDWR | os.O_CREATE)
        defer os.close(output_file_shaker_starts)

        assert(err == os.ERROR_NONE)
        for shaker_start in shaker_starts {
            fmt.fprintf(output_file_shaker_starts, "%.10f\n", shaker_start)
        }
    }
}

compaction_analysis :: proc(sim_output_directory: string) {
    using dem
    results := new(Experiment)
    defer free(results)
    read_experiment(sim_output_directory, results)

    Density_ratios_with_time :: struct {
        density: f64,
        time:    f64,
    }

    density_ratios_max_height: [dynamic]Density_ratios_with_time

    volume_all_spheres: f64
    for sphere in results.data[0].spheres {
        using sphere
        volume_all_spheres += f64(4. / 3.) * math.PI * math.pow(radius, 3)
    }

    // ignore frame 0 since the spheres fill up the whole box, making the density very small
    for current_frame: int = 1; current_frame < len(results.data) - 1; current_frame += 1 {
        using results
        spheres := data[current_frame].spheres
        walls := data[current_frame].walls
        current_time := data[current_frame].current_time
        radius := spheres[0].radius

        length_box_x := length(walls[1].center_position - walls[0].center_position)
        length_box_y := length(walls[3].center_position - walls[2].center_position)
        length_box_z := length(walls[5].center_position - walls[4].center_position)
        length_box := [3]f64{length_box_x, length_box_y, length_box_z}

        {     // max_height
            max_height := -math.F64_MAX
            for sphere in spheres {
                using sphere
                if position.y >= max_height {
                    max_height = position.y
                }
            }

            heigth_pile := max_height + abs(walls[BOTTOM_WALL_ID].center_position.y) + radius
            volume_pile := heigth_pile * length_box.x * length_box.z
            append(
                &density_ratios_max_height,
                Density_ratios_with_time{density = volume_all_spheres / volume_pile, time = current_time},
            )
        }
    }

    output_file_max_height, err := os.open(
        fmt.tprintf("%s%s", sim_output_directory, "density.txt"),
        os.O_RDWR | os.O_CREATE,
    )
    assert(err == os.ERROR_NONE)
    for density_ratio in density_ratios_max_height {
        using density_ratio
        fmt.fprintf(output_file_max_height, "%.10f\t%.10f\n", time, 100. * density)
    }

}
