package dem

import "core:fmt"
import "core:strings"
import "core:sort"
import "core:slice"
import "core:math"
import "core:math/rand"
import "core:math/linalg"
import mu "vendor:microui"
import rl "vendor:raylib"

SCALING_FACTOR: f64 : 1000.0
BENDING_EXPERIMENT: bool : true

RenderState :: struct {
    selection:               Selection_state,
    pause:                   bool,
    stepping:                bool,
    render_forces:           bool,
    render_angular_velocity: bool,
    render_collisions:       bool,
    render_transparent:      bool,
}

Physics_opts :: struct {
    sim_speed:             int,
    id_selected_sphere:    i32,
    neighbor_particle_ids: [dynamic]i32,
}

Selection_state :: enum {
    no_selection,
    possible_selection,
    confirmed_selection,
}

render_state := RenderState {
    selection               = .no_selection,
    pause                   = false,
    stepping                = false,
    render_forces           = true,
    render_angular_velocity = true,
    render_collisions       = true,
}

MAX_SIM_SPEED :: 1000

DEBUG_COLLISIONS :: false

Colors := [?]rl.Color{
    rl.GOLD,
    rl.ORANGE,
    rl.RED,
    rl.MAROON,
    rl.GREEN,
    rl.LIME,
    rl.DARKGREEN,
    rl.SKYBLUE,
    rl.BLUE,
    rl.DARKBLUE,
    rl.PURPLE,
    rl.VIOLET,
    rl.DARKPURPLE,
    rl.BEIGE,
    rl.BROWN,
    rl.DARKBROWN,
}

Total_time: f64

visualize_experiment :: proc(experiment: ^Experiment) {
    // raylib window
    using rl
    SetTraceLogLevel(.NONE)
    SetConfigFlags({.VSYNC_HINT, .MSAA_4X_HINT, .WINDOW_HIGHDPI, .WINDOW_RESIZABLE})
    InitWindow(1200, 700, "hihi test")
    SCREEN_WIDTH := GetScreenWidth()
    SCREEN_HEIGHT := GetScreenHeight()
    /* ToggleFullscreen() */
    defer CloseWindow()

    // raylib camera
    camera := Camera{}
    camera.position = Vector3{15., 0, 1.}
    camera.target = Vector3{0, 0, 0}
    camera.up = Vector3{0, 1, 0}
    camera.fovy = 45.0
    camera.projection = .PERSPECTIVE
    SetCameraMode(camera, .FREE)

    physics_opts := Physics_opts {
	sim_speed          = 1,
	id_selected_sphere = 0,
    }

    // microui
    pixels := make([][4]u8, mu.DEFAULT_ATLAS_WIDTH * mu.DEFAULT_ATLAS_HEIGHT)
    for alpha, i in mu.default_atlas_alpha {
	pixels[i] = {0xff, 0xff, 0xff, alpha}
    }
    defer delete(pixels)

    image := rl.Image {
	data    = raw_data(pixels),
	width   = mu.DEFAULT_ATLAS_WIDTH,
	height  = mu.DEFAULT_ATLAS_HEIGHT,
	mipmaps = 1,
	format  = .UNCOMPRESSED_R8G8B8A8,
    }
    ui_state.atlas_texture = rl.LoadTextureFromImage(image)
    defer rl.UnloadTexture(ui_state.atlas_texture)

    ctx := &ui_state.mu_ctx
    mu.init(ctx)
    ctx.text_width = rl_text_width
    ctx.text_height = rl_text_height
    ctx.style.spacing += 5
    experiment_frame: int
    using experiment
    using cell_context
    {
	for cell in &cells {
	    clear(&cell.particle_ids)
	}
    }

    old_angles: [dynamic]f64

    // wall stuff
    length_box := get_length_box(experiment.data[0].walls)
    
    // Main Loop
    SetTargetFPS(60)
    for !WindowShouldClose() {
	@(static)
	tick: u128 = 0
	defer {tick += 1;render_state.stepping = false}

	if experiment_frame >= len(experiment.data) do experiment_frame = 0

	spheres := experiment.data[experiment_frame].spheres
	walls := experiment.data[experiment_frame].walls
	Total_time = experiment.data[experiment_frame].current_time

	if experiment_frame == 0 do for _ in 0 ..< len(spheres) do append(&old_angles, 0.)

	mu_input(ctx)
	{ 	// INPUTS
	    using render_state, physics_opts
	    if IsKeyPressed(.SPACE) do pause = ~pause
	    if IsKeyPressed(.T) do render_transparent = ~render_transparent
	    if IsKeyPressed(.R) do experiment_frame = 0
	    if IsKeyPressed(.Q) do break
	    if !pause do experiment_frame += sim_speed
	    // step through the sim
	    if pause && IsKeyDown(.EQUAL) {
		stepping = true
		if tick % 2 == 0 do experiment_frame += 1
	    }
	    if IsKeyDown(.P) do sim_speed += 2
	    if IsKeyDown(.O) {
		sim_speed -= 2
		if sim_speed <= 0 do sim_speed = 1
	    }
	    if IsKeyDown(.O) && IsKeyDown(.P) {
		sim_speed = 5
	    }

	    mouse_ray := GetMouseRay(GetMousePosition(), camera)
	    switch selection {
	    case .possible_selection:
		if IsMouseButtonPressed(.LEFT) {
		    selection = .confirmed_selection
		    break
		}
		fallthrough

	    case .no_selection:
		context.allocator = context.temp_allocator
		distance_sphere_ids := make(map[f32]i32)

		for sphere, id in &spheres {
		    using sphere
		    ray_collision := GetRayCollisionSphere(
			mouse_ray,
			vec3_to_rl(position * SCALING_FACTOR),
			auto_cast (radius * SCALING_FACTOR),
		    )
		    if ray_collision.hit && ray_collision.distance <= math.F32_MAX {
			distance_sphere_ids[ray_collision.distance] = i32(id)
		    }
		}
		if len(distance_sphere_ids) > 0 {
		    slice.map_keys(distance_sphere_ids)
		    selected_sphere_ids, err := slice.map_values(distance_sphere_ids)
		    selection = .possible_selection
		    id_selected_sphere = selected_sphere_ids[0]
		} else do selection = .no_selection

	    case .confirmed_selection:
		if IsMouseButtonPressed(.RIGHT) do selection = .possible_selection
	    }

	    if len(spheres) == 1 {
		id_selected_sphere = 0
	    }
	}

	// Render
	BeginDrawing()
	defer EndDrawing()

	ClearBackground(RAYWHITE)
	{ 	// 3D stuff
	    BeginMode3D(camera)
	    defer EndMode3D()

	    render_spheres(spheres[:], physics_opts.id_selected_sphere, old_angles[:], &render_state)
	    //Box
	    fmt.printf("\r %.10f", walls[2].center_position.y)
	    DrawCubeWires(
		position = Vector3{
		    0,
		    f32(SCALING_FACTOR) * (auto_cast walls[2].center_position.y + auto_cast length_box.y / 2),
		    0,
		},
		width = auto_cast (length_box.x * SCALING_FACTOR),
		height = auto_cast (length_box.y * SCALING_FACTOR),
		length = auto_cast (length_box.z * SCALING_FACTOR),
		color = BLUE,
	    )

	    // neighbor cells, particles and collisions
	    clear(&physics_opts.neighbor_particle_ids)
	    if render_state.selection == .confirmed_selection && render_state.render_collisions {
		using physics_opts
		using selected_sphere := spheres[id_selected_sphere]

		cell_id := position_to_cell_id(position, cell_context, walls)
		neighbor_ids := get_real_neighbor_cell_ids(cell_context, cell_id, experiment.experiment_type)
		render_cells(cell_context, neighbor_ids[:], walls)

		{ 	// get neighbor particle ids
		    for neighbor_id in neighbor_ids {
			cell := cells[neighbor_id]
			if len(cell.particle_ids) > 0 do for particle_id in cell.particle_ids {
			    append(&neighbor_particle_ids, particle_id)
			}
		    }

		    if len(cells[cell_id].particle_ids) > 1 {
			for particle_id in cells[cell_id].particle_ids do if particle_id != id_selected_sphere {
			    append(&neighbor_particle_ids, particle_id)
			}
		    }
		}

		// render
		for neighbor_particle_id in neighbor_particle_ids {
		    using neighbor_particle := &spheres[neighbor_particle_id]

		    scaled_position_neighbor := vec3_to_rl(neighbor_particle.position * SCALING_FACTOR)
		    scaled_radius_neighbor := f32(neighbor_particle.radius * SCALING_FACTOR)

		    if measure_delta(&spheres[id_selected_sphere], neighbor_particle) <= 0 {
			DrawSphere(scaled_position_neighbor, scaled_radius_neighbor, BLACK)
		    } else do DrawSphere(scaled_position_neighbor, scaled_radius_neighbor, PINK)
		}
	    }

	    // Axes
	    render_vector(Vector3{auto_cast (SCALING_FACTOR * length_box.x / 2), 0, 0}, 0, BLACK)
	    render_vector(Vector3{0, auto_cast (SCALING_FACTOR * length_box.y / 2), 0}, 0, GREEN)
	    render_vector(Vector3{0, 0, auto_cast (SCALING_FACTOR * length_box.z / 2)}, 0, ORANGE)
	}

	{ 	//microui
	    using render_state, physics_opts
	    mu.begin(ctx)
	    defer {mu.end(ctx);render(ctx)}

	    @(static)
	    opts := mu.Options{.NO_CLOSE, .NO_INTERACT, .NO_RESIZE}

	    if mu.window(ctx, "Sim options", {0, 0, 300, GetScreenHeight() + 100}, opts) {
		if !mouse_in_ui(ctx) do UpdateCamera(&camera)
		using fmt
		mu.layout_row(ctx, {-1})
		mu.checkbox(ctx, "Render force", &render_forces)
		mu.checkbox(ctx, "Render angular velocity", &render_angular_velocity)
		mu.checkbox(ctx, "Render transparent", &render_transparent)
		mu.checkbox(ctx, "Render Collisions", &render_collisions)

		w := ctx.text_width(mu_font(FONT_SIZE), "Sim speed: ")
		mu.layout_row(ctx, {w, -1})
		mu.label(ctx, "Sim speed: ");int_slider(ctx, &sim_speed, 1, MAX_SIM_SPEED, 1, "%.0f")

		mu.layout_row(ctx, {-1})
		mu.label(ctx, tprintf("FPS: %v", GetFPS()))
		mu.label(ctx, tprintf("Total time in secs: %.7f", Total_time))
		mu.label(ctx, tprintf("Total particles: %d", len(spheres)))
		mu.label(ctx, tprintf("current frame: %v", experiment_frame))

		total_energy: f64
		for sphere in spheres {
		    using sphere
		    total_energy +=
			mass * dot(velocity, velocity) / 2. + moment_of_inertia * dot(angular_velocity, angular_velocity) / 2.
		}

		max_kinectic_energy := spheres[0].mass * G * length_box.y

		max_height := -math.F64_MAX
		max_height_id: int
		for sphere, id in spheres {
		    using sphere
		    if position.y >= max_height {
			max_height = position.y
			max_height_id = id
		    }
		}

		radius := spheres[max_height_id].radius

		heigth_pile := max_height + abs(walls[BOTTOM_WALL_ID].center_position.y) + radius
		volume_pile := heigth_pile * length_box.x * length_box.z

		volume_all_spheres: f64
		for sphere in spheres {
		    using sphere
		    volume_all_spheres += f64(4. / 3.) * math.PI * math.pow(radius, 3)
		}
		mu.label(ctx, tprintf("Total energy: %.6f%%", 100. * total_energy / (max_kinectic_energy * f64(len(spheres)))))
		mu.label(ctx, tprintf("Density ratio: %.5f%%", 100. * (volume_all_spheres / volume_pile)))
		mu.label(ctx, tprintf("Max heigth: %.10f", max_height))
		if selection != .no_selection {
		    title_seperator(ctx, "Analysis of particle")
		    using sphere := spheres[id_selected_sphere]

		    mu.layout_row(ctx, {-1})
		    mu.label(ctx, tprintf("len(w) = %.7f", length(angular_velocity)))
		    mu.label(ctx, tprintf("len(f) = %.7f", length(force)))
		    mu.label(ctx, tprintf("len(t) = %.7f", length(torque)))
		    total_velocity := length(velocity)
		    /* mu.label(ctx, tprintf("len(v)/max_speed = %.3f%%", 100 * total_velocity / max_speed)) */

		    // measure vertical component as a percentage of total length of velocity vector
		    E_y_componant := E_y * dot(E_y, velocity)
		    percentage_up := length(E_y_componant) / total_velocity
		    mu.label(ctx, tprintf("velocity vertical: %.2f%%", percentage_up))
		    mu.label(ctx, tprintf("velocity horizontal: %.2f%%", 1.0 - percentage_up))

		    title_seperator(ctx, "Info dump particle")
		    mu.layout_row(ctx, {-1})
		    mu.text(ctx, tprintf("%#v", sphere))
		}
	    }
	}

	// legends
	{
	    legends := make([dynamic]Legend, context.temp_allocator)
	    // Axes
	    append(&legends, Legend{"BLACK IS X", BLACK})
	    append(&legends, Legend{"GREEN IS Y", GREEN})
	    append(&legends, Legend{"ORANGE IS Z", ORANGE})
	    // vectors on selected particle
	    if render_state.selection != .no_selection {
		append(&legends, Legend{"Legend for particles:", BLACK})
		append(&legends, Legend{"Black is velocity", BLACK})
		append(&legends, Legend{"Red is force", RED})
		append(&legends, Legend{"Blue is angular velocity", BLUE})
	    }

	    if render_state.selection == .confirmed_selection && render_state.render_collisions {
		append(&legends, Legend{"Legend for collisions:", BLACK})
		append(&legends, Legend{"Pink for neighbor", PINK})
		append(&legends, Legend{"Black for contact", BLACK})
	    }
	    render_legends(legends[:])
	}

	// render the pause text
	if render_state.pause {
	    text := "PAUSE" if !render_state.stepping else "STEPPING"
		ctext := strings.clone_to_cstring(text, context.temp_allocator)
	    w := MeasureText(ctext, 2 * FONT_SIZE)
	    DrawText(ctext, GetScreenWidth() / 2 - w / 2, 30, 2 * FONT_SIZE, RED)
	}
    }
}

debug_sim_code :: proc() {
    using rl
    SetTraceLogLevel(.NONE)
    SetConfigFlags({.VSYNC_HINT, .MSAA_4X_HINT, .WINDOW_HIGHDPI, .WINDOW_RESIZABLE})
    InitWindow(1200, 700, "hihi test")
    SCREEN_WIDTH := GetScreenWidth()
    SCREEN_HEIGHT := GetScreenHeight()
    defer CloseWindow()

    // raylib camera
    camera := Camera{}
    camera.position = Vector3{15., 0, 1.}
    camera.target = Vector3{0, 0, 0}
    camera.up = Vector3{0, 1, 0}
    camera.fovy = 45.0
    camera.projection = .PERSPECTIVE
    SetCameraMode(camera, .FREE)

    physics_opts := Physics_opts {
	sim_speed          = 250,
	id_selected_sphere = 0,
    }

    // microui
    pixels := make([][4]u8, mu.DEFAULT_ATLAS_WIDTH * mu.DEFAULT_ATLAS_HEIGHT)
    for alpha, i in mu.default_atlas_alpha {
	pixels[i] = {0xff, 0xff, 0xff, alpha}
    }
    defer delete(pixels)

    image := rl.Image {
	data    = raw_data(pixels),
	width   = mu.DEFAULT_ATLAS_WIDTH,
	height  = mu.DEFAULT_ATLAS_HEIGHT,
	mipmaps = 1,
	format  = .UNCOMPRESSED_R8G8B8A8,
    }
    ui_state.atlas_texture = rl.LoadTextureFromImage(image)
    defer rl.UnloadTexture(ui_state.atlas_texture)

    ctx := &ui_state.mu_ctx
    mu.init(ctx)
    ctx.text_width = rl_text_width
    ctx.text_height = rl_text_height
    ctx.style.spacing += 5

    //physics
    total_chains := 1
    total_particles_per_chain := 15
    radius: f64 = 1e-3
    current_time: f64 = 0
    density_glass_beads: f64 = 2500.0
    box_length_x := 4 * 2 * radius
    box_length_z := 32 * 2 * radius
    box_length_y := 32 * 2 * radius
    static_restitution_coeff := 0.8
    length_box := [3]f64{box_length_x, box_length_y, box_length_z}
    k_pile := 1000.

    params := init_params(radius, density_glass_beads, static_restitution_coeff, k_pile, length_box)

    walls := init_walls(length_box, 2 * radius)
    cell_context := init_cell_context(radius, walls)
    using cell_context
    contacts: map[int]Contact
    //make spheres
    spheres: [dynamic]Sphere
    for _ in 0 ..< (total_chains * total_particles_per_chain) do append(&spheres, Sphere{})

    chains: [dynamic]Chain
    defer delete(chains)
    for i in 0 ..< total_chains {
	chain := init_chain(
	    walls,
	    radius,
	    density_glass_beads,
	    spheres[i * total_particles_per_chain:(i + 1) * total_particles_per_chain],
	    chains[:],
	    BENDING_EXPERIMENT,
	)
	append(&chains, chain)
    }
    old_angles := make([]f64, total_particles_per_chain * total_chains)
    for value in &old_angles do value = 0.
	// main loop
    SetTargetFPS(60)
    for !WindowShouldClose() {
	@(static)
	tick: u128 = 0
	defer {tick += 1;render_state.stepping = false}

	mu_input(ctx)
	{ 	// INPUTS
	    using render_state, physics_opts
	    if IsKeyPressed(.SPACE) do pause = ~pause
	    if IsKeyPressed(.T) do render_transparent = ~render_transparent
	    if IsKeyPressed(.Q) do break
	    if IsKeyPressed(.R) {
		clear(&chains)
		for i in 0 ..< total_chains {
		    chain := init_chain(
			walls,
			radius,
			density_glass_beads,
			spheres[i * total_particles_per_chain:(i + 1) * total_particles_per_chain],
			chains[:],
			BENDING_EXPERIMENT,
		    )
		    append(&chains, chain)
		}
		current_time = 0
	    }
	    if !pause {
		for _ in 0 ..< sim_speed {
		    physics_update_chain(chains[:], spheres[:], cell_context, walls, &contacts, params)
		}
		current_time += params.dt * f64(sim_speed)
	    }

	    if IsKeyDown(.P) do sim_speed += 2

	    if IsKeyDown(.O) {
		sim_speed -= 2
		if sim_speed <= 0 do sim_speed = 1
	    }

	    if IsKeyDown(.O) && IsKeyDown(.P) {
		sim_speed = 5
	    }

	    mouse_ray := GetMouseRay(GetMousePosition(), camera)
	    switch selection {
	    case .possible_selection:
		if IsMouseButtonPressed(.LEFT) {
		    selection = .confirmed_selection
		    break
		}
		fallthrough

	    case .no_selection:
		context.allocator = context.temp_allocator
		distance_sphere_ids := make(map[f32]i32)

		for sphere, id in &spheres {
		    using sphere
		    ray_collision := GetRayCollisionSphere(
			mouse_ray,
			vec3_to_rl(position * SCALING_FACTOR),
			auto_cast (radius * SCALING_FACTOR),
		    )
		    if ray_collision.hit && ray_collision.distance <= math.F32_MAX {
			distance_sphere_ids[ray_collision.distance] = auto_cast id
		    }
		}
		if len(distance_sphere_ids) > 0 {
		    slice.map_keys(distance_sphere_ids)
		    selected_sphere_ids, err := slice.map_values(distance_sphere_ids)
		    selection = .possible_selection
		    id_selected_sphere = selected_sphere_ids[0]
		} else do selection = .no_selection

	    case .confirmed_selection:
		if IsMouseButtonPressed(.RIGHT) do selection = .possible_selection
	    }

	    if len(spheres) == 1 {
		id_selected_sphere = 0
	    }
	}

	BeginDrawing()
	defer EndDrawing()
	ClearBackground(RAYWHITE)
	{ 	// 3D stuff
	    BeginMode3D(camera)
	    defer EndMode3D()

	    render_chains(chains[:], physics_opts.id_selected_sphere, old_angles[:], &render_state)
	    /* render_spheres(spheres[:], physics_opts.id_selected_sphere, old_angles[:], &render_state) */
	    //Box
	    DrawCubeWires(
		position = Vector3{
		    0,
		    f32(SCALING_FACTOR) * (auto_cast walls[2].center_position.y + auto_cast length_box.y / 2),
		    0,
		},
		width = auto_cast (length_box.x * SCALING_FACTOR),
		height = auto_cast (length_box.y * SCALING_FACTOR),
		length = auto_cast (length_box.z * SCALING_FACTOR),
		color = BLUE,
	    )

	    // neighbor cells, particles and collisions
	    clear(&physics_opts.neighbor_particle_ids)
	    if render_state.selection == .confirmed_selection && render_state.render_collisions {
		using physics_opts
		using selected_sphere := spheres[id_selected_sphere]

		cell_id := position_to_cell_id(position, cell_context, walls)
		neighbor_ids := get_real_neighbor_cell_ids(cell_context, cell_id, .nothing)
		render_cells(cell_context, neighbor_ids[:], walls)

		{ 	// get neighbor particle ids
		    for neighbor_id in neighbor_ids {
			cell := cells[neighbor_id]
			if len(cell.particle_ids) > 0 do for particle_id in cell.particle_ids {
			    append(&neighbor_particle_ids, particle_id)
			}
		    }

		    if len(cells[cell_id].particle_ids) > 1 {
			for particle_id in cells[cell_id].particle_ids do if particle_id != id_selected_sphere {
			    append(&neighbor_particle_ids, particle_id)
			}
		    }
		}

		// render
		for neighbor_particle_id in neighbor_particle_ids {
		    using neighbor_particle := &spheres[neighbor_particle_id]

		    scaled_position_neighbor := vec3_to_rl(neighbor_particle.position * SCALING_FACTOR)
		    scaled_radius_neighbor := f32(neighbor_particle.radius * SCALING_FACTOR)

		    if measure_delta(&spheres[id_selected_sphere], neighbor_particle) <= 0 {
			DrawSphere(scaled_position_neighbor, scaled_radius_neighbor, BLACK)
		    } else do DrawSphere(scaled_position_neighbor, scaled_radius_neighbor, PINK)
		}
	    }

	    // Axes
	    render_vector(Vector3{auto_cast (SCALING_FACTOR * length_box.x / 2), 0, 0}, 0, BLACK)
	    render_vector(Vector3{0, auto_cast (SCALING_FACTOR * length_box.y / 2), 0}, 0, GREEN)
	    render_vector(Vector3{0, 0, auto_cast (SCALING_FACTOR * length_box.z / 2)}, 0, ORANGE)
	}

	{ 	//microui
	    using render_state, physics_opts
	    mu.begin(ctx)
	    defer {mu.end(ctx);render(ctx)}

	    @(static)
	    opts := mu.Options{.NO_CLOSE, .NO_INTERACT, .NO_RESIZE}

	    if mu.window(ctx, "Sim options", {0, 0, 300, GetScreenHeight() + 100}, opts) {
		if !mouse_in_ui(ctx) do UpdateCamera(&camera)
		using fmt
		mu.layout_row(ctx, {-1})
		mu.checkbox(ctx, "Render force", &render_forces)
		mu.checkbox(ctx, "Render angular velocity", &render_angular_velocity)
		mu.checkbox(ctx, "Render transparent", &render_transparent)
		mu.checkbox(ctx, "Render Collisions", &render_collisions)

		w := ctx.text_width(mu_font(FONT_SIZE), "Sim speed: ")
		mu.layout_row(ctx, {w, -1})
		mu.label(ctx, "Sim speed: ");int_slider(ctx, &sim_speed, 1, MAX_SIM_SPEED, 1, "%.0f")

		mu.layout_row(ctx, {-1})
		mu.label(ctx, tprintf("FPS: %v", GetFPS()))
		mu.label(ctx, tprintf("Total time in secs: %.7f", current_time))
		mu.label(ctx, tprintf("Total particles: %d", len(spheres)))

		total_energy: f64
		for sphere in spheres {
		    using sphere
		    total_energy +=
			mass * dot(velocity, velocity) / 2. + moment_of_inertia * dot(angular_velocity, angular_velocity) / 2.
		}

		max_kinectic_energy := spheres[0].mass * G * length_box.y

		max_height := -math.F64_MAX
		max_height_id: int
		for sphere, id in spheres {
		    using sphere
		    if position.y >= max_height {
			max_height = position.y
			max_height_id = id
		    }
		}

		radius := spheres[max_height_id].radius

		heigth_pile := max_height + abs(walls[BOTTOM_WALL_ID].center_position.y) + radius
		volume_pile := heigth_pile * length_box.x * length_box.z

		volume_all_spheres: f64
		for sphere in spheres {
		    using sphere
		    volume_all_spheres += f64(4. / 3.) * math.PI * math.pow(radius, 3)
		}
		mu.label(ctx, tprintf("Total energy: %.6f%%", 100. * total_energy / (max_kinectic_energy * f64(len(spheres)))))
		mu.label(ctx, tprintf("Density ratio: %.5f%%", 100. * (volume_all_spheres / volume_pile)))
		mu.label(ctx, tprintf("Max heigth: %.10f", max_height))
		if selection != .no_selection {
		    title_seperator(ctx, "Analysis of particle")
		    using sphere := spheres[id_selected_sphere]

		    mu.layout_row(ctx, {-1})
		    mu.label(ctx, tprintf("len(w) = %.7f", length(angular_velocity)))
		    mu.label(ctx, tprintf("len(f) = %.7f", length(force)))
		    mu.label(ctx, tprintf("len(t) = %.7f", length(torque)))
		    total_velocity := length(velocity)
		    /* mu.label(ctx, tprintf("len(v)/max_speed = %.3f%%", 100 * total_velocity / max_speed)) */

		    // measure vertical component as a percentage of total length of velocity vector
		    E_y_componant := E_y * dot(E_y, velocity)
		    percentage_up := length(E_y_componant) / total_velocity
		    mu.label(ctx, tprintf("velocity vertical: %.2f%%", percentage_up))
		    mu.label(ctx, tprintf("velocity horizontal: %.2f%%", 1.0 - percentage_up))

		    title_seperator(ctx, "Info dump particle")
		    mu.layout_row(ctx, {-1})
		    mu.text(ctx, tprintf("%#v", sphere))
		}
	    }
	}

	// legends
	{
	    legends := make([dynamic]Legend, context.temp_allocator)
	    // Axes
	    append(&legends, Legend{"BLACK IS X", BLACK})
	    append(&legends, Legend{"GREEN IS Y", GREEN})
	    append(&legends, Legend{"ORANGE IS Z", ORANGE})
	    // vectors on selected particle
	    if render_state.selection != .no_selection {
		append(&legends, Legend{"Legend for particles:", BLACK})
		append(&legends, Legend{"Black is velocity", BLACK})
		append(&legends, Legend{"Red is force", RED})
		append(&legends, Legend{"Blue is angular velocity", BLUE})
	    }

	    if render_state.selection == .confirmed_selection && render_state.render_collisions {
		append(&legends, Legend{"Legend for collisions:", BLACK})
		append(&legends, Legend{"Pink for neighbor", PINK})
		append(&legends, Legend{"Black for contact", BLACK})
	    }
	    render_legends(legends[:])
	}

	// render the pause text
	if render_state.pause {
	    text := "PAUSE" if !render_state.stepping else "STEPPING"
		ctext := strings.clone_to_cstring(text, context.temp_allocator)
	    w := MeasureText(ctext, 2 * FONT_SIZE)
	    DrawText(ctext, GetScreenWidth() / 2 - w / 2, 30, 2 * FONT_SIZE, RED)
	}
    }
}


Legend :: struct {
    txt:   string,
    color: rl.Color,
}

render_legends :: proc(legends: []Legend) {
    y: i32 = 10
    for legend in legends {
	using legend, rl
	if txt == "Legend for particles:" do y += 30
	if txt == "Legend for collisions:" do y += 30
	cstr := strings.clone_to_cstring(txt, context.temp_allocator)
	size := MeasureTextEx(GetFontDefault(), cstr, FONT_SIZE, 1)
	DrawText(cstr, GetScreenWidth() - i32(size.x) - 25, y, FONT_SIZE, color)
	y += i32(size.y)
    }
}

render_spheres :: proc(
    spheres: []Sphere,
    id_of_selected_sphere: i32,
    old_angles: []f64,
    render_state: ^RenderState,
    is_chain := false,
    id_of_chain := 0,
) {
    using rl
    using render_state
    spheres := spheres
    for sphere, id in &spheres {
	using sphere

	angle, axis := linalg.angle_axis_from_quaternion_f64(auto_cast orientation)
	angle = math.to_degrees(angle)

	if angle <= 60 && old_angles[id] > 280 do angle = math.TAU * math.DEG_PER_RAD - angle
	old_angles[id] = angle

	rlPushMatrix()
	rlScalef(auto_cast SCALING_FACTOR, auto_cast SCALING_FACTOR, auto_cast SCALING_FACTOR)
	rlTranslatef(auto_cast position.x, auto_cast position.y, auto_cast position.z)
	if orientation != 1 do rlRotatef(auto_cast angle, auto_cast axis.x, auto_cast axis.y, auto_cast axis.z)

	id_color := id_of_chain if is_chain else id
	switch selection {
	case .no_selection:
	    DrawSphereEx(0, auto_cast radius, 5, 10, Colors[id_color % len(Colors)])
	    DrawSphereWires(0, auto_cast radius, 5, 10, BLACK)
	    rlPopMatrix()
	case .possible_selection:
	    DrawSphereEx(0, auto_cast radius, 5, 10, Colors[id_color % len(Colors)])
	    if auto_cast id == id_of_selected_sphere {
		DrawSphereWires(0, auto_cast radius, 5, 10, PINK)
		rlPopMatrix()
		render_vectors_on_sphere(render_state, &sphere)
	    } else {
		DrawSphereWires(0, auto_cast radius, 5, 10, BLACK)
		rlPopMatrix()
	    }

	case .confirmed_selection:
	    if auto_cast id == id_of_selected_sphere {
		if !render_transparent do DrawSphereEx(0, auto_cast radius, 5, 10, Colors[id % len(Colors)])
		DrawSphereWires(0, auto_cast radius, 5, 10, YELLOW)
		rlPopMatrix()
		render_vectors_on_sphere(render_state, &sphere)
	    } else {
		DrawSphereEx(0, auto_cast radius, 5, 10, Colors[id_color % len(Colors)])
		DrawSphereWires(0, auto_cast radius, 5, 10, BLACK)
		rlPopMatrix()
	    }
	}

    }
}

render_chains :: proc(chains: []Chain, id_of_selected_sphere: i32, old_angles: []f64, render_state: ^RenderState) {
    for chain, id in chains {
	render_spheres(chain.spheres, id_of_selected_sphere, old_angles[:], render_state, true, id)
    }
}

random_unit_vector :: proc() -> vec3 {
    using math
    theta := rand.float64_range(0, PI)
    phi := rand.float64_range(0, 2 * PI)
    return vec3{sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)}
}

render_vectors_on_sphere :: proc(render_state: ^RenderState, sphere: ^Sphere) {
    using sphere, render_state

    scaled_position := vec3_to_rl((SCALING_FACTOR * position))
    unit_velocity := vec3_to_rl(normalize(velocity))
    unit_force := vec3_to_rl(normalize(force))
    unit_angular_velocity := vec3_to_rl(normalize(angular_velocity))

    render_vector(unit_velocity, scaled_position, rl.BLACK)
    if render_forces do render_vector(unit_force, scaled_position, rl.RED)
    if render_angular_velocity do render_vector(unit_angular_velocity, scaled_position, rl.BLUE)
}

render_vector :: proc(unit_vector, position: rl.Vector3, color: rl.Color) {
    rl.DrawLine3D(position, position + unit_vector * 1.2, color)
    rl.DrawCylinderEx(position + unit_vector * 0.85, position + unit_vector * 1.2, 0.20, 0, 2, color)
}

sphere_is_selected :: proc(mouse_ray: rl.Ray, scaling_factor: f64, spheres: []Sphere, id_of_sphere: int) -> bool {
    spheres := spheres
    sphere := spheres[id_of_sphere]
    result :=
	rl.GetRayCollisionSphere(
	    mouse_ray,
	    vec3_to_rl(sphere.position * scaling_factor),
	    auto_cast (sphere.radius * scaling_factor),
	).hit
    return result
}

render_all_cells :: proc(cell_context: ^Cell_context, walls: []Wall) {
    using cell_context
    using rl
    for _, id in cells {
	DrawCubeWires(
	    position = vec3_to_rl(cell_coords_unique_id(auto_cast id, cell_context, walls)) * f32(SCALING_FACTOR),
	    width = auto_cast (cell_length * SCALING_FACTOR),
	    height = auto_cast (cell_length * SCALING_FACTOR),
	    length = auto_cast (cell_length * SCALING_FACTOR),
	    color = BLACK,
	)
    }
}

render_cells :: proc(cell_context: ^Cell_context, cell_ids: []i32, walls: []Wall) {
    using rl
    using cell_context
    for neighbor_id in cell_ids do if neighbor_id != 0 {
	DrawCubeWires(
	    position = vec3_to_rl(cell_coords_unique_id(neighbor_id, cell_context, walls)) *
		f32(SCALING_FACTOR),
	    width = auto_cast (cell_length * SCALING_FACTOR),
	    height = auto_cast (cell_length * SCALING_FACTOR),
	    length = auto_cast (cell_length * SCALING_FACTOR),
	    color = BLUE)
    }
}

render_cell :: proc(cell_context: ^Cell_context, cell_id: int, walls: []Wall, color: rl.Color) {
    using rl
    using cell_context
    DrawCube(
	position = vec3_to_rl(cell_coords_unique_id( auto_cast cell_id,cell_context, walls)) *
	    f32(SCALING_FACTOR),
	width = auto_cast (cell_length * SCALING_FACTOR),
	height = auto_cast (cell_length * SCALING_FACTOR),
	length = auto_cast (cell_length * SCALING_FACTOR),
	color = color,
    )

}
