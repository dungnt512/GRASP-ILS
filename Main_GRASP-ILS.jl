using Dates, Random

const EPS = 1e-9
const DEFAULT_INPUT_DIR = raw"D:\OneDrive-ntdxl\prj\school-bus-routing-sciortino-2022\opt_10s"
const DEFAULT_OUTPUT_DIR = joinpath(@__DIR__, "outputs", "opt_10s")
const DEFAULT_TIME_LIMIT_SECONDS = 10.0

mutable struct FastScanner
    data::Vector{UInt8}
    idx::Int
    n::Int
end

function FastScanner(path::AbstractString)
    bytes = read(path)
    return FastScanner(bytes, 1, length(bytes))
end

is_digit_byte(b::UInt8)::Bool = 0x30 <= b <= 0x39

function skip_ws!(fs::FastScanner)
    while fs.idx <= fs.n && fs.data[fs.idx] <= 0x20
        fs.idx += 1
    end
end

function next_int!(fs::FastScanner)::Int
    skip_ws!(fs)
    fs.idx > fs.n && error("Unexpected end of input while reading integer")

    sign = 1
    if fs.data[fs.idx] == 0x2d
        sign = -1
        fs.idx += 1
    end

    value = 0
    while fs.idx <= fs.n && is_digit_byte(fs.data[fs.idx])
        value = 10 * value + Int(fs.data[fs.idx] - 0x30)
        fs.idx += 1
    end
    return sign * value
end

function next_float!(fs::FastScanner)::Float64
    skip_ws!(fs)
    fs.idx > fs.n && error("Unexpected end of input while reading float")

    sign = 1.0
    if fs.data[fs.idx] == 0x2d
        sign = -1.0
        fs.idx += 1
    end

    value = 0.0
    while fs.idx <= fs.n && is_digit_byte(fs.data[fs.idx])
        value = 10.0 * value + Float64(fs.data[fs.idx] - 0x30)
        fs.idx += 1
    end

    if fs.idx <= fs.n && fs.data[fs.idx] == 0x2e
        fs.idx += 1
        scale = 0.1
        while fs.idx <= fs.n && is_digit_byte(fs.data[fs.idx])
            value += scale * Float64(fs.data[fs.idx] - 0x30)
            scale *= 0.1
            fs.idx += 1
        end
    end

    if fs.idx <= fs.n && (fs.data[fs.idx] == 0x45 || fs.data[fs.idx] == 0x65)
        fs.idx += 1
        exp_sign = 1
        if fs.idx <= fs.n && fs.data[fs.idx] == 0x2d
            exp_sign = -1
            fs.idx += 1
        elseif fs.idx <= fs.n && fs.data[fs.idx] == 0x2b
            fs.idx += 1
        end

        exponent = 0
        while fs.idx <= fs.n && is_digit_byte(fs.data[fs.idx])
            exponent = 10 * exponent + Int(fs.data[fs.idx] - 0x30)
            fs.idx += 1
        end
        value *= 10.0 ^ (exp_sign * exponent)
    end

    return sign * value
end

struct WalkOption
    stop::Int
    distance::Float64
    time::Float64
end

struct ProblemData
    stop_count::Int
    address_count::Int
    walk_count::Int
    max_walk_distance::Float64
    max_journey_time::Float64
    capacity::Int
    sec_per_passenger::Float64
    sec_per_stop::Float64
    passengers::Vector{Int}
    drive_time::Matrix{Float64}
    walk_options::Vector{Vector{WalkOption}}
end

struct StopChunk
    stop::Int
    weight::Int
end

mutable struct Route
    stops::Vector{Int}
    weights::Vector{Int}
end

struct EvaluationResult
    valid::Bool
    buses::Int
    objective::Float64
    message::String
end

copy_route(route::Route)::Route = Route(copy(route.stops), copy(route.weights))
copy_routes(routes::Vector{Route})::Vector{Route} = [copy_route(route) for route in routes]

function read_opt10s_instance(path::AbstractString)::ProblemData
    fs = FastScanner(path)

    stop_count = next_int!(fs)
    address_count = next_int!(fs)
    walk_count = next_int!(fs)
    next_float!(fs) # m_e is not used by this heuristic.
    max_walk_distance = next_float!(fs)
    max_journey_minutes = next_float!(fs)
    capacity = next_int!(fs)
    sec_per_passenger = next_float!(fs)
    sec_per_stop = next_float!(fs)

    for _ in 1:stop_count
        next_float!(fs)
        next_float!(fs)
    end

    passengers = Vector{Int}(undef, address_count)
    for a in 1:address_count
        next_float!(fs)
        next_float!(fs)
        passengers[a] = next_int!(fs)
    end

    for _ in 1:(stop_count * stop_count)
        next_float!(fs)
    end

    drive_time = Matrix{Float64}(undef, stop_count, stop_count)
    for i in 1:stop_count
        for j in 1:stop_count
            drive_time[i, j] = next_float!(fs)
        end
    end

    walk_options = Vector{Vector{WalkOption}}(undef, address_count)
    for a in 1:address_count
        option_count = next_int!(fs)
        options = WalkOption[]
        sizehint!(options, option_count)
        for _ in 1:option_count
            stop = next_int!(fs)
            distance = next_float!(fs)
            walk_time = next_float!(fs)
            if 0 <= stop < stop_count
                push!(options, WalkOption(stop, distance, walk_time))
            end
        end
        walk_options[a] = options
    end

    return ProblemData(
        stop_count,
        address_count,
        walk_count,
        max_walk_distance,
        max_journey_minutes * 60.0,
        capacity,
        sec_per_passenger,
        sec_per_stop,
        passengers,
        drive_time,
        walk_options,
    )
end

route_load(route::Route)::Int = sum(route.weights)

function route_time(data::ProblemData, route::Route)::Float64
    isempty(route.stops) && return 0.0

    total = data.drive_time[route.stops[end] + 1, 1]
    for i in 2:length(route.stops)
        total += data.drive_time[route.stops[i - 1] + 1, route.stops[i] + 1]
    end
    for weight in route.weights
        total += data.sec_per_stop + data.sec_per_passenger * weight
    end
    return total
end

function max_single_stop_load(data::ProblemData, stop::Int)::Int
    travel_back = data.drive_time[stop + 1, 1]
    if data.sec_per_passenger <= EPS
        return travel_back + data.sec_per_stop <= data.max_journey_time + EPS ? data.capacity : 0
    end

    by_time = floor(Int, (data.max_journey_time - travel_back - data.sec_per_stop + EPS) / data.sec_per_passenger)
    return max(0, min(data.capacity, by_time))
end

function better_walk_option(a::WalkOption, b::WalkOption)::Bool
    if abs(a.distance - b.distance) > EPS
        return a.distance < b.distance
    end
    if abs(a.time - b.time) > EPS
        return a.time < b.time
    end
    return a.stop < b.stop
end

function choose_assignments(
    data::ProblemData,
    rng::Random.AbstractRNG = Random.default_rng();
    randomized::Bool = false,
)::Tuple{Vector{Int}, Vector{Int}}
    single_stop_limits = [max_single_stop_load(data, stop) for stop in 0:(data.stop_count - 1)]
    assignment = Vector{Int}(undef, data.address_count)
    demand = zeros(Int, data.stop_count)

    feasible_by_address = Vector{Vector{WalkOption}}(undef, data.address_count)
    for a in 1:data.address_count
        feasible_options = WalkOption[]
        for option in data.walk_options[a]
            feasible_stop =
                option.stop != 0 &&
                option.distance <= data.max_walk_distance + EPS &&
                single_stop_limits[option.stop + 1] > 0

            if feasible_stop
                push!(feasible_options, option)
            end
        end

        if isempty(feasible_options)
            error("Address $(a - 1) has no non-depot feasible walking stop")
        end

        sort!(feasible_options; by = option -> (option.distance, option.time, option.stop))
        feasible_by_address[a] = feasible_options
    end

    address_order = collect(1:data.address_count)
    sort!(
        address_order;
        by = a -> (length(feasible_by_address[a]), -data.passengers[a], feasible_by_address[a][1].distance),
    )
    randomized && shuffle!(rng, address_order)

    for a in address_order
        passenger_count = data.passengers[a]
        capacity_feasible = [
            option for option in feasible_by_address[a] if demand[option.stop + 1] + passenger_count <= single_stop_limits[option.stop + 1]
        ]
        candidates = isempty(capacity_feasible) ? feasible_by_address[a] : capacity_feasible

        sort!(candidates; by = option -> (demand[option.stop + 1], option.distance, option.time, option.stop))
        selected = candidates[1]
        if randomized && length(candidates) > 1
            candidate_count = min(length(candidates), 6)
            selected = candidates[rand(rng, 1:candidate_count)]
        end

        assignment[a] = selected.stop
        demand[selected.stop + 1] += data.passengers[a]
    end

    return assignment, demand
end

function split_demands(data::ProblemData, demand::Vector{Int})::Vector{StopChunk}
    chunks = StopChunk[]
    for stop_index in 1:data.stop_count
        remaining = demand[stop_index]
        remaining <= 0 && continue

        stop = stop_index - 1
        limit = max_single_stop_load(data, stop)
        limit <= 0 && error("Stop $stop cannot be served within the journey-time limit")

        while remaining > 0
            weight = min(remaining, limit)
            push!(chunks, StopChunk(stop, weight))
            remaining -= weight
        end
    end
    return chunks
end

function find_stop(route::Route, stop::Int)::Int
    for i in eachindex(route.stops)
        route.stops[i] == stop && return i
    end
    return 0
end

function insert_chunk(route::Route, chunk::StopChunk, pos::Int)::Route
    candidate = copy_route(route)
    insert!(candidate.stops, pos, chunk.stop)
    insert!(candidate.weights, pos, chunk.weight)
    return candidate
end

function add_chunk_to_existing_stop(route::Route, chunk::StopChunk, pos::Int)::Route
    candidate = copy_route(route)
    candidate.weights[pos] += chunk.weight
    return candidate
end

function best_insert_or_add(data::ProblemData, route::Route, chunk::StopChunk)::Tuple{Bool, Route}
    if route_load(route) + chunk.weight > data.capacity
        return false, route
    end

    best_route = route
    best_time = Inf
    existing_pos = find_stop(route, chunk.stop)

    if existing_pos != 0
        candidate = add_chunk_to_existing_stop(route, chunk, existing_pos)
        candidate_time = route_time(data, candidate)
        if candidate_time <= data.max_journey_time + EPS
            return true, candidate
        end
        return false, route
    end

    for pos in 1:(length(route.stops) + 1)
        candidate = insert_chunk(route, chunk, pos)
        candidate_time = route_time(data, candidate)
        if candidate_time <= data.max_journey_time + EPS && candidate_time < best_time
            best_route = candidate
            best_time = candidate_time
        end
    end

    return isfinite(best_time), best_route
end

function compact_route(route::Route)::Route
    positions = Dict{Int, Int}()
    stops = Int[]
    weights = Int[]

    for i in eachindex(route.stops)
        stop = route.stops[i]
        weight = route.weights[i]

        if haskey(positions, stop)
            weights[positions[stop]] += weight
        else
            positions[stop] = length(stops) + 1
            push!(stops, stop)
            push!(weights, weight)
        end
    end

    return Route(stops, weights)
end

function is_route_feasible(data::ProblemData, route::Route)::Bool
    isempty(route.stops) && return false
    route_load(route) <= data.capacity || return false
    route_time(data, route) <= data.max_journey_time + EPS || return false
    length(unique(route.stops)) == length(route.stops)
end

function append_compact_segment!(
    positions::Dict{Int, Int},
    stops::Vector{Int},
    weights::Vector{Int},
    source_stops::Vector{Int},
    source_weights::Vector{Int},
    reversed::Bool,
)
    indices = reversed ? reverse(eachindex(source_stops)) : eachindex(source_stops)
    for idx in indices
        stop = source_stops[idx]
        weight = source_weights[idx]

        if haskey(positions, stop)
            weights[positions[stop]] += weight
        else
            positions[stop] = length(stops) + 1
            push!(stops, stop)
            push!(weights, weight)
        end
    end
end

function compact_merge_route(
    first::Route,
    second::Route;
    reverse_first::Bool = false,
    reverse_second::Bool = false,
)::Route
    positions = Dict{Int, Int}()
    stops = Int[]
    weights = Int[]
    sizehint!(stops, length(first.stops) + length(second.stops))
    sizehint!(weights, length(first.weights) + length(second.weights))

    append_compact_segment!(positions, stops, weights, first.stops, first.weights, reverse_first)
    append_compact_segment!(positions, stops, weights, second.stops, second.weights, reverse_second)

    return Route(stops, weights)
end

function best_merge_candidate(data::ProblemData, route_a::Route, route_b::Route)::Tuple{Bool, Route, Float64}
    base_time = route_time(data, route_a) + route_time(data, route_b)
    best_route = Route(Int[], Int[])
    best_saving = -Inf

    for case in 1:4
        candidate =
            case == 1 ? compact_merge_route(route_a, route_b) :
            case == 2 ? compact_merge_route(route_b, route_a) :
            case == 3 ? compact_merge_route(route_a, route_b; reverse_first = true) :
            compact_merge_route(route_a, route_b; reverse_second = true)

        is_route_feasible(data, candidate) || continue

        saving = base_time - route_time(data, candidate)
        if saving > best_saving
            best_route = candidate
            best_saving = saving
        end
    end

    return isfinite(best_saving), best_route, best_saving
end

function build_initial_routes(
    data::ProblemData,
    chunks::Vector{StopChunk},
    rng::Random.AbstractRNG = Random.default_rng();
    randomized::Bool = false,
    deadline::Float64 = Inf,
)::Vector{Route}
    routes = [Route([chunk.stop], [chunk.weight]) for chunk in chunks]
    for route in routes
        is_route_feasible(data, route) ||
            error("Cannot create a feasible one-stop route for stop $(route.stops[1])")
    end

    while length(routes) > 1 && time() < deadline
        candidates = Tuple{Float64, Int, Int}[]
        for i in 1:(length(routes) - 1)
            time() >= deadline && break
            for j in (i + 1):length(routes)
                time() >= deadline && break
                ok, _, saving = best_merge_candidate(data, routes[i], routes[j])
                ok && push!(candidates, (saving, i, j))
            end
        end

        isempty(candidates) && break
        sort!(candidates; by = item -> item[1], rev = true)

        selected = candidates[1]
        if randomized && length(candidates) > 1
            saving_min = candidates[end][1]
            saving_max = candidates[1][1]
            alpha = rand(rng)
            threshold = saving_max - alpha * (saving_max - saving_min)
            rcl = [candidate for candidate in candidates if candidate[1] >= threshold]
            selected = rcl[rand(rng, 1:length(rcl))]
        end

        _, i, j = selected
        ok, merged_route, _ = best_merge_candidate(data, routes[i], routes[j])
        ok || continue

        routes[i] = merged_route
        deleteat!(routes, j)
    end

    return routes
end

function improve_route_order!(data::ProblemData, route::Route, deadline::Float64 = Inf)
    length(route.stops) <= 2 && return

    improved = true
    while improved && time() < deadline
        improved = false
        base_time = route_time(data, route)

        for i in 1:(length(route.stops) - 1)
            time() >= deadline && return
            for j in (i + 1):length(route.stops)
                time() >= deadline && return
                route.stops[i:j] = reverse(route.stops[i:j])
                route.weights[i:j] = reverse(route.weights[i:j])
                new_time = route_time(data, route)

                if new_time + EPS < base_time
                    base_time = new_time
                    improved = true
                else
                    route.stops[i:j] = reverse(route.stops[i:j])
                    route.weights[i:j] = reverse(route.weights[i:j])
                end
            end
        end
    end
end

function improve_all_route_orders!(data::ProblemData, routes::Vector{Route}, deadline::Float64 = Inf)
    for route in routes
        time() >= deadline && return
        improve_route_order!(data, route, deadline)
    end
end

function try_absorb_route(data::ProblemData, target::Route, donor::Route)::Tuple{Bool, Route}
    candidate = copy_route(target)
    donor_chunks = [StopChunk(donor.stops[i], donor.weights[i]) for i in eachindex(donor.stops)]
    sort!(donor_chunks; by = chunk -> -chunk.weight)

    for chunk in donor_chunks
        ok, updated = best_insert_or_add(data, candidate, chunk)
        ok || return false, target
        candidate = updated
    end

    return true, candidate
end

function reduce_route_count!(data::ProblemData, routes::Vector{Route}, deadline::Float64 = Inf)
    changed = true
    while changed && time() < deadline
        changed = false
        sort!(routes; by = route -> route_load(route), rev = true)

        merged = false
        for donor_index in reverse(eachindex(routes))
            time() >= deadline && return
            for target_index in eachindex(routes)
                time() >= deadline && return
                target_index == donor_index && continue

                ok, candidate = try_absorb_route(data, routes[target_index], routes[donor_index])
                ok || continue

                routes[target_index] = candidate
                deleteat!(routes, donor_index)
                changed = true
                merged = true
                break
            end
            merged && break
        end
    end
end

function perturb_routes(data::ProblemData, routes::Vector{Route}, rng::Random.AbstractRNG)::Vector{Route}
    candidate = copy_routes(routes)
    isempty(candidate) && return candidate

    if rand(rng) < 0.5
        route_index = rand(rng, eachindex(candidate))
        route = candidate[route_index]
        if length(route.stops) >= 3
            i, j = sort(rand(rng, 1:length(route.stops), 2))
            route.stops[i:j] = reverse(route.stops[i:j])
            route.weights[i:j] = reverse(route.weights[i:j])
        end
        return candidate
    end

    length(candidate) < 2 && return candidate
    first_route_index, second_route_index = rand(rng, eachindex(candidate), 2)
    while second_route_index == first_route_index
        second_route_index = rand(rng, eachindex(candidate))
    end

    first_route = candidate[first_route_index]
    second_route = candidate[second_route_index]
    if isempty(first_route.stops) || isempty(second_route.stops)
        return candidate
    end

    first_pos = rand(rng, eachindex(first_route.stops))
    second_pos = rand(rng, eachindex(second_route.stops))

    first_route.stops[first_pos], second_route.stops[second_pos] =
        second_route.stops[second_pos], first_route.stops[first_pos]
    first_route.weights[first_pos], second_route.weights[second_pos] =
        second_route.weights[second_pos], first_route.weights[first_pos]

    if is_route_feasible(data, first_route) && is_route_feasible(data, second_route)
        return candidate
    end

    return copy_routes(routes)
end

function build_solution_once(
    data::ProblemData,
    deadline::Float64,
    rng::Random.AbstractRNG;
    randomized::Bool = false,
)::Tuple{Vector{Route}, Vector{Int}}
    assignment, demand = choose_assignments(data, rng; randomized = randomized)
    chunks = split_demands(data, demand)

    routes = build_initial_routes(data, chunks, rng; randomized = randomized, deadline = deadline)
    improve_all_route_orders!(data, routes, deadline)
    reduce_route_count!(data, routes, deadline)
    improve_all_route_orders!(data, routes, deadline)
    sort!(routes; by = route -> (route_load(route), route_time(data, route)), rev = true)

    return routes, assignment
end

function is_better_evaluation(candidate::EvaluationResult, incumbent::EvaluationResult)::Bool
    candidate.valid || return false
    incumbent.valid || return true

    if candidate.buses != incumbent.buses
        return candidate.buses < incumbent.buses
    end

    return candidate.objective < incumbent.objective - EPS
end

function build_solution(data::ProblemData, time_limit_seconds::Float64)::Tuple{Vector{Route}, Vector{Int}}
    rng = Random.default_rng()
    deadline = time_limit_seconds > 0 ? time() + time_limit_seconds : Inf

    best_routes, best_assignment = build_solution_once(data, deadline, rng; randomized = false)
    best_evaluation = evaluate_solution(data, best_routes, best_assignment)
    best_evaluation.valid || error("Initial solution is invalid: $(best_evaluation.message)")

    if time_limit_seconds <= 0
        return best_routes, best_assignment
    end

    while time() < deadline
        try
            if rand(rng) < 0.25
                candidate_routes, candidate_assignment = build_solution_once(data, deadline, rng; randomized = true)
            else
                candidate_routes = perturb_routes(data, best_routes, rng)
                improve_all_route_orders!(data, candidate_routes, deadline)
                reduce_route_count!(data, candidate_routes, deadline)
                improve_all_route_orders!(data, candidate_routes, deadline)
                candidate_assignment = copy(best_assignment)
            end

            candidate_evaluation = evaluate_solution(data, candidate_routes, candidate_assignment)

            if is_better_evaluation(candidate_evaluation, best_evaluation)
                best_routes = copy_routes(candidate_routes)
                best_assignment = copy(candidate_assignment)
                best_evaluation = candidate_evaluation
            end
        catch err
            GC.gc()
            time() >= deadline && break
            @warn "Ignoring failed randomized restart" exception = err
        end
    end

    return best_routes, best_assignment
end

function has_valid_walk(data::ProblemData, address::Int, stop::Int)::Bool
    for option in data.walk_options[address]
        if option.stop == stop && option.distance <= data.max_walk_distance + EPS
            return true
        end
    end
    return false
end

function evaluate_solution(data::ProblemData, routes::Vector{Route}, assignment::Vector{Int})::EvaluationResult
    length(assignment) == data.address_count ||
        return EvaluationResult(false, -1, Inf, "assignment count does not match address count")

    route_demand = zeros(Int, data.stop_count)
    assigned_demand = zeros(Int, data.stop_count)
    objective = 0.0

    for (route_index, route) in enumerate(routes)
        !isempty(route.stops) ||
            return EvaluationResult(false, -1, Inf, "route $route_index is empty")
        route_load(route) <= data.capacity ||
            return EvaluationResult(false, -1, Inf, "route $route_index exceeds capacity")

        current_route_time = route_time(data, route)
        current_route_time <= data.max_journey_time + EPS ||
            return EvaluationResult(false, -1, Inf, "route $route_index exceeds journey-time limit")
        objective += current_route_time

        used = Set{Int}()
        for i in eachindex(route.stops)
            stop = route.stops[i]
            weight = route.weights[i]

            stop != 0 || return EvaluationResult(false, -1, Inf, "route $route_index contains depot stop 0")
            0 <= stop < data.stop_count ||
                return EvaluationResult(false, -1, Inf, "route $route_index contains invalid stop $stop")
            weight > 0 || return EvaluationResult(false, -1, Inf, "route $route_index contains non-positive load")
            !(stop in used) ||
                return EvaluationResult(false, -1, Inf, "route $route_index repeats stop $stop")

            push!(used, stop)
            route_demand[stop + 1] += weight
        end
    end

    for address in 1:data.address_count
        stop = assignment[address]
        0 <= stop < data.stop_count ||
            return EvaluationResult(false, -1, Inf, "address $(address - 1) is assigned to invalid stop $stop")
        has_valid_walk(data, address, stop) ||
            return EvaluationResult(false, -1, Inf, "address $(address - 1) violates walking constraint at stop $stop")

        assigned_demand[stop + 1] += data.passengers[address]
    end

    route_demand == assigned_demand ||
        return EvaluationResult(false, -1, Inf, "route stop loads do not match assigned passenger demand")

    return EvaluationResult(true, length(routes), objective, "ok")
end

function validate_solution(data::ProblemData, routes::Vector{Route}, assignment::Vector{Int})::Tuple{Bool, String}
    evaluation = evaluate_solution(data, routes, assignment)
    return evaluation.valid, evaluation.message
end

function read_solution(path::AbstractString, address_count::Int)::Tuple{Vector{Route}, Vector{Int}}
    fs = FastScanner(path)
    route_count = next_int!(fs)
    routes = Route[]
    sizehint!(routes, route_count)

    for _ in 1:route_count
        route_size = next_int!(fs)
        stops = Int[]
        weights = Int[]
        sizehint!(stops, route_size)
        sizehint!(weights, route_size)

        for _ in 1:route_size
            push!(stops, next_int!(fs))
            push!(weights, next_int!(fs))
        end
        push!(routes, Route(stops, weights))
    end

    assignment = [next_int!(fs) for _ in 1:address_count]
    return routes, assignment
end

function evaluate_solution_file(data::ProblemData, path::AbstractString)::EvaluationResult
    if !isfile(path)
        return EvaluationResult(false, -1, Inf, "missing solution file")
    end

    try
        routes, assignment = read_solution(path, data.address_count)
        return evaluate_solution(data, routes, assignment)
    catch err
        return EvaluationResult(false, -1, Inf, "cannot read solution file: $err")
    end
end

function compare_evaluations(candidate::EvaluationResult, reference::EvaluationResult)::String
    candidate.valid || return "invalid_candidate"
    reference.valid || return "no_valid_reference"

    if candidate.buses < reference.buses
        return "better"
    elseif candidate.buses > reference.buses
        return "worse"
    elseif candidate.objective < reference.objective - EPS
        return "better"
    elseif candidate.objective > reference.objective + EPS
        return "worse"
    end

    return "equal"
end

function solver_style_cost(data::ProblemData, evaluation::EvaluationResult)::Float64
    evaluation.valid || return Inf
    vehicle_cost = max(10000000.0, data.max_journey_time * 10000.0)
    return vehicle_cost * evaluation.buses + evaluation.objective
end

function write_solution(path::AbstractString, routes::Vector{Route}, assignment::Vector{Int})
    open(path, "w") do io
        println(io, length(routes))
        for route in routes
            print(io, length(route.stops))
            for i in eachindex(route.stops)
                print(io, " ", route.stops[i], " ", route.weights[i])
            end
            println(io)
        end

        for i in eachindex(assignment)
            i > 1 && print(io, " ")
            print(io, assignment[i])
        end
        println(io)
    end
end

function output_path(output_dir::AbstractString, instance_id::Int, output_suffix::AbstractString)::String
    return joinpath(output_dir, string(instance_id, output_suffix))
end

function solve_instance(
    input_path::AbstractString,
    output_file::AbstractString,
    time_limit_seconds::Float64,
)::Tuple{ProblemData, EvaluationResult}
    data = read_opt10s_instance(input_path)
    routes, assignment = build_solution(data, time_limit_seconds)

    evaluation = evaluate_solution(data, routes, assignment)
    evaluation.valid || error("Invalid solution for $(basename(input_path)): $(evaluation.message)")

    write_solution(output_file, routes, assignment)
    return data, evaluation
end

function timestamp_string()::String
    return Dates.format(now(), "yyyymmdd_HHMMSS")
end

function parse_cli_args(args::Vector{String})::Tuple{Vector{String}, Dict{String, String}}
    positional = String[]
    options = Dict{String, String}()

    for arg in args
        if startswith(arg, "--")
            raw = arg[3:end]
            key_value = split(raw, "="; limit = 2)
            key = replace(String(key_value[1]), "_" => "-")
            value = length(key_value) == 2 ? String(key_value[2]) : "true"
            options[key] = value
        else
            push!(positional, arg)
        end
    end

    return positional, options
end

function option_value(
    options::Dict{String, String},
    key::String,
    positional::Vector{String},
    position::Int,
    default::String,
)::String
    if haskey(options, key)
        return options[key]
    end
    return length(positional) >= position ? positional[position] : default
end

function write_log_line(io::IO, message::AbstractString)
    println(message)
    println(io, message)
    flush(io)
end

function format_eval(evaluation::EvaluationResult)::String
    if !evaluation.valid
        return "invalid($(evaluation.message))"
    end
    return "(buses=$(evaluation.buses), objective=$(round(evaluation.objective; digits = 2)))"
end

function print_usage()
    println("Usage:")
    println("  julia Main_GRASP-ILS.jl [input_dir] [first_id] [last_id] [output_suffix] [output_base_dir] [time_limit_seconds]")
    println("")
    println("Defaults:")
    println("  input_dir     = $DEFAULT_INPUT_DIR")
    println("  first_id      = 0")
    println("  last_id       = 19")
    println("  output_suffix = .ans")
    println("  output_base   = $DEFAULT_OUTPUT_DIR")
    println("  time_limit    = $DEFAULT_TIME_LIMIT_SECONDS seconds per instance")
    println("")
    println("Examples:")
    println(raw"  julia Main_GRASP-ILS.jl D:\OneDrive-ntdxl\prj\school-bus-routing-sciortino-2022\opt_10s 0 19 .ans")
    println(raw"  julia Main_GRASP-ILS.jl --time-limit=30")
    println(raw"  julia Main_GRASP-ILS.jl --first=0 --last=19 --output-dir=outputs\opt_10s --suffix=.ans")
    println("")
    println("Each run creates a timestamp folder inside output_base_dir, with all .ans files and one summary log.")
end

function main(args::Vector{String})
    if any(arg -> arg == "-h" || arg == "--help", args)
        print_usage()
        return
    end

    positional, options = parse_cli_args(args)
    input_dir = option_value(options, "input-dir", positional, 1, DEFAULT_INPUT_DIR)
    first_id = parse(Int, option_value(options, "first", positional, 2, "0"))
    last_id = parse(Int, option_value(options, "last", positional, 3, "19"))
    output_suffix = option_value(options, "suffix", positional, 4, ".ans")
    output_base_dir = option_value(options, "output-dir", positional, 5, DEFAULT_OUTPUT_DIR)
    time_limit_seconds =
        parse(Float64, option_value(options, "time-limit", positional, 6, string(DEFAULT_TIME_LIMIT_SECONDS)))

    current_timestamp = timestamp_string()
    run_dir = joinpath(output_base_dir, current_timestamp)
    mkpath(run_dir)

    log_path = joinpath(run_dir, "log_$current_timestamp.txt")
    failures = 0
    better_count = 0
    equal_count = 0
    worse_count = 0
    no_reference_count = 0
    generated_total_objective = 0.0
    generated_total_solver_cost = 0.0
    generated_total_buses = 0

    open(log_path, "w") do log_io
        write_log_line(log_io, "Created run folder: $run_dir")
        write_log_line(log_io, "Input folder: $input_dir")
        write_log_line(log_io, "Instances: $first_id..$last_id")
        write_log_line(log_io, "Time limit per instance: $time_limit_seconds seconds")
        write_log_line(log_io, "Output suffix: $output_suffix")
        write_log_line(log_io, "")
        write_log_line(
            log_io,
            "instance,status,generated_buses,generated_objective,generated_solver_cost,reference_buses,reference_objective,reference_solver_cost,gap_objective,elapsed_seconds,message",
        )

        for instance_id in first_id:last_id
            input_file = joinpath(input_dir, string(instance_id, ".in"))
            out_file = output_path(run_dir, instance_id, output_suffix)
            reference_file = joinpath(input_dir, string(instance_id, ".out"))

            if !isfile(input_file)
                failures += 1
                write_log_line(log_io, "$instance_id,missing_input,-1,Inf,Inf,-1,Inf,Inf,Inf,0.0,$input_file")
                continue
            end

            try
                started = time()
                data, generated_eval = solve_instance(input_file, out_file, time_limit_seconds)
                reference_eval = evaluate_solution_file(data, reference_file)
                status = compare_evaluations(generated_eval, reference_eval)
                elapsed = time() - started

                if generated_eval.valid
                    generated_total_objective += generated_eval.objective
                    generated_total_solver_cost += solver_style_cost(data, generated_eval)
                    generated_total_buses += generated_eval.buses
                end

                if status == "better"
                    better_count += 1
                elseif status == "equal"
                    equal_count += 1
                elseif status == "worse"
                    worse_count += 1
                else
                    no_reference_count += 1
                end

                reference_buses = reference_eval.valid ? string(reference_eval.buses) : "-1"
                reference_objective = reference_eval.valid ? string(round(reference_eval.objective; digits = 2)) : "Inf"
                generated_solver_cost = string(round(solver_style_cost(data, generated_eval); digits = 2))
                reference_solver_cost =
                    reference_eval.valid ? string(round(solver_style_cost(data, reference_eval); digits = 2)) : "Inf"
                gap_objective = reference_eval.valid ? string(round(generated_eval.objective - reference_eval.objective; digits = 2)) : "Inf"

                write_log_line(
                    log_io,
                    string(
                        instance_id,
                        ",",
                        status,
                        ",",
                        generated_eval.buses,
                        ",",
                        round(generated_eval.objective; digits = 2),
                        ",",
                        generated_solver_cost,
                        ",",
                        reference_buses,
                        ",",
                        reference_objective,
                        ",",
                        reference_solver_cost,
                        ",",
                        gap_objective,
                        ",",
                        round(elapsed; digits = 2),
                        ",",
                        generated_eval.message,
                    ),
                )

                println(
                    "[$instance_id] wrote $(basename(out_file)) | generated=$(format_eval(generated_eval)) | reference=$(format_eval(reference_eval)) | status=$status",
                )
            catch err
                failures += 1
                elapsed = 0.0
                write_log_line(log_io, "$instance_id,error,-1,Inf,Inf,-1,Inf,Inf,Inf,$elapsed,$err")
            end
        end

        write_log_line(log_io, "")
        write_log_line(
            log_io,
            "SUMMARY generated_total_buses=$generated_total_buses generated_total_objective=$(round(generated_total_objective; digits = 2)) generated_total_solver_cost=$(round(generated_total_solver_cost; digits = 2)) better=$better_count equal=$equal_count worse=$worse_count no_valid_reference=$no_reference_count failures=$failures",
        )
        write_log_line(log_io, "Log file: $log_path")
    end

    println("Run folder: $run_dir")
    println("Summary log: $log_path")

    failures == 0 || error("$failures instance(s) failed")
end

main(ARGS)
