/**
 * Classes to help with route-searching.
 * Based on the A* algorithm.
 */


/**
 * Nodes for A*
 */
class astar_node extends coord3d
{
	previous = null // previous node
	cost     = -1   // cost to reach this node
	dist     = -1   // distance to target
	constructor(c, p, co, d)
	{
		x = c.x
		y = c.y
		z = c.z
		previous = p
		cost     = co
		dist     = d
	}
	function is_straight_move(d)
	{
		return d == dir ||  (previous  &&  previous.dir == d)
	}
}

/**
 * Class to perform A* searches.
 *
 * Derived classes have to implement:
 *    process_node(node): add nodes to open list reachable by node
 *
 * To use this:
 * 1) call prepare_search
 * 2) add tiles to target array
 * 3) call compute_bounding_box
 * 4) add start tiles to open list
 * 5) call search()
 * 6) use route
 */
class astar
{
	closed_list = null // table
	nodes       = null // array of astar_node
	heap        = null // binary heap
	targets     = null // array of coord3d's

	boundingbox = null // box containing all the targets

	route       = null // route, reversed: target to start

	// statistics
	calls_open = 0
	calls_closed = 0
	calls_pop = 0

	// costs - can be fine-tuned
	cost_straight = 10
	cost_curve    = 14

	constructor()
	{
		closed_list = {}
		heap        = simple_heap_x()
		targets     = []

	}

	function prepare_search()
	{
		closed_list = {}
		nodes       = []
		route       = []
		heap.clear()
		targets     = []
		calls_open = 0
		calls_closed = 0
		calls_pop = 0
	}

	// adds node c to closed list
	function add_to_close(c)
	{
		closed_list[ coord3d_to_key(c) ] <- 1
		calls_closed++
	}

	function test_and_close(c)
	{
		local key = coord3d_to_key(c)
		if (key in closed_list) {
			return false
		}
		else {
			closed_list[ key ] <- 1
			calls_closed++
			return true
		}
	}

	function is_closed(c)
	{
		local key = coord3d_to_key(c)
		return (key in closed_list)
	}

	// add node c to open list with give weight
	function add_to_open(c, weight)
	{
		local i = nodes.len()
		nodes.append(c)
		heap.insert(weight, i)
		calls_open++
	}

	function search()
	{
		// compute bounding box of targets
		compute_bounding_box()

		local current_node = null
		while (!heap.is_empty()) {
			calls_pop++

			local wi = heap.pop()
			current_node = nodes[wi.value]
			local dist = current_node.dist

			// target reached
			if (dist == 0) break;
			// already visited previously
			if (!test_and_close(current_node)) {
				current_node = null
				continue;
			}
			// investigate neighbours and put them into open list
			process_node(current_node)

			current_node = null
		}

		route = []
		if (current_node) {
			// found route
			route.append(current_node)

			while (current_node.previous) {
				current_node = current_node.previous
				route.append(current_node)
			}
		}

		print("Calls: pop = " + calls_pop + ", open = " + calls_open + ", close = " + calls_closed)
	}

	/**
	 * Computes bounding box of all targets to speed up distance computation.
	 */
	function compute_bounding_box()
	{
		if (targets.len()>0) {
			local first = targets[0]
			boundingbox = { xmin = first.x, xmax = first.x, ymin = first.y, ymax = first.y }

			for(local i=1; i < targets.len(); i++) {
				local t = targets[i]
				if (boundingbox.xmin > t.x) boundingbox.xmin = t.x;
				if (boundingbox.xmax < t.x) boundingbox.xmax = t.x;
				if (boundingbox.ymin > t.y) boundingbox.ymin = t.y;
				if (boundingbox.ymax < t.y) boundingbox.ymax = t.y;
			}
		}
	}

	/**
	 * Estimates distance to target.
	 * Returns zero if and only if c is a target tile.
	 */
	function estimate_distance(c)
	{
		local d = 0
		local curved = 0

		// distance to bounding box
		local dx = boundingbox.xmin - c.x
		if (dx <= 0) dx = c.x - boundingbox.xmax;
		if (dx > 0) d += dx; else dx = 0;

		local dy = boundingbox.ymin - c.y
		if (dy <= 0) dy = c.y - boundingbox.ymax
		if (dy > 0) d += dy; else dy = 0;

		if (d > 0) {
			// cost to bounding box
			return cost_straight * d + (dx*dy > 0 ? cost_curve - cost_straight : 0);
		}
		else {
			local t = targets[0]
			d = abs(t.x-c.x) + abs(t.y-c.y)

			// inside bounding box
			for(local i=1; i < targets.len(); i++) {
				local t = targets[i]
				local dx = abs(t.x-c.x)
				local dy = abs(t.y-c.y)
				d = min(d, cost_straight * (dx+dy) + (dx*dy > 0 ? cost_curve - cost_straight : 0))
			}
		}
		return d
	}
}

class ab_node extends ::astar_node
{
	dir = 0   // direction to reach this node
	flag = 0  // flag internal to the route searcher
	constructor(c, p, co, d, di, fl=0)
	{
		base.constructor(c, p, co, d)
		dir  = di
		flag = fl
	}
}

/**
 * Helper class to find bridges and spots to place them.
 */
class pontifex
{
	player = null
	bridge = null

	constructor(pl, way)
	{
		// print messages box
		// 1 = erreg
		// 2 = list bridges
		local print_message_box = 0
		local wt_name = ["", "road", "rail", "water"]

		if ( print_message_box > 1 ) {
			gui.add_message_at(pl, "____________ Search bridge ___________", world.get_time())
		}
		player = pl
		local list = bridge_desc_x.get_available_bridges(way.get_waytype())
		local len = list.len()
		local way_speed = way.get_topspeed()
		local bridge_min_len = 12

		if (len>0) {
			bridge = list[0]
				if ( print_message_box == 2 ) {
					gui.add_message_at(pl, " ***** way : " + wt_name[way.get_waytype()], world.get_time())
					gui.add_message_at(pl, " ***** bridge : " + bridge.get_name(), world.get_time())
					gui.add_message_at(pl, " ***** get_max_length : " + bridge.get_max_length(), world.get_time())
				}

			for(local i=1; i<len; i++) {
				local b = list[i]
				if ( print_message_box == 2 ) {
					gui.add_message_at(pl, " ***** way : " + wt_name[way.get_waytype()], world.get_time())
					gui.add_message_at(pl, " ***** bridge : " + b.get_name(), world.get_time())
					gui.add_message_at(pl, " ***** get_max_length : " + b.get_max_length(), world.get_time())
				}
				if ( b.get_max_length() > bridge_min_len || b.get_max_length() == 0 ) {
					if (bridge.get_topspeed() < way_speed) {
						if (b.get_topspeed() > bridge.get_topspeed()) {
							bridge = b
						}
					}
					else {
						if (way_speed < b.get_topspeed() && b.get_topspeed() < bridge.get_topspeed()) {
							bridge = b
						}
					}
				}
			}
		}
		if ( print_message_box > 1 ) {
			gui.add_message_at(pl, " *** bridge found : " + bridge.get_name() + " way : " + wt_name[way.get_waytype()], world.get_time())
			gui.add_message_at(our_player, "--------- Search bridge end ----------", world.get_time())
		}
	}

	function find_end(pos, dir, min_length)
	{
		return bridge_planner_x.find_end(player, pos, dir, bridge, min_length)
	}
}

/**
 * Class to search a route and to build a connection (i.e. roads).
 * Builds bridges. But not tunnels (not implemented).
 */
class astar_builder extends astar
{
	builder = null
	bridger = null
	way     = null

	function process_node(cnode)
	{
		local from = tile_x(cnode.x, cnode.y, cnode.z)
		local back = dir.backward(cnode.dir)

		for(local d = 1; d<16; d*=2) {
			// do not go backwards
			if (d == back) {
				continue
			}
			// continue straight after a bridge
			if (cnode.flag == 1  &&  d != cnode.dir) {
				continue
			}

			local to = from.get_neighbour(wt_all, d)
			if (to) {
				if (builder.is_allowed_step(from, to)  &&  !is_closed(to)) {
					// estimate moving cost
					local move = cnode.is_straight_move(d)  ?  cost_straight  :  cost_curve
					local dist   = estimate_distance(to)
					// is there already a road?
					if (!to.has_way(wt_road)) {
						move += 8
					}

					local cost   = cnode.cost + move
					local weight = cost + dist
					local node = ab_node(to, cnode, cost, dist, d)

					add_to_open(node, weight)
				}
				// try bridges
				else if (bridger  &&  d == cnode.dir  &&  cnode.flag != 1) {
					local len = 1
					local max_len = bridger.bridge.get_max_length()

					do {
						local to = bridger.find_end(from, d, len)
						if (to.x < 0  ||  is_closed(to)) {
							break
						}
						local bridge_len = abs(from.x-to.x) + abs(from.y-to.y)

						local move = bridge_len * cost_straight  * 3  /*extra bridge penalty */;
						// set distance to 1 if at a target tile
						local dist = max(estimate_distance(to), 1)

						local cost   = cnode.cost + move
						local weight = cost + dist
						local node = ab_node(to, cnode, cost, dist, d, 1 /*bridge*/)

						add_to_open(node, weight)

						len = bridge_len + 1
					} while (len <= max_len)
				}
			}
		}
	}

	function search_route(start, end)
	{
		prepare_search()
		foreach (e in end) {
			targets.append(e);
		}
		compute_bounding_box()

		foreach (s in start)
		{
			local dist = estimate_distance(s)
			add_to_open(ab_node(s, null, 1, dist+1, dist, 0), dist+1)
		}

		search()

		if (route.len() > 0) {
			remove_field( route[0] )

			// do not try to build in tunnels
			local is_tunnel_0 = tile_x(route[0].x, route[0].y, route[0].z).find_object(mo_tunnel)
			local is_tunnel_1 = is_tunnel_0

			for (local i = 1; i<route.len(); i++) {
				// remove any fields on our routes (only start & end currently)
				remove_field( route[i] )

				// check for tunnel
				is_tunnel_0 = is_tunnel_1
				is_tunnel_1 = tile_x(route[i].x, route[i].y, route[i].z).find_object(mo_tunnel)

				if (is_tunnel_0 && is_tunnel_1) {
					continue
				}

				local err
				// build
				if (route[i-1].flag == 0) {
					if ( way.get_waytype() == wt_road ) {
						err = command_x.build_road(our_player, route[i-1], route[i], way, true, true)
						if (err) {
							gui.add_message_at(our_player, "Failed to build " + way.get_name() + " from " + coord_to_string(route[i-1]) + " to " + coord_to_string(route[i]) +"\n" + err, route[i])
							remove_wayline(route, (i - 1), way.get_waytype())
						}

					} else {
						if ( settings.get_pay_for_total_distance_mode == 2 ) {
							err = command_x.build_way(our_player, route[i-1], route[i], way, true)
						} else {
							err = command_x.build_way(our_player, route[i-1], route[i], way, false)
						}
						if (err) {
							gui.add_message_at(our_player, "Failed to build " + way.get_name() + " from " + coord_to_string(route[i-1]) + " to " + coord_to_string(route[i]) +"\n" + err, route[i])
							// remove way
							// route[0] to route[i]
							//err = command_x.remove_way(our_player, route[0], route[i])
							remove_wayline(route, (i - 1), way.get_waytype())
						}
					}
				}
				else if (route[i-1].flag == 1) {
					err = command_x.build_bridge(our_player, route[i-1], route[i], bridger.bridge)
					if (err) {
						gui.add_message_at(our_player, "Failed to build bridge from " + coord_to_string(route[i-1]) + " to " + coord_to_string(route[i]) +"\n" + err, route[i])
						remove_wayline(route, (i - 1), way.get_waytype())
					}
				}
				if (err) {
					return { err =  err }
				}
			}
			return { start = route[route.len()-1], end = route[0], routes = route }
		}
		print("No route found")
		return { err =  "No route" }
	}
}

/**
 *
 *
 */
function set_marker(pos) {
	local m_tile = null
	if ( pos.len() > 0 ) {
		m_tile = pos[0]
	} else {
		m_tile = pos
	}

	local tile = square_x(m_tile.x, m_tile.y).get_ground_tile()
	local tool = command_x(tool_set_marker)
	tool.work(our_player, tile)
}

/**
 * Helper class to remove a field at a factory.
 * Used if no empty spot is available to place a station.
 */
function remove_field(pos)
{
	local tile = square_x(pos.x, pos.y).get_ground_tile()
	local tool = command_x(tool_remover)
	while(tile.find_object(mo_field)) {
		tool.work(our_player, pos)
	}
}

/**
 * remove way line
 * route	= way line
 * pos		= last field to build
 * wt			= waytype
 */
function remove_wayline(route, pos, wt) {
	local tool = command_x(tool_remover)
	//while(tile.find_object(mo_field)) {
	//	tool.work(our_player, pos)
	//}
	local i = pos
	local test = 0
	for ( i; i >= 0; i-- ) {
		local tile = square_x(route[i].x, route[i].y).get_ground_tile()
		local next_tile = null
		if ( i > 0 ) {
			next_tile = square_x(route[i-1].x, route[i-1].y).get_ground_tile()
		}
		local t_field = tile.find_object(mo_way)
		// test field has way
		if ( t_field != null && t_field.get_waytype() == wt ) {
			// test direction next tile
			// break by direction 7, 11, 13, 14, 15
			if ( i > 0 ) {
				local tiles = [7, 11, 13, 14, 15]
				for ( local k = 0; k < tiles.len(); k++ ) {
					if ( next_tile.get_way_dirs(wt) == tiles[k] || t_field.get_owner() == 1 ) {
						test = 1
					}
				}
			}

			if ( i == 0 && tile.find_object(mo_building) != null ) {
				// no remove station
				test = 1
			} else {
				if ( wt == wt_road ) {
					local test_way = tile.find_object(mo_way).get_desc()
					if ( test_way.get_player() == our_player ) {
						// remone player road from tile
						tool.work(our_player, tile)
					}
				} else {
					// remone way from tile
					tool.work(our_player, tile)
				}
			}
		}
		// test crossing and remove
		t_field = tile.find_object(mo_crossing)
		if ( t_field != null ) {
			tool.work(our_player, tile)
		}
		// break by direction 7, 11, 13, 14, 15 or owner public player next tile
		if ( test == 1 ) { break }
	}

	if ( test == 1 ) {
		test = 0
		for ( local j = 0; j < i; j++ ) {
			local tile = square_x(route[j].x, route[j].y).get_ground_tile()
			local next_tile = null
			if ( j < i ) {
				next_tile = square_x(route[j+1].x, route[j+1].y).get_ground_tile()
			}
			local t_field = tile.find_object(mo_way)
			// test field has way
			if ( t_field != null && t_field.get_waytype() == wt ) {
				// test direction
				local tiles = [7, 11, 13, 14, 15]
				for ( local k = 0; k < tiles.len(); k++ ) {
					if ( next_tile.get_way_dirs(wt) == tiles[k] ) {
						test = 1
					}
				}

				if ( j == 0 && tile.find_object(mo_building) != null ) {
					// no remove station
					test = 1
				} else {
					// remone way from tile
					tool.work(our_player, tile)
				}
			}
			// test crossing and remove
			t_field = tile.find_object(mo_crossing)
			if ( t_field != null ) {
				tool.work(our_player, tile)
			}
			// break by direction 7, 11, 13, 14, 15
			if ( test == 1 ) { break }
		}
	}



	if ( test == 0 ) {
		gui.add_message_at(our_player, "removed way from " + coord_to_string(route[pos]) + " to " + coord_to_string(route[0]), route[0])
	} else {
		gui.add_message_at(our_player, "removed way not all ", route[0])
	}

}

/**
 * remove station / tiles remove until it is empty
 * fields	= field list
 * wt			= waytype
 */
function remove_tile_to_empty(tiles, wt) {
	local tool = command_x(tool_remover)
	for ( local i = tiles.len(); i > 0; i-- ) {
		gui.add_message_at(our_player, "remove tile " + coord3d_to_string(tiles[i-1]), tiles[i-1])
		local tiles_r = square_x(tiles[i-1].x, tiles[i-1].y).get_ground_tile()
		while(true){
			tool.work(our_player, tiles_r)
			if (tiles_r.is_empty())
				break
		}
	}

}

/**
 * function for check station lenght
 *
 * pl = player
 * starts_field = tile station from plan_simple_connection
 * st_lenght = stations fields count
 * wt = waytype
 * select_station = station object
 * build = 0 -> test ; 1 -> build
 *
 */
function check_station(pl, starts_field, st_lenght, wt, select_station, build = 1) {

		// print messages box
		// 1
		// 2
		local print_message_box = 0

		if ( print_message_box == 2 ) {
			gui.add_message_at(pl, " --- start field : " + coord3d_to_string(starts_field) + "  # station lenght : " + st_lenght, world.get_time())
		}

		local a = false
		local b = 0
		local b1_tile = null
		local tile_build = 0
		local st_build = false
		local err = null

		/** alignment
			*  1	-> test north
			*  2	-> test west
			*  4	-> test south
			*  5	-> test northsouth
			*  8	-> test east
			* 10	-> test eastwest
			*
			*
			*/
		local t = tile_x(starts_field.x, starts_field.y, starts_field.z)
		local d = t.get_way_dirs(wt)

		if ( print_message_box == 2 ) {
			gui.add_message_at(pl, " --- field test : " + coord3d_to_string(starts_field), world.get_time())
			gui.add_message_at(pl, " ------ get_way_dirs : " + d, world.get_time())
		}

		local b_tile = [starts_field] // station fields array
		local i = 0
		// get_dirs().to_coord()
		for ( local j = 0; j < 2; j++ ) {
			switch (d) {
				case 1:
					// check n
					if ( print_message_box == 2 ) {
						gui.add_message_at(pl, " ---> dir : 8", world.get_time())
					}
					for ( i = 1; i < st_lenght; i++ ) {
						b1_tile = tile_x(starts_field.x, starts_field.y + i, starts_field.z)
						if ( print_message_box == 2 ) {
							gui.add_message_at(pl, " ---> test : " + coord3d_to_string(b1_tile), world.get_time())
						}

						if ( test_field(pl, b1_tile, wt, 5, starts_field.z) && b_tile.len() < st_lenght ) {
							if ( print_message_box == 2 ) {
								gui.add_message_at(pl, " ---=> add tile : " + coord3d_to_string(b1_tile), world.get_time())
							}
							b_tile.append(b1_tile)
							tile_build++
						} else {
							d = 4
							break
						}

					}

					break

				case 2:
					// check w
					if ( print_message_box == 2 ) {
						gui.add_message_at(pl, " ---> dir : 2", world.get_time())
					}
					for ( i = 1; i < st_lenght; i++ ) {
						b1_tile = tile_x(starts_field.x - i, starts_field.y, starts_field.z)
						if ( print_message_box == 2 ) {
							gui.add_message_at(pl, " ---> test : " + coord3d_to_string(b1_tile), world.get_time())
						}

						if ( test_field(pl, b1_tile, wt, 10, starts_field.z) && b_tile.len() < st_lenght ) {
							if ( print_message_box == 2 ) {
								gui.add_message_at(pl, " ---=> add tile : " + coord3d_to_string(b1_tile), world.get_time())
							}
							b_tile.append(b1_tile)
							tile_build++
						} else {
							d = 8
							break
						}

					}

					break

				case 4:
					// check s
					if ( print_message_box == 2 ) {
						gui.add_message_at(pl, " ---> dir : 4", world.get_time())
					}
					for ( i = 1; i < st_lenght; i++ ) {
						b1_tile = tile_x(starts_field.x, starts_field.y - i, starts_field.z)
						if ( print_message_box == 2 ) {
							gui.add_message_at(pl, " ---> test : " + coord3d_to_string(b1_tile), world.get_time())
						}

						if ( test_field(pl, b1_tile, wt, 5, starts_field.z) && b_tile.len() < st_lenght ) {
							if ( print_message_box == 2 ) {
								gui.add_message_at(pl, " ---=> add tile : " + coord3d_to_string(b1_tile), world.get_time())
							}
							b_tile.append(b1_tile)
							tile_build++
						} else {
							d = 1
							break
						}

					}

					break

				case 8:
					// check e
					if ( print_message_box == 2 ) {
						gui.add_message_at(pl, " ---> dir : 8", world.get_time())
					}
					for ( i = 1; i < st_lenght; i++ ) {
						b1_tile = tile_x(starts_field.x + i, starts_field.y, starts_field.z)
						if ( print_message_box == 2 ) {
							gui.add_message_at(pl, " ---> test : " + coord3d_to_string(b1_tile), world.get_time())
						}

						if ( test_field(pl, b1_tile, wt, 10, starts_field.z) && b_tile.len() < st_lenght ) {
							if ( print_message_box == 2 ) {
								gui.add_message_at(pl, " ---=> add tile : " + coord3d_to_string(b1_tile), world.get_time())
							}
							b_tile.append(b1_tile)
							tile_build++
						} else {
							d = 2
							break
						}

					}

				break
			}  // end switch

			// build station
			if ( b_tile.len() == st_lenght && build == 1) {
				st_build = expand_station(pl, b_tile, wt, select_station, starts_field)
				break
			} else if ( b_tile.len() == st_lenght && build == 0 ) {
				st_build = true
				break
			}

		}

		// station not build, then search other place
		d = t.get_way_dirs(wt)
		if ( b_tile.len() < st_lenght && st_build == false ) {
				// search other place for station
				b_tile.clear()
				if ( print_message_box == 2 ) {
					gui.add_message_at(pl, " -#-=> not place found for station ", world.get_time())
				}
				if ( d == 1 || d == 4 ) {
					// test e
					for ( i = 1; i <= st_lenght; i++ ) {
						b1_tile = tile_x(starts_field.x + i, starts_field.y, starts_field.z)
						if ( print_message_box == 2 ) {
							gui.add_message_at(pl, " ---> test : " + coord3d_to_string(b1_tile), world.get_time())
						}

						if ( test_field(pl, b1_tile, wt, 5, starts_field.z) && b_tile.len() < st_lenght ) {
							if ( print_message_box == 2 ) {
								gui.add_message_at(pl, " ---=> add tile : " + coord3d_to_string(b1_tile), world.get_time())
							}
							b_tile.append(b1_tile)
						}

					}
					if ( b_tile.len() < st_lenght ) {
						b_tile.clear()
						// test w
						for ( i = 1; i <= st_lenght; i++ ) {
							b1_tile = tile_x(starts_field.x - i, starts_field.y, starts_field.z)
							if ( print_message_box == 2 ) {
								gui.add_message_at(pl, " ---> test : " + coord3d_to_string(b1_tile), world.get_time())
							}

							if ( test_field(pl, b1_tile, wt, 5, starts_field.z) && b_tile.len() < st_lenght ) {
								if ( print_message_box == 2 ) {
									gui.add_message_at(pl, " ---=> add tile : " + coord3d_to_string(b1_tile), world.get_time())
								}
								b_tile.append(b1_tile)
							}
						}

					}
					//if ( b_tile.len() == st_lenght ) {
					//	err = command_x.build_way(pl, starts_field, b_tile[0], planned_way, true)
					//}
				} else if ( d == 2 || d == 8 ) {
					// test s
					for ( i = 1; i <= st_lenght; i++ ) {
						b1_tile = tile_x(starts_field.x, starts_field.y + i, starts_field.z)
						if ( print_message_box == 2 ) {
							gui.add_message_at(pl, " ---> test : " + coord3d_to_string(b1_tile), world.get_time())
						}

						if ( test_field(pl, b1_tile, wt, 10, starts_field.z) && b_tile.len() < st_lenght ) {
							if ( print_message_box == 2 ) {
								gui.add_message_at(pl, " ---=> add tile : " + coord3d_to_string(b1_tile), world.get_time())
							}
							b_tile.append(b1_tile)
						}

					}
					if ( b_tile.len() < st_lenght ) {
						b_tile.clear()
						// test n
						for ( i = 1; i <= st_lenght; i++ ) {
							b1_tile = tile_x(starts_field.x, starts_field.y - i, starts_field.z)
							if ( print_message_box == 2 ) {
								gui.add_message_at(pl, " ---> test : " + coord3d_to_string(b1_tile), world.get_time())
							}

							if ( test_field(pl, b1_tile, wt, 10, starts_field.z) && b_tile.len() < st_lenght ) {
								if ( print_message_box == 2 ) {
									gui.add_message_at(pl, " ---=> add tile : " + coord3d_to_string(b1_tile), world.get_time())
								}
								b_tile.append(b1_tile)
							}
						}

					}
				}

			// build station
			if ( b_tile.len() == st_lenght && build == 1) {
				st_build = expand_station(pl, b_tile, wt, select_station, starts_field)
				if ( starts_field.x != b_tile[0].x || starts_field.y != b_tile[0].y ) {
					err = command_x.build_way(pl, starts_field, b_tile[0], planned_way, true)
					// station move then set c_start/c_end new
					if ( starts_field.x == c_start.x && starts_field.y == c_start.y ) {
						c_start = tile_x(b_tile[0].x, b_tile[0].y, b_tile[0].z)
					}
					if ( starts_field.x == c_end.x && starts_field.y == c_end.y ) {
						c_end = tile_x(b_tile[0].x, b_tile[0].y, b_tile[0].z)
					}
				}
			} else if ( b_tile.len() == st_lenght && build == 0 ) {
				st_build = true
			} else {
				b_tile.clear()
			}
		}

		if ( st_build == false ) {
			// move station
			if ( print_message_box == 2 ) {
				gui.add_message_at(pl, " *#* ERROR => expand station failed", world.get_time())
				gui.add_message_at(pl, " --- field test : " + coord3d_to_string(starts_field), world.get_time())
				gui.add_message_at(pl, " ------ get_way_dirs : " + d, world.get_time())
			}
		}

		print_message_box = 0
		return st_build
}

/**
 * test fields station
 * pl				= player
 * t_tile		= field to test
 * wt				= waytype
 * rotate		= northsouth ( 5 ) or eastwest ( 10 )
 * ref_hight	= z from start field
 */
function test_field(pl, t_tile, wt, rotate, ref_hight) {

	local print_message_box = 0
	local err = null
	local z = null

	// tile out of map
	if ( !world.is_coord_valid(t_tile) ) {
		return false
	}

	// find z coord
	local r = square_x(t_tile.x, t_tile.y)
	z = r.get_ground_tile() //tile_x(t_tile.x, t_tile.y, t_tile.z) //square_x(t_tile.x, t_tile.y).get_ground_tile(t_tile.x, t_tile.y)

	if ( t_tile.is_empty() && t_tile.get_slope() == 0 && ref_hight == z.z ) {
		// tile is empty and is flat
		if ( t_tile.is_ground() ) {
			if ( print_message_box == 2 ) {
				gui.add_message_at(pl, " ---=> tile is empty and is flat ", world.get_time())
			}
			return true
		}
	} else if ( t_tile.has_way(wt) && !t_tile.has_two_ways() && t_tile.get_way_dirs(wt) == rotate && t_tile.get_slope() == 0 && !t_tile.is_bridge() ) {
		// tile has single way and is flat - no bridge ramp
		if ( print_message_box == 2 ) {
			gui.add_message_at(pl, " ---=> tile has single way and is flat ", world.get_time())
		}
		return true
	} else if ( t_tile.has_way(wt) && !t_tile.has_two_ways() && t_tile.get_way_dirs(wt) == rotate && t_tile.get_slope() > 0 && t_tile.is_bridge() ) {
		// tile has single way and has bridge start
		if ( print_message_box == 2 ) {
			gui.add_message_at(pl, " ---=> tile has single way and is bridge ", world.get_time())
		}
		return true
	} else if ( t_tile.is_empty() && ( t_tile.get_slope() > 0 || ref_hight != z.z ) ) {
		// terraform
		// return true and terraform befor build station
		return true
	}

	return false
}


/**
 * function expand station()
 * pl = player
 * fields = array fields
 * wt = waytype
 * select_station = station object
 */
function expand_station(pl, fields, wt, select_station, start_field) {

	local print_message_box = 0

	local ref_hight = start_field.z
	local err = null
	local combined_station = false

	// extension field for connect station to factory/dock
	local r = tile_x(start_field.x, start_field.y, start_field.z)
	local d = r.get_way_dirs(wt)
	local extension_tile = null
	switch(d) {
		case 1:
			extension_tile = square_x(start_field.x, start_field.y + 1)
			break
		case 2:
			extension_tile = square_x(start_field.x - 1, start_field.y)
			break
		case 4:
			extension_tile = square_x(start_field.x, start_field.y - 1)
			break
		case 8:
			extension_tile = square_x(start_field.x + 1, start_field.y)
			break
	}

	local t = fields.len()

	if ( t > 0 ) {
		// terrafom
		local i = 1
		if ( fields[0].x != start_field.x || fields[0].y != start_field.y ) {
			i = 0
		}
		for ( i; i < t; i++ ) {
			local r = square_x(fields[i].x, fields[i].y)
			local z = r.get_ground_tile()
			local f = tile_x(fields[i].x, fields[i].y, z.z)

			if ( f.is_empty() && ( f.get_slope() > 0 || ref_hight != z.z ) ) {

				if ( print_message_box == 2 ) {
					 gui.add_message_at(pl, " ---=> terraform", world.get_time())
					 gui.add_message_at(pl, " ---=> tile z " + z.z + " start tile z " + ref_hight, world.get_time())
				}

				if ( z.z < ref_hight && z.z >= (ref_hight - 2) ) {
				// terraform up
					if ( print_message_box == 2 ) {
						gui.add_message_at(pl, " ---=> tile up to flat ", world.get_time())
					}
					do {
						err = command_x.set_slope(pl, tile_x(f.x, f.y, z.z), 82 )
						if ( err != null ) { break }
						z = r.get_ground_tile()
					} while(z.z < ref_hight )

				} else if ( z.z >= ref_hight || z.z <= (ref_hight + 1) ) {
					 // terraform down
					if ( print_message_box == 2 ) {
						gui.add_message_at(pl, " ---=> tile down to flat ", world.get_time())
					}
					do {
						err = command_x.set_slope(pl, tile_x(f.x, f.y, z.z), 83 )
						if ( err != null ) { break }
						z = r.get_ground_tile()
					} while(z.z > ref_hight )
				}
				if ( err ) {
					return false
				}
			}
		}

		// build way to tiles
		for ( local i = 1; i < t; i++ ) {
			if ( fields[i].is_empty() && fields[i].get_slope() == 0 ) {
				// empty then build way
				if ( fields[i].is_empty() ) {
					err = command_x.build_way(pl, fields[0], fields[i], planned_way, true)
				}
				if ( err != null ) {
					gui.add_message_at(pl, " ---=> not build way tile at " + coord3d_to_string(fields[i]) + " err " + err, fields[i])
					return false
				}
			}
		}

		// check harbour/dock
		local st_dock = search_station(start_field, wt, 1)
		if ( fields[0] != start_field ) {
			local extension = search_extension(wt)
			if ( extension ) {
				if ( print_message_box == 2 ) {
					gui.add_message_at(our_player, "-*---> selectet extension: " + extension.get_name(), world.get_time())
				}
				if ( st_dock && extension_tile ) {
					if ( print_message_box == 2 ) {
						gui.add_message_at(our_player, "-*---> dock/harbour found at : " + coord3d_to_string(st_dock[0]), st_dock[0])
					}
					local tile = tile_x(extension_tile.x, extension_tile.y, extension_tile.get_ground_tile().z)
					if ( tile.is_empty() ) {
						if ( print_message_box == 2 ) {
							gui.add_message_at(our_player, "-*---> build extension at : " + coord3d_to_string(tile), world.get_time())
						}
						err = command_x.build_station(pl, tile, extension)
					}
				}
			}
		}
		if ( st_dock ) {
			combined_station = true
		}

		// station not build to start_field
		local build_connection = 0
		if ( fields[0] != start_field ) {

			if ( combined_station ) {
				// build connect tile to dock
				err = command_x.build_station(pl, start_field, select_station)
				build_connection = 1
			}
		}


	 	if ( err == null ) {
			// build station to tile
			for ( local i = 0; i < t; i++ ) {
				local z = square_x(fields[i].x, fields[i].y).get_ground_tile()
				local f = tile_x(fields[i].x, fields[i].y, z.z)
				if ( tile_x(fields[i].x, fields[i].y, fields[i].z).is_bridge() && f.get_slope() > 0 ) {
					// bridge start field -> build to ground
					fields[i].z -= 1
				}
				err = command_x.build_station(pl, fields[i], select_station)
				if ( err ) {
					gui.add_message_at(pl, " ---=> not build station tile at " + coord3d_to_string(fields[i]), fields[i])
					return false
				}
				if ( print_message_box == 2 ) {
					gui.add_message_at(pl, " ---=> build station tile at " + coord3d_to_string(fields[i]), fields[i])
				}
			}
		}

		// station not build to start_field
		if ( fields[0] != start_field ) {
			if ( combined_station && build_connection == 1 ) {
				// remove connect tile to dock
				local tool = command_x(tool_remover)
				tool.work(our_player, start_field)
			}
			// connect way to station
			err = command_x.build_way(pl, start_field, fields[0], planned_way, true)
		}


		// check station connect factory
		local st = halt_x.get_halt(fields[0], pl)

		if ( st ) {
			local fl_st = st.get_factory_list()
			if ( combined_station == false && fl_st.len() == 0 ) {
				local tile = tile_x(extension_tile.x, extension_tile.y, extension_tile.get_ground_tile().z)
				if ( print_message_box == 2 ) {
					gui.add_message_at(our_player, "-*---> build extension at : " + coord3d_to_string(tile), tile)
				}
				local extension = search_extension(wt)
				local new_tile = 0
				if ( tile.is_empty() ) {
					err = command_x.build_station(pl, tile, extension)
					if ( err ) {
						gui.add_message_at(pl, " -#-=> WARNING not connect factory: " + coord3d_to_string(start_field), start_field)
						new_tile = 1
					}
				} else if ( tile.has_way(wt_road) && !tile.has_two_ways() ) {
					local tiles = [1, 2, 4, 5, 8, 10]
					local test = 0
					// test direction from road
					for ( local i = 0; i < tiles.len(); i++ ) {
						if ( tile.get_way_dirs(wt_road) == tiles[i] ) {
							test++
						}
					}
					if ( test == 1 ) {
						local stations_list = building_desc_x.get_available_stations(building_desc_x.station, wt_road, good_desc_x("Passagiere"))
						err = command_x.build_station(pl, tile, stations_list[0])
						if ( err ) {
							gui.add_message_at(pl, " -#-=> WARNING not connect factory: " + coord3d_to_string(start_field), start_field)
							new_tile = 1
						}

					} else {
						new_tile = 1
					}

				} else {
					new_tile = 1
				}

				if ( new_tile == 1 ) {
					// to do connection factory other rotations

					local s_tiles = []
					local tool = command_x(tool_remover)
					if ( start_field.x < fields[0].x && start_field.y < tile.y ) {
						tile = square_x(start_field.x-1, start_field.y).get_ground_tile()
						if ( tile.is_empty ) {
							gui.add_message_at(pl, " --=> new extensions field: " + coord3d_to_string(tile), tile)
							s_tiles.append(fields[0])
							s_tiles.append(start_field)
							if ( start_field.get_way_dirs(wt_rail) == 3 ) {
								s_tiles.append(square_x(start_field.x, start_field.y-1).get_ground_tile())
							} else if ( start_field.get_way_dirs(wt_rail) == 6 ) {
								s_tiles.append(square_x(start_field.x, start_field.y+1).get_ground_tile())
							}
							// remove way for build extension
							tool.work(our_player, start_field)
							// build extensions
							err = command_x.build_station(pl, start_field, extension)
							err = command_x.build_station(pl, tile, extension)
							// remove extension for restore way
							tool.work(our_player, start_field)
							err = command_x.build_way(pl, s_tiles[0], s_tiles[1], planned_way, true)
							err = command_x.build_way(pl, s_tiles[1], s_tiles[2], planned_way, true)
						}

						fields.append(tile)
					} else {
						local tile = square_x(fields[0].x, fields[0].y).get_ground_tile()
						if ( tile.find_object(mo_building) != null ) {
							local halt_tiles = tile.get_halt().get_tile_list()
							local test = 0
							for ( local i = 0; i < halt_tiles.len(); i++ ) {
								if ( tile.has_way(wt_rail) || tile.has_way(wt_road) || tile.has_way(wt_water) ) {
									test++
								}
							}
							if ( test == halt_tiles.len() ) {
								gui.add_message_at(pl, " -#-=> combined station " + coord3d_to_string(start_field), start_field)
							} else {
								gui.add_message_at(pl, " -#-=> WARNING not connect factory: " + coord3d_to_string(start_field), start_field)
							}
						}
					}


				}
			}

		}

		return fields
	}
}

/**
	* search extension building
	*
	*
	*/
function search_extension(wt) {

	local print_message_box = 0

	local select_extension = null

	// extension building from waytype for selected good
	local extension_list = building_desc_x.get_available_stations(building_desc_x.station_extension, wt, good_desc_x(freight))

	if ( extension_list.len() > 0 ) {
		foreach(extension in extension_list) {
			local ok = (select_extension == null)

			if ( print_message_box == 2 ) {
				gui.add_message_at(our_player, "extension " + extension.get_name(), world.get_time())
			}

			if ( !ok ) {
				if ( select_extension.get_capacity() > extension.get_capacity() ) {
					select_extension = extension
				}
			} else {
				select_extension = extension
			}
		}
	} else {
		// not find extension from waytype for selected good
		// search post extension from all waytypes
		extension_list = building_desc_x.get_available_stations(building_desc_x.station_extension, wt_all, good_desc_x("post"))

		if ( extension_list.len() > 0 ) {
			foreach(extension in extension_list) {
				local ok = (select_extension == null)

				if ( print_message_box == 2 ) {
					gui.add_message_at(our_player, "extension " + extension.get_name(), world.get_time())
				}

				if ( !ok ) {
					if ( select_extension.get_capacity() > extension.get_capacity() ) {
						select_extension = extension
					}
				} else {
					select_extension = extension
				}
			}
		}

	}

	return select_extension

}

/**
 * search existing depot on range to station
 *
 */
function search_depot(field_pos, wt) {

	local list_exists_depot = depot_x.get_depot_list(our_player, wt)
	// search range
	local seach_field = 10

	if ( list_exists_depot ) {
		local tile_min = [field_pos.x - seach_field, field_pos.y - seach_field]
		local tile_max = [field_pos.x + seach_field, field_pos.y + seach_field]
		local depot_found = null

		foreach(key in list_exists_depot) {

			if ( key.x >= tile_min[0] && key.y >= tile_min[1] && key.x <= tile_max[0] && key.y <= tile_max[1] ) {
				depot_found = tile_x(key.x, key.y, key.z)
				return depot_found
			}
		}
	}
	return false
}

/**
 * search existing station on range to field
 *
 *  field_pos	= start field
 *  wt				= waytype
 *  range			= search range
 *
 */
function search_station(field_pos, wt, range) {

		local tile_min = [field_pos.x - range, field_pos.y - range]
		local tile_max = [field_pos.x + range, field_pos.y + range]
		local station_found = false

		foreach(halt in halt_list_x()) {
				if (halt.get_owner().nr == our_player_nr) {  // && halt.get_type == wt
					local tile = halt.get_tile_list()
					if ( tile[0].x >= tile_min[0] && tile[0].y >= tile_min[1] && tile[0].x <= tile_max[0] && tile[0].y <= tile_max[1] ) {
						station_found = tile
						break
					}

				}
		}

		return station_found
}

/**
 * build double way for more convoys to line
 *
 *
 */
function build_double_track(start_field, wt) {

	// 1
	// 2 - terraform
	// 3 - double track diagonal
	local print_message_box = 1

	if ( print_message_box > 0 ) {
		gui.add_message_at(our_player, " ### build_double_track ### " + coord3d_to_string(start_field), start_field)
	}

	//local way_list = way_desc_x.get_available_ways(wt, st_flat)
	local way_obj = start_field.find_object(mo_way).get_desc() //way_list[0]

	local b_player = our_player //way_obj.get_owner()

	local d = start_field.get_way_dirs(wt)

	local tiles = []
	local tiles_build_l = []
	local tiles_build_r = []
	local t = 0
	local way_len = 8
	local diagonal_st = 0
	local way_len_d = 0
	if ( d == 6 || d == 9 ) {
		way_len = 9
		way_len_d = 1
		diagonal_st = d
	} else if ( d == 3 || d == 12  ) {
		way_len = 9
		way_len_d = 1
		diagonal_st = d
	}
	local diagonal_x = 0
	local diagonal_y = 0

	local signal = []

	// check way
	// 8 fields straight
	// 11 fields diagonal

	for ( local i = 0; i < (way_len + way_len_d); i++ ) {
		if ( d == 5 ) {
			// build from n to s
			// ns - r
			local ref_ground = square_x(start_field.x, start_field.y + i).get_ground_tile()
			if ( tile_x(start_field.x + 1, start_field.y + i, start_field.z).is_empty() && square_x(start_field.x, start_field.y + i).get_ground_tile().z == ref_ground.z ) {
				tiles_build_r.append(tile_x(start_field.x + 1, start_field.y + i, ref_ground.z))
			}
			// ns - l
			if ( tile_x(start_field.x - 1, start_field.y + i, start_field.z).is_empty() && square_x(start_field.x, start_field.y + i).get_ground_tile().z == ref_ground.z ) {
				tiles_build_l.append(tile_x(start_field.x - 1, start_field.y + i, ref_ground.z))
			}
			tiles.append(ref_ground)
		} else if ( d == 10 ) {
			//gui.add_message_at(our_player, "tile r " + coord3d_to_string(tile_x(start_field.x + i, start_field.y + 1, start_field.z)) + " empty " + tile_x(start_field.x + i, start_field.y + 1, start_field.z).is_empty(), start_field)
			//gui.add_message_at(our_player, " -- tile l " + coord3d_to_string(tile_x(start_field.x + i, start_field.y - 1, start_field.z)) + " to empty " + tile_x(start_field.x - i, start_field.y + 1, start_field.z).is_empty(), start_field)

			// ew - r
			// build from w to e
			local ref_ground = square_x(start_field.x + i, start_field.y).get_ground_tile()
			if ( tile_x(start_field.x + i, start_field.y + 1, start_field.z).is_empty() && square_x(start_field.x + i, start_field.y).get_ground_tile().z == ref_ground.z ) {
				tiles_build_r.append(tile_x(start_field.x + i, start_field.y + 1, ref_ground.z))
			}
			// ew - l
			if ( tile_x(start_field.x + i, start_field.y - 1, start_field.z).is_empty() && square_x(start_field.x + i, start_field.y).get_ground_tile().z == ref_ground.z ) {
				tiles_build_l.append(tile_x(start_field.x + i, start_field.y - 1, ref_ground.z))
			}
			tiles.append(ref_ground)
		} else if ( d == 6 || d == 9 ) {
			// build from ne to sw
			// nw - r
			if ( i > 0 ) {
				if ( tiles[i-1].get_way_dirs(wt) == 6 ) {
					diagonal_y++
				}
				if ( tiles[i-1].get_way_dirs(wt) == 9 ) {
					diagonal_x++
				}
			}

			local ref_ground = square_x(start_field.x - diagonal_x, start_field.y + diagonal_y).get_ground_tile()
			if ( print_message_box == 1 ) {
				gui.add_message_at(b_player, "ref_ground " + coord3d_to_string(ref_ground), ref_ground)
			}

			if ( tiles_build_l.len() == 0 && ref_ground.get_way_dirs(wt) == 9 ) {
				if ( tile_x(ref_ground.x, ref_ground.y + 1, ref_ground.z).is_empty() && square_x(ref_ground.x, ref_ground.y + 1).get_ground_tile().z == ref_ground.z ) {
					tiles_build_l.append(tile_x(ref_ground.x, ref_ground.y + 1, ref_ground.z))
				}
				if ( tile_x(ref_ground.x, ref_ground.y + 2, ref_ground.z).is_empty() && square_x(ref_ground.x, ref_ground.y + 2).get_ground_tile().z == ref_ground.z ) {
					tiles_build_l.append(tile_x(ref_ground.x, ref_ground.y + 2, ref_ground.z))
				}
			} else if ( tiles_build_l.len() > way_len-2 && tiles_build_l.len() <= way_len ) {
				// no build tile
			} else if ( tiles_build_l.len() < way_len-2 ) {
				if ( tile_x(ref_ground.x, ref_ground.y + 2, ref_ground.z).is_empty() && square_x(ref_ground.x, ref_ground.y + 2).get_ground_tile().z == ref_ground.z ) {
					tiles_build_l.append(tile_x(ref_ground.x, ref_ground.y + 2, ref_ground.z))
				}
			}

			if ( tiles_build_r.len() == 0 && ref_ground.get_way_dirs(wt) == 6 ) {
				if ( tile_x(ref_ground.x - 1, ref_ground.y, ref_ground.z).is_empty() && square_x(ref_ground.x - 1, ref_ground.y).get_ground_tile().z == ref_ground.z ) {
					tiles_build_r.append(tile_x(ref_ground.x - 1, ref_ground.y, ref_ground.z))
				}
				if ( tile_x(ref_ground.x - 2, ref_ground.y, ref_ground.z).is_empty() && square_x(ref_ground.x - 2, ref_ground.y).get_ground_tile().z == ref_ground.z ) {
					tiles_build_r.append(tile_x(ref_ground.x - 2, ref_ground.y, ref_ground.z))
				}
			}else if ( tiles_build_r.len() > way_len-2 && i <= way_len ) {
				// no build tile
			} else if ( tiles_build_r.len() < way_len-2 ) {
				if ( tile_x(ref_ground.x - 2, ref_ground.y, ref_ground.z).is_empty() && square_x(ref_ground.x - 2, ref_ground.y).get_ground_tile().z == ref_ground.z ) {
					tiles_build_r.append(tile_x(ref_ground.x - 2, ref_ground.y, ref_ground.z))
				}
			}
			tiles.append(ref_ground)

			if ( print_message_box == 30 && i < way_len-2 ) {
				gui.add_message_at(b_player, "tiles_build_r[" + i + "] " + coord3d_to_string(tiles_build_r[i]), world.get_time())
				gui.add_message_at(b_player, "tiles_build_l[" + i + "] " + coord3d_to_string(tiles_build_l[i]), world.get_time())
			}
		} else if ( d == 3 || d == 12 ) {
			// build from nw to se
			// nw - r
			if ( i > 0 ) {
				if ( tiles[i-1].get_way_dirs(wt) == 3 ) {
					diagonal_x++
				}
				if ( tiles[i-1].get_way_dirs(wt) == 12 ) {
					diagonal_y++
				}
			}

			local ref_ground = square_x(start_field.x + diagonal_x, start_field.y + diagonal_y).get_ground_tile()
			if ( print_message_box == 2 ) {
				gui.add_message_at(b_player, "ref_ground " + coord3d_to_string(ref_ground), world.get_time())
			}

			if ( tiles_build_l.len() == 0 && ref_ground.get_way_dirs(wt) == 3 ) {
				if ( tile_x(ref_ground.x, ref_ground.y + 1, ref_ground.z).is_empty() && square_x(ref_ground.x, ref_ground.y + 1).get_ground_tile().z == ref_ground.z ) {
					tiles_build_l.append(tile_x(ref_ground.x, ref_ground.y + 1, ref_ground.z))
				}
				if ( tile_x(ref_ground.x, ref_ground.y + 2, ref_ground.z).is_empty() && square_x(ref_ground.x, ref_ground.y + 2).get_ground_tile().z == ref_ground.z ) {
					tiles_build_l.append(tile_x(ref_ground.x, ref_ground.y + 2, ref_ground.z))
				}
			} else if ( tiles_build_l.len() > way_len-2 && tiles_build_l.len() <= way_len ) {
				// no build tile
			} else if ( tiles_build_l.len() < way_len-2 ) {
				if ( tile_x(ref_ground.x, ref_ground.y + 2, ref_ground.z).is_empty() && square_x(ref_ground.x, ref_ground.y + 2).get_ground_tile().z == ref_ground.z ) {
					tiles_build_l.append(tile_x(ref_ground.x, ref_ground.y + 2, ref_ground.z))
				}
			}

			if ( tiles_build_r.len() == 0 && ref_ground.get_way_dirs(wt) == 12 ) {
				if ( tile_x(ref_ground.x + 1, ref_ground.y, ref_ground.z).is_empty() && square_x(ref_ground.x + 1, ref_ground.y).get_ground_tile().z == ref_ground.z ) {
					tiles_build_r.append(tile_x(ref_ground.x + 1, ref_ground.y, ref_ground.z))
				}
				if ( tile_x(ref_ground.x + 2, ref_ground.y, ref_ground.z).is_empty() && square_x(ref_ground.x + 2, ref_ground.y).get_ground_tile().z == ref_ground.z ) {
					tiles_build_r.append(tile_x(ref_ground.x + 2, ref_ground.y, ref_ground.z))
				}
			}else if ( tiles_build_r.len() > way_len-2 && i <= way_len ) {
				// no build tile
			} else if ( tiles_build_r.len() < way_len-2 ) {
				if ( tile_x(ref_ground.x + 2, ref_ground.y, start_field.z).is_empty() && square_x(ref_ground.x + 2, ref_ground.y).get_ground_tile().z == ref_ground.z ) {
					tiles_build_r.append(tile_x(ref_ground.x + 2, ref_ground.y, ref_ground.z))
				}
			}
			tiles.append(ref_ground)

			if ( print_message_box == 4 && i < way_len-2 ) {
					gui.add_message_at(b_player, "tiles_build_r[" + i + "] " + coord3d_to_string(tiles_build_r[i]), world.get_time())
					gui.add_message_at(b_player, "tiles_build_l[" + i + "] " + coord3d_to_string(tiles_build_l[i]), world.get_time())
			}
		}
	}

	if ( print_message_box > 0 ) {
		gui.add_message_at(b_player, "  tiles_build_r " + tiles_build_r.len(), world.get_time())
		gui.add_message_at(b_player, "  tiles_build_l " + tiles_build_l.len(), world.get_time())
	}

	// test fields flat
	local tl = 0
	local tr = 0
	if ( ( ( tiles_build_r.len() == way_len || tiles_build_l.len() == way_len ) && diagonal_st == 0 ) || ( ( tiles_build_r.len() == (way_len-2) || tiles_build_l.len() == (way_len-2) ) && diagonal_st > 0 ) ) {
		if ( diagonal_st == 0 ) {
			// test straight way
			gui.add_message_at(b_player, " -- slope test straight way ", world.get_time())

			if ( tiles_build_r.len() == way_len ) {
				for ( local i = 0; i < way_len; i++ ) {
					if ( tiles_build_r[i].get_slope() == 0 ) {
						tr++
					}
				}
			}
			if ( tiles_build_l.len() == way_len ) {
				for ( local i = 0; i < way_len; i++ ) {
					if ( tiles_build_l[i].get_slope() == 0 ) {
						tl++
					}
				}
			}
		} else if ( diagonal_st == 6 || diagonal_st == 9 || diagonal_st == 3 || diagonal_st == 12 ) {
			// test diagonal way
			gui.add_message_at(b_player, " -- slope test diagonal way ", world.get_time())

			if ( tiles_build_r.len() == way_len - 2 ) {
				for ( local i = 0; i < way_len - 2; i++ ) {
					//gui.add_message_at(b_player, " -- tiles_build_r[" + i + "] " + coord3d_to_string(tiles_build_r[i]), world.get_time())
					if ( tiles_build_r[i].get_slope() == 0 ) {
						tr++
					}
				}
			}

			if ( tiles_build_l.len() == way_len - 2 ) {
				for ( local i = 0; i < way_len - 2; i++ ) {
					//gui.add_message_at(b_player, " -- tiles_build_l[" + i + "] " + coord3d_to_string(tiles_build_l[i]), world.get_time())
					if ( tiles_build_l[i].get_slope() == 0 ) {
						tl++
					}
				}
			}

		}
		//gui.add_message_at(our_player, "  tiles r get_slope() = 0 " + tr, world.get_time())
		//gui.add_message_at(our_player, "  tiles l get_slope() = 0 " + tl, world.get_time())

		local tiles_build = null
		local err = null
		local terraform = 0

		if ( ( tr == way_len &&  diagonal_st == 0 ) || ( tr == way_len - 2 &&  diagonal_st > 0 ) ) {
			if ( print_message_box == 1 ) {
				gui.add_message_at(b_player, "build flat right ", start_field)
			}
			tiles_build = tiles_build_r
			tr = way_len
			tl = 0
		}
		else if ( ( tl == way_len &&  diagonal_st == 0 ) || ( tl == way_len - 2 &&  diagonal_st > 0 ) ) {
			if ( print_message_box == 1 ) {
				gui.add_message_at(b_player, "build flat left ", start_field)
			}
			tiles_build = tiles_build_l
			tl = way_len
			tr = 0
		}
		else if ( ( tr < tiles_build_r.len() && tiles_build_r.len() == way_len &&  diagonal_st == 0 ) || ( tr < tiles_build_r.len() && tiles_build_r.len() == way_len - 2 &&  diagonal_st > 0 ) ) {
			// check terraform tr
			if ( print_message_box == 1 ) {
				gui.add_message_at(b_player, "build right check terraform", world.get_time())
			}
			tiles_build = tiles_build_r
			tr = way_len
			tl = 0
			terraform = 1
		}
		else if ( ( tl < tiles_build_l.len() && tiles_build_l.len() == way_len &&  diagonal_st == 0 ) || ( tl < tiles_build_l.len() && tiles_build_l.len() == way_len - 2 &&  diagonal_st > 0 ) ) {
			// check terraform tl
			if ( print_message_box == 1 ) {
				gui.add_message_at(b_player, "build left check terraform", world.get_time())
			}
			tiles_build = tiles_build_l
			tl = way_len
			tr = 0
			terraform = 1
		}

		if ( terraform == 1 ) {
			// change terraform
			for ( local i = 0; i < tiles_build.len(); i++ ) {
				//local r = square_x(tiles_build[i].x, tiles_build[i].y)
				local z = 0
				//local f = tile_x(tiles_build[i].x, tiles_build[i].y, z.z)
				local build_hight = square_x(tiles_build[i].x, tiles_build[i].y).get_ground_tile()
				local ref_hight = square_x(tiles[i].x, tiles[i].y).get_ground_tile()

				local straight_slope = false
				if ( ref_hight.get_slope() == 4 || ref_hight.get_slope() == 12 || ref_hight.get_slope() == 28 || ref_hight.get_slope() == 36 ) {
					// single hight && double hight 1
					straight_slope = true
				} else if ( ref_hight.get_slope() == 8 || ref_hight.get_slope() == 24 || ref_hight.get_slope() == 56 || ref_hight.get_slope() == 72 ) {
					// double hight 2
					straight_slope = true
				}

				if ( print_message_box == 2 ) {
					gui.add_message_at(b_player, " ---=> tiles[i] ground " + coord3d_to_string(ref_hight), ref_hight)
					gui.add_message_at(b_player, " ---=> tiles_build[i] ground " + coord3d_to_string(build_hight), build_hight)
					gui.add_message_at(b_player, " ---=> ref_hight.get_slope() " + ref_hight.get_slope(), world.get_time())
				}

				if ( ref_hight.z == build_hight.z && ref_hight.get_slope() == build_hight.get_slope() ) {

					if ( print_message_box == 2 ) {
						gui.add_message_at(b_player, " ---=> slope tiles == tiles_build * tiles.z == tiles_build.z ", world.get_time())
					}

				} else if ( build_hight.is_empty() && straight_slope == true ) {
					// set slope ramp
					if ( print_message_box == 2 ) {
					 	gui.add_message_at(b_player, " ---=> terraform slope ramp", world.get_time())
					 	gui.add_message_at(b_player, " ---=> tiles_build.z " + build_hight.z + " tiles.z " + ref_hight.z, world.get_time())
					}
					err = command_x.set_slope(b_player, build_hight, ref_hight.get_slope())
					if ( err != null ) {
					 	gui.add_message_at(b_player, " ERROR " + err, world.get_time())
						err = null
					}

				} else if ( build_hight.is_empty() && ( build_hight.get_slope() > 0 || ref_hight.z != build_hight.z ) ) {

					if ( print_message_box == 2 ) {
					 	gui.add_message_at(b_player, " ---=> terraform", world.get_time())
					 	gui.add_message_at(b_player, " ---=> tiles_build.z " + build_hight.z + " tiles.z " + ref_hight.z, world.get_time())
					}

					if ( build_hight.z < ref_hight.z && build_hight.z >= (ref_hight.z - 2) ) {
					// terraform up
						if ( print_message_box == 2 ) {
							gui.add_message_at(b_player, " ---=> tile up to flat ", world.get_time())
						}
						do {
							err = command_x.set_slope(b_player, build_hight, 82 )
							if ( err != null ) { break }
							z = square_x(tiles_build[i].x, tiles_build[i].y).get_ground_tile()
						} while(z.z < ref_hight.z )

					} else if ( build_hight.z >= ref_hight.z || build_hight.z <= (ref_hight.z + 1) ) {
					 	// terraform down
						if ( print_message_box == 2 ) {
							gui.add_message_at(b_player, " ---=> tile down to flat ", world.get_time())
						}
						do {
							err = command_x.set_slope(b_player, build_hight, 83 )
							if ( err != null ) { break }
							z = square_x(tiles_build[i].x, tiles_build[i].y).get_ground_tile()
						} while(z.z > ref_hight.z )
					}
					if ( err ) {
						return false
					}
				}
			}
		}

		// set build left or right
		if ( tr == way_len ) {
			tiles_build = tiles_build_r
		}
		else if ( tl == way_len ) {
			tiles_build = tiles_build_l
		}

		// set diagonal start direction correct
		if ( diagonal_st > 0 ) {
			// tr = ribi 6 /
			// tl = ribi 9 /
			if ( ( tiles[0].get_way_dirs(wt) == 9 && tr == way_len ) || ( tiles[0].get_way_dirs(wt) == 6 && tl == way_len ) || ( tiles[0].get_way_dirs(wt) == 3 && tr == way_len ) || ( tiles[0].get_way_dirs(wt) == 12 && tl == way_len ) ) {
				gui.add_message_at(b_player, "remove first tile from tiles[] ", world.get_time())
				local n = tiles.slice(1)
				tiles.clear()
				tiles = n
			} else if ( tiles.len() > way_len ) {
				gui.add_message_at(b_player, "remove last tile from tiles[] ", world.get_time())
				local n = tiles.slice(0, way_len)
				tiles.clear()
				tiles = n
			}


			diagonal_st = tiles[0].get_way_dirs(wt)

		}

		if ( tr == way_len ) {
			if ( print_message_box == 3 ) {
				gui.add_message_at(b_player, "build flat right ", start_field)
			}
			if ( settings.get_drive_on_left() ) {
				if ( d == 10 ) {
					signal = [{coor=coord3d(tiles_build[1].x, tiles_build[1].y, tiles_build[1].z), ribi=8}, {coor=coord3d(tiles[6].x, tiles[6].y, tiles[6].z), ribi=2}]
					gui.add_message_at(b_player, "settings.get_drive_on_left() signals 10 tr " + coord3d_to_string(tiles_build[1]) + " & " + coord3d_to_string(tiles[6]), world.get_time())

				} else if ( d == 5 ) {
					signal = [{coor=coord3d(tiles_build[6].x, tiles_build[6].y, tiles_build[6].z), ribi=4}, {coor=coord3d(tiles[1].x, tiles[1].y, tiles[1].z), ribi=1}]
					gui.add_message_at(b_player, "settings.get_drive_on_left() signals 5 tr " + coord3d_to_string(tiles_build[6]) + " & " + coord3d_to_string(tiles[1]), world.get_time())

				} else if ( diagonal_st == 6 ) {
					// ribi 6 to 6
					signal = [{coor=coord3d(tiles_build[0].x, tiles_build[0].y, tiles_build[0].z), ribi=2}, {coor=coord3d(tiles[way_len - 2].x, tiles[way_len - 2].y, tiles[way_len - 2].z), ribi=8}]
					gui.add_message_at(b_player, "settings.get_drive_on_left() signals diagonal tr " + coord3d_to_string(tiles_build[0]) + " & " + coord3d_to_string(tiles[way_len - 2]), world.get_time())

				} else if ( diagonal_st == 12 ) {
					signal = [{coor=coord3d(tiles_build[way_len - 3].x, tiles_build[way_len - 3].y, tiles_build[way_len - 3].z), ribi=4}, {coor=coord3d(tiles[1].x, tiles[1].y, tiles[1].z), ribi=1}]
					gui.add_message_at(b_player, "settings.get_drive_on_left() signals diagonal tr " + coord3d_to_string(tiles_build[way_len - 3]) + " & " + coord3d_to_string(tiles[1]), world.get_time())

				}
			}
			else {
				if ( d == 10 ) {
					signal = [{coor=coord3d(tiles_build[6].x, tiles_build[6].y, tiles_build[6].z), ribi=2}, {coor=coord3d(tiles[1].x, tiles[1].y, tiles[1].z), ribi=8}]
					gui.add_message_at(b_player, "signals 10 tr " + coord3d_to_string(tiles_build[6]) + " & " + coord3d_to_string(tiles[1]), world.get_time())

				} else if ( d == 5 ) {
					signal = [{coor=coord3d(tiles[6].x, tiles[6].y, tiles[6].z), ribi=4}, {coor=coord3d(tiles_build[1].x, tiles_build[1].y, tiles_build[1].z), ribi=1}]
					gui.add_message_at(b_player, "signals 5 tr " + coord3d_to_string(tiles[6]) + " & " + coord3d_to_string(tiles_build[1]), world.get_time())

				} else if ( diagonal_st == 6 ) {
					// ribi 6 to 6
					signal = [{coor=coord3d(tiles_build[way_len - 3].x, tiles_build[way_len - 3].y, tiles_build[way_len - 3].z), ribi=4}, {coor=coord3d(tiles[1].x, tiles[1].y, tiles[1].z), ribi=1}]
					gui.add_message_at(b_player, "signals diagonal tr " + coord3d_to_string(tiles_build[way_len - 3]) + " & " + coord3d_to_string(tiles[1]), world.get_time())
				} else if ( diagonal_st == 12 ) {
					// ribi 12 to 12
					signal = [{coor=coord3d(tiles_build[0].x, tiles_build[0].y, tiles_build[0].z), ribi=8}, {coor=coord3d(tiles[way_len - 2].x, tiles[way_len - 2].y, tiles[way_len - 2].z), ribi=2}]
					gui.add_message_at(b_player, "signals diagonal tr " + coord3d_to_string(tiles_build[0]) + " & " + coord3d_to_string(tiles[way_len - 2]), world.get_time())

				}
			}
		}
		else if ( tl == way_len ) {
			if ( print_message_box == 3 ) {
				gui.add_message_at(b_player, "build flat left ", start_field)
			}
			if ( settings.get_drive_on_left() ) {
				if (  d == 10 ) {
					signal = [{coor=coord3d(tiles_build[6].x, tiles_build[6].y, tiles_build[6].z), ribi=2}, {coor=coord3d(tiles[1].x, tiles[1].y, tiles[1].z), ribi=8}]
					gui.add_message_at(b_player, "settings.get_drive_on_left() signals 10 tl " + coord3d_to_string(tiles_build[6]) + " & " + coord3d_to_string(tiles[1]), world.get_time())

				} else if ( d == 5 ) {
					signal = [{coor=coord3d(tiles_build[1].x, tiles_build[1].y, tiles_build[1].z), ribi=1}, {coor=coord3d(tiles[6].x, tiles[6].y, tiles[6].z), ribi=4}]
					gui.add_message_at(b_player, "settings.get_drive_on_left() signals 5 tl " + coord3d_to_string(tiles_build[1]) + " & " + coord3d_to_string(tiles[6]), world.get_time())
				} else if ( diagonal_st == 9 ) {
					// ribi 9 to 9
					signal = [{coor=coord3d(tiles_build[way_len - 3].x, tiles_build[way_len - 3].y, tiles_build[way_len - 3].z), ribi=8}, {coor=coord3d(tiles[1].x, tiles[1].y, tiles[1].z), ribi=2}]
					gui.add_message_at(b_player, "settings.get_drive_on_left() signals diagonal tl " + coord3d_to_string(tiles_build[way_len - 3]) + " & " + coord3d_to_string(tiles[2]), world.get_time())
				} else if ( diagonal_st == 3 ) {
					// ribi 3 to 3
					signal = [{coor=coord3d(tiles_build[0].x, tiles_build[0].y, tiles_build[0].z), ribi=1}, {coor=coord3d(tiles[way_len - 2].x, tiles[way_len - 2].y, tiles[way_len - 2].z), ribi=4}]
					gui.add_message_at(b_player, "settings.get_drive_on_left() signals diagonal tl " + coord3d_to_string(tiles_build[0]) + " & " + coord3d_to_string(tiles[way_len - 2]), world.get_time())

				}
			}
			else {
				if (  d == 10 ) {
					signal = [{coor=coord3d(tiles_build[1].x, tiles_build[1].y, tiles_build[1].z), ribi=8}, {coor=coord3d(tiles[6].x, tiles[6].y, tiles[6].z), ribi=2}]
					gui.add_message_at(b_player, "signals 10 tl " + coord3d_to_string(tiles_build[1]) + " & " + coord3d_to_string(tiles[6]), world.get_time())

				} else if ( d == 5 ) {
					signal = [{coor=coord3d(tiles_build[6].x, tiles_build[6].y, tiles_build[6].z), ribi=4}, {coor=coord3d(tiles[1].x, tiles[1].y, tiles[1].z), ribi=1}]
					gui.add_message_at(b_player, "signals 5 tl " + coord3d_to_string(tiles_build[6]) + " & " + coord3d_to_string(tiles[1]), world.get_time())
				} else if ( diagonal_st == 9 ) {
					// ribi 9 to 9
					signal = [{coor=coord3d(tiles_build[0].x, tiles_build[0].y, tiles_build[0].z), ribi=1}, {coor=coord3d(tiles[way_len - 2].x, tiles[way_len - 2].y, tiles[way_len - 2].z), ribi=4}]
					gui.add_message_at(b_player, "signals diagonal tl " + coord3d_to_string(tiles_build[0]) + " & " + coord3d_to_string(tiles[way_len - 1]), world.get_time())
				} else if ( diagonal_st == 3 ) {
					// ribi 3 to 3
					signal = [{coor=coord3d(tiles_build[way_len - 3].x, tiles_build[way_len - 3].y, tiles_build[way_len - 3].z), ribi=2}, {coor=coord3d(tiles[1].x, tiles[1].y, tiles[1].z), ribi=8}]
					gui.add_message_at(b_player, "signals diagonal tl " + coord3d_to_string(tiles_build[way_len - 3]) + " & " + coord3d_to_string(tiles[1]), world.get_time())

				}
			}
		}

		if ( tiles_build == null ) {
			gui.add_message_at(b_player, " ERROR no double way found " + coord3d_to_string(tiles[0]), start_field)
			return false
		}


		// build way
		local err = null
		local sig_field = 0
		if ( start_field.get_way_dirs(wt) == 5 || start_field.get_way_dirs(wt) == 10 ) {
			if ( print_message_box == 1 ) {
				gui.add_message_at(b_player, "5/10 build tiles[0] " + coord3d_to_string(tiles[0]) + " to tiles_build[0] " + coord3d_to_string(tiles_build[0]), start_field)
			}
 			err = command_x.build_way(b_player, tiles[0], tiles_build[0], way_obj, true)
			if ( err == null ) {
				if ( print_message_box == 1 ) {
					gui.add_message_at(b_player, "  build tiles_build[0] " + coord3d_to_string(tiles_build[0]) + " to tiles_build[" + (way_len - 1) + "] " + coord3d_to_string(tiles_build[way_len - 1]), start_field)
				}
				err = command_x.build_way(b_player, tiles_build[0], tiles_build[way_len - 1], way_obj, true)
				if ( err != null ) {
					gui.add_message_at(b_player, " ERROR => build tile " + coord3d_to_string(tiles[0]) + " to tile " + coord3d_to_string(tiles_build[way_len - 1]), tiles[0])
					err = null
				}
				if ( print_message_box == 1 ) {
					gui.add_message_at(b_player, "  build tiles_build[" + (way_len - 1) + "] " + coord3d_to_string(tiles_build[way_len - 1]) + " to tiles[" + (way_len - 1) + "] " + coord3d_to_string(tiles[way_len - 1]), start_field)
				}
				err = command_x.build_way(b_player, tiles_build[way_len - 1], tiles[way_len - 1], way_obj, true)
				if ( err != null ) {
					gui.add_message_at(b_player, " ERROR => build tile " + coord3d_to_string(tiles_build[way_len - 1]) + " to tile " + coord3d_to_string(tiles[way_len - 1]), tiles[0])
					err = null
					// remove last tile
					local tool = command_x(tool_remover)
					tool.work(our_player, tiles_build[way_len - 1])
					// build
					err = command_x.build_way(b_player, tiles_build[way_len - 2], tiles[way_len - 2], way_obj, true)
					if ( settings.get_drive_on_left() ) {

					} else {
						if ( (tile_x(signal[1].coor.x, signal[1].coor.y, signal[1].coor.z).get_way_dirs(wt) == 11 || tile_x(signal[1].coor.x, signal[1].coor.y, signal[1].coor.z).get_way_dirs(wt) == 13) ) {
						sig_field = 1
						}
					}
				}
			}
		}
		else if ( tiles[0].get_way_dirs(wt) == 6 ) {
			if ( print_message_box == 1 ) {
				gui.add_message_at(b_player, "6 build tiles[0] " + coord3d_to_string(tiles[0]) + " to tiles_build[1] " + coord3d_to_string(tiles_build[1]), start_field)
			}
			err = command_x.build_way(b_player, tiles[0], tiles_build[1], way_obj, true)
			if ( err == null ) {
				if ( print_message_box == 1 ) {
					gui.add_message_at(b_player, "  build tiles_build[1] " + coord3d_to_string(tiles_build[1]) + " to tiles_build[" + (way_len - 3) + "] " + coord3d_to_string(tiles_build[way_len - 3]), start_field)
				}
				err = command_x.build_way(b_player, tiles_build[1], tiles_build[way_len - 3], way_obj, true)
				if ( err != null ) {
					gui.add_message_at(b_player, " ERROR => build tile " + coord3d_to_string(tiles_build[1]) + " to tile " + coord3d_to_string(tiles_build[way_len - 3]), tiles[0])
					err = null
				}
				if ( print_message_box == 1 ) {
					gui.add_message_at(b_player, "  build tiles_build[" + (way_len - 3) + "] " + coord3d_to_string(tiles_build[way_len - 3]) + " to tiles[" + (way_len - 1) + "] " + coord3d_to_string(tiles[way_len - 1]), start_field)
				}
				err = command_x.build_way(b_player, tiles_build[way_len - 3], tiles[way_len - 1], way_obj, true)
				if ( err != null ) {
					gui.add_message_at(b_player, " ERROR => build tile " + coord3d_to_string(tiles_build[way_len - 3]) + " to tile " + coord3d_to_string(tiles[way_len]), tiles[0])
					err = null
				}

			}
		}
		else if ( tiles[0].get_way_dirs(wt) == 9 ) {
			if ( print_message_box == 1 ) {
				gui.add_message_at(b_player, "9 build tiles[0] " + coord3d_to_string(tiles[0]) + " to tiles_build[1] " + coord3d_to_string(tiles_build[1]), start_field)
			}
			err = command_x.build_way(b_player, tiles[0], tiles_build[1], way_obj, true)
			if ( err == null ) {
				if ( print_message_box == 1 ) {
					gui.add_message_at(b_player, "  build tiles_build[1] " + coord3d_to_string(tiles_build[1]) + " to tiles_build[" + (way_len - 4) + "] " + coord3d_to_string(tiles_build[way_len - 4]), start_field)
				}
				err = command_x.build_way(b_player, tiles_build[1], tiles_build[way_len - 4], way_obj, true)
				if ( err != null ) {
					gui.add_message_at(b_player, " ERROR => build tile " + coord3d_to_string(tiles_build[1]) + " to tile " + coord3d_to_string(tiles_build[way_len - 4]), tiles[0])
					err = null
				}
				if ( print_message_box == 1 ) {
					gui.add_message_at(b_player, "  build tiles_build[" + (way_len - 4) + "] " + coord3d_to_string(tiles_build[way_len - 4]) + " to tiles[" + (way_len - 1) + "] " + coord3d_to_string(tiles[way_len - 1]), start_field)
				}
				err = command_x.build_way(b_player, tiles_build[way_len - 4], tiles[way_len - 1], way_obj, true)
				if ( err != null ) {
					gui.add_message_at(b_player, " ERROR => build tile " + coord3d_to_string(tiles_build[way_len - 4]) + " to tile " + coord3d_to_string(tiles[way_len - 1]), tiles[0])
					err = null
				}

			}
		}
		else if ( tiles[0].get_way_dirs(wt) == 3 ) {
			if ( print_message_box == 1 ) {
				gui.add_message_at(b_player, "3 build tiles[0] " + coord3d_to_string(tiles[0]) + " to tiles_build[1] " + coord3d_to_string(tiles_build[1]), start_field)
			}
			err = command_x.build_way(b_player, tiles[0], tiles_build[1], way_obj, true)
			if ( err == null ) {
				if ( print_message_box == 1 ) {
					gui.add_message_at(b_player, "  build tiles_build[1] " + coord3d_to_string(tiles_build[1]) + " to tiles_build[" + (way_len - 4) + "] " + coord3d_to_string(tiles_build[way_len - 4]), start_field)
				}
				err = command_x.build_way(b_player, tiles_build[1], tiles_build[way_len - 4], way_obj, true)
				if ( err != null ) {
					gui.add_message_at(b_player, " ERROR => build tile " + coord3d_to_string(tiles_build[1]) + " to tile " + coord3d_to_string(tiles_build[way_len - 4]), tiles[0])
					err = null
				}
				if ( print_message_box == 1 ) {
					gui.add_message_at(b_player, "  build tiles_build[" + (way_len - 4) + "] " + coord3d_to_string(tiles_build[way_len - 4]) + " to tiles[" + (way_len - 1) + "] " + coord3d_to_string(tiles[way_len - 1]), start_field)
				}
				err = command_x.build_way(b_player, tiles_build[way_len - 4], tiles[way_len - 1], way_obj, true)
				if ( err != null ) {
					gui.add_message_at(b_player, " ERROR => build tile " + coord3d_to_string(tiles_build[way_len - 4]) + " to tile " + coord3d_to_string(tiles[way_len - 1]), tiles[0])
					err = null
				}

			}
		}
		else if ( tiles[0].get_way_dirs(wt) == 12 ) {
			if ( print_message_box == 1 ) {
				gui.add_message_at(b_player, "12 build tiles[0] " + coord3d_to_string(tiles[0]) + " to tiles_build[1] " + coord3d_to_string(tiles_build[1]), start_field)
			}
			err = command_x.build_way(b_player, tiles[0], tiles_build[1], way_obj, true)
			if ( err == null ) {
				if ( print_message_box == 1 ) {
					gui.add_message_at(b_player, "  build tiles_build[1] " + coord3d_to_string(tiles_build[1]) + " to tiles_build[" + (way_len - 3) + "] " + coord3d_to_string(tiles_build[way_len - 3]), start_field)
				}
				err = command_x.build_way(b_player, tiles_build[1], tiles_build[way_len - 3], way_obj, true)
				if ( err != null ) {
					gui.add_message_at(b_player, " ERROR => build tile " + coord3d_to_string(tiles_build[1]) + " to tile " + coord3d_to_string(tiles_build[way_len - 3]), tiles[0])
					err = null
				}
				local t = 1
				if ( tiles[way_len - t].get_way_dirs(wt) == 3 ) {
					t = 0
				}
				if ( print_message_box == 1 ) {
					gui.add_message_at(b_player, "  build tiles_build[" + (way_len - 3) + "] " + coord3d_to_string(tiles_build[way_len - 3]) + " to tiles[" + (way_len - 1) + "] " + coord3d_to_string(tiles[way_len - 1]), start_field)
				}
				err = command_x.build_way(b_player, tiles_build[way_len - 3], tiles[way_len - 1], way_obj, true)
				if ( err != null ) {
					gui.add_message_at(b_player, " ERROR => build tile " + coord3d_to_string(tiles_build[way_len - 3]) + " to tile " + coord3d_to_string(tiles[way_len - 1]), tiles[0])
					err = null
				}

			}
		}

		// build signals
		if ( diagonal_st == 0 ) {
				// build signals
				local list = sign_desc_x.get_available_signs(wt)
				local obj_sign = null
				foreach(o in list) {
					if ( print_message_box == 2 ) {
						gui.add_message_at(b_player, "signals " + o.get_name(), start_field)
					}
					if (o.is_signal()) {
						obj_sign = o
						break
					}
				}

				local sig_1 = tile_x(signal[0].coor.x, signal[0].coor.y, signal[0].coor.z)
				local sig_2 = tile_x(signal[1].coor.x, signal[1].coor.y, signal[1].coor.z)
				if ( sig_1.find_object(mo_way) != null && sig_2.find_object(mo_way) != null ) {
					local tiles = [3, 5, 6, 9, 10, 12]
					local test = 0
					for ( local i = 0; i < tiles.len(); i++ ) {
						if ( sig_1.get_way_dirs(wt) == tiles[i] ) {
							test++
						}
						if ( sig_2.get_way_dirs(wt) == tiles[i] ) {
							test++
						}
					}
					if ( test == 2 ) {
						for ( local j=0; j < signal.len(); j++ ) {

							if ( print_message_box == 1 ) {
								gui.add_message_at(b_player, "signal to tile " + coord3d_to_string(tile_x(signal[j].coor.x, signal[j].coor.y, signal[j].coor.z)), start_field)
							}

							local fx = 0
							local fy = 0
							if ( j == 1 && d == 5 ) { //&& signal[j].coor.y == tiles[tiles.len()-2].y
								fy = sig_field
							} else if ( j == 1 && d == 10 ) { //&& signal[j].coor.x == tiles[tiles.len()-2].x
								fx = sig_field
							}

							local s_ribi = signal[j].ribi
							test = tile_x(signal[j].coor.x-fx, signal[j].coor.y-fy, signal[j].coor.z).get_way_dirs(wt)
							//gui.add_message_at(b_player, "signal set to ribi " + s_ribi, world.get_time())
							if ( test == 9 && s_ribi == 2 ) { s_ribi = 1 }
							if ( test == 12 && s_ribi == 1 ) { s_ribi = 8 }
							if ( print_message_box == 1 && signal[j].ribi != s_ribi ) {
								gui.add_message_at(b_player, coord3d_to_string(tile_x(signal[j].coor.x-fx, signal[j].coor.y-fy, signal[j].coor.z)) + " signal set to ribi new " + s_ribi, world.get_time())
							}

							while(true){
								local err = command_x.build_sign_at(b_player, tile_x(signal[j].coor.x-fx, signal[j].coor.y-fy, signal[j].coor.z), obj_sign)
								local ribi = tile_x(signal[j].coor.x-fx, signal[j].coor.y-fy, signal[j].coor.z).get_way_dirs_masked(wt)
								if (ribi == s_ribi)
									break
							}

						}

					}
				}

		}
		else if ( diagonal_st == 6 || diagonal_st == 9 || diagonal_st == 3 || diagonal_st == 12 ) {
				// build signals
				local list = sign_desc_x.get_available_signs(wt)
				local obj_sign = null
				foreach(o in list) {
					if ( print_message_box == 2 ) {
						gui.add_message_at(b_player, "signals " + o.get_name(), start_field)
					}
					if (o.is_signal()) {
						obj_sign = o
						break
					}
				}

				local sig_1 = tile_x(signal[0].coor.x, signal[0].coor.y, signal[0].coor.z)
				local sig_2 = tile_x(signal[1].coor.x, signal[1].coor.y, signal[1].coor.z)
				if ( sig_1.find_object(mo_way) != null && sig_2.find_object(mo_way) != null ) {
					local tiles = [3, 5, 6, 9, 10, 12]
					local test = 0
					for ( local i = 0; i < tiles.len(); i++ ) {
						if ( sig_1.get_way_dirs(wt) == tiles[i] ) {
							test++
						}
						if ( sig_2.get_way_dirs(wt) == tiles[i] ) {
							test++
						}
					}
					if ( test == 2 ) {

						for ( local j=0; j < signal.len(); j++ ) {

							if ( print_message_box == 3 ) {
								gui.add_message_at(b_player, "signal to tile " + coord3d_to_string(tile_x(signal[j].coor.x, signal[j].coor.y, signal[j].coor.z)), world.get_time())
							}

							while(true){
								local err = command_x.build_sign_at(b_player, tile_x(signal[j].coor.x, signal[j].coor.y, signal[j].coor.z), obj_sign)
								local ribi = tile_x(signal[j].coor.x, signal[j].coor.y, signal[j].coor.z).get_way_dirs_masked(wt)
								if (ribi == signal[j].ribi)
									break
							}

						}

					}
				}
		}

		print_message_box = 0
		return true
	}
}

/*
 *	start	=	start field line
 *	end		=	end field line
 *	wt		=	waytype
 *	l			= stations distance
 *	c			= count of double ways
 *
 */
function check_way_line(start, end, wt, l, c) {
	/*
	 * 1 =
	 * 2 = straight
	 * 3 = diagonal
	 */
	local print_message_box = 1

	if ( print_message_box > 0 ) {
		gui.add_message_at(our_player, " ### check_way_line ### " + coord3d_to_string(start), start)
	}
/*
	gui.add_message_at(our_player, "end line " + coord3d_to_string(end), end)
	gui.add_message_at(our_player, "length " + l, world.get_time())
	gui.add_message_at(our_player, "count " + c, world.get_time())
*/
	local nexttile = [start]
	local d = 0

	// count double ways
	local start_fields = []
	local fc = 0

	local dfcl = 0
	local dfcr = 0

	local dc = 0
	local di = 0
	local r = 0
	// distance double ways
	local as = (l / (c + 1))
	/*
	if ( l < 140 ) {
		as - 25
	} else if ( l >= 140 && l < 400 ) {
		as - 14
	} else {
		as + 20
	}*/
	//as = as - ( c * 16 )
	if ( print_message_box >= 0 ) {
		gui.add_message_at(our_player, c + " double way search", start)
		gui.add_message_at(our_player, "as " + as + " l " + l + " c " + c, start)
	}
	local s = []
	for (local i = 0; i < c; i++ ) {
		if ( i == 0 ) {
			if ( c == 1 ) {
				s.append(as - 10 )
			} else {
				s.append(as - (as * 0.4).tointeger() - 10 )
			}
		} else {
			s.append(s[i-1]+as)
		}


		if ( print_message_box == 1 ) {
			gui.add_message_at(our_player, "  s[" + (i) + "] " + s[i], world.get_time())
		}
	}

	local sign = 0
	local reset = 0

	d = start.get_way_dirs(wt)

	if ( d == 10 ) {
		if ( start.x < end.x ) {
			d = 2
		} else {
			d = 8
		}
	} else if ( d == 5 ) {
		if ( start.y < end.y ) {
			d = 4
		} else {
			d = 1
		}
	}


	local stl = 0
	local str = 0
	local dst = 0
	for ( local i = 1; i <= l; i++ ) {

		// check to signal
		local sig = nexttile[i-1].find_object(mo_signal)
		//gui.add_message_at(our_player, "find_object(mo_signal) " + sig, nexttile[i-1])
		if ( sig != null ) {
			sign++
			if ( sign == c ) {
				gui.add_message_at(our_player, c + " double way found, no build ", world.get_time())
				return true
			}
			//gui.add_message_at(our_player, "signals " + sign, world.get_time())
		}

		di = dc
		dc = d

		local st_tile = null
		if ( i < 5 ) {
			st_tile = nexttile[i-1].find_object(mo_building)
		}

		if ( d == 10 || d == 11 || d == 14 ) {

	 		if ( (i >= 1 && ( nexttile[i-2].x < nexttile[i-1].x) || reset == 1 ) ) {
				d = 2
			} else if ( di == 10 && d == 11 ) {
				d = 1
			} else if ( di == 5 && d == 14 ) {
				d = 2
			} else {
				d = 8
			}

		} else if ( d == 5 || d == 7 || d == 13 ) {
			//gui.add_message_at(our_player, " * i " + i + " d == 5 || d == 7 || d == 13 " + coord3d_to_string(nexttile[i-1]), world.get_time())
			if ( (i >= 1 && ( nexttile[i-2].y < nexttile[i-1].y) || reset == 1 ) ) {
				d = 4
			} else if ( di == 9 && d == 5 ) {
				d = 1
			} else if ( ( di == 9 || di == 10 ) && d == 7 ) {
				gui.add_message_at(our_player, " * i " + i + " di == 9 || di == 10 ) && d == 7 " + coord3d_to_string(nexttile[i-1]), world.get_time())
				// next 1 or 4
				if ( di == 9 && d == 7 && nexttile[i-1].y < nexttile[i-2].y ) {
					d = 1
				} else {
					local t = nexttile[i-1].get_neighbour(wt, 4)
					gui.add_message_at(our_player, " *  " + coord3d_to_string(t) + " d " + t.get_way_dirs(wt), world.get_time())
					if ( t.get_way_dirs(wt) == 4 ) {
						d = 1
					} else {
						d = 4
					}
				}
			} else {
				d = 1
			}
		} else if ( d == 12 ) {
			if ( nexttile[i-2].x < nexttile[i-1].x ) {
				d = 4
			} else {
				d = 8
			}
		} else if ( d == 9 ) {
			if ( nexttile[i-2].y < nexttile[i-1].y ) {
					d = 8
			} else if ( ( di == 13 || di == 7 ) && d == 9 ) {
				d = 1
			} else {
				d = 1
			}

		} else if ( d == 6 ) {
			if ( nexttile[i-2].x > nexttile[i-1].x ) {
				d = 4
			} else if ( nexttile[i-2].y > nexttile[i-1].y ) {
				d = 2
			} else if ( (di == 5 || di == 9) && d == 6 ) {
				d = 2
			} else if ( di == 13 && d == 6 ) {
				d = 4
			} else {
				d = 8
			}

		} else if ( d == 3 ) {
			if ( nexttile[i-2].y < nexttile[i-1].y ) {
				d = 2
			} else {
				d = 1
			}
		} else if ( d == 15 ) {
			if ( di == 10 && nexttile[i-2].x < nexttile[i-1].x ) {
				d = 2
			} else {
				d = 8
			}
			if ( di == 5 && nexttile[i-2].y < nexttile[i-1].y ) {
				d = 1
			} else {
				d = 4
			}
		}


		//gui.add_message_at(our_player, " * " + coord3d_to_string(nexttile[i-1]) + " d " + d + " i " + i + " dc " + dc + " di " + di, world.get_time())
		local t = nexttile[i-1].get_neighbour(wt, d)
		if ( t == null && print_message_box == 4 ) {
			gui.add_message_at(our_player, " next field pos " + t, world.get_time())
		} else if ( print_message_box == 4 ) {
			gui.add_message_at(our_player, " next field pos " + coord3d_to_string(t), world.get_time())
		}

		local test = null
		local test_d = t.get_way_dirs(wt)
		if ( wt == wt_rail && ( test_d == 1 || test_d == 2 || test_d == 4 || test_d == 8 ) ) {
			test = t.find_object(mo_depot_rail)
		}

		if ( i < 6 && print_message_box == 5 ) {
			gui.add_message_at(our_player, " tile * " + coord3d_to_string(t) + " dc " + dc + " di " + di + " d " + d, t)
		}

		if ( test == null ) {
			d = t.get_way_dirs(wt)
		} else {
			if ( nexttile[i-1].get_way_dirs(wt) == 11 && nexttile[i-2].get_way_dirs(wt) == 10 ) {
				d = 1
			} else if ( nexttile[i-1].get_way_dirs(wt) == 14 && nexttile[i-2].get_way_dirs(wt) == 10 ) {
				d = 4
			} else if ( nexttile[i-1].get_way_dirs(wt) == 7 && nexttile[i-2].get_way_dirs(wt) == 5 ) {
				d = 2
			} else if ( nexttile[i-1].get_way_dirs(wt) == 7 && nexttile[i-2].get_way_dirs(wt) == 10 ) {
				d = 1
			} else if ( nexttile[i-1].get_way_dirs(wt) == 13 && nexttile[i-2].get_way_dirs(wt) == 5 ) {
				d = 8
			}
			//gui.add_message_at(our_player, " * tile * " + coord3d_to_string(nexttile[i-1]) + " d " + d, nexttile[i-1])
			t = nexttile[i-1].get_neighbour(wt, d)
			d = t.get_way_dirs(wt)
		}


		// len from double track
		local way_len = 8
		if ( d == 6 || d == 9 ) {
			way_len = 9
		} else if ( d == 3 || d == 12 ) {
			way_len = 9
		}

		// ribi 7, 11, 13, 14
		if ( dc == 7 && ( d == 1 || d == 4 ) ) {
			d = 2
			t = nexttile[i-1].get_neighbour(wt, d)
			d = t.get_way_dirs(wt)
		} else if ( dc == 13 && ( d == 1 || d == 4 ) ) {
			d = 8
			t = nexttile[i-1].get_neighbour(wt, d)
			d = t.get_way_dirs(wt)
		} else if ( dc == 11 && ( d == 2 || d == 8 ) ) {
			d = 1
			t = nexttile[i-1].get_neighbour(wt, d)
			d = t.get_way_dirs(wt)
		} else if ( di != 5 && dc == 14 && ( d == 2 || d == 8 ) ) {
			d = 4
			t = nexttile[i-1].get_neighbour(wt, d)
			d = t.get_way_dirs(wt)
		}
		// reset by wrong direction
		// end from way
		if ( i > 1 && i < 5 && dc == 10 && ( d == 2 || d == 8 ) ) { //nexttile[0].x == t.x && nexttile[0].y == t.y && nexttile[0].z == t.z ) {
			nexttile.clear()
			l = l + i - 1
			i = 0
			reset = 1
			if ( print_message_box == 1 ) {
				gui.add_message_at(our_player, "*#* nexttile reset - t " + coord3d_to_string(t), world.get_time())
			}
		} else if ( i > 1 && i < 5 && dc == 5 && ( d == 1 || d == 4 ) ) { //nexttile[0].x == t.x && nexttile[0].y == t.y && nexttile[0].z == t.z ) {
			nexttile.clear()
			l = l + i - 1
			i = 0
			reset = 1
			if ( print_message_box == 1 ) {
				gui.add_message_at(our_player, "*#* nexttile reset - t " + coord3d_to_string(t), world.get_time())
			}
		} else {
			reset = 0
		}

		nexttile.append(t)

		// diagonal start ribi
		if ( ( dc == 5 && d == 6 ) || ( dc == 5 && d == 9 ) ) {
			dst = 5
		} else if ( ( dc == 10 && d == 6 ) || ( dc == 10 && d == 9 ) ) {
			dst = 10
		} else if ( ( dc == 5 && d == 3 ) || ( dc == 5 && d == 12 ) ) {
			dst = 5
		} else if ( ( dc == 10 && d == 3 ) || ( dc == 10 && d == 12 ) ) {
			dst = 10
		} else if ( d == 5 || d == 10 ) {
			dst = 0
		}

		local st = 0
		if ( dst == 0 && fc == 0 && ( t.get_slope() > 0 || t.is_bridge() ) ) {
			// check slope to start field for double way
			//gui.add_message_at(our_player, " ### check first tile " + coord3d_to_string(t), t)
			st = 1
		} else if ( t.is_bridge() ) {
			// check bridge
			//gui.add_message_at(our_player, " ### check bridge " + coord3d_to_string(t), t)
			st = 1
		} else if ( t.get_way_dirs(wt) == 10 ) {
			// check left & right ground and empty
			if ( !tile_x(t.x, t.y + 1, t.z).is_ground() && !tile_x(t.x, t.y - 1, t.z).is_ground() ) {
				str = 0
			} else if ( !tile_x(t.x, t.y + 1, t.z).is_empty() && !tile_x(t.x, t.y - 1, t.z).is_empty() ) {
				stl = 0
			} else {
				// field right empty and ground
				if ( tile_x(t.x, t.y + 1, t.z).is_ground() && tile_x(t.x, t.y + 1, t.z).is_empty() ) {
					str++
				} else {
					str = 0
				}
				// field left empty and ground
				if ( tile_x(t.x, t.y - 1, t.z).is_ground() && tile_x(t.x, t.y - 1, t.z).is_empty() ) {
					stl++
				} else {
					stl = 0
				}
			}
			// end diagonal way
			dst = 0
			dfcl = 0
			dfcr = 0
		} else if ( t.get_way_dirs(wt) == 5 ) {
			// check left & right ground and empty
			if ( !tile_x(t.x + 1, t.y, t.z).is_ground() && !tile_x(t.x - 1, t.y, t.z).is_ground() ) {
				str = 0
			} else if ( !tile_x(t.x + 1, t.y, t.z).is_empty() && !tile_x(t.x - 1, t.y, t.z).is_empty() ) {
				stl = 0
			} else {
				// field right empty and ground
				if ( tile_x(t.x + 1, t.y, t.z).is_ground() && tile_x(t.x + 1, t.y, t.z).is_empty() ) {
					str++
				} else {
					str = 0
				}
				// field left empty and ground
				if ( tile_x(t.x - 1, t.y, t.z).is_ground() && tile_x(t.x - 1, t.y, t.z).is_empty() ) {
					stl++
				} else {
					stl = 0
				}
			}
			// end diagonal way
			dst = 0
			dfcl = 0
			dfcr = 0
		} else if ( dst > 0 && t.get_way_dirs(wt) == 6 || t.get_way_dirs(wt) == 9 ) {
			if ( print_message_box == 3 && i >= s[0] && i < (s[0] + way_len) && str == 0 && stl == 0 ) {
				gui.add_message_at(our_player, " # test 6/9 " + coord3d_to_string(t), t)
				gui.add_message_at(our_player, " diagonal start dst " + dst, world.get_time())
				gui.add_message_at(our_player, " dc " + dc + " dfcl " + dfcl + " dfcr " + dfcr + " - stl " + stl + " - str " + str, world.get_time())
			}
			if ( dst == 5 || dst == 10 ) {
				local check_tile_strs = null
				local check_tile_str = null
				local check_tile_stls = null
				local check_tile_stl = null
				if ( nexttile[i-2].x < nexttile[i-1].x || nexttile[i-2].y > nexttile[i-1].y ) {
					check_tile_strs = tile_x(t.x, t.y - 1, t.z)
					check_tile_str = tile_x(t.x, t.y - 2, t.z)
					check_tile_stls = tile_x(t.x + 1, t.y, t.z)
					check_tile_stl = tile_x(t.x + 2, t.y, t.z)
					//gui.add_message_at(our_player, " diagonal test A " + " i " + i + " - tile check_tile_str " + coord3d_to_string(check_tile_str), world.get_time())
				} else {
					check_tile_strs = tile_x(t.x - 1, t.y, t.z)
					check_tile_str = tile_x(t.x - 2, t.y, t.z)
					check_tile_stls = tile_x(t.x, t.y + 1, t.z)
					check_tile_stl = tile_x(t.x, t.y + 2, t.z)
					//gui.add_message_at(our_player, " diagonal test B " + " i " + i + " - tile check_tile_str " + coord3d_to_string(check_tile_str), world.get_time())
				}

				local tile_on_map_r = 0
				local tile_on_map_l = 0
				if ( world.is_coord_valid(check_tile_strs) && world.is_coord_valid(check_tile_str) ) {
					// tile is in range map
					tile_on_map_r = 1
				} else {
					// tile is out of range map
					str = 0
				}

				if ( world.is_coord_valid(check_tile_stls) && world.is_coord_valid(check_tile_stl) ) {
					// tile is in range map
					tile_on_map_l = 1
				} else {
					// tile is out of range map
					stl = 0
				}

				if ( ( dc == 6 && t.get_way_dirs(wt) == 9 ) || ( dc == 9 && t.get_way_dirs(wt) == 6 ) ) {
					if ( t.get_way_dirs(wt) == 6 && str == 0 ) {
						if ( print_message_box == 3 && i >= s[0] && i < (s[0] + way_len) ) {
							gui.add_message_at(our_player, " - check start ribi 6 " + coord3d_to_string(t), t)
						}
						if ( tile_on_map_r == 1 ) {
							if ( check_tile_strs.is_ground() && check_tile_strs.is_empty() ) { //&& tile_x(t.x, t.y - 2, t.z).is_ground() && tile_x(t.x, t.y - 2, t.z).is_empty()
								str++
							} else {
								str = 0
							}
						} else {
							gui.add_message_at(our_player, " ERROR test tile out of map " + coord3d_to_string(check_tile_strs) + " / " + coord3d_to_string(check_tile_str), t)
						}
					} else if ( t.get_way_dirs(wt) == 9 && stl == 0 ) {
						if ( print_message_box == 3 && i >= s[0] && i < (s[0] + way_len) ) {
							gui.add_message_at(our_player, " - check start ribi 9 " + coord3d_to_string(t), t)
						}
						if ( tile_on_map_l == 1 ) {
							if ( check_tile_stls.is_ground() && check_tile_stls.is_empty() ) { //&& tile_x(t.x, t.y + 2, t.z).is_ground() && tile_x(t.x, t.y + 2, t.z).is_empty()
								stl++
							} else {
								stl = 0
							}
						} else {
							gui.add_message_at(our_player, " ERROR test tile out of map " + coord3d_to_string(check_tile_stls) + " / " + coord3d_to_string(check_tile_stl), t)
						}
					}

					if ( t.get_way_dirs(wt) == 6 || t.get_way_dirs(wt) == 9 ) {
						if ( print_message_box == 3 && i >= s[0] && i < (s[0] + way_len) ) {
							gui.add_message_at(our_player, " - check ribi 6/9 l/r " + coord3d_to_string(t) + " d " + t.get_way_dirs(wt), t)
						}
						if ( tile_on_map_r == 1 ) {
							if ( str > 0 && str <= way_len - 3 ) {
								if ( check_tile_str.is_ground() && check_tile_str.is_empty() ) {
									//gui.add_message_at(player_x(1), " + str " + str, world.get_time())
									str++
								} else {
									str = 0
								}

							} else if ( str > way_len - 3 && str <= way_len ) {
								//gui.add_message_at(player_x(1), " ++ str " + str, world.get_time())
								str++
							}
						} else {
							gui.add_message_at(our_player, " ERROR test tile out of map ", t)
						}

						if ( tile_on_map_l == 1 ) {
							if ( stl > 0 && stl <= way_len - 3 ) {
								if ( check_tile_stl.is_ground() && check_tile_stl.is_empty() ) {
									//gui.add_message_at(player_x(1), " + stl " + stl, world.get_time())
									stl++
								} else {
									stl = 0
								}
							} else if ( stl > way_len - 3 && stl <= way_len ) {
								//gui.add_message_at(player_x(1), " ++ stl " + stl, world.get_time())
								stl++
							}
						} else {
							gui.add_message_at(our_player, " ERROR test tile out of map ", t)
						}

					}
					if ( print_message_box == 2 && i >= s[0] && i < (s[0] + way_len + 1) ) {
						gui.add_message_at(our_player, " -- stl " + stl + " - str " + str + " way_len " + way_len, world.get_time())
					}
				}
			}
		} else if ( dst > 0 && t.get_way_dirs(wt) == 3 || t.get_way_dirs(wt) == 12 ) {
			if ( print_message_box == 4 && i >= s[0] && i < (s[0] + way_len) && str == 0 && stl == 0 ) { //
				gui.add_message_at(our_player, " # test 3/12 " + coord3d_to_string(t), t)
				gui.add_message_at(our_player, " diagonal start dst " + dst, world.get_time())
				gui.add_message_at(our_player, " dc " + dc + " dfcl " + dfcl + " dfcr " + dfcr + " - stl " + stl + " - str " + str, world.get_time())
			}
			if ( dst == 5 || dst == 10 ) {
				local check_tile_strs = null
				local check_tile_str = null
				local check_tile_stls = null
				local check_tile_stl = null
				if ( nexttile[i-2].x > nexttile[i-1].x || nexttile[i-2].y > nexttile[i-1].y ) {
					check_tile_strs = tile_x(t.x - 1, t.y, t.z)
					check_tile_str = tile_x(t.x - 2, t.y, t.z)
					check_tile_stls = tile_x(t.x, t.y + 1, t.z)
					check_tile_stl = tile_x(t.x, t.y + 2, t.z)
				} else {
					check_tile_strs = tile_x(t.x, t.y - 1, t.z)
					check_tile_str = tile_x(t.x, t.y - 2, t.z)
					check_tile_stls = tile_x(t.x + 1, t.y, t.z)
					check_tile_stl = tile_x(t.x + 2, t.y, t.z)
				}

				local tile_on_map_r = 0
				local tile_on_map_l = 0
				if ( world.is_coord_valid(check_tile_strs) && world.is_coord_valid(check_tile_str) ) {
					// tile is in range map
					tile_on_map_r = 1
				} else {
					// tile is out of range map
					str = 0
				}

				if ( world.is_coord_valid(check_tile_stls) && world.is_coord_valid(check_tile_stl) ) {
					// tile is in range map
					tile_on_map_l = 1
				} else {
					// tile is out of range map
					stl = 0
				}

				if ( ( dc == 3 && t.get_way_dirs(wt) == 12 ) || ( dc == 12 && t.get_way_dirs(wt) == 3 ) ) {
					if ( t.get_way_dirs(wt) == 3 && str == 0 ) {
						if ( print_message_box == 4 ) { //&& i >= s[0] && i < (s[0] + way_len )
							gui.add_message_at(our_player, " - check start ribi 3 " + coord3d_to_string(t), t)
						}
						if ( tile_on_map_r == 1 ) {
							if ( check_tile_strs.is_ground() && check_tile_strs.is_empty()  ) {
								str++
							} else {
								str = 0
							}
						} else {
							gui.add_message_at(our_player, " ERROR test tile out of map ", t)
						}
					} else if ( t.get_way_dirs(wt) == 12 && stl == 0 ) {
						if ( print_message_box == 4 && i >= s[0] && i < (s[0] + way_len) ) { //
							gui.add_message_at(our_player, " - check start ribi 12 " + coord3d_to_string(t), t)
						}
						if ( tile_on_map_l == 1 ) {
							if ( check_tile_stls.is_ground() && check_tile_stls.is_empty() ) {
								stl++
							} else {
								stl = 0
							}
						} else {
							gui.add_message_at(our_player, " ERROR test tile out of map ", t)
						}
					}

					if ( t.get_way_dirs(wt) == 3 || t.get_way_dirs(wt) == 12 ) {
						if ( print_message_box == 4 && i >= s[0] && i < (s[0] + way_len) ) {
							gui.add_message_at(our_player, " - check ribi 3/12 l/r " + coord3d_to_string(t) + " d " + t.get_way_dirs(wt), t)
						}
						if ( tile_on_map_r == 1 ) {
							if ( str > 0 && str <= way_len - 3 ) {
								if ( check_tile_str.is_ground() && check_tile_str.is_empty() ) {
									//gui.add_message_at(player_x(1), " + str " + str, world.get_time())
									str++
								} else {
									str = 0
								}
							} else if ( str > way_len - 3 && str <= way_len ) {
								//gui.add_message_at(player_x(1), " ++ str " + str, world.get_time())
								str++
							}
						} else {
							gui.add_message_at(our_player, " ERROR test tile out of map ", t)
						}

						if ( tile_on_map_l == 1 ) {
							if ( stl > 0 && stl <= way_len - 3 ) {
								if ( check_tile_stl.is_ground() && check_tile_stl.is_empty() ) {
									//gui.add_message_at(player_x(1), " + stl " + stl, world.get_time())
									stl++
								} else {
									stl = 0
								}
							} else if ( stl > way_len - 3 && stl <= way_len ) {
								//gui.add_message_at(player_x(1), " ++ stl " + stl, world.get_time())
								stl++
							}
						} else {
							gui.add_message_at(our_player, " ERROR test tile out of map ", t)
						}
					}
					if ( print_message_box == 4 && i >= s[0] && i < (s[0] + way_len + 1) ) {
						gui.add_message_at(our_player, " -- stl " + stl + " - str " + str + " way_len " + way_len, world.get_time())
					}

				}
			}

		}


		if ( dst == 0 && dc == d && i >= s[r] && !t.has_two_ways() && st == 0 ) {
			if ( print_message_box == 2 ) {
				gui.add_message_at(our_player, "  stl " + stl + " str " + str + " fc " + fc + " * " + coord3d_to_string(t), t)
			}
			if ( stl == fc + 1 || str == fc + 1 ) {
				fc++
				dfcr = 0
				dfcl = 0
			} else {
				fc = 0
				stl = 0
				str = 0
			}
		} else if ( dst > 0 && i >= s[r] && st == 0 ) {
			if ( print_message_box == 3 ) {
				gui.add_message_at(player_x(1), "  stl " + stl + " str " + str + " fc " + fc + " * " + coord3d_to_string(t), t)
				gui.add_message_at(player_x(1), "  dfcl " + dfcl + " dfcr " + dfcr, world.get_time())
			}
			if ( stl == dfcl + 2 ) {
				dfcl++
				fc = 0
			} else {
				dfcl = 0
				stl = 0
			}
			if ( str == dfcr + 2 ) {
				dfcr++
				fc = 0
			} else {
				dfcr = 0
				str = 0
			}
		} else {
			fc = 0
			stl = 0
			str = 0
		}

		if ( print_message_box == 4 ) {
			gui.add_message_at(player_x(0), "  fc " + fc + " s[" + r + "] " + s[r] + " i " + i + " - " + l + " dc " + dc + " di " + di + " d " + d + " * " + coord3d_to_string(t), world.get_time())
		}

		if ( i == s[r] ) {
			gui.add_message_at(our_player, " s[" + r + "] " + s[r] + " i " + i + " is tile " + coord3d_to_string(t), t)
		}

		if ( i >= s[r] && ( fc >= way_len || dfcl >= way_len || dfcr >= way_len ) && start_fields.len() < c) {
			if ( nexttile[i-1].x > nexttile[i].x || ( nexttile[i-1].y > nexttile[i].y && fc > 0 ) || ( nexttile[i-1].y > nexttile[i].y && fc == 0 ) ) {
				if ( nexttile[i].get_slope() == 0 ) {
					gui.add_message_at(our_player, " add nexttile[i] id = " + i + " " + coord3d_to_string(t), t)
					start_fields.append(nexttile[i])
				/*} else if ( nexttile[i].get_slope() == 0 && ( nexttile[i-1].get_way_dirs(wt) == 10 || nexttile[i-1].get_way_dirs(wt) == 5 ) ) {
					start_fields.append(nexttile[i-1])*/
				}
			} else {
				if ( nexttile[i-way_len-1].get_slope() == 0 ) {
				//if ( fc >= way_len ) {
					if ( nexttile[i].get_way_dirs(wt) == 3 || nexttile[i].get_way_dirs(wt) == 12 ) {
						start_fields.append(nexttile[i-way_len])
						gui.add_message_at(our_player, " add nexttile[i-way_len] id = " + (i-way_len) + " " + coord3d_to_string(t), t)

					} else {
						start_fields.append(nexttile[i-way_len+1])
						gui.add_message_at(our_player, " add nexttile[i-way_len+1] id = " + (i-way_len+1) + " " + coord3d_to_string(t), t)
					}
				/*} else if ( dfcl >= way_len ) {
					start_fields.append(nexttile[i-way_len+1])
				} else if ( dfcr >= way_len ) {
					start_fields.append(nexttile[i-way_len+1])*/
				}
				/*} else if ( nexttile[i-way_len].get_slope() == 0 && ( nexttile[i-way_len].get_way_dirs(wt) == 10 || nexttile[i-way_len].get_way_dirs(wt) == 5 ) ) {
					start_fields.append(nexttile[i-way_len])
				}*/
			}
			if ( print_message_box == 2 ) {
				gui.add_message_at(our_player, "  tile s[" + r + "] " + coord3d_to_string(start_fields[r]), start_fields[r])
				gui.add_message_at(our_player, "  i " + i + "  r " + r + "  s[" + r + "] " + s[r], world.get_time())
				gui.add_message_at(our_player, "* " + coord3d_to_string(t), world.get_time())
			}
			if ( r < c ) {
				if ( r < s.len() - 1 ) { r++ }
				if ( i > s[r] ) {
					s[r] = i + 10
				}
			}
		}


		//gui.add_message_at(our_player, "  tile d " + d + " * " + coord3d_to_string(t), world.get_time())
	}

	if ( c == 0 ) {
		return nexttile
	} else {
		return start_fields
	}

}