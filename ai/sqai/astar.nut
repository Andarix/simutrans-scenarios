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
	weight   = -1   // heuristic cost to reach target
	dist     = -1   // distance to target
	constructor(c, p, co, w, d)
	{
		x = c.x
		y = c.y
		z = c.z
		previous = p
		cost     = co
		weight   = w
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
	constructor(c, p, co, w, d, di, fl=0)
	{
		base.constructor(c, p, co, w, d)
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
		local print_message_box = 1
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
					local node = ab_node(to, cnode, cost, weight, dist, d)

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
						// set distance to 1 if at a target tile,
						// still route might come back to this tile in a loop (?)
						// but if there is space for a loop there is also place for another target tile (?)
						local dist = max(estimate_distance(to), 1)

						local cost   = cnode.cost + move
						local weight = cost + dist
						local node = ab_node(to, cnode, cost, weight, dist, d, 1 /*bridge*/)

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
					err = command_x.build_road(our_player, route[i-1], route[i], way, true, true)
					if (err) gui.add_message_at(our_player, "Failed to build road from  " + coord_to_string(route[i-1]) + " to " + coord_to_string(route[i]) +"\n" + err, route[i])
				}
				else if (route[i-1].flag == 1) {
					err = command_x.build_bridge(our_player, route[i], route[i-1], bridger.bridge)
					if (err) gui.add_message_at(our_player, "Failed to build bridge from  " + coord_to_string(route[i-1]) + " to " + coord_to_string(route[i]) +"\n" + err, route[i])
				}
				if (err) {
					return { err =  err }
				}
			}
			return { start = route[ route.len()-1], end = route[0] }
		}
		print("No route found")
		return { err =  "No route" }
	}
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
 * function for check station lenght
 * 
 * pl = player
 * starts_field = tile station from plan_simple_connection
 * st_lenght = stations fields count
 * wt = waytype
 * build = 0 -> test ; 1 -> build
 * 
 */
function check_station(pl, starts_field, st_lenght, wt, build = 1) {

		// print messages box 
		// 1 
		// 2 
		local print_message_box = 2

		if ( print_message_box == 2 ) {
			gui.add_message_at(pl, " --- start field : " + coord3d_to_string(starts_field), world.get_time())
		}
					
		local a = false
		local b = 0
		local b1_tile = null
		local tile_build = 0 
		local st_build = false 
		local err = null
		
		local t_start = tile_x(c_start.x, c_start.y, c_start.z)
		local t_end = tile_x(c_end.x, c_end.y, c_end.z)
	
					
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
		local x = 0  
		// get_dirs().to_coord()
		for ( local i = 0; i < 2; i++ ) {
			switch (d) {
		    case 1:
					// check n
					if ( print_message_box == 2 ) { 
						gui.add_message_at(pl, " ---> dir : 8", world.get_time())
					}	
          for ( x = 1; x < st_lenght; x++ ) {
						b1_tile = tile_x(starts_field.x, starts_field.y + x, starts_field.z) 						
						if ( print_message_box == 2 ) { 
							gui.add_message_at(pl, " ---> test : " + coord3d_to_string(b1_tile), world.get_time())
						}	

						if ( test_field(pl, b1_tile, wt, 5, starts_field.z) && b_tile.len() < st_lenght ) {
							if ( print_message_box == 2 ) { 
								gui.add_message_at(pl, " ---=> add tile : " + coord3d_to_string(b1_tile), world.get_time())
							}
						  b_tile.append(b1_tile) 
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
          for ( x = 1; x < st_lenght; x++ ) {
						b1_tile = tile_x(starts_field.x - x, starts_field.y, starts_field.z) 
						if ( print_message_box == 2 ) { 
							gui.add_message_at(pl, " ---> test : " + coord3d_to_string(b1_tile), world.get_time())
						}

						if ( test_field(pl, b1_tile, wt, 10, starts_field.z) && b_tile.len() < st_lenght ) {
							if ( print_message_box == 2 ) { 
								gui.add_message_at(pl, " ---=> add tile : " + coord3d_to_string(b1_tile), world.get_time())
							}
						  b_tile.append(b1_tile) 
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
          for ( x = 1; x < st_lenght; x++ ) {
						b1_tile = tile_x(starts_field.x, starts_field.y - x, starts_field.z) 
						if ( print_message_box == 2 ) { 
							gui.add_message_at(pl, " ---> test : " + coord3d_to_string(b1_tile), world.get_time())
						}

						if ( test_field(pl, b1_tile, wt, 5, starts_field.z) && b_tile.len() < st_lenght ) {
							if ( print_message_box == 2 ) { 
								gui.add_message_at(pl, " ---=> add tile : " + coord3d_to_string(b1_tile), world.get_time())
							}
						  b_tile.append(b1_tile) 
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
          for ( x = 1; x < st_lenght; x++ ) {
						b1_tile = tile_x(starts_field.x + x, starts_field.y, starts_field.z) 
						if ( print_message_box == 2 ) { 
							gui.add_message_at(pl, " ---> test : " + coord3d_to_string(b1_tile), world.get_time())
						}	

						if ( test_field(pl, b1_tile, wt, 10, starts_field.z) && b_tile.len() < st_lenght ) {
							if ( print_message_box == 2 ) { 
								gui.add_message_at(pl, " ---=> add tile : " + coord3d_to_string(b1_tile), world.get_time())
							}
						  b_tile.append(b1_tile) 
						} else {
							d = 2
							break
						}
							
					}
          
					break

			}  // end switch 
			
			if ( b_tile.len() < st_lenght && i == 1 ) { 
				// search other place for station
				if ( print_message_box == 2 ) { 
					gui.add_message_at(pl, " -#-=> not place found for station ", world.get_time())
				} 
				if ( d == 1 || d == 4 ) {
					b_tile.clear()
					// test e
          for ( x = 1; x <= st_lenght; x++ ) {
						b1_tile = tile_x(starts_field.x + x, starts_field.y, starts_field.z) 
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
          	for ( x = 1; x <= st_lenght; x++ ) {
							b1_tile = tile_x(starts_field.x - x, starts_field.y, starts_field.z) 
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
					if ( b_tile.len() == st_lenght ) {
						err = command_x.build_way(pl, starts_field, b_tile[0], planned_way, true) 

					}				
				} else if ( d == 2 || d == 8 ) {
					b_tile.clear()
					// test s
          for ( x = 1; x <= st_lenght; x++ ) {
						b1_tile = tile_x(starts_field.x, starts_field.y + x, starts_field.z) 
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
          	for ( x = 1; x <= st_lenght; x++ ) {
							b1_tile = tile_x(starts_field.x, starts_field.y - x, starts_field.z) 
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
					if ( b_tile.len() == st_lenght ) {
						err = command_x.build_way(pl, starts_field, b_tile[0], planned_way, true) 

					}				
				}
				if ( !err ) {
					if ( c_start == starts_field ) {
						c_start = b_tile[0]
					} else if ( c_end == starts_field ) {
						c_end == b_tile[0]
					}
				}
			}
			
			// build station
			if ( b_tile.len() == st_lenght && build == 1) {
				st_build = expand_station(pl, b_tile, wt) 
				break
			} else if ( b_tile.len() == st_lenght && build == 0 ) {
				st_build = true 
				break
			}
						
		} 
		
		if ( st_build ) {
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

	local print_message_box = 2 
	local err = null 
	local z = null 
	
	if ( t_tile.is_empty() && t_tile.get_slope() == 0 ) {
		// tile is empty and is flat 
		if ( t_tile.is_ground() ) {
			if ( print_message_box == 2 ) { 
				gui.add_message_at(pl, " ---=> tile is empty and is flat ", world.get_time()) 
			}
			return true 
		}
	} else if ( t_tile.has_way(wt) && !t_tile.has_two_ways() && t_tile.get_way_dirs(wt) == rotate && t_tile.get_slope() == 0 ) { 
		// tile has single way and is flat
		if ( print_message_box == 2 ) { 
			gui.add_message_at(pl, " ---=> tile has single way and is flat ", world.get_time())
		}	
		return true 
	} else if ( t_tile.is_empty() && t_tile.get_slope() > 0 ) {
		// terraform 
		if ( print_message_box == 2 ) { 
			gui.add_message_at(pl, " ---=> terraform", world.get_time())
			gui.add_message_at(pl, " ---=> tile z " + z + " start tile z " + ref_hight, world.get_time())
		}	
		
    // find z coord
		z = square_x(t_tile.x, t_tile.y).get_ground_tile(t_tile.x, t_tile.y)
		local s = 0;
		if ( z.z < ref_hight && z.z >= (ref_hight - 2) ) { 
			// terraform up   
			if ( print_message_box == 2 ) { 
				gui.add_message_at(pl, " ---=> tile up to flat ", world.get_time())
			}	
			do {
				err = command_x.set_slope(pl, tile_x(t_tile.x, t_tile.y, z.z), 82 )
				z = square_x(t_tile.x, t_tile.y).get_ground_tile(t_tile.x, t_tile.y)
				if ( err ) { break }
			} while(z.z < ref_hight ) 

		} else if ( z.z == ref_hight || z.z <= (ref_hight + 1) ) {
			// terraform down   
			do {
				err = command_x.set_slope(pl, tile_x(t_tile.x, t_tile.y, z.z), 83 )
				z = square_x(t_tile.x, t_tile.y).get_ground_tile(t_tile.x, t_tile.y)
				if ( err ) { break }
			} while(z.z > ref_hight ) 
		}
		if ( err ) {
			return false 
		}			
    return true
	} 

	return false
}


/**
 * expand station
 * pl = player
 * fields = array fields
 * wt = waytype
 */
function expand_station(pl, fields, wt) {

	local err = null
	local t = fields.len()
	

	// build way to tiles 
	if ( t > 0 ) { 
		for ( local x = 1; x < t; x++ ) {
			local f = tile_x(fields[x].x, fields[x].y, fields[x].z)
			if ( f.is_empty() ) {
				err = command_x.build_way(pl, fields[0], f, planned_way, true) 
			}
  		if ( err ) {
				return false
			} 
		}
   	if ( err == null ) {
			// build station to tile
			for ( local x = 0; x < t; x++ ) {
				err = command_x.build_station(pl, fields[x], planned_station) 
				if ( err ) { 
					gui.add_message_at(pl, " ---=> not build station tile at " + coord3d_to_string(fields[x]), world.get_time()) 
				} else {
					gui.add_message_at(pl, " ---=> build station tile at " + coord3d_to_string(fields[x]), world.get_time())
				}
			}
		}				
  	if ( err ) {
			return false
		} 
		
		return true
	}
}

/**
 * search existing depot on range to station
 *
 */
function search_depot(field_pos, wt) {
	
	local list_exists_depot = depot_x.get_depot_list(our_player, wt) 
	// search range
	local seach_field = 10

	local tile_min = [field_pos.x - seach_field, field_pos.y - seach_field]
	local tile_max = [field_pos.x + seach_field, field_pos.y + seach_field]
	local depot_found = false

	foreach(key in list_exists_depot) {

		if ( key.x >= tile_min[0] && key.y >= tile_min[1] && key.x <= tile_max[0] && key.y <= tile_max[1] ) {
			depot_found = tile_x(key.x, key.y, key.z)
			break
		} 
				
	}

	return depot_found

}