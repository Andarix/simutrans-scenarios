class rail_connector_t extends manager_t
{
	// input data
	fsrc = null
	fdest = null
	freight = null
	planned_way = null
	planned_station = null
	planned_depot = null
	planned_convoy = null
	finalize = true

	// step-by-step construct the connection
	phase = 0
	// can be provided optionally
	c_start = null // array
	c_end   = null // array
	// generated data
	c_depot = null
	c_sched = null
	c_line  = null
	c_cnv   = null

	// print messages box 
	// 1 = way
	// 2 = stations 
	// 3 = depot
	print_message_box = 2

	constructor()
	{
		base.constructor("rail_connector_t")
		debug = true
	}

	function work()
	{
		// TODO check if child does the right thing
		local pl = our_player
		local tic = get_ops_total();

		switch(phase) {
			case 0: // find places for stations
				if ( print_message_box > 0 ) { gui.add_message_at(our_player, "______________________ build rail ______________________", world.get_time()) }
				if (c_start == null) {
					c_start = ::finder.find_station_place(fsrc, fdest)
				}
				if (c_start  &&  c_end == null) {
					c_end   = ::finder.find_station_place(fdest, c_start, finalize)
				}

				if (c_start.len()>0  &&  c_end.len()>0) {
					phase ++
				}
				else {
					print("No station places found")
					return error_handler()
				}
			case 1: // build way
				{
					sleep()
					local d = pl.get_current_cash();
					local err = construct_rail(pl, c_start, c_end, planned_way )
					print("Way construction cost: " + (d-pl.get_current_cash()) )
					if ( print_message_box == 1 && c_start.len()>0  &&  c_end.len()>0) { 
						gui.add_message_at(our_player, "Build rail from " + coord_to_string(c_start[0])+ " to " + coord_to_string(c_end[0]), world.get_time()) 
					}
					if (err && c_start.len()>0  &&  c_end.len()>0) {
						print("Failed to build way from " + coord_to_string(c_start[0])+ " to " + coord_to_string(c_end[0]))
						return error_handler()
					} else if (err) {
						print("Failed to build way from " + coord_to_string(c_start)+ " to " + coord_to_string(c_end))
						return error_handler()
					}
					phase ++
				}
			case 2: // build station
				{
					local err = command_x.build_station(pl, c_start, planned_station )
					if (err) {
						print("Failed to build station at " + coord_to_string(c_start))
						if ( print_message_box == 2 ) { gui.add_message_at(pl, "Failed to build rail station at  " + coord_to_string(c_start) + " error " + err, world.get_time()) }
						return error_handler()
					}
					
					if ( print_message_box == 2 ) { 
						gui.add_message_at(our_player, " planned_convoy.length " + planned_convoy.length, world.get_time()) 
					} 
					
					// stations lenght
					local a = planned_convoy.length
					local count = 0
					do {
    				a -= 16
						count += 1
					} while(a > 0)					
					
					if ( print_message_box == 2 ) { 
						gui.add_message_at(our_player, " stations lenght: " + count, world.get_time()) 
					} 
					
					local err = command_x.build_station(pl, c_end, planned_station )
					if (err) {
						if ( print_message_box == 2 ) {
							gui.add_message_at(pl, "Failed to build rail station at  " + coord_to_string(c_end) + " error " + err, world.get_time())
						}
						print("Failed to build station at " + coord_to_string(c_end))
						return error_handler()
					}
					if (finalize) {
						// store place of unload station for future use
						local fs = ::station_manager.access_freight_station(fdest)
						if (fs.rail_unload == null) {
							fs.rail_unload = c_end

							print( recursive_save({unload = c_end}, "\t\t\t", []) )
						}
					}
					
					err = check_station(our_player, c_start, count)
					err = check_station(our_player, c_end, count)
					
					// Felder prüfen auf Schiene oder frei
					
					
					// wenn ja dann Station anbauen
					
					// wenn nein Schienen bauen und Station anbauen
					
					if ( print_message_box == 2 ) { 
						//gui.add_message_at(our_player, " ... rotate " + rotate, world.get_time()) 
						gui.add_message_at(our_player, "Build station on " + coord_to_string(c_start) + " and " + coord_to_string(c_end), world.get_time()) 
					} 
					
					phase ++
				}
			case 3: // find depot place
				{
					local err = construct_rail_to_depot(pl, c_start, planned_way)
					if (err) {
						print("Failed to build depot access from " + coord_to_string(c_start))
						return error_handler()
					}
					phase += 2
				}
			case 5: // build depot
				{
					if ( print_message_box == 3 ) {
						gui.add_message_at(our_player, "___________ exists depots rail ___________", world.get_time())
			 			gui.add_message_at(our_player," c_start pos: " + coord_to_string(c_start) + " : c_end pos: " + coord_to_string(c_end), world.get_time())      
					}
					local list_exists_depot = depot_x.get_depot_list(our_player, wt_road) 
					local seach_field = 5 
					local tile_min = [c_start.x - seach_field, c_start.y - seach_field, c_end.x - seach_field, c_end.y - seach_field]
					local tile_max = [c_start.x + seach_field, c_start.y + seach_field, c_end.x + seach_field, c_end.y + seach_field]
					local depot_found = false

					foreach(key in list_exists_depot) {

						if ( print_message_box == 3 ) {
			 				gui.add_message_at(our_player," ... depot pos: " + key.get_pos(), world.get_time())      
						}

						if ( key.x >= tile_min[0] && key.y >= tile_min[1] && key.x <= tile_max[0] && key.y <= tile_max[1] ) {
							c_depot = depot_x(key.x, key.y, key.z)
							if ( print_message_box == 3 ) {
			 					gui.add_message_at(our_player," ---> depot found c_start: " + key.get_pos(), world.get_time())      
							}
							depot_found = true
							break
						} else if ( key.x >= tile_min[2] && key.y >= tile_min[3] && key.x <= tile_max[2] && key.y <= tile_max[3] ) {
							c_depot = depot_x(key.x, key.y, key.z)
							if ( print_message_box == 3 ) {
			 					gui.add_message_at(our_player," ---> depot found c_end: " + key.get_pos(), world.get_time())      
							}
							depot_found = true
							break
						} else {
						}
				
					}

					if ( depot_found && print_message_box == 3 ) {
			 			gui.add_message_at(our_player," *** depot not found *** ", world.get_time())      
					}

					// depot already existing ?
					if (c_depot.find_object(mo_depot_rail) == null) {
						// no: build
						local err = command_x.build_depot(pl, c_depot, planned_depot )
						if (err) {
							print("Failed to build depot at " + coord_to_string(c_depot))
							return error_handler()
						}
						if (finalize) {
							// store depot location
							local fs = ::station_manager.access_freight_station(fsrc)
							if (fs.rail_depot == null) {
								fs.rail_depot = c_depot
							}
						}
					}
					if ( print_message_box == 1 ) { gui.add_message_at(our_player, "Build depot on " + coord_to_string(c_depot), world.get_time()) }
					phase ++
				}
			case 6: // create schedule
				{
					local sched = schedule_x(wt_rail, [])
					sched.entries.append( schedule_entry_x(c_start, 100, 0) );
					sched.entries.append( schedule_entry_x(c_end, 0, 0) );
					c_sched = sched
					phase ++
				}

			case 7: // create line and set schedule
				{
					pl.create_line(wt_rail)

					// find the line - it is a line without schedule and convoys
					local list = pl.get_line_list()
					foreach(line in list) {
						if (line.get_waytype() == wt_rail  &&  line.get_schedule().entries.len()==0) {
							// right type, no schedule -> take this.
							c_line = line
							break
						}
					}
					// set schedule
					c_line.change_schedule(pl, c_sched);
					phase ++
				}
			case 8: // append vehicle_constructor
				{
					local c = vehicle_constructor_t()
					c.p_depot  = depot_x(c_depot.x, c_depot.y, c_depot.z)
					c.p_line   = c_line
					c.p_convoy = planned_convoy
					c.p_count  = min(planned_convoy.nr_convoys, 1)
					append_child(c)

					local toc = get_ops_total();
					print("rail_connector wasted " + (toc-tic) + " ops")

					phase ++
					return r_t(RT_PARTIAL_SUCCESS)
				}
			case 9: // build station extension

		}

		if (finalize) {
			industry_manager.set_link_state(fsrc, fdest, freight, industry_link_t.st_built)
		}
		industry_manager.access_link(fsrc, fdest, freight).append_line(c_line)

		return r_t(RT_TOTAL_SUCCESS)
	}

	function error_handler()
	{
		local r = r_t(RT_TOTAL_FAIL)
		// TODO rollback
		if (reports.len()>0) {
			// there are alternatives
			print("Delivering alternative connector")
			r.report = reports.pop()

			if (r.report.action  &&  r.report.action.getclass() == amphibious_connection_planner_t) {
				print("Delivering amphibious_connection_planner_t")
				r.node   = r.report.action
				r.report = null
			}
		}
		else {
			industry_manager.set_link_state(fsrc, fdest, freight, industry_link_t.st_failed);
		}
		return r
	}

	function construct_rail(pl, starts, ends, way)
	{
		local as = astar_builder()
		as.builder = way_planner_x(pl)
		as.way = way
		as.builder.set_build_types(way)
		as.bridger = pontifex(pl, way)
		if (as.bridger.bridge == null) {
			as.bridger = null
		}

		local res = as.search_route(starts, ends)

		if ("err" in res) {
			return res.err
		}
		c_start = res.start
		c_end   = res.end
	}

	function construct_rail_to_depot(pl, start, way)
	{
		local as = depot_pathfinder()
		as.builder = way_planner_x(pl)
		as.way = way
		as.builder.set_build_types(way)
		local res = as.search_route(start)

		if ("err" in res) {
			return res.err
		}
		local d = res.end
		c_depot = tile_x(d.x, d.y, d.z)
	}

	static function find_rail_depot_place(start, target)
	{
		// TODO UNUSED
		{
			// try depot location from station manager
			local res = ::station_manager.access_freight_station(fsrc).rail_depot
			if (res) {
				return res
			}
		}

		local cov = 5
		local area = []

		local t = tile_x(start.x, start.y, start.z)

		// follow the rail up to the next crossing
		// check whether depot can be built next to rail
		local d = t.get_way_dirs(wt_rail)
		if (!dir.is_single(d)) {
			d = d	& dir.southeast // only s or e
		}
		for(local i=0; i<4; i++) {
			local nt = t.get_neighbour(wt_rail, d)
			if (nt == null) break
				// should have a rail
				local rd = nt.get_way_dirs(wt_rail)
				// find direction to proceed: not going back, where we were coming from
				local nd = rd & (~(dir.backward(d)))
				// loop through neighbor tiles not on the rail
				foreach(d1 in dir.nsew) {
					if (d1 & rd ) continue
						// test this spot
						local dp = nt.get_neighbour(wt_all, d1)
						if (dp  &&  dp.is_empty()  &&  dp.get_slope()==0) {
							return dp
						}
				}
				if (!dir.is_single(nd)  ||  nd==0) break
					// proceed
					t = nt
					d = nd
		}

		t = tile_x(start.x, start.y, start.z)
		// now go into direction perpendicular
		// to the rail in the station
		local dirs = dir.double( t.get_way_dirs(wt_rail) ) ^ dir.all

		// try to find depot spot next to the station
		for(local i = 1;  i < 16; i=i<<1)
		{
			if (i & dirs) {
				local n = t.get_neighbour(wt_all, i)
				if (n  &&  n.is_empty()  &&  n.get_slope()==0) {
					return n
				}
			}
		}

		// generate a list of tiles near the station
		for(local dx = -cov; dx <= cov; dx++) {
			for(local dy = -cov; dy <= cov; dy++) {
				if (dx==0 && dy==0) continue;

				local x = start.x+dx
				local y = start.y+dy

				if (x>=0 && y>=0) area.append( (x << 16) + y );
			}
		}
		return find_empty_place(area, target)
	}
	
	function check_station(pl, starts_field, st_lenght) {

		if ( print_message_box == 2 ) {
			gui.add_message_at(our_player, " --- start field : " + coord3d_to_string(starts_field), world.get_time())
		}
					
		local a = false
		local a_1 = null
		local a_2 = null
		local b1_tile = null
		local b2_tile = null
		local f1 = 1
		local f2 = 1
		local tile_build = 0 
		local st_build = false 
		local err = null
		
					
	  // Ausrichtung ermitteln
	  // 0 - .x      eastwest
	  // 1 - .y      northsouth
		local t = tile_x(starts_field.x, starts_field.y, starts_field.z)
		local d = t.get_way_dirs(wt_rail)
		local d1 = null
		local d2 = null
		local loop = 0
		if ( d == 2 || d == 4 ) {
			d1 = dir.eastwest
			d2 = dir.northsouth
		}
		//if (dir.is_signle(d)) {
		//	d1 = t.get_neighbour(wt_rail, d) // only w - e
		//	d2 = d & dir.northsouth() // only s - n 
		//}
		if ( print_message_box == 2 ) { 
			gui.add_message_at(pl, " --- field test : " + coord3d_to_string(starts_field), world.get_time())
			gui.add_message_at(pl, " ------ get_way_dirs : " + d, world.get_time()) 
			gui.add_message_at(pl, " ------ test eastwest : " + d1, world.get_time()) 
			gui.add_message_at(pl, " ------ test northsouth : " + d2, world.get_time()) 
		}	
									

					
		//if ( d1 == 8 ) {			
					// check  w - e 
					// 1 - w
					// 2 - e
					do {
						b1_tile = tile_x(starts_field.x + f1, starts_field.y, starts_field.z)
						b2_tile = tile_x(starts_field.x - f2, starts_field.y, starts_field.z)
						
						// w tile empty or single rail
						if ( (b1_tile.is_empty() || b1_tile.has_way(wt_rail)) && !b1_tile.has_two_ways() ) { 
							// tile empty then build rail
					  	if ( b1_tile.is_empty() ) {
								// buld way to tile
								//err = construct_rail(pl, starts_field, b1_tile, planned_way)
							}
            	if ( err ) { 
						  	a_1 = false 
								err = null
							} else {
								// buld station to tile
								err = command_x.build_station(pl, b1_tile, planned_station ) 
            		if ( err ) { 
						  		a_1 = false 
									err = null
								} else {
									f1++
						  		tile_build++
								}
							}
							if ( print_message_box == 2 ) {
								gui.add_message_at(pl, " --- field test w empty : " + coord3d_to_string(b1_tile) + " -> " + b1_tile.is_empty(), world.get_time()) 
								gui.add_message_at(pl, " --- st build : " + coord3d_to_string(b1_tile) + " tile_build : " + a_1, world.get_time())
							}	
						}
					
												
						// e tile empty or single rail
						if ( !a_1 && (b2_tile.is_empty() || b2_tile.has_way(wt_rail)) && !b2_tile.has_two_ways() ) { 
							// tile empty then build rail
					  	if ( b2_tile.is_empty() ) {
								// buld way to tile
								//err = construct_rail(pl, starts_field, b2_tile, planned_way)
							}
            	if ( err ) { 
						  	a_2 = false 
								err = null
							} else {
								// buld station to tile
								err = command_x.build_station(pl, b2_tile, planned_station ) 
            		if ( err ) { 
						  		a_2 = false 
									err = null
								} else {
									f2++
						  		tile_build++
								}
							} 
							
							if ( print_message_box == 2 ) {
								gui.add_message_at(pl, " --- field test e empty : " + coord3d_to_string(b2_tile) + " -> " + b2_tile.is_empty(), world.get_time()) 
								gui.add_message_at(pl, " --- st build : " + coord3d_to_string(b2_tile) + " tile_build : " + a_2, world.get_time())
							}	
						}
					
						loop++
						if ( loop >= st_lenght ) {
							a = true
						}
						if ( print_message_box == 2 ) {
							gui.add_message_at(pl, " --- loop : " + loop, world.get_time())
						}	
							
						if ( tile_build == (st_lenght - 1) ) { 
							st_build = true
							a = true 
						} 
					} while(a == false)	
		//}					
					
		if ( !st_build ) { 
			f1 = 1
			f2 = 1
			a = false
			loop = 0
					// check  n - s 
					// 1 - n
					// 2 - s
					do {
						b1_tile = tile_x(starts_field.x, starts_field.y - f1, starts_field.z)
						b2_tile = tile_x(starts_field.x, starts_field.y + f2, starts_field.z)
						
						// n tile empty or single rail
						if ( (b1_tile.is_empty() || b1_tile.has_way(wt_rail)) && !b1_tile.has_two_ways() ) { 
							// tile empty then build rail
					  	if ( b1_tile.is_empty() ) {
								// buld way to tile
								//err = construct_rail(pl, starts_field, b1_tile, planned_way)
							}
            	if ( err ) { 
						  	a_1 = false 
								err = null
							} else {
								// buld station to tile
								err = command_x.build_station(pl, b1_tile, planned_station ) 
            		if ( err ) { 
						  		a_1 = false 
									err = null
								} else {
									f1++
						  		tile_build++
								}
							}
							if ( print_message_box == 2 ) {
								gui.add_message_at(pl, " --- field test n empty : " + coord3d_to_string(b1_tile) + " -> " + b1_tile.is_empty(), world.get_time()) 
								gui.add_message_at(pl, " --- st build : " + coord3d_to_string(b1_tile) + " tile_build : " + a_1, world.get_time())
							}	
						}
					
												
						// s tile empty or single rail
						if ( !a_1 && (b2_tile.is_empty() || b2_tile.has_way(wt_rail)) && !b2_tile.has_two_ways() ) { 
							// tile empty then build rail
					  	if ( b2_tile.is_empty() ) {
								// buld way to tile
								//err = construct_rail(pl, starts_field, b2_tile, planned_way)
							}
            	if ( err ) { 
						  	a_2 = false 
								err = null
							} else {
								// buld station to tile
								err = command_x.build_station(pl, b2_tile, planned_station ) 
            		if ( err ) { 
						  		a_2 = false 
									err = null
								} else {
									f2++
						  		tile_build++
								}
							} 
							
							if ( print_message_box == 2 ) {
								gui.add_message_at(pl, " --- field test s empty : " + coord3d_to_string(b2_tile) + " -> " + b2_tile.is_empty(), world.get_time()) 
								gui.add_message_at(pl, " --- st build : " + coord3d_to_string(b2_tile) + " tile_build : " + a_2, world.get_time())
							}	
						}
					
						loop++
						if ( loop >= st_lenght ) {
							a = true
						}
						if ( print_message_box == 2 ) {
							gui.add_message_at(pl, " --- loop : " + loop, world.get_time())
						}	

						if ( tile_build == (st_lenght - 1) ) { 
							st_build = true
							a = true 
						} 
					} while(a == false)	
		}			
					
	
	  return st_build
	}
}


class depot_pathfinder extends astar_builder
{
	function estimate_distance(c)
	{
		local t = tile_x(c.x, c.y, c.z)
		if (t.is_empty()  &&  t.get_slope()==0) {
			return 0
		}
		local depot = t.find_object(mo_depot_rail)
		if (depot  &&  depot.get_owner().nr == our_player_nr) {
			return 0
		}
		return 10
	}
	function add_to_open(c, weight)
	{
		if (c.dist == 0) {
			// test for depot
			local t = tile_x(c.x, c.y, c.z)
			if (t.is_empty()) {
				// depot not existing, we must build, increase weight
				weight += 25 * cost_straight
			}
		}
		base.add_to_open(c, weight)
	}

	function search_route(start)
	{
		prepare_search()

		local dist = estimate_distance(start)
		add_to_open(ab_node(start, null, 1, dist+1, dist, 0), dist+1)

		search()

		if (route.len() > 0) {

			for (local i = 1; i<route.len(); i++) {
				local err = command_x.build_way(our_player, route[i-1], route[i], way, false )
				if (err) {
					gui.add_message_at(our_player, "Failed to build rail from  " + coord_to_string(route[i-1]) + " to " + coord_to_string(route[i]) +"\n" + err, route[i])
					return { err =  err }
				}
			}
			return { start = route[ route.len()-1], end = route[0] }
		}
		print("No route found")
		return { err =  "No route" }
	}
}
