
function abs(a) { return a >= 0 ? a : -a }

openwater <- {
	function get_cost() { return 0; }
	function get_maintenance()  { return 0; }
	function get_name() { return "open water"}
}

function get_max_convoi_length(wt)
{
	switch(wt) {
		case wt_rail:   return settings.get_max_rail_convoi_length();
		case wt_road:   return settings.get_max_road_convoi_length();
		case wt_water:  return settings.get_max_ship_convoi_length();
	}
	return 4;
}

class industry_connection_planner_t extends manager_t
{
	fsrc = null       // factory_x
	fdest = null      // factory_x
	freight = null    // string
	prod = -1   	// integer

	// print messages box 
	// 1 = vehicles
	// 2 = stations 
	// 3 = depots  
	// 4 = reports 
	// 5 = factorys
	print_message_box = 4
	wt_name = ["", "road", "rail", "water"]
	
	constructor(s,d,f)
	{
		base.constructor("industry_connection_planner_t");
		fsrc = s; fdest = d; freight = f;
	}

	function step()
	{
		debug = true
		local tic = get_ops_total();

		print("Plan link for " + freight + " from " + fsrc.get_name() + " at " + fsrc.x + "," + fsrc.y + " to "+ fdest.get_name() + " at " + fdest.x + "," + fdest.y)

		// TODO check if factories are still existing
		// TODO check if connection is plannable

		// compute monthly production
		if (prod < 0) {
			prod = calc_production()
		}
		dbgprint("production = " + prod);

		// rail
		local rprt = plan_simple_connection(wt_rail, null, null)
		if (rprt) {
			append_report(rprt)
		}
		// road
		rprt = plan_simple_connection(wt_road, null, null)
		if (rprt) {
			append_report(rprt)
		}
		// water
		rprt = plan_simple_connection(wt_water, null, null)
		if (rprt) {
			append_report(rprt)
		}

		if (reports.len() == 0) {
			dbgprint("Set link for " + freight + " from " + fsrc.get_name() + " at " + fsrc.x + "," + fsrc.y + " to "+ fdest.get_name() + " at " + fdest.x + "," + fdest.y + " to MISSING")

			industry_manager.set_link_state(fsrc, fdest, freight, industry_link_t.st_missing)
			return r_t(RT_TOTAL_FAIL)
		}

		// deliver it
		local r = r_t(RT_READY)
		r.report = get_report()

		// append a chain of alternative connector nodes
		local rchain = r.report
		while (reports.len()>0) {
			local rep = get_report()
			if (rep == null) {
				// may happen if no nice reports are available
				break;
			}
			rchain.action.reports.append( rep )
			rchain = rep
		}

		local r_amph = report_t()
		r_amph.action = amphibious_connection_planner_t(fsrc, fdest, freight)

		if (rchain) {
			rchain.action.reports.append( r_amph )
		}
		else {
			r.report = r_amph
		}

		local toc = get_ops_total();
		print("industry_connection_planner wasted " + (toc-tic) + " ops")
		return r
	}

	// if start or target are null then use fsrc/fdest
	function plan_simple_connection(wt, start, target, distance = 0)
	{
		// compute correct distance
		if (distance == 0) {
			foreach(i in ["x", "y"]) {
				distance += abs( (start ? start[i] : fsrc[i]) - (target ? target[i] : fdest[i]))
			}
		}
		if (distance == 0) {
			// still zero? avoid division by zero in the prototyper 
			// distance factorys
			distance = abs(fsrc.x - fdest.x) + abs(fsrc.y - fdest.y) 
			// add 10% from distance
			distance += distance / 100 * 10
		}
		// plan convoy prototype
		local prototyper = prototyper_t(wt, freight)

		prototyper.min_speed = 1

		prototyper.max_vehicles = get_max_convoi_length(wt)
		prototyper.max_length = prototyper.max_vehicles * 8    

		if (wt == wt_rail) {
			//prototyper.max_length = 8
		}

		if (wt == wt_water) {
			//prototyper.max_length = 4
		}

		local cnv_valuator = valuator_simple_t()
		cnv_valuator.wt = wt
		cnv_valuator.freight = freight
		cnv_valuator.volume = prod
		cnv_valuator.max_cnvs = 200   
		// no signals and double tracks - limit 1 convoy for rail
		if (wt == wt_rail) {
			cnv_valuator.max_cnvs = 1
		}
		cnv_valuator.distance = distance

		if ( print_message_box > 0 ) { 
			gui.add_message_at(our_player, "___________________________ Start plan_simple_connection __________________________", world.get_time())
			//gui.add_message_at(our_player, "plan way ", world.get_time())
			local t = tile_x(fsrc.x, fsrc.y, 0)
			gui.add_message_at(our_player, "Plan link for " + freight + " from " + fsrc.get_name() + " at " + fsrc.x + "," + fsrc.y + " to "+ fdest.get_name() + " at " + fdest.x + "," + fdest.y, t)
		}
		
		local bound_valuator = valuator_simple_t.valuate_monthly_transport.bindenv(cnv_valuator)
		prototyper.valuate = bound_valuator

		if (prototyper.step().has_failed()) {
			return null
		}
		local planned_convoy = prototyper.best
		print("best " + planned_convoy.min_top_speed + " / " + planned_convoy.max_speed)

		// fill in report when best way is found
		local r = report_t()
		// plan way
		local planned_way = null
		if (wt == wt_water) {
			planned_way = openwater
		}
		else {
			local way_list = way_desc_x.get_available_ways(wt, st_flat)
			local best_way = null
			local best = null

			foreach(way in way_list) {
				cnv_valuator.way_maintenance = way.get_maintenance()
				cnv_valuator.way_max_speed   = way.get_topspeed()

				local test = cnv_valuator.valuate_monthly_transport(planned_convoy)
				if (best == null  ||  test > best) {
					best = test 
					// max track speed 160
					if (cnv_valuator.way_max_speed < 161 && wt == wt_rail ) {
						best_way = way
					}  
					else {
						best_way = way
					}
				}
			}
			dbgprint("Best value = " + best + " way = " + best_way.get_name())

			// valuate again with best way
			cnv_valuator.way_maintenance = 0
			cnv_valuator.way_max_speed   = best_way.get_topspeed()

			planned_way = best_way
		}

		// valuate again with best way
		r.gain_per_m = cnv_valuator.valuate_monthly_transport(planned_convoy)

		if ( print_message_box == 1 ) { 
			gui.add_message_at(our_player, "*** ", world.get_time())
			gui.add_message_at(our_player, "plan station ", world.get_time())
		}
		// plan station
		local planned_station = null  
		local planned_harbour_flat = null
		if ( wt == wt_rail ) {
			//planned_convoy.length = 12
		}
		
		if ( print_message_box == 1 ) { 
			gui.add_message_at(our_player, "wt " + wt_name[wt], world.get_time())
			gui.add_message_at(our_player, "planned_convoy.length " + planned_convoy.length, world.get_time())
		}
		if (wt != wt_water) {
			local station_list = building_desc_x.get_available_stations(building_desc_x.station, wt, good_desc_x(freight)) 
			if ( wt == wt_rail ) {
				planned_station = select_station(station_list, 8, planned_convoy.capacity)
			}
			else {
				planned_station = select_station(station_list, planned_convoy.length, planned_convoy.capacity)
			}
		
		}
		else {
			// find harbour building
			local station_list = building_desc_x.get_available_stations(building_desc_x.harbour, wt, good_desc_x(freight))
			planned_station = select_station(station_list, 1, planned_convoy.capacity)
			// find flat harbour building
			station_list = building_desc_x.get_available_stations(building_desc_x.flat_harbour, wt_water, good_desc_x(freight))  
			planned_harbour_flat = select_station(station_list, 1, planned_convoy.capacity)
				
		}


		// plan depot
		local planned_depot = null
		{ 
			
			local depot_list = building_desc_x.get_available_stations(building_desc_x.depot, wt, {})

			if (depot_list.len()) {
				planned_depot = depot_list[0]
			}
		}

		if (planned_convoy == null  ||  planned_way == null || planned_station == null || planned_depot == null) {
			return null
		}
    
		// create action node
		local cn = null
		switch(wt) {
			case wt_rail:  cn = rail_connector_t(); break
			case wt_road:  cn = road_connector_t(); break
			case wt_water: cn = ship_connector_t(); break
		}
		cn.fsrc = fsrc
		cn.fdest = fdest
		cn.freight = freight
		cn.planned_way = planned_way
		cn.planned_station = planned_station
		cn.planned_depot = planned_depot
		cn.planned_convoy = planned_convoy

		if (start) {
			cn.c_start = [start]
			print("Connector from " + coord_to_string(start))
		}
		if (target) {
			cn.c_end = [target]
			print("Connector to " + coord_to_string(target))
		}
		
		r.distance = cnv_valuator.distance
		
		// stations lenght
		local a = planned_convoy.length
		local count = 0
		do {
    	a -= 16
			count += 1
		} while(a > 0)					
       
		// build cost for way, stations and depot
		local build_cost = r.distance * planned_way.get_cost() + ((count*2)*planned_station.get_cost()) + planned_depot.get_cost()
		// build cost / 12 months
		build_cost = build_cost / 13
		
		local conv_capacity = planned_convoy.capacity
		local freight_input = fdest.input[freight].get_base_consumption()
		local input_convoy = freight_input/conv_capacity
		local freight_output = fsrc.output[freight].get_base_production()
		local output_convoy = freight_output/conv_capacity
		
		r.points = 100
		if ( output_convoy > 250 ) {
			r.points = 0
		} 
		if ( ( freight_input < 700 ) && ( wt == wt_rail && r.distance < 150 ) ) {
			r.points = 0
		}
		
		// successfull - complete report
		r.cost_fix     = build_cost
		r.cost_monthly = (r.distance * planned_way.get_maintenance()) + ((count*2)*planned_station.get_maintenance()) + planned_depot.get_maintenance()
		r.gain_per_m  -= r.cost_monthly

		// successfull - complete report
		r.action = cn  
		
		dbgprint("Plan: way = " + planned_way.get_name() + ", station = " + planned_station.get_name() + ", depot = " + planned_depot.get_name());
		dbgprint("Report: gain_per_m  = " + r.gain_per_m + ", nr_convoys  = " + planned_convoy.nr_convoys + ", cost_fix  = " + r.cost_fix + ", cost_monthly  = " + r.cost_monthly)
		dbgprint("Report: dist = " + cnv_valuator.distance + " way_cost = " + planned_way.get_cost())
		dbgprint("Report: station = " + planned_station.get_cost()+ " depot = " + planned_depot.get_cost())
		if ( print_message_box == 4 ) { 
			gui.add_message_at(our_player, "----- ", world.get_time())
			gui.add_message_at(our_player, "Plan: way = " + planned_way.get_name() + ", station = " + planned_station.get_name() + ", depot = " + planned_depot.get_name(), world.get_time())
			gui.add_message_at(our_player, "Report: gain_per_m  = " + r.gain_per_m + ", nr_convoys = " + planned_convoy.nr_convoys + ", cost_build = " + r.cost_fix + ", cost_monthly = " + r.cost_monthly, world.get_time())
			gui.add_message_at(our_player, "Report: dist = " + r.distance + " way_cost = " + planned_way.get_cost(), world.get_time())
			gui.add_message_at(our_player, "Report: station = " + planned_station.get_cost() + " depot = " + planned_depot.get_cost(), world.get_time()) 


			gui.add_message_at(our_player, " * Report: freight = " + freight + " convoy capacity = " + conv_capacity, world.get_time()) 
			gui.add_message_at(our_player, " * Report: input amount = " + freight_input + " input amount / convoy capacity = " + input_convoy, world.get_time()) 
			gui.add_message_at(our_player, " * Report: output = " + freight_output + " output / convoy capacity = " + output_convoy, world.get_time()) 
		}
		if ( print_message_box > 0 ) { 
			gui.add_message_at(our_player, "___________________________ End  plan_simple_connection __________________________", world.get_time())
		}
		return r
	}


	function calc_production()
	{
		local src_prod = fsrc.output[freight].get_base_production();
		local dest_con = fdest.input[freight].get_base_consumption();

		// TODO implement production factors
		local src_prod_faktor = fsrc.output[freight].get_production_factor();
		local dest_con_faktor = fdest.input[freight].get_consumption_factor();
		 
		if ( print_message_box == 5 ) { 
			gui.add_message_at(our_player, "----- factory info -----", world.get_time())
			gui.add_message_at(our_player, " fsrc : " + fsrc.get_name(), world.get_time())
			gui.add_message_at(our_player, "freight = " + freight + ", output = " + dest_con + ", factor = " + dest_con_faktor, world.get_time())

			gui.add_message_at(our_player, "-----", world.get_time())

			gui.add_message_at(our_player, " fdest : " + fdest.get_name(), world.get_time())  
			gui.add_message_at(our_player, "freight = " + freight + ", input = " + src_prod + ", factor = " + src_prod_faktor, world.get_time())

			gui.add_message_at(our_player, "--- factory info end ---", world.get_time())
		}
		


		dbgprint("production = " + src_prod + " / " + dest_con);
		return min(src_prod,dest_con)
	}

	static function select_station(list, length, capacity)
	{
		local station_length = (length + CARUNITS_PER_TILE - 1) / CARUNITS_PER_TILE
		local capacity       =  capacity

		local station_capacity = 0
		local station_is_terminus = false
		local best_station = null

		if ( print_message_box == 2 ) { 
			gui.add_message_at(our_player, "---- station list ----", world.get_time())
		}

		foreach(station in list) {
			local ok = (best_station == null)
			local s_capacity = station_length * station.get_capacity()

		if ( print_message_box == 2 ) { 
			gui.add_message_at(our_player, "station " + station.get_name() + ", type = " + station.get_type(), world.get_time())
		}

			if (!ok  &&  station_length == 1) {
				// prefer terminus
				ok = station.is_terminus()  &&  !station_is_terminus
				if (!ok) {
					// then prefer stations with enough capacity
					ok = station_capacity < capacity ? station_capacity < s_capacity
					 : capacity < s_capacity  &&  s_capacity < station_capacity
				}
			}
			if (station_length >  1) {
				// force non-terminus
				ok = !station_is_terminus
			}
			if (ok) {
				best_station = station
				station_capacity = station.get_capacity()
				station_is_terminus = station.is_terminus()
			}
		}
		if ( print_message_box == 2 ) { 
			gui.add_message_at(our_player, "----> selectet station: " + best_station.get_name(), world.get_time())
		}
		return best_station
	}
}
