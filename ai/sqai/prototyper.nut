class cnv_proto_t
{
	weight = 0
	power = 0
	min_top_speed = 0
	max_speed = 1000000000 //1000000000000
	length = 0
	capacity = 0
	maintenance = 0
	running_cost = 0
	price = 0
	missing_freight = true
	veh = null

	// set by valuator
	nr_convoys = 0
  
	constructor()
	{
		veh = []
	}

	function append(newveh, freight)
	{
		local cnv = getclass().instance()
		cnv.constructor()

		cnv.veh.extend(veh)
		cnv.veh.append(newveh)
		cnv.weight = weight + freight.get_weight_per_unit() * newveh.get_capacity() + newveh.get_weight()

		cnv.power = power + newveh.get_power()
		cnv.max_speed = min(max_speed, newveh.get_topspeed())
		cnv.length = length + newveh.get_length()

		local fits = newveh.get_freight().is_interchangeable(freight)
		cnv.missing_freight = missing_freight &&  (newveh.get_capacity()==0 ||  !fits)

		cnv.min_top_speed = convoy_x.calc_max_speed(cnv.power, cnv.weight, cnv.max_speed)
//    print("XXX Power " +cnv.power + " weight = " +cnv.weight + " amx = " + cnv.max_speed+ " speed = " + cnv.min_top_speed)
		cnv.capacity = capacity + (fits ? newveh.get_capacity() : 0)
		cnv.maintenance = maintenance + newveh.get_maintenance()
		cnv.running_cost = running_cost + newveh.get_running_cost()
		cnv.price = price + newveh.get_cost()

		return cnv
	}

	static function from_convoy(cnv, freight)
	{
		local p = cnv_proto_t()
		local list = cnv.get_vehicles()
		foreach(v in list) {
			p = p.append(v, freight)
		}
		return p
	}

	function _save()
	{
		return ::saveinstance("cnv_proto_t", this)
	}
}

class prototyper_t extends node_t
{
	wt = 0
	freight = 0
	max_vehicles = 0
	max_length   = 0
	min_speed   = 0

	valuate = null

	best = null
	best_value = 0

	// print messages box 
	print_message_box = 0
	wt_name = ["", "road", "rail", "water"]
	
	constructor(w, /*string*/f)
	{
		base.constructor("prototyper");
		wt = w
		freight = good_desc_x(f)
	}

	function step()
	{ 
		if ( print_message_box == 1 ) {
			local units = get_max_convoi_length(wt)
			gui.add_message_at(our_player, "**** ", world.get_time())
			gui.add_message_at(our_player, "create convoy ", world.get_time())
			gui.add_message_at(our_player, "wt " + wt_name[wt], world.get_time())
			gui.add_message_at(our_player, "units: " + units, world.get_time())
			gui.add_message_at(our_player, "CARUNITS_PER_TILE: " + CARUNITS_PER_TILE, world.get_time())
			gui.add_message_at(our_player, "max_length: " + max_length, world.get_time())
    }
		
		local list = vehicle_desc_x.get_available_vehicles(wt)

		local list_first = []
		local list_other = []

		foreach(veh in list) {

			local first = veh.can_be_first()
			local fits  = veh.get_freight().is_interchangeable(freight)
			local pwer  = veh.get_power()>0
			local none  = veh.get_freight().get_name()=="None" || veh.get_capacity()==0
			local timeline = !veh.is_retired(world.get_time()) //true //veh.get_retire_date() > world.get_time()
			local electrified = !veh.needs_electrification()

			//gui.add_message_at(our_player, "timeline: " + veh.get_name() + " - " + timeline, world.get_time())
			// use vehicles that can carry freight
			// or that are powered and have no freight capacity
			if ( (fits || (pwer && none)) && timeline && electrified) {
				if (first)
					list_first.append(veh)
				//gui.add_message_at(our_player, "vehicle found: " + veh.get_name(), world.get_time())

				list_other.append(veh)
				
				if ( print_message_box == 1 ) {
					gui.add_message_at(our_player, "* vehicle found: " + veh.get_name(), world.get_time())
				}
			}

		}

//    foreach(veh in list_first) print("candidate...leading " + veh.get_name())
//    foreach(veh in list_other) print("candidate...        " + veh.get_name())

		// array of lists we try to iterate
		local it_lists = []; it_lists.resize(max_vehicles+1)


		local it_ind = [];     it_ind.resize(max_vehicles+1)

		// current convoy candidate - array of desc
		local cnv = [];          cnv.resize(max_vehicles+1)

		// initialize
		cnv[0] =  cnv_proto_t()
		it_ind[1] = -1
		it_lists[1] = list_first

		// iterating ind-th position in convoy
		local ind = 1


		while(true) {

			it_ind[ind] ++
			// done with iterating?
			if (it_ind[ind] >= it_lists[ind].len() ) {
				if (ind>1) {
					ind--
					continue // iterating position ind-1
				}
				else {
					break // end of the iteration
				}
			}

			local test = it_lists[ind][ it_ind[ind] ]

			// check couplings
			if ( ind==1 ? !test.can_be_first() : !vehicle_desc_x.is_coupling_allowed(cnv[ind-1].veh.top(), test) ) {
				continue;
			}
//      print("Test[" + ind + "] = " + test.get_name())
			// append
			cnv[ind] = cnv[ind-1].append(test, freight)
			local c = cnv[ind]

//      local ccc = ["weight","power","min_top_speed","max_speed","length","missing_freight","capacity","maintenance","price","running_cost"]
//      foreach(key in ccc) print(" ... " + key + " = " + c[key] )

			// check constraints
			// .. length
			local l = (ind > 1 ?  cnv[ind-1].length : 0) + max( CARUNITS_PER_TILE/2, test.get_length());
			//gui.add_message_at(our_player, "convoy length max: " + max( CARUNITS_PER_TILE/2, test.get_length()), test)
			//max_vehicles 
			local a = 0
			if ( wt == wt_water ) {
				a = CARUNITS_PER_TILE * 4
			}
			else if ( wt == wt_rail ) {
				a = CARUNITS_PER_TILE * 3
			}
			else {
				a = CARUNITS_PER_TILE
			} 
			
			if (l > a || c["min_top_speed"] < c["max_speed"] ) { //) { max_length   CARUNITS_PER_TILE
				continue;
			}
			// .. more ??
			//gui.add_message_at(our_player, "convoy length: " + l, world.get_time())
			//local ccc = ["power","min_top_speed","max_speed","capacity","length"]
			//foreach(key in ccc) gui.add_message_at(our_player," ... " + key + " = " + c[key], world.get_time())

			// check if convoy finished
			if (test.can_be_last() && !c.missing_freight  &&  c.min_top_speed >= min_speed) {
				// evaluate this candidate
					//gui.add_message_at(our_player, "valuate: " + valuate, world.get_time())
				if (valuate) {
					local value = valuate.call(getroottable(), c)
//          print(" === " + value)
					//gui.add_message_at(our_player, "evaluate this candidate: " + value, world.get_time())
					if (best==null  ||  value > best_value) {
						best = c
						best_value = value
					}
				}
				else {
					// no valuator function -> take first valid convoy and return
					best = c;
					break
				}

//        print("..... ***")
			}

			// move on to next position
			if (ind >= max_vehicles) {
				continue;
			}

			ind++

			local list_succ = test.get_successors()
			it_lists[ind] = list_succ.len()==0 ? list_other : list_succ
			it_ind[ind] = -1
		}

		if (best) {
			foreach(ind, test in best.veh) {
				print("Best[" + ind + "] = " + test.get_name())
			}

//      local ccc = ["weight","power","min_top_speed","max_speed","length","missing_freight","capacity","maintenance","price","running_cost"]
//      foreach(key in ccc) print(" ... " + key + " = " + best[key] )
//
			return r_t(RT_SUCCESS)
		}
		return r_t(RT_ERROR)
	}

}

class valuator_simple_t {
	wt = 0
	freight = null
	volume = 0 // monthly transport volume
	max_cnvs = 0
	distance = 0

	way_max_speed = -1
	way_maintenance = 0

	function valuate_monthly_transport(cnv) {

		local speed = way_max_speed > 0 ? min(way_max_speed, cnv.min_top_speed) : cnv.min_top_speed

		local frev = good_desc_x(freight).calc_revenue(wt, speed)

		local capacity = cnv.capacity
		// tiles per month of one convoy
		local tpm = convoy_x.speed_to_tiles_per_month(speed) / 2 + 1

		// needed convoys to transport everything
		local n1 = max(1, volume * 2 * distance / (tpm * cnv.capacity))

		// realistic number of convoys
		local ncnv = min(n1, min(max_cnvs, max(distance / 8, 3) ) )
		cnv.nr_convoys = ncnv

		// double track and signals missing
		if (wt == wt_rail) {
			cnv.nr_convoys = 1
		}

		if (way_max_speed > 0) {
			// correction factor to prefer faster ways:
			// factor = 0 .. if 2*distance < ncnv
			// factor = 1 .. if distance/3 > ncnv
			// linear interpolated in between
			// without scaling almost always the cheapest way is chosen ...
			local factor = max(0, min(10*distance, 6*(2*distance - ncnv) ) );
			// rescale tiles per month
			tpm = (tpm*factor) / (10*distance);
		}

		// monthly costs and revenue
		local value = ncnv*( (frev*cnv.capacity+1500)/3000*tpm - cnv.running_cost*tpm - cnv.maintenance) - distance * way_maintenance

		return value
	}

	function _save()
	{
		return ::saveinstance("valuator_simple_t", this)
	}
}
