class factorysearcher_t extends manager_t
{
	froot = null    // factory_x, complete this tree

	constructor()
	{
		base.constructor("factorysearcher_t")
		debug = false
		::factorysearcher = this
	}

	function work()
	{
		// root still has missing links?
		if (froot) {
			if (count_missing_factories(froot) <= 0) {
				froot = null
			}
		}
		// determine new root
		if (froot == null) {
			// find factory with incomplete connected tree
			local min_mfc = 10000;
			local list = factory_list_x()
			foreach(fab in list) {
				if (fab.output.len() == 0) {

					local n = count_missing_factories(fab)

					if ((n > 0)  &&  (n < min_mfc)) {
						// TODO add some random here
						min_mfc = n
						froot = fab
					}
				}
			}
			if (froot) {
				local fab = froot
				dbgprint("Choose consumer " + fab.get_name() + " at " + fab.x + "," + fab.y + ", which has " + min_mfc + " missing links")
			}
		}

		// nothing found??
		if (froot==null) return r_t(RT_DONE_NOTHING);

		dbgprint("Connect  " + froot.get_name() + " at " + froot.x + "," + froot.y)

		// find link to connect
		if (!find_missing_link(froot)) {
			dbgprint(".. no missing link")
			// no missing link found - reset froot
			froot = null
			return r_t(RT_SUCCESS);
		}
		return r_t(RT_PARTIAL_SUCCESS);
	}

	/**
	 * Creates the planner node
	 */
	static function plan_connection(fsrc, fdest, freight)
	{
		dbgprint("Close link for " + freight + " from " + fsrc.get_name() + " at " + fsrc.x + "," + fsrc.y + " to "+ fdest.get_name() + " at " + fdest.x + "," + fdest.y)

		industry_manager.set_link_state(fsrc, fdest, freight, industry_link_t.st_planned);

		local icp = industry_connection_planner_t(fsrc, fdest, freight);
		append_child(icp)
	}

	/**
	 * @returns -1 if factory tree is incomplete, otherwise number of missing connections
	 */
	// TODO cache the results per factory
	static function count_missing_factories(fab, indent = "")
	{
		// source of raw material?
		if (fab.input.len() == 0) return 0;

		local end_consumer = fab.output.len() == 0

		// build list of supplying factories
		local suppliers = [];
		foreach(c in fab.get_suppliers()) {
			suppliers.append( factory_x(c.x, c.y) );
		}

		local count = 0;
		local g_atleastone = false;
		// iterate all input goods and search for supply
		foreach(good, islot in fab.input) {

			// test for in-storage or in-transit goods
			local st = islot.get_storage()
			local it = islot.get_in_transit()
			if (st[0] + st[1] + it[0] + it[1] > 0) {
				// something stored/in-transit in last and current month
				// no need to search for more supply
				g_atleastone = true
				continue
			}

			// there is a complete tree to produce this good
			local g_complete = false;
			// minimum of missing links for one input good
			local g_count    = 10000;
			foreach(s in suppliers) {

				if (good in s.output) {
					// check state of connection
					local state = industry_manager.get_link_state(s, fab, good);

					if (state == industry_link_t.st_failed) {
						// treat is as incomplete tree (only if not end consumer)
						if (!end_consumer) return -1; else continue // foreach
					}
					if (state != industry_link_t.st_free) {
						// planned / built -> nothing missing
						g_complete = true
						g_count = 0
						continue

					}

					local n = count_missing_factories(s, indent + "  ");
					if ( n<0) {
						// incomplete tree
					}
					else {
						// complete tree
						g_complete = true;
						g_count = min(g_count, n+1)
					}
				}
			}

			if (!g_complete  &&  !end_consumer) {
				dbgprint(indent + "No supply of " + good + " for " + fab.get_name())
				// no suppliers for this good
				return -1
			}
			g_atleastone = g_atleastone || g_complete

			if (!end_consumer) {
				count += g_count // sum missing links
			}
			else {
				if (g_count > 0  &&  (count == 0  ||  g_count < count)) {
					count = g_count;
				}
// 				count = min(count, g_count) // only take the minimum of the subtrees
			}
			dbgprint(indent + "Supply of " + good + " for " + fab.get_name() + " has " + g_count + " missing links")
		}

		if (end_consumer  &&  !g_atleastone) {
			dbgprint(indent + "No supply for " + fab.get_name())
			count = -1
		}

		dbgprint(indent + "Factory " + fab.get_name() + " at " + fab.x + "," + fab.y + " has " + count + " missing links")
		return count
	}

	/**
	 * find link to connect in tree of factory @p fab.
	 * sets fsrc, fdest, lgood if true was returned
	 * @returns true if link is found
	 */
	function find_missing_link(fab, indent = "")
	{
		dbgprint(indent + "Missing link for factory " + fab.get_name() + " at " + fab.x + "," + fab.y)
		// source of raw material?
		if (fab.input.len() == 0) return false;

		// build list of supplying factories
		local suppliers = [];
		foreach(c in fab.get_suppliers()) {
			suppliers.append( factory_x(c.x, c.y) );
		}

		local count = 0;
		// iterate all input goods and search for supply
		foreach(good, islot in fab.input) {
			// check for current supply
			if ( 4*(islot.get_storage()[0] + islot.get_in_transit()[0]) > islot.max_storage) {
				dbgprint(indent + ".. enough supply of " + good)
				continue
			}
			// find suitable supplier
			foreach(s in suppliers) {

				if ( !(good in s.output)) continue;

				// connection forbidden? planned? built?
				local state = industry_manager.get_link_state(s, fab, good)
				if (state != industry_link_t.st_free) {
					if (state == industry_link_t.st_built  ||  state == industry_link_t.st_planned) {
						dbgprint(indent + ".. connection for " + good + " from " + s.get_name() + " to " + fab.get_name() + " already "
							   + (state == industry_link_t.st_built ? "built" : "planned") )
						break
					}
					continue // if connection state is 'failed'
				}

				local oslot = s.output.rawget(good)

				dbgprint(indent + ".. Factory " + s.get_name() + " at " + s.x + "," + s.y + " supplies " + good)

				if (8*oslot.get_storage()[0] > oslot.max_storage  ||  !find_missing_link(s, indent + "  ")) {
					// this is our link
					dbgprint(indent + ".. plan this connection")
					plan_connection(s, fab, good)
				}
				return true
			}
		}
		return false // all links are connected
	}
}
