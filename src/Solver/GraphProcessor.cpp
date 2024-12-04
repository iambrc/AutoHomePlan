#include "Components/GraphProcessor.h"

std::vector<EdgeType> edgetypes = { LeftOf, RightOf, FrontOf, Behind, Above, Under, CloseBy, AlignWith };
std::vector<std::string> edgenames = { "LeftOf", "RightOf", "FrontOf", "Behind", "Above", "Under", "CloseBy", "AlignWith" };
std::vector<Orientation> orientations = { UP, DOWN, LEFT, RIGHT, FRONT, BACK };
std::vector<std::string> orientationnames = { "UP", "DOWN", "LEFT", "RIGHT", "FRONT", "BACK" };

Orientation opposite_orientation(Orientation o) {
    switch (o) {
    case UP: return DOWN;
    case DOWN: return UP;
    case LEFT: return RIGHT;
    case RIGHT: return LEFT;
    case FRONT: return BACK;
    case BACK: return FRONT;
    }
    return UP;
}

bool check_overlap(std::vector<double> r1, std::vector<double> r2)
{
    // r1 : x ,y, l, w
	if (std::fabs(r1[0] - r2[0]) > r1[2] / 2 + r2[2] / 2 || 
        std::fabs(r1[1] - r1[1]) > r1[3] / 2 + r2[3] / 2)
		return false;
	return true;
}

bool check_inside(std::vector<double> r, std::vector<double> R)
{
    // r : x, y, l, w
	if (r[0] - r[2] / 2 < R[0] - R[2] / 2 || r[0] + r[2] / 2 > R[0] + R[2] / 2 ||
		r[1] - r[3] / 2 < R[1] - R[3] / 2 || r[1] + r[3] / 2 > R[1] + R[3] / 2)
		return false;
	return true;
}

void remove_cycles(SceneGraph& g, EdgeType edge_type) {
    EdgeTypeFilter edge_filter(g, edge_type);
    boost::filtered_graph<SceneGraph, EdgeTypeFilter> filtered_g(g, edge_filter);

    // Find the strongly connected component
    std::vector<int> component(num_vertices(filtered_g));
    int num = boost::strong_components(filtered_g, &component[0]);

    // Find and delete one edge in the ring
    for (int i = 0; i < num; ++i) {
        std::vector<EdgeDescriptor> edges_to_remove;
        for (const auto &e : boost::make_iterator_range(edges(filtered_g))) {
            if (component[source(e, filtered_g)] == i && component[target(e, filtered_g)] == i) {
                edges_to_remove.push_back(e);
            }
        }
        if (!edges_to_remove.empty()) {
            boost::remove_edge(edges_to_remove.front(), g);
        }
    }
}

void remove_position_constraint(SceneGraph& g, const Boundary& boundary, std::vector<Obstacles> obstacles)
{
	int boundary_edges = boundary.Orientations.size();
    VertexIterator vi, vi_end;
	for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
		if (g[*vi].boundary >= 0 && !g[*vi].target_pos.empty() && !g[*vi].target_size.empty() && 
            !g[*vi].pos_tolerance.empty() && !g[*vi].size_tolerance.empty()) {
            double x_ = g[*vi].target_pos[0], y_ = g[*vi].target_pos[1];
            double x_t = g[*vi].pos_tolerance[0], y_t = g[*vi].pos_tolerance[1];
            double l_ = g[*vi].target_size[0], w_ = g[*vi].target_size[1];
            double l_t = g[*vi].size_tolerance[0], w_t = g[*vi].size_tolerance[1];
            double x1_ = boundary.points[g[*vi].boundary][0];
            double x2_ = boundary.points[(g[*vi].boundary + 1) % boundary_edges][0];
            double y1_ = boundary.points[g[*vi].boundary][1];
			double y2_ = boundary.points[(g[*vi].boundary + 1) % boundary_edges][1];
            double x1 = std::min(x1_, x2_), x2 = std::max(x1_, x2_);
            double y1 = std::min(y1_, y2_), y2 = std::max(y1_, y2_);
            switch (boundary.Orientations[g[*vi].boundary])
            {
			case FRONT: 
                if (!check_overlap({ x_, y_, x_t, y_t }, { (x1 + x2) / 2, y1 - w_ / 2, x2 - x1, y_t / 2 }))
				    g[*vi].target_pos = {};
				break;
			case BACK: 
				if (!check_overlap({ x_, y_, x_t, y_t }, { (x1 + x2) / 2, y1 + w_ / 2, x2 - x1, y_t / 2 }))
					g[*vi].target_pos = {};
				break;
			case LEFT:
				if (!check_overlap({ x_, y_, x_t, y_t }, { x1 + l_ / 2, (y1 + y2) / 2, l_t / 2, y2 - y1 }))
					g[*vi].target_pos = {};
				break;
			case RIGHT:
                if (!check_overlap({ x_, y_, x_t, y_t }, { x1 - l_ / 2, (y1 + y2) / 2, l_t / 2, y2 - y1 }))
                    g[*vi].target_pos = {};
				break;
			default:
				break;
            }
			if (!g[*vi].target_pos.empty()) {
                if (!check_inside({ x_ - x_t / 2, y_ - y_t / 2, l_ - 2 * x_t, w_ - 2 * y_t },
                    { boundary.origin_pos[0] + boundary.size[0] / 2, boundary.origin_pos[1] + boundary.size[1] / 2,
                    boundary.size[0], boundary.size[1] }))
                    g[*vi].target_pos = {};
                else
                {
                    for (auto i = 0; i < obstacles.size(); ++i) {
                        if (check_overlap({ x_ - x_t / 2, y_ - y_t / 2, l_ - 2 * x_t, w_ - 2 * y_t },
                            { obstacles[i].pos[0], obstacles[i].pos[1], obstacles[i].size[0], obstacles[i].size[1] }))
                        {
                            g[*vi].target_pos = {};
                            break;
                        }
                    }
                }
            }
            if (g[*vi].target_pos.empty())
                g[*vi].pos_tolerance = {};
		}
	}
}

SceneGraph Processor(const SceneGraph& inputGraph, const Boundary& boundary, std::vector<Obstacles> obstacles)
{
    SceneGraph outputGraph = inputGraph;
    // Find and work with rings in each type of edge
    std::vector<std::pair<VertexDescriptor, VertexDescriptor>> edges_to_reverse, edges_to_remove;
	std::vector<EdgeProperties> new_edge_properties;
    EdgeIterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(outputGraph); ei != ei_end; ++ei) {
		VertexDescriptor v1 = boost::source(*ei, outputGraph), v2 = boost::target(*ei, outputGraph);
        EdgeProperties ep = outputGraph[*ei];
        if (ep.type == RightOf) {
            ep.type = LeftOf;
			edges_to_reverse.push_back(std::make_pair(v1, v2));
			new_edge_properties.push_back(ep);
        }
            
        if (ep.type == Behind) {
            ep.type = FrontOf;
			edges_to_reverse.push_back(std::make_pair(v1, v2));
			new_edge_properties.push_back(ep);
        }
			
        if (ep.type == Under) {
            ep.type = Above;
			edges_to_reverse.push_back(std::make_pair(v1, v2));
			new_edge_properties.push_back(ep);
        }
			
    }
    for (size_t i = 0; i < edges_to_reverse.size(); ++i) {
        boost::remove_edge(edges_to_reverse[i].first, edges_to_reverse[i].second, outputGraph);
        boost::add_edge(edges_to_reverse[i].second, edges_to_reverse[i].first, new_edge_properties[i], outputGraph);
    }
    for (EdgeType edgetype : {LeftOf, RightOf, FrontOf, Behind, Above, Under}) {
        remove_cycles(outputGraph, edgetype);
    }
    // Find the contradiction between boundary constraints and position/orientation constraints
    VertexIterator vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(outputGraph); vi != vi_end; ++vi) {
        Orientation o_vi = outputGraph[*vi].orientation;
        if (outputGraph[*vi].boundary >= 0 && boundary.Orientations[outputGraph[*vi].boundary] == o_vi)
			outputGraph[*vi].orientation = opposite_orientation(o_vi);
        // check for contradictions between position/size constraints and on-floor constarints
        if (!outputGraph[*vi].target_pos.empty() && !outputGraph[*vi].pos_tolerance.empty() &&
            !outputGraph[*vi].target_size.empty() && !outputGraph[*vi].size_tolerance.empty() &&
            outputGraph[*vi].target_pos[2] - outputGraph[*vi].pos_tolerance[2] >
            outputGraph[*vi].target_size[2] / 2 + outputGraph[*vi].size_tolerance[2] / 2)
            outputGraph[*vi].target_pos[2] = outputGraph[*vi].target_size[2] / 2;
    }
    remove_position_constraint(outputGraph, boundary, obstacles);
    // Find contradiction between on floor constraints and above/under constraints
    for (boost::tie(ei, ei_end) = boost::edges(outputGraph); ei != ei_end; ++ei) {
		VertexDescriptor v1 = boost::source(*ei, outputGraph);
		VertexDescriptor v2 = boost::target(*ei, outputGraph);
        if (outputGraph[v1].on_floor && (outputGraph[*ei].type == Above || outputGraph[*ei].type == Under))
            outputGraph[v1].on_floor = false;
        //REMOVE EDGES IF TWO NODES BOTH HAVE BOUNDARY CONSTRAINTS
		if (outputGraph[v1].boundary >= 0 && outputGraph[v2].boundary >= 0)
			edges_to_remove.push_back(std::make_pair(v1, v2));
    }
	for (auto i = 0; i < edges_to_remove.size(); ++i) {
		boost::remove_edge(edges_to_remove[i].first, edges_to_remove[i].second, outputGraph);
	}
    return outputGraph;
}

SceneGraph SplitGraph4(const SceneGraph& g, const Boundary& boundary)
{
    SceneGraph g_split = {};
	auto num_vertices = boost::num_vertices(g), num_edges = boost::num_edges(g);
    std::map<int, VertexDescriptor> id_to_vertex;
	// Split the graph into four parts
    VertexIterator vi, vi_end;
	for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
        VertexProperties vp1 = g[*vi], vp2 = g[*vi], vp3 = g[*vi], vp4 = g[*vi];
		vp2.id = g[*vi].id + num_vertices; vp3.id = g[*vi].id + 2 * num_vertices; vp4.id = g[*vi].id + 3 * num_vertices;

		if (!g[*vi].target_size.empty()) {
            vp1.target_size = { g[*vi].target_size[0] / 2, g[*vi].target_size[1] / 2, g[*vi].target_size[2]};
			vp2.target_size = vp1.target_size;
			vp3.target_size = vp1.target_size;
			vp4.target_size = vp1.target_size;
			if (!g[*vi].size_tolerance.empty()) {
				vp1.size_tolerance = { g[*vi].size_tolerance[0] / 2, g[*vi].size_tolerance[1] / 2, g[*vi].size_tolerance[2] };
				vp2.size_tolerance = vp1.size_tolerance;
				vp3.size_tolerance = vp1.size_tolerance;
				vp4.size_tolerance = vp1.size_tolerance;
            }
            if (!g[*vi].target_pos.empty()){
				vp1.target_pos = { g[*vi].target_pos[0] - g[*vi].target_size[0] / 4, g[*vi].target_pos[1] - g[*vi].target_size[1] / 4, g[*vi].target_pos[2] };
				vp2.target_pos = { g[*vi].target_pos[0] + g[*vi].target_size[0] / 4, g[*vi].target_pos[1] - g[*vi].target_size[1] / 4, g[*vi].target_pos[2] };
				vp3.target_pos = { g[*vi].target_pos[0] - g[*vi].target_size[0] / 4, g[*vi].target_pos[1] + g[*vi].target_size[1] / 4, g[*vi].target_pos[2] };
				vp4.target_pos = { g[*vi].target_pos[0] + g[*vi].target_size[0] / 4, g[*vi].target_pos[1] + g[*vi].target_size[1] / 4, g[*vi].target_pos[2] };
            }
            if (!g[*vi].pos_tolerance.empty()) {
				vp1.pos_tolerance = { g[*vi].pos_tolerance[0] / 2, g[*vi].pos_tolerance[1] / 2, g[*vi].pos_tolerance[2] };
				vp2.pos_tolerance = vp1.pos_tolerance;
				vp3.pos_tolerance = vp1.pos_tolerance;
				vp4.pos_tolerance = vp1.pos_tolerance;
            }
        }

		if (g[*vi].boundary >= 0) {
            switch (boundary.Orientations[g[*vi].boundary]) {
			case FRONT:
				vp1.boundary = -1;
				vp2.boundary = -1;
				break;
			case BACK:
				vp3.boundary = -1;
				vp4.boundary = -1;
				break;
			case LEFT:
                vp2.boundary = -1;
                vp4.boundary = -1;
				break;
			case RIGHT:
                vp1.boundary = -1;
                vp3.boundary = -1;
				break;
			default:
				break;
            }
        }

		auto v1 = boost::add_vertex(vp1, g_split), v2 = boost::add_vertex(vp2, g_split),
			v3 = boost::add_vertex(vp3, g_split), v4 = boost::add_vertex(vp4, g_split);
		boost::add_edge(v1, v2, EdgeProperties{ -1, {0, g[*vi].target_size[1] / 4}, -1, LeftOf}, g_split);
		boost::add_edge(v3, v4, EdgeProperties{ -1, {0, g[*vi].target_size[1] / 4}, -1, LeftOf}, g_split);
		boost::add_edge(v3, v1, EdgeProperties{ -1, {g[*vi].target_size[0] / 4, 0}, -1, FrontOf}, g_split);
		boost::add_edge(v4, v2, EdgeProperties{ -1, {g[*vi].target_size[0] / 4, 0}, -1, FrontOf}, g_split);
	}

    for (boost::tie(vi, vi_end) = boost::vertices(g_split); vi != vi_end; ++vi) {
        id_to_vertex[g_split[*vi].id] = *vi;
    }

	EdgeIterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei) {
		VertexDescriptor vs = boost::source(*ei, g), vt = boost::target(*ei, g);
		EdgeProperties ep = g[*ei];
		std::vector<VertexDescriptor> vss = { id_to_vertex[g[vs].id], id_to_vertex[g[vs].id + num_vertices],
					id_to_vertex[g[vs].id + 2 * num_vertices], id_to_vertex[g[vs].id + 3 * num_vertices] },
            vts = { id_to_vertex[g[vt].id], id_to_vertex[g[vt].id + num_vertices], 
                    id_to_vertex[g[vt].id + 2 * num_vertices], id_to_vertex[g[vt].id + 3 * num_vertices] };

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 1), dis2(0, 3);
        switch (ep.type) {
        case LeftOf: {
            boost::add_edge(vss[2 * dis(gen) + 1], vts[2 * dis(gen)], ep, g_split);
			break;
        }
        case RightOf: {
            boost::add_edge(vss[2 * dis(gen)], vts[2 * dis(gen) + 1], ep, g_split);
			break;
        }
        case FrontOf: {
            boost::add_edge(vss[dis(gen)], vts[dis(gen) + 2], ep, g_split);
			break;
        }
        case Behind: {
            boost::add_edge(vss[dis(gen) + 2], vts[dis(gen)], ep, g_split);
			break;
        }
        default: {
            boost::add_edge(vss[dis2(gen)], vts[dis2(gen)], ep, g_split);
            break;
        }
        }
    }
	return g_split;
}

SceneGraph SplitGraph2(const SceneGraph& g, const Boundary& boundary)
{
    SceneGraph g_split = {};
    auto num_vertices = boost::num_vertices(g), num_edges = boost::num_edges(g);
    std::map<int, VertexDescriptor> id_to_vertex;
	std::vector<int> split_type(num_vertices, 0);
    // Split the graph into two parts
    VertexIterator vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
        VertexProperties vp1 = g[*vi], vp2 = g[*vi];
        vp2.id = g[*vi].id + num_vertices;
        if (!g[*vi].target_size.empty() && g[*vi].target_size[1] > g[*vi].target_size[0])
        {
            if (!g[*vi].target_pos.empty()) {
                vp1.target_pos = { g[*vi].target_pos[0], g[*vi].target_pos[1] + g[*vi].target_size[1] / 4, g[*vi].target_size[2] };
                vp2.target_pos = { g[*vi].target_pos[0], g[*vi].target_pos[1] - g[*vi].target_size[1] / 4, g[*vi].target_size[2] };
            }
            vp1.target_size = { g[*vi].target_size[0], g[*vi].target_size[1] / 2, g[*vi].target_size[2] };
            vp2.target_size = { g[*vi].target_size[0], g[*vi].target_size[1] / 2, g[*vi].target_size[2] };
            if (!g[*vi].size_tolerance.empty())
            {
                vp1.size_tolerance = { g[*vi].size_tolerance[0], g[*vi].size_tolerance[1] / 2, g[*vi].size_tolerance[2] };
                vp2.size_tolerance = vp1.size_tolerance;
            }
            if (!g[*vi].pos_tolerance.empty())
            {
				vp1.pos_tolerance = { g[*vi].pos_tolerance[0], g[*vi].pos_tolerance[1] / 2, g[*vi].pos_tolerance[2] };
				vp2.pos_tolerance = vp1.pos_tolerance;
            }
            if (g[*vi].boundary >= 0)
            {
                switch (boundary.Orientations[g[*vi].boundary])
                {
                case FRONT:
                    vp2.boundary = -1;
                    break;
                case BACK:
                    vp1.boundary = -1;
                    break;
                default: break;
                }
            }
            auto v1 = boost::add_vertex(vp1, g_split);
            auto v2 = boost::add_vertex(vp2, g_split);
            boost::add_edge(v1, v2, EdgeProperties{ -1, {g[*vi].target_size[0] / 2, 0}, -1, FrontOf}, g_split);
			split_type[g[*vi].id] = 1;
        }
        // Here we default to the vertical split |
        else
        {
            if (!g[*vi].target_size.empty())
            {
                vp1.target_size = { g[*vi].target_size[0] / 2, g[*vi].target_size[1], g[*vi].target_size[2] };
                vp2.target_size = { g[*vi].target_size[0] / 2, g[*vi].target_size[1], g[*vi].target_size[2] };
                if (!g[*vi].target_pos.empty())
                {
                    vp1.target_pos = { g[*vi].target_pos[0] + g[*vi].target_size[0] / 4, g[*vi].target_pos[1], g[*vi].target_size[2] };
                    vp2.target_pos = { g[*vi].target_pos[0] - g[*vi].target_size[0] / 4, g[*vi].target_pos[1], g[*vi].target_size[2] };
                }
            }
            if (!g[*vi].size_tolerance.empty())
            {
                vp1.size_tolerance = { g[*vi].size_tolerance[0] / 2, g[*vi].size_tolerance[1], g[*vi].size_tolerance[2] };
                vp2.size_tolerance = vp1.size_tolerance;
            }
            if (!g[*vi].pos_tolerance.empty())
            {
                vp1.pos_tolerance = { g[*vi].pos_tolerance[0] / 2, g[*vi].pos_tolerance[1], g[*vi].pos_tolerance[2] };
                vp2.pos_tolerance = vp1.pos_tolerance;
            }
            if (g[*vi].boundary >= 0)
            {
                switch (boundary.Orientations[g[*vi].boundary])
                {
                case LEFT:
                    vp1.boundary = -1;
                    break;
                case RIGHT:
                    vp2.boundary = -1;
                    break;
                default: break;
                }
            }
            auto v1 = boost::add_vertex(vp1, g_split);
            auto v2 = boost::add_vertex(vp2, g_split);
            boost::add_edge(v2, v1, EdgeProperties{ -1, {0, g[*vi].target_size[1] / 2}, -1, LeftOf}, g_split);
        }
    }

    for (boost::tie(vi, vi_end) = boost::vertices(g_split); vi != vi_end; ++vi) {
        id_to_vertex[g_split[*vi].id] = *vi;
    }

    EdgeIterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei) {
        VertexDescriptor vs = boost::source(*ei, g), vt = boost::target(*ei, g);
        EdgeProperties ep = g[*ei];
		std::vector<VertexDescriptor> vss = { id_to_vertex[g[vs].id], id_to_vertex[g[vs].id + num_vertices] },
			vts = { id_to_vertex[g[vt].id], id_to_vertex[g[vt].id + num_vertices] };

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 1);
        switch (ep.type) {
        case LeftOf:
        {
			if (split_type[g[vs].id] == 0 && split_type[g[vt].id] == 0)
				boost::add_edge(vss[0], vts[1], ep, g_split);
			else if (split_type[g[vs].id] == 0 && split_type[g[vt].id] == 1)
				boost::add_edge(vss[0], vts[dis(gen)], ep, g_split);
			else if (split_type[g[vs].id] == 1 && split_type[g[vt].id] == 0)
				boost::add_edge(vss[dis(gen)], vts[1], ep, g_split);
			else
				boost::add_edge(vss[dis(gen)], vts[dis(gen)], ep, g_split);
			break;
        }
		case RightOf:
        {
            if (split_type[g[vs].id] == 0 && split_type[g[vt].id] == 0)
                boost::add_edge(vss[1], vts[0], ep, g_split);
            else if (split_type[g[vs].id] == 0 && split_type[g[vt].id] == 1)
                boost::add_edge(vss[1], vts[dis(gen)], ep, g_split);
            else if (split_type[g[vs].id] == 1 && split_type[g[vt].id] == 0)
                boost::add_edge(vss[dis(gen)], vts[0], ep, g_split);
            else
                boost::add_edge(vss[dis(gen)], vts[dis(gen)], ep, g_split);
            break;
        }
		case FrontOf:
		{
            if (split_type[g[vs].id] == 0 && split_type[g[vt].id] == 0)
                boost::add_edge(vss[dis(gen)], vts[dis(gen)], ep, g_split);
            else if (split_type[g[vs].id] == 0 && split_type[g[vt].id] == 1)
                boost::add_edge(vss[dis(gen)], vts[0], ep, g_split);
            else if (split_type[g[vs].id] == 1 && split_type[g[vt].id] == 0)
                boost::add_edge(vss[1], vts[dis(gen)], ep, g_split);
            else
                boost::add_edge(vss[1], vts[0], ep, g_split);
            break;
		}
        case Behind:
        {
            if (split_type[g[vs].id] == 0 && split_type[g[vt].id] == 0)
                boost::add_edge(vss[dis(gen)], vts[dis(gen)], ep, g_split);
            else if (split_type[g[vs].id] == 0 && split_type[g[vt].id] == 1)
                boost::add_edge(vss[dis(gen)], vts[1], ep, g_split);
            else if (split_type[g[vs].id] == 1 && split_type[g[vt].id] == 0)
                boost::add_edge(vss[0], vts[dis(gen)], ep, g_split);
            else
                boost::add_edge(vss[0], vts[1], ep, g_split);
            break;
        }
        default:
        {
            boost::add_edge(vss[dis(gen)], vts[dis(gen)], ep, g_split);
            break;
        }
        }
    }
    return g_split;
}