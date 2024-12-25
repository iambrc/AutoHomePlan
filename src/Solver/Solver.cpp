#include "Components/Solver.h"

#include <boost/graph/graphviz.hpp>
#include <fstream>

std::vector<std::string> show_edges = { "Left of", "Right of", "Front of", "Behind", "Above", "Under", "Close by", "Align with" };
std::vector<std::string> show_orientations = { "up", "down", "left", "right", "front", "back" };

Solver::Solver() : env(), model(env) {
    // Initialize solver-related data if needed
    hyperparameters = {1, 1, 1, 1};
	scalingFactor = 3;
}

Solver::~Solver() {}

bool Solver::has_path(const SceneGraph& g, VertexDescriptor start, VertexDescriptor target)
{
    std::vector<bool> visited_1(boost::num_vertices(g), false), visited_2(boost::num_vertices(g), false),
		visited_3(boost::num_vertices(g), false), visited_4(boost::num_vertices(g), false),
		visited_5(boost::num_vertices(g), false), visited_6(boost::num_vertices(g), false);
	return (dfs_check_path(g, start, target, LeftOf, visited_1) || dfs_check_path(g, start, target, RightOf, visited_1) || 
		dfs_check_path(g, start, target, FrontOf, visited_3) || dfs_check_path(g, start, target, Behind, visited_4) || 
		dfs_check_path(g, start, target, Above, visited_5) || dfs_check_path(g, start, target, Under, visited_6));
}

bool Solver::dfs_check_path(const SceneGraph& g, VertexDescriptor u, VertexDescriptor target, EdgeType required_type, std::vector<bool>& visited)
{
	if (g[u].id == g[target].id) return true;
	visited[g[u].id] = true;

	for (const auto& edge : boost::make_iterator_range(boost::out_edges(u, g))) {
		VertexDescriptor v = boost::target(edge, g);
		if (!visited[g[v].id] && g[edge].type == required_type) {
			if (dfs_check_path(g, v, target, required_type, visited)) {
				return true;
			}
		}
	}
	return false;
}

void Solver::addConstraints()
{
	int num_vertices = boost::num_vertices(g);
	int num_obstacles = obstacles.size();
	double M = boundary.size[0] + boundary.size[1] + boundary.size[2];
	std::vector<GRBVar> x_i(num_vertices), y_i(num_vertices), z_i(num_vertices),
		l_i(num_vertices), w_i(num_vertices), h_i(num_vertices);
	std::vector<std::vector<GRBVar>> sigma_L(num_vertices, std::vector<GRBVar>(num_vertices)),
		sigma_R(num_vertices, std::vector<GRBVar>(num_vertices)),
		sigma_F(num_vertices, std::vector<GRBVar>(num_vertices)),
		sigma_B(num_vertices, std::vector<GRBVar>(num_vertices)),
		sigma_U(num_vertices, std::vector<GRBVar>(num_vertices)),
		sigma_D(num_vertices, std::vector<GRBVar>(num_vertices)),
		adjacency(num_vertices, std::vector<GRBVar>(num_vertices)),
		sigma_oL(num_vertices, std::vector<GRBVar>(num_obstacles)),
		sigma_oR(num_vertices, std::vector<GRBVar>(num_obstacles)),
		sigma_oF(num_vertices, std::vector<GRBVar>(num_obstacles)),
		sigma_oB(num_vertices, std::vector<GRBVar>(num_obstacles)),
		sigma_oU(num_vertices, std::vector<GRBVar>(num_obstacles)),
		sigma_oD(num_vertices, std::vector<GRBVar>(num_obstacles));
	for (int i = 0; i < num_vertices; ++i) {
		x_i[i] = model.addVar(boundary.origin_pos[0], boundary.origin_pos[0] + boundary.size[0], 0.0, GRB_CONTINUOUS, "x_" + std::to_string(i));
		y_i[i] = model.addVar(boundary.origin_pos[1], boundary.origin_pos[1] + boundary.size[1], 0.0, GRB_CONTINUOUS, "y_" + std::to_string(i));
		l_i[i] = model.addVar(0.0, boundary.size[0], 0.0, GRB_CONTINUOUS, "l_" + std::to_string(i));
		w_i[i] = model.addVar(0.0, boundary.size[1], 0.0, GRB_CONTINUOUS, "w_" + std::to_string(i));
		if (!floorplan) {
			z_i[i] = model.addVar(boundary.origin_pos[2], boundary.origin_pos[2] + boundary.size[2], 0.0, GRB_CONTINUOUS, "z_" + std::to_string(i));
			h_i[i] = model.addVar(0.0, boundary.size[2], 0.0, GRB_CONTINUOUS, "h_" + std::to_string(i));
		}
	}
	// Inside Constraints & tolerance Constraint
	VertexIterator vi, vi_end;
	for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
		model.addConstr(x_i[g[*vi].id] - l_i[g[*vi].id] / 2 >= boundary.origin_pos[0], "Inside_Object_" + std::to_string(g[*vi].id) + "_x_left");
		model.addConstr(x_i[g[*vi].id] + l_i[g[*vi].id] / 2 <= boundary.origin_pos[0] + boundary.size[0], "Inside_Object_" + std::to_string(g[*vi].id) + "_x_right");
		model.addConstr(y_i[g[*vi].id] - w_i[g[*vi].id] / 2 >= boundary.origin_pos[1], "Inside_Object_" + std::to_string(g[*vi].id) + "_y_back");
		model.addConstr(y_i[g[*vi].id] + w_i[g[*vi].id] / 2 <= boundary.origin_pos[1] + boundary.size[1], "Inside_Object_" + std::to_string(g[*vi].id) + "_y_front");
		if (!floorplan) {
			model.addConstr(z_i[g[*vi].id] - h_i[g[*vi].id] / 2 >= boundary.origin_pos[2], "Inside_Object_" + std::to_string(g[*vi].id) + "_z_bottom");
			model.addConstr(z_i[g[*vi].id] + h_i[g[*vi].id] / 2 <= boundary.origin_pos[2] + boundary.size[2], "Inside_Object_" + std::to_string(g[*vi].id) + "_z_top");
		}
		if (!g[*vi].pos_tolerance.empty() && !g[*vi].target_pos.empty()) {
			model.addConstr(x_i[g[*vi].id] >= g[*vi].target_pos[0] - g[*vi].pos_tolerance[0], "Pos_Tolerance_Object_" + std::to_string(g[*vi].id) + "_x_left");
			model.addConstr(x_i[g[*vi].id] <= g[*vi].target_pos[0] + g[*vi].pos_tolerance[0], "Pos_Tolerance_Object_" + std::to_string(g[*vi].id) + "_x_right");
			model.addConstr(y_i[g[*vi].id] >= g[*vi].target_pos[1] - g[*vi].pos_tolerance[1], "Pos_Tolerance_Object_" + std::to_string(g[*vi].id) + "_y_back");
			model.addConstr(y_i[g[*vi].id] <= g[*vi].target_pos[1] + g[*vi].pos_tolerance[1], "Pos_Tolerance_Object_" + std::to_string(g[*vi].id) + "_y_front");
			if (!floorplan) {
				model.addConstr(z_i[g[*vi].id] >= g[*vi].target_pos[2] - g[*vi].pos_tolerance[2], "Pos_Tolerance_Object_" + std::to_string(g[*vi].id) + "_z_bottom");
				model.addConstr(z_i[g[*vi].id] <= g[*vi].target_pos[2] + g[*vi].pos_tolerance[2], "Pos_Tolerance_Object_" + std::to_string(g[*vi].id) + "_z_top");
			}
		}
		if (!g[*vi].size_tolerance.empty() && !g[*vi].target_size.empty()) {
			model.addConstr(l_i[g[*vi].id] >= g[*vi].target_size[0] - g[*vi].size_tolerance[0], "Size_Tolerance_Object_" + std::to_string(g[*vi].id) + "_l_min");
			model.addConstr(l_i[g[*vi].id] <= g[*vi].target_size[0] + g[*vi].size_tolerance[0], "Size_Tolerance_Object_" + std::to_string(g[*vi].id) + "_l_max");
			model.addConstr(w_i[g[*vi].id] >= g[*vi].target_size[1] - g[*vi].size_tolerance[1], "Size_Tolerance_Object_" + std::to_string(g[*vi].id) + "_w_min");
			model.addConstr(w_i[g[*vi].id] <= g[*vi].target_size[1] + g[*vi].size_tolerance[1], "Size_Tolerance_Object_" + std::to_string(g[*vi].id) + "_w_max");
			if (!floorplan) {
				model.addConstr(h_i[g[*vi].id] >= g[*vi].target_size[2] - g[*vi].size_tolerance[2], "Size_Tolerance_Object_" + std::to_string(g[*vi].id) + "_h_min");
				model.addConstr(h_i[g[*vi].id] <= g[*vi].target_size[2] + g[*vi].size_tolerance[2], "Size_Tolerance_Object_" + std::to_string(g[*vi].id) + "_h_max");
			}
		}
	}
	// On floor Constraints
	for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
		if (g[*vi].on_floor && !floorplan) {
			model.addConstr(z_i[g[*vi].id] == boundary.origin_pos[2] + h_i[g[*vi].id] / 2, "On_Floor_Object_" + std::to_string(g[*vi].id));
		}
	}
	// Adjacency Constraints
	EdgeIterator ei, ei_end;
	for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei) {
		VertexDescriptor source = boost::source(*ei, g);
		VertexDescriptor target = boost::target(*ei, g);
		int ids = g[source].id, idt = g[target].id;
		std::vector<double> min_adj = g[*ei].closeby_tolerance;
		if (min_adj.empty()) {
			min_adj = { 0.0, 0.0 };
		}
		switch(g[*ei].type)
		{
		case LeftOf:
			if (g[*ei].distance >= 0)
				model.addConstr(x_i[ids] + l_i[ids] / 2 <= x_i[idt] - l_i[idt] / 2, "Object_" + std::to_string(ids) + "_LeftOf_Object_" + std::to_string(idt));
			else
			{
				model.addConstr(x_i[ids] + l_i[ids] / 2 == x_i[idt] - l_i[idt] / 2, "Object_" + std::to_string(ids) + "_LeftOf_Object_" + std::to_string(idt) + "eq1");
				model.addConstr(y_i[ids] + w_i[ids] / 2 >= y_i[idt] - w_i[idt] / 2 + min_adj[1], "Object_" + std::to_string(ids) + "_LeftOf_Object_" + std::to_string(idt) + "ieq1");
				model.addConstr(y_i[ids] - w_i[ids] / 2 <= y_i[idt] + w_i[idt] / 2 - min_adj[1], "Object_" + std::to_string(ids) + "_LeftOf_Object_" + std::to_string(idt) + "ieq2");
			}
			break;
		case RightOf:
			if (g[*ei].distance >= 0)
				model.addConstr(x_i[ids] - l_i[ids] / 2 >= x_i[idt] + l_i[idt] / 2, "Object_" + std::to_string(ids) + "_RightOf_Object_" + std::to_string(idt));
			else
			{
				model.addConstr(x_i[ids] - l_i[ids] / 2 == x_i[idt] + l_i[idt] / 2, "Object_" + std::to_string(ids) + "_RightOf_Object_" + std::to_string(idt) + "eq1");
				model.addConstr(y_i[ids] + w_i[ids] / 2 >= y_i[idt] - w_i[idt] / 2 + min_adj[1], "Object_" + std::to_string(ids) + "_RightOf_Object_" + std::to_string(idt) + "ieq1");
				model.addConstr(y_i[ids] - w_i[ids] / 2 <= y_i[idt] + w_i[idt] / 2 - min_adj[1], "Object_" + std::to_string(ids) + "_RightOf_Object_" + std::to_string(idt) + "ieq2");
			}
			break;
		case Behind:
			if (g[*ei].distance >= 0)
				model.addConstr(y_i[ids] + w_i[ids] / 2 <= y_i[idt] - w_i[idt] / 2, "Object_" + std::to_string(ids) + "_Behind_Object_" + std::to_string(idt));
			else
			{
				model.addConstr(y_i[ids] + w_i[ids] / 2 == y_i[idt] - w_i[idt] / 2, "Object_" + std::to_string(ids) + "_Behind_Object_" + std::to_string(idt) + "eq1");
				model.addConstr(x_i[ids] + l_i[ids] / 2 >= x_i[idt] - l_i[idt] / 2 + min_adj[0], "Object_" + std::to_string(ids) + "_Behind_Object_" + std::to_string(idt) + "ieq1");
				model.addConstr(x_i[ids] - l_i[ids] / 2 <= x_i[idt] + l_i[idt] / 2 - min_adj[0], "Object_" + std::to_string(ids) + "_Behind_Object_" + std::to_string(idt) + "ieq2");
			}
			break;
		case FrontOf:
			if (g[*ei].distance >= 0)
				model.addConstr(y_i[ids] - w_i[ids] / 2 >= y_i[idt] + w_i[idt] / 2, "Object_" + std::to_string(ids) + "_FrontOf_Object_" + std::to_string(idt));
			else
			{
				model.addConstr(y_i[ids] - w_i[ids] / 2 == y_i[idt] + w_i[idt] / 2, "Object_" + std::to_string(ids) + "_FrontOf_Object_" + std::to_string(idt) + "eq1");
				model.addConstr(x_i[ids] + l_i[ids] / 2 >= x_i[idt] - l_i[idt] / 2 + min_adj[0], "Object_" + std::to_string(ids) + "_FrontOf_Object_" + std::to_string(idt) + "ieq1");
				model.addConstr(x_i[ids] - l_i[ids] / 2 <= x_i[idt] + l_i[idt] / 2 - min_adj[0], "Object_" + std::to_string(ids) + "_FrontOf_Object_" + std::to_string(idt) + "ieq2");
			}
			break;
		case Under:
			if (g[*ei].distance >= 0)
				model.addConstr(z_i[ids] + h_i[ids] / 2 <= z_i[idt] - h_i[idt] / 2, "Object_" + std::to_string(ids) + "_Under_Object_" + std::to_string(idt));
			else
				model.addConstr(z_i[ids] + h_i[ids] / 2 == z_i[idt] - h_i[idt] / 2, "Object_" + std::to_string(ids) + "_Under_Object_" + std::to_string(idt));
			model.addConstr(x_i[ids] - l_i[ids] / 2 <= x_i[idt] - l_i[idt] / 2, "Object_" + std::to_string(ids) + "_Under_Object_" + std::to_string(idt) + "ieq1");
			model.addConstr(x_i[ids] + l_i[ids] / 2 >= x_i[idt] + l_i[idt] / 2, "Object_" + std::to_string(ids) + "_Under_Object_" + std::to_string(idt) + "ieq2");
			model.addConstr(y_i[ids] - w_i[ids] / 2 <= y_i[idt] - w_i[idt] / 2, "Object_" + std::to_string(ids) + "_Under_Object_" + std::to_string(idt) + "ieq3");
			model.addConstr(y_i[ids] + w_i[ids] / 2 >= y_i[idt] + w_i[idt] / 2, "Object_" + std::to_string(ids) + "_Under_Object_" + std::to_string(idt) + "ieq4");
			break;
		case Above:
			if (g[*ei].distance >= 0)
				model.addConstr(z_i[ids] - h_i[ids] / 2 >= z_i[idt] + h_i[idt] / 2, "Object_" + std::to_string(ids) + "_Above_Object_" + std::to_string(idt));
			else
				model.addConstr(z_i[ids] - h_i[ids] / 2 == z_i[idt] + h_i[idt] / 2, "Object_" + std::to_string(ids) + "_Above_Object_" + std::to_string(idt));
			model.addConstr(x_i[ids] - l_i[ids] / 2 >= x_i[idt] - l_i[idt] / 2, "Object_" + std::to_string(ids) + "_Above_Object_" + std::to_string(idt) + "ieq1");
			model.addConstr(x_i[ids] + l_i[ids] / 2 <= x_i[idt] + l_i[idt] / 2, "Object_" + std::to_string(ids) + "_Above_Object_" + std::to_string(idt) + "ieq2");
			model.addConstr(y_i[ids] - w_i[ids] / 2 >= y_i[idt] - w_i[idt] / 2, "Object_" + std::to_string(ids) + "_Above_Object_" + std::to_string(idt) + "ieq3");
			model.addConstr(y_i[ids] + w_i[ids] / 2 <= y_i[idt] + w_i[idt] / 2, "Object_" + std::to_string(ids) + "_Above_Object_" + std::to_string(idt) + "ieq4");
			break;
		case CloseBy:
			adjacency[ids][idt] = model.addVar(0, 1, 0, GRB_BINARY);
			model.addConstr(x_i[ids] - l_i[ids] / 2 <= x_i[idt] + l_i[idt] / 2 - min_adj[0] * adjacency[ids][idt], "Object_" + std::to_string(ids) + "_CloseBy_Object_" + std::to_string(idt) + "ieq1");
			model.addConstr(x_i[ids] + l_i[ids] / 2 >= x_i[idt] - l_i[idt] / 2 + min_adj[0] * adjacency[ids][idt], "Object_" + std::to_string(ids) + "_CloseBy_Object_" + std::to_string(idt) + "ieq2");
			model.addConstr(y_i[ids] - w_i[ids] / 2 <= y_i[idt] + w_i[idt] / 2 - min_adj[1] * (1 - adjacency[ids][idt]), "Object_" + std::to_string(ids) + "_CloseBy_Object_" + std::to_string(idt) + "ieq3");
			model.addConstr(y_i[ids] + w_i[ids] / 2 >= y_i[idt] - w_i[idt] / 2 + min_adj[1] * (1 - adjacency[ids][idt]), "Object_" + std::to_string(ids) + "_CloseBy_Object_" + std::to_string(idt) + "ieq4");
			break;
		case AlignWith:
			switch (g[*ei].align_edge)
			{
			case 0:
				model.addConstr(y_i[ids] - w_i[ids] / 2 == y_i[idt] - w_i[idt] / 2, "Object_" + std::to_string(ids) + "_AlignWith_Object_" + std::to_string(idt));
				break;
			case 1:
				model.addConstr(x_i[ids] + l_i[ids] / 2 == x_i[idt] + l_i[idt] / 2, "Object_" + std::to_string(ids) + "_AlignWith_Object_" + std::to_string(idt));
				break;
			case 2:
				model.addConstr(y_i[ids] + w_i[ids] / 2 == y_i[idt] + w_i[idt] / 2, "Object_" + std::to_string(ids) + "_AlignWith_Object_" + std::to_string(idt));
				break;
			case 3:
				model.addConstr(x_i[ids] - l_i[ids] / 2 == x_i[idt] - l_i[idt] / 2, "Object_" + std::to_string(ids) + "_AlignWith_Object_" + std::to_string(idt));
				break;
			case 4:
				model.addConstr(z_i[ids] - h_i[ids] / 2 == z_i[idt] + h_i[idt] / 2, "Object_" + std::to_string(ids) + "_AlignWith_Object_" + std::to_string(idt));
				break;
			case 5:
				model.addConstr(z_i[ids] + h_i[ids] / 2 == z_i[idt] - h_i[idt] / 2, "Object_" + std::to_string(ids) + "_AlignWith_Object_" + std::to_string(idt));
				break;
			default:break;
			}
			break;
		default:break;
		}
	}
	// Non overlap Constraints
	auto vi_start_end = boost::vertices(g);
	VertexIterator vj;
	for (vi = vi_start_end.first; vi != vi_start_end.second; ++vi) {
		for (vj = vi_start_end.first; vj != vi_start_end.second; ++vj) {
			if (g[*vi].id < g[*vj].id && !has_path(g, *vi, *vj) && !has_path(g, *vj, *vi)) {
				sigma_R[g[*vi].id][g[*vj].id] = model.addVar(0, 1, 0, GRB_BINARY);
				sigma_L[g[*vi].id][g[*vj].id] = model.addVar(0, 1, 0, GRB_BINARY);
				sigma_F[g[*vi].id][g[*vj].id] = model.addVar(0, 1, 0, GRB_BINARY);
				sigma_B[g[*vi].id][g[*vj].id] = model.addVar(0, 1, 0, GRB_BINARY);
				model.addConstr(x_i[g[*vi].id] - l_i[g[*vi].id] / 2 >= x_i[g[*vj].id] + l_i[g[*vj].id] / 2 -
					M * (1 - sigma_R[g[*vi].id][g[*vj].id]), "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Object_" + std::to_string(g[*vj].id) + "R");
				model.addConstr(x_i[g[*vi].id] + l_i[g[*vi].id] / 2 <= x_i[g[*vj].id] - l_i[g[*vj].id] / 2 +
					M * (1 - sigma_L[g[*vi].id][g[*vj].id]), "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Object_" + std::to_string(g[*vj].id) + "L");
				model.addConstr(y_i[g[*vi].id] - w_i[g[*vi].id] / 2 >= y_i[g[*vj].id] + w_i[g[*vj].id] / 2 - 
					M * (1 - sigma_F[g[*vi].id][g[*vj].id]), "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Object_" + std::to_string(g[*vj].id) + "F");
				model.addConstr(y_i[g[*vi].id] + w_i[g[*vi].id] / 2 <= y_i[g[*vj].id] - w_i[g[*vj].id] / 2 + 
					M * (1 - sigma_B[g[*vi].id][g[*vj].id]), "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Object_" + std::to_string(g[*vj].id) + "B");

				if (!floorplan) {
					sigma_U[g[*vi].id][g[*vj].id] = model.addVar(0, 1, 0, GRB_BINARY);
					sigma_D[g[*vi].id][g[*vj].id] = model.addVar(0, 1, 0, GRB_BINARY);
					model.addConstr(z_i[g[*vi].id] - h_i[g[*vi].id] / 2 >= z_i[g[*vj].id] + h_i[g[*vj].id] / 2 -
						M * (1 - sigma_U[g[*vi].id][g[*vj].id]), "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Object_" + std::to_string(g[*vj].id) + "U");
					model.addConstr(z_i[g[*vi].id] + h_i[g[*vi].id] / 2 <= z_i[g[*vj].id] - h_i[g[*vj].id] / 2 +
						M * (1 - sigma_D[g[*vi].id][g[*vj].id]), "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Object_" + std::to_string(g[*vj].id) + "D");
					model.addConstr(sigma_L[g[*vi].id][g[*vj].id] + sigma_R[g[*vi].id][g[*vj].id] +
						sigma_F[g[*vi].id][g[*vj].id] + sigma_B[g[*vi].id][g[*vj].id] +
						sigma_U[g[*vi].id][g[*vj].id] + sigma_D[g[*vi].id][g[*vj].id] >= 1, "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Object_" + std::to_string(g[*vj].id));
				}
				else {
					model.addConstr(sigma_L[g[*vi].id][g[*vj].id] + sigma_R[g[*vi].id][g[*vj].id] +
						sigma_F[g[*vi].id][g[*vj].id] + sigma_B[g[*vi].id][g[*vj].id] >= 1, "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Object_" + std::to_string(g[*vj].id));
				}
			}
		}
	}
	// Obstacle Constraints
	for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
		for (int i = 0; i < num_obstacles; ++i) {
			GRBLinExpr sigma_o = 0;
			if (obstacles[i].pos[0] - obstacles[i].size[0] / 2 - boundary.origin_pos[0] > 1e-3)
			{
				sigma_oL[g[*vi].id][i] = model.addVar(0, 1, 0, GRB_BINARY);
				model.addConstr(x_i[g[*vi].id] + l_i[g[*vi].id] / 2 <= obstacles[i].pos[0] - obstacles[i].size[0] / 2 +
					M * (1 - sigma_oL[g[*vi].id][i]), "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Obstacle_" + std::to_string(i) + "L");
				sigma_o += sigma_oL[g[*vi].id][i];
			}
			if (boundary.origin_pos[0] + boundary.size[0] / 2 - obstacles[i].pos[0] - obstacles[i].size[0] / 2 > 1e-3)
			{
				sigma_oR[g[*vi].id][i] = model.addVar(0, 1, 0, GRB_BINARY);
				model.addConstr(x_i[g[*vi].id] - l_i[g[*vi].id] / 2 >= obstacles[i].pos[0] + obstacles[i].size[0] / 2 -
					M * (1 - sigma_oR[g[*vi].id][i]), "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Obstacle_" + std::to_string(i) + "R");
				sigma_o += sigma_oR[g[*vi].id][i];
			}
			if (obstacles[i].pos[1] - obstacles[i].size[1] / 2 - boundary.origin_pos[1] > 1e-3)
			{
				sigma_oB[g[*vi].id][i] = model.addVar(0, 1, 0, GRB_BINARY);
				model.addConstr(y_i[g[*vi].id] + w_i[g[*vi].id] / 2 <= obstacles[i].pos[1] - obstacles[i].size[1] / 2 +
					M * (1 - sigma_oB[g[*vi].id][i]), "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Obstacle_" + std::to_string(i) + "B");	
				sigma_o += sigma_oB[g[*vi].id][i];
			}
			if (boundary.origin_pos[1] + boundary.size[1] / 2 - obstacles[i].pos[1] - obstacles[i].size[1] / 2 > 1e-3)
			{
				sigma_oF[g[*vi].id][i] = model.addVar(0, 1, 0, GRB_BINARY);
				model.addConstr(y_i[g[*vi].id] - w_i[g[*vi].id] / 2 >= obstacles[i].pos[1] + obstacles[i].size[1] / 2 -
					M * (1 - sigma_oF[g[*vi].id][i]), "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Obstacle_" + std::to_string(i) + "F");
				sigma_o += sigma_oF[g[*vi].id][i];
			}

			if (!floorplan) {
				if (obstacles[i].pos[2] - obstacles[i].size[2] / 2 - boundary.origin_pos[2] > 1e-3)
				{
					sigma_oD[g[*vi].id][i] = model.addVar(0, 1, 0, GRB_BINARY);
					model.addConstr(z_i[g[*vi].id] + h_i[g[*vi].id] / 2 <= obstacles[i].pos[2] - obstacles[i].size[2] / 2 +
						M * (1 - sigma_oD[g[*vi].id][i]), "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Obstacle_" + std::to_string(i) + "D");
					sigma_o += sigma_oD[g[*vi].id][i];
				}
				if (boundary.origin_pos[2] + boundary.size[2] / 2 - obstacles[i].pos[2] - obstacles[i].size[2] / 2 > 1e-3)
				{
					sigma_oU[g[*vi].id][i] = model.addVar(0, 1, 0, GRB_BINARY);
					model.addConstr(z_i[g[*vi].id] - h_i[g[*vi].id] / 2 >= obstacles[i].pos[2] + obstacles[i].size[2] / 2 -
						M * (1 - sigma_oU[g[*vi].id][i]), "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Obstacle_" + std::to_string(i) + "U");
					sigma_o += sigma_oU[g[*vi].id][i];
				}
			}
			model.addConstr(sigma_o >= 1, "NonOverlap_Object_" + std::to_string(g[*vi].id) + "and_Obstacle_" + std::to_string(i));	
		}
	}
	// Boundary Constraints
	for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
		if (g[*vi].boundary >= 0) {
			double x1 = boundary.points[g[*vi].boundary][0], x2 = boundary.points[(g[*vi].boundary + 1) % boundary.Orientations.size()][0];
			double y1 = boundary.points[g[*vi].boundary][1], y2 = boundary.points[(g[*vi].boundary + 1) % boundary.Orientations.size()][1];
			double x1_ = std::min(x1, x2), x2_ = std::max(x1, x2);
			double y1_ = std::min(y1, y2), y2_ = std::max(y1, y2);
			switch (boundary.Orientations[g[*vi].boundary])
			{
			case LEFT:
				model.addConstr(x_i[g[*vi].id] - l_i[g[*vi].id] / 2 == x1_, "Boundary_Object_" + std::to_string(g[*vi].id) + "_Left_eq1");
				// Here we assume that on boundary means at least half of length is on the wall
				model.addConstr(y_i[g[*vi].id] >= y1_, "Boundary_Object_" + std::to_string(g[*vi].id) + "_Left_ieq1");
				model.addConstr(y_i[g[*vi].id] <= y2_, "Boundary_Object_" + std::to_string(g[*vi].id) + "_Left_ieq2");
				break;
			case RIGHT:
				model.addConstr(x_i[g[*vi].id] + l_i[g[*vi].id] / 2 == x1_, "Boundary_Object_" + std::to_string(g[*vi].id) + "_Right_eq1");
				model.addConstr(y_i[g[*vi].id] >= y1_, "Boundary_Object_" + std::to_string(g[*vi].id) + "_Right_ieq1");
				model.addConstr(y_i[g[*vi].id] <= y2_, "Boundary_Object_" + std::to_string(g[*vi].id) + "_Right_ieq2");
				break;
			case FRONT:
				model.addConstr(y_i[g[*vi].id] + w_i[g[*vi].id] / 2 == y1_, "Boundary_Object_" + std::to_string(g[*vi].id) + "_Front_eq1");
				model.addConstr(x_i[g[*vi].id] >= x1_, "Boundary_Object_" + std::to_string(g[*vi].id) + "_Front_ieq1");
				model.addConstr(x_i[g[*vi].id] <= x2_, "Boundary_Object_" + std::to_string(g[*vi].id) + "_Front_ieq2");
				break;
			case BACK:
				model.addConstr(y_i[g[*vi].id] - w_i[g[*vi].id] / 2 == y1_, "Boundary_Object_" + std::to_string(g[*vi].id) + "_Back_eq1");
				model.addConstr(x_i[g[*vi].id] >= x1_, "Boundary_Object_" + std::to_string(g[*vi].id) + "_Back_ieq1");
				model.addConstr(x_i[g[*vi].id] <= x2_, "Boundary_Object_" + std::to_string(g[*vi].id) + "_Back_ieq2");
				break;
			default:break;
			}
		}
	}
	// Floor Plan Constraints :AREA

	if (floorplan)
	{
		GRBQuadExpr total_area = 0;
		double unuse_area = boundary.size[0] * boundary.size[1];
		for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
			total_area += l_i[g[*vi].id] * w_i[g[*vi].id];
		}
		for (int i = 0; i < num_obstacles; ++i) {
			unuse_area -= obstacles[i].size[0] * obstacles[i].size[1];
		}
		model.addQConstr(total_area == unuse_area, "Area_Constraint_for_FloorPlan");
	}
	
	
	// Objective Function
	// Notice that hyperparameters are the weights of area, size error, position error, adjacency error.
	GRBQuadExpr obj = hyperparameters[0] * boundary.size[0] * boundary.size[1];
	for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
		obj -= hyperparameters[0] * l_i[g[*vi].id] * w_i[g[*vi].id];
		if (!g[*vi].target_size.empty()) {
			obj += hyperparameters[1] * (l_i[g[*vi].id] - g[*vi].target_size[0]) * (l_i[g[*vi].id] - g[*vi].target_size[0]);
			obj += hyperparameters[1] * (w_i[g[*vi].id] - g[*vi].target_size[1]) * (w_i[g[*vi].id] - g[*vi].target_size[1]);
			if (!floorplan)
				obj += hyperparameters[1] * (h_i[g[*vi].id] - g[*vi].target_size[2]) * (h_i[g[*vi].id] - g[*vi].target_size[2]);
		}
		if (!g[*vi].target_pos.empty()) {
			obj += hyperparameters[2] * (x_i[g[*vi].id] - g[*vi].target_pos[0]) * (x_i[g[*vi].id] - g[*vi].target_pos[0]);
			obj += hyperparameters[2] * (y_i[g[*vi].id] - g[*vi].target_pos[1]) * (y_i[g[*vi].id] - g[*vi].target_pos[1]);
			if (!floorplan)
				obj += hyperparameters[2] * (z_i[g[*vi].id] - g[*vi].target_pos[2]) * (z_i[g[*vi].id] - g[*vi].target_pos[2]);
		}
	}
	for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei) {
		VertexDescriptor source = boost::source(*ei, g);
		VertexDescriptor target = boost::target(*ei, g);
		if (g[*ei].distance >= 0) {
			switch (g[*ei].type)
			{
			case LeftOf:
				obj += hyperparameters[3] * (x_i[g[target].id] - l_i[g[target].id] / 2 - x_i[g[source].id] - l_i[g[source].id] / 2 - g[*ei].distance) *
					(x_i[g[target].id] - l_i[g[target].id] / 2 - x_i[g[source].id] - l_i[g[source].id] / 2 - g[*ei].distance);
				break;
			case RightOf:
				obj += hyperparameters[3] * (x_i[g[source].id] - l_i[g[source].id] / 2 - x_i[g[target].id] - l_i[g[target].id] / 2 - g[*ei].distance) *
					(x_i[g[source].id] - l_i[g[source].id] / 2 - x_i[g[target].id] - l_i[g[target].id] / 2 - g[*ei].distance);
				break;
			case Behind:
				obj += hyperparameters[3] * (y_i[g[target].id] - w_i[g[target].id] / 2 - y_i[g[source].id] - w_i[g[source].id] / 2 - g[*ei].distance) *
					(y_i[g[target].id] - w_i[g[target].id] / 2 - y_i[g[source].id] - w_i[g[source].id] / 2 - g[*ei].distance);
				break;
			case FrontOf:
				obj += hyperparameters[3] * (y_i[g[source].id] - w_i[g[source].id] / 2 - y_i[g[target].id] - w_i[g[target].id] / 2 - g[*ei].distance) *
					(y_i[g[source].id] - w_i[g[source].id] / 2 - y_i[g[target].id] - w_i[g[target].id] / 2 - g[*ei].distance);
				break;
			case Under:
				obj += hyperparameters[3] * (z_i[g[target].id] - h_i[g[target].id] / 2 - z_i[g[source].id] - h_i[g[source].id] / 2 - g[*ei].distance) *
					(z_i[g[target].id] - h_i[g[target].id] / 2 - z_i[g[source].id] - h_i[g[source].id] / 2 - g[*ei].distance);
				break;
			case Above:
				obj += hyperparameters[3] * (z_i[g[source].id] - h_i[g[source].id] / 2 - z_i[g[target].id] - h_i[g[target].id] / 2 - g[*ei].distance) *
					(z_i[g[source].id] - h_i[g[source].id] / 2 - z_i[g[target].id] - h_i[g[target].id] / 2 - g[*ei].distance);
				break; 
			default:break;
			}
		}
	}
	model.setObjective(obj, GRB_MINIMIZE);
}

void Solver::optimizeModel()
{
    try {
        model.set(GRB_DoubleParam_TimeLimit, 10);
		if (floorplan)
        	model.set(GRB_DoubleParam_MIPGap, 0.11);
		else
			model.set(GRB_DoubleParam_MIPGap, 0.01);
		model.set(GRB_IntParam_MIPFocus, 1);
        model.set(GRB_IntParam_Method, 2);
        model.set(GRB_DoubleParam_BarConvTol, 1e-4);
        model.set(GRB_IntParam_Cuts, 2);
        model.set(GRB_IntParam_Presolve, 2);
        model.optimize();
        if (model.get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
            model.computeIIS();
            model.write("model.ilp");
            std::cout << "Infeasible constraints written to 'model.ilp'" << std::endl;
        }

        GRBVar* vars = model.getVars();
        int numVars = model.get(GRB_IntAttr_NumVars);
        for (auto i = 0; i < numVars; ++i) {
            std::string varName = vars[i].get(GRB_StringAttr_VarName);
            double varValue = vars[i].get(GRB_DoubleAttr_X);
            std::cout << "Variable " << varName << ": Value = " << varValue << std::endl;
        }
        VertexIterator vi1, vi_end1;
		for (boost::tie(vi1, vi_end1) = boost::vertices(g); vi1 != vi_end1; ++vi1) {
            g[*vi1].pos = { 0, 0, 0 };
            g[*vi1].size = { 0, 0, 0 };
			std::string varName = "x_" + std::to_string(g[*vi1].id);
			double varValue = model.getVarByName(varName).get(GRB_DoubleAttr_X);
			g[*vi1].pos[0] = varValue;

			varName = "y_" + std::to_string(g[*vi1].id);
			varValue = model.getVarByName(varName).get(GRB_DoubleAttr_X);
			g[*vi1].pos[1] = varValue;

			varName = "l_" + std::to_string(g[*vi1].id);
			varValue = model.getVarByName(varName).get(GRB_DoubleAttr_X);
			g[*vi1].size[0] = varValue;

			varName = "w_" + std::to_string(g[*vi1].id);
			varValue = model.getVarByName(varName).get(GRB_DoubleAttr_X);
			g[*vi1].size[1] = varValue;

            if (!floorplan) {
                varName = "z_" + std::to_string(g[*vi1].id);
                varValue = model.getVarByName(varName).get(GRB_DoubleAttr_X);
                g[*vi1].pos[2] = varValue;

                varName = "h_" + std::to_string(g[*vi1].id);
                varValue = model.getVarByName(varName).get(GRB_DoubleAttr_X);
                g[*vi1].size[2] = varValue;
            }
            else {
				g[*vi1].pos[2] = g[*vi1].target_size[2] / 2;
				g[*vi1].size[2] = g[*vi1].target_size[2];
            }
        }
        std::cout << "Value of objective function: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
    }
    catch (GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }
    catch (...) {
        std::cout << "Exception during optimization" << std::endl;
    }
	

    VertexIterator vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
        std::cout << "Vertex " << g[*vi].id << " (" << g[*vi].label << ")" <<
            " Boundary Constraint: " << g[*vi].boundary <<
            //" Priority :" << outputGraph[*vi].priority <<
            " Orientation: " << graphProcessor.orientationnames[g[*vi].orientation] <<
            " Target Position: " << g[*vi].target_pos.size() <<
            " Target Size: " << g[*vi].target_size.size() << std::endl;
    }

    EdgeIterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei) {
        std::cout << "Edge (" << g[boost::source(*ei, g)].label << " -> "
            << g[boost::target(*ei, g)].label << ") "
            << ", Type: " << graphProcessor.edgenames[g[*ei].type] << std::endl;
    }
}

void Solver::saveGraph()
{
    std::ofstream file_in(std::string(ASSETS_DIR) + "/" + "SceneGraph/graph_in.dot");
    if (!file_in.is_open()) {
        std::cerr << "Failed to open file for writing: graph_in.dot" << std::endl;
    } else {
        boost::write_graphviz(file_in, inputGraph, vertex_writer_in<SceneGraph::vertex_descriptor>(inputGraph), 
            edge_writer<SceneGraph::edge_descriptor>(inputGraph));
    }
    
    std::ofstream file_out(std::string(ASSETS_DIR) + "/" + "SceneGraph/graph_out.dot");
    if (!file_out.is_open()) {
        std::cerr << "Failed to open file for writing: graph_out.dot" << std::endl;
    } else {
        boost::write_graphviz(file_out, g, vertex_writer_out<SceneGraph::vertex_descriptor>(g),
            edge_writer<SceneGraph::edge_descriptor>(g));
    }
}

void Solver::solve()
{
	if (inputGraph.m_vertices.empty()) {
		std::cout << "Scene Graph is empty!" << std::endl;
	}
	else {
		addConstraints();
    	optimizeModel();
    	saveGraph();
	}
}

void Solver::readSceneGraph(const std::string& path, float wallwidth)
{
	reset();
	// Read JSON file
	std::ifstream file(path);
    nlohmann::json scene_graph_json;
    file >> scene_graph_json;

	floorplan = scene_graph_json["floorplan"];

    // Parse JSON to set boundary
    boundary.origin_pos = scene_graph_json["boundary"]["origin_pos"].get<std::vector<double>>();
    boundary.size = scene_graph_json["boundary"]["size"].get<std::vector<double>>();
    boundary.points = scene_graph_json["boundary"]["points"].get<std::vector<std::vector<double>>>();
	// calculate orientations
    boundary.Orientations = std::vector<Orientation>(boundary.points.size(), FRONT);
	
	if (!floorplan)
	{
		boundary.origin_pos[0] += wallwidth / 2; boundary.origin_pos[1] += wallwidth / 2;
		boundary.size[0] -= wallwidth; boundary.size[1] -= wallwidth;
		ClipperLib::Path boundaryp;
		ClipperLib::Paths solu;
		for (const auto& point : boundary.points) {
			boundaryp.push_back({ (int)(point[0] * std::pow(10, scalingFactor)), (int)(point[1] * std::pow(10, scalingFactor)) });
		}
		ClipperLib::ClipperOffset co;
		co.AddPath(boundaryp, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
		co.Execute(solu, -(wallwidth / 2) * std::pow(10, scalingFactor));
		std::vector<std::vector<double>> new_points = {};
		for (const auto& point : solu[0]) {
			new_points.push_back({point.X / std::pow(10, scalingFactor), point.Y / std::pow(10, scalingFactor)});
		}
		int idx_ = 0;
		for (auto i = 0; i < new_points.size(); ++i)
		{
			if (std::fabs(std::fabs(new_points[i][0] - boundary.points[0][0]) - wallwidth / 2) < 1e-3 
			&& std::fabs(std::fabs(new_points[i][1] - boundary.points[0][1]) - wallwidth / 2) < 1e-3)
			{
				idx_ = i;
				break;
			}
		}
		for (auto i = 0; i < new_points.size(); ++i)
			boundary.points[i] = new_points[(i + idx_) % new_points.size()];
	}
	
	for (auto i = 0; i < boundary.points.size(); ++i) {
        // Get the current edge
        std::vector<double> p1 = boundary.points[i];
        std::vector<double> p2 = boundary.points[(i + 1) % boundary.points.size()];

        // Calculate direction vector
        std::vector<double> direction = {p2[0] - p1[0], p2[1] - p1[1]};

        // Determine normal vector (rotate direction by 90 degrees)
        std::vector<double> normal = {direction[1], -direction[0]};

        // Determine orientation
        if (normal[0] > 0) {
            boundary.Orientations[i] = RIGHT;
        } else if (normal[0] < 0) {
            boundary.Orientations[i] = LEFT;
        } else if (normal[1] > 0) {
            boundary.Orientations[i] = FRONT;
        } else {
            boundary.Orientations[i] = BACK;
        }
    }

    // set obstacles from boundary and doors/windows
	for (const auto& door : scene_graph_json["doors"]) {
		Doors d;
		d.orientation = door["orientation"];
		d.pos = door["pos"].get<std::vector<double>>();
		d.size = door["size"].get<std::vector<double>>();
		doors.push_back(d);
		Obstacles door_obstacle;
		switch (d.orientation) {
		case FRONT:
			door_obstacle.pos = {d.pos[0], d.pos[1] - d.size[0] / 2, d.pos[2]};
			door_obstacle.size = {d.size[0], d.size[0], d.size[2]};
			break;
		case BACK:
			door_obstacle.pos = {d.pos[0], d.pos[1] + d.size[0] / 2, d.pos[2]};
			door_obstacle.size = {d.size[0], d.size[0], d.size[2]};
			break;
		case LEFT:
			door_obstacle.pos = {d.pos[0] + d.size[1] / 2, d.pos[1], d.pos[2]};
			door_obstacle.size = {d.size[1], d.size[1], d.size[2]};
			break;
		case RIGHT:
			door_obstacle.pos = {d.pos[0] - d.size[1] / 2, d.pos[1], d.pos[2]};
			door_obstacle.size = {d.size[1], d.size[1], d.size[2]};
			break;
		default:break;
		}
		obstacles.push_back(door_obstacle);
	}
	for (const auto& window : scene_graph_json["windows"]) {
		Windows w;
		w.orientation = window["orientation"];
		w.pos = window["pos"].get<std::vector<double>>();
		w.size = window["size"].get<std::vector<double>>();
		windows.push_back(w);
		Obstacles window_obstacle;
		switch (w.orientation) {
		case FRONT:
			window_obstacle.pos = {w.pos[0], w.pos[1] - w.size[0] / 2, w.pos[2]};
			window_obstacle.size = {w.size[0], w.size[0], w.size[2]};
			break;
		case BACK:
			window_obstacle.pos = {w.pos[0], w.pos[1] + w.size[0] / 2, w.pos[2]};
			window_obstacle.size = {w.size[0], w.size[0], w.size[2]};
			break;
		case LEFT:
			window_obstacle.pos = {w.pos[0] + w.size[1] / 2, w.pos[1], w.pos[2]};
			window_obstacle.size = {w.size[1], w.size[1], w.size[2]};
			break;
		case RIGHT:
			window_obstacle.pos = {w.pos[0] - w.size[1] / 2, w.pos[1], w.pos[2]};
			window_obstacle.size = {w.size[1], w.size[1], w.size[2]};
			break;
		default:break;
		}
		obstacles.push_back(window_obstacle);
	}
    for (const auto& obs : scene_graph_json["obstacles"]) {
        Obstacles obstacle;
        obstacle.pos = obs["pos"].get<std::vector<double>>();
        obstacle.size = obs["size"].get<std::vector<double>>();
        obstacles.push_back(obstacle);
    }
	// set obstacles from boundary
	ClipperLib::Path boundary_path, bounding_box;
	bounding_box << ClipperLib::IntPoint(
    	std::round(boundary.origin_pos[0] * std::pow(10, scalingFactor)),
    	std::round(boundary.origin_pos[1] * std::pow(10, scalingFactor)))
		<< ClipperLib::IntPoint(
    	std::round((boundary.origin_pos[0] + boundary.size[0]) * std::pow(10, scalingFactor)),
    	std::round(boundary.origin_pos[1] * std::pow(10, scalingFactor)))
		<< ClipperLib::IntPoint(
		std::round((boundary.origin_pos[0] + boundary.size[0]) * std::pow(10, scalingFactor)),
		std::round((boundary.origin_pos[1] + boundary.size[1]) * std::pow(10, scalingFactor)))
		<< ClipperLib::IntPoint(
		std::round(boundary.origin_pos[0] * std::pow(10, scalingFactor)),
		std::round((boundary.origin_pos[1] + boundary.size[1]) * std::pow(10, scalingFactor)));
	for (const auto& point : boundary.points) {
		boundary_path.push_back({ (int)(point[0] * std::pow(10, scalingFactor)), (int)(point[1] * std::pow(10, scalingFactor)) });
	}
	ClipperLib::Clipper clipper;
    clipper.AddPath(bounding_box, ClipperLib::ptSubject, true);
    clipper.AddPath(boundary_path, ClipperLib::ptClip, true);
    ClipperLib::Paths solution;
    clipper.Execute(ClipperLib::ctDifference, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
	for (const auto& path : solution) {
        Obstacles ob_from_boundary;
		double minX = std::min({path[0].X, path[1].X, path[2].X, path[3].X});
		double maxX = std::max({path[0].X, path[1].X, path[2].X, path[3].X});
		double minY = std::min({path[0].Y, path[1].Y, path[2].Y, path[3].Y});
		double maxY = std::max({path[0].Y, path[1].Y, path[2].Y, path[3].Y});
		ob_from_boundary.pos = {(minX + maxX) / 2.0 / std::pow(10, scalingFactor), (minY + maxY)/ 2.0 / std::pow(10, scalingFactor), boundary.size[2] / 2.0};
		ob_from_boundary.size = {(maxX - minX) / std::pow(10, scalingFactor), (maxY - minY)/ std::pow(10, scalingFactor), boundary.size[2]};
		obstacles.push_back(ob_from_boundary);
    }
    // Parse JSON to set vertices
    for (const auto& vertex : scene_graph_json["vertices"]) {
        VertexProperties vp;
        vp.label = vertex["label"];
        vp.id = vertex["id"];
        vp.boundary = vertex["boundary"];
        vp.on_floor = vertex["on_floor"];
        vp.target_pos = vertex["target_pos"].get<std::vector<double>>();
        vp.target_size = vertex["target_size"].get<std::vector<double>>();
		vp.orientation = vertex["orientation"];
		vp.size_tolerance = {};
		vp.pos_tolerance = {};
		vp.priority = 0;
		if (!vp.target_size.empty()) {
			vp.size_tolerance = vertex["size_tolerance"].get<std::vector<double>>();
		}
		if (!vp.target_pos.empty()) {
			vp.pos_tolerance = vertex["pos_tolerance"].get<std::vector<double>>();
		}
        auto v = add_vertex(vp, inputGraph);
    }

    // Parse JSON to set edges
    for (const auto& edge : scene_graph_json["edges"]) {
        EdgeProperties ep;
        ep.type = edge["type"];
		if (ep.type == AlignWith) {
			ep.align_edge = edge["align_edge"];
			ep.distance = -1;
			ep.closeby_tolerance = {};
		}
		else if (ep.type == CloseBy) {
			ep.closeby_tolerance = edge["closeby_tolerance"].get<std::vector<double>>();
			ep.distance = -1;
			ep.align_edge = -1;
		}
		else {
			ep.distance = edge["distance"];
			ep.align_edge = -1;
			ep.closeby_tolerance = {};
		}
        auto source = vertex(edge["source"], inputGraph);
        auto target = vertex(edge["target"], inputGraph);
        add_edge(source, target, ep, inputGraph);
    }

    g = graphProcessor.process(inputGraph, boundary, obstacles);
	if (floorplan)
    	g = graphProcessor.splitGraph2(g, boundary);
}

void Solver::reset()
{
	inputGraph.clear();
	g.clear();
	boundary = Boundary();
	obstacles.clear();
	doors.clear();
	windows.clear();
}

float Solver::getboundaryMaxSize()
{
	return std::max(boundary.size[0], std::max(boundary.size[1], boundary.size[2]));
}