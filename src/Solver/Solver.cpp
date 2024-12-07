#include "Components/Solver.h"

#include <boost/graph/graphviz.hpp>
#include <fstream>

std::vector<std::string> show_edges = { "Left of", "Right of", "Front of", "Behind", "Above", "Under", "Close by", "Align with" };
std::vector<std::string> show_orientations = { "up", "down", "left", "right", "front", "back" };

Solver::Solver() : env(), model(env) {
    // Initialize solver-related data if needed
    hyperparameters = {1, 1, 1, 1};
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
		model.addConstr(x_i[g[*vi].id] - l_i[g[*vi].id] / 2 >= boundary.origin_pos[0]);
		model.addConstr(x_i[g[*vi].id] + l_i[g[*vi].id] / 2 <= boundary.origin_pos[0] + boundary.size[0]);
		model.addConstr(y_i[g[*vi].id] - w_i[g[*vi].id] / 2 >= boundary.origin_pos[1]);
		model.addConstr(y_i[g[*vi].id] + w_i[g[*vi].id] / 2 <= boundary.origin_pos[1] + boundary.size[1]);
		if (!floorplan) {
			model.addConstr(z_i[g[*vi].id] - h_i[g[*vi].id] / 2 >= boundary.origin_pos[2]);
			model.addConstr(z_i[g[*vi].id] + h_i[g[*vi].id] / 2 <= boundary.origin_pos[2] + boundary.size[2]);
		}
		if (!g[*vi].pos_tolerance.empty() && !g[*vi].target_pos.empty()) {
			model.addConstr(x_i[g[*vi].id] >= g[*vi].target_pos[0] - g[*vi].pos_tolerance[0]);
			model.addConstr(x_i[g[*vi].id] <= g[*vi].target_pos[0] + g[*vi].pos_tolerance[0]);
			model.addConstr(y_i[g[*vi].id] >= g[*vi].target_pos[1] - g[*vi].pos_tolerance[1]);
			model.addConstr(y_i[g[*vi].id] <= g[*vi].target_pos[1] + g[*vi].pos_tolerance[1]);
			if (!floorplan) {
				model.addConstr(z_i[g[*vi].id] >= g[*vi].target_pos[2] - g[*vi].pos_tolerance[2]);
				model.addConstr(z_i[g[*vi].id] <= g[*vi].target_pos[2] + g[*vi].pos_tolerance[2]);
			}
		}
		if (!g[*vi].size_tolerance.empty() && !g[*vi].target_size.empty()) {
			model.addConstr(l_i[g[*vi].id] >= g[*vi].target_size[0] - g[*vi].size_tolerance[0]);
			model.addConstr(l_i[g[*vi].id] <= g[*vi].target_size[0] + g[*vi].size_tolerance[0]);
			model.addConstr(w_i[g[*vi].id] >= g[*vi].target_size[1] - g[*vi].size_tolerance[1]);
			model.addConstr(w_i[g[*vi].id] <= g[*vi].target_size[1] + g[*vi].size_tolerance[1]);
			if (!floorplan) {
				model.addConstr(h_i[g[*vi].id] >= g[*vi].target_size[2] - g[*vi].size_tolerance[2]);
				model.addConstr(h_i[g[*vi].id] <= g[*vi].target_size[2] + g[*vi].size_tolerance[2]);
			}
		}
	}
	// On floor Constraints
	for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
		if (g[*vi].on_floor && !floorplan) {
			model.addConstr(z_i[g[*vi].id] == boundary.origin_pos[2] + h_i[g[*vi].id] / 2);
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
				model.addConstr(x_i[ids] + l_i[ids] / 2 <= x_i[idt] - l_i[idt] / 2);
			else
			{
				model.addConstr(x_i[ids] + l_i[ids] / 2 == x_i[idt] - l_i[idt] / 2);
				model.addConstr(y_i[ids] + w_i[ids] / 2 >= y_i[idt] - w_i[idt] / 2 + min_adj[1]);
				model.addConstr(y_i[ids] - w_i[ids] / 2 <= y_i[idt] + w_i[idt] / 2 - min_adj[1]);
			}
			break;
		case RightOf:
			if (g[*ei].distance >= 0)
				model.addConstr(x_i[ids] - l_i[ids] / 2 >= x_i[idt] + l_i[idt] / 2);
			else
			{
				model.addConstr(x_i[ids] - l_i[ids] / 2 == x_i[idt] + l_i[idt] / 2);
				model.addConstr(y_i[ids] + w_i[ids] / 2 >= y_i[idt] - w_i[idt] / 2 + min_adj[1]);
				model.addConstr(y_i[ids] - w_i[ids] / 2 <= y_i[idt] + w_i[idt] / 2 - min_adj[1]);
			}
			break;
		case Behind:
			if (g[*ei].distance >= 0)
				model.addConstr(y_i[ids] + w_i[ids] / 2 <= y_i[idt] - w_i[idt] / 2);
			else
			{
				model.addConstr(y_i[ids] + w_i[ids] / 2 == y_i[idt] - w_i[idt] / 2);
				model.addConstr(x_i[ids] + l_i[ids] / 2 >= x_i[idt] - l_i[idt] / 2 + min_adj[0]);
				model.addConstr(x_i[ids] - l_i[ids] / 2 <= x_i[idt] + l_i[idt] / 2 - min_adj[0]);
			}
			break;
		case FrontOf:
			if (g[*ei].distance >= 0)
				model.addConstr(y_i[ids] - w_i[ids] / 2 >= y_i[idt] + w_i[idt] / 2);
			else
			{
				model.addConstr(y_i[ids] - w_i[ids] / 2 == y_i[idt] + w_i[idt] / 2);
				model.addConstr(x_i[ids] + l_i[ids] / 2 >= x_i[idt] - l_i[idt] / 2 + min_adj[0]);
				model.addConstr(x_i[ids] - l_i[ids] / 2 <= x_i[idt] + l_i[idt] / 2 - min_adj[0]);
			}
			break;
		case Under:
			if (g[*ei].distance >= 0)
				model.addConstr(z_i[ids] + h_i[ids] / 2 <= z_i[idt] - h_i[idt] / 2);
			else
				model.addConstr(z_i[ids] + h_i[ids] / 2 == z_i[idt] - h_i[idt] / 2);
			model.addConstr(x_i[ids] - l_i[ids] / 2 <= x_i[idt] - l_i[idt] / 2);
			model.addConstr(x_i[ids] + l_i[ids] / 2 >= x_i[idt] + l_i[idt] / 2);
			model.addConstr(y_i[ids] - w_i[ids] / 2 <= y_i[idt] - w_i[idt] / 2);
			model.addConstr(y_i[ids] + w_i[ids] / 2 >= y_i[idt] + w_i[idt] / 2);
			break;
		case Above:
			if (g[*ei].distance >= 0)
				model.addConstr(z_i[ids] - h_i[ids] / 2 >= z_i[idt] + h_i[idt] / 2);
			else
				model.addConstr(z_i[ids] - h_i[ids] / 2 == z_i[idt] + h_i[idt] / 2);
			model.addConstr(x_i[ids] - l_i[ids] / 2 >= x_i[idt] - l_i[idt] / 2);
			model.addConstr(x_i[ids] + l_i[ids] / 2 <= x_i[idt] + l_i[idt] / 2);
			model.addConstr(y_i[ids] - w_i[ids] / 2 >= y_i[idt] - w_i[idt] / 2);
			model.addConstr(y_i[ids] + w_i[ids] / 2 <= y_i[idt] + w_i[idt] / 2);
			break;
		case CloseBy:
			adjacency[ids][idt] = model.addVar(0, 1, 0, GRB_BINARY);
			model.addConstr(x_i[ids] - l_i[ids] / 2 <= x_i[idt] + l_i[idt] / 2 - min_adj[0] * adjacency[ids][idt]);
			model.addConstr(x_i[ids] + l_i[ids] / 2 >= x_i[idt] - l_i[idt] / 2 + min_adj[0] * adjacency[ids][idt]);
			model.addConstr(y_i[ids] - w_i[ids] / 2 <= y_i[idt] + w_i[idt] / 2 - min_adj[1] * (1 - adjacency[ids][idt]));
			model.addConstr(y_i[ids] + w_i[ids] / 2 >= y_i[idt] - w_i[idt] / 2 + min_adj[1] * (1 - adjacency[ids][idt]));
			break;
		case AlignWith:
			switch (g[*ei].align_edge)
			{
			case 0:
				model.addConstr(y_i[ids] - w_i[ids] / 2 == y_i[idt] - w_i[idt] / 2);
				break;
			case 1:
				model.addConstr(x_i[ids] + l_i[ids] / 2 == x_i[idt] + l_i[idt] / 2);
				break;
			case 2:
				model.addConstr(y_i[ids] + w_i[ids] / 2 == y_i[idt] + w_i[idt] / 2);
				break;
			case 3:
				model.addConstr(x_i[ids] - l_i[ids] / 2 == x_i[idt] - l_i[idt] / 2);
				break;
			case 4:
				model.addConstr(z_i[ids] - h_i[ids] / 2 == z_i[idt] + h_i[idt] / 2);
				break;
			case 5:
				model.addConstr(z_i[ids] + h_i[ids] / 2 == z_i[idt] - h_i[idt] / 2);
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
					M * (1 - sigma_R[g[*vi].id][g[*vj].id]));
				model.addConstr(x_i[g[*vi].id] + l_i[g[*vi].id] / 2 <= x_i[g[*vj].id] - l_i[g[*vj].id] / 2 +
					M * (1 - sigma_L[g[*vi].id][g[*vj].id]));
				model.addConstr(y_i[g[*vi].id] - w_i[g[*vi].id] / 2 >= y_i[g[*vj].id] + w_i[g[*vj].id] / 2 - 
					M * (1 - sigma_F[g[*vi].id][g[*vj].id]));
				model.addConstr(y_i[g[*vi].id] + w_i[g[*vi].id] / 2 <= y_i[g[*vj].id] - w_i[g[*vj].id] / 2 + 
					M * (1 - sigma_B[g[*vi].id][g[*vj].id]));

				if (!floorplan) {
					sigma_U[g[*vi].id][g[*vj].id] = model.addVar(0, 1, 0, GRB_BINARY);
					sigma_D[g[*vi].id][g[*vj].id] = model.addVar(0, 1, 0, GRB_BINARY);
					model.addConstr(z_i[g[*vi].id] - h_i[g[*vi].id] / 2 >= z_i[g[*vj].id] + h_i[g[*vj].id] / 2 -
						M * (1 - sigma_U[g[*vi].id][g[*vj].id]));
					model.addConstr(z_i[g[*vi].id] + h_i[g[*vi].id] / 2 <= z_i[g[*vj].id] - h_i[g[*vj].id] / 2 +
						M * (1 - sigma_D[g[*vi].id][g[*vj].id]));
					model.addConstr(sigma_L[g[*vi].id][g[*vj].id] + sigma_R[g[*vi].id][g[*vj].id] +
						sigma_F[g[*vi].id][g[*vj].id] + sigma_B[g[*vi].id][g[*vj].id] +
						sigma_U[g[*vi].id][g[*vj].id] + sigma_D[g[*vi].id][g[*vj].id] >= 1);
				}
				else {
					model.addConstr(sigma_L[g[*vi].id][g[*vj].id] + sigma_R[g[*vi].id][g[*vj].id] +
						sigma_F[g[*vi].id][g[*vj].id] + sigma_B[g[*vi].id][g[*vj].id] >= 1);
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
					M * (1 - sigma_oL[g[*vi].id][i]));
				sigma_o += sigma_oL[g[*vi].id][i];
			}
			if (boundary.origin_pos[0] + boundary.size[0] / 2 - obstacles[i].pos[0] - obstacles[i].size[0] / 2 > 1e-3)
			{
				sigma_oR[g[*vi].id][i] = model.addVar(0, 1, 0, GRB_BINARY);
				model.addConstr(x_i[g[*vi].id] - l_i[g[*vi].id] / 2 >= obstacles[i].pos[0] + obstacles[i].size[0] / 2 -
					M * (1 - sigma_oR[g[*vi].id][i]));
				sigma_o += sigma_oR[g[*vi].id][i];
			}
			if (obstacles[i].pos[1] - obstacles[i].size[1] / 2 - boundary.origin_pos[1] > 1e-3)
			{
				sigma_oB[g[*vi].id][i] = model.addVar(0, 1, 0, GRB_BINARY);
				model.addConstr(y_i[g[*vi].id] + w_i[g[*vi].id] / 2 <= obstacles[i].pos[1] - obstacles[i].size[1] / 2 +
					M * (1 - sigma_oB[g[*vi].id][i]));
				sigma_o += sigma_oB[g[*vi].id][i];
			}
			if (boundary.origin_pos[1] + boundary.size[1] / 2 - obstacles[i].pos[1] - obstacles[i].size[1] / 2 > 1e-3)
			{
				sigma_oF[g[*vi].id][i] = model.addVar(0, 1, 0, GRB_BINARY);
				model.addConstr(y_i[g[*vi].id] - w_i[g[*vi].id] / 2 >= obstacles[i].pos[1] + obstacles[i].size[1] / 2 -
					M * (1 - sigma_oF[g[*vi].id][i]));
				sigma_o += sigma_oF[g[*vi].id][i];
			}

			if (!floorplan) {
				if (obstacles[i].pos[2] - obstacles[i].size[2] / 2 - boundary.origin_pos[2] > 1e-3)
				{
					sigma_oD[g[*vi].id][i] = model.addVar(0, 1, 0, GRB_BINARY);
					model.addConstr(z_i[g[*vi].id] + h_i[g[*vi].id] / 2 <= obstacles[i].pos[2] - obstacles[i].size[2] / 2 +
											M * (1 - sigma_oD[g[*vi].id][i]));
					sigma_o += sigma_oD[g[*vi].id][i];
				}
				if (boundary.origin_pos[2] + boundary.size[2] / 2 - obstacles[i].pos[2] - obstacles[i].size[2] / 2 > 1e-3)
				{
					sigma_oU[g[*vi].id][i] = model.addVar(0, 1, 0, GRB_BINARY);
					model.addConstr(z_i[g[*vi].id] - h_i[g[*vi].id] / 2 >= obstacles[i].pos[2] + obstacles[i].size[2] / 2 -
						M * (1 - sigma_oU[g[*vi].id][i]));
					sigma_o += sigma_oU[g[*vi].id][i];
				}
			}
			model.addConstr(sigma_o >= 1);	
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
				model.addConstr(x_i[g[*vi].id] - l_i[g[*vi].id] / 2 == x1_);
				// Here we assume that on boundary means at least half of length is on the wall
				model.addConstr(y_i[g[*vi].id] >= y1_);
				model.addConstr(y_i[g[*vi].id] <= y2_);
				break;
			case RIGHT:
				model.addConstr(x_i[g[*vi].id] + l_i[g[*vi].id] / 2 == x1_);
				model.addConstr(y_i[g[*vi].id] >= y1_);
				model.addConstr(y_i[g[*vi].id] <= y2_);
				break;
			case FRONT:
				model.addConstr(y_i[g[*vi].id] + w_i[g[*vi].id] / 2 == y1_);
				model.addConstr(x_i[g[*vi].id] >= x1_);
				model.addConstr(x_i[g[*vi].id] <= x2_);
				break;
			case BACK:
				model.addConstr(y_i[g[*vi].id] - w_i[g[*vi].id] / 2 == y1_);
				model.addConstr(x_i[g[*vi].id] >= x1_);
				model.addConstr(x_i[g[*vi].id] <= x2_);
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
		model.addQConstr(total_area == unuse_area);
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
        model.set(GRB_DoubleParam_TimeLimit, 100);
        model.set(GRB_DoubleParam_MIPGap, 0.05);
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

void Solver::setSceneGraph()
{
    floorplan = true;
    boundary.origin_pos = { 0, 0, 0 };
    boundary.size = { 1, 1, 1 };
    boundary.points = { {0, 0}, {0.8, 0}, {0.8, 0.3}, {1, 0.3}, {1, 1}, {0.3, 1}, {0.3, 0.6}, {0, 0.6} };
    boundary.Orientations = { BACK, RIGHT, BACK, RIGHT, FRONT, LEFT, FRONT, LEFT };

    Obstacles obstacle1, obstacle2;
    obstacle1.pos = { 0.9, 0.15, 0.5 };
    obstacle1.size = { 0.2, 0.3, 1 };
    obstacle2.pos = { 0.15, 0.8, 0.5 };
    obstacle2.size = { 0.3, 0.4, 1 };
    obstacles = { obstacle1, obstacle2 };

    auto v1 = add_vertex(VertexProperties{ "LivingRoom", 0, 0, 0, {}, {0.45, 0.55, 1}, {}, {}, {}, {0.2, 0.2, 0}, LEFT, true }, inputGraph);
    auto v2 = add_vertex(VertexProperties{ "BedRoom", 1, 3, 0, {0.8,0.8,0.5}, {0.4,0.4,1}, {}, {}, {}, {0.15,0.15,0}, LEFT, true }, inputGraph);
    auto v3 = add_vertex(VertexProperties{ "Kitchen", 2, 0, 0, {}, {0.3,0.3,1}, {}, {}, {}, {0.1,0.1,0}, LEFT, true }, inputGraph);
    auto v4 = add_vertex(VertexProperties{ "KidRoom", 3, -1, 0, {}, {0.3, 0.3, 1}, {}, {}, {}, {0.1,0.1,0}, LEFT, true }, inputGraph);
    auto v5 = add_vertex(VertexProperties{ "BathRoom", 4, -1, 0, {}, {0.2, 0.3, 1}, {}, {}, {}, {0.1,0.1,0}, LEFT, true }, inputGraph);
    auto v6 = add_vertex(VertexProperties{ "WorkRoom", 5, 7, 0, {}, {0.4,0.3,1}, {}, {}, {}, {0.2,0.15,0}, LEFT, true }, inputGraph);

    add_edge(v2, v1, EdgeProperties{ -1, {}, -1, FrontOf }, inputGraph);
    add_edge(v3, v1, EdgeProperties{ -1, {}, -1, LeftOf }, inputGraph);
    add_edge(v4, v1, EdgeProperties{ -1, {}, -1, FrontOf }, inputGraph);
    add_edge(v5, v1, EdgeProperties{ -1, {}, -1, RightOf }, inputGraph);

    // Convert Doors/Windows Constraints to Obstacles
    int num_doors = doors.size(), num_windows = windows.size();
    for (int i = 0; i < num_doors + num_windows; ++i) {
        int idx = i;
        if (i < num_doors) {
            Obstacles door_obstacle = { doors[idx].obstacle_pos, doors[idx].obstacle_size };
            obstacles.push_back(door_obstacle);
        }
        else {
            idx -= num_doors;
            Obstacles window_obstacle = { windows[idx].obstacle_pos, windows[idx].obstacle_size };
            obstacles.push_back(window_obstacle);
        }
    }

    g = graphProcessor.process(inputGraph, boundary, obstacles);
	g = graphProcessor.splitGraph2(g, boundary);
}

void Solver::saveGraph()
{
    std::ofstream file_in("../../AutoHomePlan/Assets/SceneGraph/graph_in.dot");
    if (!file_in.is_open()) {
        std::cerr << "Failed to open file for writing: graph_in.dot" << std::endl;
    } else {
        boost::write_graphviz(file_in, inputGraph, vertex_writer_in<SceneGraph::vertex_descriptor>(inputGraph), 
            edge_writer<SceneGraph::edge_descriptor>(inputGraph));
    }
    
    std::ofstream file_out("../../AutoHomePlan/Assets/SceneGraph/graph_out.dot");
    if (!file_out.is_open()) {
        std::cerr << "Failed to open file for writing: graph_out.dot" << std::endl;
    } else {
        boost::write_graphviz(file_out, g, vertex_writer_out<SceneGraph::vertex_descriptor>(g),
            edge_writer<SceneGraph::edge_descriptor>(g));
    }
}

void Solver::solve()
{
    setSceneGraph();
    addConstraints();
    optimizeModel();
    saveGraph();
}

/*
    SceneGraph inputGraph;
    Boundary boundary;
    boundary.origin_pos = { 0, 0, 0 };
    boundary.size = { 1, 1, 1 };
    boundary.points = { {0, 0}, {0.7, 0}, {0.7, 0.3}, {1, 0.3}, {1, 1}, {0, 1} };
    boundary.Orientations = { BACK, RIGHT, BACK, RIGHT, FRONT, LEFT };
    Doors door;
    door.orientation = FRONT;
    door.pos = { 0.4, 0, 0.4 };
    door.size = { 0.2, 0, 0.8 };
    door.obstacle_pos = { 0.4, 0.1, 0.4 };
    door.obstacle_size = { 0.2, 0.2, 0.8 };
    Windows window;
    window.orientation = RIGHT;
    window.pos = { 0, 0.6, 0.5 };
    window.size = { 0, 0.2, 0.6 };
    window.obstacle_pos = { 0.1, 0.6, 0.5 };
    window.obstacle_size = { 0.2, 0.2, 0.6 };
    Obstacles obstacle;
    obstacle.pos = { 0.85, 0.15, 0.5 };
    obstacle.size = { 0.3, 0.3, 1 };

    auto v1 = add_vertex(VertexProperties{ "Bed", 0, 3, 0, {}, {0.4, 0.2, 0.2}, {}, {}, {}, {0.1, 0.1, 0}, LEFT, true }, inputGraph);
    auto v2 = add_vertex(VertexProperties{ "NightStand1", 1, -1, 0, {}, {0.1,0.1,0.2}, {}, {}, {}, {0.02,0.02,0}, LEFT, true }, inputGraph);
    auto v3 = add_vertex(VertexProperties{ "NightStand2", 2, -1, 0, {}, {0.1,0.1,0.2}, {}, {}, {}, {0.02,0.02,0}, LEFT, true }, inputGraph);
    auto v4 = add_vertex(VertexProperties{ "Closer", 3, 4, 0, {}, {0.4, 0.15, 0.7}, {}, {}, {}, {0.05,0.03,0}, BACK, true }, inputGraph);
    auto v5 = add_vertex(VertexProperties{ "Desk", 4, 5, 0, {}, {0.1, 0.2, 0.3}, {}, {}, {}, {0.02,0.02,0}, UP, true }, inputGraph);
    auto v6 = add_vertex(VertexProperties{ "Chair", 5, -1, 0, {}, {0.1,0.1,0.15}, {}, {}, {}, {0.02,0.02,0}, LEFT, true }, inputGraph);
    auto v7 = add_vertex(VertexProperties{ "Airconditioner", 6, 5, 0, {0, 0.6, 0.8}, {0.05, 0.15, 0.05}, {}, {}, {0.05,0.05,0.05}, {0.02,0.02,0.02}, LEFT, false }, inputGraph);
    auto v8 = add_vertex(VertexProperties{ "Cup", 7, -1, 0, {}, {0.02,0.02,0.02}, {}, {}, {}, {0,0,0}, UP, false }, inputGraph);

    add_edge(v2, v1, EdgeProperties{ -1, {}, 1, AlignWith }, inputGraph);
    add_edge(v3, v1, EdgeProperties{ -1, {}, 1, AlignWith }, inputGraph);
    add_edge(v2, v1, EdgeProperties{ 0, {}, -1, FrontOf }, inputGraph);
    add_edge(v3, v1, EdgeProperties{ 0, {}, -1, Behind }, inputGraph);
    add_edge(v4, v1, EdgeProperties{ 0.2, {}, -1, LeftOf }, inputGraph);
    add_edge(v5, v4, EdgeProperties{ 0.2, {}, -1, Behind }, inputGraph);
    add_edge(v6, v5, EdgeProperties{ -1, {0.04, 0.04}, -1, CloseBy }, inputGraph);
    add_edge(v6, v5, EdgeProperties{ 0.05, {}, -1, RightOf }, inputGraph);
    add_edge(v8, v5, EdgeProperties{ 0, {}, -1, Above }, inputGraph);
    add_edge(v8, v5, EdgeProperties{ -1, {}, 4, AlignWith }, inputGraph);
*/

/*
    SceneGraph inputGraph;
    Boundary boundary;
    boundary.origin_pos = { 0, 0, 0 };
    boundary.size = { 1, 1, 1 };
    boundary.points = { {0, 0}, {0.7, 0}, {0.7, 0.3}, {1, 0.3}, {1, 1}, {0.4, 1}, {0.4, 0.7}, {0, 0.7} };
    boundary.Orientations = { BACK, RIGHT, BACK, RIGHT, FRONT, LEFT, FRONT, LEFT };

    Obstacles obstacle1, obstacle2;
    obstacle1.pos = { 0.85, 0.15, 0.5 };
    obstacle1.size = { 0.3, 0.3, 1 };
    obstacle2.pos = { 0.2, 0.85, 0.5 };
    obstacle2.size = { 0.4, 0.3, 1 };

    auto v1 = add_vertex(VertexProperties{ "LivingRoom", 0, 0, 0, {}, {0.5, 0.4, 1}, {}, {}, {}, {0.2, 0.2, 0}, LEFT, true }, inputGraph);
    auto v2 = add_vertex(VertexProperties{ "BedRoom", 1, 3, 0, {0.8,0.8,0.5}, {0.3,0.3,1}, {}, {}, {}, {0.15,0.15,0}, LEFT, true }, inputGraph);
    auto v3 = add_vertex(VertexProperties{ "Kitchen", 2, 1, 0, {}, {0.2,0.3,1}, {}, {}, {}, {0.1,0.15,0}, LEFT, true }, inputGraph);
    auto v4 = add_vertex(VertexProperties{ "KidRoom", 3, -1, 0, {}, {0.25, 0.25, 1}, {}, {}, {}, {0.125,0.125,0}, LEFT, true }, inputGraph);
    auto v5 = add_vertex(VertexProperties{ "BathRoom", 4, -1, 0, {}, {0.2, 0.2, 1}, {}, {}, {}, {0.1,0.1,0}, LEFT, true }, inputGraph);
    auto v6 = add_vertex(VertexProperties{ "WorkRoom", 5, -1, 0, {}, {0.4,0.3,1}, {}, {}, {}, {0.2,0.15,0}, LEFT, true }, inputGraph);

    add_edge(v2, v1, EdgeProperties{ -1, {}, -1, RightOf }, inputGraph);
    add_edge(v3, v1, EdgeProperties{ -1, {}, -1, RightOf }, inputGraph);
    add_edge(v4, v1, EdgeProperties{ -1, {}, -1, FrontOf }, inputGraph);
    add_edge(v6, v1, EdgeProperties{ -1, {}, -1, FrontOf }, inputGraph);
*/