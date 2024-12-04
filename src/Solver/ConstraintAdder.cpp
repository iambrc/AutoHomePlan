#include "Components/ConstraintAdder.h"

void ConstraintAdder(GRBModel& model, SceneGraph g, Boundary boundary, std::vector<Obstacles> obstacles, 
	std::vector<Doors> doors, std::vector<Windows> windows, std::vector<double> hyperparameters, bool floorplan)
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
	// 非常影响效率！！！！！

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

bool dfs_check_path(const SceneGraph& g, VertexDescriptor u, VertexDescriptor target, EdgeType required_type, std::vector<bool>& visited) {
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

bool has_path(const SceneGraph& g, VertexDescriptor start, VertexDescriptor target) {
	std::vector<bool> visited_1(boost::num_vertices(g), false), visited_2(boost::num_vertices(g), false),
		visited_3(boost::num_vertices(g), false), visited_4(boost::num_vertices(g), false),
		visited_5(boost::num_vertices(g), false), visited_6(boost::num_vertices(g), false);
	return (dfs_check_path(g, start, target, LeftOf, visited_1) || dfs_check_path(g, start, target, RightOf, visited_1) || 
		dfs_check_path(g, start, target, FrontOf, visited_3) || dfs_check_path(g, start, target, Behind, visited_4) || 
		dfs_check_path(g, start, target, Above, visited_5) || dfs_check_path(g, start, target, Under, visited_6));
}
