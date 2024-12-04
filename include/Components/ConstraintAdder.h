#pragma once
#include <gurobi_c++.h>
#include <vector>
#include <boost/graph/depth_first_search.hpp>

#include "InputScene.h"
#include "SceneGraph.h"

bool has_path(const SceneGraph& g, VertexDescriptor u, VertexDescriptor v);
bool dfs_check_path(const SceneGraph& g, VertexDescriptor u, VertexDescriptor target, EdgeType required_type, std::vector<bool>& visited);

void ConstraintAdder(GRBModel& model, SceneGraph g, Boundary boundary, std::vector<Obstacles> obstacles,
	std::vector<Doors> doors, std::vector<Windows> windows, std::vector<double> hyperparameters, bool floorplan);