#pragma once
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/strong_components.hpp>
#include "SceneGraph.h"
#include "InputScene.h"

#include <map>
#include <random>

extern std::vector<EdgeType> edgetypes;
extern std::vector<std::string> edgenames;
extern std::vector<Orientation> orientations;
extern std::vector<std::string> orientationnames;

struct EdgeTypeFilter {
    EdgeTypeFilter() : g(nullptr), type(LeftOf) {}
    EdgeTypeFilter(const SceneGraph& g, EdgeType type) : g(&g), type(type) {}

    bool operator()(const EdgeDescriptor& e) const {
        return (*g)[e].type == type;
    }

    const SceneGraph* g;
    EdgeType type;
};

bool check_overlap(std::vector<double> r1, std::vector<double> r2);
bool check_inside(std::vector<double> r, std::vector<double> R);
void remove_cycles(SceneGraph& g, EdgeType edge_type);
void remove_position_constraint(SceneGraph& g, const Boundary& boundary, std::vector<Obstacles> obstacles);
SceneGraph Processor(const SceneGraph& inputGraph, const Boundary& boundary, std::vector<Obstacles> obstalces);
Orientation opposite_orientation(Orientation o);
SceneGraph SplitGraph4(const SceneGraph& g, const Boundary& boundary);
SceneGraph SplitGraph2(const SceneGraph& g, const Boundary& boundary);