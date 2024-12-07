#pragma once
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/strong_components.hpp>
#include "SceneGraph.h"
#include "InputScene.h"
#include <map>
#include <random>

struct EdgeTypeFilter {
    EdgeTypeFilter() : g(nullptr), type(LeftOf) {}
    EdgeTypeFilter(const SceneGraph& g, EdgeType type) : g(&g), type(type) {}

    bool operator()(const EdgeDescriptor& e) const {
        return (*g)[e].type == type;
    }

    const SceneGraph* g;
    EdgeType type;
};

class GraphProcessor {
public:
    GraphProcessor();
    ~GraphProcessor();

    SceneGraph process(const SceneGraph& inputGraph, const Boundary& boundary, std::vector<Obstacles> obstacles);
    SceneGraph splitGraph4(const SceneGraph& g, const Boundary& boundary);
    SceneGraph splitGraph2(const SceneGraph& g, const Boundary& boundary);

    std::vector<EdgeType> edgetypes;
    std::vector<std::string> edgenames;
    std::vector<Orientation> orientations;
    std::vector<std::string> orientationnames;

private:
    bool checkOverlap(std::vector<double> r1, std::vector<double> r2);
    bool checkInside(std::vector<double> r, std::vector<double> R);
    void removeCycles(SceneGraph& g, EdgeType edge_type);
    void removePositionConstraint(SceneGraph& g, const Boundary& boundary, std::vector<Obstacles> obstacles);
    Orientation oppositeOrientation(Orientation o);
};