#include "Components/GraphProcessor.h"
#include "Components/ConstraintAdder.h"

#include <boost/graph/graphviz.hpp>
#include <fstream>

std::vector<std::string> show_edges = { "Left of", "Right of", "Front of", "Behind", "Above", "Under", "Close by", "Align with" };
std::vector<std::string> show_orientations = { "up", "down", "left", "right", "front", "back" };

template <class Vertex>
struct vertex_writer_out {
    vertex_writer_out(const SceneGraph& g) : g(g) {}
    void operator()(std::ostream& out, const Vertex& v) const {
        const VertexProperties& vp = g[v];
		double l = !vp.size.empty() ? vp.size[0] : 0.1;
		double w = !vp.size.empty() ? vp.size[1] : 0.1;
        double show_l = !vp.size.empty() ? vp.size[0] : -1;
        double show_w = !vp.size.empty() ? vp.size[1] : -1;
        out << "[label=\"" << vp.label << "\\n"
            << "Size: " << show_l << " " << show_w << "\\n"
            << "Priority: " << vp.priority << "\\n"
            << "Orientation: " << show_orientations[vp.orientation] << "\", "
            << "shape=rect, style=filled, fillcolor=\"lightblue\", "
            << "width=" << l << ", height=" << w << "]";
    }
    const SceneGraph& g;
};
template <class Vertex>
struct vertex_writer_in {
    vertex_writer_in(const SceneGraph& g) : g(g) {}
    void operator()(std::ostream& out, const Vertex& v) const {
        const VertexProperties& vp = g[v];
        double l = !vp.target_size.empty() ? vp.target_size[0] : 0.1;
        double w = !vp.target_size.empty() ? vp.target_size[1] : 0.1;
        double show_l = !vp.target_size.empty() ? vp.target_size[0] : -1;
        double show_w = !vp.target_size.empty() ? vp.target_size[1] : -1;
        out << "[label=\"" << vp.label << "\\n"
            << "Size: " << show_l << " " << show_w << "\\n"
            << "Priority: " << vp.priority << "\\n"
            << "Orientation: " << show_orientations[vp.orientation] << "\", "
            << "shape=rect, style=filled, fillcolor=\"lightblue\", "
            << "width=" << l << ", height=" << w << "]";
    }
    const SceneGraph& g;
};

template <class Edge>
struct edge_writer {
    edge_writer(const SceneGraph& g) : g(g) {}
    void operator()(std::ostream& out, const Edge& e) const {
        const EdgeProperties& ep = g[e];
        std::string color;
        switch (ep.type) {
        case LeftOf: color = "red"; break;
        case RightOf: color = "blue"; break;
        case FrontOf: color = "green"; break;
        case Behind: color = "orange"; break;
        case Above: color = "purple"; break;
        case Under: color = "brown"; break;
        case CloseBy: color = "cyan"; break;
        case AlignWith: color = "magenta"; break;
        }
        out << "[label=\"";
        out << show_edges[ep.type] << "\\n";

        if (ep.type == AlignWith) {
            out << "Align edge: " << ep.align_edge << "\\n";
        }
        else if (ep.type != CloseBy && ep.distance >= 0) {
            out << "Distance: " << ep.distance << "\\n";
        }
        out << "\", color=\"" << color << "\"]";
    }
    const SceneGraph& g;
};

int solve()
{
    bool floorplan = true;
    SceneGraph inputGraph;
    Boundary boundary;
    boundary.origin_pos = { 0, 0, 0 };
    boundary.size = { 1, 1, 1 };
    boundary.points = { {0, 0}, {0.8, 0}, {0.8, 0.3}, {1, 0.3}, {1, 1}, {0.3, 1}, {0.3, 0.6}, {0, 0.6} };
    boundary.Orientations = { BACK, RIGHT, BACK, RIGHT, FRONT, LEFT, FRONT, LEFT };

    Obstacles obstacle1, obstacle2;
    obstacle1.pos = { 0.9, 0.15, 0.5 };
    obstacle1.size = { 0.2, 0.3, 1 };
    obstacle2.pos = { 0.15, 0.8, 0.5 };
    obstacle2.size = { 0.3, 0.4, 1 };
    std::vector<Obstacles> obstacles = { obstacle1, obstacle2 };

    std::vector<Doors> doors = {}; std::vector<Windows> windows = {};

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

    SceneGraph outputGraph = Processor(inputGraph, boundary, obstacles);
	outputGraph = SplitGraph2(outputGraph, boundary);
    
	//SceneGraph outputGraph = Processor(inputGraph, boundary);

    // Set Model
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);
    try {
        //ConstraintAdder(model, outputGraph, boundary, {obstacle}, {door}, {window}, { 1, 1, 1, 1 }, floorplan);
        ConstraintAdder(model, outputGraph, boundary, { obstacle1, obstacle2 }, {}, {}, { 0, 1, 1, 1 }, floorplan);
        model.set(GRB_DoubleParam_TimeLimit, 100);
        model.set(GRB_DoubleParam_MIPGap, 0.95);
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
		for (boost::tie(vi1, vi_end1) = boost::vertices(outputGraph); vi1 != vi_end1; ++vi1) {
            outputGraph[*vi1].pos = { 0, 0, 0 };
            outputGraph[*vi1].size = { 0, 0, 0 };
			std::string varName = "x_" + std::to_string(outputGraph[*vi1].id);
			double varValue = model.getVarByName(varName).get(GRB_DoubleAttr_X);
			outputGraph[*vi1].pos[0] = varValue;

			varName = "y_" + std::to_string(outputGraph[*vi1].id);
			varValue = model.getVarByName(varName).get(GRB_DoubleAttr_X);
			outputGraph[*vi1].pos[1] = varValue;

			varName = "l_" + std::to_string(outputGraph[*vi1].id);
			varValue = model.getVarByName(varName).get(GRB_DoubleAttr_X);
			outputGraph[*vi1].size[0] = varValue;

			varName = "w_" + std::to_string(outputGraph[*vi1].id);
			varValue = model.getVarByName(varName).get(GRB_DoubleAttr_X);
			outputGraph[*vi1].size[1] = varValue;

            if (!floorplan) {
                varName = "z_" + std::to_string(outputGraph[*vi1].id);
                varValue = model.getVarByName(varName).get(GRB_DoubleAttr_X);
                outputGraph[*vi1].pos[2] = varValue;

                varName = "h_" + std::to_string(outputGraph[*vi1].id);
                varValue = model.getVarByName(varName).get(GRB_DoubleAttr_X);
                outputGraph[*vi1].size[2] = varValue;
            }
            else {
				outputGraph[*vi1].pos[2] = outputGraph[*vi1].target_size[2] / 2;
				outputGraph[*vi1].size[2] = outputGraph[*vi1].target_size[2];
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
    for (boost::tie(vi, vi_end) = boost::vertices(outputGraph); vi != vi_end; ++vi) {
        std::cout << "Vertex " << outputGraph[*vi].id << " (" << outputGraph[*vi].label << ")" <<
            " Boundary Constraint: " << outputGraph[*vi].boundary <<
            //" Priority :" << outputGraph[*vi].priority <<
            " Orientation: " << orientationnames[outputGraph[*vi].orientation] <<
            " Target Position: " << outputGraph[*vi].target_pos.size() <<
            " Target Size: " << outputGraph[*vi].target_size.size() << std::endl;
    }

    EdgeIterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(outputGraph); ei != ei_end; ++ei) {
        std::cout << "Edge (" << outputGraph[boost::source(*ei, outputGraph)].label << " -> "
            << outputGraph[boost::target(*ei, outputGraph)].label << ") "
            << ", Type: " << edgenames[outputGraph[*ei].type] << std::endl;
    }
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
        boost::write_graphviz(file_out, outputGraph, vertex_writer_out<SceneGraph::vertex_descriptor>(outputGraph),
            edge_writer<SceneGraph::edge_descriptor>(outputGraph));
    }
    return 0;
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