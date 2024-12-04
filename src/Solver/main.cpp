#include <iostream>
#include "gurobi_c++.h"
#include <boost/version.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
using namespace std;

// 定义顶点属性结构体
struct VertexProperties {
	std::string name;
	int id;
};

// 定义边属性结构体
struct EdgeProperties {
	double weight;
	std::string label;
};

// 使用 Boost 库定义图类型
typedef boost::adjacency_list<
	boost::vecS,
	boost::vecS,
	boost::directedS,
	VertexProperties,
	EdgeProperties
> Graph;

/*
int main()
{
	try {
		GRBEnv env = GRBEnv();
		GRBModel model = GRBModel(env);
		// Create variables
		GRBVar x = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x");
		GRBVar y = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y");
		GRBVar z = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z");
		// Set objective
		GRBQuadExpr obj = x * x + y * y;
		model.setObjective(obj, GRB_MAXIMIZE);
		model.addConstr(x + y + z == 1, "c0");
		model.addQConstr(x + y <= z , "c1");
		model.optimize();
		model.optimize();
		std::cout << model.get(GRB_IntAttr_Status) << std::endl;
		cout << x.get(GRB_StringAttr_VarName) << " "
			<< x.get(GRB_DoubleAttr_X) << endl;
		cout << y.get(GRB_StringAttr_VarName) << " "
			<< y.get(GRB_DoubleAttr_X) << endl;
		cout << z.get(GRB_StringAttr_VarName) << " "
			<< z.get(GRB_DoubleAttr_X) << endl;
		cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (...) {
		cout << "Exception during optimization" << endl;
	}
	std::cout << "Boost version: " << BOOST_LIB_VERSION << std::endl;

	
	Graph g;

	Graph::vertex_descriptor v1 = boost::add_vertex(g);
	g[v1].name = "Vertex 1";
	g[v1].id = 1;

	Graph::vertex_descriptor v2 = boost::add_vertex(g);
	g[v2].name = "Vertex 2";
	g[v2].id = 2;

	Graph::edge_descriptor e1;
	bool inserted;
	boost::tie(e1, inserted) = boost::add_edge(v1, v2, g);
	g[e1].weight = 1.5;
	g[e1].label = "Edge from Vertex 1 to Vertex 2";

	boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
	for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
		std::cout << "Vertex " << g[*vi].id << " (" << g[*vi].name << ")" << std::endl;
	}

	boost::graph_traits<Graph>::edge_iterator ei, ei_end;
	for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei) {
		std::cout << "Edge (" << g[boost::source(*ei, g)].name << " -> "
			<< g[boost::target(*ei, g)].name << ") "
			<< "Weight: " << g[*ei].weight << ", Label: " << g[*ei].label << std::endl;
	}
	return 0;
}
*/