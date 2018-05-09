/******************************************************************************
 * Copyright (c) 2017-18, Texas Instruments Incorporated - http://www.ti.com/
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of Texas Instruments Incorporated nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *  THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#pragma once

#include <cstdint>
#include "tidl_create_params.h"

#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>

using boost::adjacency_list;
using boost::property;
using boost::subgraph;
using boost::get_property;
using boost::add_vertex;
using boost::add_edge;
using boost::edge_index_t;
using boost::edge_attribute_t;
using boost::vertex_attribute_t;
using boost::vertex_attribute;
using boost::graph_name_t;
using boost::graph_name;
using boost::graph_graph_attribute_t;
using boost::graph_graph_attribute;
using boost::graph_vertex_attribute_t;
using boost::graph_vertex_attribute;
using boost::graph_edge_attribute_t;
using boost::graph_edge_attribute;
using boost::vecS;

using GraphvizAttributes = std::map<std::string, std::string>;



// Boost graph with notes and edges annotated with Dot properties.
// These properties are used by the write_graphviz function.
//
// From https://www.boost.org/doc/libs/1_55_0/libs/graph/doc/subgraph.html
// When creating a subgraph, the underlying graph type is required to have
// vertex_index and edge_index internal properties. Add an edge index property
// to the adjacency list. We do not need to add a vertex index property
// because it is built in to the adjacency_list.
using Graph =
  subgraph<
    adjacency_list<vecS, vecS, boost::directedS,
      property<vertex_attribute_t, GraphvizAttributes>,
      property<edge_index_t,int,property<edge_attribute_t, GraphvizAttributes>>,
      property<graph_name_t, std::string,
       property<graph_graph_attribute_t,  GraphvizAttributes,
        property<graph_vertex_attribute_t, GraphvizAttributes,
         property<graph_edge_attribute_t,   GraphvizAttributes>
      >>>
  >>;

using Vertex =  boost::graph_traits<Graph>::vertex_descriptor;

class DotGraph
{
  public:
    DotGraph(const sTIDL_Network_t& net);
    ~DotGraph() {}

    void Write(const std::string& filename) const;

  private:
    void AddVertices();
    void AddEdges();
    void AddMetaData();
    void AddVertexProperties(Vertex& V, Graph* g, const sTIDL_Layer_t& layer);

    Graph graph_m;
    const sTIDL_Network_t& net_m;
};

extern const char* TIDL_LayerString[];
