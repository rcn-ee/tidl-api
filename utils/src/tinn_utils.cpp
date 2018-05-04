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
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>
#include <iostream>

#include "util.h"
#include "tinn_utils.h"
#include "tidl_create_params.h"

using namespace tinn::util;

const char* TIDL_LayerString[] =
{
    "Data",
    "Convolution",
    "Pooling",
    "ReLU",
    "PReLU",
    "EltWise",
    "InnerProduct",
    "SoftMax",
    "BatchNorm",
    "Bias",
    "Scale",
    "Deconv2D",
    "Concat",
    "Split",
    "Slice",
    "Crop",
    "Flatten",
    "DropOut",
    "ArgMax",
    "DetectionOutput",
    "Reshape",
};


static bool CreateGraph(const sTIDL_Network_t& net,
                        const std::string& dot_file);

static const char* GetVertexColor(uint32_t layer_type);

bool tinn::util::PrintNetwork(const std::string& network_binary,
                              std::ostream& os)
{
    if (network_binary.empty())
        return false;

    sTIDL_Network_t net;
    bool status = ReadBinary(network_binary,
                             reinterpret_cast<char *>(&net),
                             sizeof(sTIDL_Network_t));
    if (!status)
       return false;

    printf("%3s  %-20s  %3s  %3s  %3s "
           " %3s  %3s  %3s  %3s  %3s  %3s  %3s  %3s %3s "
           " %5s  %5s  %5s  %5s  %5s  %5s  %5s  %5s\n",
            "#", "Name", "gId", "#i", "#o",
            "i0", "i1", "i2", "i3", "i4", "i5", "i6", "i7", "o",
            "#roi", "#ch", "h", "w", "#roi", "#ch", "h", "w");

    for (int i = 0 ; i < net.numLayers; i++)
    {
        printf("%3d, %-20s,",i,
                    TIDL_LayerString[net.TIDLLayers[i].layerType]);
        printf("%3d, %3d ,%3d ,",net.TIDLLayers[i].layersGroupId,
                                 net.TIDLLayers[i].numInBufs,
                                 net.TIDLLayers[i].numOutBufs);

        for (int j = 0; j < net.TIDLLayers[i].numInBufs; j++)
        {
          printf("%3d ,",net.TIDLLayers[i].inData[j].dataId);
        }
        for (int j = (net.TIDLLayers[i].numInBufs > 0 ?
              net.TIDLLayers[i].numInBufs : 0); j < 8; j++)
        {
          printf("  x ,");
        }
        printf("%3d ,",net.TIDLLayers[i].outData[0].dataId);
        for (int j = 0; j < 4; j++)
        {
          printf("%5d ,",net.TIDLLayers[i].inData[0].dimValues[j]);
        }
        for (int j = 0; j < 4; j++)
        {
          printf("%5d ,",net.TIDLLayers[i].outData[0].dimValues[j]);
        }
        printf("\n");
    }

    return true;
}

bool tinn::util::GenerateDotGraphForNetwork(const std::string& network_binary,
                                            const std::string& dot_file)
{
    if (network_binary.empty())
        return false;

    sTIDL_Network_t net;
    bool status = ReadBinary(network_binary,
                             reinterpret_cast<char *>(&net),
                             sizeof(sTIDL_Network_t));
    if (!status)
       return false;

    return CreateGraph(net, dot_file);
}



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

using Edge =  boost::graph_traits<Graph>::edge_descriptor;

template<typename T>
using VertexPropertyMap = typename boost::property_map<T, boost::vertex_attribute_t>::type;

template<typename T>
using EdgePropertyMap = typename boost::property_map<T, boost::edge_attribute_t>::type;

using LayerIdMap = std::map<uint32_t, Graph*>;


bool CreateGraph(const sTIDL_Network_t& net, const std::string& dot_file)
{
    static const char* FILLED = "filled";
    static const char* LABEL  = "label";
    static const char* COLOR  = "color";
    static const char* STYLE  = "style";
    static const char* SHAPE  = "shape";

    Graph tidl(net.numLayers);

    // Add a vertex for each layer
    LayerIdMap layerid_map;
    for (int i = 0 ; i < net.numLayers; i++)
    {
        uint32_t layer_type = net.TIDLLayers[i].layerType;

        // Special handling for input & output data layers:
        // Do not put them in the same subgraph
        if (net.TIDLLayers[i].layerType == TIDL_DataLayer)
        {
            auto V = vertex(i, tidl);
            VertexPropertyMap<Graph> vpm = boost::get(vertex_attribute, tidl);
            vpm[V][LABEL] = TIDL_LayerString[layer_type];
            vpm[V][COLOR] = GetVertexColor(layer_type);
            vpm[V][STYLE] = FILLED;
            vpm[V][SHAPE] = "ellipse";

            continue;
        }

        // Use the layer group ID to create subgraphs
        // Place all layers with the same ID into a single subgraph
        uint32_t layerGroupId = net.TIDLLayers[i].layersGroupId;
        Graph* sub = nullptr;
        if (layerid_map.find(layerGroupId) == layerid_map.end())
        {
            sub = &(tidl.create_subgraph());
            layerid_map[layerGroupId] = sub;
            get_property(*sub, graph_name) = std::string("cluster") +
                                             std::to_string(layerGroupId);
            get_property(*sub, graph_graph_attribute)[LABEL] =
                                    std::to_string(layerGroupId);
            get_property(*sub, graph_graph_attribute)[STYLE] = FILLED;
            get_property(*sub, graph_graph_attribute)["fillcolor"] = "lightgrey";
        }
        else
            sub = layerid_map[layerGroupId];

        auto V = add_vertex(i, *sub);
        VertexPropertyMap<Graph> vpm = boost::get(vertex_attribute, *sub);
        vpm[V][LABEL] = TIDL_LayerString[layer_type];
        vpm[V][COLOR] = GetVertexColor(layer_type);
        vpm[V][STYLE] = FILLED;

    }

    // Add edges based on dataId i.e. there is an edge from layer A -> B
    // iff dataId is in outData for A and inData for B.
    EdgePropertyMap<Graph> ep = boost::get(boost::edge_attribute, tidl);
    for (int i = 0 ; i < net.numLayers; i++)
    {
        if (net.TIDLLayers[i].numOutBufs < 0)
            continue;

        // Create a string to  annotate the edge - num_channels, height, width
        int32_t out_id = net.TIDLLayers[i].outData[0].dataId;
        std::string edge_info =
                    std::to_string(net.TIDLLayers[i].outData[0].dimValues[1])+
                    "x" +
                    std::to_string(net.TIDLLayers[i].outData[0].dimValues[2])+
                    "x" +
                    std::to_string(net.TIDLLayers[i].outData[0].dimValues[3]);

        for (int j = 0; j < net.numLayers; j++)
        {
            if (j == i) continue;

            for (int x = 0; x < net.TIDLLayers[j].numInBufs; x++)
                if (net.TIDLLayers[j].inData[x].dataId == out_id)
                {
                    Edge e = add_edge(i, j, tidl).first;
                    ep[e][LABEL] = edge_info;
                }

        }
    }

    get_property(tidl, graph_name) = "TIDL Network";
    get_property(tidl, graph_vertex_attribute)[SHAPE] = "Mrecord";

    // Generate dot file from boost graph
    std::ofstream dot(dot_file);
    boost::write_graphviz(dot, tidl);

    return true;
}


const char* GetVertexColor(uint32_t layer_type)
{
    static const char* LayerColors[] =
    {
        "lightblue",
        "lightseagreen",
        "lightpink",
        "sienna",
        "lightyellow",
        "palegoldenrod",
        "lightsalmon",
        "lightcyan",
        "wheat",
        "olivedrab",
        "lightskyblue",
        "beige",
        "lavender",
        "linen"

    };

    return LayerColors[layer_type % (sizeof(LayerColors)/sizeof(char *))];
}
