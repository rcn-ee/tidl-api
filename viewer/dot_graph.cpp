/******************************************************************************
 * Copyright (c) 2017-2018, Texas Instruments Incorporated - http://www.ti.com/
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
#include "dot_graph.h"

using Edge   =  boost::graph_traits<Graph>::edge_descriptor;

template<typename T>
using VertexPropertyMap =
              typename boost::property_map<T, boost::vertex_attribute_t>::type;

template<typename T>
using EdgePropertyMap =
              typename boost::property_map<T, boost::edge_attribute_t>::type;

using LayerIdMap = std::map<uint32_t, Graph*>;

static const char* FILLED = "filled";
static const char* LABEL  = "label";
static const char* COLOR  = "color";
static const char* STYLE  = "style";
static const char* SHAPE  = "shape";
static const char* BOLD   = "bold";

static const char* GetVertexColor(uint32_t layer_type);
static std::string PoolingProperties(const sTIDL_PoolingParams_t& p);
static std::string BiasProperties(const sTIDL_BiasParams_t& p);
static std::string ReLUProperties(const sTIDL_ReLUParams_t& p);


DotGraph::DotGraph(const sTIDL_Network_t& net):
                    graph_m(net.numLayers), net_m(net)
{
    AddVertices();
    AddEdges();
    AddMetaData();
}


void DotGraph::AddVertices()
{
    // Add a vertex for each layer
    LayerIdMap layerid_map;
    for (int i = 0 ; i < net_m.numLayers; i++)
    {
        const sTIDL_Layer_t& layer = net_m.TIDLLayers[i];

        // Special handling for input & output data layers:
        // Do not put them in the same subgraph
        if (layer.layerType == TIDL_DataLayer)
        {
            auto V = vertex(i, graph_m);
            VertexPropertyMap<Graph> vpm = boost::get(vertex_attribute, graph_m);
            vpm[V][LABEL] = TIDL_LayerString[layer.layerType];
            vpm[V][COLOR] = GetVertexColor(layer.layerType);
            vpm[V][STYLE] = FILLED;
            vpm[V][SHAPE] = "ellipse";

            continue;
        }

        // Use the layer group ID to create subgraphs
        // Place all layers with the same ID into a single subgraph
        uint32_t layerGroupId = layer.layersGroupId;
        Graph* sub = nullptr;
        if (layerid_map.find(layerGroupId) == layerid_map.end())
        {
            sub = &(graph_m.create_subgraph());
            layerid_map[layerGroupId] = sub;
            get_property(*sub, graph_name) = std::string("cluster") +
                                             std::to_string(layerGroupId);
            get_property(*sub, graph_graph_attribute)[LABEL] =
                                    "Group " + std::to_string(layerGroupId);
            get_property(*sub, graph_graph_attribute)[STYLE] = BOLD;
        }
        else
            sub = layerid_map[layerGroupId];

        auto V = add_vertex(i, *sub);

        AddVertexProperties(V, sub, layer, i);
    }

}

void DotGraph::AddVertexProperties(Vertex& V, Graph* g,
                                   const sTIDL_Layer_t& layer,
                                   int index)
{
    VertexPropertyMap<Graph> vpm = boost::get(vertex_attribute, *g);
    vpm[V][COLOR] = GetVertexColor(layer.layerType);
    vpm[V][STYLE] = BOLD;
    vpm[V]["xlabel"] = std::to_string(index);
    vpm[V][LABEL] = "{";

    switch (layer.layerType)
    {
        case TIDL_ConvolutionLayer:
        {
            vpm[V][LABEL] += TIDL_LayerString[layer.layerType];
            const sTIDL_ConvParams_t& p = layer.layerParams.convParams;
            vpm[V][LABEL] += " " + std::to_string(p.kernelW) + "x" +
                             std::to_string(p.kernelH);

            if (p.enablePooling)
                vpm[V][LABEL] += "|" + PoolingProperties(p.poolParams);

            if (p.enableRelU)
                vpm[V][LABEL] += "|" + ReLUProperties(p.reluParams);

            if (p.enableBias)
                vpm[V][LABEL] += "|" +
                                 std::string(TIDL_LayerString[TIDL_BiasLayer]);

            break;
        }

        case TIDL_PoolingLayer:
        {
            const sTIDL_PoolingParams_t& p = layer.layerParams.poolParams;
            vpm[V][LABEL] += PoolingProperties(p);
            break;
        }

        case TIDL_BiasLayer:
        {
            const sTIDL_BiasParams_t& p = layer.layerParams.biasParams;
            vpm[V][LABEL] += BiasProperties(p);
            break;
        }

        case TIDL_ReLULayer:
        {
            const sTIDL_ReLUParams_t& p = layer.layerParams.reluParams;
            vpm[V][LABEL] += ReLUProperties(p);
            break;
        }

        case TIDL_EltWiseLayer:
        {

            vpm[V][LABEL] += TIDL_LayerString[layer.layerType];
            const sTIDL_EltWiseParams_t& p = layer.layerParams.eltWiseParams;
            if (p.eltWiseType == TIDL_EltWiseProduct)
                vpm[V][LABEL] += " Product";
            else if (p.eltWiseType == TIDL_EltWiseSum)
                vpm[V][LABEL] += " Sum";
            else if (p.eltWiseType == TIDL_EltWiseMax)
                vpm[V][LABEL] += " Max";

            break;
        }

        default:
            vpm[V][LABEL] += TIDL_LayerString[layer.layerType];
            break;
    }

    vpm[V][LABEL] += "}";
}


void DotGraph::AddEdges()
{
    // Add edges based on dataId i.e. there is an edge from layer A -> B
    // iff dataId is in outData for A and inData for B.
    EdgePropertyMap<Graph> ep = boost::get(boost::edge_attribute, graph_m);
    for (int i = 0 ; i < net_m.numLayers; i++)
    {
        const sTIDL_Layer_t& layer = net_m.TIDLLayers[i];

        if (layer.numOutBufs < 0)
            continue;

        // Create a string to  annotate the edge - num_channels, height, width
        const sTIDL_DataParams_t& outData = layer.outData[0];
        int32_t out_id = outData.dataId;
        std::string edge_info = std::to_string(outData.dimValues[1])+ "x" +
                                std::to_string(outData.dimValues[2])+ "x" +
                                std::to_string(outData.dimValues[3]);

        for (int j = 0; j < net_m.numLayers; j++)
        {
            if (j == i) continue;

            for (int x = 0; x < net_m.TIDLLayers[j].numInBufs; x++)
                if (net_m.TIDLLayers[j].inData[x].dataId == out_id)
                {
                    Edge e = add_edge(i, j, graph_m).first;
                    ep[e][LABEL] = edge_info;
                }

        }
    }
}


void DotGraph::AddMetaData()
{
    get_property(graph_m, graph_name) = "TIDL Network";
    get_property(graph_m, graph_vertex_attribute)[SHAPE] = "Mrecord";
    get_property(graph_m, graph_graph_attribute)["fontname"] = "Arial";
    get_property(graph_m, graph_vertex_attribute)["fontname"] = "Arial";
    get_property(graph_m, graph_vertex_attribute)["fontsize"] = "12";
    get_property(graph_m, graph_edge_attribute)["fontname"] = "Arial";
    get_property(graph_m, graph_edge_attribute)["fontsize"] = "10";
}


// Generate dot file from boost graph
void DotGraph::Write(const std::string& filename) const
{
    std::ofstream dot(filename);
    boost::write_graphviz(dot, graph_m);
}

std::string PoolingProperties(const sTIDL_PoolingParams_t& p)
{
    std::string s = TIDL_LayerString[TIDL_PoolingLayer];

    if (p.poolingType == TIDL_MaxPooling)
        s += " Max";
    else if (p.poolingType == TIDL_AveragePooling)
        s += " Average";

    if (p.kernelW != 0 && p.kernelH != 0)
        s+= "\\n" + std::to_string(p.kernelW) + "x" + std::to_string(p.kernelH);

    return s;
}

std::string ReLUProperties(const sTIDL_ReLUParams_t& p)
{
    std::string s;

    if (p.reluType == TIDL_RelU)
        s += "ReLU";
    else if (p.reluType == TIDL_PRelU)
        s += "PReLU";
    else if (p.reluType == TIDL_RelU6)
        s += "ReLU6";

    return s;
}

std::string BiasProperties(const sTIDL_BiasParams_t& p)
{
    std::string s = TIDL_LayerString[TIDL_BiasLayer];
    s += "\\n (BQ: " + std::to_string(p.biasQ) + ")";
    return s;
}



const char* GetVertexColor(uint32_t layer_type)
{
    static const char* LayerColors[] =
    {
        "lightblue",
        "red",
        "darkorange",
        "royalblue",
        "darkgreen",
        "yellow",
        "magenta",
        "darkviolet",
        "brown",
        "darksalmon",
        "violet",
        "darkturquoise",
        "darkseagreen",
        "limegreen"

    };

    return LayerColors[layer_type % (sizeof(LayerColors)/sizeof(char *))];
}


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
