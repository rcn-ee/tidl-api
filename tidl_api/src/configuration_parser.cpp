/******************************************************************************
 * Copyright (c) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
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

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/fusion/include/std_pair.hpp>

#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <utility>
#include <map>

#include "configuration.h"

namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
namespace ph = boost::phoenix;

using namespace tidl;

template <typename Iterator>
struct ConfigParser : qi::grammar<Iterator, ascii::space_type>
{
    ConfigParser(Configuration &x) : ConfigParser::base_type(entry)
    {
        using qi::int_;
        using qi::float_;
        using qi::bool_;
        using qi::lit;
        using qi::lexeme;
        using ascii::char_;
        using qi::_1;

        // Rules for parsing layer id assignments: { {int, int}, ... }
        id2group  = '{' >> int_ >> ',' >> int_ >> '}';
        id2groups = '{' >> id2group >> *(qi::lit(',') >> id2group) >> '}';

        // Rules for parsing paths. Discard '"'
        path %= lexeme[+(char_ - '"')];
        q_path = qi::omit[*char_('"')] >> path >> qi::omit[*char_('"')];

        // Rules for parsing subgraph data conversion information
        intvec = int_ >> *int_;
        floatvec = float_ >> *float_;

        // Grammar for parsing configuration file
        entry %=
         lit("layerIndex2LayerGroupId") >> '=' >>
                        id2groups[ph::ref(x.layerIndex2LayerGroupId) = _1]    |
         lit("#")             >>  *(char_) /* discard comments */             |
         lit("numFrames")     >> '=' >> int_[ph::ref(x.numFrames) = _1]       |
         lit("preProcType")   >> '=' >> int_[ph::ref(x.preProcType) = _1]     |
         lit("inWidth")       >> '=' >> int_[ph::ref(x.inWidth) = _1]         |
         lit("inHeight")      >> '=' >> int_[ph::ref(x.inHeight) = _1]        |
         lit("inNumChannels") >> '=' >> int_[ph::ref(x.inNumChannels) = _1]   |
         lit("inData")        >> '=' >> q_path[ph::ref(x.inData) = _1]        |
         lit("outData")       >> '=' >> q_path[ph::ref(x.outData) = _1]       |
         lit("netBinFile")    >> '=' >> q_path[ph::ref(x.netBinFile) = _1]    |
         lit("paramsBinFile") >> '=' >> q_path[ph::ref(x.paramsBinFile) = _1] |
         lit("enableTrace")   >> '=' >> bool_[ph::ref(x.enableOutputTrace)= _1] |
         lit("quantHistoryParam1")   >> '=' >>
                                   int_[ph::ref(x.quantHistoryParam1)= _1] |
         lit("quantHistoryParam2")   >> '=' >>
                                   int_[ph::ref(x.quantHistoryParam2)= _1] |
         lit("quantMargin")   >> '=' >> int_[ph::ref(x.quantMargin)= _1] |
         lit("inConvType")    >> '=' >> intvec[ph::ref(x.inConvType) = _1] |
         lit("inIsSigned")    >> '=' >> intvec[ph::ref(x.inIsSigned) = _1] |
         lit("inScaleF2Q")    >> '=' >> floatvec[ph::ref(x.inScaleF2Q) = _1] |
         lit("inIsNCHW")      >> '=' >> intvec[ph::ref(x.inIsNCHW) = _1] |
         lit("outConvType")   >> '=' >> intvec[ph::ref(x.outConvType) = _1] |
         lit("outIsSigned")   >> '=' >> intvec[ph::ref(x.outIsSigned) = _1] |
         lit("outScaleF2Q")   >> '=' >> floatvec[ph::ref(x.outScaleF2Q) = _1] |
         lit("outIsNCHW")     >> '=' >> intvec[ph::ref(x.outIsNCHW) = _1]
         ;
    }

    qi::rule<Iterator, std::string(), ascii::space_type> path;
    qi::rule<Iterator, std::string(), ascii::space_type> q_path;
    qi::rule<Iterator, ascii::space_type> entry;

    qi::rule<Iterator, std::pair<int, int>(), ascii::space_type> id2group;
    qi::rule<Iterator, std::map<int, int>(), ascii::space_type> id2groups;

    qi::rule<Iterator, std::vector<int>(), ascii::space_type> intvec;
    qi::rule<Iterator, std::vector<float>(), ascii::space_type> floatvec;
};

bool Configuration::ReadFromFile(const std::string &file_name)
{
    std::ifstream IFS(file_name);

    if (!IFS.good())
        return false;

    typedef ConfigParser<std::string::const_iterator> ConfigParser;

    ConfigParser G(*this);
    std::string str;

    bool result = true;

    int line_num = 0;
    while (getline(IFS, str))
    {
        line_num++;

        // Skip lines with whitespace
        auto f = [](unsigned char const c) { return std::isspace(c); };
        if (std::all_of(str.begin(),str.end(), f))
            continue;

        result = phrase_parse(str.cbegin(), str.cend(), G, ascii::space);

        if (!result)
        {
            std::cout << "Parsing failed on line " << line_num
                      << ": " << str << std::endl;
            break;
        }
    }

    IFS.close();

    // If read failed, return false
    if (!result)
        return false;

    // If validate fails, return false
    if (!Validate())
        return false;

    return true;
}

#if 0
--- test.cfg ---
numFrames     = 1
preProcType   = 0
inData        = ../test/testvecs/input/preproc_0_224x224.y
outData       = stats_tool_out.bin
netBinFile    = ../test/testvecs/config/tidl_models/tidl_net_imagenet_jacintonet11v2.bin
paramsBinFile = ../test/testvecs/config/tidl_models/tidl_param_imagenet_jacintonet11v2.bin
inWidth       = 224
inHeight      = 224
inNumChannels = 3

# Enable tracing of output buffers
enableTrace = true

# Override layer group id assignments in the network
layerIndex2LayerGroupId = { {12, 2}, {13, 2}, {14, 2} }
----------------
#endif

#if TEST_PARSING
int main()
{
    Configuration c;
    c.ReadFromFile("test.cfg");

    return 0;
}
#endif
