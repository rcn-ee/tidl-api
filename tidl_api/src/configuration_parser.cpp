/******************************************************************************
 * Copyright (c) 2017 Texas Instruments Incorporated - http://www.ti.com/
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

#include <string>
#include <fstream>
#include <iostream>

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
        using qi::lit;
        using qi::lexeme;
        using ascii::char_;
        using qi::_1;

        //TODO: Ignore blank lines and comments
        path %= lexeme[+(char_ - '"')];

        // Discard '"'
        q_path = qi::omit[*char_('"')] >> path >> qi::omit[*char_('"')];

        entry %=
          lit("numFrames")   >> '=' >> int_[ph::ref(x.numFrames) = _1]    |
          lit("preProcType") >> '=' >> int_[ph::ref(x.preProcType) = _1]    |
          lit("inWidth")     >> '=' >> int_[ph::ref(x.inWidth) = _1]   |
          lit("inHeight")    >> '=' >> int_[ph::ref(x.inHeight) = _1]  |
          lit("inNumChannels") >> '=' >> int_[ph::ref(x.inNumChannels) = _1]  |

          lit("inData")     >> "=" >>  q_path[ph::ref(x.inData) = _1]     |
          lit("outData")    >> "=" >> q_path[ph::ref(x.outData) = _1]     |
          lit("netBinFile") >> "=" >> q_path[ph::ref(x.netBinFile) = _1]  |

          lit("paramsBinFile") >> "=" >> q_path[ph::ref(x.paramsBinFile) = _1]
          ;
    }

    qi::rule<Iterator, std::string(), ascii::space_type> path;
    qi::rule<Iterator, std::string(), ascii::space_type> q_path;
    qi::rule<Iterator, ascii::space_type> entry;
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

    while (getline(IFS, str))
    {
        result = phrase_parse(str.cbegin(), str.cend(), G, ascii::space);

        if (!result)
        {
            std::cout << "Parsing failed at: " << str << std::endl;
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
