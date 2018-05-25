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
#include <signal.h>
#include <getopt.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cassert>
#include <string>
#include <cstdlib>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>


#include "tidl_viewer.h"

using namespace tidl;

static void ProcessArgs(int argc, char *argv[], std::string& network_file,
                        bool& do_print, std::string& dot_file);

static void DisplayHelp();
static bool fs_exists(std::string path);

int main(int argc, char *argv[])
{
    // Catch ctrl-c to ensure a clean exit
    signal(SIGABRT, exit);
    signal(SIGTERM, exit);

    // Process arguments
    std::string network_file;
    std::string dot_file;
    bool do_print = false;
    ProcessArgs(argc, argv, network_file, do_print, dot_file);

    bool status = true;

    // Dump network to stdout if requested
    if (do_print)
        status &= util::PrintNetwork(network_file);

    // Generate a dot file
    status &= util::GenerateDotGraphForNetwork(network_file, dot_file);

    // If the dot utility exists, generate a SVG file
    const std::string dot_bin("/usr/bin/dot");
    if (fs_exists(dot_bin))
    {
        std::string command = dot_bin;
        command += " -Tsvg " + dot_file + " -o " + dot_file + ".svg";
        int x = std::system(command.c_str());
        if (x != 0) status = false;

    }

    if (!status)
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}


void ProcessArgs(int argc, char *argv[], std::string& network_file,
                 bool& do_print, std::string& dot_file)
{
    if (argc < 4)
    {
        DisplayHelp();
        exit(EXIT_SUCCESS);
    }

    const struct option long_options[] =
    {
        {"help",         no_argument,       0, 'h'},
        {"dot",          required_argument, 0, 'd'},
        {"print",        no_argument,       0, 'p'},
        {0, 0, 0, 0}
    };

    int option_index = 0;

    while (true)
    {
        int this_option_optind = optind ? optind : 1;
        int c = getopt_long(argc, argv, "-d:ph", long_options, &option_index);

        if (c == -1)
            break;

        switch (c)
        {
            case 1:   network_file = argv[this_option_optind];
                      break;

            case 'd': dot_file = optarg;
                      break;

            case 'p': do_print = true;
                      break;

            case 'h': DisplayHelp();
                      exit(EXIT_SUCCESS);
                      break;

            case '?': // Error in getopt_long
                      exit(EXIT_FAILURE);
                      break;

            default:
                      std::cerr << "Unsupported option: " << c << std::endl;
                      break;
        }
    }

    if (dot_file.empty())
    {
        std::cerr << "ERROR: output dot file not specified." << std::endl;
        DisplayHelp();
        exit(EXIT_FAILURE);
    }
}

#define STRING(S)  XSTRING(S)
#define XSTRING(S) #S
void DisplayHelp()
{
    std::string version = STRING(_BUILD_VER);
    version += ".";
    version += STRING(_BUILD_SHA);

    std::cout << "Usage: tidl_viewer -d <dot file name> <network binary file>\n"
              << "Version: " << version << std::endl
              << "Options:  \n"
                 " -p              Print network layer info\n"
                 " -h              Display this help message\n";
}

bool fs_exists(std::string path)
{
    struct stat statbuf;
    if (stat(path.c_str(), &statbuf) == 0) return true;
    else                                   return false;
}


