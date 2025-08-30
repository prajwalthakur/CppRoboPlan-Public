#pragma once 
#include <unordered_set>
#include <string>


namespace cpproboplan
{
    enum class crRandomType
        {
            R_NONE = 0,
            R_UNIFORM_REAL = 1,
            R_NORMAL = 2,
            R_COUNT = 3, // mark end of list
        };


    // Return Distribution name according to the type.
    std::string toString(const crRandomType& type);

    // Return Distribution Type according to the name.
    crRandomType toRandomType(std::string& str);

    // Return all the distribution names.
    std::unordered_set<std::string> getRandomTypes();
} 
