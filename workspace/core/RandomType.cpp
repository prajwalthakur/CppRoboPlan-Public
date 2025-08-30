#include "RandomType.hpp"
namespace cpproboplan
{
    std::string toString(const crRandomType& type)
    {
        switch(type)
        {
            case crRandomType::R_NONE:
                return "none";
            case crRandomType::R_UNIFORM_REAL:
                return "uniform_real";
            case crRandomType::R_NORMAL:
                return "normal";
            default:
                return "not None";
        }
    }

    crRandomType toRandomType(std::string& str)
    {
        if(str == "none")
        {
            return crRandomType::R_NONE;

        }
        else if(str == "uniform_real")
        {
            return crRandomType::R_UNIFORM_REAL;
        }
        else if(str == "normal")
        {
            return crRandomType::R_NORMAL; 
        }
        else 
        {
            return crRandomType::R_NONE;
        }

    }
    std::unordered_set<std::string> getRandomTypes()
    {
        std::unordered_set<std::string> randomCollection;
        for (int i = 0; i < static_cast<int>(crRandomType::R_COUNT); ++i) 
        {
            auto name = toString(static_cast<crRandomType>(i));
            randomCollection.insert(name);    
        }
        return randomCollection;
    }

}

