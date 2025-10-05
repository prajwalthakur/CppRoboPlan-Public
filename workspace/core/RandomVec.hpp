/**
 * @file RandomVec.hpp
 * @author Prajwal Thakur (you@domain.com)
 * @brief  A class for generating a vector of  random numbers.
 * */

#pragma once 
#include "core.h"
#include "Random.hpp"
namespace cpproboplan
{   
    template <typename T>
    class crRandVecGenerator
    {
        public:
            // Constructor.
            crRandVecGenerator()=default;
            // Destructor.
            ~crRandVecGenerator()=default;
            crRandVecGenerator(const rplUnSignedInt dim);
            crRandVecGenerator(const rplUnSignedInt dim, const rplCollection<rplUnSignedInt>& seed);
            void setDimension(const rplUnSignedInt dim);
            bool setRanges(const rplCollection<T>& minRanges,const rplCollection<T>& maxRanges);
            bool setSeed(const rplCollection<rplUnSignedInt>&  seed);
            bool setDistribution(const std::string& distName);
            void reset();
            rplCollection<T> getRandomNumbers();
        private:
            rplUnSignedInt mDim{0};
            rplCollection<rplUnSignedInt> mSeed;
            rplCollection<T> mMinRanges;
            rplCollection<T> mMaxRanges;
            rplStlCollection<crRandomGenerator<T>> mRandomVecGenerator;
    };

}

// Include implementation
#include "RandomVec.tpp"