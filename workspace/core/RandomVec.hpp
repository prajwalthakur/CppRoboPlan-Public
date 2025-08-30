/**
 * @file RandomVec.hpp
 * @author Prajwal Thakur (you@domain.com)
 * @brief  A class for generating a vector of  random numbers.
 * */

#pragma once 
#include "Random.hpp"
#include <vector>
#include<iostream>
namespace cpproboplan
{   
    template <typename T>
    class crRandVecGenerator
    {
        public:
            crRandVecGenerator();
            crRandVecGenerator(const std::size_t& dim);
            crRandVecGenerator(const std::size_t& dim, const std::vector<T>& seed);
            ~crRandVecGenerator();
            void setDimension(const std::size_t& dim);
            bool setRanges(const std::vector<T>& minRanges,const  std::vector<T>& maxRanges);
            bool setSeed(const std::vector<std::size_t>&  seed);
            bool setDistribution(std::string);
            void reset();
            std::vector<T> getRandomNumbers();
        private:
            int mDim{-1};
            std::vector<std::size_t> mSeed;
            std::vector<crRandomGenerator<T>> mRandomVecGenerator;
            std::vector<T> mMinRanges;
            std::vector<T> mMaxRanges;
            std::vector<T> mRandomNumbers;

    };

}

// Include implementation
#include "RandomVec.tpp"