/**
 * @file Random.hpp
 * @author Prajwal Thakur (you@domain.com)
 * @brief  A class for generating scalar random numbers 
 * */


#pragma once 
#include "RandomType.hpp"
#include <random>
namespace cpproboplan
{   
    template <typename T>
    class crRandomGenerator
    {
    public:
        crRandomGenerator();
        crRandomGenerator(const std::size_t seed);
        ~crRandomGenerator();

        //double randomNumber(doble limitMin = -inf, double limitMax = +inf);

        void reset(); //TODO:
        
        void setSeed(const std::size_t seedValue);

        void setRange(T min, T max);

        bool setDistribution(std::string distributionType);

        double getRandomNumber();
    private:
        bool randNumberNone();
        bool randNumberUniform();
        //double randNumberNormal();

    private:
        // seed value for the random number generator
        std::size_t mSeed;

        // Initial value for the random number generator
        T mMin = std::numeric_limits<T>::max();
        T mMax = std::numeric_limits<T>::max();
        T mMean = std::numeric_limits<T>::infinity();
        crRandomType mType = crRandomType::R_NONE;

        // Specify the generation type.
        std::mt19937 mDefaultGenerator;

        std::uniform_real_distribution<double> mUniformReal;

        // Default random engine generator.
        //std::default_random_engine mDefaultGenerator;   

    };


} 

#include "Random.tpp"