#include <iostream>
namespace cpproboplan
{   
    template <typename T>
    crRandomGenerator<T>::crRandomGenerator()
    {
        this->setSeed(time(0));
    }
    template <typename T>
    crRandomGenerator<T>::crRandomGenerator(const size_t seed)
    {
        this->setSeed(seed);
    }
    template <typename T>
    crRandomGenerator<T>::~crRandomGenerator()
    {

    }
    template <typename T>
    void crRandomGenerator<T>::setSeed(const std::size_t seed)
    {
        mSeed = seed;
        mDefaultGenerator.seed(mSeed);
    }
    
    template <typename T>
    bool crRandomGenerator<T>::setDistribution(std::string str)
    {   crRandomType type =  toRandomType(str);
        bool isSucess = false;
        switch(type)
        {
            case crRandomType::R_NONE:
                {
                    std::cerr<<"none type distribution, select something else";
                    mType = crRandomType::R_NONE;
                    isSucess = randNumberNone();
                    return isSucess;
                }
            case crRandomType::R_UNIFORM_REAL:
            {
                mType = crRandomType::R_UNIFORM_REAL;
                isSucess = randNumberUniform();
                return isSucess;
            }
            default:
            {

                std::cerr<<"invalid selection";
                return false;
            }

        }
    }
    template <typename T>
    void crRandomGenerator<T>::setRange(T min, T max)
    {
        mMin = min;
        mMax = max;
    }

    template <typename T>
    bool crRandomGenerator<T>::randNumberUniform()
    {   
        if(mMin == std::numeric_limits<double>::infinity() || mMax == std::numeric_limits<double>::infinity())
        {
            return false;
        }
        mMean = (mMin + mMax)/2.0;
        mUniformReal.param(std::uniform_real_distribution<double>::param_type(mMin, mMax));
        return true;

    }
    template <typename T>
    bool crRandomGenerator<T>::randNumberNone()
    {
        if(mMin == std::numeric_limits<double>::infinity() || mMax == std::numeric_limits<double>::infinity())
        {
            return false;
        }
        mMean = (mMin + mMax)/2.0;
        return true;
    }

    template <typename T>
    double crRandomGenerator<T>::getRandomNumber()
    {
        switch(mType)
        {
            case crRandomType::R_UNIFORM_REAL:
                return mUniformReal(mDefaultGenerator);
            case crRandomType::R_NONE:
                return mMean; 
            default:
                return std::numeric_limits<double>::infinity();
        }
    }
    template <typename T>
    void crRandomGenerator<T>::reset()
    {
        
    }

}