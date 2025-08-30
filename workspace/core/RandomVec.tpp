namespace cpproboplan
{
    template <typename T>
    crRandVecGenerator<T>::crRandVecGenerator(){}
    
    //////////////
    template <typename T>
    crRandVecGenerator<T>::crRandVecGenerator(const std::size_t& dim)
    {
        mDim = (int)dim;
    }
    
    ////////////////
    template <typename T>
    crRandVecGenerator<T>::crRandVecGenerator(const std::size_t& dim, const std::vector<T>& seed)
    {
        
        if(dim!= seed.size())
        {
            std::cerr << " ERROR seed size and Dimension doesnt match";
            return;       
        }
        mDim = (int)dim;
        mSeed = seed;
        return;
    }
    
    /////////////////
    template <typename T>
    crRandVecGenerator<T>::~crRandVecGenerator(){}
    
    /////////////////
    template <typename T>
    void crRandVecGenerator<T>::setDimension(const std::size_t& dim)
    {
        mDim = dim;
    }
    
    //////////
    template <typename T>
    bool crRandVecGenerator<T>::setSeed(const std::vector<std::size_t>& seed)
    {
        bool isSuccess  = false;
        if(seed.size() != mDim)
        {
            std::cerr << " ERROR seed size and Dimension doesnt match";
            return isSuccess;
        }
        isSuccess = true;
        mSeed = seed;
        return isSuccess;

    }   

    /////////////
    template <typename T>
    bool crRandVecGenerator<T>::setRanges(const std::vector<T>& minRanges,const std::vector<T>& maxRanges)
    {
        bool isSuccess  = false;
        if(minRanges.size() != mDim || maxRanges.size() != mDim)
        {
            std::cerr << " ERROR seed size and Dimension doesnt match";
            return isSuccess;
        }
        isSuccess = true;
        mMinRanges = minRanges;
        mMaxRanges = maxRanges;
        return isSuccess;
    }

    /////////////////////////////////////

    template <typename T>
    bool crRandVecGenerator<T>::setDistribution(std::string distributionType)
    {
        
        if(mSeed.size() != mDim)      mSeed.resize(mDim, 0);
        if(mMinRanges.size() != mDim) mMinRanges.resize(mDim, 0.0);
        if(mMaxRanges.size() != mDim) mMaxRanges.resize(mDim, 1.0);
        if(mRandomVecGenerator.size() != mDim) mRandomVecGenerator.resize(mDim);

        
        bool isSuccess = false;
        for(int i=0;i< static_cast<int>(mDim);++i )
        {
            crRandomGenerator<T> randomGen(mSeed[i]);
            randomGen.setRange(mMinRanges[i],mMaxRanges[i]);
            //randomGen.setRange(0.0,1.0);

            isSuccess = randomGen.setDistribution(distributionType);
            if(!isSuccess)
            {
                std::cerr<< " error generating random vec";
                return isSuccess;
            }
            mRandomVecGenerator[i] = std::move(randomGen);
        }
        isSuccess=true;
        return isSuccess;

    }
    template <typename T>
    std::vector<T> crRandVecGenerator<T>::getRandomNumbers() 
    {
        if(mDim==-1)
        {
            std::cerr<<"set dimension,seed and distribution first";
            return mRandomNumbers;
        }
        mRandomNumbers.resize(mDim);
        for(int i=0;i<mDim;++i)
        {   
            mRandomNumbers[i] = mRandomVecGenerator[i].getRandomNumber();
        }
        return mRandomNumbers;
    }
    template <typename T>
    void crRandVecGenerator<T>::reset()
    {
        for(int i=0;i<mDim;++i)
        {   
            mRandomVecGenerator[i].reset();
        }
    }

}