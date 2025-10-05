////////////////////////////////////////////////////////////////////////

template <typename T>
cpproboplan::crRandVecGenerator<T>::crRandVecGenerator(const rplUnSignedInt dim)
{
    mDim = dim;
}

////////////////////////////////////////////////////////////////////////

template <typename T>
cpproboplan::crRandVecGenerator<T>::crRandVecGenerator(const rplUnSignedInt dim, const rplCollection<rplUnSignedInt>& seed)
{
    
    if(dim!= seed.size())
    {
        std::cerr << " ERROR seed size and Dimension doesnt match";
        return;       
    }
    mDim = dim;
    mSeed = seed;
    return;
}

////////////////////////////////////////////////////////////////////////

template <typename T>
void cpproboplan::crRandVecGenerator<T>::setDimension(const rplUnSignedInt dim)
{
    mDim = dim;
}
////////////////////////////////////////////////////////////////////////

template <typename T>
bool cpproboplan::crRandVecGenerator<T>::setRanges(const  rplCollection<T>& minRanges,const  rplCollection<T>& maxRanges)
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

////////////////////////////////////////////////////////////////////////

template <typename T>
bool cpproboplan::crRandVecGenerator<T>::setSeed(const rplCollection<rplUnSignedInt>& seed)
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

////////////////////////////////////////////////////////////////////////

template <typename T>
bool cpproboplan::crRandVecGenerator<T>::setDistribution(const std::string& distributionType)
{
    bool isSuccess = false;
    if(mMinRanges.size() != mDim || mMaxRanges.size() != mDim)
    {
        std::cerr << " ERROR seed size and Dimension doesnt match";
        return isSuccess;
    }
    mRandomVecGenerator.resize(mDim);
    for(rplUnSignedInt i=0;i< mDim ;++i )
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

////////////////////////////////////////////////////////////////////////

template <typename T>
rplCollection<T> cpproboplan::crRandVecGenerator<T>::getRandomNumbers() 
{
    rplCollection<T> randomNumbers;
    randomNumbers.resize(mDim);
    if(mDim==-1)
    {
        std::cerr<<"set dimension,seed and distribution first";
        return randomNumbers;
    }
    randomNumbers.resize(mDim);
    for(rplUnSignedInt i=0; i<mDim; ++i)
    {   
        randomNumbers[i] = mRandomVecGenerator[i].getRandomNumber();
    }
    return randomNumbers;
}

////////////////////////////////////////////////////////////////////////

template <typename T>
void cpproboplan::crRandVecGenerator<T>::reset()
{
    for(rplUnSignedInt i=0; i<mDim; ++i)
    {   
        mRandomVecGenerator[i].reset();
    }
}

