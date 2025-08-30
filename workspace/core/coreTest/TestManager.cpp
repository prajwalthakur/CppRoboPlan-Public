
#include "TestManager.hpp"

#include "gtest/gtest.h"

////////////////////////////////////////////////////////////////////////////////

int rplTestManager::pseudoMain(int argc_, char** argv_, cpproboplan::rplOpt<int> numRun)
{
    instance().mNumRuns = numRun;
    return instance().runTests(argc_, argv_);
}

////////////////////////////////////////////////////////////////////////////////

int rplTestManager::runTests(int argc_, char** argv_)
{
    testing::InitGoogleTest(&argc_, argv_);
    return RUN_ALL_TESTS();
}

////////////////////////////////////////////////////////////////////////////////
