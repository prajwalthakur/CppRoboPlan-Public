
#pragma once

#include "/root/workspace/src/core/core.h"
#include "/root/workspace/src/core/OnceInstance.h"

class rplTestManager : public crOnceInstance<rplTestManager>
{
public:
    // Constructor.
    rplTestManager() = default;
    // Destructor.
    ~rplTestManager() = default;

public:
    // numRuns - Sets the number of times to run this suite of tests.
    static int pseudoMain(int argc_, char** argv_, cpproboplan::rplOpt<int> numRun = std::nullopt);
    // Wrapper call to run the tests.
    int runTests(int argc_, char** argv_);

private:
    // Number of runs to do in a test.
    cpproboplan::rplOpt<int> mNumRuns;
};
