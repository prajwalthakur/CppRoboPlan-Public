#pragma once
#include <string>
#include <gtest/gtest.h>

#define rplTest TEST_F
#define rplParameterizeTest TEST_P
#define rplInstantiateTestSuiteData INSTANTIATE_TEST_SUITE_P

// variadic macros
//https://learn.microsoft.com/en-us/cpp/preprocessor/variadic-macros?view=msvc-170

// generic formatter
template <typename... Args>
std::string rplFormat(Args&&... args) {
    std::ostringstream oss;
    (oss << ... << args);
    return oss.str();
}

// logging helpers
template <typename... Args>
inline void rplTestPrint(Args&&... args) {
    GTEST_LOG_(INFO) << rplFormat(std::forward<Args>(args)...);
}

template <typename... Args>
inline void rplTestError(Args&&... args) {
    GTEST_LOG_(ERROR) << rplFormat(std::forward<Args>(args)...);
}

template <typename... Args>
inline void rplTestWarning(Args&&... args) {
    GTEST_LOG_(WARNING) << rplFormat(std::forward<Args>(args)...);
}

class rplTestFixture : public ::testing::Test
{
    public:
        // Constructor
        rplTestFixture()
        {
        
        }
        // Destructor
        ~rplTestFixture()
        {

        }
    protected:
        // Setup for the test fixture.
        void SetUp() override
        {

        }
        // Clear for the test fixture.
        void TearDown() override
        {

        }
        std::string failureMessages;


};

template <typename T>
class rplTestParameterize : public rplTestFixture, public ::testing::WithParamInterface<T>
{
    public:
        rplTestParameterize() = default;
        ~rplTestParameterize() override = default;
    protected:
        void SetUp() override
        {
            rplTestFixture::SetUp();
        }

        void TearDown() override
        {
            rplTestFixture::TearDown();
        }
};
