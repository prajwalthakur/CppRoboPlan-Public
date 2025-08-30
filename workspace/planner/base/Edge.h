#pragma once
#include <iostream>
#include "core/core.h"
#include "Node.h"
namespace  cpproboplan::planner
{   
    class plEdge
    {
        public:
            plEdge()=default;
            plEdge(plNode* nodeFrom, plNode* nodeTo):mnodeFrom(nodeFrom), mNodeTo(nodeTo){};
            ~plEdge()=default;
        private:
            rplwkPtr<plNode> mNodeFrom{nullptr};
            rplwkPtr<plNode> mNodeTo{nullptr};
            double mWeight{1.0};
            double mMinDistFromObs{std::numeric_limits<double>::infinity};
            bool mIsBiDirctional{false};

    };


}
