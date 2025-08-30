#include "PlannerBase.hpp"

namespace cpproboplan::planner
{

    //-----------------------------------------------------
    
    /**
     * @brief Sets the start time for measuring elapsed duration.
     * @param start The starting time point.
     */
    void PlannerBase::setStartTime(const std::chrono::high_resolution_clock::time_point& start)
    {
        mStartTime = start;
    }

    //-------------------------------------------------------

    /**
     * @brief Calculates the time elapsed since the start time was set.
     * @param currTime The current time point.
     * @return The elapsed time in seconds.
     */
    double PlannerBase::getElapsedTime(const std::chrono::high_resolution_clock::time_point& currTime)
    {
        std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::duration<double>>(currTime-mStartTime);
        return duration.count();
    }

    //-----------------------------------------------------

    /**
     * @brief Discretizes the path between two nodes and checks for collisions.
     * * This function linearly interpolates between the start and goal configurations,
     * creating a series of intermediate configurations. It then calls an external
     * collision checker to verify that the path is collision-free.
     * @param startNode Shared pointer to the starting node.
     * @param goalNode Shared pointer to the goal node.
     * @param model The Pinocchio robot model.
     * @param collisionModel The Pinocchio robot collision model.
     * @param data Pinocchio robot data.
     * @param collisionData Pinocchio robot collision data.
     * @param max_step_size The maximum distance between two consecutive states in the discretized path.
     * @param collision_safety_margin The safety margin for collision checking.
     * @param stopAtFirstCollision A flag to stop checking as soon as the first collision is found.
     * @return True if the entire sub-path is collision-free, false otherwise.
     */
    bool PlannerBase::discretizeAndCheckCollision(plSharedNodePtr startNode, 
        plSharedNodePtr goalNode, 
        pin::Model& model, 
        pin::GeometryModel& collisionModel, 
        pin::Data& data, 
        pin::GeometryData& collisionData, 
        double max_step_size,
        double collision_safety_margin,
        bool stopAtFirstCollision )
        {
            pin::Model::ConfigVectorType qStart = pin::Model::ConfigVectorType::Map(startNode->getState().data() , startNode->getState().size());
            pin::Model::ConfigVectorType qEnd = pin::Model::ConfigVectorType::Map(goalNode->getState().data() , goalNode->getState().size());
            std::vector<pin::Model::ConfigVectorType> qPath;
            bool isSubPathCollisionFree = false;
            while(true)
            {
                qPath.emplace_back(qStart);
                double dist =  (qEnd-qStart).norm();
                if(dist<=max_step_size)
                {
                    qPath.emplace_back(qEnd);
                    break;
                }
                qStart = qStart +  max_step_size*(qEnd-qStart)/dist;
            }

            isSubPathCollisionFree =  cpproboplan::crCheckCollisionAlongPath( model,
                                        collisionModel,
                                        qPath,
                                        data,
                                        collisionData,
                                        collision_safety_margin,
                                        stopAtFirstCollision);

            return isSubPathCollisionFree;
        }
        
    //------------------------------------------------------
    
    /**
     * @brief Reconstructs the path from the final node and populates the result struct.
     * * This function traces the parent pointers from the final node back to the start node
     * to reconstruct the path. It also calculates the total path cost and updates the
     * `plResult` member.
     * @param finalNode A raw pointer to the final node of the path.
     * @param isPathFound A boolean flag indicating if a path was successfully found.
     */
    void PlannerBase::constructResult( plNodeType* finalNode, bool isPathFound)
    {
        std::vector<std::vector<double>> revPath;
        auto node = finalNode;
        while(node != nullptr)
        {
            revPath.emplace_back(node->getState());
            node = node->getParent();
        }

        std::vector<std::vector<double>> path;
        std::size_t  sizePath = revPath.size();
        path.resize(sizePath);
        double cost = 0.0;
        int j=0;
        for(int i = static_cast<int>(sizePath) - 1 ; i>=0 ; --i )
        {
            path[j] = revPath[i];
            
            if(j>0 && j<sizePath)
            {
                cost += cpproboplan::distancemetric::Euclidean::calcDistance(path[j-1],path[j]);
            }
            ++j;
        }
        mResult.path = std::move(path);
        mResult.cost  =  cost;
        mResult.isSuccess = isPathFound;
    }

    //----------------------------------------------------
    
    /**
     * @brief Generates a steered node.
     * * This function modifies the `goalNode`'s state to be at a maximum distance
     * `max_connection_dist` from the `startNode`. If the original distance is
     * less than this maximum, no changes are made.
     * @param startNode The starting node.
     * @param goalNode The node to be steered.
     * @param max_connection_dist The maximum distance to steer.
     */
    void PlannerBase::generateSteerNode(plSharedNodePtr startNode, plSharedNodePtr goalNode, double max_connection_dist)
    {
        Eigen::Map<pin::Model::ConfigVectorType> qStart(startNode->getStateRef().data(), startNode->getStateRef().size());
        Eigen::Map<pin::Model::ConfigVectorType> qEnd(goalNode->getStateRef().data() , goalNode->getStateRef().size());

        double dist =  (qEnd-qStart).norm();
        if(dist<=max_connection_dist)
        {
            return;
        }
        qEnd = qStart + max_connection_dist*(qEnd- qStart)/dist;
    }
    
    //---------------------------------------------------

    /**
     * @brief Post-processes the path by discretizing long segments.
     * * This function iterates through the computed path and inserts new waypoints
     * into any segment that is longer than the `max_connection_dist`. This is
     * useful for ensuring a path is smooth or for subsequent operations that
     * require a fine-grained path.
     * @param max_connection_dist The maximum distance for each path segment.
     */
    void PlannerBase::postProcess(double max_connection_dist)
    {
        std::vector<std::vector<double>>& path = mResult.path;
        int numPathSeg = path.size();
        std::vector<std::vector<double>> newPath;
        bool isSubPathCollisionFree = false;
        auto dist = 0.0;
        int i=0;
        for(; i <numPathSeg-1;++i )
        {
            Eigen::Map<pin::Model::ConfigVectorType> qStart(path[i].data(), path[i].size());
            Eigen::Map<pin::Model::ConfigVectorType> qEnd(path[i+1].data() , path[i+1].size());
            double dist =  (qEnd-qStart).norm();
            if(dist <= max_connection_dist )
            {
                newPath.emplace_back(path[i]);
                continue;
            }
            while(dist > max_connection_dist)
            {
                newPath.emplace_back(std::vector<double>(qStart.data(),qStart.data() + qStart.size()));
                qStart = qStart +  max_connection_dist*(qEnd-qStart)/dist;
                dist =  (qEnd-qStart).norm();
            }
        }
        newPath.emplace_back(path[i]);
        mResult.path = std::move(newPath);
    }
}