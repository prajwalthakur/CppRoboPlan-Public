#include "utils.hpp"
#include "MacrosExpression.hpp"
namespace cpproboplan
{
    bool crCheckCollision(pin::Model& model, 
        pin::GeometryModel& collisionModel, 
        pin::Model::ConfigVectorType& q, 
        pin::Data& data, pin::GeometryData& collisionData, 
        double& distancePadding,
        bool stopAtFirstCollision)
        {   
            bool isCollliding = false;
            for(auto& elem: collisionData.collisionRequests)
            {
                elem.security_margin = distancePadding;
            }
            isCollliding = pin::computeCollisions(model,data,collisionModel,collisionData,q,stopAtFirstCollision);
            return isCollliding;
    }

   
    //////////////////////////////////////////////////////////////

    double crMinDistanceToCollision(pin::Model& model,
        pin::Data& data,
        pin::GeometryModel& collisionModel,
        pin::GeometryData& collisionData,
        pin::Model::ConfigVectorType& q,
        double distancePadding)
        {
            double minDistance = std::numeric_limits<double>::infinity();
            if(collisionModel.collisionPairs.size()==0)
            {
                // no collision pairs
                minDistance = std::numeric_limits<double>::infinity();
            }
            std::size_t minIndex = pin::computeDistances(model, data, collisionModel, collisionData, q); //minIndex: index at which collision pair has minimum distance among all the collision pairs
            minDistance = collisionData.distanceResults[minIndex].min_distance; // get the minimum distance
            return minDistance;
    }
    
    
    ////////////////////////////////////////////////////////////////////////

    bool crCheckCollisionAlongPath(pin::Model & model,
        pin::GeometryModel & collisionModel,
        std::vector<pin::Model::ConfigVectorType> & q_path,
        pin::Data & data,
        pin::GeometryData & collisionData,
        double distancePadding,
        bool stopAtFirstCollision)
        {   
            bool isSubPathCollisionFree = true;
            for(auto& q: q_path)
            {
                if( crCheckCollision(model, collisionModel, q, data, collisionData, distancePadding, stopAtFirstCollision) ==true)
                {
                    return !(isSubPathCollisionFree);
                }
            }
            return isSubPathCollisionFree;
        }
    
    /////////////////////////////////

    std::vector<int> getCollisionGeometryIds(pin::Model & model,pin::GeometryModel& collisionModel,std::string body)
    {
            std::vector<int> bodyCollisionIds;

            auto bodyCollisionId = collisionModel.getGeometryId(body);
            if (bodyCollisionId < collisionModel.ngeoms)
            {
                bodyCollisionIds.push_back(bodyCollisionId);
            }
            else
            {
                auto bodyFrameId = model.getFrameId(body);
                if(bodyFrameId < model.nframes)
                {
                    for(std::size_t id = 0; id< collisionModel.geometryObjects.size(); ++id)
                    {   
                        auto& obj = collisionModel.geometryObjects[id];
                        if(obj.parentFrame == bodyFrameId)
                        {
                            bodyCollisionIds.push_back(id);
                        }
                    }
                }
            }
            return bodyCollisionIds;
    }

    /////////////////////////////////////////

    void crSetCollisions(pin::Model & model,pin::GeometryModel& collisionModel,std::string body1,std::string body2, bool enable)
    {

        std::vector<int> body1CollisionIds = getCollisionGeometryIds(model,collisionModel,body1);
        std::vector<int> body2CollisionIds = getCollisionGeometryIds(model,collisionModel,body2);
        pin::CollisionPair collPair;
        for(int id1:body1CollisionIds)
        {   
            for(int id2:body2CollisionIds)
            {
                collPair = pin::CollisionPair(id1,id2);
            }
            if(enable)
            {
                collisionModel.addCollisionPair(collPair);
            }
            else
            {
                collisionModel.removeCollisionPair(collPair);
            }
        }


    }

    ////////////////////////////

    inline double crConfigurationDistance(pin::Model::ConfigVectorType& q1, pin::Model::ConfigVectorType& q2)
    {
            double dist = (q2-q1).norm();
            return dist;
    }

    //////////////////////////////////////////////////////////////////////////////
    
    double crGetPathLength(std::vector<pin::Model::ConfigVectorType>& q_path)
    {
        double pathLength = 0.0;
        if(q_path.size()<2)
        {
            pathLength = -1;
        }
        for(int i=1;i<q_path.size();++i)
        {
            pathLength += crConfigurationDistance(q_path[i],q_path[i-1]);
        }
        return pathLength;
    }
    
    /////////////////////////////////////////////////////////////////////////////////


    rplspatialPose  crGetCartesianPose(pin::Model& model, std::string targetFrame, rplqState& q, pin::Data& data)
    {

        checkQSize(model,q);
        rplspatialPose spatialPose;
        auto targetFrameId = model.getFrameId(targetFrame);

        pin::forwardKinematics(model,data,q);
        spatialPose = data.oMf[targetFrameId];
        return spatialPose;
    }

    ////////////////////////////////////////////////////////////////////////////////

    rplspatialPoseCollection  crGetCartesianPoses(pin::Model& model, std::string targetFrame,  rplqStateCollection& qVec, pin::Data& data)
    {
        checkQSize(model,qVec[0]);
        rplspatialPoseCollection spatialPoseList;
        auto targetFrameId = model.getFrameId(targetFrame);
        for(int i=0;i<qVec.size();++i)
        {
            pin::forwardKinematics(model, data, qVec[i]);
            spatialPoseList.push_back(data.oMf[targetFrameId]); //o->origin M-> SE(3) f-> target frame
        }
        return spatialPoseList;

    }
 

    pin::Model::ConfigVectorType crGetRandomSample(crRandVecGenerator<double>& mRandomVecGenerator)
    {
        // if(!//isRandomGenInit)
        // {
        //     std::cerr<<"Initialize random number gernerator first, call createRandomGenerator ";
        //     return pin::Model::ConfigVectorType();
        // }
        std::vector<double> randomVec = mRandomVecGenerator.getRandomNumbers();
        Eigen::Map<pin::Model::ConfigVectorType> qtemp(randomVec.data(), randomVec.size());
        return qtemp;
    }
    
    //////////////

    std::pair<pin::Model::ConfigVectorType,bool> crGetRandomCollisionFreeSample(pin::Model& model, 
        pin::Data& data, 
        pin::GeometryModel& collisionModel, 
        pin::GeometryData& collisionData, 
        double& distancePadding, 
        std::size_t& maxTries,
        crRandVecGenerator<double>& randomVecGenerator)
    {
        // if(!//isRandomGenInit)
        // {
        //     std::cerr<<"Initialize random number gernerator first, call createRandomGenerator ";
        //     return std::make_pair(pin::Model::ConfigVectorType(), true);
        // }
        bool isColliding = true;
        pin::Model::ConfigVectorType qTemp;
        for(int i=0; i< maxTries;++i)
        {   if(!isColliding)
            {
                return std::make_pair(qTemp,true);
            }
            qTemp = crGetRandomSample(randomVecGenerator);
            //std::cerr << " checking collisions : try num "<< i << "vec" << qTemp.transpose() << std::endl;
            isColliding = crCheckCollision(model, collisionModel, qTemp, data, collisionData, distancePadding,true);
        }
        
        return std::make_pair(qTemp,false);
    }

    /////////////////////////////////////////////////////////
    
    void crWrapToPi(pin::Model::ConfigVectorType& q)
    {
          
        for(int i = 0; i < q.size(); ++i)
        {
            q[i] = std::atan2(std::sin(q[i]), std::cos(q[i]));
        }

    }

    //////////////////////////////////////////////////////////////////////////

    bool crCheckWithinLimit(pin::Model& model, pin::Model::ConfigVectorType& q)
    {   
        crWrapToPi(q);
        double eps = 1e-9;
        Eigen::Array<bool, Eigen::Dynamic, 1> result =(model.lowerPositionLimit.array() + eps  <= q.array()  && (q.array() <= model.upperPositionLimit.array() -eps));
        //std::cerr << result << std::endl;
        if(result.all())
        {
            return true;
        }
        else{
            return false;
        }
        
    }
    
    void crClamp(pin::Model& model, pin::Model::ConfigVectorType& q)
    {
        crWrapToPi(q);
        double eps = 1e-9;
        Eigen::Array<bool, Eigen::Dynamic, 1> result =(model.lowerPositionLimit.array() + eps  <= q.array()  && (q.array() <= model.upperPositionLimit.array() -eps));
        for(int i=0;i< result.size();++i)
        {
            if(!result[i])
            {
                 q[i] =  std::clamp(q[i], model.lowerPositionLimit[i] + eps , model.upperPositionLimit[i] - eps);
            }
        }
    }
    //////////////////////////////////////////////////////////////////////////

    crRandomGenerator<double> crCreateRandomGenerator(std::string distributionType, const std::size_t seed, const double minRange, const double maxRange)
    {   crRandomGenerator<double> mRandomGenerator;
        mRandomGenerator.setSeed(seed);
        bool isSuccess = mRandomGenerator.setDistribution(distributionType);
        if(isSuccess)
        {
            mRandomGenerator.setRange(minRange,maxRange);
        }
        //isRandomGenInit = isSuccess;
        return mRandomGenerator;
    }

    /////////////////////////////////////////////////////////////////////////////////
    
    crRandVecGenerator<double> crCreateRandVecGenerator(std::string distributionType,
        const std::size_t dim, 
        const std::vector<std::size_t>seed, 
        const std::vector<double>& minRanges, const std::vector<double>& maxRanges,
        double padding)
        {   crRandVecGenerator<double> mRandomVecGenerator;
            mRandomVecGenerator.setDimension(dim);
            mRandomVecGenerator.setSeed(seed);
            bool isSuccess = mRandomVecGenerator.setDistribution(distributionType);
            if(isSuccess)
            {      
                std::vector<double> minR;
                std::vector<double> maxR;
                for(std::size_t i=0;i<dim;++i)
                {
                    minR.push_back(minRanges[i] + padding);
                    maxR.push_back(maxRanges[i] - padding);
                }
                mRandomVecGenerator.setRanges(minR,maxR);
            }  
            //isRandomGenInit = isSuccess;
            return mRandomVecGenerator;
        }

    ////////////

    crRandVecGenerator<double> crCreateRandVecGenerator(std::string distributionType,
        const std::vector<std::size_t> seed, 
        const pin::Model& model, 
        double padding)
        {   
            crRandVecGenerator<double> mRandomVecGenerator;
            const std::size_t dim  = model.lowerPositionLimit.size();
            Eigen::ArrayXd lowerPosLimit = model.lowerPositionLimit.array() + padding;
            Eigen::ArrayXd upperPosLimit = model.upperPositionLimit.array() - padding;
            const std::vector<double> minRanges(lowerPosLimit.data(), lowerPosLimit.data() + lowerPosLimit.size());
            const std::vector<double> maxRanges(upperPosLimit.data(), upperPosLimit.data() + upperPosLimit.size());


            mRandomVecGenerator.setDimension(dim);
            bool isSetSeedSuccess = mRandomVecGenerator.setSeed(seed);
            if(isSetSeedSuccess)
            {
                mRandomVecGenerator.setRanges(minRanges,maxRanges);
            }  
            //isRandomGenInit = isSuccess;
            bool isSuccess = mRandomVecGenerator.setDistribution(distributionType);

            return mRandomVecGenerator;
        }

    std::vector<std::size_t> generateRandomSeed(std::size_t rng_seed, std::size_t dim)
    {
        std::mt19937 gen(rng_seed);     // Mersenne Twister PRNG
        
        std::vector<std::size_t> seeds;
        for (std::size_t i = 0; i < dim; ++i) {
            unsigned int new_seed = gen(); // generate next random number
            seeds.push_back(new_seed);
        }

        return seeds;
    }
    //////////////////////////////////////////////////////////////////
    char getKeyPress()
    {
        char buf = 0;
        struct termios old = {};
        tcgetattr(STDIN_FILENO, &old);
        struct termios newt = old;
        newt.c_lflag &= ~ICANON; // disable buffered input
        newt.c_lflag &= ~ECHO;   // disable echo
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        read(STDIN_FILENO, &buf, 1);
        tcsetattr(STDIN_FILENO, TCSANOW, &old);
        return buf;
    }

    /////////////////////////////
    bool hasNonpositive(std::vector<double> vec)
    {   
        bool ans= false;
        for(auto num:vec)
        {
            if(num<=0)
            {
                return true;
            }
        }
        return ans;
    }
    //////////////////////////////////

    void crWaitForKeyPress(char triggerKey )
    {
        char c;    
        do {
            c = cpproboplan::getKeyPress();
        } while (c != triggerKey); // wait for triggerKey
       
    }
}
    
