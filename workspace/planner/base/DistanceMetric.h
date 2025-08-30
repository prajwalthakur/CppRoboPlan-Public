#pragma once
#include <math.h>
#include <vector>

/**
 * @namespace cpproboplan::distancemetric
 * @brief Provides various distance metrics for geometric calculations.
 */
namespace cpproboplan::distancemetric
{
    /**
     * @namespace cpproboplan::distancemetric::Euclidean
     * @brief Contains functions for calculating Euclidean distances.
     */
    namespace Euclidean
    {
        /**
         * @brief Calculates the Euclidean distance between two vectors.
         * * The Euclidean distance is the straight-line distance between two points in Euclidean space.
         * * @tparam T The type of the elements in the vectors (e.g., int, float, double).
         * @param vec1 The first vector.
         * @param vec2 The second vector.
         * @return The Euclidean distance, or -1.0 if the vectors have different sizes.
         */
        template <typename T>
        double calcDistance(const std::vector<T>& vec1, const std::vector<T>& vec2)
        {
            if (vec1.size() != vec2.size()) {
                return -1.0; // or throw an exception
            }

            double ans = 0.0;
            for (std::size_t i = 0; i < vec1.size(); ++i) {
                double diff = static_cast<double>(vec1[i]) - static_cast<double>(vec2[i]);
                ans += diff * diff;
            }
            return std::sqrt(ans);
        }

        /**
         * @brief Calculates the projected Euclidean distance along a single axis.
         * * This function calculates the distance between two vectors by considering only a
         * single specified axis. This is equivalent to finding the length of the projection
         * of the vector connecting the two points onto that axis.
         * * @tparam T The type of the elements in the vectors.
         * @param vec1 The first vector.
         * @param vec2 The second vector.
         * @param axis The index of the axis (dimension) to project onto.
         * @return The projected Euclidean distance, or -1.0 if the vectors have different sizes.
         */
        template <typename T>
        double calcProjDistance(const std::vector<T>& vec1, 
            const std::vector<T>& vec2, int axis)
        {
            double ans =0.0;
            if(vec1.size()!=vec2.size())
                {
                    return -1.0;
                }
            double diff = static_cast<double>(vec1[axis]) - static_cast<double>(vec2[axis]);
            ans += diff * diff;
            return std::sqrt(ans);
        }
    }
}