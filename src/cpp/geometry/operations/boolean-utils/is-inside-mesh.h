/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/*
    This function was revised on 2024-03-17.  Further review is needed once we have an overview
    of the entire package.
    
    Parameter normal is assumed to be normalised.
    Function computeNormal returns a normalised vector.
    
    The constants toleranceBoundaryPoint and toleranceParallel are defined in eps.h.
*/
    #pragma once

#include <glm/glm.hpp>
#include <iostream>
#include <stack>
#include "intersect-ray-tri.h"
#include "geometry.h"
#include "bvh.h"

using Vec = glm::dvec3;

namespace fuzzybools
{
    enum class MeshLocation
    {
        INSIDE,
        OUTSIDE,
        BOUNDARY
    };

    struct InsideResult
    {
        MeshLocation loc;
        Vec normal;
    };

    inline InsideResult isInsideMesh
    (
        const Vec& pt,
        Vec normal,
        const Geometry& g,
        BVH& bvh
    )
    {
        Vec dir(1.0, 1.1, 1.4);   // From where does this come? // This is an arbitrary direction to evaluate inside/outside
        int winding = 0;

        InsideResult result;
        result.loc = MeshLocation::BOUNDARY;
        result.normal = glm::dvec3(0);
        
        double dirLength = dir.length();

        bool hasResult = bvh.IntersectRay(pt, dir, [&](uint32_t i) -> bool
            {
                Face f = g.GetFace(i);
                const Vec a = g.GetPoint(f.i0);
                const Vec b = g.GetPoint(f.i1);
                const Vec c = g.GetPoint(f.i2);

                Vec intersection;
                double distance;
                bool hasIntersection = intersect_ray_triangle(pt, pt + dir, a, b, c, intersection, distance, true);
                if (hasIntersection)
                {
                    Vec otherNormal = computeNormal(a, b, c);  // normalised
                    double d = glm::dot(otherNormal, dir);
                    double dn = glm::dot(otherNormal, normal);
                    if (std::fabs(distance) < toleranceBoundaryPoint)
                    {
                        if (dn > 1.0 - toleranceParallel)
                        {
/*
                            The normals point in the same direction, which means that the boundary is
                            an inside boundary.
*/
                            result.loc = MeshLocation::BOUNDARY;
                            result.normal = normal;
                            return true;
                        }
                        else if (dn < -1.0 + toleranceParallel)
                        {
/*
                            The normals point in opposite directions, which means that the boundary is
                            an outside boundary.
*/
                            result.loc = MeshLocation::OUTSIDE;
                            result.normal = normal;
                            return true;
                        }
                        else
                        {
                            result.loc = MeshLocation::BOUNDARY;
                            result.normal = otherNormal;
                            return true;
                        }    
                    }

                    // It will never reach the --else-- code
//                  if (true || d >= 0)
                    // if (true || d > toleranceParallelTight)
                    // {
                    winding++;
                    // }
                    // else
                    // {
                    //     winding--;
                    // }
                }

                // Continue to search.
                return false;
            }
        );

        if (hasResult)
        {
            return result;
        }

        result.loc = winding % 2 == 1 ? MeshLocation::INSIDE : MeshLocation::OUTSIDE;

        return result;
    }
}
