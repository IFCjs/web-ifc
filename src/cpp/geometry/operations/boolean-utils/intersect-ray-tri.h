/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/*
    2024-03-17
    ==========
    
    This function was rewritten on 2024-03-17 using the following reference as a guide.   
    https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution.html
    Also useful is the site
    https://iquilezles.org/articles/intersectors/
    
    The constants toleranceInsideOutside and toleranceParallelTight are set in eps.h.
    
    The parameter infiniteLength is retained but not used.  It probably could be purged along with references elsewhere.
*/    


#pragma once
#include <glm/glm.hpp>
#include "eps.h"
#include "math.h"
using Vec = glm::dvec3;
namespace fuzzybools
{
    static bool intersect_ray_triangle
    (
        const Vec& origin,
        const Vec& end,
        const Vec& v0,
        const Vec& v1,
        const Vec& v2,
        Vec& hitPosition,
        double& t,
        bool infiniteLength = false
    )
    {
/*
        Compute the direction of the ray from origin to end.
*/
        Vec dir = end - origin;
        Vec dirNormalised = glm::normalize(dir);

/*
        Compute the normal to the plane defined by v0, v1 and v2.
        This normal vector is not normalised.
*/    
        Vec v0v1 = v1 - v0;
        Vec v0v2 = v2 - v0;
        Vec n = glm::cross(v0v1, v0v2);

        double nLength = n.length();	
        double dirLength = dir.length();
        
/*
        Step 0
        ------
    
        Check if the ray is parallel to the plane.  If so, return false because there is no point
        of intersection.
*/   
        double NdotRayDirection = glm::dot(n, dir);

        // Vec n_norm = glm::normalize(n);
        // Vec dir_norm = glm::normalize(dir);
        // double NdotRayDirection = glm::dot(n_norm, dir_norm);

//      if (std::fabs(NdotRayDirection) < toleranceParallelTight * nLength * dirLength)
        if (std::fabs(NdotRayDirection) < toleranceParallelTight)
            return false; 

/*
        Step 1
        ------
    
        Find the point of intersection p of the ray with the plane defined by v0, v1 and v2.

        (a) Compute d, the distance from the origin to the plane.
*/    
        double d = -glm::dot(n, v0);
/*
        (b) Compute t.  The distance from the origin to the point of intersection is t |dir|, where
            |dir| is the length of dir.
*/    
        double d_origin = glm::dot(n, origin);

        t = -(d_origin + d) / NdotRayDirection;   
/*
        (c) Check if the triangle is behind the ray.  If so, return false.
*/    
//      if (t < -toleranceInsideOutside * dirLength) return false;
        if (t < -toleranceInsideOutsideToPlane) return false;
/*
        (d) Compute p, the point of intersection.
*/    
        Vec p = origin + t * dir;
 
/*
        Step 2
        ------
    
        Apply the inside-outside test to decide whether the point of intersection p lies inside
        the triangle defined by v0, v1 and v2.
    
        The edges of the triangle are considered in turn.  For the vertex v0 and the edge defined by
        the vector from v0 to v1, the vectors
            edge0 = (v1 - v0)
        and
            v0p = (p - v0)
        both lie in the plane, so vector 
            c = edge0 x v0p
        is perpendicular to the plane.  If c and n point in opposite directions, then point p lies to the
        right of edge0, and so will lie outside the triangle.  In that case, return false.
    
        Check the other vertices and edges similarly. 
*/
    
        Vec c;
        double cLength = 0.0;
 
        // edge 0
        Vec edge0 = v1 - v0; 
        Vec v0p = p - v0;
        c = glm::cross(edge0, v0p);
        cLength = c.length();
        double valdot = glm::dot(n, c);
//      if (glm::dot(n, c) < -toleranceInsideOutside * nLength * cLength) return false;
        if (valdot < -toleranceInsideOutside) return false;
 
        // edge 1
        Vec edge1 = v2 - v1; 
        Vec v1p = p - v1;
        c = glm::cross(edge1, v1p);
        cLength = c.length();
        valdot = glm::dot(n, c);
//      if (glm::dot(n, c) < -toleranceInsideOutside * nLength * cLength) return false;
        if (valdot < -toleranceInsideOutside) return false;
 
        // edge 2
        Vec edge2 = v0 - v2; 
        Vec v2p = p - v2;
        c = glm::cross(edge2, v2p);
        cLength = c.length();
        valdot = glm::dot(n, c);	
//      if (glm::dot(n, c) < -toleranceInsideOutside * nLength * cLength) return false;
        if (valdot < -toleranceInsideOutside) return false;
    
        hitPosition = p;

        return true; // The ray hits the inside of the triangle.
    }
}
