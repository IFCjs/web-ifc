/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#pragma once

#include <glm/glm.hpp>

#include "util.h"
#include "is-inside-mesh.h"
#include "geometry.h"
#include "bvh.h"

namespace fuzzybools
{
    static void doubleClipSingleMesh(Geometry& mesh, BVH& bvh1, BVH& bvh2, Geometry& result)
    {
        for (uint32_t i = 0; i < mesh.numFaces; i++)
        {
            Face tri = mesh.GetFace(i);
            glm::dvec3 a = mesh.GetPoint(tri.i0);
            glm::dvec3 b = mesh.GetPoint(tri.i1);
            glm::dvec3 c = mesh.GetPoint(tri.i2);

            auto aabb = mesh.GetFaceBox(i);

            if (!aabb.intersects(bvh2.box))
            {
                // Why is this commented?

                // when subtracting, if box is outside the second operand, its guaranteed to remain
                result.AddFace(a, b, c);
                //continue;
            }
            else if (!aabb.intersects(bvh1.box))
            {
                // when subtracting, if box is outside the first operand, it won't remain ever
                //continue;
            }

            glm::dvec3 n = computeNormal(a, b, c);

            glm::dvec3 triCenter = (a + b + c) * 1.0 / 3.0;

            auto isInsideTarget = MeshLocation::INSIDE;

            auto isInside1Loc = isInsideMesh(triCenter, n, *bvh1.ptr, bvh1);
            auto isInside2Loc = isInsideMesh(triCenter, n, *bvh2.ptr, bvh2);

            auto isInside1 = isInside1Loc.loc;
            auto isInside2 = isInside2Loc.loc;

            if (isInside1 == MeshLocation::OUTSIDE && isInside2 == MeshLocation::OUTSIDE)
            {
                // both outside, no dice
            }
            if (isInside1 != MeshLocation::BOUNDARY && isInside2 != MeshLocation::BOUNDARY)
            {
                // neither boundary, no dice
            }
            else if (isInside1 == MeshLocation::BOUNDARY && isInside2 == MeshLocation::BOUNDARY)
            {
                // both boundary, no dice if normals are same direction
                auto dot = glm::dot(isInside1Loc.normal, isInside2Loc.normal);

                // since both are on the boundary, and we're sampling the center of the tri, these two faces are coplanar
                // hence we can test dot > 0 to see if normals point the same way
                if (dot < 0)
                {
                    // normals face away from eachother, we can keep this face
                    // furthermore, since the first operand is the first added, we don't flip
                    result.AddFace(a, b, c);
                }
            }
            else
            {
                if (isInside2 == MeshLocation::INSIDE || isInside1 == MeshLocation::OUTSIDE)
                {
                    // inside 2, with subtract, means don't include
                    // outside 1, with subtract, means don't include
                }
                else
                {
                    // boundary or outside 2, and boundary or inside 1, means keep 
                    if (isInside2 == MeshLocation::BOUNDARY && isInside1 == MeshLocation::INSIDE)
                    {
                        // we're taking the face of the second operand, but we must match the winding
                        if (glm::dot(n, isInside2Loc.normal) < 0)
                        {
                            result.AddFace(a, b, c);
                        }
                        else
                        {
                            result.AddFace(b, a, c);
                        }
                    }
                    else if (isInside1 == MeshLocation::BOUNDARY)
                    {
                        // we're taking the face of the second operand, but we must match the winding
                        if (glm::dot(n, isInside1Loc.normal) < 0)
                        {
                            result.AddFace(b, a, c);
                        }
                        else
                        {
                            result.AddFace(a, b, c);
                        }
                    }
                    else
                    {
                        result.AddFace(a, b, c);
                    }
                }
            }
        }
    }

    static void doubleClipSingleMesh2(Geometry& mesh, BVH& bvh1, BVH& bvh2, Geometry& result)
    {
        for (uint32_t i = 0; i < mesh.numFaces; i++)
        {
            Face tri = mesh.GetFace(i);
            glm::dvec3 a = mesh.GetPoint(tri.i0);
            glm::dvec3 b = mesh.GetPoint(tri.i1);
            glm::dvec3 c = mesh.GetPoint(tri.i2);

            glm::dvec3 n = computeNormal(a, b, c);

            auto area = areaOfTriangle(a, b, c);

            glm::dvec3 triCenter = (a + b + c) * 1.0 / 3.0;

            auto isInsideTarget = MeshLocation::INSIDE;

            auto isInside1Loc = isInsideMesh(triCenter, n, *bvh1.ptr, bvh1);
            auto isInside2Loc = isInsideMesh(triCenter, n, *bvh2.ptr, bvh2);

            auto isInside1 = isInside1Loc.loc;
            auto isInside2 = isInside2Loc.loc;

            if (isInside1 == MeshLocation::OUTSIDE && isInside2 == MeshLocation::OUTSIDE)
            {
                // both outside, no dice, should be impossible though
            }
            else if (isInside1 == MeshLocation::INSIDE || isInside2 == MeshLocation::INSIDE)
            {
                // we only keep boundaries, no dice
            }
            else if (isInside1 == MeshLocation::BOUNDARY && isInside2 == MeshLocation::BOUNDARY)
            {
                // both boundary, no dice if normals are opposite direction
                auto dot = glm::dot(isInside1Loc.normal, isInside2Loc.normal);

                // since both are on the boundary, and we're sampling the center of the tri, these two faces are coplanar
                // hence we can test dot < 0 to see if normals point the opposite way
                if (dot > 0)
                {
                    // normals face away from eachother, we can keep this face
                    // furthermore, since the first operand is the first added, we don't flip
                    result.AddFace(a, b, c);
                }
            }
            else if (isInside1 == MeshLocation::BOUNDARY)
            {
                // either is a boundary, keep
                if (glm::dot(n, isInside1Loc.normal) < 0)
                {
                    result.AddFace(b, a, c);
                }
                else
                {
                    result.AddFace(a, b, c);
                }
            }
            else if (isInside2 == MeshLocation::BOUNDARY)
            {
                // either is a boundary, keep
                if (glm::dot(n, isInside2Loc.normal) < 0)
                {
                    result.AddFace(b, a, c);
                }
                else
                {
                    result.AddFace(a, b, c);
                }
            }
            else
            {
                // neither a boundary, neither inside, neither outside, nothing left
            }
        }
    }

    static Geometry clipJoin(Geometry& mesh, BVH bvh1, BVH bvh2)
    {
        Geometry resultingMesh;

        doubleClipSingleMesh2(mesh, bvh1, bvh2, resultingMesh);

        return resultingMesh;
    }

    static Geometry clipSubtract(Geometry& mesh, BVH bvh1, BVH bvh2)
    {

        Geometry resultingMesh;

        doubleClipSingleMesh(mesh, bvh1, bvh2, resultingMesh);

        return resultingMesh;
    }
}