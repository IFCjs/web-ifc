#pragma once

#include <vector>
#include <map>
#include <unordered_map>
#include <set>

#include <glm/glm.hpp>

#pragma warning( push )
#pragma warning( disable : 4267)
#include <CDT.h>
#pragma warning( pop )

#include "geometry.h"
#include "aabb.h"
#include "bvh.h"
#include "util.h"
#include "svg.h"
#include "math.h"
#include "loop-finder.h"
#include "obj-exporter.h"
#include "is-inside-mesh.h"

using Vec2 = glm::dvec2;
using Vec3 = glm::dvec3;
namespace fuzzybools
{

//============================================================================================

    struct PlaneBasis
    {
        Vec3 origin;

        Vec3 up;
        Vec3 left;
        Vec3 right;

        Vec2 project(const Vec3& pt)
        {
            auto relative = pt - origin;
            return Vec2(glm::dot(relative, left), glm::dot(relative, right));
        }
    };

//============================================================================================

    struct ReferencePlane
    {
        size_t planeID;
        size_t pointID;
        size_t lineID;
        Vec2 location;
    };

//============================================================================================

    struct ReferenceLine
    {
        size_t lineID;
        size_t pointID;
        double location;
    };

//============================================================================================
//============================================================================================
/*
    The vector direction is not necessarily a unit vector!  It is the vector pointing
    from the start of the line (identified with origin) to the end.  Thus,
        end = origin + direction
*/    
        struct Line
    {
        size_t id;
        size_t globalID;
        Vec3 origin;
        Vec3 direction;

//============================================================================================

        Line()
        {
            static size_t idcounter = 0;
            idcounter++;
            globalID = idcounter;
        }

//============================================================================================

/*
        Does point a lie on the line defined by vectors origin and direction?

*/    
        bool IsPointOnLine(const Vec3& a) const
        {
/*
            Vector d is the unit vector pointing along the line from origin towards
                origin + direction.
*/
            Vec3 d = glm::normalize(direction);
            Vec3 v = a - origin;
/*
            Drop a perpendicular from point a to the line.  The quantity t
            is the distance from origin along the line to the foot of the perpendicular,
            denoted by p.
*/
            double t = glm::dot(v, d);
            Vec3 p = origin + t * d;
/*
            If point a is sufficently close to point p, then decide that point a
            lies on the line.
*/
            return glm::distance(p, a) < tolerancePointOnLine;
        }

//============================================================================================

/*
        The original version of GetPosOnline seemed to assume that direction is a unit vector.
        This version normalises direction first.
*/        

        double GetPosOnLine(const Vec3& pos) const
        {
            Vec3 unitDirection = glm::normalize(direction);
            return glm::dot(pos - origin, unitDirection);
        }

        Vec3 GetPosOnLine(const double dist) const
        {
            Vec3 unitDirection = glm::normalize(direction);
            return origin + dist * unitDirection;
        }

//============================================================================================

        bool IsCollinear(const Line& other) const
        {
            Vec3 unitDirection = glm::normalize(direction);
            Vec3 unitOtherDirection = glm::normalize(other.direction);
            return (equals(unitOtherDirection, unitDirection, toleranceCollinear) || equals(unitOtherDirection, -unitDirection, toleranceCollinear));
//          return (equals(other.direction,    direction,     toleranceCollinear) || equals(other.direction,    -direction,     toleranceCollinear));
        }
        
//============================================================================================

/*
        The original version of IsEqualTo compared dir and direction.  This version compares
        normalised copies of dir and direction.
*/
        bool IsEqualTo(const Vec3& pos, const Vec3& dir) const
        {
            // check dir
            Vec3 unitDir = glm::normalize(dir);
            Vec3 unitDirection = glm::normalize(direction);
            if (!(equals(unitDir, unitDirection, EPS_SMALL) || equals(unitDir, -unitDirection, EPS_SMALL)))
            {
                return false;
            }

            // check pos
            if (!IsPointOnLine(pos))
            {
                return false;
            }

            return true;
        }

//============================================================================================

        void AddPointToLine(double dist, size_t id)
        {
            // check existing
            for (auto& p : points)
            {
                if (p.second == id) return;
            }

            // add new point
            points.push_back(std::make_pair(dist, id));

            // re-sort all
            std::sort
            (
                points.begin(),
                points.end(),
                [&](const std::pair<double, size_t>& left, const std::pair<double, size_t>& right)
                {
                    return left.first < right.first;
                }
            );
        }

//============================================================================================

        std::vector<std::pair<size_t, size_t>> GetSegments() const
        {
            std::vector<std::pair<size_t, size_t>> retval;

            for (size_t i = 1; i < points.size(); i++)
            {
                retval.push_back(std::make_pair(points[i - 1].second, points[i].second));
            }

            return retval;
        }

        std::vector<std::pair<double, size_t>> points;

        std::vector<ReferencePlane> planes;
    };

//============================================================================================
//============================================================================================

    struct Point
    {
        size_t id;
        size_t globalID;
        Vec3 location3D;

        Point()
        {
            static size_t idcounter = 0;
            idcounter++;
            globalID = idcounter;
        }

//============================================================================================

        bool operator==(const Vec3& pt)
        {
            return equals(location3D, pt, toleranceVectorEquality);
        }

        std::vector<ReferenceLine> lines;
        std::vector<ReferencePlane> planes;
    };

//============================================================================================
//============================================================================================

    struct Plane
    {
        size_t id;
        size_t globalID;
        double distance;
        Vec3 normal;

        std::vector<Line> lines;
        AABB aabb;

//============================================================================================

        void AddPoint(const Vec3& pt)
        {
            aabb.merge(pt);
        }

//============================================================================================

        Plane()
        {
            static size_t idcounter = 0;
            idcounter++;
            globalID = idcounter;
        }

//============================================================================================

        double round(double input)
        {
            input = std::fabs(input) < EPS_BIG ? 0.0 : input;
            input = std::fabs(input) < (1.0 - EPS_BIG) ? input :
                    input > 0.0 ? 1.0 : -1.0;
            return input;
        }

//============================================================================================

        Vec3 round(Vec3 in)
        {
            in.x = round(in.x);
            in.y = round(in.y);
            in.z = round(in.z);

            return in;
        }

//============================================================================================

        Vec3 GetDirection(Vec3 a, Vec3 b)
        {
            auto dir = b - a;
            return glm::normalize(dir);
        }

//============================================================================================

        std::pair<size_t, bool> AddLine(const Point& a, const Point& b)
        {
            Vec3 pos = a.location3D;
            Vec3 dir = GetDirection(pos, b.location3D);

            auto lineId = AddLine(pos, dir);

            if (!lines[lineId.first].IsPointOnLine(a.location3D))
            {
                if (messages) { printf("bad point in AddLine\n"); }
            }
            if (!lines[lineId.first].IsPointOnLine(b.location3D))
            {
                if (messages) { printf("bad point in AddLine\n"); }
            }

            if (!aabb.contains(a.location3D))
            {
                if (messages) { printf("bad points in AddLine\n"); }
            }
            if (!aabb.contains(b.location3D))
            {
                if (messages) { printf("bad points in AddLine\n"); }
            }

            lines[lineId.first].AddPointToLine(lines[lineId.first].GetPosOnLine(a.location3D), a.id);
            lines[lineId.first].AddPointToLine(lines[lineId.first].GetPosOnLine(b.location3D), b.id);

            return lineId;
        }

//============================================================================================

        std::pair<size_t, bool> AddLine(const Vec3& pos, const Vec3& dir)
        {
            for (auto& line : lines)
            {
                if (line.IsEqualTo(pos, dir))
                {
                    return { line.id, false };
                }
            }

            Line l;
            l.id = lines.size();
            l.origin = pos;
//          l.direction = dir;
            Vec3 temp = glm::normalize(dir);
            l.direction = temp;

            lines.push_back(l);

            return { l.id, true };
        }

//============================================================================================

        void RemoveLastLine()
        {
            lines.pop_back();
        }

//============================================================================================

        bool IsEqualTo(const Vec3& n, double d)
        {
            return (equals(normal,  n, toleranceVectorEquality) && equals(distance,  d, toleranceScalarEquality)) ||
                   (equals(normal, -n, toleranceVectorEquality) && equals(distance, -d, toleranceScalarEquality));
        }

//============================================================================================

        glm::dvec2 GetPosOnPlane(const glm::dvec3& pos)
        {
            return {};
        }

//============================================================================================

        bool HasOverlap(const std::pair<size_t, size_t>& A, const std::pair<size_t, size_t>& B)
        {
            return (A.first == B.first || A.first == B.second || A.second == B.first || A.second == B.second);
        }

//============================================================================================

        void PutPointOnLines(Point& p)
        {
            for (auto& l : lines)
            {
                if (l.IsPointOnLine(p.location3D))
                {
                    ReferenceLine ref;
                    ref.pointID = p.id;
                    ref.lineID = l.id;
                    ref.location = l.GetPosOnLine(p.location3D);
                    p.lines.push_back(ref);
                }
            }
        }

//============================================================================================

/*
        Normal is assumed to be normalised.
*/
        bool IsPointOnPlane(const glm::dvec3& pos)
        {
            double d = glm::dot(normal, pos);
            double posLength = pos.length();
//          return equals(distance, d, toleranceVectorEquality * posLength);
            return equals(distance, d, toleranceVectorEquality);
        }

//============================================================================================

        PlaneBasis MakeBasis()
        {
            glm::dvec3 origin = normal * distance;
            glm::dvec3 up = normal;

            glm::dvec3 worldUp = glm::dvec3(0, 1, 0);
            glm::dvec3 worldRight = glm::dvec3(1, 0, 0);

            bool normalIsUp = equals(up, worldUp, EPS_SMALL) || equals(-up, worldUp, EPS_SMALL);
            glm::dvec3 left = normalIsUp ? glm::cross(up, worldRight) : glm::cross(up, worldUp);
            glm::dvec3 right = glm::cross(left, up);

            PlaneBasis basis;

            basis.origin = origin;
            basis.up = glm::normalize(up);
            basis.left = glm::normalize(left);
            basis.right = glm::normalize(right);

            return basis;
        }
    };

//============================================================================================
//============================================================================================

    struct Triangle
    {
        size_t id;

        size_t a;
        size_t b;
        size_t c;

//============================================================================================

        void Flip()
        {
            auto temp = a;
            a = b;
            b = temp;
        }

//============================================================================================

        bool HasPoint(size_t p)
        {
            return a == p || b == p || c == p;
        }

//============================================================================================

        bool IsNeighbour(Triangle& t)
        {
            return HasPoint(t.a) || HasPoint(t.b) || HasPoint(t.c);
        }

//============================================================================================

        bool SamePoints(Triangle& other)
        {
            return other.HasPoint(a) && other.HasPoint(b) && other.HasPoint(c);
        }

//============================================================================================

        size_t GetNotShared(Triangle& other)
        {
            if (!other.HasPoint(a))
            {
                return a;
            }
            else if (!other.HasPoint(b))
            {
                return b;
            }
            else if (!other.HasPoint(c))
            {
                return c;
            }

            // throw std::exception("Missing common point in GetNotShared");
        }
    };

//============================================================================================
//============================================================================================

    struct SegmentSet
    {
        std::vector<std::pair<size_t, size_t>> segments;
        std::vector<Triangle> triangles;
        std::map<std::pair<size_t, size_t>, size_t> segmentCounts;
        std::vector<size_t> irrelevantFaces;

        std::map<size_t, std::vector<std::pair<size_t, size_t>>> planeSegments;
        std::map<size_t, std::map<std::pair<size_t, size_t>, size_t>> planeSegmentCounts;

//============================================================================================

        void AddSegment(size_t planeId, size_t a, size_t b)
        {
            if (a == b)
            {
                if (messages) { printf("a == b in AddSegment\n"); }
                return;
            }

            auto seg = a < b ? std::make_pair(a, b) : std::make_pair(b, a);

            segments.emplace_back(seg);
            segmentCounts[seg]++;

            planeSegments[planeId].emplace_back(seg);
            planeSegmentCounts[planeId][seg]++;
        }

//============================================================================================

        void AddFace(size_t planeId, size_t a, size_t b, size_t c)
        {
            AddSegment(planeId, a, b);
            AddSegment(planeId, b, c);
            AddSegment(planeId, c, a);

            Triangle t;
            t.id = triangles.size();
            t.a = a;
            t.b = b;
            t.c = c;

            triangles.push_back(t);
        }

//============================================================================================

        std::vector<size_t> GetTrianglesWithPoint(size_t p)
        {
            std::vector<size_t> returnTriangles;

            for (auto& t : triangles)
            {
                if (t.HasPoint(p))
                {
                    returnTriangles.push_back(t.id);
                }
            }

            return returnTriangles;
        }

//============================================================================================

        std::vector<size_t> GetTrianglesWithEdge(size_t a, size_t b)
        {
            std::vector<size_t> returnTriangles;

            for (auto& t : triangles)
            {
                if (t.HasPoint(a) && t.HasPoint(b))
                {
                    returnTriangles.push_back(t.id);
                }
            }

            return returnTriangles;
        }
		
//============================================================================================

        std::vector<std::pair<std::pair<size_t, size_t>, std::vector<size_t>>> GetNeighbourTriangles(Triangle& triangle)
        {
            std::vector<std::pair<std::pair<size_t, size_t>, std::vector<size_t>>> returnTriangles;

            {
                auto tris = GetTrianglesWithEdge(triangle.a, triangle.b);
                returnTriangles.emplace_back(std::make_pair(triangle.a, triangle.b), tris);
            }
            {
                auto tris = GetTrianglesWithEdge(triangle.b, triangle.c);
                returnTriangles.emplace_back(std::make_pair(triangle.b, triangle.c), tris);
            }
            {
                auto tris = GetTrianglesWithEdge(triangle.c, triangle.a);
                returnTriangles.emplace_back(std::make_pair(triangle.c, triangle.a), tris);
            }

            return returnTriangles;
        }

        bool IsManifold()
        {
            std::vector<std::pair<size_t, size_t>> contours;

            for (auto& [pair, count] : segmentCounts)
            {
                if (count != 2)
                {
                    contours.push_back(pair);
                }
            }

            return contours.empty();
        }

        std::map<size_t, std::vector<std::pair<size_t, size_t>>> GetContourSegments()
        {
            std::map<size_t, std::vector<std::pair<size_t, size_t>>> contours;

            for (auto& [plane, segmentCounts] : planeSegmentCounts)
            {
                for (auto& [pair, count] : segmentCounts)
                {
                    if (count == 1)
                    {
                        contours[plane].push_back(pair);
                    }
                }
            }

            return contours;
        }
    };
///////////////////////////////////////////////////

//============================================================================================
//============================================================================================

    struct SharedPosition
    {
        std::vector<Point> points;
        std::vector<Plane> planes;

        SegmentSet A;
        SegmentSet B;

// TODO: design flaw
        const Geometry* _linkedA;
        const Geometry* _linkedB;

        Geometry relevantA;
        Geometry relevantB;

        BVH relevantBVHA;
        BVH relevantBVHB;

//============================================================================================

//      assumes all triangleIds are connected to base with an edge and are flipped correctly
        size_t FindUppermostTriangleId(Triangle& base, const std::vector<size_t>& triangleIds)
        {
            if (triangleIds.size() == 1)
            {
                return triangleIds[0];
            }

            auto baseNorm = GetNormal(base);

            size_t maxDotId = -1;
            double maxDot = -DBL_MAX;

            for (auto id : triangleIds)
            {
                if (id == base.id) continue;

                auto triNorm = GetNormal(A.triangles[id]);

                auto other = A.triangles[id].GetNotShared(base);

                auto above = CalcTriPt(base, other);

                auto dot = (glm::dot(baseNorm, triNorm) + 1.0) / 2.0; // range dot from 0 to 1, positive for above, negative for below

                if (above == TriangleVsPoint::BELOW)
                {
                    dot = dot - 1.0;
                }
                else
                {
                    dot = 1.0 - dot;
                }

                if (dot > maxDot)
                {
                    maxDot = dot;
                    maxDotId = id;
                }
            }

            return maxDotId;
        }
        
//============================================================================================

        size_t GetPointWithMaxY()
        {
            double max = -DBL_MAX;
            size_t pointID = 0;

            for (auto& p : points)
            {
                if (p.location3D.y > max)
                {
                    max = p.location3D.y;
                    pointID = p.id;
                }
            }

            return pointID;
        }

//============================================================================================

        enum class TriangleVsPoint
        {
            ABOVE,
            BELOW,
            ON
        };

//============================================================================================

        TriangleVsPoint CalcTriPt(Triangle& T, size_t point)
        {
            auto norm = GetNormal(T);

            auto dpt3d = points[point].location3D - points[T.a].location3D;
            auto dot = glm::dot(norm, dpt3d);

            if (std::fabs(dot) < EPS_BIG) return TriangleVsPoint::ON;
            if (dot > 0.0) return TriangleVsPoint::ABOVE;
            return TriangleVsPoint::BELOW;
        }

//============================================================================================

// simplify, see FindUppermostTriangleId
        bool ShouldFlip(Triangle& T, Triangle& neighbour)
        {
/*        
            for each n in N, orient n by formula: 
            if triangle T and N are vertices ABCDEF with BCDE as the shared edge, consider three cases:
            
            E is above T, then A is above n
            if dot(normal(T), E) > 0 => dot(normal(n), A) > 0, else flip n
            E is below T, then A is below n
            if dot(normal(T), E) < 0 => dot(normal(n), A) < 0, else flip n
            E is on T, then normal of n and T are equal
            if dot(normal(T), E) == 0 => dot(normal(n), normal(T)) == 1, else flip n
*/
            auto normT = GetNormal(T);
            auto normNB = GetNormal(neighbour);

            if (T.SamePoints(neighbour))
            {
                // same tri, different winding, flip if not the same normal
                return glm::dot(normT, normNB) < 1.0 - EPS_BIG;
            }

            auto A = T.GetNotShared(neighbour);
            auto E = neighbour.GetNotShared(T);

            TriangleVsPoint EvsT = CalcTriPt(T, E);
            TriangleVsPoint AvsN = CalcTriPt(neighbour, E);

            if (EvsT == TriangleVsPoint::ABOVE && AvsN != TriangleVsPoint::ABOVE)
            {
                return true;
            }
            else if (EvsT == TriangleVsPoint::BELOW && AvsN != TriangleVsPoint::BELOW)
            {
                return true;
            }
            else if (EvsT == TriangleVsPoint::ON && AvsN != TriangleVsPoint::ON)
            {
                return true;
            }
            else
            {
                // neighbour already good!
                return false;
            }
        }

//============================================================================================

        Vec3 GetNormal(Triangle& tri)
        {
            Vec3 temp(-1.0, -1.0, -1.0);
            Vec3 norm = glm::normalize(temp);
            computeSafeNormal(points[tri.a].location3D, points[tri.b].location3D, points[tri.c].location3D, norm, EPS_SMALL);
            return norm;
        }

//============================================================================================

        SegmentSet& GetSegSetA()
        {
            return A;
        }

//============================================================================================

        size_t AddPoint(const Vec3& newPoint)
        {
            for (auto& pt : points)
            {
                if (pt == newPoint)
                {
                    return pt.id;
                }
            }

            Point p;
            p.id = points.size();
            p.location3D = newPoint;

            points.push_back(p);

            return p.id;
        }

//============================================================================================

        size_t AddPlane(const Vec3& normal, double d)
        {
            for (auto& plane : planes)
            {
                if (plane.IsEqualTo(normal, d))
                {
                    return plane.id;
                }
            }

            Plane p;
            p.id = planes.size();
//          p.normal = normal;
            p.normal = glm::normalize(normal);
            p.distance = d;

            planes.push_back(p);

            return p.id;
        }

//============================================================================================

        void Construct(const Geometry& A, const Geometry& B)
        {
            auto boxA = A.GetAABB();
            auto boxB = B.GetAABB();
			
            AddGeometry(A, boxB, true);
            AddGeometry(B, boxA, false);

            _linkedA = &A;
            _linkedB = &B;
        }

//============================================================================================

        void AddGeometry(const Geometry& geom, const AABB& relevantBounds, bool isA)
        {
            Geometry relevant;

            for (size_t i = 0; i < geom.numFaces; i++)
            {
                Face f = geom.GetFace(i);

                auto faceBox = geom.GetFaceBox(i);

                if (!faceBox.intersects(relevantBounds))
                {
                    if (isA)
                    {
                        A.irrelevantFaces.push_back(i);
                    }
                    else
                    {
                        B.irrelevantFaces.push_back(i);
                    }

                    continue;
                }

                auto a = geom.GetPoint(f.i0);
                auto b = geom.GetPoint(f.i1);
                auto c = geom.GetPoint(f.i2);

                relevant.AddFace(a, b, c);

                Vec3 norm;
                if (computeSafeNormal(a, b, c, norm, EPS_SMALL))
                {
                    auto ia = AddPoint(a);
                    auto ib = AddPoint(b);
                    auto ic = AddPoint(c);

                    double da = glm::dot(norm, a);
                    double db = glm::dot(norm, b);
                    double dc = glm::dot(norm, c);

                    size_t planeId = AddPlane(norm, da);

                    if (!planes[planeId].IsPointOnPlane(a))
                    {
                        if (messages) { printf("unexpected point on plane in AddGeometry\n"); }
                        if (messages) { printf("a = (%12.8f, %12.8f, %12.8f), da = %12.8f, distance = %12.8f\n", a.x, a.y, a.z, da, planes[planeId].distance); }
                    }
                    if (!planes[planeId].IsPointOnPlane(b))
                    {
                        if (messages) { printf("unexpected point on plane in AddGeometry\n"); }
                        if (messages) { printf("b = (%12.8f, %12.8f, %12.8f), db = %12.8f, distance = %12.8f\n", b.x, b.y, b.z, db, planes[planeId].distance); }
                    }
                    if (!planes[planeId].IsPointOnPlane(c))
                    {
                        if (messages) { printf("unexpected point on plane in AddGeometry\n"); }
                        if (messages) { printf("c = (%12.8f, %12.8f, %12.8f), dc = %12.8f, distance = %12.8f\n", c.x, c.y, c.z, dc, planes[planeId].distance); }
                    }
                    planes[planeId].AddPoint(a);
                    planes[planeId].AddPoint(b);
                    planes[planeId].AddPoint(c);

                    if (isA)
                    {
                        A.AddFace(planeId, ia, ib, ic);
                        relevantA.AddFace(a, b, c);
                    }
                    else
                    {
                        B.AddFace(planeId, ia, ib, ic);
                        relevantB.AddFace(a, b, c);
                    }
                }
                else
                {
                    if (messages) { printf("Degenerate face in AddGeometry\n"); }
                }
            }

            if (isA)
            {
                relevantBVHA = MakeBVH(relevantA);
                DumpGeometry(relevant, L"relevantA.obj");
            }
            else
            {
                relevantBVHB = MakeBVH(relevantB);
                DumpGeometry(relevant, L"relevantB.obj");
            }
        }
        
//============================================================================================

        std::vector<size_t> GetPointsOnPlane(Plane& p)
        {
            auto cp = planeToPoints[p.id];
            std::sort(cp.begin(), cp.end());
            cp.erase(std::unique(cp.begin(), cp.end()), cp.end());
            return cp;
        }

//============================================================================================

        // pair of lineID, distance
        std::vector<std::pair<double, double>> BuildSegments(const std::vector<double>& a, const std::vector<double>& b) const
        {
            if (a.size() == 0 || b.size() == 0)
            {
                return {};
            }

            // we need to figure out the overlap between the two lists of intersections
            // we can be clever here and try to conclude that the first point must be the start of a segment and the next point would end that segment
	    // however, we would really shoot ourselves in the foot as any coplanar results are missing from these two sets
            // let's just make some segments that span both intersection lists, and eat the overhead

            double min = std::max(a[0], b[0]);
            double max = std::min(a[a.size() - 1], b[b.size() - 1]);

            std::vector<double> points;

            for (size_t i = 0; i < a.size(); i++)
            {
                double val = a[i];
                if (val >= min && val <= max)
                {
                    points.push_back(val);
                }
            }

            for (size_t i = 0; i < b.size(); i++)
            {
                double val = b[i];
                if (val >= min && val <= max)
                {
                    points.push_back(val);
                }
            }

            std::sort
            (
                points.begin(), points.end(), [&](const double& left, const double& right)
                {
                    return left < right;
                }
            );

            std::vector<std::pair<double, double>> result;

            for (size_t i = 1; i < points.size(); i++)
            {
                result.emplace_back(points[i - 1], points[i]);
            }
            
            // Remove redundant lines
            result.erase(std::unique(result.begin(), result.end()), result.end());

            return result;
        }

//============================================================================================

        std::vector<std::pair<size_t, size_t>> GetNonIntersectingSegments(Line& l)
        {
            std::vector<std::pair<size_t, double>> pointsInOrder;

            for (auto& segment : l.GetSegments())
            {
                if (!l.IsPointOnLine(points[segment.first].location3D))
                {
                    if (messages) { printf("point not on line in GetNonIntersectingSegments\n"); }
                    if (messages) { printf("points[segment.first].location3D = (%12.8f, %12.8f, %12.8f)\n", points[segment.first].location3D.x, points[segment.first].location3D.y, points[segment.first].location3D.z); }
                    if (messages) { printf("l.origin                         = (%12.8f, %12.8f, %12.8f)\n", l.origin.x, l.origin.y, l.origin.z); }
                    if (messages) { printf("l.direction                      = (%12.8f, %12.8f, %12.8f)\n", l.direction.x, l.direction.y, l.direction.z); }
                }

                if (!l.IsPointOnLine(points[segment.second].location3D))
                {
                    if (messages) { printf("point not on line in GetNonIntersectingSegments\n"); }
                    if (messages) { printf("points[segment.second].location3D = (%12.8f, %12.8f, %12.8f)\n", points[segment.second].location3D.x, points[segment.second].location3D.y, points[segment.second].location3D.z); }
                    if (messages) { printf("l.origin                          = (%12.8f, %12.8f, %12.8f)\n", l.origin.x, l.origin.y, l.origin.z); }
                    if (messages) { printf("l.direction                       = (%12.8f, %12.8f, %12.8f)\n", l.direction.x, l.direction.y, l.direction.z); }
                }

                pointsInOrder.emplace_back(segment.first, l.GetPosOnLine(points[segment.first].location3D));
                pointsInOrder.emplace_back(segment.second, l.GetPosOnLine(points[segment.second].location3D));
            }

            std::sort
            (
                pointsInOrder.begin(), pointsInOrder.end(), [&](const std::pair<size_t, double>& left, const std::pair<size_t, double>& right)
                {
                    return left.second > right.second;
                }
            );

            std::vector<std::pair<size_t, size_t>> segmentsWithoutIntersections;

            if (pointsInOrder.empty()) return {};

            size_t cur = pointsInOrder[0].first;
            for (size_t i = 1; i < pointsInOrder.size(); i++)
            {
                // this fills gaps, but we don't care about that in this stage
                // gaps filled will be removed during inside/outside checking
                size_t next = pointsInOrder[i].first;
                if (cur != next)
                {
                    segmentsWithoutIntersections.emplace_back(cur, next);
                    cur = next;
                }
            }
            return segmentsWithoutIntersections;
        }

//============================================================================================

        void TriangulatePlane(Geometry& geom, Plane& p)
        {
            // grab all points on the plane
            auto pointsOnPlane = GetPointsOnPlane(p);

            // temporarily project all points for triangulation
            // NOTE: these points should not be used as output, only as placeholder for triangulation!

            auto basis = p.MakeBasis();

            std::unordered_map<size_t, size_t> pointToProjectedPoint;
            std::unordered_map<size_t, size_t> projectedPointToPoint;

            std::vector<glm::dvec2> projectedPoints;

            for (auto& pointId : pointsOnPlane)
            {
                pointToProjectedPoint[pointId] = projectedPoints.size();
                projectedPointToPoint[projectedPoints.size()] = pointId;
                projectedPoints.push_back(basis.project(points[pointId].location3D));
            }

            std::set<std::pair<size_t, size_t>> edges;
            std::set<std::pair<size_t, size_t>> defaultEdges;
            static int i = 0;
            i++;

            for (auto& line : p.lines)
            {
                // these segments might intersect internally, lets resolve that so we get a valid chain
                auto segments = GetNonIntersectingSegments(line);

                for (auto& segment : segments)
                {
                    if (pointToProjectedPoint.count(segment.first) == 0)
                    {
                        bool expectedOnPlane = p.IsPointOnPlane(points[segment.first].location3D);
                        if (messages) { printf("unknown point in list, repairing in TriangulateLine\n"); }

                        pointToProjectedPoint[segment.first] = projectedPoints.size();
                        projectedPointToPoint[projectedPoints.size()] = segment.first;
                        projectedPoints.push_back(basis.project(points[segment.first].location3D));
                    }
                    if (pointToProjectedPoint.count(segment.second) == 0)
                    {
                        bool expectedOnPlane = p.IsPointOnPlane(points[segment.second].location3D);
                        if (messages) { printf("unknown point in list, repairing in TriangulateLine\n"); }

                        pointToProjectedPoint[segment.second] = projectedPoints.size();
                        projectedPointToPoint[projectedPoints.size()] = segment.second;
                        projectedPoints.push_back(basis.project(points[segment.second].location3D));
                    }

                    auto projectedIndexA = pointToProjectedPoint[segment.first];
                    auto projectedIndexB = pointToProjectedPoint[segment.second];

                    if (projectedIndexA != projectedIndexB)
                    {
                        defaultEdges.insert(segment);
                        edges.insert(std::make_pair(projectedIndexA, projectedIndexB));
                    }
                }
            }

            if (true)
            {
                std::vector<std::vector<glm::dvec2>> edgesPrinted;

                for (auto& e : edges)
                {
                    edgesPrinted.push_back({ projectedPoints[e.first], projectedPoints[e.second] });
                }

                DumpSVGLines(edgesPrinted, L"poly.html");
            }

            CDT::Triangulation<double> cdt(CDT::VertexInsertionOrder::AsProvided);
            std::vector<CDT::Edge> cdt_edges;
            std::vector<CDT::V2d<double>> cdt_verts;

            for (auto& point : projectedPoints)
            {
                cdt_verts.emplace_back(CDT::V2d<double>::make(point.x, point.y));
            }

            for (auto& edge : edges)
            {
                cdt_edges.emplace_back((uint32_t)edge.first, (uint32_t)edge.second);
            }

            auto mapping = CDT::RemoveDuplicatesAndRemapEdges(cdt_verts, cdt_edges).mapping;

            cdt.insertVertices(cdt_verts);
            cdt.insertEdges(cdt_edges);

            cdt.eraseSuperTriangle();

            auto triangles = cdt.triangles;

            //auto contourLoop = FindLargestEdgeLoop(projectedPoints, edges);

            std::set<std::pair<size_t, size_t>> edgesTriangles;
            std::set<std::pair<size_t, size_t>> finalEdgesTriangles;

            for (auto& tri : triangles)
            {
                edgesTriangles.insert(std::make_pair(tri.vertices[0], tri.vertices[1]));
                edgesTriangles.insert(std::make_pair(tri.vertices[1], tri.vertices[2]));
                edgesTriangles.insert(std::make_pair(tri.vertices[0], tri.vertices[2]));
                size_t pointIdA = projectedPointToPoint[mapping[tri.vertices[0]]];
                size_t pointIdB = projectedPointToPoint[mapping[tri.vertices[1]]];
                size_t pointIdC = projectedPointToPoint[mapping[tri.vertices[2]]];

                auto ptA = points[pointIdA].location3D;
                auto ptB = points[pointIdB].location3D;
                auto ptC = points[pointIdC].location3D;

                auto pt2DA = projectedPoints[mapping[tri.vertices[0]]];
                auto pt2DB = projectedPoints[mapping[tri.vertices[1]]];
                auto pt2DC = projectedPoints[mapping[tri.vertices[2]]];

                auto triCenter = (ptA + ptB + ptC) / 3.0;

                auto posA = isInsideMesh(triCenter, glm::dvec3(0), relevantA, relevantBVHA);
                auto posB = isInsideMesh(triCenter, glm::dvec3(0), relevantB, relevantBVHB);

                if (posA.loc != MeshLocation::BOUNDARY && posB.loc != MeshLocation::BOUNDARY)
                {
                    continue;
                }

                // although CDT is great, it spits out too many or too little tris, we fix it manually
                //if (!IsPointInsideLoop(projectedPoints, contourLoop, triCenter))
                //{
                //    printf("removing point outside loop\n");
                //    continue;
                //}

                // TODO: why is this swapped? winding doesnt matter much, but still
                geom.AddFace(ptB, ptA, ptC);
                finalEdgesTriangles.insert(std::make_pair(tri.vertices[0], tri.vertices[1]));
                finalEdgesTriangles.insert(std::make_pair(tri.vertices[1], tri.vertices[2]));
                finalEdgesTriangles.insert(std::make_pair(tri.vertices[0], tri.vertices[2]));
            }
        
            if (true)
            {
                std::vector<std::vector<glm::dvec2>> edgesPrinted;

                for (auto& e : edgesTriangles)
                {
                    edgesPrinted.push_back({ projectedPoints[e.first], projectedPoints[e.second] });
                }

                DumpSVGLines(edgesPrinted, L"poly_triangulation.html");

                std::vector<std::vector<glm::dvec2>> finalEdgesPrinted;

                for (auto& e : finalEdgesTriangles)
                {
                    finalEdgesPrinted.push_back({ projectedPoints[e.first], projectedPoints[e.second] });
                }

                DumpSVGLines(finalEdgesPrinted, L"final_poly_triangulation.html");
            }
        }
//============================================================================================

        std::unordered_map<size_t, std::vector<size_t>> planeToLines;

//============================================================================================

        std::unordered_map<size_t, std::vector<size_t>> planeToPoints;

//============================================================================================

        void AddRefPlaneToPoint(size_t point, size_t plane)
        {
            ReferencePlane ref;
            ref.pointID = point;
            ref.planeID = plane;
            points[point].planes.push_back(ref);
            planeToPoints[plane].push_back(point);
        }
    };

//============================================================================================

    inline void AddSegments(Plane& p, SharedPosition& sp, Line& templine, const std::vector<std::pair<double, double>>& segments)
    {
        // NOTE: this is a design flaw, the addline may return a line that is
        // EQUIVALENT BUT NOT IDENTICAL
        // hence line distances mentioned in "segments" DO apply to templine
        // but not necessary (but possibly) to isectLineId
        auto isectLineId = p.AddLine(templine.origin, templine.direction);

        auto& isectLine = p.lines[isectLineId.first];

        if (!p.IsPointOnPlane(isectLine.origin) || !p.IsPointOnPlane(isectLine.origin + isectLine.direction * 100.))
        {
            if (messages) {  printf("Bad isect line in AddSegments\n"); }
        }

        for (auto& seg : segments)
        {
            auto pos = templine.GetPosOnLine(seg.first);

            if (!p.aabb.contains(pos))
            {
                if (messages) { printf("making pos outside in AddSegments]\n"); }
            }

            size_t ptA = sp.AddPoint(pos);
            size_t ptB = sp.AddPoint(templine.GetPosOnLine(seg.second));

            if (!p.aabb.contains(sp.points[ptA].location3D))
            {
                if (messages) { printf("bad points in AddSegments\n"); }
            }
            if (!p.aabb.contains(sp.points[ptB].location3D))
            {
                if (messages) { printf("bad points in AddS.segments\n"); }
            }

            //if (ptA != ptB)
            {
                isectLine.AddPointToLine(isectLine.GetPosOnLine(sp.points[ptA].location3D), ptA);
                isectLine.AddPointToLine(isectLine.GetPosOnLine(sp.points[ptB].location3D), ptB);
            }

            if (!p.IsPointOnPlane(sp.points[ptA].location3D))
            {
                if (messages) { printf("bad point in AddSegments\n"); }
            }
            if (!p.IsPointOnPlane(sp.points[ptB].location3D))
            {
                if (messages) { printf("bad point in AddSegments\n"); }
            }

            sp.AddRefPlaneToPoint(ptA, p.id);
            sp.AddRefPlaneToPoint(ptB, p.id);
        }
    }

//============================================================================================

    inline std::vector<double> ComputeInitialIntersections(Plane& p, SharedPosition& sp, const Line& lineA)
    {
        double size = 1.0E+04; // TODO: this is bad
	
        for (auto& point : sp.points)
        {
            double d = glm::distance(lineA.origin, point.location3D);
            if (size < d)
            {
                size = d;
            }
        }

        auto Astart = lineA.origin + lineA.direction * size;
        auto Aend = lineA.origin - lineA.direction * size;

        std::vector<double> distances;

        // line B is expected to have the segments already filled, line A is not
        for (auto& line : p.lines)
        {
            // skip collinear
            if (lineA.IsCollinear(line)) continue;

            for (const auto& seg : line.GetSegments())
            {
                auto result = LineLineIntersection
                              (
                                  Astart,
                                  Aend,
                                  sp.points[seg.first].location3D,
                                  sp.points[seg.second].location3D
                              );

                if (result.distance < SCALED_EPS_BIG)
                {
                    if (!p.aabb.contains(sp.points[seg.first].location3D))
                    {
                        if (messages) { printf("bad points in ComputeInitialIntersections\n"); }
                    }
                    if (!p.aabb.contains(sp.points[seg.second].location3D))
                    {
                        if (messages) { printf("bad points in ComputeInitialIntersections\n"); }
                    }

                    // intersection, mark index of line B and distance on line A
                    distances.emplace_back(lineA.GetPosOnLine(result.point2));
                    auto pt = lineA.GetPosOnLine(distances[distances.size() - 1]);

                    if (!p.aabb.contains(result.point2))
                    {
                        if (messages) { printf("bad points in ComputeInitialIntersections\n"); }
                    }

                    if (!equals(pt, result.point2, SCALED_EPS_BIG))
                    {
                        if (messages) { printf("BAD POINT in ComputeInitialIntersections\n"); }
                    }
                }
            }
        }
	
        std::sort
        (
            distances.begin(),
            distances.end(),
            [&](const double& left, const double& right)
            {
	        return left < right;
            }
        );

        distances.erase(std::unique(distances.begin(), distances.end()), distances.end());

        return distances;
    }

//============================================================================================

    inline void AddLineLineIntersections(Plane& p, SharedPosition& sp, Line& lineA, Line& lineB)
    {
        for (auto& segA : lineA.GetSegments())
        {
            for (auto& segB : lineB.GetSegments())
            {
                // check isect A vs B
                if (!p.HasOverlap(segA, segB))
                {
                    // no overlap, possibility of intersection
                    auto result = LineLineIntersection
                                  (
                                    sp.points[segA.first].location3D,
                                    sp.points[segA.second].location3D,
                                    sp.points[segB.first].location3D,
                                    sp.points[segB.second].location3D
                                  );

                    if (result.distance < SCALED_EPS_BIG)
                    {
                        // intersection! Take center and insert
                        if (!p.aabb.contains(result.point1))
                        {
                            if (messages) { printf("bad points in AddLineLineIntersections\n"); }
                            continue;
                        }

                        size_t point = sp.AddPoint((result.point1));

                        lineA.AddPointToLine(lineA.GetPosOnLine(sp.points[point].location3D), point);
                        lineB.AddPointToLine(lineB.GetPosOnLine(sp.points[point].location3D), point);

                        // point falls on intersection of two lines, so is part of those lines and all planes of those lines
                        {
                            ReferenceLine ref;
                            ref.pointID = point;
                            ref.lineID = lineA.id;
                            ref.location = lineA.GetPosOnLine(result.point1);
                            sp.points[point].lines.push_back(ref);

                            // TODO: FIX THIS POINT MIGHT NOT BE ON THE PLANE ACTUALLY

                            for (auto& plane : lineA.planes)
                            {
                                sp.AddRefPlaneToPoint(point, plane.planeID);
                            }
                        }
                        {
                            ReferenceLine ref;
                            ref.pointID = point;
                            ref.lineID = lineB.id;
                            ref.location = lineB.GetPosOnLine(result.point2);
                            sp.points[point].lines.push_back(ref);

                            for (auto& plane : lineB.planes)
                            {
                                sp.AddRefPlaneToPoint(point, plane.planeID);
                            }
                        }
                    }
                }
            }
        }
    }

//============================================================================================

    inline void AddLineLineIsects(Plane& p, SharedPosition& sp)
    {
        for (size_t lineAIndex = 0; lineAIndex < p.lines.size(); lineAIndex++)
        {
            for (size_t lineBIndex = lineAIndex + 1; lineBIndex < p.lines.size(); lineBIndex++)
            {
                AddLineLineIntersections(p, sp, p.lines[lineAIndex], p.lines[lineBIndex]);
            }
        }
    }

//============================================================================================

    inline Geometry Normalize(SharedPosition& sp)
    {
        // construct all contours, derive lines
        auto contoursA = sp.A.GetContourSegments();

        for (auto& [planeId, contours] : contoursA)
        {
            std::vector<std::vector<glm::dvec2>> edges;

            Plane& p = sp.planes[planeId];

            if (false)
            {
                auto basis = p.MakeBasis();

                for (auto& segment : contours)
                {
                    edges.push_back({ basis.project(sp.points[segment.first].location3D), basis.project(sp.points[segment.second].location3D) });
                }
                DumpSVGLines(edges, L"contour.html");
            }
            for (auto& segment : contours)
            {
                auto lineId = sp.planes[planeId].AddLine(sp.points[segment.first], sp.points[segment.second]);
            }
        }

        auto contoursB = sp.B.GetContourSegments();

        for (auto& [planeId, contours] : contoursB)
        {
            for (auto& segment : contours)
            {
                auto lineId = sp.planes[planeId].AddLine(sp.points[segment.first], sp.points[segment.second]);
            }
        }

        // put all points on lines/planes
        for (auto& p : sp.points)
        {
            for (auto& plane : sp.planes)
            {
                if (plane.IsPointOnPlane(p.location3D))
                {
                    sp.AddRefPlaneToPoint(p.id, plane.id);
                    plane.PutPointOnLines(p);
                }
            }
        }

        for (auto& plane : sp.planes)
        {
            AddLineLineIsects(plane, sp);
        }

        if (true)
        {
            // intersect planes
            for (size_t planeAIndex = 0; planeAIndex < sp.planes.size(); planeAIndex++)
            {
                for (size_t planeBIndex = 0; planeBIndex < sp.planes.size(); planeBIndex++)
                {
                    auto& planeA = sp.planes[planeAIndex];
                    auto& planeB = sp.planes[planeBIndex];

                    if (!planeA.aabb.intersects(planeB.aabb))
                    {
                        continue;
                    }

                    // plane intersect results in new lines
                    // new lines result in new line intersects
                    // new line intersects result in new points

                    if (std::fabs(glm::dot(planeA.normal, planeB.normal)) > 1.0 - EPS_BIG)
                    {
                        // parallel planes, don't care
                        continue;
                    }

                    // calculate plane intersection line
                    auto result = PlanePlaneIsect(planeA.normal, planeA.distance, planeB.normal, planeB.distance);

                    // TODO: invalid temp line object
                    Line intersectionLine;
                    intersectionLine.origin = result.pos;
                    intersectionLine.direction = result.dir;

                    if (!planeA.IsPointOnPlane(intersectionLine.origin) || !planeA.IsPointOnPlane(intersectionLine.origin + intersectionLine.direction * 1000.))
                    {
                        if (messages) { printf("Bad isect line in Normalize\n"); }
                    }
                    if (!planeB.IsPointOnPlane(intersectionLine.origin) || !planeB.IsPointOnPlane(intersectionLine.origin + intersectionLine.direction * 1000.))
                    {
                        if (messages) { printf("Bad isect line in Normalize\n"); }
                    }

                    // get all intersection points with the shared line and both planes
                    auto isectA = ComputeInitialIntersections(planeA, sp, intersectionLine);
                    auto isectB = ComputeInitialIntersections(planeB, sp, intersectionLine);

                    // from these, figure out the shared segments on the current line produced by these two planes
                    auto segments = sp.BuildSegments(isectA, isectB);

                    if (segments.empty())
                    {
                        // nothing resulted from this plane-plane intersection
                        continue;
                    }
                    else
                    {
                        AddSegments(planeA, sp, intersectionLine, segments);
                        AddSegments(planeB, sp, intersectionLine, segments);
                    }
                }
            }
        }

        for (auto& plane : sp.planes)
        {
            AddLineLineIsects(plane, sp);
        }

        for (auto& p : sp.points)
        {
            for (auto& plane : sp.planes)
            {
                if (plane.IsPointOnPlane(p.location3D))
                {
                    sp.AddRefPlaneToPoint(p.id, plane.id);
                }
            }
        }

        // from the inserted geometries, all lines planes and points are now merged into a single set of shared planes lines and points
        // from this starting point, we can triangulate all planes and obtain the triangulation of the intersected set of geometries
        // this mesh itself is not a boolean result, but rather a merging of all operands

        Geometry geom;
        for (auto& plane : sp.planes)
        {
            sp.TriangulatePlane(geom, plane);
        }

        // re-add irrelevant faces
        for (auto& faceIndex : sp.A.irrelevantFaces)
        {
            const Face& f = sp._linkedA->GetFace(faceIndex);

            auto a = sp._linkedA->GetPoint(f.i0);
            auto b = sp._linkedA->GetPoint(f.i1);
            auto c = sp._linkedA->GetPoint(f.i2);

            geom.AddFace(a, b, c);
        }

/*
        for (auto& faceIndex : sp.B.irrelevantFaces)
        {
            const Face& f = sp._linkedB->GetFace(faceIndex);

            auto a = sp._linkedB->GetPoint(f.i0);
            auto b = sp._linkedB->GetPoint(f.i1);
            auto c = sp._linkedB->GetPoint(f.i2);

            geom.AddFace(a, b, c);
        }
*/

        return geom;
    }
}
