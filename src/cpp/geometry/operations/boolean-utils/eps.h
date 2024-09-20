#pragma once

constexpr bool messages = false;

/*
    Constants from the original eps.h.  However,
        EPS_TINY, EPS_SMALL, EPS_BIG, EPS_BIG2 and SCALED_EPS_BIG
    have been assigned a new, common value.  Use of these constants
    needs to be rationalised and eliminated!
*/
constexpr double EPS_NONZERO    = 1.0E-20;
constexpr double EPS_MINISCULE  = 1.0E-12;
constexpr double EPS_TINY       = 1.0E-04;
constexpr double EPS_SMALL      = 1.0E-04;
constexpr double EPS_BIG        = 1.0E-04;
constexpr double EPS_BIG2       = 1.0E-04;
constexpr double SCALED_EPS_BIG = 1.0E-04;

/*
    Constants used in math.h
*/    
constexpr double toleranceIsInsideCenterExtents = 1.0E-10;

/*
    Constants used in shared-position.h
*/    
constexpr double toleranceVectorEquality = 1.0E-04;
constexpr double toleranceScalarEquality = 1.0E-04;
constexpr double toleranceCollinear      = 1.0E-04;

/*
    Constants used in shared-position
*/    
constexpr double tolerancePointOnLine     = 1.0E-04;

/*
    Constants used in function intersect_ray_triangle
*/    
constexpr double toleranceParallel      = 1.0E-04;
constexpr double toleranceParallelTight = 1.0E-10;
constexpr double toleranceInsideOutside = 1.0E-10; // This eps works for some models at 10-4 but others require 10-10
constexpr double toleranceInsideOutsideToPlane = 1.0E-04; // This eps works for some models at 10-4 but others require 10-10
constexpr double toleranceThinTriangle = 1.0e-10;
/*
    Constants used in function isInsideMesh
*/    
constexpr double toleranceBoundaryPoint = 1.0E-04;

/*
    Constants used in function AddFace in geometry.h
*/    
constexpr double toleranceAddFace = 1.0E-10;

/*
        Round all coordinates when they are read to a specified number of decimal places.
        For example, if the rounding is to be in the fourth decimal place, then set
            ROUNDING = 1.0E-04
        and
            ROUNDING_RECIPROCAL = 1.0E+04.
            
        To disable/enable rounding, set ROUNDING_ENABLE to 0/1.            
*/                    
constexpr int    ROUNDING_ENABLE     = 1;
constexpr double ROUNDING            = 1.0E-04;
constexpr double ROUNDING_RECIPROCAL = 1.0E+04;

