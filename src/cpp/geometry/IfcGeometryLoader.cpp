/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.   */

#include <spdlog/spdlog.h>
#include "IfcGeometryLoader.h"
#include "operations/curve-utils.h"
#include "operations/geometryutils.h"
#ifdef DEBUG_DUMP_SVG
#include "../test/io_helpers.h"
#endif

namespace webifc::geometry
{

  IfcGeometryLoader::IfcGeometryLoader(const webifc::parsing::IfcLoader &loader, const webifc::schema::IfcSchemaManager &schemaManager, uint16_t circleSegments)
      : _loader(loader), _schemaManager(schemaManager), _relVoidRel(PopulateRelVoidsRelMap()), _relVoids(PopulateRelVoidsMap()), _relAggregates(PopulateRelAggregatesMap()), _relNests(PopulateRelNestsMap()),
        _relElementAggregates(PopulateRelElementAggregatesMap()), _styledItems(PopulateStyledItemMap()), _relMaterials(PopulateRelMaterialsMap()), _materialDefinitions(PopulateMaterialDefinitionsMap()), _circleSegments(circleSegments)
  {
    ReadLinearScalingFactor();
  }

  void IfcGeometryLoader::Clear() const{
      _expressIDToPlacement.clear();
      std::unordered_map<uint32_t, glm::dmat4>().swap(_expressIDToPlacement);
  }

  IfcCrossSections IfcGeometryLoader::GetCrossSections2D(uint32_t expressID) const
  {
    spdlog::debug("[GetCrossSections2D({})]",expressID);
    auto lineType = _loader.GetLineType(expressID);
    IfcCrossSections sections;
    switch (lineType)
    {
    case schema::IFCSECTIONEDSOLIDHORIZONTAL:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      auto curveId = _loader.GetRefArgument();

      // faces
      _loader.MoveToArgumentOffset(expressID, 1);
      auto faces = _loader.GetSetArgument();

      // linear position
      _loader.MoveToArgumentOffset(expressID, 2);
      auto linearPositions = _loader.GetSetArgument();

      IfcCurve curve = GetCurve(curveId, 3);

      std::vector<IfcProfile> profiles;
      std::vector<IfcCurve> curves;
      std::vector<uint32_t> expressIds;

      uint32_t id = 0;
      for (auto &face : faces)
      {
        auto expressID = _loader.GetRefArgument(face);
        IfcProfile profile = GetProfile(expressID);
        profiles.push_back(profile);
        curves.push_back(profile.curve);
        expressIds.push_back(expressID);
        id++;
      }

      sections.curves = curves;
      sections.expressID = expressIds;

      return sections;

      break;
    }
    case schema::IFCSECTIONEDSOLID:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      auto curveId = _loader.GetRefArgument();

      // faces
      _loader.MoveToArgumentOffset(expressID, 1);
      auto faces = _loader.GetSetArgument();

      // linear position
      _loader.MoveToArgumentOffset(expressID, 2);
      auto linearPositions = _loader.GetSetArgument();

      IfcCurve curve = GetCurve(curveId, 3);

      std::vector<IfcProfile> profiles;
      std::vector<IfcCurve> curves;
      std::vector<uint32_t> expressIds;

      uint32_t id = 0;
      for (auto &face : faces)
      {
        auto expressID = _loader.GetRefArgument(face);
        IfcProfile profile = GetProfile(expressID);
        profiles.push_back(profile);
        curves.push_back(profile.curve);
        expressIds.push_back(expressID);
        id++;
      }

      sections.curves = curves;
      sections.expressID = expressIds;

      return sections;

      break;
    }
    case schema::IFCSECTIONEDSURFACE:
    {
      // faces
      _loader.MoveToArgumentOffset(expressID, 1);
      auto linearPositions = _loader.GetSetArgument();

      // linear position
      _loader.MoveToArgumentOffset(expressID, 2);
      auto faces = _loader.GetSetArgument();

      std::vector<IfcProfile> profiles;
      std::vector<IfcCurve> curves;
      std::vector<uint32_t> expressIds;

      uint32_t id = 0;
      for (auto &face : faces)
      {
        auto expressID = _loader.GetRefArgument(face);
        IfcProfile profile = GetProfile(expressID);
        profiles.push_back(profile);
        curves.push_back(profile.curve);
        expressIds.push_back(expressID);
        id++;
      }

      sections.curves = curves;
      sections.expressID = expressIds;
      return sections;
    }
    }
  }

  IfcCrossSections IfcGeometryLoader::GetCrossSections3D(uint32_t expressID,  bool scaled, glm::dmat4 coordination) const
  {
   spdlog::debug("[GetCrossSections3D({})]",expressID);
    auto lineType = _loader.GetLineType(expressID);
    IfcCrossSections sections;
    double scale = 1;
    if (scaled)
    {
      scale = GetLinearScalingFactor();
      glm::dvec4 ps = coordination[3];
      double y = ps[1];
      double z = -ps[2];
      ps[1] = z;
      ps[2] = y;
      coordination[3] = ps;
    }
    switch (lineType)
    {
    case schema::IFCSECTIONEDSOLIDHORIZONTAL:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      auto curveId = _loader.GetRefArgument();

      // faces
      _loader.MoveToArgumentOffset(expressID, 1);
      auto faces = _loader.GetSetArgument();

      // linear position
      _loader.MoveToArgumentOffset(expressID, 2);
      auto linearPositions = _loader.GetSetArgument();

      IfcCurve curve = GetCurve(curveId, 3);

      std::vector<IfcProfile> profiles;
      std::vector<IfcCurve> curves;
      std::vector<uint32_t> expressIds;

      std::vector<glm::dmat4> transform;
      for (auto &linearPosition : linearPositions)
      {
        auto expressID = _loader.GetRefArgument(linearPosition);
        glm::dmat4 linearPlacement = GetLocalPlacement(expressID) * scale;
        transform.push_back(linearPlacement);
      }

      uint32_t id = 0;
      for (auto &face : faces)
      {
        auto expressID = _loader.GetRefArgument(face);
        IfcProfile profile = GetProfile(expressID);
        for (uint32_t i = 0; i < profile.curve.points.size(); i++)
        {
          glm::dvec3 pTemp = transform[id] * glm::dvec4(profile.curve.points[i], 1);
          profile.curve.points[i] = coordination * glm::dvec4(pTemp.x, pTemp.y, pTemp.z, 1);
        }
        profiles.push_back(profile);
        curves.push_back(profile.curve);
        expressIds.push_back(expressID);
        id++;
      }

      sections.curves = curves;
      sections.expressID = expressIds;

      return sections;

      break;
    }
    case schema::IFCSECTIONEDSOLID:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      auto curveId = _loader.GetRefArgument();

      // faces
      _loader.MoveToArgumentOffset(expressID, 1);
      auto faces = _loader.GetSetArgument();

      // linear position
      _loader.MoveToArgumentOffset(expressID, 2);
      auto linearPositions = _loader.GetSetArgument();

      IfcCurve curve = GetCurve(curveId, 3);

      std::vector<IfcProfile> profiles;
      std::vector<IfcCurve> curves;
      std::vector<uint32_t> expressIds;

      std::vector<glm::dmat4> transform;
      for (auto &linearPosition : linearPositions)
      {
        auto expressID = _loader.GetRefArgument(linearPosition);
        glm::dmat4 linearPlacement = GetLocalPlacement(expressID) * scale;
        transform.push_back(linearPlacement);
      }

      uint32_t id = 0;
      for (auto &face : faces)
      {
        auto expressID = _loader.GetRefArgument(face);
        IfcProfile profile = GetProfile(expressID);
        for (uint32_t i = 0; i < profile.curve.points.size(); i++)
        {
          glm::dvec3 pTemp = transform[id] * glm::dvec4(profile.curve.points[i], 1);
          profile.curve.points[i] = coordination * glm::dvec4(pTemp.x, pTemp.y, pTemp.z, 1);
        }
        profiles.push_back(profile);
        curves.push_back(profile.curve);
        expressIds.push_back(expressID);
        id++;
      }

      sections.curves = curves;
      sections.expressID = expressIds;
      return sections;

      break;
    }
    case schema::IFCSECTIONEDSURFACE:
    {
      // faces
      _loader.MoveToArgumentOffset(expressID, 1);
      auto linearPositions = _loader.GetSetArgument();

      // linear position
      _loader.MoveToArgumentOffset(expressID, 2);
      auto faces = _loader.GetSetArgument();

      std::vector<glm::dmat4> transform;
      for (auto &linearPosition : linearPositions)
      {
        auto expressID = _loader.GetRefArgument(linearPosition);
        glm::dmat4 linearPlacement = GetLocalPlacement(expressID) * scale;
        transform.push_back(linearPlacement);
      }

      uint32_t id = 0;
      std::vector<IfcProfile> profiles;
      std::vector<IfcCurve> curves;
      std::vector<uint32_t> expressIds;

      for (auto &face : faces)
      {
        auto expressID = _loader.GetRefArgument(face);
        IfcProfile profile = GetProfile(expressID);
        for (uint32_t i = 0; i < profile.curve.points.size(); i++)
        {
          glm::dvec3 pTemp = transform[id] * glm::dvec4(profile.curve.points[i], 1);
          profile.curve.points[i] = coordination * glm::dvec4(pTemp.x, pTemp.y, pTemp.z, 1);
        }
        profiles.push_back(profile);
        curves.push_back(profile.curve);
        expressIds.push_back(expressID);
        id++;
      }

      sections.curves = curves;
      sections.expressID = expressIds;
      return sections;
    }
    }
  }

  IfcAlignment IfcGeometryLoader::GetAlignment(uint32_t expressID, IfcAlignment alignment, glm::dmat4 transform, uint32_t sourceExpressID) const
  {
    spdlog::debug("[GetAlignment({})]",expressID);
    auto lineType = _loader.GetLineType(expressID);

    // TODO: Add support for IFCREFERENT

    switch (lineType)
    {
    case schema::IFCALIGNMENT:
    {
      _loader.MoveToArgumentOffset(expressID, 5);
      uint32_t localPlacement = 0;
      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        localPlacement = _loader.GetRefArgument();
      }

      glm::dmat4 transform_t = glm::dmat4(1);
      if (localPlacement != 0 && _loader.IsValidExpressID(localPlacement))
      {
        transform_t = GetLocalPlacement(localPlacement);
      }

      auto &relAggVector = GetRelAggregates();
      if (relAggVector.count(expressID) == 1)
      {
        auto &relAgg = relAggVector.at(expressID);
        for (auto expressID : relAgg)
        {
          alignment = GetAlignment(expressID, alignment, transform * transform_t, expressID);
        }
      }

      auto &relNestsVector = GetRelNests();
      if (relNestsVector.count(expressID) == 1)
      {
        auto &relNest = relNestsVector.at(expressID);
        for (auto expressID : relNest)
        {
          alignment = GetAlignment(expressID, alignment, transform * transform_t, expressID);
        }
      }

      break;
    }
    case schema::IFCALIGNMENTHORIZONTAL:
    {
      _loader.MoveToArgumentOffset(expressID, 5);
      uint32_t localPlacement = 0;
      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        localPlacement = _loader.GetRefArgument();
      }

      glm::dmat4 transform_t = glm::dmat4(1);
      if (localPlacement != 0 && _loader.IsValidExpressID(localPlacement))
      {
        transform_t = GetLocalPlacement(localPlacement);
      }

      auto &relAggVector = GetRelAggregates();
      if (relAggVector.count(expressID) == 1)
      {
        auto &relAgg = relAggVector.at(expressID);
        for (auto expressID : relAgg)
        {
          alignment.Horizontal.curves.push_back(GetAlignmentCurve(expressID, sourceExpressID));
        }

        for (size_t i = 0; i < alignment.Horizontal.curves.size(); i++)
        {
          for (size_t j = 0; j < alignment.Horizontal.curves[i].points.size(); j++)
          {
            alignment.Horizontal.curves[i].points[j] =
                glm::dvec4(alignment.Horizontal.curves[i].points[j].x, alignment.Horizontal.curves[i].points[j].y, 0, 1) * transform * transform_t;
          }
        }
      }

      auto &relNestVector = GetRelNests();
      if (relNestVector.count(expressID) == 1)
      {
        auto &relNest = relNestVector.at(expressID);
        for (auto expressID : relNest)
        {
          alignment.Horizontal.curves.push_back(GetAlignmentCurve(expressID, sourceExpressID));
        }

        for (size_t i = 0; i < alignment.Horizontal.curves.size(); i++)
        {
          for (size_t j = 0; j < alignment.Horizontal.curves[i].points.size(); j++)
          {
            alignment.Horizontal.curves[i].points[j] =
                glm::dvec4(alignment.Horizontal.curves[i].points[j].x, alignment.Horizontal.curves[i].points[j].y, 0, 1) * transform * transform_t;
          }
        }
      }

      break;
    }
    case schema::IFCALIGNMENTVERTICAL:
    {
      _loader.MoveToArgumentOffset(expressID, 5);
      uint32_t localPlacement = 0;
      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        localPlacement = _loader.GetRefArgument();
      }

      glm::dmat4 transform_t = glm::dmat4(1);
      if (localPlacement != 0 && _loader.IsValidExpressID(localPlacement))
      {
        transform_t = GetLocalPlacement(localPlacement);
      }

      auto &relAggVector = GetRelAggregates();
      if (relAggVector.count(expressID) == 1)
      {
        auto &relAgg = relAggVector.at(expressID);
        for (auto expressID : relAgg)
        {
          alignment.Vertical.curves.push_back(GetAlignmentCurve(expressID, sourceExpressID));
        }

        for (size_t i = 0; i < alignment.Vertical.curves.size(); i++)
        {
          for (size_t j = 0; j < alignment.Vertical.curves[i].points.size(); j++)
          {
            alignment.Vertical.curves[i].points[j] =
                glm::dvec4(alignment.Vertical.curves[i].points[j].x, alignment.Vertical.curves[i].points[j].y, 0, 1) * transform * transform_t;
          }
        }
      }

      auto &relNestVector = GetRelNests();
      if (relNestVector.count(expressID) == 1)
      {
        auto &relNest = relNestVector.at(expressID);
        for (auto expressID : relNest)
        {
          alignment.Vertical.curves.push_back(GetAlignmentCurve(expressID, sourceExpressID));
        }

        for (size_t i = 0; i < alignment.Vertical.curves.size(); i++)
        {
          for (size_t j = 0; j < alignment.Vertical.curves[i].points.size(); j++)
          {
            alignment.Vertical.curves[i].points[j] =
                glm::dvec4(alignment.Vertical.curves[i].points[j].x, alignment.Vertical.curves[i].points[j].y, 0, 1) * transform * transform_t;
          }
        }
      }

      break;
    }
    default:
    {
      break;
    }
    }
    return alignment;
  }

  IfcCurve IfcGeometryLoader::GetAlignmentCurve(uint32_t expressID, uint32_t parentExpressID) const
  {
    spdlog::debug("[GetAlignmentCurve({})]",expressID);
    auto lineType = _loader.GetLineType(expressID);

    IfcCurve alignmentCurve;

    switch (lineType)
    {
    case schema::IFCALIGNMENTSEGMENT:
    {

      _loader.MoveToArgumentOffset(expressID, 5);
      uint32_t localPlacement = 0;
      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        localPlacement = _loader.GetRefArgument();
      }

      glm::dmat4 transform_t = glm::dmat4(1);
      if (localPlacement != 0 && _loader.IsValidExpressID(localPlacement))
      {
        transform_t = GetLocalPlacement(localPlacement);
      }

      _loader.MoveToArgumentOffset(expressID, 7);
      uint32_t curveID = 0;
      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        curveID = _loader.GetRefArgument();
      }
      if (curveID != 0 && _loader.IsValidExpressID(curveID))
      {
        IfcCurve temp = GetAlignmentCurve(curveID, parentExpressID);
        alignmentCurve = temp;
      }

      for (size_t i = 0; i < alignmentCurve.points.size(); i++)
      {
        alignmentCurve.points[i] = glm::dvec4(alignmentCurve.points[i].x, alignmentCurve.points[i].y, 0, 1) * transform_t;
      }

      break;
    }
    case schema::IFCALIGNMENTHORIZONTALSEGMENT:
    {

      _loader.MoveToArgumentOffset(expressID, 8);
      std::string_view type = _loader.GetStringArgument();

      _loader.MoveToArgumentOffset(expressID, 2);
      uint32_t ifcStartPoint = _loader.GetRefArgument();
      glm::dvec2 StartPoint = GetCartesianPoint2D(ifcStartPoint);

      _loader.MoveToArgumentOffset(expressID, 3);
      double ifcStartDirection = _loader.GetDoubleArgument();

      _loader.MoveToArgumentOffset(expressID, 4);
      double StartRadiusOfCurvature = _loader.GetDoubleArgument();

      _loader.MoveToArgumentOffset(expressID, 5);
      double EndRadiusOfCurvature = _loader.GetDoubleArgument();

      _loader.MoveToArgumentOffset(expressID, 6);
      double SegmentLength = _loader.GetDoubleArgument();

      _loader.MoveToArgumentOffset(expressID, 7);
      double GravityCenterLineHeight = _loader.GetDoubleArgument();

      std::string str(type);

      switch (Horizontal_alignment_type.at(type))
      {
      default:
      {
        break;
      }
      case 1: // LINE
      {

        IfcCurve curve;
        glm::dvec2 Direction(
            glm::cos(ifcStartDirection),
            glm::sin(ifcStartDirection));
        glm::dvec2 EndPoint = StartPoint + Direction * SegmentLength;

        curve.Add(StartPoint);
        curve.Add(EndPoint);

        alignmentCurve = curve;
        alignmentCurve.userData.push_back("ID: " + std::to_string(parentExpressID));
        alignmentCurve.userData.push_back("TYPE: " + str);
        alignmentCurve.userData.push_back("LENGHT: " + std::to_string(SegmentLength));
        alignmentCurve.userData.push_back("DIRECTION: " + std::to_string(ifcStartDirection));

        break;
      }
      case 2: // ARC
      {
        IfcCurve curve;
        double span = (SegmentLength / StartRadiusOfCurvature);
        ifcStartDirection = ifcStartDirection - (CONST_PI / 2);

        bool sw = true;
        auto curve2D = GetEllipseCurve(StartRadiusOfCurvature, StartRadiusOfCurvature, 20, glm::dmat3(1), ifcStartDirection, ifcStartDirection + span, sw);
        glm::dvec2 desp = glm::dvec2(StartPoint.x - curve2D.points[0].x, StartPoint.y - curve2D.points[0].y);

        for (size_t i = 0; i < curve2D.points.size(); i++)
        {
          curve.Add(curve2D.Get2d(i) + desp);
        }

        alignmentCurve = curve;
        alignmentCurve.userData.push_back("ID: " + std::to_string(parentExpressID));
        alignmentCurve.userData.push_back("TYPE: " + str);
        alignmentCurve.userData.push_back("RADIUS: " + std::to_string(StartRadiusOfCurvature));

        break;
      }
      case 3: // CLOTHOID
      {
        bool inverse = false;
        if (abs(StartRadiusOfCurvature) > abs(EndRadiusOfCurvature))
        {
          inverse = true;
        }
        IfcCurve curve;
        double A = sqrt(abs(EndRadiusOfCurvature - StartRadiusOfCurvature) * SegmentLength);
        double Api = A * sqrt(CONST_PI);
        double uMax = SegmentLength / Api;

        double s = A * uMax * sqrt(CONST_PI);
        double radFin = (A * A * A) / (A * s);

        double vSin = 0;
        double vCos = 0;

        glm::dvec2 DirectionX(
            glm::cos(ifcStartDirection),
            glm::sin(ifcStartDirection));
        glm::dvec2 DirectionY(
            -glm::sin(ifcStartDirection),
            glm::cos(ifcStartDirection));

        if (EndRadiusOfCurvature < 0 || StartRadiusOfCurvature < 0)
        {
          DirectionY.x = -DirectionY.x;
          DirectionY.y = -DirectionY.y;
        }

        if (inverse)
        {
          DirectionX.x = -DirectionX.x;
          DirectionX.y = -DirectionX.y;
        }

        double def = 1000;
        double dif = def / 10;
        double count = 0;
        double tram = uMax / def;
        glm::dvec2 end(0, 0);
        glm::dvec2 prev(0, 0);
        glm::dvec2 endDir;
        for (double c = 1; c < def + 1; c++)
        {
          prev = end;
          end = StartPoint + Api * (DirectionX * vCos + DirectionY * vSin);
          if (c == def || c == 1 || count >= dif)
          {
            curve.Add(end);
            count = 0;
          }
          if (c == def)
          {
            endDir = prev - end;
          }
          double val = c * tram;
          vSin += sin(CONST_PI * ((A * val * val) / (2 * abs(A)))) * tram;
          vCos += cos(CONST_PI * ((A * val * val) / (2 * abs(A)))) * tram;
          count++;
        }

        if (inverse)
        {
          DirectionX.x = -DirectionX.x;
          DirectionX.y = -DirectionX.y;

          glm::dvec2 newDirectionX(
              endDir.x,
              endDir.y);
          glm::dvec2 newDirectionY(
              -endDir.y,
              endDir.x);

          if (EndRadiusOfCurvature < 0 || StartRadiusOfCurvature < 0)
          {
            newDirectionY.x = -newDirectionY.x;
            newDirectionY.y = -newDirectionY.y;
          }

          newDirectionX = glm::normalize(newDirectionX);
          newDirectionY = glm::normalize(newDirectionY);

          for (uint32_t i = 0; i < curve.points.size(); i++)
          {
            double xx = curve.points[i].x - end.x;
            double yy = curve.points[i].y - end.y;
            double dx = xx * newDirectionX.x + yy * newDirectionX.y;
            double dy = xx * newDirectionY.x + yy * newDirectionY.y;
            double newDx = StartPoint.x + DirectionX.x * dx + DirectionY.x * dy;
            double newDy = StartPoint.y + DirectionX.y * dx + DirectionY.y * dy;
            curve.points[i].x = newDx;
            curve.points[i].y = newDy;
          }
        }

        alignmentCurve = curve;
        alignmentCurve.userData.push_back("ID: " + std::to_string(parentExpressID));
        alignmentCurve.userData.push_back("TYPE: " + str);
        alignmentCurve.userData.push_back("START RADIUS: " + std::to_string(StartRadiusOfCurvature));
        alignmentCurve.userData.push_back("END RADIUS: " + std::to_string(EndRadiusOfCurvature));
        alignmentCurve.userData.push_back("A: " + std::to_string(A));

        break;
      }
      case 4: // CUBIC
      {
        break;
      }
      case 5:
      {
        break;
      }
      case 6:
      {
        break;
      }
      case 7:
      {
        break;
      }
      case 8:
      {
        break;
      }
      case 9:
      {
        break;
      }
      }

      break;
    }
    case schema::IFCALIGNMENTVERTICALSEGMENT:
    {
      _loader.MoveToArgumentOffset(expressID, 2);
      double StartDistAlong = _loader.GetDoubleArgument();

      _loader.MoveToArgumentOffset(expressID, 3);
      double HorizontalLength = _loader.GetDoubleArgument();

      _loader.MoveToArgumentOffset(expressID, 4);
      double StartHeight = _loader.GetDoubleArgument();

      _loader.MoveToArgumentOffset(expressID, 5);
      double StartGradient = _loader.GetDoubleArgument();

      _loader.MoveToArgumentOffset(expressID, 6);
      double EndGradient = _loader.GetDoubleArgument();

      _loader.MoveToArgumentOffset(expressID, 7);
      double RadiusOfCurvature = _loader.GetDoubleArgument();

      _loader.MoveToArgumentOffset(expressID, 8);
      std::string_view type = _loader.GetStringArgument();

      IfcProfile profile;

      std::string str(type);

      switch (Vertical_alignment_type.at(type))
      {
      default:
      {
        break;
      }
      case 1: // CONSTANTGRADIENT
      {
        IfcCurve curve;

        glm::dvec3 iPoint = glm::dvec3(StartDistAlong, StartHeight, 1);
        glm::dvec3 jPoint = glm::dvec3(StartDistAlong + HorizontalLength, StartHeight + HorizontalLength * StartGradient, 1);
        glm::dvec3 Normal = glm::dvec3(0, 0, 1);

        curve.Add(iPoint);
        curve.Add(jPoint);

        alignmentCurve = curve;
        alignmentCurve.userData.push_back("ID: " + std::to_string(parentExpressID));
        alignmentCurve.userData.push_back("TYPE: " + str);
        alignmentCurve.userData.push_back("START HEIGHT: " + std::to_string(StartHeight));
        alignmentCurve.userData.push_back("START GRADIENT: " + std::to_string(StartGradient));
        break;
      }
      case 2: // ARC
      {
        IfcCurve curve;

        double ifcStartDirection = atan(StartGradient);
        double ifcEndDirection = atan(EndGradient);

        ifcStartDirection = ifcStartDirection - (CONST_PI / 2);
        ifcEndDirection = ifcEndDirection - (CONST_PI / 2);

        glm::dvec2 StartPoint(StartDistAlong, StartHeight);

        bool sw = true;
        if (ifcStartDirection > ifcEndDirection)
        {
          ifcStartDirection = ifcStartDirection + CONST_PI;
          ifcEndDirection = ifcEndDirection + CONST_PI;
        }
        auto curve2D = GetEllipseCurve(RadiusOfCurvature, RadiusOfCurvature, _circleSegments, glm::dmat3(1), ifcStartDirection, ifcEndDirection, sw);
        glm::dvec2 desp = glm::dvec2(StartPoint.x - curve2D.points[0].x, StartPoint.y - curve2D.points[0].y);

        for (size_t i = 0; i < curve2D.points.size(); i++)
        {

          curve.Add(curve2D.Get2d(i) + desp);
        }

        alignmentCurve = curve;
        alignmentCurve.userData.push_back("ID: " + std::to_string(parentExpressID));
        alignmentCurve.userData.push_back("TYPE: " + str);
        alignmentCurve.userData.push_back("RADIUS: " + std::to_string(RadiusOfCurvature));
        alignmentCurve.userData.push_back("START GRADIENT: " + std::to_string(StartGradient));
        alignmentCurve.userData.push_back("END GRADIENT: " + std::to_string(EndGradient));

        break;
      }
      case 3: // PARABOLIC
      {
        IfcCurve curve;

        glm::dvec2 StartPoint(StartDistAlong, StartHeight);

        double R = HorizontalLength / (EndGradient - StartGradient);

        std::vector<glm::dvec2> points;

        for (double i = 0; i <= _circleSegments; i++)
        {
          double pr = i / _circleSegments;
          double grad = ((HorizontalLength * pr) / R) + StartGradient;
          double alt = (HorizontalLength * pr * (grad + StartGradient) * 0.5) + StartHeight;
          points.push_back(glm::dvec2(HorizontalLength * pr, alt));
        }

        glm::dvec2 desp = glm::dvec2(StartPoint.x - points[0].x, StartPoint.y - points[0].y);

        for (auto &pt2D : points)
        {
          curve.Add(pt2D + desp);
        }

        alignmentCurve = curve;
        alignmentCurve.userData.push_back("ID: " + std::to_string(parentExpressID));
        alignmentCurve.userData.push_back("TYPE: " + str);
        alignmentCurve.userData.push_back("LENGHT: " + std::to_string(HorizontalLength));
        alignmentCurve.userData.push_back("START GRADIENT: " + std::to_string(StartGradient));
        alignmentCurve.userData.push_back("END GRADIENT: " + std::to_string(EndGradient));
        alignmentCurve.userData.push_back("R: " + std::to_string(R));

        break;
      }
      }
      break;
    }
    }

    return alignmentCurve;
  }

  std::optional<glm::dvec4> IfcGeometryLoader::GetColor(uint32_t expressID) const
  {
   spdlog::debug("[GetColor({})]",expressID);
    auto lineType = _loader.GetLineType(expressID);
    switch (lineType)
    {
    case schema::IFCPRESENTATIONSTYLEASSIGNMENT:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      auto ifcPresentationStyleSelects = _loader.GetSetArgument();

      for (auto &styleSelect : ifcPresentationStyleSelects)
      {
        uint32_t styleSelectID = _loader.GetRefArgument(styleSelect);
        auto foundColor = GetColor(styleSelectID);
        if (foundColor)
          return foundColor;
      }

      return {};
    }
    case schema::IFCDRAUGHTINGPREDEFINEDCOLOUR:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      std::string_view color = _loader.GetStringArgument();
      if (color == "black")
        return glm::dvec4(0.0, 0.0, 0.0, 1.0);
      else if (color == "red")
        return glm::dvec4(1.0, 0, 0, 1.0);
      else if (color == "green")
        return glm::dvec4(0, 1.0, 0, 1.0);
      else if (color == "blue")
        return glm::dvec4(0, 0, 1.0, 1.0);
      else if (color == "yellow")
        return glm::dvec4(1.0, 1.0, 0, 1.0);
      else if (color == "magenta")
        return glm::dvec4(1.0, 0, 1.0, 1.0);
      else if (color == "cyan")
        return glm::dvec4(0, 1.0, 1.0, 1.0);
      else if (color == "white")
        return glm::dvec4(1.0, 1.0, 1.0, 1.0);
      return {};
    }
    case schema::IFCCURVESTYLE:
    {
      _loader.MoveToArgumentOffset(expressID, 3);
      auto foundColor = GetColor(_loader.GetRefArgument());
      if (foundColor)
        return foundColor;
      return {};
    }
    case schema::IFCFILLAREASTYLEHATCHING:
    {
      // we cannot properly support this but for now use its colour as solid
      _loader.MoveToArgumentOffset(expressID, 0);
      auto foundColor = GetColor(_loader.GetRefArgument());
      if (foundColor)
        return foundColor;
      return {};
    }
    case schema::IFCSURFACESTYLE:
    {
      _loader.MoveToArgumentOffset(expressID, 2);
      auto ifcSurfaceStyleElementSelects = _loader.GetSetArgument();

      for (auto &styleElementSelect : ifcSurfaceStyleElementSelects)
      {
        uint32_t styleElementSelectID = _loader.GetRefArgument(styleElementSelect);
        auto foundColor = GetColor(styleElementSelectID);
        if (foundColor)
          return foundColor;
      }

      return {};
    }
    case schema::IFCSURFACESTYLERENDERING:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      auto outputColor = GetColor(_loader.GetRefArgument());
      _loader.MoveToArgumentOffset(expressID, 1);

      if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
      {
        _loader.StepBack();
        outputColor.value().a = 1 - _loader.GetDoubleArgument();
      }

      return outputColor;
    }
    case schema::IFCSURFACESTYLESHADING:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      return GetColor(_loader.GetRefArgument());
    }
    case schema::IFCSTYLEDREPRESENTATION:
    {
      _loader.MoveToArgumentOffset(expressID, 3);
      auto repItems = _loader.GetSetArgument();

      for (auto &repItem : repItems)
      {
        uint32_t repItemID = _loader.GetRefArgument(repItem);
        auto foundColor = GetColor(repItemID);
        if (foundColor)
          return foundColor;
      }

      return {};
    }
    case schema::IFCSTYLEDITEM:
    {
      _loader.MoveToArgumentOffset(expressID, 1);
      auto styledItems = _loader.GetSetArgument();

      for (auto &styledItem : styledItems)
      {
        uint32_t styledItemID = _loader.GetRefArgument(styledItem);
        auto foundColor = GetColor(styledItemID);
        if (foundColor)
          return foundColor;
      }

      return {};
    }
    case schema::IFCCOLOURRGB:
    {
      _loader.MoveToArgumentOffset(expressID, 1);
      glm::dvec4 outputColor;
      outputColor.r = _loader.GetDoubleArgument();
      outputColor.g = _loader.GetDoubleArgument();
      outputColor.b = _loader.GetDoubleArgument();
      outputColor.a = 1;

      return outputColor;
    }
    case schema::IFCMATERIALLAYERSETUSAGE:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      uint32_t layerSetID = _loader.GetRefArgument();
      return GetColor(layerSetID);
    }
    case schema::IFCMATERIALLAYERSET:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      auto layers = _loader.GetSetArgument();

      for (auto &layer : layers)
      {
        uint32_t layerID = _loader.GetRefArgument(layer);
        auto foundColor = GetColor(layerID);
        if (foundColor)
          return foundColor;
      }

      return {};
    }
    case schema::IFCMATERIALLAYER:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      uint32_t matRepID = _loader.GetRefArgument();
      return GetColor(matRepID);
    }
    case schema::IFCMATERIAL:
    {
      if (GetMaterialDefinitions().count(expressID) != 0)
      {
        auto &defs = GetMaterialDefinitions().at(expressID);
        for (auto def : defs)
        {
          auto success = GetColor(def.second);
          if (success)
            return success;
        }

        return {};
      }
      return {};
    }
    case schema::IFCFILLAREASTYLE:
    {
      _loader.MoveToArgumentOffset(expressID, 1);
      auto ifcFillStyleSelects = _loader.GetSetArgument();

      for (auto &styleSelect : ifcFillStyleSelects)
      {
        uint32_t styleSelectID = _loader.GetRefArgument(styleSelect);
        auto foundColor = GetColor(styleSelectID);
        if (foundColor)
          return foundColor;
      }

      return {};
    }
    case schema::IFCMATERIALLIST:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      auto materials = _loader.GetSetArgument();

      std::optional<glm::dvec4> lastColor;
      bool result = false;
      for (auto &material : materials)
      {
        uint32_t materialID = _loader.GetRefArgument(material);
        auto foundColor = GetColor(materialID);
        if (foundColor)
          lastColor = foundColor;
          result = true;
      }
      if(result)
      {
        return lastColor;
      }
      else
      {
        return {};
      }    
    }
    case schema::IFCMATERIALCONSTITUENTSET:
    {
      _loader.MoveToArgumentOffset(expressID, 2);
      auto materialContituents = _loader.GetSetArgument();

      for (auto &materialContituent : materialContituents)
      {
        uint32_t materialContituentID = _loader.GetRefArgument(materialContituent);
        auto foundColor = GetColor(materialContituentID);
        if (foundColor)
          return foundColor;
      }
      return {};
    }
    case schema::IFCMATERIALCONSTITUENT:
    {
      _loader.MoveToArgumentOffset(expressID, 2);
      auto material = _loader.GetRefArgument();
      auto foundColor = GetColor(material);
      if (foundColor)
        return foundColor;
      return {};
    }
    case schema::IFCMATERIALPROFILESETUSAGE:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      auto profileSet = _loader.GetRefArgument();
      auto foundColor = GetColor(profileSet);
      if (foundColor)
        return foundColor;
      return {};
    }
    case schema::IFCMATERIALPROFILE:
    {
      _loader.MoveToArgumentOffset(expressID, 2);
      auto profileSet = _loader.GetRefArgument();
      auto foundColor = GetColor(profileSet);
      if (foundColor)
        return foundColor;
      return {};
    }
    case schema::IFCMATERIALPROFILESET:
    {
      _loader.MoveToArgumentOffset(expressID, 2);
      auto materialProfiles = _loader.GetSetArgument();

      for (auto &materialProfile : materialProfiles)
      {
        uint32_t materialProfileID = _loader.GetRefArgument(materialProfile);
        auto foundColor = GetColor(materialProfileID);
        if (foundColor)
          return foundColor;
      }
      return {};
    }
    default:
      spdlog::error("[GetColor()] unexpected style type {}", expressID, lineType);
      break;
    }

    return {};
  }

  IfcBound3D IfcGeometryLoader::GetBound(uint32_t expressID) const
  {
    spdlog::debug("[GetBound({})]",expressID);
    auto lineType = _loader.GetLineType(expressID);

    switch (lineType)
    {
    case schema::IFCFACEOUTERBOUND:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      uint32_t loop = _loader.GetRefArgument();
      _loader.MoveToArgumentOffset(expressID, 1);
      std::string_view orientValue = _loader.GetStringArgument();
      bool orient = orientValue == "T";

      IfcBound3D bound;
      bound.curve = GetLoop(loop);
      bound.orientation = orient;
      bound.type = IfcBoundType::OUTERBOUND;

      if (!orient)
      {
        std::reverse(bound.curve.points.begin(), bound.curve.points.end());
      }

      return bound;
    }
    case schema::IFCFACEBOUND:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      uint32_t loop = _loader.GetRefArgument();
      _loader.MoveToArgumentOffset(expressID, 1);
      std::string_view orientValue = _loader.GetStringArgument();
      bool orient = orientValue == "T";

      IfcBound3D bound;
      bound.curve = GetLoop(loop);
      bound.orientation = orient;
      bound.type = IfcBoundType::BOUND;

      if (!orient)
      {
        std::reverse(bound.curve.points.begin(), bound.curve.points.end());
      }

      return bound;
    }
    default:
      spdlog::error("[(GetBounds)] unexpected bound type {}", expressID, lineType);
      break;
    }

    return IfcBound3D();
  }

  IfcCurve IfcGeometryLoader::GetLoop(uint32_t expressID) const
  {
    spdlog::debug("[GetLoop({})]",expressID);
    auto lineType = _loader.GetLineType(expressID);

    switch (lineType)
    {
    case schema::IFCPOLYLOOP:
    {
      IfcCurve curve;

      _loader.MoveToArgumentOffset(expressID, 0);
      auto points = _loader.GetSetArgument();

      curve.points.reserve(points.size());

      uint32_t prevID = 0;
      for (auto &token : points)
      {
        uint32_t pointId = _loader.GetRefArgument(token);

        // trim out consecutive equal points
        if (pointId != prevID)
        {
          curve.points.push_back(GetCartesianPoint3D(pointId));
        }

        prevID = pointId;
      }

      return curve;
    }
    case schema::IFCEDGELOOP:
    {
      IfcCurve curve;

      _loader.MoveToArgumentOffset(expressID, 0);
      auto edges = _loader.GetSetArgument();
      int id = 0;

      for (auto &token : edges)
      {
        uint32_t edgeId = _loader.GetRefArgument(token);
        IfcCurve edgeCurve = GetOrientedEdge(edgeId);

        // Important not to repeat the last point otherwise triangulation fails
        // if the list has zero points this is initial, no repetition is possible, otherwise we must check
        if (curve.points.size() == 0)
        {
          for (auto &pt : edgeCurve.points)
          {
            curve.points.push_back(pt);
            curve.indices.push_back(id);
          }
        }
        else
        {
          for (auto &pt : edgeCurve.points)
          {
            if (notPresent(pt, curve.points))
            {
              curve.points.push_back(pt);
              curve.indices.push_back(id);
            }
          }
        }
        id++;
      }

      return curve;
    }
    default:
      spdlog::error("[GetLoop()] unexpected loop type {}", expressID, lineType);
      break;
    }

    IfcCurve curve;
    return curve;
  }

  IfcCurve IfcGeometryLoader::GetOrientedEdge(uint32_t expressID) const
  {
    spdlog::debug("[GetOrientedEdge({})]",expressID);
    _loader.MoveToArgumentOffset(expressID, 3);
    std::string_view orientValue = _loader.GetStringArgument();
    bool orient = orientValue == "T";
    _loader.MoveToArgumentOffset(expressID, 2);
    uint32_t edgeCurveRef = _loader.GetRefArgument();
    IfcCurve curveEdge = GetEdge(edgeCurveRef);

    // Read edgeCurve

    if (orient)
    {
      std::reverse(curveEdge.points.begin(), curveEdge.points.end());
    }

    return curveEdge;
  }

  glm::dvec3 IfcGeometryLoader::GetVertexPoint(uint32_t expressID) const
  {
    spdlog::debug("[GetVertexPoint({})]",expressID);
    _loader.MoveToArgumentOffset(expressID, 0);
    uint32_t pointRef = _loader.GetRefArgument();
    auto point = _loader.GetLineType(pointRef);
    if (point == schema::IFCCARTESIANPOINT)
    {
      return GetCartesianPoint3D(pointRef);
    }
    else
    {
      spdlog::error("[GetVertexPoint()] unexpected vertxpoint type {}", pointRef, point);
      return {};
    }
  }

  IfcCurve IfcGeometryLoader::GetEdge(uint32_t expressID) const
  {
    spdlog::debug("[GetEdge({})]",expressID);
    auto lineType = _loader.GetLineType(expressID);

    switch (lineType)
    {
    case schema::IFCEDGECURVE:
    {
      IfcTrimmingArguments ts;
      ts.exist = true;
      _loader.MoveToArgumentOffset(expressID, 0);
      glm::dvec3 p1 = GetVertexPoint(_loader.GetRefArgument());
      _loader.MoveToArgumentOffset(expressID, 1);
      glm::dvec3 p2 = GetVertexPoint(_loader.GetRefArgument());
      ts.start.pos3D = p1;
      ts.start.hasPos = true;
      ts.end.hasPos = true;
      ts.end.pos3D = p2;

      _loader.MoveToArgumentOffset(expressID, 2);
      uint32_t CurveRef = _loader.GetRefArgument();
      IfcCurve curve;
      ComputeCurve(CurveRef, curve, 3, true, -1, -1, ts);

      return curve;
    }
    default:
      spdlog::error("[GetEdge())] unexpected edgecurve type {}", expressID, lineType);
      break;
    }
    return IfcCurve();
  }

  IfcTrimmingSelect IfcGeometryLoader::GetTrimSelect(uint32_t DIM, std::vector<uint32_t> &tapeOffsets) const
  {
    IfcTrimmingSelect ts;

    for (size_t i = 0; i < tapeOffsets.size(); i++)
    {
      auto tokenType = _loader.GetTokenType(tapeOffsets[i]);

      _loader.StepBack();
      if (tokenType == parsing::IfcTokenType::REF)
      {
        // caresian point
        uint32_t cartesianPointRef = _loader.GetRefArgument();
        ts.hasPos = true;
        if (DIM == 2)
        {
          ts.pos = GetCartesianPoint2D(cartesianPointRef);
        }
        if (DIM == 3)
        {
          ts.pos3D = GetCartesianPoint3D(cartesianPointRef);
        }
      }
      else if (tokenType == parsing::IfcTokenType::LABEL)
      {
        // parametervalue
        std::string_view type = _loader.GetStringArgument();

        if (type == "IFCPARAMETERVALUE")
        {
          ts.hasParam = true;
          i++;
          ts.param = _loader.GetDoubleArgument(tapeOffsets[i]);
        }
      }
    }

    return ts;
  }

  glm::dvec3 IfcGeometryLoader::GetCartesianPoint3D(const uint32_t expressID) const
  {
    spdlog::debug("[GetCartesianPoint3D({})]",expressID);
    _loader.MoveToArgumentOffset(expressID, 0);
    _loader.GetTokenType();
    // because these calls cannot be reordered we have to use intermediate variables
    double x = _loader.GetDoubleArgument();
    double y = _loader.GetDoubleArgument();
    double z = _loader.GetOptionalDoubleParam(0);
    glm::dvec3 point(x, y, z);
    return point;
  }

  glm::dvec2 IfcGeometryLoader::GetCartesianPoint2D(const uint32_t expressID) const
  {
   spdlog::debug("[GetCartesianPoint2D({})]",expressID);
    _loader.MoveToArgumentOffset(expressID, 0);
    _loader.GetTokenType();
    // because these calls cannot be reordered we have to use intermediate variables
    double x = _loader.GetDoubleArgument();
    double y = _loader.GetDoubleArgument();
    glm::dvec2 point(x, y);
    return point;
  }

  bool IfcGeometryLoader::ReadIfcCartesianPointList(uint32_t expressID) const
  {
    spdlog::debug("[ReadIfcCartesianPointList({})]",expressID);  
    auto listType = _loader.GetLineType(expressID);
    _loader.MoveToArgumentOffset(expressID, 0);

    if (listType == schema::IFCCARTESIANPOINTLIST3D)
    {
      return false;
    }
    if (listType == schema::IFCCARTESIANPOINTLIST2D)
    {
      return true;
    }
  }

  std::vector<glm::dvec3> IfcGeometryLoader::ReadIfcCartesianPointList3D(uint32_t expressID) const
  {
    spdlog::debug("[ReadIfcCartesianPointList3D({})]",expressID);
    _loader.MoveToArgumentOffset(expressID, 0);

    std::vector<glm::dvec3> result;

    _loader.GetTokenType();

    // while we have point set begin
    while (_loader.GetTokenType() == parsing::IfcTokenType::SET_BEGIN)
    {
      // because these calls cannot be reordered we have to use intermediate variables
      double x = _loader.GetDoubleArgument();
      double y = _loader.GetDoubleArgument();
      double z = _loader.GetDoubleArgument();
      result.emplace_back(x, y, z);

      // read point set end
      _loader.GetTokenType();
    }

    return result;
  }

  std::vector<glm::dvec2> IfcGeometryLoader::ReadIfcCartesianPointList2D(uint32_t expressID) const
  {
    spdlog::debug("[ReadIfcCartesianPointList2D({})]",expressID);
    _loader.MoveToArgumentOffset(expressID, 0);

    std::vector<glm::dvec2> result;

    _loader.GetTokenType();

    // while we have point set begin
    while (_loader.GetTokenType() == parsing::IfcTokenType::SET_BEGIN)
    {
      // because these calls cannot be reordered we have to use intermediate variables
      double x = _loader.GetDoubleArgument();
      double y = _loader.GetDoubleArgument();
      result.emplace_back(x, y);

      // read point set end
      _loader.GetTokenType();
    }

    return result;
  }

  IfcCurve IfcGeometryLoader::GetCurve(uint32_t expressID, uint8_t dimensions,bool edge) const
  {
    spdlog::debug("[GetCurve({})]",expressID);
    IfcCurve curve;
    ComputeCurve(expressID, curve, dimensions,  edge);
    return curve;
  }

  void IfcGeometryLoader::ComputeCurve(uint32_t expressID, IfcCurve &curve, uint8_t dimensions, bool edge, int sameSense, int trimSense , IfcTrimmingArguments trim) const
  {
    spdlog::debug("[ComputeCurve({})]",expressID);
    auto lineType = _loader.GetLineType(expressID);
    switch (lineType)
    {
    case schema::IFCPOLYLINE:
      {
        _loader.MoveToArgumentOffset(expressID, 0);
        auto points = _loader.GetSetArgument();

        for (auto &token : points)
        {
          uint32_t pointId = _loader.GetRefArgument(token);
          if (dimensions == 2) curve.Add(GetCartesianPoint2D(pointId));
          else curve.Add(GetCartesianPoint3D(pointId)); 
          
        }

        if (edge)
        {
          if (sameSense == 1 || sameSense == -1)
          {
            std::reverse(curve.points.begin(), curve.points.end());
          }
        }

        break;
      }
    case schema::IFCCOMPOSITECURVE:
      {
        _loader.MoveToArgumentOffset(expressID, 0);
        auto segments = _loader.GetSetArgument();
        auto selfIntersects = _loader.GetStringArgument();

        if (selfIntersects == "T")
        {
          // TODO: this is probably bad news
          spdlog::error("[ComputeCurve()] Self intersecting composite curve {}", expressID);
        }

        for (auto &token : segments)
        {
          // DEBUG
          // #ifdef DEBUG_DUMP_SVG
          //     io::DumpSVGCurve(curve.points, "partial_curve.html");
          // #endif

          uint32_t segmentId = _loader.GetRefArgument(token);

          ComputeCurve(segmentId, curve, dimensions, edge, sameSense, trimSense);
        }

        break;
      }
    case schema::IFCCOMPOSITECURVESEGMENT:
      {
        _loader.MoveToArgumentOffset(expressID, 0);
        auto transition = _loader.GetStringArgument();
        auto sameSenseS = _loader.GetStringArgument();
        auto parentID = _loader.GetRefArgument();

        bool sameSense = sameSenseS == "T";

        ComputeCurve(parentID, curve, dimensions,  edge, sameSense, trimSense);

        break;
      }

      // TODO: review and simplify
    case schema::IFCLINE:
      {
        bool condition = sameSense == 1 || sameSense == -1;
        if (edge)
        {
          condition = !condition;
        }
        if (dimensions== 2 && trim.exist)
        {
          if (trim.start.hasPos && trim.end.hasPos)
          {

            if (condition)
            {
              curve.Add(trim.start.pos);
              curve.Add(trim.end.pos);
            }
            else
            {
              curve.Add(trim.end.pos);
              curve.Add(trim.start.pos);
            }
          }
          else if (trim.start.hasParam && trim.end.hasParam)
          {
            _loader.MoveToArgumentOffset(expressID, 0);
            auto positionID = _loader.GetRefArgument();
            auto vectorID = _loader.GetRefArgument();
            glm::dvec3 placement = glm::dvec3(GetCartesianPoint2D(positionID), 0);
            glm::dvec3 vector;
            vector = GetVector(vectorID);

            if (condition)
            {
              glm::dvec3 p1 = placement + vector * trim.start.param;
              glm::dvec3 p2 = placement + vector * trim.end.param;
              curve.Add(p1);
              curve.Add(p2);
            }
            else
            {
              glm::dvec3 p2 = placement + vector * trim.start.param;
              glm::dvec3 p1 = placement + vector * trim.end.param;
              curve.Add(p1);
              curve.Add(p2);
            }
          }
          else
          {
            spdlog::error("[ComputeCurve()] Unsupported trimmingselect 2D IFCLINE {}", expressID, lineType);
          }
        }
        else if (dimensions == 3 && trim.exist)
        {
          if (trim.start.hasPos && trim.end.hasPos)
          {
            if (condition)
            {
              curve.Add(trim.start.pos3D);
              curve.Add(trim.end.pos3D);
            }
            else
            {
              curve.Add(trim.end.pos3D);
              curve.Add(trim.start.pos3D);
            }
          }
          else if (trim.start.hasParam && trim.end.hasParam)
          {
            _loader.MoveToArgumentOffset(expressID, 0);
            auto positionID = _loader.GetRefArgument();
            auto vectorID = _loader.GetRefArgument();
            glm::dvec3 placement = GetCartesianPoint3D(positionID);
            glm::dvec3 vector;
            vector = GetVector(vectorID);

            if (condition)
            {
              glm::dvec3 p1 = placement + vector * trim.start.param;
              glm::dvec3 p2 = placement + vector * trim.end.param;
              curve.Add(p1);
              curve.Add(p2);
            }
            else
            {
              glm::dvec3 p2 = placement + vector * trim.start.param;
              glm::dvec3 p1 = placement + vector * trim.end.param;
              curve.Add(p1);
              curve.Add(p2);
            }
          }
          else
          {
            spdlog::error("[ComputeCurve()] Unsupported trimmingselect 3D IFCLINE {}", expressID, lineType);
          }
        }
      break;
      }
    case schema::IFCTRIMMEDCURVE:
      {
        _loader.MoveToArgumentOffset(expressID, 0);
        auto basisCurveID = _loader.GetRefArgument();
        auto trim1Set = _loader.GetSetArgument();
        auto trim2Set = _loader.GetSetArgument();

        auto senseAgreementS = _loader.GetStringArgument();
        auto trimmingPreference = _loader.GetStringArgument();

        auto trim1 = GetTrimSelect(dimensions, trim1Set);
        auto trim2 = GetTrimSelect(dimensions, trim2Set);

        IfcTrimmingArguments trim;

        trim.exist = true;
        trim.start = trim1;
        trim.end = trim2;

        bool senseAgreement = senseAgreementS == "T";

        if (trimSense == 0)
        {
          senseAgreement = !senseAgreement;
        }

        ComputeCurve(basisCurveID, curve, dimensions,  edge, sameSense, senseAgreement, trim);

        break;
      }
    case schema::IFCINDEXEDPOLYCURVE:
      {
        _loader.MoveToArgumentOffset(expressID, 0);
        auto ptsRef = _loader.GetRefArgument();

        _loader.MoveToArgumentOffset(expressID, 2);

        if (_loader.GetTokenType() != parsing::IfcTokenType::EMPTY)
        {
          _loader.StepBack();
          auto selfIntersects = _loader.GetStringArgument();

          if (selfIntersects == "T")
          {
            // TODO: this is probably bad news
            spdlog::error("[ComputeCurve()] Self intersecting ifcindexedpolycurve {}", expressID);
          }
        }

        if (ReadIfcCartesianPointList(ptsRef))
        {
          _loader.MoveToArgumentOffset(expressID, 1);
          if (_loader.GetTokenType() != parsing::IfcTokenType::EMPTY)
          {
            auto pnSegment = ReadCurveIndices();
            for (auto &sg : pnSegment)
            {
              if (sg.type == "IFCLINEINDEX")
              {

                auto pts = ReadIfcCartesianPointList2D(ptsRef);
                for (auto &pt : sg.indexs)
                {
                  curve.Add(pts[pt - 1]);
                }
              }
              if (sg.type == "IFCARCINDEX")
              {
                auto pts = ReadIfcCartesianPointList2D(ptsRef);
                IfcCurve arc = BuildArc3Pt(pts[sg.indexs[0] - 1], pts[sg.indexs[1] - 1], pts[sg.indexs[2] - 1],_circleSegments);
                for (auto &pt : arc.points)
                {
                  curve.Add(pt);
                }
              }
            }
          }
          else
          {
            auto pts = ReadIfcCartesianPointList2D(ptsRef);
            for (auto &pt : pts)
            {
              curve.Add(pt);
            }
          }
        }
        else if (!ReadIfcCartesianPointList(ptsRef))
        {
          _loader.MoveToArgumentOffset(expressID, 1);
          if (_loader.GetTokenType() != parsing::IfcTokenType::EMPTY)
          {
            auto pnSegment = ReadCurveIndices();
            for (auto &sg : pnSegment)
            {
              if (sg.type == "IFCLINEINDEX")
              {
                auto pts = ReadIfcCartesianPointList3D(ptsRef);
                for (auto &pt : sg.indexs)
                {
                  curve.Add(pts[pt - 1]);
                }
              }
              if (sg.type == "IFCARCINDEX")
              {
                auto pts = ReadIfcCartesianPointList3D(ptsRef);
                IfcCurve arc = Build3DArc3Pt(pts[sg.indexs[0] - 1], pts[sg.indexs[1] - 1], pts[sg.indexs[2] - 1],_circleSegments, EPS_MINISCULE);
                for (auto &pt : arc.points)
                {
                  curve.Add(pt);
                }
              }
            }
          }
          else
          {
            auto pts = ReadIfcCartesianPointList3D(ptsRef);
            for (auto &pt : pts)
            {
              curve.Add(pt);
            }
          }
        }
        else
        {
          spdlog::error("[ComputeCurve()] Parsing ifcindexedpolycurve in 3D is not possible {}", expressID);
        }

        break;
      }

      case schema::IFCELLIPSE:
      case schema::IFCCIRCLE:
      {
        _loader.MoveToArgumentOffset(expressID, 0);
        auto positionID = _loader.GetRefArgument();

        // TODO: Bad solution, must add a method to define dimensions 2 or 3 in all cases not only circles
        auto typePlacement = _loader.GetLineType(positionID);
        if(typePlacement == schema::IFCAXIS2PLACEMENT3D)
        {
          dimensions = 3;
        }

        _loader.MoveToArgumentOffset(expressID, 0);
        positionID = _loader.GetRefArgument();

        double radius1 = 0;
        double radius2 = 0;

        if(lineType == schema::IFCCIRCLE)
        {
          radius1 = _loader.GetDoubleArgument();
          radius2 = radius1;
        }
        if(lineType == schema::IFCELLIPSE)
        {
          radius1 = _loader.GetDoubleArgument();
          radius2 = _loader.GetDoubleArgument();
        }

        double startDegrees = 0;
        double endDegrees = 360;

        bool byPos = false;
        bool byParam = false;

        if (trim.exist)
        {
          if (trim.start.hasParam && trim.end.hasParam)
          {
            byParam = true;
            startDegrees = trim.start.param;
            endDegrees = trim.end.param;
          }
          else if (trim.start.hasPos && trim.end.hasPos)
          {
            byPos = true;
            if (dimensions == 2)
            {
              glm::dmat3 placement = GetAxis2Placement2D(positionID);
              double xx = trim.start.pos.x - placement[2].x;
              double yy = trim.start.pos.y - placement[2].y;
              startDegrees = VectorToAngle2D(xx, yy);
              xx = trim.end.pos.x - placement[2].x;
              yy = trim.end.pos.y - placement[2].y;
              endDegrees = VectorToAngle2D(xx, yy);
            }
            else if (dimensions == 3)
            {
              glm::dmat4 placement = GetLocalPlacement(positionID);
              glm::dvec4 vecX = placement[0];
              glm::dvec4 vecY = placement[1];
              glm::dvec4 vecZ = placement[2];

              glm::dvec3 v1 = glm::dvec3(
                  trim.start.pos3D.x - placement[3].x,
                  trim.start.pos3D.y - placement[3].y,
                  trim.start.pos3D.z - placement[3].z);
              glm::dvec3 v2 = glm::dvec3(
                  trim.end.pos3D.x - placement[3].x,
                  trim.end.pos3D.y - placement[3].y,
                  trim.end.pos3D.z - placement[3].z);

              double dxS = vecX.x * v1.x + vecX.y * v1.y + vecX.z * v1.z;
              double dyS = vecY.x * v1.x + vecY.y * v1.y + vecY.z * v1.z;
              // double dzS = vecZ.x * v1.x + vecZ.y * v1.y + vecZ.z * v1.z;

              double dxE = vecX.x * v2.x + vecX.y * v2.y + vecX.z * v2.z;
              double dyE = vecY.x * v2.x + vecY.y * v2.y + vecY.z * v2.z;
              // double dzE = vecZ.x * v2.x + vecZ.y * v2.y + vecZ.z * v2.z;

              endDegrees = VectorToAngle(dxS, dyS) - 90;
              startDegrees = VectorToAngle(dxE, dyE) - 90;
            }
          }
        }

        double startRad = startDegrees;
        double endRad = endDegrees;

        convertAngleUnits(startDegrees, startRad);
        convertAngleUnits(endDegrees, endRad);

        while (startDegrees < 0)
        {
          startDegrees += 360;
          startRad += 2 * CONST_PI;
        }

        while (endDegrees < 0)
        {
          endDegrees += 360;
          endRad += 2 * CONST_PI;
        }

        while (startDegrees > 360)
        {
          startDegrees -= 360;
          startRad -= 2 * CONST_PI;
        }

        while (endDegrees > 360)
        {
          endDegrees -= 360;
          endRad -= 2 * CONST_PI;
        }

        double lengthDegrees = 0;

        if (trimSense == 1 || trimSense == -1)
        {
          if (startDegrees >= endDegrees)
          {
            endDegrees += 360;
            endRad += 2 * CONST_PI;
          }
          lengthDegrees = endDegrees - startDegrees; 
        }

        if (trimSense == 0)
        {
          if (startDegrees <= endDegrees)
          {
            startDegrees += 360;
            startRad += 2 * CONST_PI;
          }
          lengthDegrees = endDegrees - startDegrees;
        }

        double lengthRad = lengthDegrees / 180 * CONST_PI;

        size_t startIndex = curve.points.size();

        for (int i = 0; i < _circleSegments; i++)
        {
          double ratio = static_cast<double>(i) / (_circleSegments - 1);
          double angle = 0;
          angle = startRad + ratio * lengthRad;

          if(sameSense == 0)
          {
            angle = startRad + (1 - ratio) * lengthRad;; // not sure why we need this, but we apparently do
          }

          if (dimensions == 2)
          {
            glm::dvec2 vec(0);
            vec[0] = radius1 * std::cos(angle);
            vec[1] = radius2 * std::sin(angle);
            glm::dmat3 dmat = GetAxis2Placement2D(positionID);
            // If trimming by points no rotation is required
            if (byPos)
            {
              dmat[0] = glm::dvec3(1.0, 0.0, 0.0); // Assigning [1, 0, 0] to the X vector
              dmat[1] = glm::dvec3(0.0, 1.0, 0.0); // Assigning [0, 1, 0] to the Y vector
            }
            glm::dvec2 pos = dmat * glm::dvec3(vec, 1);
            curve.Add(pos);
          }
          else
          {
            glm::dvec3 vec(0);
            vec[0] = radius1 * std::cos(angle);
            vec[1] = radius2 * std::sin(angle) * sameSense; // negative or not???
            glm::dvec3 pos = GetLocalPlacement(positionID) * glm::dvec4(glm::dvec3(vec), 1);
            curve.Add(pos);
          }
        }

        // without a trim, we close the circle
        if (!trim.exist)
        {
          curve.Add(curve.points[startIndex]);
        }

        break;
      }
      case schema::IFCGRADIENTCURVE:
      {
        _loader.MoveToArgumentOffset(expressID, 0);
        auto tokens = _loader.GetSetArgument();
        auto u = _loader.GetStringArgument();
        auto masterCurveID = _loader.GetRefArgument();
        curve = GetCurve(masterCurveID, 3, false);

        std::vector<IfcCurve> curveList;
        for (auto token : tokens)
        {
          auto curveID = _loader.GetRefArgument(token);
          IfcCurve gradientCurve = GetCurve(curveID, 3, false);
          curveList.push_back(gradientCurve);
        }
        // #ifdef DEBUG_DUMP_SVG
        //   webifc::io::DumpGradientCurve(curveList, curve,"V_gradient.obj", "H_gradient.obj");
        // #endif
        break;
      }
      case schema::IFCCURVESEGMENT:
      {
        _loader.MoveToArgumentOffset(expressID, 0);
        auto type = _loader.GetStringArgument();
        _loader.MoveToArgumentOffset(expressID, 1);
        auto placementID = _loader.GetRefArgument();
        _loader.MoveToArgumentOffset(expressID, 2);
        double SegmentStart = ReadLenghtMeasure();
        _loader.MoveToArgumentOffset(expressID, 4);
        double SegmentEnd = ReadLenghtMeasure();
        _loader.MoveToArgumentOffset(expressID, 6);
        auto curveID = _loader.GetRefArgument();

        IfcTrimmingArguments trim = IfcTrimmingArguments();
        trim.start.param = SegmentStart;
        trim.end.param = SegmentEnd;
        trim.start.hasParam = true;
        trim.end.hasParam = true;
        ComputeCurve(curveID, curve, 3, false, -1, -1, trim);

        glm::dmat3 placement = GetAxis2Placement2D(placementID);

        for (size_t j = 0; j < curve.points.size(); j++)
        {
          curve.points[j] = placement * glm::dvec3(curve.points[j].x, curve.points[j].y, curve.points[j].z);
        }
        break;
      }
      case schema::IFCBSPLINECURVE:
      {
        bool condition = sameSense == 0;
        if (edge)
        {
          condition = !condition;
        }
        std::vector<uint32_t> knotMultiplicities;
        std::vector<glm::f64> distinctKnots;
        std::vector<glm::f64> knots;
        std::vector<glm::f64> indexes;
        std::vector<glm::f64> weights;

        _loader.MoveToArgumentOffset(expressID, 0);
        int degree = _loader.GetIntArgument();
        auto points = _loader.GetSetArgument();
        auto curveType = _loader.GetStringArgument();
        auto closed = _loader.GetStringArgument();
        auto selfIntersect = _loader.GetStringArgument();


        // build default knots
        for (size_t k = 0; k < points.size() + degree + 1; k++)
        {
          knots.push_back(k);
        }

        if (knots.size() != points.size() + degree + 1)
        {
          std::cout << "Error: Knots and control points do not match" << std::endl;
        }

        // build default weights vector
        for (size_t i = 0; i < points.size(); i++)
        {
          weights.push_back(1);
        }

        if (dimensions == 2) {
          std::vector<glm::dvec2> ctrolPts;
          for (auto &token : points)
          {
            uint32_t pointId = _loader.GetRefArgument(token);
            ctrolPts.push_back(GetCartesianPoint2D(pointId));
          }
        
          std::vector<glm::dvec2> tempPoints = GetRationalBSplineCurveWithKnots(degree, ctrolPts, knots, weights);
          for (size_t i = 0; i < tempPoints.size(); i++) curve.Add(tempPoints[i]);
        } else if (dimensions == 3) {
          std::vector<glm::dvec3> ctrolPts;
          for (auto &token : points)
          {
            uint32_t pointId = _loader.GetRefArgument(token);
            ctrolPts.push_back(GetCartesianPoint3D(pointId));
          }
        
          std::vector<glm::dvec3> tempPoints = GetRationalBSplineCurveWithKnots(degree, ctrolPts, knots, weights);
          for (size_t i = 0; i < tempPoints.size(); i++) curve.Add(tempPoints[i]);
        }        

        if (condition)
        {
          std::reverse(curve.points.begin(), curve.points.end());
        }

        break;
      }
    case schema::IFCBSPLINECURVEWITHKNOTS:
      {
        bool condition = sameSense == 0;
        if (edge)
        {
          condition = !condition;
        }

        // The IfcBSplineCurveWithKnots is a spline curve parameterized by spline functions for which the knot values are explicitly given.
        // This is the type of b-spline curve for which the knot values are explicitly given. This subtype shall be used to represent non-uniform B-spline curves and may be used for other knot types.
        // Let L denote the number of distinct values amongst the d+k+2 knots in the knot list; L will be referred to as the ‘upper index on knots’. Let mj denote the multiplicity (i.e., number of repetitions) of the _j_th distinct knot.

        // All knot multiplicities except the first and the last shall be in the range 1,...,d; the first and last may have a maximum value of d + 1. In evaluating the basis functions, a knot u of, e.g., multiplicity 3 is interpreted as a sequence u, u, u,; in the knot array.
        std::vector<uint32_t> knotMultiplicities;
        std::vector<glm::f64> distinctKnots;
        std::vector<glm::f64> knots;
        std::vector<glm::f64> indexes;
        std::vector<glm::f64> weights;

        _loader.MoveToArgumentOffset(expressID, 0);
        int degree = _loader.GetIntArgument();
        auto points = _loader.GetSetArgument();
        auto curveType = _loader.GetStringArgument();
        auto closed = _loader.GetStringArgument();
        auto selfIntersect = _loader.GetStringArgument();
        auto knotMultiplicitiesSet = _loader.GetSetArgument(); // The multiplicities of the knots. This list defines the number of times each knot in the knots list is to be repeated in constructing the knot array.
        auto knotSet = _loader.GetSetArgument();         // The list of distinct knots used to define the B-spline basis functions.



        for (auto &token : knotMultiplicitiesSet)
        {
          knotMultiplicities.push_back(_loader.GetIntArgument(token));
        }

        for (auto &token : knotSet)
        {
          distinctKnots.push_back(_loader.GetDoubleArgument(token));
        }

        for (size_t k = 0; k < distinctKnots.size(); k++)
        {
          double knot = distinctKnots[k];
          for (size_t i = 0; i < knotMultiplicities[k]; i++)
          {
            knots.push_back(knot);
          }
        }

        if (knots.size() != points.size() + degree + 1)
        {
          std::cout << "Error: Knots and control points do not match" << std::endl;
        }

        // build default weights vector
        for (size_t i = 0; i < points.size(); i++)
        {
          weights.push_back(1);
        }

        if (dimensions == 2) 
        {
          std::vector<glm::dvec2> ctrolPts;
          for (auto &token : points)
          {
            uint32_t pointId = _loader.GetRefArgument(token);
            ctrolPts.push_back(GetCartesianPoint3D(pointId));
          }
          std::vector<glm::dvec2> tempPoints = GetRationalBSplineCurveWithKnots(degree, ctrolPts, knots, weights);
          for (size_t i = 0; i < tempPoints.size(); i++) curve.Add(tempPoints[i]);
        } 
      else if (dimensions == 3)
      {
        std::vector<glm::dvec3> ctrolPts;
        for (auto &token : points)
        {
          uint32_t pointId = _loader.GetRefArgument(token);
          ctrolPts.push_back(GetCartesianPoint3D(pointId));
        }
        std::vector<glm::dvec3> tempPoints = GetRationalBSplineCurveWithKnots(degree, ctrolPts, knots, weights);
        for (size_t i = 0; i < tempPoints.size(); i++) curve.Add(tempPoints[i]);
      }

    if (condition)
    {
      std::reverse(curve.points.begin(), curve.points.end());
    }

    break;
  }
case schema::IFCRATIONALBSPLINECURVEWITHKNOTS:
  {

    bool condition = sameSense == 0;
    if (edge)
    {
      condition = !condition;
    }

    std::vector<glm::f64> distinctKnots;
    std::vector<uint32_t> knotMultiplicities;
    std::vector<glm::f64> knots;
    std::vector<glm::f64> weights;
    _loader.MoveToArgumentOffset(expressID, 0);
    int degree = _loader.GetIntArgument();
    auto points = _loader.GetSetArgument();
    auto curveType = _loader.GetStringArgument();
    auto closed = _loader.GetStringArgument();
    auto selfIntersect = _loader.GetStringArgument();
        auto knotMultiplicitiesSet = _loader.GetSetArgument(); // The multiplicities of the knots. This list defines the number of times each knot in the knots list is to be repeated in constructing the knot array.
        auto knotSet = _loader.GetSetArgument();
        auto knotSpec = _loader.GetStringArgument(); // The description of the knot type. This is for information only.
        auto weightsSet = _loader.GetSetArgument();

        for (auto &token : knotMultiplicitiesSet)
        {
          knotMultiplicities.push_back(_loader.GetIntArgument(token));
        }

        for (auto &token : knotSet)
        {
          distinctKnots.push_back(_loader.GetDoubleArgument(token));
        }

        for (auto &token : weightsSet)
        {
          weights.push_back(_loader.GetDoubleArgument(token));
        }

        for (size_t k = 0; k < distinctKnots.size(); k++)
        {
          double knot = distinctKnots[k];
          for (size_t i = 0; i < knotMultiplicities[k]; i++)
          {
            knots.push_back(knot);
          }
        }

        if (knots.size() != points.size() + degree + 1)
        {
          std::cout << "Error: Knots and control points do not match" << std::endl;
        }

        if (dimensions == 2) 
        {
          std::vector<glm::dvec2> ctrolPts;
          for (auto &token : points)
          {
            uint32_t pointId = _loader.GetRefArgument(token);
            ctrolPts.push_back(GetCartesianPoint3D(pointId));
          }

          std::vector<glm::dvec2> tempPoints = GetRationalBSplineCurveWithKnots(degree, ctrolPts, knots, weights);
          for (size_t i = 0; i < tempPoints.size(); i++) curve.Add(tempPoints[i]);
        }
      else if (dimensions == 3)
      {
        std::vector<glm::dvec3> ctrolPts;
        for (auto &token : points)
        {
          uint32_t pointId = _loader.GetRefArgument(token);
          ctrolPts.push_back(GetCartesianPoint3D(pointId));
        }

        std::vector<glm::dvec3> tempPoints = GetRationalBSplineCurveWithKnots(degree, ctrolPts, knots, weights);
        for (size_t i = 0; i < tempPoints.size(); i++) curve.Add(tempPoints[i]);
      }




    if (condition)
    {
      std::reverse(curve.points.begin(), curve.points.end());
    }

    break;
  }
default:

  spdlog::error("[ComputeCurve()] Unsupported curve type {}", expressID, lineType);
  break;
}
  // DEBUG
  // #ifdef DEBUG_DUMP_SVG
  //     io::DumpSVGCurve(curve.points, "partial_curve.html");
  // #endif

}

void IfcGeometryLoader::convertAngleUnits(double &Degrees, double &Rad) const
{
  if (_angleUnits == "RADIAN")
  {
    Degrees = (Rad / CONST_PI) * 180;
  }
  else
  {
    Rad = Degrees / 180 * CONST_PI;
  }
}

IfcProfile IfcGeometryLoader::GetProfile(uint32_t expressID) const
{
  spdlog::debug("[GetProfile({})]",expressID);
  auto profile = GetProfileByLine(expressID);

  if (!profile.isComposite)
  {
    if (!profile.curve.IsCCW())
    {
      profile.curve.Invert();
    }
    for (auto &hole : profile.holes)
    {
      if (hole.IsCCW())
      {
        hole.Invert();
      }
    }
  }
  else
  {
    for (uint32_t i = 0; i < profile.profiles.size(); i++)
    {
      if (!profile.profiles[i].curve.IsCCW())
      {
        profile.profiles[i].curve.Invert();
      }
      for (auto &hole : profile.profiles[i].holes)
      {
        if (hole.IsCCW())
        {
          hole.Invert();
        }
      }
    }
  }

  return profile;
}

  IfcProfile IfcGeometryLoader::GetProfileByLine(uint32_t expressID) const
  {
    spdlog::debug("[GetProfileByLine({})]",expressID);
    auto lineType = _loader.GetLineType(expressID);
    switch (lineType)
    {
    case schema::IFCARBITRARYOPENPROFILEDEF:
    case schema::IFCARBITRARYCLOSEDPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      _loader.MoveToArgumentOffset(expressID, 2);
      // ISSUE 765 requires dimension 3, not sure how to solve it without a stopgap
      profile.curve = GetCurve(_loader.GetRefArgument(), 2);
      profile.isConvex = IsCurveConvex(profile.curve);

      return profile;
    }
    case schema::IFCARBITRARYPROFILEDEFWITHVOIDS:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      _loader.MoveToArgumentOffset(expressID, 2);
      profile.curve = GetCurve(_loader.GetRefArgument(), 2);
      profile.isConvex = IsCurveConvex(profile.curve);

      _loader.MoveToArgumentOffset(expressID, 3);
      auto holes = _loader.GetSetArgument();

      for (auto &hole : holes)
      {
        IfcCurve holeCurve = GetCurve(_loader.GetRefArgument(hole), 2);
        #ifdef CSG_DEBUG_OUTPUT
          io::DumpSVGCurve(holeCurve.points, "IFCEXTRUDEDAREASOLID_hole_curve.html");
        #endif

        profile.holes.push_back(holeCurve);
      }

      return profile;
    }
    case schema::IFCRECTANGLEPROFILEDEF:
    case schema::IFCROUNDEDRECTANGLEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(expressID, 2);
      uint32_t placementID = _loader.GetRefArgument();
      double xdim = _loader.GetDoubleArgument();
      double ydim = _loader.GetDoubleArgument();

      if (placementID != 0)
      {
        glm::dmat3 placement = GetAxis2Placement2D(placementID);
        profile.curve = GetRectangleCurve(xdim, ydim, placement);
      }
      else
      {
        glm::dmat3 placement = glm::dmat3(
            glm::dvec3(1, 0, 0),
            glm::dvec3(0, 1, 0),
            glm::dvec3(0, 0, 1));
        profile.curve = GetRectangleCurve(xdim, ydim, placement);
      }
      return profile;
    }
    case schema::IFCRECTANGLEHOLLOWPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(expressID, 2);
      uint32_t placementID = _loader.GetRefArgument();
      double xdim = _loader.GetDoubleArgument();
      double ydim = _loader.GetDoubleArgument();
      double thickness = _loader.GetDoubleArgument();

      // fillets not implemented yet

      glm::dmat3 placement = GetAxis2Placement2D(placementID);

      profile.curve = GetRectangleCurve(xdim, ydim, placement);
      profile.holes.push_back(GetRectangleCurve(xdim - thickness, ydim - thickness, placement));
      std::reverse(profile.holes[0].points.begin(), profile.holes[0].points.end());

      return profile;
    }
    case schema::IFCCIRCLEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(expressID, 2);
      uint32_t placementID = _loader.GetOptionalRefArgument();
      double radius = _loader.GetDoubleArgument();

      glm::dmat3 placement(1);

      if (placementID)
      {
        placement = GetAxis2Placement2D(placementID);
      }

      profile.curve = GetCircleCurve(radius, _circleSegments, placement);

      return profile;
    }
    case schema::IFCELLIPSEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(expressID, 2);
      uint32_t placementID = _loader.GetRefArgument();
      double radiusX = _loader.GetDoubleArgument();
      double radiusY = _loader.GetDoubleArgument();

      glm::dmat3 placement = GetAxis2Placement2D(placementID);

      profile.curve = GetEllipseCurve(radiusX, radiusY, _circleSegments, placement);

      return profile;
    }
    case schema::IFCCIRCLEHOLLOWPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(expressID, 2);
      uint32_t placementID = _loader.GetOptionalRefArgument();
      double radius = _loader.GetDoubleArgument();
      double thickness = _loader.GetDoubleArgument();

      glm::dmat3 placement = glm::dmat3(1.0);

      if(placementID)
      {
        placement = GetAxis2Placement2D(placementID);
      }

      profile.curve = GetCircleCurve(radius, _circleSegments, placement);
      profile.holes.push_back(GetCircleCurve(radius - thickness, _circleSegments, placement));
      std::reverse(profile.holes[0].points.begin(), profile.holes[0].points.end());

      return profile;
    }
    case schema::IFCISHAPEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(expressID, 2);

      glm::dmat3 placement(1);

      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();

        uint32_t placementID = _loader.GetRefArgument();
        placement = GetAxis2Placement2D(placementID);
      }

      _loader.MoveToArgumentOffset(expressID, 3);

      double width = _loader.GetDoubleArgument();
      double depth = _loader.GetDoubleArgument();
      double webThickness = _loader.GetDoubleArgument();
      double flangeThickness = _loader.GetDoubleArgument();

      // optional fillet
      bool hasFillet = false;
      double filletRadius = 0;
      if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
      {
        _loader.StepBack();

        hasFillet = true;
        filletRadius = _loader.GetDoubleArgument();
      }

      profile.curve = GetIShapedCurve(width, depth, webThickness, flangeThickness, hasFillet, filletRadius, placement);

      return profile;
    }
    case schema::IFCLSHAPEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = false;

      _loader.MoveToArgumentOffset(expressID, 2);

      glm::dmat3 placement(1);

      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();

        uint32_t placementID = _loader.GetRefArgument();
        placement = GetAxis2Placement2D(placementID);
      }

      _loader.MoveToArgumentOffset(expressID, 3);
      double filletRadius = 0;
      double depth = _loader.GetDoubleArgument();
      double width = _loader.GetDoubleArgument();
      double thickness = _loader.GetDoubleArgument();
      filletRadius = _loader.GetDoubleArgument();
      double edgeRadius = _loader.GetDoubleArgument();
      double legSlope = _loader.GetDoubleArgument();
      // double centreOfGravityInX =
      _loader.GetDoubleArgument();
      // double centreOfGravityInY =
      _loader.GetDoubleArgument();

      // optional fillet
      bool hasFillet = false;

      if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
      {
        _loader.StepBack();

        hasFillet = true;
        filletRadius = _loader.GetDoubleArgument();
      }

      profile.curve = GetLShapedCurve(width, depth, thickness, hasFillet, filletRadius, edgeRadius, legSlope, placement);

      return profile;
    }
    case schema::IFCTSHAPEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = false;

      _loader.MoveToArgumentOffset(expressID, 2);

      glm::dmat3 placement(1);

      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();

        uint32_t placementID = _loader.GetRefArgument();
        placement = GetAxis2Placement2D(placementID);
      }

      _loader.MoveToArgumentOffset(expressID, 3);
      double depth = _loader.GetDoubleArgument();
      double width = _loader.GetDoubleArgument();
      double webThickness = _loader.GetDoubleArgument();
      // double flangeThickness =
      _loader.GetDoubleArgument();
      double filletRadius = _loader.GetDoubleArgument();
      double flangeEdgeRadius = _loader.GetDoubleArgument();
      // double webEdgeRadius =
      _loader.GetDoubleArgument();
      // double webSlope =
      _loader.GetDoubleArgument();
      double flangeSlope = _loader.GetDoubleArgument();

      // optional fillet
      bool hasFillet = false;

      if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
      {
        _loader.StepBack();

        hasFillet = true;
        filletRadius = _loader.GetDoubleArgument();
      }

      profile.curve = GetTShapedCurve(width, depth, webThickness, hasFillet, filletRadius, flangeEdgeRadius, flangeSlope, placement);

      return profile;
    }
    case schema::IFCUSHAPEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(expressID, 2);

      glm::dmat3 placement(1);

      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();

        uint32_t placementID = _loader.GetRefArgument();
        placement = GetAxis2Placement2D(placementID);
      }

      _loader.MoveToArgumentOffset(expressID, 3);

      double depth = _loader.GetDoubleArgument();
      double flangeWidth = _loader.GetDoubleArgument();
      double webThickness = _loader.GetDoubleArgument();
      double flangeThickness = _loader.GetDoubleArgument();

      // optional parameters
      // double filletRadius = GetOptionalDoubleParam();
      // double edgeRadius = GetOptionalDoubleParam();
      // double flangeSlope = GetOptionalDoubleParam();
      double filletRadius = 0;
      double edgeRadius = 0;
      double flangeSlope = 0;

      profile.curve = GetUShapedCurve(depth, flangeWidth, webThickness, flangeThickness, filletRadius, edgeRadius, flangeSlope, placement);

      return profile;
    }
    case schema::IFCCSHAPEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(expressID, 2);

      glm::dmat3 placement(1);

      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();

        uint32_t placementID = _loader.GetRefArgument();
        placement = GetAxis2Placement2D(placementID);
      }

      bool hasFillet = false;

      _loader.MoveToArgumentOffset(expressID, 3);

      double depth = _loader.GetDoubleArgument();
      double Width = _loader.GetDoubleArgument();
      double Thickness = _loader.GetDoubleArgument();
      double girth = _loader.GetDoubleArgument();
      double filletRadius = _loader.GetDoubleArgument();

      profile.curve = GetCShapedCurve(Width, depth, girth, Thickness, hasFillet, filletRadius, placement);

      return profile;
    }
    case schema::IFCZSHAPEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(expressID, 2);

      glm::dmat3 placement(1);

      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();

        uint32_t placementID = _loader.GetRefArgument();
        glm::dmat3 placement = GetAxis2Placement2D(placementID);
      }

      bool hasFillet = false;

      _loader.MoveToArgumentOffset(expressID, 3);

      double depth = _loader.GetDoubleArgument();
      double flangeWidth = _loader.GetDoubleArgument();
      double webThickness = _loader.GetDoubleArgument();
      double flangeThickness = _loader.GetDoubleArgument();
      double filletRadius = _loader.GetDoubleArgument();
      double edgeRadius = _loader.GetDoubleArgument();

      profile.curve = GetZShapedCurve(depth, flangeWidth, webThickness, flangeThickness, filletRadius, edgeRadius, placement);

      return profile;
    }
    case schema::IFCDERIVEDPROFILEDEF:
    {
      _loader.MoveToArgumentOffset(expressID, 2);
      uint32_t profileID = _loader.GetRefArgument();
      IfcProfile profile = GetProfileByLine(profileID);

      _loader.MoveToArgumentOffset(expressID, 3);
      uint32_t transformID = _loader.GetRefArgument();
      glm::dmat3 transformation = GetAxis2Placement2D(transformID);

      if (!profile.isComposite)
      {
        for (uint32_t i = 0; i < profile.curve.points.size(); i++)
        {
          profile.curve.points[i] = transformation * glm::dvec3(profile.curve.points[i].x, profile.curve.points[i].y, 1);
        }
      }
      else
      {
        for (uint32_t j = 0; j < profile.profiles.size(); j++)
        {
          for (uint32_t i = 0; i < profile.profiles[j].curve.points.size(); i++)
          {
            profile.profiles[j].curve.points[i] = transformation * glm::dvec3(profile.profiles[j].curve.points[i].x, profile.profiles[j].curve.points[i].y, 1);
          }
        }
      }

      return profile;
    }
    case schema::IFCCOMPOSITEPROFILEDEF:
    {
      IfcProfile profile = IfcProfile();

      std::vector<uint32_t> lst;

      _loader.MoveToArgumentOffset(expressID, 2);
      parsing::IfcTokenType t = _loader.GetTokenType();
      if (t == parsing::IfcTokenType::SET_BEGIN)
      {
        while (_loader.GetTokenType() == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          uint32_t profileID = _loader.GetRefArgument();
          lst.push_back(profileID);
        }
      }

      profile.isComposite = true;

      for (uint32_t i = 0; i < lst.size(); i++)
      {
        IfcProfile profile_t = GetProfileByLine(lst[i]);
        profile.profiles.push_back(profile_t);
      }

      return profile;
    }
    case schema::IFCOPENCROSSPROFILEDEF:
    {
      IfcProfile profile = IfcProfile();

      _loader.MoveToArgumentOffset(expressID, 2);
      auto horizontalWidth = _loader.GetStringArgument();
      _loader.MoveToArgumentOffset(expressID, 3);
      auto widthsTokens = _loader.GetSetListArgument();
      _loader.MoveToArgumentOffset(expressID, 4);
      auto slopesTokens = _loader.GetSetListArgument();
      _loader.MoveToArgumentOffset(expressID, 5);
      auto tagsTokens = _loader.GetSetListArgument();

      std::vector<double> listWidths;
      for (auto &set : widthsTokens)
      {
        for (auto &token : set)
        {
          listWidths.push_back(_loader.GetDoubleArgument(token));
        }
      }

      std::vector<double> listSlopes;
      for (auto &set : slopesTokens)
      {
        for (auto &token : set)
        {
          listSlopes.push_back(_loader.GetDoubleArgument(token));
        }
      }

      std::vector<double> listTags;
      for (auto &set : tagsTokens)
      {
        for (auto &token : set)
        {
          listTags.push_back(_loader.GetDoubleArgument(token));
        }
      }

      // Initialize starting point of profile curve at origin
      double x = 0.0;
      double y = 0.0;

      // Iterate over each width and slope in the lists
      for (size_t i = 0; i < listWidths.size(); ++i)
      {
        double width = listWidths[i];
        double slope_degrees = listSlopes[i];

        // Convert slope from degrees to radians

        double slope_radians = slope_degrees;

        convertAngleUnits(slope_degrees, slope_radians);

        // Calculate the change in x and y based on the width and slope
        double dx = width * cos(slope_radians);
        double dy = width * sin(slope_radians);

        // Add the new (x,y) coordinate to the profile curve vector
        profile.curve.Add(glm::dvec2(x, y));

        // Update the (x,y) coordinates for the next line segment
        x += dx;
        y += dy;
      }

      return profile;
    }
    case schema::IFCTRAPEZIUMPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(expressID, 2);
      uint32_t placementID = _loader.GetRefArgument();
      double bottomXDim = _loader.GetDoubleArgument();
      double topXDim = _loader.GetDoubleArgument();
      double yDim = _loader.GetDoubleArgument();
      double topXOffset = _loader.GetDoubleArgument();

      glm::dmat3 placement;
      if (placementID != 0)
      {
        placement = GetAxis2Placement2D(placementID);
      }
      else
      {
        placement = glm::dmat3(
            glm::dvec3(1, 0, 0),
            glm::dvec3(0, 1, 0),
            glm::dvec3(0, 0, 1));
      }
      profile.curve = GetTrapeziumCurve(bottomXDim, topXDim, yDim, topXOffset, placement);
      return profile;
    }
    default:
      spdlog::error("[GetProfileByLine()] unexpected profile type {}", expressID, lineType);
      break;
    }

    return IfcProfile();
  }

  IfcProfile IfcGeometryLoader::GetProfile3D(uint32_t expressID) const
  {
   spdlog::debug("[GetProfile3D({})]",expressID);
    auto lineType = _loader.GetLineType(expressID);
    switch (lineType)
    {
    case schema::IFCARBITRARYOPENPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(expressID, 0);
      profile.type = _loader.GetStringArgument();
      _loader.MoveToArgumentOffset(expressID, 2);
      profile.curve = GetCurve(_loader.GetRefArgument(), 3);

      return profile;
    }
    default:
      spdlog::error("[GetProfilebyLine()] unexpected 3D profile type {}", expressID, lineType);
      break;
    }

    return IfcProfile();
  }

  glm::dvec3 IfcGeometryLoader::GetVector(uint32_t expressID) const
  {
    spdlog::debug("[GetVector({})]",expressID);
    _loader.MoveToArgumentOffset(expressID, 0);
    auto positionID = _loader.GetRefArgument();
    double length = _loader.GetDoubleArgument();

    glm::dvec3 direction = GetCartesianPoint3D(positionID);
    direction.x = direction.x * length;
    direction.y = direction.y * length;
    direction.z = direction.z * length;

    return direction;
  }

  glm::dmat3 IfcGeometryLoader::GetAxis2Placement2D(uint32_t expressID) const
  {
    // TODO: Bad solution
    if(expressID < 1)
    {
      return glm::dmat3(1.0);
    }

    spdlog::debug("[GetAxis2Placement2D({})]",expressID);
    auto lineType = _loader.GetLineType(expressID);
    switch (lineType)
    {
    case schema::IFCAXIS2PLACEMENT2D:
    {
      _loader.MoveToArgumentOffset(expressID, 0);
      uint32_t locationID = _loader.GetRefArgument();
      parsing::IfcTokenType dirToken = _loader.GetTokenType();

      glm::dvec2 xAxis = glm::dvec2(1, 0);
      if (dirToken == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        xAxis = glm::normalize(GetCartesianPoint2D(_loader.GetRefArgument()));
      }

      glm::dvec2 pos = GetCartesianPoint2D(locationID);

      glm::dvec2 yAxis = glm::normalize(glm::dvec2(-xAxis.y, xAxis.x));

      return glm::dmat3(
          glm::dvec3(xAxis, 0),
          glm::dvec3(yAxis, 0),
          glm::dvec3(pos, 1));
    }
    case schema::IFCCARTESIANTRANSFORMATIONOPERATOR2D:
    case schema::IFCCARTESIANTRANSFORMATIONOPERATOR2DNONUNIFORM:
    {
      double scale1 = 1.0;
      double scale2 = 1.0;

      glm::dvec2 Axis1(1, 0);
      glm::dvec2 Axis2(0, 1);

      _loader.MoveToArgumentOffset(expressID, 0);
      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        Axis1 = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
      }
      _loader.MoveToArgumentOffset(expressID, 1);
      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        Axis2 = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
      }

      _loader.MoveToArgumentOffset(expressID, 2);
      uint32_t posID = _loader.GetRefArgument();
      glm::dvec2 pos = GetCartesianPoint2D(posID);

      _loader.MoveToArgumentOffset(expressID, 3);
      if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
      {
        _loader.StepBack();
        scale1 = _loader.GetDoubleArgument();
      }

      if (lineType == schema::IFCCARTESIANTRANSFORMATIONOPERATOR2DNONUNIFORM)
      {
        _loader.MoveToArgumentOffset(expressID, 4);
        if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
        {
          _loader.StepBack();
          scale2 = _loader.GetDoubleArgument();
        }
      }

      if (lineType == schema::IFCCARTESIANTRANSFORMATIONOPERATOR2D)
      {
        scale2 = scale1;
      }

      return glm::dmat3(
          glm::dvec3(Axis1 * scale1, 0),
          glm::dvec3(Axis2 * scale2, 0),
          glm::dvec3(pos, 1));
    }
    default:
      spdlog::error("[GetAxis2DPlacement()] unexpected 2D placement type {}", expressID, lineType);
      break;
    }
    return glm::dmat3();
  }

  IfcCurve IfcGeometryLoader::GetLocalCurve(uint32_t expressID) const
  {
    spdlog::debug("[GetLocalCurve({})]",expressID);
    for (uint32_t i = 0; i < LocalcurvesIndices.size(); i++)
    {
      if (LocalcurvesIndices[i] == expressID)
      {
        return LocalCurvesList[i];
      }
    }
    IfcCurve curve = GetCurve(expressID, 3, false);
    LocalcurvesIndices.push_back(expressID);
    LocalCurvesList.push_back(curve);
    return curve;
  }

  glm::dmat4 IfcGeometryLoader::GetLocalPlacement(uint32_t expressID, glm::dvec3 vector) const
  {
    if(_expressIDToPlacement.contains(expressID)) {
      return _expressIDToPlacement[expressID];
    }
    else
    {
      spdlog::debug("[GetLocalPlacement({})]",expressID);
      auto lineType = _loader.GetLineType(expressID);
      switch (lineType)
      {
      case schema::IFCPOINTBYDISTANCEEXPRESSION:
      {
        _loader.MoveToArgumentOffset(expressID, 0);
        IfcCurve curve;
        auto lnSegment = 0;

        if (_loader.GetTokenType() != parsing::IfcTokenType::EMPTY)
        {
          _loader.StepBack();
          lnSegment = ReadLenghtMeasure();
        }

        _loader.MoveToArgumentOffset(expressID, 5);
        auto t = _loader.GetTokenType();
        if (t == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          auto curveId = _loader.GetRefArgument();
          curve = GetLocalCurve(curveId);
          glm::dmat4 result = curve.getPlacementAtDistance(lnSegment);
          _expressIDToPlacement[expressID] = result;
          return result;
        }
        else
        {
          curve.Add(glm::dvec3(0, 0, 0));
          curve.Add(glm::dvec3(0, 10000, 0));
          glm::dmat4 result = curve.getPlacementAtDistance(lnSegment);
          _expressIDToPlacement[expressID] = result;
          return result;
        }
      }
      case schema::IFCAXIS1PLACEMENT:
      {
        glm::dvec3 zAxis(0, 0, 1);
        glm::dvec3 xAxis(1, 0, 0);
        _loader.MoveToArgumentOffset(expressID, 0);
        uint32_t posID = _loader.GetRefArgument();
        parsing::IfcTokenType zID = _loader.GetTokenType();
        if (zID == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          zAxis = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
        }
        glm::dvec3 pos = GetCartesianPoint3D(posID);
        if (std::abs(glm::dot(xAxis, zAxis)) > 0.9)
        {
          xAxis = glm::dvec3(0, 1, 0);
        }
        glm::dvec3 yAxis = glm::normalize(glm::cross(zAxis, xAxis));
        xAxis = glm::normalize(glm::cross(zAxis, yAxis));

        glm::dmat4 result = glm::dmat4(
          glm::dvec4(xAxis, 0),
          glm::dvec4(yAxis, 0),
          glm::dvec4(zAxis, 0),
          glm::dvec4(pos, 1));

        _expressIDToPlacement[expressID] = result;
        return result;
      }
      case schema::IFCAXIS2PLACEMENT3D:
      {
        glm::dvec3 zAxis(0, 0, 1);
        glm::dvec3 xAxis(1, 0, 0);

        _loader.MoveToArgumentOffset(expressID, 0);
        uint32_t posID = _loader.GetRefArgument();
        parsing::IfcTokenType zID = _loader.GetTokenType();
        if (zID == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          auto tmpVec = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
          if (glm::length(tmpVec) > 0) zAxis = tmpVec;
        }

        _loader.MoveToArgumentOffset(expressID, 2);
        parsing::IfcTokenType xID = _loader.GetTokenType();
        if (xID == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          auto tmpVec = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
          if (glm::length(tmpVec) > 0) xAxis = tmpVec;
        }

        glm::dvec3 pos = GetCartesianPoint3D(posID);

        glm::dvec3 yAxis = glm::normalize(glm::cross(zAxis, xAxis));
        xAxis = glm::normalize(glm::cross(yAxis, zAxis));

        glm::dmat4 result = glm::dmat4(
          glm::dvec4(xAxis, 0),
          glm::dvec4(yAxis, 0),
          glm::dvec4(zAxis, 0),
          glm::dvec4(pos, 1));

        _expressIDToPlacement[expressID] = result;

        return result;
      }
      case schema::IFCAXIS2PLACEMENT2D:
      {
        glm::dvec3 xAxis(1, 0, 0);
        glm::dvec3 zAxis(0, 0, 1);

        _loader.MoveToArgumentOffset(expressID, 0);
        uint32_t posID = _loader.GetRefArgument();

        _loader.MoveToArgumentOffset(expressID, 1);
        parsing::IfcTokenType xID = _loader.GetTokenType();
        if (xID == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          auto tmpVec = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
          if (glm::length(tmpVec) > 0) xAxis = tmpVec;
        }

        glm::dvec3 pos = GetCartesianPoint3D(posID);

        glm::dvec3 yAxis = glm::normalize(glm::cross(zAxis, xAxis));
        xAxis = glm::normalize(glm::cross(yAxis, zAxis));

        glm::dmat4 result = glm::dmat4(
            glm::dvec4(xAxis, 0),
            glm::dvec4(yAxis, 0),
            glm::dvec4(zAxis, 0),
            glm::dvec4(pos, 1));
        
        _expressIDToPlacement[expressID] = result;
        return result;
      }
      case schema::IFCLOCALPLACEMENT:
      {
        glm::dmat4 relPlacement(1);

        _loader.MoveToArgumentOffset(expressID, 0);
        parsing::IfcTokenType relPlacementToken = _loader.GetTokenType();
        if (relPlacementToken == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          relPlacement = GetLocalPlacement(_loader.GetRefArgument());
        }

        _loader.MoveToArgumentOffset(expressID, 1);
        uint32_t axis2PlacementID = _loader.GetRefArgument();

        glm::dmat4 axis2Placement = GetLocalPlacement(axis2PlacementID);

        auto result = relPlacement * axis2Placement;

        _expressIDToPlacement[expressID] = result;
        return result;
      }
      case schema::IFCCARTESIANTRANSFORMATIONOPERATOR3D:
      case schema::IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM:
      {
        double scale1 = 1.0;
        double scale2 = 1.0;
        double scale3 = 1.0;

        glm::dvec3 Axis1(1, 0, 0);
        glm::dvec3 Axis2(0, 1, 0);
        glm::dvec3 Axis3(0, 0, 1);

        _loader.MoveToArgumentOffset(expressID, 0);
        if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          Axis1 = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
        }
        _loader.MoveToArgumentOffset(expressID, 1);
        if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          Axis2 = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
        }

        _loader.MoveToArgumentOffset(expressID, 2);
        uint32_t posID = _loader.GetRefArgument();
        glm::dvec3 pos = GetCartesianPoint3D(posID);

        _loader.MoveToArgumentOffset(expressID, 3);
        if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
        {
          _loader.StepBack();
          scale1 = _loader.GetDoubleArgument();
        }

        _loader.MoveToArgumentOffset(expressID, 4);
        if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          Axis3 = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
        }

        if (lineType == schema::IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM)
        {
          _loader.MoveToArgumentOffset(expressID, 5);
          if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
          {
            _loader.StepBack();
            scale2 = _loader.GetDoubleArgument();
          }

          _loader.MoveToArgumentOffset(expressID, 6);
          if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
          {
            _loader.StepBack();
            scale3 = _loader.GetDoubleArgument();
          }
        }

        if (lineType == schema::IFCCARTESIANTRANSFORMATIONOPERATOR3D)
        {
          scale2 = scale1;
          scale3 = scale1;
        }

        glm::dmat4 result = glm::dmat4(
            glm::dvec4(Axis1 * scale1, 0),
            glm::dvec4(Axis2 * scale2, 0),
            glm::dvec4(Axis3 * scale3, 0),
            glm::dvec4(pos, 1));
        
        _expressIDToPlacement[expressID] = result;
        return result;
      }
      case schema::IFCAXIS2PLACEMENTLINEAR:
      {
        glm::dvec3 vector = glm::dvec3(0, 0, 1);
        _loader.MoveToArgumentOffset(expressID, 0);
        uint32_t posID = _loader.GetRefArgument();
        if (_loader.GetRefArgument() == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          glm::dvec3 vector = GetCartesianPoint3D(_loader.GetRefArgument());
        }
        glm::dmat4 result = GetLocalPlacement(posID, vector);

        _expressIDToPlacement[expressID] = result;
        return result;
      }
      case schema::IFCLINEARPLACEMENT:
      {
        _loader.MoveToArgumentOffset(expressID, 1);
        uint32_t posID = _loader.GetRefArgument();
        glm::dmat4 result =  GetLocalPlacement(posID);

        _expressIDToPlacement[expressID] = result;
        return result;
      }
      default:
      spdlog::error("[GetLocalPlacement()] unexpected placement type {}", expressID, lineType);
        break;
      }

      return glm::dmat4(1);
    }
  }

  std::array<glm::dvec3, 2> IfcGeometryLoader::GetAxis1Placement(const uint32_t expressID) const
  {
    spdlog::debug("[GetAxis1Placement({})]",expressID);
    _loader.MoveToArgumentOffset(expressID, 0);
    uint32_t locationID = _loader.GetRefArgument();
    parsing::IfcTokenType dirToken = _loader.GetTokenType();

    glm::dvec3 axis = glm::dvec3(0, 0, 1);
    if (dirToken == parsing::IfcTokenType::REF)
    {
      _loader.StepBack();
      axis = GetCartesianPoint3D(_loader.GetRefArgument());
    }

    glm::dvec3 pos = GetCartesianPoint3D(locationID);

    return {axis, pos};
  }

  std::unordered_map<uint32_t, std::vector<uint32_t>> IfcGeometryLoader::PopulateRelVoidsMap()
  {
    std::unordered_map<uint32_t, std::vector<uint32_t>> resultVector;
    auto relVoids = _loader.GetExpressIDsWithType(schema::IFCRELVOIDSELEMENT);

    for (uint32_t relVoidID : relVoids)
    {
      _loader.MoveToArgumentOffset(relVoidID, 4);

      uint32_t relatingBuildingElement = _loader.GetRefArgument();
      uint32_t relatedOpeningElement = _loader.GetRefArgument();

      resultVector[relatingBuildingElement].push_back(relatedOpeningElement);
    }
    return resultVector;
  }

  std::unordered_map<uint32_t, std::vector<uint32_t>> IfcGeometryLoader::PopulateRelVoidsRelMap()
  {
    std::unordered_map<uint32_t, std::vector<uint32_t>> resultVector;
    auto relVoids = _loader.GetExpressIDsWithType(schema::IFCRELVOIDSELEMENT);

    for (uint32_t relVoidID : relVoids)
    {
      _loader.MoveToArgumentOffset(relVoidID, 4);

      uint32_t relatingBuildingElement = _loader.GetRefArgument();

      resultVector[relatingBuildingElement].push_back(relVoidID);
    }
    return resultVector;
  }

  std::unordered_map<uint32_t, std::vector<uint32_t>> IfcGeometryLoader::PopulateRelAggregatesMap()
  {
    std::unordered_map<uint32_t, std::vector<uint32_t>> resultVector;
    auto relAggregates = _loader.GetExpressIDsWithType(schema::IFCRELAGGREGATES);

    for (uint32_t relAggregateID : relAggregates)
    {
      _loader.MoveToArgumentOffset(relAggregateID, 4);

      uint32_t relatingBuildingElement = _loader.GetRefArgument();
      auto aggregates = _loader.GetSetArgument();

      for (auto &aggregate : aggregates)
      {
        uint32_t aggregateID = _loader.GetRefArgument(aggregate);
        resultVector[relatingBuildingElement].push_back(aggregateID);
      }
    }
    return resultVector;
  }

  std::unordered_map<uint32_t, std::vector<uint32_t>> IfcGeometryLoader::PopulateRelNestsMap()
  {
    std::unordered_map<uint32_t, std::vector<uint32_t>> resultVector;
    auto relNests = _loader.GetExpressIDsWithType(schema::IFCRELNESTS);

    for (uint32_t relNestID : relNests)
    {
      _loader.MoveToArgumentOffset(relNestID, 4);

      uint32_t relatingBuildingElement = _loader.GetRefArgument();
      auto nests = _loader.GetSetArgument();

      for (auto &nest : nests)
      {
        uint32_t nestID = _loader.GetRefArgument(nest);
        resultVector[relatingBuildingElement].push_back(nestID);
      }
    }
    return resultVector;
  }

  std::unordered_map<uint32_t, std::vector<uint32_t>> IfcGeometryLoader::PopulateRelElementAggregatesMap()
  {
    std::unordered_map<uint32_t, std::vector<uint32_t>> resultVector;
    auto relElements = _loader.GetExpressIDsWithType(schema::IFCRELAGGREGATES);

    for (uint32_t relElementID : relElements)
    {
      _loader.MoveToArgumentOffset(relElementID, 4);

      uint32_t relatingBuildingElement = _loader.GetRefArgument();
      auto aggregates = _loader.GetSetArgument();

      auto lineType2 = _loader.GetLineType(relatingBuildingElement);

      if (_schemaManager.IsIfcElement(lineType2))
      {
        for (auto &aggregate : aggregates)
        {
          uint32_t aggregateID = _loader.GetRefArgument(aggregate);
          resultVector[aggregateID].push_back(relatingBuildingElement);
        }
      }
    }
    return resultVector;
  }

  std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> IfcGeometryLoader::PopulateStyledItemMap()
  {
    std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> returnVector;
    auto styledItems = _loader.GetExpressIDsWithType(schema::IFCSTYLEDITEM);

    for (uint32_t styledItemID : styledItems)
    {
      _loader.MoveToArgumentOffset(styledItemID, 0);

      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        uint32_t representationItem = _loader.GetRefArgument();

        auto styleAssignments = _loader.GetSetArgument();

        for (auto &styleAssignment : styleAssignments)
        {
          uint32_t styleAssignmentID = _loader.GetRefArgument(styleAssignment);
          returnVector[representationItem].emplace_back(styledItemID, styleAssignmentID);
        }
      }
    }
    return returnVector;
  }

  std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> IfcGeometryLoader::PopulateRelMaterialsMap()
  {
    std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> resultVector;
    auto styledItems = _loader.GetExpressIDsWithType(schema::IFCRELASSOCIATESMATERIAL);

    for (uint32_t styledItemID : styledItems)
    {
      _loader.MoveToArgumentOffset(styledItemID, 5);

      uint32_t materialSelect = _loader.GetRefArgument();

      _loader.MoveToArgumentOffset(styledItemID, 4);

      auto RelatedObjects = _loader.GetSetArgument();

      for (auto &ifcRoot : RelatedObjects)
      {
        uint32_t ifcRootID = _loader.GetRefArgument(ifcRoot);
        resultVector[ifcRootID].emplace_back(styledItemID, materialSelect);
      }
    }
    return resultVector;
  }

  std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> IfcGeometryLoader::PopulateMaterialDefinitionsMap()
  {
    std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> resultVector;
    auto matDefs = _loader.GetExpressIDsWithType(schema::IFCMATERIALDEFINITIONREPRESENTATION);

    for (uint32_t styledItemID : matDefs)
    {
      _loader.MoveToArgumentOffset(styledItemID, 2);

      auto representations = _loader.GetSetArgument();

      _loader.MoveToArgumentOffset(styledItemID, 3);

      uint32_t material = _loader.GetRefArgument();

      for (auto &representation : representations)
      {
        uint32_t representationID = _loader.GetRefArgument(representation);
        resultVector[material].emplace_back(styledItemID, representationID);
      }
    }
    return resultVector;
  }

  void IfcGeometryLoader::ReadLinearScalingFactor()
  {
    auto projects = _loader.GetExpressIDsWithType(schema::IFCPROJECT);

    if (projects.size() != 1)
    {
      spdlog::error("[ReadLinearScalingFactor()] unexpected empty ifc project");
      return;
    }

    auto projectEID = projects[0];
    _loader.MoveToArgumentOffset(projectEID, 8);

    auto unitsID = _loader.GetRefArgument();
    _loader.MoveToArgumentOffset(unitsID, 0);

    auto unitIds = _loader.GetSetArgument();

    for (auto &unitID : unitIds)
    {
      auto unitRef = _loader.GetRefArgument(unitID);
      auto lineType = _loader.GetLineType(unitRef);

      if (lineType == schema::IFCSIUNIT)
      {
        _loader.MoveToArgumentOffset(unitRef, 1);
        std::string_view unitType = _loader.GetStringArgument();

        std::string_view unitPrefix;

        _loader.MoveToArgumentOffset(unitRef, 2);
        if (_loader.GetTokenType() == parsing::IfcTokenType::ENUM)
        {
          _loader.StepBack();
          unitPrefix = _loader.GetStringArgument();
        }

        _loader.MoveToArgumentOffset(unitRef, 3);
        std::string_view unitName = _loader.GetStringArgument();

        if (unitType == "LENGTHUNIT" && unitName == "METRE")
        {
          double prefix = ConvertPrefix(unitPrefix);
          _linearScalingFactor *= prefix;
        }
        if (unitType == "PLANEANGLEUNIT")
        {
          _angleUnits = unitName;
        }
      }
      if (lineType == schema::IFCCONVERSIONBASEDUNIT)
      {
        _loader.MoveToArgumentOffset(unitRef, 1);
        std::string_view unitType = _loader.GetStringArgument();
        _loader.MoveToArgumentOffset(unitRef, 3);
        auto unitRefLine = _loader.GetRefArgument();

        _loader.MoveToArgumentOffset(unitRefLine, 1);
        auto ratios = _loader.GetSetArgument();

        /// Scale Correction

        _loader.MoveToArgumentOffset(unitRefLine, 2);
        auto scaleRefLine = _loader.GetRefArgument();

        _loader.MoveToArgumentOffset(scaleRefLine, 1);
        std::string_view unitTypeScale = _loader.GetStringArgument();

        std::string_view unitPrefix;

        _loader.MoveToArgumentOffset(scaleRefLine, 2);
        if (_loader.GetTokenType() == parsing::IfcTokenType::ENUM)
        {
          _loader.StepBack();
          unitPrefix = _loader.GetStringArgument();
        }

        _loader.MoveToArgumentOffset(scaleRefLine, 3);
        std::string_view unitName = _loader.GetStringArgument();

        if (unitTypeScale == "LENGTHUNIT" && unitName == "METRE")
        {
          double prefix = ConvertPrefix(unitPrefix);
          _linearScalingFactor *= prefix;
        }

        double ratio = _loader.GetDoubleArgument(ratios[0]);
        if (unitType == "LENGTHUNIT")
        {
          _linearScalingFactor *= ratio;
        }
        else if (unitType == "AREAUNIT")
        {
          _squaredScalingFactor *= ratio;
        }
        else if (unitType == "VOLUMEUNIT")
        {
          _cubicScalingFactor *= ratio;
        }
        else if (unitType == "PLANEANGLEUNIT")
        {
          _angularScalingFactor *= ratio;
        }
      }
    }
  }

  double IfcGeometryLoader::ConvertPrefix(const std::string_view &prefix)
  {
    if (prefix == "")
      return 1;
    else if (prefix == "EXA")
      return 1e18;
    else if (prefix == "PETA")
      return 1e15;
    else if (prefix == "TERA")
      return 1e12;
    else if (prefix == "GIGA")
      return 1e9;
    else if (prefix == "MEGA")
      return 1e6;
    else if (prefix == "KILO")
      return 1e3;
    else if (prefix == "HECTO")
      return 1e2;
    else if (prefix == "DECA")
      return 10;
    else if (prefix == "DECI")
      return 1e-1;
    else if (prefix == "CENTI")
      return 1e-2;
    else if (prefix == "MILLI")
      return 1e-3;
    else if (prefix == "MICRO")
      return 1e-6;
    else if (prefix == "NANO")
      return 1e-9;
    else if (prefix == "PICO")
      return 1e-12;
    else if (prefix == "FEMTO")
      return 1e-15;
    else if (prefix == "ATTO")
      return 1e-18;
    else
      return 1;
  }

  const std::unordered_map<uint32_t, std::vector<uint32_t>> &IfcGeometryLoader::GetRelVoids() const
  {
    return _relVoids;
  }

  const std::unordered_map<uint32_t, std::vector<uint32_t>> &IfcGeometryLoader::GetRelVoidRels() const
  {
    return _relVoidRel;
  }

  const std::unordered_map<uint32_t, std::vector<uint32_t>> &IfcGeometryLoader::GetRelAggregates() const
  {
    return _relAggregates;
  }

  const std::unordered_map<uint32_t, std::vector<uint32_t>> &IfcGeometryLoader::GetRelNests() const
  {
    return _relNests;
  }

  const std::unordered_map<uint32_t, std::vector<uint32_t>> &IfcGeometryLoader::GetRelElementAggregates() const
  {
    return _relElementAggregates;
  }

  const std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> &IfcGeometryLoader::GetStyledItems() const
  {
    return _styledItems;
  }

  const std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> &IfcGeometryLoader::GetRelMaterials() const
  {
    return _relMaterials;
  }

  const std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> &IfcGeometryLoader::GetMaterialDefinitions() const
  {
    return _materialDefinitions;
  }

  double IfcGeometryLoader::GetLinearScalingFactor() const
  {
    return _linearScalingFactor;
  }

  std::string IfcGeometryLoader::GetAngleUnits() const
  {
    return _angleUnits;
  }

  double IfcGeometryLoader::ReadLenghtMeasure() const
  {
    parsing::IfcTokenType t = _loader.GetTokenType();
    if (t == parsing::IfcTokenType::LABEL)
    {
      _loader.StepBack();
      if (_loader.GetStringArgument() == "IFCNONNEGATIVELENGTHMEASURE")
      {
        _loader.GetTokenType();
        return _loader.GetDoubleArgument();
      }
    }
  }

  std::vector<IfcSegmentIndexSelect> IfcGeometryLoader::ReadCurveIndices() const
  {
    std::vector<IfcSegmentIndexSelect> result;

    parsing::IfcTokenType t = _loader.GetTokenType();
    // If you receive a reference then go to the reference
    if (t == parsing::IfcTokenType::REF)
    {
      _loader.StepBack();
      _loader.MoveToArgumentOffset(_loader.GetRefArgument(), 0);
    }

    _loader.StepBack();
    while (_loader.GetTokenType() != parsing::IfcTokenType::SET_END)
    {
      _loader.StepBack();
      if (_loader.GetTokenType() == parsing::IfcTokenType::LABEL)
      {
        IfcSegmentIndexSelect segment;
        _loader.StepBack();
        segment.type = _loader.GetStringArgument();
        while (_loader.GetTokenType() != parsing::IfcTokenType::SET_END)
        {
          _loader.StepBack();
          while (_loader.GetTokenType() != parsing::IfcTokenType::SET_END)
          {
            _loader.StepBack();
            t = _loader.GetTokenType();
            // If you receive a real then add the real to the list
            if (t == parsing::IfcTokenType::INTEGER)
            {
              _loader.StepBack();
              segment.indexs.push_back(static_cast<uint32_t>(_loader.GetIntArgument()));
            }
          }
        }
        result.push_back(segment);
      }
    }

    return result;
  }

}