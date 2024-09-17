/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.  */

#include "io_helpers.h"
#include <fstream>
#include "../geometry/operations/boolean-utils/fuzzy-bools.h"
#include "../geometry/representation/IfcGeometry.h"
#include "../geometry/representation/geometry.h"

namespace webifc::io
{

    inline glm::dvec2 cmin(glm::dvec2 m, Point p)
    {
        return glm::dvec2(
            std::min(m.x, p.x),
            std::min(m.y, p.y));
    }

    inline glm::dvec2 cmax(glm::dvec2 m, Point p)
    {
        return glm::dvec2(
            std::max(m.x, p.x),
            std::max(m.y, p.y));
    }

    inline glm::dvec2 rescale(Point p, Bounds b, glm::dvec2 size, glm::dvec2 offset)
    {
        return glm::dvec2(
            ((p.x - b.min.x) / (b.max.x - b.min.x)) * size.x + offset.x,
            ((p.y - b.min.y) / (b.max.y - b.min.y)) * size.y + offset.y);
    }

    inline glm::dvec2 rescale(glm::dvec2 p, Bounds b, glm::dvec2 size, glm::dvec2 offset)
    {
        return glm::dvec2(
            ((p.x - b.min.x) / (b.max.x - b.min.x)) * size.x + offset.x,
            ((p.y - b.min.y) / (b.max.y - b.min.y)) * size.y + offset.y);
    }

    inline std::vector<glm::dvec2> rescale(std::vector<glm::dvec2> input, glm::dvec2 size, glm::dvec2 offset)
    {
        std::vector<glm::dvec2> retval;

        glm::dvec2 min(
            DBL_MAX,
            DBL_MAX);

        glm::dvec2 max(
            -DBL_MAX,
            -DBL_MAX);

        for (auto &pt : input)
        {
            min = glm::min(min, pt);
            max = glm::max(max, pt);
        }

        double width = max.x - min.x;
        double height = max.y - min.y;

        double maxSize = std::max(width, height);

        if (width == 0 && height == 0)
        {
            printf("asdf\n");
        }

        for (auto &pt : input)
        {
            // here we invert Y, since the canvas +y is down, but makes more sense to think about +y as up
            retval.emplace_back(
                ((pt.x - min.x) / (maxSize)) * size.x + offset.x,
                (size.y - ((pt.y - min.y) / (maxSize)) * size.y) + offset.y);
        }

        return retval;
    }

    void writeFile(std::string filename, std::string data)
    {
        std::string newFileName(filename.begin(), filename.end());
        std::ofstream out(newFileName);
        out << data;
        out.close();
    }

    void DumpSVGCurve(std::vector<glm::dvec3> points, std::string filename, std::vector<uint32_t> indices)
    {
        std::vector<glm::dvec2> points2D;
        for (auto &pt : points)
        {
            points2D.emplace_back(pt.x, pt.y);
        }
        writeFile(filename, makeSVGLines(points2D, indices));
    }

    void svgMakeLine(glm::dvec2 a, glm::dvec2 b, std::stringstream &svg, std::string col)
    {
        svg << "<line x1=\"" << a.x << "\" y1=\"" << a.y << "\" ";
        svg << "x2=\"" << b.x << "\" y2=\"" << b.y << "\" ";
        svg << "style = \"stroke:" + col + ";stroke-width:1\" />";
    }

    void SVGLinesToString(Bounds bounds, glm::dvec2 size, glm::dvec2 offset, SVGLineSet lineSet, std::stringstream &svg)
    {
        for (auto &line : lineSet.lines)
        {
            if (line.size() > 1)
            {
                for (size_t i = 1; i < line.size(); i++)
                {
                    glm::dvec2 a = rescale(line[i], bounds, size, offset);
                    glm::dvec2 b = rescale(line[i - 1], bounds, size, offset);

                    svgMakeLine(a, b, svg, lineSet.color);
                }
            }
            else
            {
                glm::dvec2 a = rescale(line[0], bounds, size, offset);
                svg << "<circle cx = \"" << a.x << "\" cy = \"" << a.y << "\" r = \"3\" style = \"stroke:rgb(0,0,255);stroke-width:2\" />";
            }
        }
    }

    std::string makeSVGLines(SVGDrawing drawing)
    {
        glm::dvec2 size(2048, 2048);
        glm::dvec2 offset(5, 5);

        Bounds bounds = drawing.GetBounds(size, offset);

        std::stringstream svg;

        svg << "<svg width=\"" << size.x + offset.x * 2 << "\" height=\"" << size.y + offset.y * 2 << " \" xmlns=\"http://www.w3.org/2000/svg\">";

        for (auto &set : drawing.sets)
        {
            SVGLinesToString(bounds, size, offset, set, svg);
        }

        svg << "</svg>";

        return svg.str();
    }

    std::string ToObj(const webifc::geometry::IfcGeometry &geom, size_t &offset, glm::dmat4 transform, double inputScale)
    {
        std::stringstream obj;

        double scale = inputScale;

        for (uint32_t i = 0; i < geom.numPoints; i++)
        {
            glm::dvec4 t = transform * glm::dvec4(geom.GetPoint(i), 1);
            obj << "v " << t.x * scale << " " << t.y * scale << " " << t.z * scale << "\n";
        }

        for (uint32_t i = 0; i < geom.numFaces; i++)
        {
            webifc::geometry::Face f = geom.GetFace(i);
            obj << "f " << (f.i0 + 1 + offset) << "// " << (f.i1 + 1 + offset) << "// " << (f.i2 + 1 + offset) << "//\n";
        }

        offset += geom.numPoints;

        return obj.str();
    }

    std::string ToObj(webifc::geometry::IfcComposedMesh &mesh, webifc::geometry::IfcGeometryProcessor &processor, size_t &offset, glm::dmat4 mat)
    {
        std::string complete;

        glm::dmat4 trans = mat * mesh.transformation;

        auto &geom = processor.GetGeometry(mesh.expressID);

        complete += ToObj(geom, offset, trans);

        for (auto c : mesh.children)
        {
            complete += ToObj(c, processor, offset, trans);
        }

        return complete;
    }

    void DumpGradientCurve(std::vector<webifc::geometry::IfcCurve> &curves, webifc::geometry::IfcCurve &curve, std::string filenameV, std::string filenameH)
    {
        writeFile(filenameV, GradientVerticalToObj(curves));
        writeFile(filenameH, GradientHorizontalToObj(curve));
    }

    void DumpSectionCurves(std::vector<webifc::geometry::IfcCurve> &curves, std::string filename)
    {
        writeFile(filename, GradientVerticalToObj(curves));
    }

    void DumpAlignment(std::vector<webifc::geometry::IfcAlignment> &align, std::string filenameV, std::string filenameH)
    {
        writeFile(filenameV, VAlignmentToObj(align));
        writeFile(filenameH, HAlignmentToObj(align));
    }

    void DumpCrossSections(std::vector<webifc::geometry::IfcCrossSections> &crossSection, std::string filename)
    {
        writeFile(filename, CrossSectionToObj(crossSection));
    }

    void DumpIfcGeometryToPath(const webifc::geometry::IfcGeometry &geom, std::string path, double inputScale)
    {
        size_t offset = 0;
        std::ofstream out(path.c_str());
        out << ToObj(geom, offset, glm::dmat4(1), inputScale);
    }

    std::string ToObj(webifc::geometry::IfcGeometry &geom, size_t &offset, glm::dmat4 transform)
    {
        std::stringstream obj;

        double scale = 1.0;

        for (uint32_t i = 0; i < geom.numPoints; i++)
        {
            auto p = glm::dvec4(geom.GetPoint(i), 1);
            glm::dvec4 t = transform * p;
            obj << "v " << t.x * scale << " " << t.y * scale << " " << t.z * scale << "\n";
        }

        for (uint32_t i = 0; i < geom.numFaces; i++)
        {
            webifc::geometry::Face f = geom.GetFace(i);
            obj << "f " << (f.i0 + 1 + offset) << "// " << (f.i1 + 1 + offset) << "// " << (f.i2 + 1 + offset) << "//\n";
        }

        offset += geom.numPoints;

        return obj.str();
    }

    void DumpIfcGeometry(webifc::geometry::IfcGeometry &geom, std::string filename)
    {
        size_t offset = 0;
        writeFile(filename, ToObj(geom, offset));
    }

    void DumpFlatMesh(webifc::geometry::IfcFlatMesh &mesh, webifc::geometry::IfcGeometryProcessor &processor, std::string filename)
    {
        size_t offset = 0;
        writeFile(filename, ToObj(mesh, processor, offset));
    }

    std::string ToObj(webifc::geometry::IfcFlatMesh &mesh, webifc::geometry::IfcGeometryProcessor &processor, size_t &offset, glm::dmat4 mat)
    {
        std::string complete;

        for (auto &geom : mesh.geometries)
        {
            auto flatGeom = processor.GetGeometry(geom.geometryExpressID);

            glm::dmat4 trans = mat * geom.transformation;

            complete += ToObj(flatGeom, offset, trans);
        }

        return complete;
    }

    void DumpSVGLines(std::vector<std::vector<glm::dvec2>> lines, std::string filename)
    {
        writeFile(filename, makeSVGLines(lines));
    }

    std::string makeSVGLines(std::vector<glm::dvec2> input, std::vector<uint32_t> indices)
    {
        glm::dvec2 size(512, 512);
        glm::dvec2 offset(5, 5);

        auto rescaled = rescale(input, size, offset);

        std::stringstream svg;

        svg << "<svg width=\"" << size.x + offset.x * 2 << "\" height=\"" << size.y + offset.y * 2 << "\" xmlns=\"http://www.w3.org/2000/svg\" >";

        if (!rescaled.empty())
        {
            for (int i = 1; i < 2; i++)
            {
                auto &start = rescaled[i - 1];
                auto &end = rescaled[i];
                svg << "<line x1=\"" << start.x << "\" y1=\"" << start.y << "\" ";
                svg << "x2=\"" << end.x << "\" y2=\"" << end.y << "\" ";
                svg << "style = \"stroke:rgb(0,255,0);stroke-width:2\" />";
            }

            for (size_t i = 2; i < rescaled.size() - 1; i++)
            {
                auto &start = rescaled[i - 1];
                auto &end = rescaled[i];
                svg << "<line x1=\"" << start.x << "\" y1=\"" << start.y << "\" ";
                svg << "x2=\"" << end.x << "\" y2=\"" << end.y << "\" ";
                svg << "style = \"stroke:rgb(0,0,0);stroke-width:2\" />";
            }

            for (size_t i = rescaled.size() - 1; i < rescaled.size(); i++)
            {
                auto &start = rescaled[i - 1];
                auto &end = rescaled[i];
                svg << "<line x1=\"" << start.x << "\" y1=\"" << start.y << "\" ";
                svg << "x2=\"" << end.x << "\" y2=\"" << end.y << "\" ";
                svg << "style = \"stroke:rgb(255,0,0);stroke-width:2\" />";
            }
        }

        for (size_t i = 0; i < indices.size(); i += 3)
        {
            glm::dvec2 a = rescaled[indices[i + 0]];
            glm::dvec2 b = rescaled[indices[i + 1]];
            glm::dvec2 c = rescaled[indices[i + 2]];

            svg << "<polygon points=\"" << a.x << "," << a.y << " " << b.x << "," << b.y << " " << c.x << "," << c.y << "\" style=\"fill:gray; stroke:none; stroke - width:0\" />`;";
        }

        svg << "</svg>";

        return svg.str();
    }

    std::string makeSVGLines(std::vector<std::vector<glm::dvec2>> lines)
    {
        glm::dvec2 size(2048, 2048);
        glm::dvec2 offset(5, 5);

        Bounds bounds = getBounds(lines, size, offset);

        std::stringstream svg;

        svg << "<svg width=\"" << size.x + offset.x * 2 << "\" height=\"" << size.y + offset.y * 2 << " \" xmlns=\"http://www.w3.org/2000/svg\">";

        for (auto &line : lines)
        {
            if (line.size() > 1)
            {
                for (size_t i = 1; i < line.size(); i++)
                {
                    glm::dvec2 a = rescale(line[i], bounds, size, offset);
                    glm::dvec2 b = rescale(line[i - 1], bounds, size, offset);

                    svgMakeLine(a, b, svg);
                }
            }
            else
            {
                glm::dvec2 a = rescale(line[0], bounds, size, offset);
                svg << "<circle cx = \"" << a.x << "\" cy = \"" << a.y << "\" r = \"3\" style = \"stroke:rgb(0,0,255);stroke-width:2\" />";
            }
        }

        svg << "</svg>";

        return svg.str();
    }

    void DumpMesh(webifc::geometry::IfcComposedMesh &mesh, webifc::geometry::IfcGeometryProcessor &processor, std::string filename)
    {
        size_t offset = 0;
        writeFile(filename, ToObj(mesh, processor, offset, webifc::geometry::NormalizeIFC));
    }

    std::string GradientVerticalToObj(const std::vector<webifc::geometry::IfcCurve> &geom)
    {
        std::stringstream obj;

        for (uint32_t ia = 0; ia < geom.size(); ia++)
        {
            if (geom[ia].points.size() > 0)
            {
                for (uint32_t ic = 0; ic < geom[ia].points.size(); ic++)
                {
                    glm::dvec3 t = glm::dvec4(geom[ia].points[ic].x, geom[ia].points[ic].y, 0, 1);
                    obj << "v " << t.x << " " << t.y << " " << t.z << "\n";
                }
            }
        }

        uint32_t idx = 0;

        for (uint32_t ia = 0; ia < geom.size(); ia++)
        {
            if (geom[ia].points.size() > 0)
            {
                for (uint32_t ic = 0; ic < geom[ia].points.size() - 1; ic++)
                {
                    obj << "l " << (idx) << " " << (idx + 1) << "\n";
                    idx++;
                }
            }
        }

        return obj.str();
    }

    std::string GradientHorizontalToObj(const webifc::geometry::IfcCurve &geom)
    {
        std::stringstream obj;

        for (uint32_t ic = 0; ic < geom.points.size(); ic++)
        {
            glm::dvec3 t = glm::dvec4(geom.points[ic].x, geom.points[ic].y, 0, 1);
            obj << "v " << t.x << " " << t.y << " " << t.z << "\n";
        }

        uint32_t idx = 0;

        for (uint32_t ic = 0; ic < geom.points.size() - 1; ic++)
        {
            obj << "l " << (idx) << " " << (idx + 1) << "\n";
            idx++;
        }

        return obj.str();
    }

    std::string VAlignmentToObj(const std::vector<webifc::geometry::IfcAlignment> &geom)
    {
        std::stringstream obj;

        for (uint32_t ia = 0; ia < geom.size(); ia++)
        {
            for (uint32_t ic = 0; ic < geom[ia].Vertical.curves.size(); ic++)
            {
                for (uint32_t ip = 0; ip < geom[ia].Vertical.curves[ic].points.size(); ip++)
                {
                    glm::dvec3 t = glm::dvec4(geom[ia].Vertical.curves[ic].points[ip].x, geom[ia].Vertical.curves[ic].points[ip].y, 0, 1);
                    obj << "v " << t.x << " " << t.y << " " << t.z << "\n";
                }
            }
        }

        uint32_t idx = 0;

        for (uint32_t ia = 0; ia < geom.size(); ia++)
        {
            for (uint32_t ic = 0; ic < geom[ia].Vertical.curves.size(); ic++)
            {
                for (uint32_t ip = 0; ip < geom[ia].Vertical.curves[ic].points.size() - 1; ip++)
                {
                    obj << "l " << (idx) << " " << (idx + 1) << "\n";
                    ;
                    idx++;
                }
            }
        }

        return obj.str();
    }

    std::string HAlignmentToObj(const std::vector<webifc::geometry::IfcAlignment> &geom)
    {
        std::stringstream obj;

        for (uint32_t ia = 0; ia < geom.size(); ia++)
        {
            for (uint32_t ic = 0; ic < geom[ia].Horizontal.curves.size(); ic++)
            {
                for (uint32_t ip = 0; ip < geom[ia].Horizontal.curves[ic].points.size(); ip++)
                {
                    glm::dvec3 t = glm::dvec4(geom[ia].Horizontal.curves[ic].points[ip].x, geom[ia].Horizontal.curves[ic].points[ip].y, 0, 1);
                    obj << "v " << t.x << " " << t.y << " " << t.z << "\n";
                }
            }
        }

        uint32_t idx = 0;

        for (uint32_t ia = 0; ia < geom.size(); ia++)
        {
            for (uint32_t ic = 0; ic < geom[ia].Horizontal.curves.size(); ic++)
            {
                for (uint32_t ip = 0; ip < geom[ia].Horizontal.curves[ic].points.size() - 1; ip++)
                {
                    obj << "l " << (idx) << " " << (idx + 1) << "\n";
                    ;
                    idx++;
                }
            }
        }

        return obj.str();
    }

    std::string CrossSectionToObj(const std::vector<webifc::geometry::IfcCrossSections> &geom)
    {
        std::stringstream obj;

        for (uint32_t ia = 0; ia < geom.size(); ia++)
        {
            for (uint32_t ic = 0; ic < geom[ia].curves.size(); ic++)
            {
                for (uint32_t ip = 0; ip < geom[ia].curves[ic].points.size(); ip++)
                {
                    glm::dvec3 t = glm::dvec4(geom[ia].curves[ic].points[ip].x,geom[ia].curves[ic].points[ip].y, 0, 1);
                    obj << "v " << t.x << " " << t.y << " " << t.z << "\n";
                }
            }
        }

        uint32_t idx = 0;

        for (uint32_t ia = 0; ia < geom.size(); ia++)
        {
            for (uint32_t ic = 0; ic < geom[ia].curves.size(); ic++)
            {
                for (uint32_t ip = 0; ip < geom[ia].curves[ic].points.size() - 1; ip++)
                {
                    obj << "l " << (idx) << " " << (idx + 1) << "\n";;
                    idx++;
                }
            }
        }

        return obj.str();
    }

    Bounds getBounds(std::vector<std::vector<glm::dvec2>> input, glm::dvec2 size, glm::dvec2 offset)
    {
        std::vector<glm::dvec2> retval;

        glm::dvec2 min(
            DBL_MAX,
            DBL_MAX);

        glm::dvec2 max(
            -DBL_MAX,
            -DBL_MAX);

        for (auto &loop : input)
        {
            for (auto &point : loop)
            {
                min = glm::min(min, point);
                max = glm::max(max, point);
            }
        }

        double width = max.x - min.x;
        double height = max.y - min.y;

        if (width == 0 && height == 0)
        {
            printf("asdf");
        }

        return {min, max};
    }

}