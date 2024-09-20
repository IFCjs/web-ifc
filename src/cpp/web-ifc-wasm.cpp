/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */
 
#include <string>
#include <algorithm>
#include <vector>
#include <stack>
#include <cstdint>
#include <memory>
#include <emscripten/bind.h>
#include <spdlog/spdlog.h>
#include "modelmanager/ModelManager.h"
#include "version.h"

namespace webifc::parsing { 
    void p21encode(std::string_view input, std::ostringstream &output);
    std::string p21decode(std::string_view & str);    
}

#ifdef __EMSCRIPTEN_PTHREADS__
    constexpr bool MT_ENABLED = true;
#else
    constexpr bool MT_ENABLED = false;
#endif

webifc::manager::ModelManager manager = new webifc::manager::ModelManager(MT_ENABLED);

int CreateModel(webifc::manager::LoaderSettings settings) {
    return manager.CreateModel(settings);
}

void CloseAllModels() {
    return manager.CloseAllModels();
}

int OpenModel(webifc::manager::LoaderSettings settings, emscripten::val callback)
{
    auto modelID = manager.CreateModel(settings);
    const std::function<uint32_t(char *, size_t, size_t)> loaderFunc= [callback](char* dest, size_t sourceOffset, size_t destSize) {    
        emscripten::val retVal = callback((uint32_t)dest,sourceOffset, destSize);
        uint32_t len = retVal.as<uint32_t>();
        return len;
    };
    
    manager.GetIfcLoader(modelID)->LoadFile(loaderFunc);
    return modelID;
}

void SaveModel(uint32_t modelID, emscripten::val callback)
{
    if (!manager.IsModelOpen(modelID)) return;
    manager.GetIfcLoader(modelID)->SaveFile([&](char* src, size_t srcSize)
        {
            emscripten::val retVal = callback((uint32_t)src, srcSize);
        }
    );
}

int GetModelSize(uint32_t modelID) {
    return manager.IsModelOpen(modelID) ? manager.GetIfcLoader(modelID)->GetTotalSize() : 0;
}

void CloseModel(uint32_t modelID) {
    return manager.CloseModel(modelID);
}

webifc::geometry::IfcFlatMesh GetFlatMesh(uint32_t modelID, uint32_t expressID)
{
    if (!manager.IsModelOpen(modelID)) return {};  
    webifc::geometry::IfcFlatMesh mesh = manager.GetGeometryProcessor(modelID)->GetFlatMesh(expressID);
    for (auto& geom : mesh.geometries) manager.GetGeometryProcessor(modelID)->GetGeometry(geom.geometryExpressID).GetVertexData();
    return mesh;
}

void StreamMeshes(uint32_t modelID, const std::vector<uint32_t> & expressIds, emscripten::val callback) {
    if (!manager.IsModelOpen(modelID)) return;    
    auto geomLoader = manager.GetGeometryProcessor(modelID);
    int index = 0;
    int total = expressIds.size();

    for (const auto& id : expressIds)
    {
        // read the mesh from IFC
        webifc::geometry::IfcFlatMesh mesh = geomLoader->GetFlatMesh(id);

        // prepare the geometry data
        for (auto& geom : mesh.geometries)
        {
            auto& flatGeom = geomLoader->GetGeometry(geom.geometryExpressID);
            flatGeom.GetVertexData();
        }   

        if (!mesh.geometries.empty())
        {
            // transfer control to client, geometry data is alive for the time of the callback
            callback(mesh, index, total);
        }

        // clear geometry, freeing memory, client is expected to have consumed the data
        geomLoader->Clear();

        index++;
    }
}

void StreamMeshesWithExpressID(uint32_t modelID, emscripten::val expressIdsVal, emscripten::val callback)
{
    std::vector<uint32_t> expressIds;

    uint32_t size = expressIdsVal["length"].as<uint32_t>();
    for (size_t i=0; i < size; i++) 
    {
        emscripten::val expressIdVal = expressIdsVal[std::to_string(i)];

        uint32_t expressId = expressIdVal.as<uint32_t>();

        expressIds.push_back(expressId);
    }
    
    StreamMeshes(modelID, expressIds, callback);
}

void StreamAllMeshesWithTypes(uint32_t modelID, const std::vector<uint32_t>& types, emscripten::val callback)
{
    if (!manager.IsModelOpen(modelID)) return;
    auto loader = manager.GetIfcLoader(modelID);
   
    for (auto& type : types)
    {
        auto elements = loader->GetExpressIDsWithType(type);
        StreamMeshes(modelID, elements, callback);
    }
}

void StreamAllMeshesWithTypesVal(uint32_t modelID, emscripten::val typesVal, emscripten::val callback)
{
    if (!manager.IsModelOpen(modelID)) return;
    std::vector<uint32_t> types;
    uint32_t size = typesVal["length"].as<uint32_t>();
    uint32_t index = 0;
    while (index < size)
    {
        emscripten::val typeVal = typesVal[std::to_string(index++)];
        uint32_t type = typeVal.as<uint32_t>();
        types.push_back(type);
    }
    StreamAllMeshesWithTypes(modelID, types, callback);
}

void StreamAllMeshes(uint32_t modelID, emscripten::val callback) {
    if (!manager.IsModelOpen(modelID)) return;
    std::vector<uint32_t> types;

    for (auto& type : manager.GetSchemaManager().GetIfcElementList())
    {
        if (type == webifc::schema::IFCOPENINGELEMENT || type == webifc::schema::IFCSPACE || type == webifc::schema::IFCOPENINGSTANDARDCASE)
        {
            continue;
        }

        types.push_back(type);
    }
    StreamAllMeshesWithTypes(modelID, types, callback);
}

std::vector<webifc::geometry::IfcFlatMesh> LoadAllGeometry(uint32_t modelID)
{
    if (!manager.IsModelOpen(modelID)) return std::vector<webifc::geometry::IfcFlatMesh>();
    auto loader = manager.GetIfcLoader(modelID);
    auto geomLoader = manager.GetGeometryProcessor(modelID);
    std::vector<webifc::geometry::IfcFlatMesh> meshes;

    for (auto type : manager.GetSchemaManager().GetIfcElementList())
    {
        auto elements = loader->GetExpressIDsWithType(type);

        if (type == webifc::schema::IFCOPENINGELEMENT || type == webifc::schema::IFCSPACE || type == webifc::schema::IFCOPENINGSTANDARDCASE)
        {
            continue;
        }

        for (uint32_t i = 0; i < elements.size(); i++)
        {
            webifc::geometry::IfcFlatMesh mesh = geomLoader->GetFlatMesh(elements[i]);
            for (auto& geom : mesh.geometries)
            {
                auto& flatGeom = geomLoader->GetGeometry(geom.geometryExpressID);
                flatGeom.GetVertexData();
            }   
            meshes.push_back(std::move(mesh));
        }
    }

    return meshes;
}

webifc::geometry::IfcGeometry GetGeometry(uint32_t modelID, uint32_t expressID)
{
    return manager.IsModelOpen(modelID) ? manager.GetGeometryProcessor(modelID)->GetGeometry(expressID) : webifc::geometry::IfcGeometry();
}

std::vector<webifc::geometry::IfcCrossSections> GetAllCrossSections(uint32_t modelID,uint8_t dimensions)
{
    if (!manager.IsModelOpen(modelID)) return std::vector<webifc::geometry::IfcCrossSections>();
    auto geomLoader =  manager.GetGeometryProcessor(modelID);

    std::vector<uint32_t> typeList; 
    typeList.push_back(webifc::schema::IFCSECTIONEDSOLIDHORIZONTAL);  
    typeList.push_back(webifc::schema::IFCSECTIONEDSOLID);
    typeList.push_back(webifc::schema::IFCSECTIONEDSURFACE);

    std::vector<webifc::geometry::IfcCrossSections> crossSections;

    for(auto& type: typeList)
    {
        auto elements = manager.GetIfcLoader(modelID)->GetExpressIDsWithType(type);

        for (size_t i = 0; i < elements.size(); i++)
        {
            webifc::geometry::IfcCrossSections crossSection;
            if (dimensions == 2) crossSection = geomLoader->GetLoader().GetCrossSections2D(elements[i]);
            else crossSection = geomLoader->GetLoader().GetCrossSections3D(elements[i]);
            crossSections.push_back(crossSection);
        }
    }

    return crossSections;
}

std::vector<webifc::geometry::IfcAlignment> GetAllAlignments(uint32_t modelID)
{
    if (!manager.IsModelOpen(modelID)) return std::vector<webifc::geometry::IfcAlignment>();
    auto geomLoader = manager.GetGeometryProcessor(modelID);
    auto type = webifc::schema::IFCALIGNMENT;

    auto elements = manager.GetIfcLoader(modelID)->GetExpressIDsWithType(type);

    std::vector<webifc::geometry::IfcAlignment> alignments;

    for (size_t i = 0; i < elements.size(); i++)
    {
        webifc::geometry::IfcAlignment alignment = geomLoader->GetLoader().GetAlignment(elements[i]);
        alignment.transform(geomLoader->GetCoordinationMatrix());
        alignments.push_back(alignment);
    }

    return alignments;
}

void SetGeometryTransformation(uint32_t modelID, std::array<double, 16> m)
{
    if (manager.IsModelOpen(modelID)) manager.GetGeometryProcessor(modelID)->SetTransformation(m);
}

std::array<double, 16> GetCoordinationMatrix(uint32_t modelID)
{
    return  manager.IsModelOpen(modelID) ?  manager.GetGeometryProcessor(modelID)->GetFlatCoordinationMatrix(): std::array<double,16>();
}

std::vector<uint32_t> GetLineIDsWithType(uint32_t modelID, emscripten::val types)
{
    if (!manager.IsModelOpen(modelID)) return {};
    auto loader = manager.GetIfcLoader(modelID);
    std::vector<uint32_t> expressIDs;

    uint32_t size = types["length"].as<uint32_t>();
    for (uint32_t i=0; i < size; i++) {
    
        uint32_t type = types[std::to_string(i)].as<uint32_t>();
        auto ids = loader->GetExpressIDsWithType(type);
        expressIDs.insert(expressIDs.end(),ids.begin(),ids.end());    
    }
    return expressIDs;
}

std::vector<uint32_t> GetInversePropertyForItem(uint32_t modelID, uint32_t expressID, emscripten::val targetTypes, uint32_t position, bool set)
{
    if (!manager.IsModelOpen(modelID)) return {};
    auto loader = manager.GetIfcLoader(modelID);
    std::vector<uint32_t> inverseIDs;
    auto expressIDs = GetLineIDsWithType(modelID,targetTypes);
    for (auto foundExpressID : expressIDs)
    {
      loader->MoveToLineArgument(foundExpressID, position);

      webifc::parsing::IfcTokenType t = loader->GetTokenType();
      if (t == webifc::parsing::IfcTokenType::REF) 
      {
        loader->StepBack();
        uint32_t val = loader->GetRefArgument();
        if (val == expressID)
        {
          inverseIDs.push_back(foundExpressID);
          if (!set) return inverseIDs;
        }
      }
      else if (t == webifc::parsing::IfcTokenType::SET_BEGIN)
      {
          while (!loader->IsAtEnd())
          {
              webifc::parsing::IfcTokenType setValueType = loader->GetTokenType();
              if (setValueType == webifc::parsing::IfcTokenType::SET_END) break;
              if (setValueType == webifc::parsing::IfcTokenType::REF) 
              {
                loader->StepBack();
                uint32_t val = loader->GetRefArgument();
                if (val == expressID)
                {
                  inverseIDs.push_back(foundExpressID);
                  if (!set) return inverseIDs;
                }
              }
          }
      }
    }
    return inverseIDs;
}

bool ValidateExpressID(uint32_t modelID, uint32_t expressId) {
   return manager.IsModelOpen(modelID) ?  manager.GetIfcLoader(modelID)->IsValidExpressID(expressId) : false;
}

void ExtendLineStorage(uint32_t modelID, uint32_t lineStorageSize) {
    if (manager.IsModelOpen(modelID)) manager.GetIfcLoader(modelID)->ExtendLineStorage(lineStorageSize);
}

uint32_t GetNextExpressID(uint32_t modelID, uint32_t expressId) { 
    return manager.IsModelOpen(modelID) ?  manager.GetIfcLoader(modelID)->GetNextExpressID(expressId) : 0;
}

std::vector<uint32_t> GetAllLines(uint32_t modelID) {
    return manager.IsModelOpen(modelID) ? manager.GetIfcLoader(modelID)->GetAllLines() : std::vector<uint32_t>();
}

bool WriteValue(uint32_t modelID, webifc::parsing::IfcTokenType t, emscripten::val value)
{
    bool responseCode = true;
    auto loader =manager.GetIfcLoader(modelID);
    switch (t)
    {
    case webifc::parsing::IfcTokenType::STRING:
    case webifc::parsing::IfcTokenType::ENUM:
    {
        std::string copy;
        if (value.isTrue()) copy="T";
        else if (value.isFalse()) copy="F";
        else copy = value.as<std::string>();
        if (copy == "true") copy="T";
        else if (copy == "false") copy="F";
        uint16_t length = copy.size();
        loader->Push<uint16_t>((uint16_t)length);
        loader->Push((void*)copy.c_str(), copy.size());

        break;
    }
    case webifc::parsing::IfcTokenType::REF:
    {
        uint32_t val = value.as<uint32_t>();
        loader->Push<uint32_t>(val);

        break;
    }
    case webifc::parsing::IfcTokenType::REAL:
    {
        double val = value.as<double>();
        loader->PushDouble(val);
        break;
    }
    case webifc::parsing::IfcTokenType::INTEGER:
    {
        int val = value.as<int>();
        loader->PushInt(val);
        break;
    }
    default:
        // use undefined to signal val parse issue
        loader->Push<uint8_t>('?');
        responseCode = false;
    }
    return responseCode;
}

bool WriteSet(uint32_t modelID, emscripten::val& val)
{
    bool responseCode = true;
    auto loader = manager.GetIfcLoader(modelID);
    loader->Push<uint8_t>(webifc::parsing::IfcTokenType::SET_BEGIN);

    uint32_t size = val["length"].as<uint32_t>();
    for (size_t i=0; i < size;i++) 
    {
        emscripten::val child = val[std::to_string(i)];
        if (child.isNull()) loader->Push<uint8_t>(webifc::parsing::IfcTokenType::EMPTY);
        else if (child.isUndefined()) loader->Push<uint8_t>(webifc::parsing::IfcTokenType::EMPTY);
        else if (child.isArray()) WriteSet(modelID,child);
        else if (child["value"].isArray())
        {
            
            emscripten::val innerVal = child["value"];
            webifc::parsing::IfcTokenType type = static_cast<webifc::parsing::IfcTokenType>(child["type"].as<uint32_t>());
            loader->Push(webifc::parsing::IfcTokenType::SET_BEGIN);
            uint32_t sz = innerVal["length"].as<uint32_t>();
            for (size_t z=0; z < sz;z++) {
                loader->Push<uint8_t>(type);
                if (type == webifc::parsing::IfcTokenType::INTEGER) {
                    int value = innerVal[std::to_string(z)].as<int>();
                    loader->PushInt(value);
                } else {
                    double value = innerVal[std::to_string(z)].as<double>();
                    loader->PushDouble(value);
                }
            }
            loader->Push(webifc::parsing::IfcTokenType::SET_END);
        }
        else if (child["type"].isNumber())
        {
            webifc::parsing::IfcTokenType type = static_cast<webifc::parsing::IfcTokenType>(child["type"].as<uint32_t>());
            loader->Push(type);
            switch(type)
            {
                case webifc::parsing::IfcTokenType::LINE_END:
                case webifc::parsing::IfcTokenType::EMPTY:
                case webifc::parsing::IfcTokenType::SET_BEGIN:
                case webifc::parsing::IfcTokenType::SET_END:
                {
                    // ignore, we should not be seeing this
                    break;
                }
                case webifc::parsing::IfcTokenType::UNKNOWN:
                {
                    // ignore, already pushed above, no further data
                    break;
                }
                case webifc::parsing::IfcTokenType::LABEL:
                {
                    auto label = child["label"];
                    auto valueType = static_cast<webifc::parsing::IfcTokenType>(child["valueType"].as<uint32_t>());
                    auto value = child["value"];

                    std::string copy = label.as<std::string>();

                    uint16_t length = copy.size();
                    loader->Push<uint16_t>((uint16_t)length);
                    loader->Push((void*)copy.c_str(), copy.size());

                    loader->Push<uint8_t>(webifc::parsing::IfcTokenType::SET_BEGIN);

                    loader->Push<uint8_t>(valueType);
                    WriteValue(modelID,valueType, value);

                    loader->Push<uint8_t>(webifc::parsing::IfcTokenType::SET_END);

                    break;
                }
                case webifc::parsing::IfcTokenType::STRING:
                case webifc::parsing::IfcTokenType::ENUM:
                case webifc::parsing::IfcTokenType::REF:
                case webifc::parsing::IfcTokenType::REAL:
                case webifc::parsing::IfcTokenType::INTEGER:
                {
                    WriteValue(modelID,type, child["value"]);
                    break;
                }
                default:
                    break;
            }
        } 
        else if (child.isNumber() || child.isTrue() || child.isFalse() || child.isString())
        {
            webifc::parsing::IfcTokenType type;
            if (child.isNumber()) type = webifc::parsing::IfcTokenType::REAL;
            else if (child.isString()) type = webifc::parsing::IfcTokenType::STRING;
            else type = webifc::parsing::IfcTokenType::ENUM;
            loader->Push<uint8_t>(type);
            WriteValue(modelID, type, child);
        }
        else
        {
            spdlog::error("[WriteSet()] Error in writeline: unknown object received");
            responseCode = false;
        }
    }

    loader->Push<uint8_t>(webifc::parsing::IfcTokenType::SET_END);
    return responseCode;
}

std::string GetNameFromTypeCode(uint32_t type) {
    return std::string(manager.GetSchemaManager().IfcTypeCodeToType(type));
}

uint32_t GetTypeCodeFromName(std::string typeName) {
    return manager.GetSchemaManager().IfcTypeToTypeCode(typeName);
}

bool IsIfcElement(uint32_t type) {
    return manager.GetSchemaManager().IsIfcElement(type);
}

bool WriteHeaderLine(uint32_t modelID,uint32_t type, emscripten::val parameters)
{
    if (!manager.IsModelOpen(modelID)) return false;
    auto loader = manager.GetIfcLoader(modelID);
    uint32_t start = loader->GetTotalSize();
    std::string ifcName = manager.GetSchemaManager().IfcTypeCodeToType(type);
    std::transform(ifcName.begin(), ifcName.end(), ifcName.begin(), ::toupper);
    loader->Push<uint8_t>(webifc::parsing::IfcTokenType::LABEL);
    loader->Push<uint16_t>((uint16_t)ifcName.size());
    loader->Push((void*)ifcName.data(), ifcName.size());
    bool responseCode = WriteSet(modelID,parameters);
    loader->Push<uint8_t>(webifc::parsing::IfcTokenType::LINE_END);
    loader->AddHeaderLineTape(type, start);
    return responseCode;
}

void RemoveLine(uint32_t modelID, uint32_t expressID) {
    if (manager.IsModelOpen(modelID)) manager.GetIfcLoader(modelID)->RemoveLine(expressID);
}

bool WriteLine(uint32_t modelID, uint32_t expressID, uint32_t type, emscripten::val parameters)
{
    if (!manager.IsModelOpen(modelID)) return false;
    auto loader = manager.GetIfcLoader(modelID);
    uint32_t start = loader->GetTotalSize();

    // line ID
    loader->Push<uint8_t>(webifc::parsing::IfcTokenType::REF);
    loader->Push<uint32_t>(expressID);

    // line TYPE
    std::string ifcName = manager.GetSchemaManager().IfcTypeCodeToType(type);
    std::transform(ifcName.begin(), ifcName.end(), ifcName.begin(), ::toupper);
    loader->Push<uint8_t>(webifc::parsing::IfcTokenType::LABEL);
    loader->Push<uint16_t>((uint16_t)ifcName.size());
    loader->Push((void*)ifcName.data(), ifcName.size());
    bool responseCode = WriteSet(modelID,parameters);
    // end line
    loader->Push<uint8_t>(webifc::parsing::IfcTokenType::LINE_END);

    loader->UpdateLineTape(expressID, type, start);
    return responseCode;
}


emscripten::val ReadValue(uint32_t modelID, webifc::parsing::IfcTokenType t)
{
    auto loader = manager.GetIfcLoader(modelID);
    switch (t)
    {
    case webifc::parsing::IfcTokenType::STRING:
    {
        return emscripten::val(loader->GetDecodedStringArgument());
    }
    case webifc::parsing::IfcTokenType::ENUM:
    {
        std::string_view s = loader->GetStringArgument();
        return emscripten::val(std::string(s));
    }
    case webifc::parsing::IfcTokenType::REAL:
    {
        std::string_view s = loader->GetDoubleArgumentAsString();
        return emscripten::val(std::string(s));
    }
    case webifc::parsing::IfcTokenType::INTEGER:
    {
        long d = loader->GetIntArgument();
        return emscripten::val(d);
    }
    case webifc::parsing::IfcTokenType::REF:
    {
        uint32_t ref = loader->GetRefArgument();
        return emscripten::val(ref);
    }
    default:
        // use undefined to signal val parse issue
        return emscripten::val::undefined();
    }
}

emscripten::val GetArgs(uint32_t modelID, bool inObject=false, bool inList=false)
{
    auto loader = manager.GetIfcLoader(modelID);
    auto arguments = emscripten::val::array();
    size_t size = 0;
    bool endOfLine = false;
    while (!loader->IsAtEnd() && !endOfLine)
    {
        webifc::parsing::IfcTokenType t = loader->GetTokenType();

        switch (t)
        {
            case webifc::parsing::IfcTokenType::LINE_END:
            {
                endOfLine = true;
                break;
            }
            case webifc::parsing::IfcTokenType::EMPTY:
            {
                arguments.set(size++,emscripten::val::null());
                break;
            }
            case webifc::parsing::IfcTokenType::SET_BEGIN:
            {
                arguments.set(size++,GetArgs(modelID, false, true));
                break;
            }
            case webifc::parsing::IfcTokenType::SET_END:
            {
                endOfLine = true;
                break;
            }
            case webifc::parsing::IfcTokenType::LABEL:
            {
                // read label
                auto obj = emscripten::val::object(); 
                obj.set("type", emscripten::val(static_cast<uint32_t>(webifc::parsing::IfcTokenType::LABEL)));
                loader->StepBack();
                auto s=loader->GetStringArgument();
                auto typeCode = manager.GetSchemaManager().IfcTypeToTypeCode(s);
                obj.set("typecode", emscripten::val(typeCode));
                // read set open
                loader->GetTokenType();
                obj.set("value", GetArgs(modelID,true));
                arguments.set(size++, obj);
                break;
            }
            case webifc::parsing::IfcTokenType::STRING:
            case webifc::parsing::IfcTokenType::ENUM:
            case webifc::parsing::IfcTokenType::REAL:
            case webifc::parsing::IfcTokenType::INTEGER:
            case webifc::parsing::IfcTokenType::REF:
            {
                loader->StepBack();
                emscripten::val obj;
                if (inObject) obj = ReadValue(modelID,t);
                else {
                    obj = emscripten::val::object(); 
                    obj.set("type", emscripten::val(static_cast<uint32_t>(t)));
                    obj.set("value", ReadValue(modelID,t));
                }
                arguments.set(size++, obj);
                break;
            }
            default:
                break;
        }
    }
    if (size == 0 && !inList) return emscripten::val::null();
    if (size == 1 && inObject) return arguments[0];
    return arguments;
}

emscripten::val GetHeaderLine(uint32_t modelID, uint32_t headerType)
{
    if (!manager.IsModelOpen(modelID)) return emscripten::val::undefined();
    auto loader = manager.GetIfcLoader(modelID);
    auto lines = loader->GetHeaderLinesWithType(headerType);

    if(lines.size() <= 0) return emscripten::val::undefined(); 
    auto line = lines[0];
    loader->MoveToHeaderLineArgument(line, 0);

    std::string s(manager.GetSchemaManager().IfcTypeCodeToType(headerType));
    auto arguments = GetArgs(modelID);
    auto retVal = emscripten::val::object();
    retVal.set("ID", line);
    retVal.set("type", s);
    retVal.set("arguments", arguments);
    return retVal;
}

emscripten::val GetLine(uint32_t modelID, uint32_t expressID)
{
    if (!manager.IsModelOpen(modelID)) return emscripten::val::undefined();
    auto loader = manager.GetIfcLoader(modelID);
    if (!loader->IsValidExpressID(expressID)) return emscripten::val::object();
    uint32_t lineType = loader->GetLineType(expressID);
    if (lineType==0) return emscripten::val::object();

    loader->MoveToArgumentOffset(expressID, 0);

    auto arguments = GetArgs(modelID);

    auto retVal = emscripten::val::object();
    retVal.set(emscripten::val("ID"), expressID);
    retVal.set(emscripten::val("type"), lineType);
    retVal.set(emscripten::val("arguments"), arguments);
    return retVal;
}

uint32_t GetLineType(uint32_t modelID, uint32_t expressID) {
    return manager.IsModelOpen(modelID) ?  manager.GetIfcLoader(modelID)->GetLineType(expressID) :  0;
}

std::string GetVersion() {
    return std::string(WEB_IFC_VERSION_NUMBER);
}

uint32_t GetMaxExpressID(uint32_t modelID) {
    return manager.IsModelOpen(modelID) ? manager.GetIfcLoader(modelID)->GetMaxExpressId() : 0;
}

bool IsModelOpen(uint32_t modelID) {
    return manager.IsModelOpen(modelID);
}

void SetLogLevel(uint8_t levelArg) {
    manager.SetLogLevel(levelArg);
}

std::string EncodeText(std::string text) {
    const std::string_view strView{text};
    std::ostringstream output;
    webifc::parsing::p21encode(strView,output);
    return output.str();
}

std::string DecodeText(std::string text) {
    std::string_view strView{text};
    return webifc::parsing::p21decode(strView);
}

EMSCRIPTEN_BINDINGS(my_module) {

    emscripten::class_<webifc::geometry::IfcGeometry>("IfcGeometry")
        .constructor<>()
        .function("GetVertexData", &webifc::geometry::IfcGeometry::GetVertexData)
        .function("GetVertexDataSize", &webifc::geometry::IfcGeometry::GetVertexDataSize)
        .function("GetIndexData", &webifc::geometry::IfcGeometry::GetIndexData)
        .function("GetIndexDataSize", &webifc::geometry::IfcGeometry::GetIndexDataSize)
        ;


    emscripten::value_object<glm::dvec4>("dvec4")
        .field("x", &glm::dvec4::x)
        .field("y", &glm::dvec4::y)
        .field("z", &glm::dvec4::z)
        .field("w", &glm::dvec4::w)
        ;

    emscripten::value_object<webifc::manager::LoaderSettings>("LoaderSettings")
        .field("COORDINATE_TO_ORIGIN", &webifc::manager::LoaderSettings::COORDINATE_TO_ORIGIN)
        .field("CIRCLE_SEGMENTS", &webifc::manager::LoaderSettings::CIRCLE_SEGMENTS)
        .field("TAPE_SIZE", &webifc::manager::LoaderSettings::TAPE_SIZE)
        .field("MEMORY_LIMIT", &webifc::manager::LoaderSettings::MEMORY_LIMIT)
        .field("LINEWRITER_BUFFER",&webifc::manager::LoaderSettings::LINEWRITER_BUFFER)
    ;

    emscripten::value_array<std::array<double, 16>>("array_double_16")
            .element(emscripten::index<0>())
            .element(emscripten::index<1>())
            .element(emscripten::index<2>())
            .element(emscripten::index<3>())
            .element(emscripten::index<4>())
            .element(emscripten::index<5>())
            .element(emscripten::index<6>())
            .element(emscripten::index<7>())
            .element(emscripten::index<8>())
            .element(emscripten::index<9>())
            .element(emscripten::index<10>())
            .element(emscripten::index<11>())
            .element(emscripten::index<12>())
            .element(emscripten::index<13>())
            .element(emscripten::index<14>())
            .element(emscripten::index<15>())
            ;

    emscripten::value_object<webifc::geometry::IfcPlacedGeometry>("IfcPlacedGeometry")
        .field("color", &webifc::geometry::IfcPlacedGeometry::color)
        .field("flatTransformation", &webifc::geometry::IfcPlacedGeometry::flatTransformation)
        .field("geometryExpressID", &webifc::geometry::IfcPlacedGeometry::geometryExpressID)
        ;

    emscripten::register_vector<std::string>("stringVector");

    emscripten::register_vector<webifc::geometry::IfcPlacedGeometry>("IfcPlacedGeometryVector");

    emscripten::value_object<webifc::geometry::IfcFlatMesh>("IfcFlatMesh")
        .field("geometries", &webifc::geometry::IfcFlatMesh::geometries)
        .field("expressID", &webifc::geometry::IfcFlatMesh::expressID)
        ;

    emscripten::register_vector<webifc::geometry::IfcFlatMesh>("IfcFlatMeshVector");
    emscripten::register_vector<uint32_t>("UintVector");

    emscripten::register_vector<webifc::geometry::IfcCrossSections>("IfcCrossSectionsVector");

    emscripten::value_object<webifc::geometry::IfcCrossSections>("IfcCrossSections")
        .field("curves", &webifc::geometry::IfcCrossSections::curves)
        .field("expressID", &webifc::geometry::IfcCrossSections::expressID);

    emscripten::register_vector<webifc::geometry::IfcAlignment>("IfcAlignmentVector");

    emscripten::value_object<webifc::geometry::IfcAlignment>("IfcAlignment")
        .field("Horizontal", &webifc::geometry::IfcAlignment::Horizontal)
        .field("Vertical", &webifc::geometry::IfcAlignment::Vertical);

    emscripten::value_object<glm::dvec3>("glmDvec3")
        .field("x", &glm::dvec3::x)
        .field("y", &glm::dvec3::y)
        .field("z", &glm::dvec3::z);

    emscripten::value_object<webifc::geometry::IfcAlignmentSegment>("IfcAlignmentSegment")
        .field("curves", &webifc::geometry::IfcAlignmentSegment::curves);

    emscripten::register_vector<webifc::geometry::IfcCurve>("IfcCurveVector");

    emscripten::value_object<webifc::geometry::IfcCurve>("IfcCurve")
        .field("points", &webifc::geometry::IfcCurve::points)
        .field("userData", &webifc::geometry::IfcCurve::userData);

    emscripten::register_vector<glm::vec<2, glm::f64>>("vector2doubleVector");

    emscripten::value_object<glm::vec<2, glm::f64>>("vector2double")
        .field("x", &glm::vec<2, glm::f64>::x)
        .field("y", &glm::vec<2, glm::f64>::y);

    emscripten::register_vector<glm::vec<3, double>>("vector3double");

    emscripten::register_vector<double>("DoubleVector");

    emscripten::function("ExtendLineStorage", &ExtendLineStorage);
    emscripten::function("LoadAllGeometry", &LoadAllGeometry);
    emscripten::function("GetAllCrossSections", &GetAllCrossSections);
    emscripten::function("GetAllAlignments", &GetAllAlignments);
    emscripten::function("OpenModel", &OpenModel);
    emscripten::function("CreateModel", &CreateModel);
    emscripten::function("GetMaxExpressID", &GetMaxExpressID);
    emscripten::function("CloseModel", &CloseModel);
    emscripten::function("GetModelSize", &GetModelSize);
    emscripten::function("IsModelOpen", &IsModelOpen);
    emscripten::function("GetGeometry", &GetGeometry);
    emscripten::function("GetFlatMesh", &GetFlatMesh);
    emscripten::function("GetCoordinationMatrix", &GetCoordinationMatrix);
    emscripten::function("StreamMeshes", &StreamMeshesWithExpressID);
    emscripten::function("StreamAllMeshes", &StreamAllMeshes);
    emscripten::function("StreamAllMeshesWithTypes", &StreamAllMeshesWithTypesVal);
    emscripten::function("GetLine", &GetLine);
    emscripten::function("GetLineType", &GetLineType);
    emscripten::function("GetHeaderLine", &GetHeaderLine);
    emscripten::function("WriteLine", &WriteLine);
    emscripten::function("RemoveLine", &RemoveLine);
    emscripten::function("WriteHeaderLine", &WriteHeaderLine);
    emscripten::function("SaveModel", &SaveModel);
    emscripten::function("ValidateExpressID", &ValidateExpressID);
    emscripten::function("GetNextExpressID", &GetNextExpressID);
    emscripten::function("GetLineIDsWithType", &GetLineIDsWithType);
    emscripten::function("GetInversePropertyForItem", &GetInversePropertyForItem);
    emscripten::function("GetAllLines", &GetAllLines);
    emscripten::function("SetGeometryTransformation", &SetGeometryTransformation);
    emscripten::function("SetLogLevel", &SetLogLevel);
    emscripten::function("GetNameFromTypeCode", &GetNameFromTypeCode);
    emscripten::function("GetTypeCodeFromName", &GetTypeCodeFromName);
    emscripten::function("IsIfcElement", &IsIfcElement);
    emscripten::function("GetVersion", &GetVersion);
    emscripten::function("CloseAllModels", &CloseAllModels);
    emscripten::function("DecodeText", &DecodeText);
    emscripten::function("EncodeText", &EncodeText);
}
