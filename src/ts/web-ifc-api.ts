/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/**
 * Web-IFC Main API Class
 * @module web-ifc
 */

import {
    Handle,
    IfcLineObject,
    TypeInitialisers,
    FILE_SCHEMA,
    FILE_NAME, 
    FILE_DESCRIPTION,
    FromRawLineData,
    Constructors,
    InheritanceDef,
    InversePropertyDef,
    ToRawLineData,
    SchemaNames
} from "./ifc-schema";

declare var __WASM_PATH__:string;

let WebIFCWasm: any;

if (typeof self !== 'undefined' && self.crossOriginIsolated) {
    try {
        WebIFCWasm = require("./web-ifc-mt");
    } catch (ex){
        WebIFCWasm = require(__WASM_PATH__);
    }
} else WebIFCWasm = require(__WASM_PATH__);

export * from "./ifc-schema";
import { Properties } from "./helpers/properties";
export { Properties };
import { Log, LogLevel } from "./helpers/log";
export { LogLevel };

export const UNKNOWN = 0;
export const STRING = 1;
export const LABEL = 2;
export const ENUM = 3;
export const REAL = 4;
export const REF = 5;
export const EMPTY = 6;
export const SET_BEGIN = 7;
export const SET_END = 8;
export const LINE_END = 9;
export const INTEGER = 10;

/**
 * Settings for the IFCLoader
 * @property {boolean} COORDINATE_TO_ORIGIN - If true, the model will be translated to the origin.
 * @property {number} CIRCLE_SEGMENTS - Number of segments for circles. 
 * @property {number} MEMORY_LIMIT - The amount of memory to be reserved for storing IFC data in memory
 * @property {number} TAPE_SIZE - Size of the tape for the loader.
 * @property {number} LINEWRITER_BUFFER - The number of lines to write to memory at a time when writing an IFC file.
 */
export interface LoaderSettings {
    COORDINATE_TO_ORIGIN?: boolean;
    CIRCLE_SEGMENTS?: number;
    MEMORY_LIMIT?: number;
    TAPE_SIZE? : number;
    LINEWRITER_BUFFER?: number;
}

export interface Vector<T> extends Iterable<T> {
    get(index: number): T;
    size(): number;
}

export interface Color {
    x: number;
    y: number;
    z: number;
    w: number;
}

export interface RawLineData {
    ID: number;
    type: number;
    arguments: any[];
}

export interface PlacedGeometry {
    color: Color;
    geometryExpressID: number;
    flatTransformation: Array<number>;
}

export interface FlatMesh {
    geometries: Vector<PlacedGeometry>;
    expressID: number;
    delete(): void;
}

export interface Point {
    x: number;
    y: number;
    z?: number;
}

export interface Curve {
    points: Array<Point>;
}

export interface CrossSection {
    curves: Array<Curve>;
    expressID: Array<number>;
}

export interface AlignmentSegment {
    curves: Array<Curve>;
}

export interface Alignment {
    FlatCoordinationMatrix: Array<number>;
    Horizontal: AlignmentSegment;
    Vertical: AlignmentSegment;
}

export interface IfcGeometry {
    GetVertexData(): number;
    GetVertexDataSize(): number;
    GetIndexData(): number;
    GetIndexDataSize(): number;
    delete(): void;
}

export interface IfcType {
    typeID: number;
    typeName: string;
}

export interface NewIfcModel {
    schema: string;
    name?: string;
    description?: string[];
    authors?: string[];
    organizations?: string[];
    authorization?: string;
}

export type ModelLoadCallback = (offset:number, size: number) => Uint8Array;
export type ModelSaveCallback = (data:Uint8Array) => void;

/** @ignore */
export function ms() {
    return new Date().getTime();
}

export type LocateFileHandlerFn = (path: string, prefix: string) => string;

export class IfcAPI {
    /** @ignore */
    wasmModule: undefined | any = undefined;
    private wasmPath: string = "";
    private isWasmPathAbsolute = false;

    private modelSchemaList: Array<number> = [];
    private modelSchemaNameList: Array<string> = [];

    /** @ignore */
    ifcGuidMap: Map<number, Map<string | number, string | number>> = new Map<number, Map<string | number, string | number>>();

    private deletedLines: Map<number,Set<number>> = new Map<number,Set<number>>();

    /**
     * Contains all the logic and methods regarding properties, psets, qsets, etc.
     */
    properties = new Properties(this);

    /**
     * Initializes the WASM module (WebIFCWasm), required before using any other functionality.
     *
     * @param customLocateFileHandler An optional locateFile function that let's
     * you override the path from which the wasm module is loaded.
     */
    async Init(customLocateFileHandler?: LocateFileHandlerFn) {
        if (WebIFCWasm && this.wasmModule == undefined) {
            let locateFileHandler: LocateFileHandlerFn = (path, prefix) => {
                // when the wasm module requests the wasm file, we redirect to include the user specified path
                if (path.endsWith(".wasm")) {
                    if (this.isWasmPathAbsolute) {
                        return this.wasmPath + path;
                    }

                    return prefix + this.wasmPath + path;
                }
                // otherwise use the default path
                return prefix + path;
            }

            //@ts-ignore
            this.wasmModule = await WebIFCWasm({ noInitialRun: true, locateFile: customLocateFileHandler || locateFileHandler });
            this.SetLogLevel(LogLevel.LOG_LEVEL_ERROR);
        }
        else {
			Log.error(`Could not find wasm module at './web-ifc' from web-ifc-api.ts`);
        }
    }

     /**
     * Opens a set of models and returns model IDs 
     * @param dataSets Array of Buffers containing IFC data (bytes)
     * @param settings Settings for loading the model @see LoaderSettings
	 * @returns Array of model IDs
    */
    OpenModels(dataSets: Array<Uint8Array>, settings?: LoaderSettings): Array<number> {
        let s: LoaderSettings = {
            MEMORY_LIMIT :  2147483648,
            ...settings
        };
        s.MEMORY_LIMIT = s.MEMORY_LIMIT! / dataSets.length;
        let modelIDs:Array<number> = [];

        for (let dataSet of dataSets) modelIDs.push(this.OpenModel(dataSet,s));
        return modelIDs
    }


    private CreateSettings(settings?: LoaderSettings) {
        let s: LoaderSettings = {
            COORDINATE_TO_ORIGIN: false,
            CIRCLE_SEGMENTS: 12,
            TAPE_SIZE: 67108864,
            MEMORY_LIMIT: 2147483648,
            LINEWRITER_BUFFER: 10000,
            ...settings
        };
        return s;
    }

    private LookupSchemaId(schemaName: string) {
        for (var i=0; i < SchemaNames.length;i++)
        {
            if (typeof SchemaNames[i] !== "undefined") {
                for (var j=0; j < SchemaNames[i].length;j++) 
                {
                    if (SchemaNames[i][j] == schemaName) return i;
                }
            }
        }
        return -1;
    }

    /**
     * Opens a model and returns a modelID number
     * @param data Buffer containing IFC data (bytes)
     * @param settings Settings for loading the model @see LoaderSettings
	 * @returns ModelID or -1 if model fails to open
    */
    OpenModel(data: Uint8Array, settings?: LoaderSettings): number {
        let s = this.CreateSettings(settings);
        let result = this.wasmModule.OpenModel(s, (destPtr: number, offsetInSrc: number, destSize: number) => {
            let srcSize = Math.min(data.byteLength - offsetInSrc, destSize);
            let dest = this.wasmModule.HEAPU8.subarray(destPtr, destPtr + srcSize);
            let src = data.subarray(offsetInSrc, offsetInSrc + srcSize );
            dest.set(src);
            return srcSize;
        });
        this.deletedLines.set(result,new Set());
        var schemaName = this.GetHeaderLine(result, FILE_SCHEMA).arguments[0][0].value;
        this.modelSchemaList[result] = this.LookupSchemaId(schemaName);
        this.modelSchemaNameList[result] = schemaName;
        if (this.modelSchemaList[result] == -1) 
        {
            Log.error("Unsupported Schema:"+schemaName);
            this.CloseModel(result)
            return -1;
        } 
        Log.debug("Parsing Model using " + schemaName + " Schema");
        return result;
    }

    /**
     * Opens a model and returns a modelID number
     * @param callback a function of signature (offset:number, size: number) => Uint8Array that will retrieve the IFC data
     * @param settings Settings for loading the model @see LoaderSettings
     * @returns ModelID or -1 if model fails to open
    */
    OpenModelFromCallback(callback:  ModelLoadCallback , settings?: LoaderSettings): number {
        let s = this.CreateSettings(settings);
        let result = this.wasmModule.OpenModel(s, (destPtr: number, offsetInSrc: number, destSize: number) => {
            let data = callback(offsetInSrc,destSize);
            let srcSize = Math.min(data.byteLength, destSize);
            let dest = this.wasmModule.HEAPU8.subarray(destPtr, destPtr + srcSize);
            dest.set(data);
            return srcSize;
        });
        this.deletedLines.set(result,new Set());
        var schemaName = this.GetHeaderLine(result, FILE_SCHEMA).arguments[0][0].value;
        this.modelSchemaList[result] = this.LookupSchemaId(schemaName);
        this.modelSchemaNameList[result] = schemaName;
        if (this.modelSchemaList[result] == -1) 
        {
            Log.error("Unsupported Schema:"+schemaName);
            this.CloseModel(result)
            return -1;
        } 
        Log.debug("Parsing Model using " + schemaName + " Schema");
        return result;
    }

    /**
     * Fetches the ifc schema version of a given model
     * @param modelID Model ID
	 * @returns IFC Schema version
    */
    GetModelSchema(modelID: number) {
        return this.modelSchemaNameList[modelID];
    }

    /**
     * Creates a new model and returns a modelID number
     * @param schema ifc schema version
	 * @returns ModelID
    */
    CreateModel(model: NewIfcModel, settings?: LoaderSettings): number {
        let s = this.CreateSettings(settings);
        let result = this.wasmModule.CreateModel(s);
        this.modelSchemaList[result] = this.LookupSchemaId(model.schema);
        this.modelSchemaNameList[result] = model.schema;
        if (this.modelSchemaList[result] == -1) 
        {
            Log.error("Unsupported Schema:"+model.schema);
            this.CloseModel(result)
            return -1;
        } 
        this.deletedLines.set(result,new Set());
        const modelName = model.name || "web-ifc-model-"+result+".ifc";
        const timestamp = new Date().toISOString().slice(0,19);
        const description = model.description?.map((d) => ({type: STRING, value: d})) || [{type: STRING, value: 'ViewDefinition [CoordinationView]'}];
        const authors = model.authors?.map((a) => ({type: STRING, value: a})) || [null];
        const orgs = model.organizations?.map((o) => ({type: STRING, value: o})) || [null];
        const auth = model.authorization ? {type: STRING, value: model.authorization} : null;
        
        this.wasmModule.WriteHeaderLine(result,FILE_DESCRIPTION,[
            description, 
            {type: STRING, value: '2;1'}
        ]);
        this.wasmModule.WriteHeaderLine(result,FILE_NAME,[
            {type: STRING, value: modelName},
            {type: STRING, value: timestamp},
            authors,
            orgs,
            {type: STRING, value: "thatopen/web-ifc-api"},
            {type: STRING, value: "thatopen/web-ifc-api"},
            auth,
        ]);
        this.wasmModule.WriteHeaderLine(result,FILE_SCHEMA,[[{type: STRING, value: model.schema}]]);

        return result;
    }

	/**
	 * Saves a model to a Buffer
	 * @param modelID Model ID
	 * @returns Buffer containing the model data
	 */
    SaveModel(modelID: number): Uint8Array {
        let dataBuffer:Uint8Array = new Uint8Array(0);
        this.wasmModule.SaveModel(modelID, (srcPtr: number, srcSize: number) => {
            let origSize: number = dataBuffer.byteLength;
            let src = this.wasmModule.HEAPU8.subarray(srcPtr, srcPtr + srcSize);
            let newBuffer = new Uint8Array(origSize+srcSize);
            newBuffer.set(dataBuffer)
            newBuffer.set(src, origSize);
            dataBuffer = newBuffer;
        });
       return dataBuffer;
    }

    /**
     * Saves a model to a Buffer
     * @param modelID Model ID
     * @returns Buffer containing the model data
     */
    SaveModelToCallback(modelID: number, callback: ModelSaveCallback) {
        this.wasmModule.SaveModel(modelID, (srcPtr: number, srcSize: number) => {
            let src = this.wasmModule.HEAPU8.subarray(srcPtr, srcPtr + srcSize);
            let newBuffer = new Uint8Array(srcSize);
            newBuffer.set(src);
            callback(newBuffer);
        });
    }

    /**
    * Retrieves the geometry of an element
    * @param modelID Model handle retrieved by OpenModel
    * @param geometryExpressID express ID of the element
    * @returns Geometry of the element as a list of vertices and indices
    */
    GetGeometry(modelID: number, geometryExpressID: number): IfcGeometry {
        return this.wasmModule.GetGeometry(modelID, geometryExpressID);
    }

    /**
     * Gets the header information required by the user
     * @param modelID Model handle retrieved by OpenModel
     * @param headerType Type of header data you want to retrieve
     * ifc.FILE_NAME, ifc.FILE_DESCRIPTION or ifc.FILE_SCHEMA
     * @returns An object with parameters ID, type and arguments
     */
    GetHeaderLine(modelID: number, headerType: number) {
        return this.wasmModule.GetHeaderLine(modelID, headerType);
    }

    /**
     * Gets the list of all ifcTypes contained in the model
     * @param modelID Model handle retrieved by OpenModel
     * @returns Array of objects containing typeID and typeName
     */
    GetAllTypesOfModel(modelID: number): IfcType[] {
        let typesNames: IfcType[] = [];
        const elements = Object.keys(FromRawLineData[this.modelSchemaList[modelID]]).map((e) => parseInt(e));
        for (let i = 0; i < elements.length; i++) {
            const lines = this.GetLineIDsWithType(modelID, elements[i]);
            if (lines.size() > 0) typesNames.push({ typeID: elements[i], typeName: this.wasmModule.GetNameFromTypeCode(elements[i]) });
        }
        return typesNames;
    }

	/**
	 * Gets the ifc line data for a given express ID
	 * @param modelID Model handle retrieved by OpenModel
	 * @param expressID express ID of the line
	 * @param flatten recursively flatten the line, default false
	 * @param inverse get the inverse properties of the line, default false
	 * @param inversePropKey filters out all other properties from a inverse search, for a increase in performance. Default null
	 * @returns lineObject
	 */
    GetLine(modelID: number, expressID: number, flatten = false, inverse = false, inversePropKey: string | null | undefined = null) {
        let expressCheck = this.wasmModule.ValidateExpressID(modelID, expressID);
        if (!expressCheck) {
            return;
        }

        let rawLineData = this.GetRawLineData(modelID, expressID);
        let lineData;
        try {
            lineData = FromRawLineData[this.modelSchemaList[modelID]][rawLineData.type](rawLineData.arguments);
            lineData.expressID = rawLineData.ID;
        } catch (e) {
           Log.error("Invalid IFC Line:"+expressID);
	         // throw an error when the line is defined 
           if (rawLineData.ID) {
               throw e;
           } else {
               return;
           }
       }

        if (flatten) {
            this.FlattenLine(modelID, lineData);
        }

        let inverseData = InversePropertyDef[this.modelSchemaList[modelID]][rawLineData.type];
        if (inverse && inverseData != null) 
        {
          for (let inverseProp of inverseData) 
          {
            if (inversePropKey && inverseProp[0] !== inversePropKey) continue;
			  
            if (!inverseProp[3]) lineData[inverseProp[0]] = null;
            else lineData[inverseProp[0]] = [];
            
            let targetTypes = [inverseProp[1]];
            if (typeof InheritanceDef[this.modelSchemaList[modelID]][inverseProp[1]] != "undefined")
            {
              targetTypes=targetTypes.concat(InheritanceDef[this.modelSchemaList[modelID]][inverseProp[1]]);
            }
            let inverseIDs = this.wasmModule.GetInversePropertyForItem(modelID, expressID, targetTypes, inverseProp[2], inverseProp[3]);
            if (!inverseProp[3] && inverseIDs.size()>0) 
            {
              if (!flatten) lineData[inverseProp[0]] = { type: 5,  value: inverseIDs.get(0) };
              else lineData[inverseProp[0]] = this.GetLine(modelID, inverseIDs.get(0));
            }
            else 
            {
                for (let x = 0; x < inverseIDs.size(); x++) {
                  if (!flatten) lineData[inverseProp[0]].push({ type: 5,  value: inverseIDs.get(x) });
                  else lineData[inverseProp[0]].push(this.GetLine(modelID, inverseIDs.get(x)));
                }
            }
          }
        }
        return lineData;
    }

    /**
     * Gets the next unused expressID
     * @param modelID Model handle retrieved by OpenModel
     * @param expressID Starting expressID value 
     * @returns The next unused expressID starting from the value provided
     */
    GetNextExpressID(modelID: number, expressID: number): number {
        return this.wasmModule.GetNextExpressID(modelID, expressID);
    }

    /**
     * Creates a new ifc entity
     * @param modelID Model handle retrieved by OpenModel
     * @param type Type code
     * @param args Arguments required by the entity
     * @returns An object contining the parameters of the new entity
     */
    CreateIfcEntity(modelID: number, type:number, ...args: any[] ): IfcLineObject
    {
        return Constructors[this.modelSchemaList[modelID]][type](args);
    }

    /**
     * Creates a new ifc type i.e. IfcLabel, IfcReal, ...
     * @param modelID Model handle retrieved by OpenModel
     * @param type Type code
     * @param value Type value
     * @returns An object with the parameters of the type
     */
    CreateIfcType(modelID: number, type:number, value: any) 
    {
        return TypeInitialisers[this.modelSchemaList[modelID]][type](value);
    }

    /**
     * Gets the name from a type code
     * @param type Code
     * @returns Name
     */
    GetNameFromTypeCode(type:number): string 
    {
       Log.warn("GetNameFromTypeCode() now returns type names in camel case");
       return this.wasmModule.GetNameFromTypeCode(type);
    }

    /**
     * Gets the type code  from a name code
     * @param name
     * @returns type code
     */
    GetTypeCodeFromName(typeName:string): number 
    {
       return this.wasmModule.GetTypeCodeFromName(typeName);
    }

    /**
     * Evaluates if a type is subtype of IfcElement 
     * @param type Type code
     * @returns True if subtype of Ifcelement, False if it is not subtype
     */
    IsIfcElement(type:number) : boolean
    {
        return this.wasmModule.IsIfcElement(type);
    }

    /**
     * Returns a list with all entity types that are present in the current schema
     * @param modelID Model handle retrieved by OpenModel
     * @returns Array of type codes
     */
    GetIfcEntityList(modelID: number) : Array<number>
    {
        return Object.keys(FromRawLineData[this.modelSchemaList[modelID]]).map(x=>parseInt(x));
    }

    /**
     * Deletes an IFC line from the model
     * @param modelID Model handle retrieved by OpenModel
     * @param expressID express ID of the line to remove
     */
    DeleteLine(modelID: number, expressID: number) {
        this.wasmModule.RemoveLine(modelID,expressID);
        this.deletedLines.get(modelID)!.add(expressID);
    }

	/**
     * Writes a line to the model, can be used to write new lines or to update existing lines
     * @param modelID Model handle retrieved by OpenModel
     * @param lineObject array of line object to write
     */
    WriteLines<Type extends IfcLineObject>(modelID: number, lineObjects: Array<Type>) {
        this.wasmModule.ExtendLineStorage(modelID,lineObjects.length);
        for (let lineObject of lineObjects) this.WriteLine(modelID,lineObject);
    }

    /**
	 * Writes a set of line to the model, can be used to write new lines or to update existing lines
	 * @param modelID Model handle retrieved by OpenModel
	 * @param lineObject line object to write
	 */
    WriteLine<Type extends IfcLineObject>(modelID: number, lineObject: Type) {
        if (lineObject.expressID!= -1 && this.deletedLines.get(modelID)!.has(lineObject.expressID)) 
        {
            Log.error(`Cannot re-use deleted express ID`);
            return;
        }
        if (lineObject.expressID != -1 && lineObject.expressID <= this.GetMaxExpressID(modelID) && this.GetLineType(modelID,lineObject.expressID) != lineObject.type && this.GetLineType(modelID,lineObject.expressID) != 0) 
        {
            Log.error(`Cannot change type of existing IFC Line`);
            return;
        }

        let property: keyof Type;
        for (property in lineObject)
        {
            const lineProperty: any = lineObject[property];
            if (lineProperty  && (lineProperty as IfcLineObject).expressID !== undefined) {
                // this is a real object, we have to write it as well and convert to a handle
                // TODO: detect if the object needs to be written at all, or if it's unchanged
                this.WriteLine(modelID, lineProperty as IfcLineObject);

                // overwrite the reference
                // NOTE: this modifies the parameter
                (lineObject[property] as any)= new Handle((lineProperty as IfcLineObject).expressID);
            }
            else if (Array.isArray(lineProperty) && lineProperty.length > 0) {
                for (let i = 0; i < lineProperty.length; i++) {
                    if ((lineProperty[i] as IfcLineObject).expressID !== undefined) {
                        // this is a real object, we have to write it as well and convert to a handle
                        // TODO: detect if the object needs to be written at all, or if it's unchanged
                        this.WriteLine(modelID, lineProperty[i] as IfcLineObject);

                        // overwrite the reference
                        // NOTE: this modifies the parameter
                        ((lineObject[property]as any)[i] as any) = new Handle((lineProperty[i] as IfcLineObject).expressID);
                    }
                }
            }
        }
        
        if(lineObject.expressID === undefined || lineObject.expressID < 0) {
            lineObject.expressID = this.GetMaxExpressID(modelID)+1;
        }


        let rawLineData: RawLineData = {
            ID: lineObject.expressID,
            type: lineObject.type,
            arguments: ToRawLineData[this.modelSchemaList[modelID]][lineObject.type](lineObject) as any[]
        }
        this.WriteRawLineData(modelID, rawLineData);
    }

    /** @ignore */
    FlattenLine(modelID: number, line: any) {
        Object.keys(line).forEach(propertyName => {
            let property = line[propertyName];
            if (property && property.type === 5) {
                if (property.value) line[propertyName] = this.GetLine(modelID, property.value, true);
            }
            else if (Array.isArray(property) && property.length > 0 && property[0] && property[0].type === 5) {
                for (let i = 0; i < property.length; i++) {
                    if (property[i].value) line[propertyName][i] = this.GetLine(modelID, property[i].value, true);
                }
            }
        });
    }

    /** @ignore */
    GetRawLineData(modelID: number, expressID: number): RawLineData {
        return this.wasmModule.GetLine(modelID, expressID) as RawLineData;
    }

    /** @ignore */
    WriteRawLineData(modelID: number, data: RawLineData) {
        this.wasmModule.WriteLine(modelID, data.ID, data.type, data.arguments);
    }

    /** @ignore */
    WriteRawLinesData(modelID: number, data: Array<RawLineData>) {
        this.wasmModule.ExtendLineStorage(modelID,data.length);
        for (let rawLine of data)  this.wasmModule.WriteLine(modelID, rawLine.ID, rawLine.type, rawLine.arguments);
    }

	/**
	 * Get all line IDs of a specific ifc type
	 * @param modelID model ID
	 * @param type ifc type, @see IfcEntities
	 * @param includeInherited if true, also returns all inherited types 
	 * @returns vector of line IDs
	 */
    GetLineIDsWithType(modelID: number, type: number, includeInherited: boolean = false): Vector<number> {
        let types: Array<number> = [];
        types.push(type);
        if (includeInherited && typeof InheritanceDef[this.modelSchemaList[modelID]][type] != "undefined")
        {
          types = types.concat(InheritanceDef[this.modelSchemaList[modelID]][type]);
        } 
        let lineIds = this.wasmModule.GetLineIDsWithType(modelID, types);
        lineIds[Symbol.iterator] = function*() { for (let i=0; i < lineIds.size();i++) yield lineIds.get(i); }
        return lineIds;
    }

	/**
	 * Get all line IDs of a model
	 * @param modelID model ID
	 * @returns vector of all line IDs
	 */
    GetAllLines(modelID: number): Vector<number> {
        let lineIds = this.wasmModule.GetAllLines(modelID);
        lineIds[Symbol.iterator] = function*() { for (let i=0; i < lineIds.size();i++) yield lineIds.get(i); }
        return lineIds;
    }

    /**
     * Returns all crossSections in 2D contained in IFCSECTIONEDSOLID, IFCSECTIONEDSURFACE, IFCSECTIONEDSOLIDHORIZONTAL (IFC4x3 or superior) 
     * @param modelID model ID
     * @returns Lists with the cross sections curves as sets of points
     */
    GetAllCrossSections2D(modelID: number): Array<CrossSection>
    {
        const crossSections =  this.wasmModule.GetAllCrossSections(modelID,2);
        const crossSectionList = [];
        for (let i = 0; i < crossSections.size(); i++) {
            const alignment = crossSections.get(i);
            const curveList = [];
            const expressList = [];
            for (let j = 0; j < alignment.curves.size(); j++) {
                const curve = alignment.curves.get(j);
                const ptList: Array<Point> = [];
                for (let p = 0; p < curve.points.size(); p++) {
                const pt = curve.points.get(p);
                const newPoint: Point = { x: pt.x, y: pt.y, z: pt.z };
                ptList.push(newPoint);
                }
                const newCurve: Curve = { points: ptList };
                curveList.push(newCurve);
                expressList.push(alignment.expressID.get(j));
            }
            const align = { FlatCoordinationMatrix: this.GetCoordinationMatrix(modelID), curves: curveList, expressID: expressList };
            crossSectionList.push(align);
        }
        return crossSectionList;
    }

    /**
     * Returns all crossSections in 3D contained in IFCSECTIONEDSOLID, IFCSECTIONEDSURFACE, IFCSECTIONEDSOLIDHORIZONTAL (IFC4x3 or superior) 
     * @param modelID model ID
     * @returns Lists with the cross sections curves as sets of points
     */
    GetAllCrossSections3D(modelID: number): Array<CrossSection>
    {
        const crossSections =  this.wasmModule.GetAllCrossSections(modelID,3);
        const crossSectionList = [];
        for (let i = 0; i < crossSections.size(); i++) {
            const alignment = crossSections.get(i);
            const curveList: Array<Curve> = [];
            const expressList: Array<number> = [];
            for (let j = 0; j < alignment.curves.size(); j++) {
                const curve = alignment.curves.get(j);
                const ptList = [];
                for (let p = 0; p < curve.points.size(); p++) {
                const pt = curve.points.get(p);
                const newPoint = { x: pt.x, y: pt.y, z: pt.z };
                ptList.push(newPoint);
                }
                const newCurve: Curve = { points: ptList };
                curveList.push(newCurve);
                expressList.push(alignment.expressID.get(j));
            }
            const align = { FlatCoordinationMatrix: this.GetCoordinationMatrix(modelID), curves: curveList, expressID: expressList };
            crossSectionList.push(align);
        }
        return crossSectionList;
    }

    /**
     * Returns all alignments contained in the IFC model (IFC4x3 or superior)
     * @param modelID model ID
     * @returns Lists with horizontal and vertical curves as sets of points
     */
    GetAllAlignments(modelID: number): any {
        const alignments = this.wasmModule.GetAllAlignments(modelID);
        const alignmentList = [];
        if (alignments.size() > 0)
        {
            console.log("RAW");
            console.log(alignments);
            for (let i = 0; i < alignments.size(); i++) {
            const alignment = alignments.get(i);
            const horList = [];
            for (let j = 0; j < alignment.Horizontal.curves.size(); j++) {
                const curve = alignment.Horizontal.curves.get(j);
                const ptList: Array<Point> = [];
                for (let p = 0; p < curve.points.size(); p++) {
                const pt = curve.points.get(p);
                const newPoint = { x: pt.x, y: pt.y };
                ptList.push(newPoint);
                }
                const dtList = [];
                for (let p = 0; p < curve.userData.size(); p++) {
                    const dt = curve.userData.get(p);
                    dtList.push(dt);
                }
                const newCurve = { points: ptList, data: dtList };
                horList.push(newCurve);
            }
            const verList = [];
            for (let j = 0; j < alignment.Vertical.curves.size(); j++) {
                const curve = alignment.Vertical.curves.get(j);
                const ptList = [];
                for (let p = 0; p < curve.points.size(); p++) {
                const pt = curve.points.get(p);
                const newPoint = { x: pt.x, y: pt.y };
                ptList.push(newPoint);
                }
                const dtList = [];
                for (let p = 0; p < curve.userData.size(); p++) {
                    const dt = curve.userData.get(p);
                    dtList.push(dt);
                }
                const newCurve = { points: ptList, data: dtList };
                verList.push(newCurve);
            }
        
            const curve3DList = [];
            if (
                alignment.Horizontal.curves.size() > 0 &&
                alignment.Vertical.curves.size() > 0
            ) {
                const startH = { x: 0, y: 0, z: 0 };
                const startV = { x: 0, y: 0, z: 0 };
        
                // Construct 3D polyline from horizontal and vertical polylines
        
                let lastx = 0;
                let lasty = 0;
                let length = 0;
                for (let j = 0; j < alignment.Horizontal.curves.size(); j++) {
                const curve = alignment.Horizontal.curves.get(j);
                const points = [];
                for (let k = 0; k < curve.points.size(); k++) {
                    let alt = 0;
                    const pt = curve.points.get(k);
                    if (j === 0 && k === 0) {
                    lastx = pt.x;
                    lasty = pt.y;
                    }
                    const valueX = pt.x - lastx;
                    const valueY = pt.y - lasty;
                    lastx = pt.x;
                    lasty = pt.y;
                    length += Math.sqrt(valueX * valueX + valueY * valueY);
                    let first = true;
                    let lastAlt = 0;
                    let lastX = 0;
                    let done = false;
                    for (let ii = 0; ii < alignment.Vertical.curves.size(); ii++) {
                    const curve = alignment.Vertical.curves.get(ii);
                    for (let jj = 0; jj < curve.points.size(); jj++) {
                        const pt = curve.points.get(jj);
                        if (first) {
                        first = false;
                        alt = pt.y;
                        lastAlt = pt.y;
                        if (pt.x >= length) {
                            break;
                        }
                        }
                        if (pt.x >= length) {
                        const value1 = pt.x - lastX;
                        const value2 = length - lastX;
                        const value3 = value2 / value1;
                        alt = lastAlt * (1 - value3) + pt.y * value3;
                        done = true;
                        break;
                        }
                        lastAlt = pt.y;
                        lastX = pt.x;
                    }
                    if (done) {
                        break;
                    }
                    }
                    points.push({
                    x: pt.x - startH.x,
                    y: alt - startV.y,
                    z: startH.y - pt.y,
                    });
                }
                const newCurve = { points: points };
                curve3DList.push(newCurve);
                }
            }
        
            const align = {
                FlatCoordinationMatrix: this.GetCoordinationMatrix(modelID),
                horizontal: horList,
                vertical: verList,
                curve3D: curve3DList,
            };
            alignmentList.push(align);

            }
        }
        return alignmentList;
      }

	/**
	 * Set the transformation matrix
	 * @param modelID model ID
	 * @param transformationMatrix transformation matrix, flat 4x4 matrix as array[16] 
	 */
    SetGeometryTransformation(modelID: number, transformationMatrix: Array<number>) {
        if (transformationMatrix.length != 16) {
            throw new Error(`invalid matrix size: ${transformationMatrix.length}`);
        }
        this.wasmModule.SetGeometryTransformation(modelID, transformationMatrix);
    }

	/**
	 * Get the coordination matrix
	 * @param modelID model ID
	 * @returns flat 4x4 matrix as array[16]
	 */
    GetCoordinationMatrix(modelID: number): Array<number> {
        return this.wasmModule.GetCoordinationMatrix(modelID) as Array<number>;
    }

    GetVertexArray(ptr: number, size: number): Float32Array {
        return this.getSubArray(this.wasmModule.HEAPF32, ptr, size);
    }

    GetIndexArray(ptr: number, size: number): Uint32Array {
        return this.getSubArray(this.wasmModule.HEAPU32, ptr, size);
    }

    getSubArray(heap: any, startPtr: number, sizeBytes: number) {
        return heap.subarray(startPtr / 4, startPtr / 4 + sizeBytes).slice(0);
    }

    /**
     * Closes a model and frees all related memory
     * @param modelID Model handle retrieved by OpenModel, model must be closed after use
    */
    CloseModel(modelID: number) {
        this.ifcGuidMap.delete(modelID);
        this.wasmModule.CloseModel(modelID);
    }

    /**
     * Closes all models and frees all related memory. Please note that after calling this you must call Init() again to ensure web-ifc is in a working state.
    */
    Dispose() {
        this.ifcGuidMap.clear()
        this.wasmModule.CloseAllModels();
        this.wasmModule = undefined;
    }

    /**
	 * Streams meshes of a model with specific express id
	 * @param modelID Model handle retrieved by OpenModel
     * @param expressIDs expressIDs of elements to stream
	 * @param meshCallback callback function that is called for each mesh
	 */
    StreamMeshes(modelID: number, expressIDs: Array<number>, meshCallback: (mesh: FlatMesh, index:number, total:number) => void) {
        this.wasmModule.StreamMeshes(modelID, expressIDs,meshCallback);
    }

	/**
	 * Streams all meshes of a model
	 * @param modelID Model handle retrieved by OpenModel
	 * @param meshCallback callback function that is called for each mesh
	 */
    StreamAllMeshes(modelID: number, meshCallback: (mesh: FlatMesh, index:number, total:number) => void) {
        this.wasmModule.StreamAllMeshes(modelID, meshCallback);
    }

	/**
	 * Streams all meshes of a model with a specific ifc type
	 * @param modelID Model handle retrieved by OpenModel
	 * @param types types of elements to stream
	 * @param meshCallback callback function that is called for each mesh
	 */
    StreamAllMeshesWithTypes(modelID: number, types: Array<number>, meshCallback: (mesh: FlatMesh, index:number, total:number) => void) {
        this.wasmModule.StreamAllMeshesWithTypes(modelID, types, meshCallback);
    }

    /**
     * Checks if a specific model ID is open or closed
     * @param modelID Model handle retrieved by OpenModel
	 * @returns true if model is open, false if model is closed
    */
    IsModelOpen(modelID: number): boolean {
        return this.wasmModule.IsModelOpen(modelID);
    }

    /**
     * Load all geometry in a model
     * @param modelID Model handle retrieved by OpenModel
	 * @returns Vector of FlatMesh objects
    */
    LoadAllGeometry(modelID: number): Vector<FlatMesh> {
        let flatMeshes =  this.wasmModule.LoadAllGeometry(modelID);
        flatMeshes[Symbol.iterator] = function*() { for (let i=0; i < flatMeshes.size();i++) yield flatMeshes.get(i); }
        return flatMeshes;
    }

    /**
     * Load geometry for a single element
     * @param modelID Model handle retrieved by OpenModel
	 * @param expressID ExpressID of the element
	 * @returns FlatMesh object
    */
    GetFlatMesh(modelID: number, expressID: number): FlatMesh {
        return this.wasmModule.GetFlatMesh(modelID, expressID);
    }

    /**
    * Returns the maximum ExpressID value in the IFC file, ex.- #9999999
    * @param modelID Model handle retrieved by OpenModel
    * @returns Express numerical value
    */
    GetMaxExpressID(modelID: number) {
        return this.wasmModule.GetMaxExpressID(modelID) as number;
    }

    /**
    * Returns the type of a given ifc entity in the fiule.
    * @param modelID Model handle retrieved by OpenModel
    * @param expressID Line Number
    * @returns IFC Type Code
    */
    GetLineType(modelID: number, expressID: number) {
        return this.wasmModule.GetLineType(modelID, expressID);
    }

    /**
    * Returns the version number of web-ifc
    * @returns The current version number as a string
    */
    GetVersion() {
        return this.wasmModule.GetVersion();
    }

     /**
     * Looks up an entities express ID from its GlobalID.
     * @param modelID Model handle retrieved by OpenModel
     * @param guid GobalID to be looked up
     * @returns expressID numerical value
     */
    GetExpressIdFromGuid(modelID: number, guid: string)
    {
        if (!this.ifcGuidMap.has(modelID)) this.CreateIfcGuidToExpressIdMapping(modelID);
        return this.ifcGuidMap.get(modelID)?.get(guid);
    }

    /**
     * Looks up an entities GlobalID from its ExpressID.
     * @param modelID Model handle retrieved by OpenModel
     * @param expressID express ID to be looked up
     * @returns globalID string value
     */
    GetGuidFromExpressId(modelID: number, expressID: number)
    {
        if (!this.ifcGuidMap.has(modelID)) this.CreateIfcGuidToExpressIdMapping(modelID);
        return this.ifcGuidMap.get(modelID)?.get(expressID);
    }

    /** @ignore */
    CreateIfcGuidToExpressIdMapping(modelID: number): void {
        const map = new Map<string | number, string | number>();
        let entities = this.GetIfcEntityList(modelID);
        for (const typeId of entities) {
            if (!this.IsIfcElement(typeId)) continue;
            const lines = this.GetLineIDsWithType(modelID, typeId);
            const size = lines.size();
            for (let y = 0; y < size; y++) {
                const expressID = lines.get(y);
                const info = this.GetLine(modelID, expressID);
                try{
                    if ("GlobalId" in info) {
                        const globalID = info.GlobalId.value;
                        map.set(expressID, globalID);
                        map.set(globalID, expressID);
                    }
                } catch (e) {
                    continue;
                }
            }
        }
        this.ifcGuidMap.set(modelID, map);
    }

	/**
	 * Sets the path to the wasm file
	 * @param path path to the wasm file
	 * @param absolute if true, path is absolute, otherwise it is relative to executing script
	 */
    SetWasmPath(path: string, absolute = false) {
        this.wasmPath = path;
        this.isWasmPathAbsolute = absolute;
    }

	/**
	 * Sets the log level
	 * @param level Log level to set
	 */
    SetLogLevel(level: LogLevel): void {
        Log.setLogLevel(level);
        this.wasmModule.SetLogLevel(level);
    }
}
