import * as WebIFC from "../../dist";

let EID = 1;

function real(v: number): any
{
    return { type: 4, value: v}
}

function ref(v: number): any
{
    return { type: 5, value: v}
}

function empty(): any
{
    return { type: 6}
}

function str(v: string): any
{
    return { type: 1, value: v}
}

function enm(v: string): any
{
    return { type: 3, value: v}
}

interface pt
{
    x: number, y: number, z: number;
}

interface pt2D
{
    x: number, y: number;
}

function Point(model: number, api: WebIFC.IfcAPI, o: pt): any
{
    let ID = EID++;
    let pt = new WebIFC.IfcCartesianPoint(ID, 
        WebIFC.IFCCARTESIANPOINT, 
                    [real(o.x), real(o.y), real(o.z)]);
    api.WriteLine(model, pt);
    return ref(ID);
}

function Dir(model: number, api: WebIFC.IfcAPI, o: pt): any
{
    let ID = EID++;
    let pt = new WebIFC.IfcDirection(ID, 
        WebIFC.IFCDIRECTION, 
                    [real(o.x), real(o.y), real(o.z)]);
    api.WriteLine(model, pt);
    return ref(ID);
}

function Point2D(model: number, api: WebIFC.IfcAPI, o: pt2D): any
{
    let ID = EID++;
    let pt = new WebIFC.IfcCartesianPoint(ID, 
        WebIFC.IFCCARTESIANPOINT, 
                    [real(o.x), real(o.y)]);
    api.WriteLine(model, pt);
    return ref(ID);
}

function AxisPlacement(model: number, api: WebIFC.IfcAPI, o: pt): any
{
    let locationID = Point(model, api, o);
    let ID = EID++;
    let pt = new WebIFC.IfcAxis2Placement3D(ID, 
        WebIFC.IFCAXIS2PLACEMENT3D, 
                    locationID, 
                    empty(),
                    empty());
    api.WriteLine(model, pt);
    return ref(ID);
}

function AxisPlacement2D(model: number, api: WebIFC.IfcAPI, o: pt2D): any
{
    let locationID = Point2D(model, api, o);
    let ID = EID++;
    let pt = new WebIFC.IfcAxis2Placement2D(ID, 
        WebIFC.IFCAXIS2PLACEMENT2D,
                    locationID, 
                    empty());
    api.WriteLine(model, pt);
    return ref(ID);
}

function Placement(model: number, api: WebIFC.IfcAPI, o: pt)
{
    let axisID = AxisPlacement(model, api, o);
    let ID = EID++;
    let pt = new WebIFC.IfcLocalPlacement(ID, 
        WebIFC.IFCLOCALPLACEMENT,
                    empty(),
                    axisID);
    api.WriteLine(model, pt);
    return ref(ID);
}

function CircleProfile(model: number, api: WebIFC.IfcAPI, rad: number, o: pt2D)
{
    let ID = EID++;
    let pt = new WebIFC.IfcCircleProfileDef(ID,
        WebIFC.IFCCIRCLEPROFILEDEF,
                    enm(WebIFC.IfcProfileTypeEnum.AREA),
                    str('column-prefab'),
                    AxisPlacement2D(model, api, o),
                    real(rad));
    api.WriteLine(model, pt);
    return ref(ID);
}

function ExtrudedAreaSolid(model: number, api: WebIFC.IfcAPI, pos: pt, dir: pt, rad: number, len: number)
{
    let ID = EID++;
    let pt = new WebIFC.IfcExtrudedAreaSolid(ID, 
        WebIFC.IFCEXTRUDEDAREASOLID,
                    CircleProfile(model, api, rad, { x: 0, y: 0 }),
                    AxisPlacement(model, api, pos),
                    Dir(model, api, dir),
                    real(len));
    api.WriteLine(model, pt);
    return ref(ID);
}

function StandardColumn(model: number, api: WebIFC.IfcAPI, pos: pt)
{
    let shapeID = ExtrudedAreaSolid(model, api, 
        { x: -2, y: 0, z: -1 }, 
        { x: 0, y: 0, z: 1 },
        0.25,
        2);

    let ID = EID++;
    let pt = new WebIFC.IfcColumn(ID, 
        WebIFC.IFCCOLUMN,
                    str("GUID"),
                    empty(),
                    str("name"),
                    empty(),
                    str("label"),
                    Placement(model, api, pos),
                    shapeID,
                    str("sadf"),
                    empty());
    api.WriteLine(model, pt);
    return ref(ID);
}

function BuildModel(model: number, api: WebIFC.IfcAPI)
{
    console.log("Building model " + model);

    const gridSize = 6;

    for (let i = 0; i < gridSize; i++)
    {
        for (let j = 0; j < gridSize; j++)
        {
            StandardColumn(model, api, {x: i, y: j, z: 0});
        }
    }
}