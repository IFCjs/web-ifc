import * as WebIFC from "../../../dist";
import { Equals, WithIFCFileLoaded, TestInfo } from "./utils";

export default async function() {
    await WithIFCFileLoaded("modify_single_line", (ifcapi: WebIFC.IfcAPI, modelID: number, info: TestInfo) => {

        let newGuidValue = "<MODIFIED_GUID>";

        // lets modify a line
        {
            // this returns a propertyset object at line number 244
            let propertySet = ifcapi.GetLine(modelID, 244) as WebIFC.IfcPropertySet;

            // check the guid before modification
            Equals("GUID before", propertySet.GlobalId.value, "0uNK5AgoP1Vw6UlaHiS$iF");

            propertySet.GlobalId.value = newGuidValue;

            ifcapi.WriteLine(modelID, propertySet);
        }

        // and read it back modified
        {
            let propertySet = ifcapi.GetLine(modelID, 244) as WebIFC.IfcPropertySet;
            
            // check guid after modification
            Equals("GUID after", propertySet.GlobalId.value, newGuidValue);
        }

    });

    await WithIFCFileLoaded("modify_nested_line", (ifcapi: WebIFC.IfcAPI, modelID: number, info: TestInfo) => {

        let modifiedNameValue = "<MODIFIED_NAME>";

        // lets modify a line
        {
            let propertySetFlat = ifcapi.GetLine(modelID, 244, true) as WebIFC.IfcPropertySet;

            // check the guid before modification
            let prop = propertySetFlat.HasProperties[0] as WebIFC.IfcPropertySingleValue;
            Equals("Name before", prop.Name.value, "Reference");

            prop.Name.value = modifiedNameValue;

            ifcapi.WriteLine(modelID, prop);
        }

        let propID = -1;
        // read the root back modified
        {
            let propertySetFlat = ifcapi.GetLine(modelID, 244, true) as WebIFC.IfcPropertySet;
            let prop = propertySetFlat.HasProperties[0] as WebIFC.IfcPropertySingleValue;
            propID = prop.expressID;

            // check guid after modification
            Equals("Name after flat", prop.Name.value, modifiedNameValue);
        }

        // and the subelement
        {
            let prop = ifcapi.GetLine(modelID, propID) as WebIFC.IfcPropertySingleValue;
            
            // check guid after modification
            Equals("Name after sub", prop.Name.value, modifiedNameValue);
        }

    });
}