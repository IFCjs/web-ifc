import { IfcApplication } from './../../src/ifc-schema';
import { IfcAPI, LogLevel,ms, Schemas, IFCUNITASSIGNMENT, IFCAXIS2PLACEMENT3D,IFCLENGTHMEASURE,IFCCARTESIANPOINT,IFCAXIS2PLACEMENT2D,IFCCIRCLEPROFILEDEF,IFCDIRECTION,IFCREAL,IFCPOSITIVELENGTHMEASURE,IFCCOLUMN,IFCEXTRUDEDAREASOLID,IFCGLOBALLYUNIQUEID,IFCLABEL,IFCIDENTIFIER } from '../../dist/web-ifc-api';
import { IfcThree } from './web-ifc-three';
import { Init3DView, InitBasicScene, ClearScene, scene } from './web-ifc-scene';
import * as Monaco from 'monaco-editor';
import * as ts_decl from "./ts_src";
import * as ts from "typescript";
import { exampleCode } from './example';

let ifcAPI = new IfcAPI();
ifcAPI.SetWasmPath("")
let ifcThree = new IfcThree(ifcAPI);

let timeout = undefined;

function Edited(monacoEditor: Monaco.editor.IStandaloneCodeEditor)
{
    let code = monacoEditor.getValue();

    window.localStorage.setItem('code', code);
    console.log("Saved code...");

}

if (typeof window != 'undefined')
{
//@ts-ignore
window.InitMonaco = (monaco: any) => {
    console.log(ts_decl.ifc_schema);
    // validation settings
    monaco.languages.typescript.typescriptDefaults.setDiagnosticsOptions({
        noSemanticValidation: true,
        noSyntaxValidation: true
    });
    
    // compiler options
    monaco.languages.typescript.typescriptDefaults.setCompilerOptions({
        target: monaco.languages.typescript.ScriptTarget.ES6,
        allowNonTsExtensions: true
    });
    //@ts-ignore
    console.log(monaco.languages.typescript.typescriptDefaults.addExtraLib(ts_decl.ifc_schema));
    console.log(monaco.languages.typescript.typescriptDefaults.addExtraLib(ts_decl.wifcapi));
}
}

function initMonacoEditor(monacoEditor: Monaco.editor.IStandaloneCodeEditor)
{
    let item = window.localStorage.getItem("code");
    if (item)
    {
        monacoEditor.setValue(item);
    }
    else
    {
        monacoEditor.setValue(exampleCode);
    }

    monacoEditor.onDidChangeModelContent((e) => {
        if (timeout)
        {
            clearTimeout(timeout);
        }
        timeout = setTimeout(() => Edited(monacoEditor), 1000);
    });

    setTimeout(() => {
        Edited(monacoEditor);
    }, 1000);
}

if (typeof window != 'undefined')
{
//@ts-ignore
window.InitWebIfcViewer = async (monacoEditor: Monaco.editor.IStandaloneCodeEditor) => {
  await ifcAPI.Init();
  initMonacoEditor(monacoEditor);
  const fileInput = document.getElementById('finput');
  fileInput.addEventListener('change', fileInputChanged);
  const codereset = document.getElementById('rcode');
  codereset.addEventListener('click', resetCode);
  const coderun = document.getElementById('runcode');
  coderun.addEventListener('click', runCode);
  const clearmem = document.getElementById('cmem');
  clearmem.addEventListener('click', clearMem);
  const changeLogLevelSelect = document.getElementById('logLevel');
  changeLogLevelSelect.addEventListener('change', changeLogLevel);
  Init3DView();
};
}

async function changeLogLevel() 
{
    let fileInput = <HTMLInputElement>document.getElementById('logLevel');
    ifcAPI.SetLogLevel(fileInput.value);
    console.log("Log Level Set to:"+fileInput.value);
}

async function runCode() {
  let model = ifcAPI.CreateModel({schema: Schemas.IFC4});

  scene.clear();
  InitBasicScene();

  let code = window.localStorage.getItem('code');
  let compiled = ts.transpileModule(code, { compilerOptions: { module: ts.ModuleKind.CommonJS }})

  // this is where we do evil stuff
  {
      console.log(` --- Starting EVAL!`);
      eval("(function (ifcAPI,IFCAXIS2PLACEMENT3D,IFCLENGTHMEASURE,IFCCARTESIANPOINT,IFCAXIS2PLACEMENT2D,IFCCIRCLEPROFILEDEF,IFCDIRECTION,IFCREAL,IFCPOSITIVELENGTHMEASURE,IFCCOLUMN,IFCEXTRUDEDAREASOLID,IFCGLOBALLYUNIQUEID,IFCLABEL,IFCIDENTIFIER) {"+compiled.outputText+"})")(ifcAPI,IFCAXIS2PLACEMENT3D,IFCLENGTHMEASURE,IFCCARTESIANPOINT,IFCAXIS2PLACEMENT2D,IFCCIRCLEPROFILEDEF,IFCDIRECTION,IFCREAL,IFCPOSITIVELENGTHMEASURE,IFCCOLUMN,IFCEXTRUDEDAREASOLID,IFCGLOBALLYUNIQUEID,IFCLABEL,IFCIDENTIFIER);
      console.log(` --- Ending EVAL!`);
  }


  let ifcData = ifcAPI.SaveModel(model);
  let ifcDataString = new TextDecoder('ascii').decode(ifcData);

  ifcAPI.CloseModel(model);

  let m2 = ifcAPI.OpenModel(ifcData);
  ifcThree.LoadAllGeometry(scene, m2);
}

async function resetCode() {
    window.localStorage.setItem('code', exampleCode);
    location.reload();
}

async function clearMem() {
    ClearScene();
    ifcAPI.Dispose();
    await ifcAPI.Init();
}

async function fileInputChanged() {
  let fileInput = <HTMLInputElement>document.getElementById('finput');
  if (fileInput.files.length == 0) return console.log('No files selected!');
  const file = fileInput.files[0];
  const reader = getFileReader(fileInput);
  reader.readAsArrayBuffer(file);
}

function getFileReader(fileInput: HTMLInputElement){
  var reader = new FileReader();
  reader.onload = () => {
    const data = getData(reader);
    LoadModel(data);
    fileInput.value = '';
  };
  return reader;
}

function getData(reader : FileReader){
  const startRead = ms();
  //@ts-ignore
  const data = new Uint8Array(reader.result);
  const readTime = ms() - startRead;
  console.log(`Reading took ${readTime} ms`);
  return data;
}

async function LoadModel(data: Uint8Array) {
    const start = ms();
    //TODO: This needs to be fixed in the future to rely on elalish/manifold
    const modelID = ifcAPI.OpenModel(data, { COORDINATE_TO_ORIGIN: true }); 
    const time = ms() - start;
    console.log(`Opening model took ${time} ms`);
    ifcThree.LoadAllGeometry(scene, modelID);


    if(ifcAPI.GetModelSchema(modelID) == 'IFC2X3' || 
    ifcAPI.GetModelSchema(modelID) == 'IFC4' ||
    ifcAPI.GetModelSchema(modelID) == 'IFC4X3_RC4')
    {   
        //Example to get all types used in the model
        let types = await ifcAPI.GetAllTypesOfModel(modelID);
        if(types)
        {
            for (let i = 0; i < types.length; i++) {
                let type = types[i];
                //console.log(type);
                //console.log(type.typeID);
                //console.log(type.typeName);
            }
        }
    }

    try
    {
        // This function should activate only if we are in IFC4X3
        let alignments = await ifcAPI.GetAllAlignments(modelID);
        console.log("Alignments: ", alignments);
    } catch (error) {
        // Code to handle the error
        console.error("An error occurred:", error);
    }

    let lines = ifcAPI.GetLineIDsWithType(modelID,  IFCUNITASSIGNMENT);
    //console.log(lines.size());
    for(let l = 0; l < lines.size(); l++)
    {
        //console.log(lines.get(l));
        let unitList = ifcAPI.GetLine(modelID, lines.get(l));
        //console.log(unitList);
        //console.log(unitList.Units);
        //console.log(unitList.Units.length);
        for(let u = 0; u < unitList.Units.length; u++) {
            //console.log(ifcAPI.GetLine(modelID, unitList.Units[u].value));
        }
    }
    ifcAPI.CloseModel(modelID);
}