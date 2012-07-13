within OpenFDM.Aerodynamics;
  
model DatcomTable_File
  parameter input String fileName;
  extends DatcomTable(cLp.tableOnFile = true, cLp.tableName = "cLp", cLp.columns = {2}, cLp.fileName = fileName);
end DatcomTable_File;

