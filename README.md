#OpenFDM: An open source flight dynamics library for Modelica

## License

* OpenFDM  Copyright (C) 2012 James Goppert, Jennifer Wu
* License: GPL v3
* This program comes with ABSOLUTELY NO WARRANTY; This is free software, and you are welcome to redistribute it under certain conditions. For details see license.txt.

## Development

* Modified MultiBody library used to fix linearization issues with the OpenModelica Compiler.
* A test suite is provided using python. It can be run using python nose:

```bash
nosetests -v
```

## Scripts

* The scripts in the scripts directory can be run using omc. The project path must be appended to OPENMODELICALIBRARY. To run the script datcom.mos from the command line type:

```bash
OPENMODELICALIBRARY=$OPENMODELICALIBRARY:/path/to/openfdm omc +s scripts/datcom.mos
```

### Helper Scripts

* run: Prepends openfdm path to OPENMODELICALIBRARY and runs a script.

* term: Starts OMShell-terminal with the proper paths for OpenFDM.

* omset: Sets the modelica path for svn or installed versions.
