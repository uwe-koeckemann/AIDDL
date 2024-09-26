# The AIDDL Core Library

This is the Python implementation of the AIDDL Core Library.

It provides everything needed for 

- Parsing AIDDL files
- Working with containers and modules
- Creating terms and types
- Evaluating terms and types
- Default functions for evaluation

## Changes

### 0.3.6

- Fixed initialization error for string term

### 0.3.4

- AIDDL files now packaged in sdist and bdist

### 0.3.2

- Refactored parser
- Added aiddl resource module containing .aiddl files to remove dependency on AIDDL_PATH environment variable
- Modules loaded from symbolic terms now need to be in a path that follows their name
  - Example: `x.y.z` must be in a file `x/y/z.aiddl` in any known path
- Python modules can be added as path sources for .aiddl files to the new `Parser` class. 
  - For example: `parser = Parser(c, aiddl_modules=[a, b, c])` assumes `a`, `b`, and `c` are python modules
