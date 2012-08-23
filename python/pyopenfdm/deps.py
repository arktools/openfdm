import os, sys, inspect
 # realpath() with make your script run, even if you symlink it :)
dep_root = os.path.realpath(os.path.abspath(os.path.join(
    os.path.split(inspect.getfile( inspect.currentframe() ))[0],'deps')))

for dep in ['OMPython','DyMat']:
    path = os.path.join(dep_root,dep)
    if path not in sys.path:
        sys.path.insert(0, path)
