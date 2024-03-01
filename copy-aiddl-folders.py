#!/bin/python

import os

# Copy aiddl folders for core and common into the resource folders of the
# corresponding libraries

# Clear current
os.system("rm -rf ./core/scala/src/main/resources/aiddl")
os.system("rm -rf ./core/scala/src/main/resources/aiddl-test")
os.system("rm -rf ./core/python/aiddl_core/aiddl")
os.system("rm -rf ./core/python/aiddl_core/test/aiddl")
os.system("rm -rf ./common/scala/src/main/resources/aiddl")
os.system("rm -rf ./common/scala/src/main/resources/aiddl-test")
os.system("rm -rf ./common/python/aiddl_common/aiddl")

# Create directory again
os.system("mkdir ./core/scala/src/main/resources/aiddl")
os.system("mkdir ./core/scala/src/main/resources/aiddl-test")
os.system("mkdir ./core/python/aiddl_core/aiddl")
os.system("mkdir ./core/python/aiddl_core/test/aiddl")
os.system("mkdir ./common/python/aiddl_common/aiddl")
os.system("mkdir ./common/scala/src/main/resources/aiddl")
os.system("mkdir ./common/scala/src/main/resources/aiddl-test")

# Copy
os.system("cp -r ./core/aiddl/* ./core/scala/src/main/resources/aiddl/")
os.system("cp -r ./core/test/* ./core/scala/src/main/resources/aiddl-test/")
os.system("cp -r ./core/aiddl/* ./core/python/aiddl_core/aiddl/")
os.system("cp -r ./core/test/* ./core/python/aiddl_core/test/aiddl/")
os.system("cp -r ./common/aiddl/* ./common/scala/src/main/resources/aiddl/")
os.system("cp -r ./common/test/* ./common/scala/src/main/resources/aiddl-test/")
os.system("cp -r ./common/aiddl/* ./common/python/aiddl_common/aiddl")

for d in os.walk("./core/python/aiddl_core/aiddl"):
    os.system(f"touch {d[0]}/__init__.py")

for d in os.walk("./core/python/aiddl_core/test/aiddl"):
    os.system(f"touch {d[0]}/__init__.py")

for d in os.walk("./common/python/aiddl_common/aiddl"):
    os.system(f"touch {d[0]}/__init__.py")

