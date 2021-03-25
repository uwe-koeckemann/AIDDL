#!/bin/bash
cd java
cmd="./gradlew run -Dexec.args='"$@"'"
echo $cmd
eval $cmd
cd ..
