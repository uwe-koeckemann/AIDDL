cd java
SET cmd=gradlew.bat run -Dexec.args=%*
echo %cmd%
call %cmd%
cd ..

