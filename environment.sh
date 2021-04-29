AIDDL_HOME=$(pwd)/$line
export AIDDL_HOME

AIDDL_PATH="$AIDDL_HOME/core/aiddl:$AIDDL_HOME/common/aiddl:$AIDDL_PATH"
export AIDDL_PATH

AIDDL_WORK="/tmp"
export AIDDL_WORK

PYTHONPATH="$AIDDL_HOME/core/python/src:$AIDDL_HOME/network/python:$PYTHONPATH"
export PYTHONPATH
