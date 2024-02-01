export LOGNAME=/tmp/process_manager/`date "+%Y%m%d_%H%M%S"`
mkdir -p $LOGNAME
/usr/bin/python3 process_manager.py # > logs/process_manager.log 2>&1