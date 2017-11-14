#!/bin/bash

# start.sh accepts any number of arguments that will be passed on to brain.py
# example: ./start.sh OTHER ARGUMENTS config_file
# OTHER ARGUMENTS can specify starting behavior or robot ip/port
#   (this will override config file, see brain.py for details)
# the last argument is the config file
# if 'start_speech = true' is in the config file,
#   speech recognition will be started (if speech.jar can be found where expected!)
# on ctrl-c or enter press, both brain.py and speech recognition will be killed.
# However, if something weird happens a manual kill of the subprocesses might be needed.
# Try 'killall python' and maybe 'killall java'

######################################
# INITIALIZE VARIABLES AND FUNCTIONS #
######################################


#take care aliases are actually used
shopt -s expand_aliases

#get the name of the config file
if [ "$#" == "0" ]; 
    then 
        ARGS=config/config
        CONFIG=config/config
    else 
        ARGS=$@             #all command line arguments
        eval CONFIG=\$$#    #the last argument should be the config file
                            #eval makes bash expands the command first, then execute it
fi

echo "Arguments are        : " $ARGS
echo "Configuration file is: " $CONFIG

trap ctrl_c INT #Catch CTRL-C

#cleaning function
function cleanup() {
        echo "*** Killing all created processes   ***"
        alldead=1
        for PID in $pids
        do
            ps -p $PID > /dev/null
            ret="$?"
            if [ "$ret" == 0 ]
            then
                kill $PID #kills all processes for which pid was saved
                alldead=0
            fi
        done
        if [ "$alldead" == "1" ]
        then
            exit
        fi
        sleep 2
        for PID in $pids
        do
            ps -p $PID > /dev/null
            ret="$?"
            if [ "$ret" == 0 ]
            then
                kill -9 $PID #kills all processes for which pid was saved
            fi
        done
        killall batchflow
        exit
}

#function for handling CTRL-C presses
function ctrl_c() {
        echo
        echo "*** CTRL-C was pressed, cleaning up ***"
        cleanup #cleanup after CTRL-C is pressed
}

#########################
# GET LOGGING OPTIONS   #
#########################
if [ ! "$LOG_LEVEL" ]
then
    LOG_LEVEL="INFO"
fi
if [ ! "$LOG_NAME" ]
then
    LOG_NAME="Borg.Brain"
fi

LOGGING_OPTS="--log=${LOG_NAME} --log-level=${LOG_LEVEL}"
if [ "$LOG_FILE" ]
then
    LOGGING_OPTS="${LOGGING_OPTS} --log-file=${LOG_FILE}"
fi
if [ "$LOG_FORMAT" ]
then
    LOGGING_OPTS="${LOGGING_OPTS} --log-format=${LOG_FORMAT}"
fi

echo "Logging options: ${LOGGING_OPTS} "

########################
## Port configuration ##
########################

if [ "$PIONEER_PORT" == "" ]
then
    PIONEER_PORT=12345
fi
if [ "$COMMUNICATOR_PORT" == "" ]
then
    COMMUNICATOR_PORT=49152
fi

if [ "$RANDOM_PORTS" != "" ]
then
    START_PORT=49152
    END_PORT=50000
    let "RANGE = $END_PORT - $START_PORT"
    let "PIONEER_PORT = ($RANDOM % $RANGE) + $START_PORT"
    let "COMMUNICATOR_PORT = ($RANDOM % $RANGE) + $START_PORT"
fi

let "SPEECH_PORT = $RANDOM % 100 + 50000"

$BORG/scripts/robot-check_servers.sh --pioneer-port=$PIONEER_PORT --communicator-port=$COMMUNICATOR_PORT
SERVERS_STARTED=$?
HOSTNAME=`hostname`

if [ "$SERVERS_STARTED" != "0" ]
then
    # BEEP
    echo -en '\007'
        
    echo "**********************************************************************"
    echo "* One or more of the robot servers failed to start. Please check if  *"
    echo "* all laptops and the pioneer are online and try again               *" 
    echo -en '\007'
    if [ "$HOSTNAME" == "brain" ]
    then
        echo "**********************************************************************"
#        exit 1
    else
        echo "*                                                                     "
        echo "* WARNING: Brain not running on the brain laptop (but on $HOSTNAME);  "
        echo "* ignoring server start failure                                       "
        echo "**********************************************************************"
    fi
fi

#########################
# START ACTUAL PROGRAMS #
#########################

#start main program, storing the pid
python brain.py $LOGGING_OPTS --pioneer_port=$PIONEER_PORT --communicator_port=$COMMUNICATOR_PORT --speech_port=$SPEECH_PORT $ARGS &
#/opt/python/usr/bin/python brain.py $LOGGING_OPTS --pioneer_port=$PIONEER_PORT --communicator_port=$COMMUNICATOR_PORT --speech_port=$SPEECH_PORT $ARGS &
pids=$!
brain="$pids"

#give main program some time to start the server
sleep 6


#if config contains a line 'start_speech = true' (with possible whitespace around the strings)
#dospeech=`awk -F= --source '$1~/^[ \t\r\n\v\f]*start_speech[ \t\r\n\v\f]*$/ && $2~/^[ \t\r\n\v\f]*true[ \t\r\n\v\f]*$/ {print "true"}' $CONFIG`
#dospeech now contains "true" for every line, might be "true true true"
#if [ "${dospeech:0:4}" == "true" ]; then #if the first four characters are "true"
#    echo starting java
#    #start java for speech recognition, storing the pid
#    java -jar $BORG/Brain/src/speech/client/speech.jar vcc.config.xml &
#    pids="$pids $!"
#fi


#if config contains a line 'start_speech = true' (with possible whitespace around the strings)
dospeech=`awk -F= -W source '$1~/^[ \t\r\n\v\f]*start_speech[ \t\r\n\v\f]*$/ && $2~/^[ \t\r\n\v\f]*true[ \t\r\n\v\f]*$/ {print "true"}' $CONFIG`
#dospeech now contains "true" for every line, might be "true true true"
if [ "${dospeech:0:4}" == "true" ]; then #if the first four characters are "true"
    echo setting grammar
    
    ### start extracting which vcc to use from config file ###
    # Key in Property File
    key="grammar"
    # Variable to hold the Property Value
    prop_value=""

    getProperty()
    {
            prop_key=$1
            prop_value=`cat ${CONFIG} | grep ${prop_key} | cut -d'=' -f2`
    }

    getProperty ${key}
    echo "Key = ${key} ; Value = " ${prop_value}
    ### config.xml extracted ###  
       
#    #start java for speech recognition, storing the pid
    #java -jar speech/client/speech.jar ${prop_value} ${SPEECH_PORT} &
    pids="$pids $!"
fi


#stand alone executables can be added here. Don't forget to store the pid!
#example:
#
#python folder/file.py $CONFIG &
#pids="$pids $!" #save the pid to kill the process later
#

echo "**************************************"
echo "* When you press enter or ctrl-c,    *"
echo "* all created process will be killed *"
echo "**************************************"

#wait until enter is pressed
while ps -p $brain > /dev/null
do
    read -t 1 input
    ret=$?
    if [ "$ret" == 0 ]
    then
        cleanup         #cleanup after enter is pressed
        break
    fi
done

