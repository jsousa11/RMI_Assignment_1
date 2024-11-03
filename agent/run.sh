#!/bin/bash

challenge="1"
host="localhost"
robname="theAgent"
pos="0"

while getopts "c:h:r:p:" op; do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        *)
            echo "ERROR in parameters"
            exit 1
            ;;
    esac
done

shift $((OPTIND - 1))

case $challenge in
    1)
        # Call agent for Challenge 1 with --scoring 1
        python3 mainRobC1.py -h "$host" -p "$pos" -r "$robname"
        ;;
    2)
        # Call agent for Challenge 2 with --scoring 2
        python3 mainRobC2.py -h "$host" -p "$pos" -r "$robname"
        # Ensure output is correctly renamed for Challenge 2
        mv mymap.txt agent_map.map
        ;;
    3)
        # Not done
        echo "Challenge 3 not implemented"
        ;;
    *)
        echo "Unknown challenge option: $challenge"
        exit 1
        ;;
esac
