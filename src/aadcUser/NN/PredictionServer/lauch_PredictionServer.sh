#!/bin/bash
if [ -f /home/aadc/AADC/src/aadcUser/NN/PredictionServer/build_active.txt ]; then
   echo "Not launching PredictionServer to avoid blocking build process.";
   exit;
fi
export PATH="/home/aadc/anaconda3/bin:$PATH";
source /home/aadc/anaconda3/bin/activate aadc-env;
cd /home/aadc/AADC/src/aadcUser/NN/PredictionServer;
python PredictionServer.py > terminal_output.txt 2>&1;