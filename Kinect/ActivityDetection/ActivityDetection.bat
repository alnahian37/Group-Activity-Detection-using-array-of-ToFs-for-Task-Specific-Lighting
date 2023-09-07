@echo off

rem Set the working directory two levels above the batch file location
cd /d "%~dp0..\.."

call conda activate pyKinectAzure

python ".\Kinect\ActivityDetection\ActivityDetection.py" ^
    --activ_mode 1 ^
    --on_off 1 ^
    --activ_slowDown 5 ^
    --recordFlag 0 ^
    --recTime 10 ^
    --person_present 0 ^
    --plot_flag 1

call conda deactivate
