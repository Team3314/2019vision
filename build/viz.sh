#!/bin/bash
sleep 5
#/3314/src/bad_settings.sh 1
/3314/src/good_settings.sh 0
/3314/src/bad_settings.sh 0
/3314/src/good_settings.sh 0
/3314/src/build/vizsingle minhsv 65 0 100 maxhsv 100 245 254 leftcameraangle 17.25 leftcameraid 0
exit 0
