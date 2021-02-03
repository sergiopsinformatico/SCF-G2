@echo off

REM https://docs.influxdata.com/influxdb/v1.8/write_protocols/line_protocol_reference/
echo Warning!!! The database will be deleted if already exists
set /p CONTINUE=Continue? (y/N)

IF NOT "%CONTINUE%"=="y" (exit /b 0)


curl -i "http://localhost:8086/query" --data-urlencode "q=DROP DATABASE residentAngel" 
curl -i "http://localhost:8086/query" --data-urlencode "q=CREATE DATABASE residentAngel" 
curl -i -XPOST "http://localhost:8086/write?db=residentAngel" --data-binary @residences.data

curl -i http://localhost:8086/query --data-urlencode "db=residentAngel" --data-urlencode "q=SELECT * FROM residences"