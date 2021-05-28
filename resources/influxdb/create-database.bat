@echo off

REM https://docs.influxdata.com/influxdb/v1.8/write_protocols/line_protocol_reference/
echo Warning!!! The database will be deleted if already exists
set /p CONTINUE=Continue? (y/N)

IF NOT "%CONTINUE%"=="y" (exit /b 0)


curl -i "http://localhost:8086/query" --data-urlencode "q=DROP DATABASE residentAngel" 
curl -i "http://localhost:8086/query" --data-urlencode "q=DROP DATABASE aemet" 

curl -i "http://localhost:8086/query" --data-urlencode "q=CREATE DATABASE residentAngel" 
curl -i "http://localhost:8086/query" --data-urlencode "q=CREATE DATABASE aemet" 
REM Politica de retención de datos en bdd aemet. 1 Hora.
curl -i "http://localhost:8086/query" --data-urlencode "q=CREATE RETENTION POLICY aemet_retention ON aemet DURATION 1h REPLICATION 1" 
curl -i -XPOST "http://localhost:8086/write?db=residentAngel" --data-binary @residences.data

exit /b


REM Useful sentences:

curl -i -XPOST "http://localhost:8086/write?db=residentAngel" --data-binary @residences.data

curl -i http://localhost:8086/query --data-urlencode "db=residentAngel" --data-urlencode "q=SELECT * FROM residences"
curl -i http://localhost:8086/query --data-urlencode "db=residentAngel" --data-urlencode "q=SELECT * FROM light"
curl -i http://localhost:8086/query --data-urlencode "db=residentAngel" --data-urlencode "q=SELECT * FROM temperature  "
curl -i http://localhost:8086/query --data-urlencode "db=residentAngel" --data-urlencode "q=SELECT * FROM room_status "
curl -i http://localhost:8086/query --data-urlencode "db=residentAngel" --data-urlencode "q=SELECT * FROM residences"
curl -i http://localhost:8086/query --data-urlencode "db=residentAngel" --data-urlencode "q=SELECT municipio FROM residences where residenceId=#idResidence;"
curl -i http://localhost:8086/query --data-urlencode "db=aemet" --data-urlencode "q=SELECT orto,ocaso,temp FROM weather_forecast where municipio=13035 order by time DESC limit 1"
