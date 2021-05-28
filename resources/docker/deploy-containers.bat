@echo off

if "%BROKER_IP%" == "" (
    echo BROKER_IP not setted. 
    echo Run win-setup.bat before deploy containers
    exit /b 1
)

docker pull nodered/node-red:1.2.7
docker pull influxdb:1.8.3
docker pull grafana/grafana:6.5.0-ubuntu
docker pull eclipse-mosquitto:1.6.14

docker network create mybridge
docker run -d -p 8086:8086 --name=influxdb --net=mybridge influxdb:1.8.3
docker run -d -p 2883:1883 --name=broker --net=mybridge eclipse-mosquitto:1.6.14
docker run -d -p 3000:3000 --name=grafana --net=mybridge grafana/grafana:6.5.0-ubuntu 
docker run -d -it -p 1880:1880 -v node_red_data:/data ^
-e TZ=Europe/Madrid ^
-e BROKER_IP=%BROKER_IP% ^
-e INFLUX_IP=%BROKER_IP% ^
-e SENSOR_DATABASE_NAME=residentAngel ^
-e AEMET_DATABASE_NAME=aemet ^
--name mynodered --net=mybridge  nodered/node-red:1.2.7

