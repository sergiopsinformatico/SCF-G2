# Proyect Setup

1. Ejecutar win-setup.bat, para configurar las variables de entorno necesarias.

2. Ejecutar resources/docker/deploy-containers.bat para desplegar los contenedores de InfluxDB, Node-RED, Grafana y Mosquitto.

3. Ejecutar el script resources/influxDB/create-database.bat para crear las bdd y tablas de InfluxDB.

4. Conectar a la consola del contendor de Node-RED e instalar.

> npm install node-red-contrib-protobuf --production

5. En el contenedor de nodeRed copiar el fichero environment.proto en la carpeta /usr/src/node-red

6. Reiniciar nodeRed e importar los flujos.


