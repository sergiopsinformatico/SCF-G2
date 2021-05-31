# Proyect Setup

1. Ejecutar win-setup.bat, para configurar las variables de entorno necesarias.

2. Ejecutar resources/docker/deploy-containers.bat para desplegar los contenedores de InfluxDB, Node-RED, Grafana y Mosquitto.

3. Ejecutar el script resources/influxDB/create-database.bat para crear las bdd y tablas de InfluxDB.

4. Conectar a la consola del contendor de Node-RED e instalar.

```
npm install node-red-contrib-protobuf --production
```

5. En el contenedor de nodeRed copiar el fichero environment.proto en la carpeta /usr/src/node-red

6. Reiniciar nodeRed e importar los flujos.

7. Abrir la carpeta web en el terminal.

8. Ejecutar desde la carpeta /web los siguientes comandos

```
npm install
npm start
```

9. Acceder a la url http://localhost:4200/

9. Ejecutar desde la raíz del proyecto el siguiente comando:

```
pio run --target clean &  pio run --target erase & pio run --target upload & pio device monitor -b 115200
```

