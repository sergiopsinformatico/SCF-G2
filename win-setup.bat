@echo off

REM  netsh wlan show profile name=MOVISTAR_ABCD key=clear

echo PIO_WIFI %PIO_WIFI%
echo PIO_WIFI_PASS %PIO_WIFI_PASS%
echo BROKER_IP %BROKER_IP%

SET /P WIFISID=Introduce WIFI SID: 
SET /P PIO_WIFI_PASS=Introduce WIFI PASS:
SET /P BROKER_IP=Introduce la ip de red local de tu pc:

SETX PIO_WIFI %WIFISID%
SETX PIO_WIFI_PASS %WIFIPASS%
SETX BROKER_IP %SERVERIP%

echo Antes de compilar con pio refrescar variables de entorno:
echo RefereshEnv (Disponible con cmder). Alternativamente cerrar y abrir cmd.
