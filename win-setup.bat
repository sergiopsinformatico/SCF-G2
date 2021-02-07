@echo off

REM  netsh wlan show profile name=MOVISTAR_ABCD key=clear

echo WIFI SID
SET /P WIFISID=Introduce WIFI SID: 
SET /P WIFIPASS=Introduce WIFI PASS:
SET /P SERVERIP=Introduce la ip de red local de tu pc:

SETX PIO_WIFI %WIFISID%
SETX PIO_WIFI_PASS %WIFIPASS%
SETX BROKER_IP %SERVERIP%

echo Antes de compilar con pio refrescar variables de entorno:
echo RefereshEnv (Disponible con cmder). Alternativamente cerrar y abrir cmd.
