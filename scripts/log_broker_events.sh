##!/bin/bash
# Configuración del broker y topic a escuchar.
# Configuración inicial:
BROKER="localhost"
PORT="1883"
TOPIC="data/#"
BROKER_FILES_PATH="../broker_files"
LOG_FILE="log.txt"
#--------------------------------------------------
# Ponemos el cliente de mosquitto a escuchar
mosquitto_sub -t $TOPIC -h $BROKER -p $PORT -v | while read value;    do
TIMESTAMP=$(date "+%d/%m/%Y %H:%M:%S") # Agregamos una marca de tiempo  en la variable ts (opcional).
SPLIT_DATA=($(echo $value | tr "/" "\n"))

# Se crea la carpeta destino
mkdir -p ${BROKER_FILES_PATH}

# Guardamos valores:
echo "$TIMESTAMP MQTT-${SPLIT_DATA[1]^^} $value" >> "${BROKER_FILES_PATH}/${LOG_FILE}"   # guardamos datos en archivo
echo "$value"                  # mostramos el resultado por consola
done
