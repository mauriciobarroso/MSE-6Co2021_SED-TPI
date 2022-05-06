# Trabajo Pŕactico Integrado para la materia Sistemas Embebidos Distribuidos

## Oganización
El proyecto está compuesto por varias carpetas donde se encuentran todos los entregables del trabajo.

### broker_files
En esta carpeta se encuentra el archivo ``log.txt`` que tiene registrados todos los mensajes MQTT intercambiados entre el nodo y la aplicación. Esta carpeta y su contenido son creados automáticamente con el script ``log_broker_events.sh`` que se encuentra en la carpeta ``scripts``

### firmware
El código fuente principal y todos sus componentes se encuentran en esta carpeta. Su organización es la empleada y reocomendada por Espressif.

### node_files
Los archivos generados por el nodo, es decir, el archivo de registro ``log.txt`` los archivos .csv que contienen las muestras de las mediciones se encuentran en esta carpeta. Esta carpeta y su contenido son creados automáticamente con el script ``download_node_files.sh`` que se encuentra en la carpeta ``scripts``

### octave
Las archivos .m propios de Octave se encuentran en esta carpeta, uno para cada una de las mediciones requeridas.

### scripts
Contiene los scripts para descargar y generar los archivos de las carpetas ``broker_files`` y ``node_files``

## Informe
El informe se encuentra en el archivo ``MSE-6Co2021_SED-TPI-Report.pdf`` tal como se requirió en el enunciado del trabajo.
