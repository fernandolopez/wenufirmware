Firmware para el proyecto Wenu
==============================

Funcionalidades:

1. Toma mediciones de:
    * Temperatura
    * Humedad
    * Nivel de batería (medición interna)
    * Corriente (phidget o sensor analógico para medir la corriente de un aire acondicionado por ejemplo)
    * Movimiento (phidget o sensor analógico, actualmente un PIR)

2. Envía las mediciones codificadas en JSON por MQTT a un broker utilizando
IPv6 (6lowpan).

3. Espera mensajes de acción para controlar algún actuador (actualmente puede
apagar un aire acondicionado utilizando hardware extra conectado por i2c)

Para esto se subscribe a un tópico de MQTT propio de la mota:
`<mote_id>/action` y espera un formato codificado en binario.

El formato esperado es (en network byte order):
```
1 byte -> tipo de mensaje
1 byte -> longitud del primer argumento
n bytes -> primer argumento
[1 byte -> longitud del segundo argumento]
[n bytes -> segundo argumento]
[...]
```



Setup
-----

1. Descargar CONTIKI 3.0 y establecer la variable de entorno CONTIKI que apunte
a su directorio.

2. Descargar msp430-gcc http://lihuen.linti.unlp.edu.ar/bajar/msp430/
descomprimirlo y agregar a la variable de entorno PATH el directorio
que contiene los binarios.

3. Instalar pyserial 2.7 (la versión 3 no funciona con contiki 3.0)

Por ejemplo con virualenv:

```sh
virtualenv .venv
. .venv/bin/activate
pip install pyserial\<3
```

Setup de la mota
----------------

La mota debe tener asignado un node\_id de forma de determinar su identidad,
esta identidad es utilizada inferir un string que identifique a la mota
de forma descriptiva. La asociación entre node\_id y esta descripción
(mote\_id) se realiza en las primeras líneas del proceso principal.
