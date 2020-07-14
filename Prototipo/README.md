# PROTOTIPO

| | | | |
|-|-|-|-|
|[**Arduino**](#Arduino) |[**Azure**](#Azure) |[**Nodered**](#Nodered) |[**Raspberry**](#Raspberry)|
| | | | |

Hay toda la programación utilizada y explicada para poder enseñar al cliente un prototipo del futuro funcionamiento según nuestra propuesta. Es solo un prototipo ya que se han utilizado dispositivos y sensórica simple, para el proyecto final se utilizará el material descrito en la documentación. 

El prototipo que se ha creado es una representación reducida de toda la comunicación del proyecto. Des de el sensor o dispositivo en la granja hasta el dashboard en la nube.

En la siguiente imagen se pueden apreciar los sensores, dispositivos y comunicaciones entre ellos que se usan para poder llevar a cabo la adquisición de datos y cumplir los requisitos establecidos por el cliente. 

![Comunicacion image](Imágenes/Diagrama_comunicacion.png)

Como se puede visualizar en el esquema, se tienen tres ESP32 con diversos sensores según el dato que se quiera controlar. La comunicación de estos dispositivos se va a encargar de diferentes áreas, tales como el control de la puerta de la fabrica, la cuba de las granjas, las vacas y el camión, donde a continuación se explicará cada una de ellas para entender el prototipo que se llevara a cabo.
La adquisición de datos de las vacas se determinará por medio de ESP32, la cual tendrá dos sensores conectados a la mismas, tales como el sensor de temperatura, y GPS. Es importante destacar que la transmisión de datos se va a llevar a cabo a través de LoraWan ya que se utilizara un GatewayLora. 
Por otra parte, en el esquema se puede determinar que la cuba se controlara a su vez a través de la tarjeta la cual tendrá conectada un sensor de temperatura y en este caso para simular el nivel de tanque se conectara el sensor de ultrasonido para así poder simular la capacidad que contiene el mismo. La comunicación se llevará a cabo a través del protocolo MQTT.
Los camiones estarán controlados a través de una ESP32, la cual tiene conectado un sensor de temperatura, sensor de ultrasonido para determinar el nivel en el que se encuentra el tanque al cargarse del producto o descargase y un GPS para su localización.
La puerta de la fábrica se contrala a través de una ESP32, tendrá conectado un relé, el cual recibirá la orden de apertura por el usuario a través de telegram. Todo esto se realizará con el protocolo de comunicación MQTT. 
Los datos correspondientes a la cubas, camiones y puerta se transmitirán a un mismo Gateway como se puede apreciar en el esquema anteriormente
La comunicación de las ESP32 a la raspberry la cual hace de Gateweay y también de Wifi,  mediante el protocolo de MQTT transmitirá los datos a la nube Azure, donde se tendra un broker y Node-red para así poder tener la información en el dashboard para los usuarios.

En el apartado de las granjas en el Dashboard, a través de la programación en el sensor de temperatura DHT11 y el sensor de ultrasonido logramos representar datos ficticios de la cuba en el dashboard, los cuales se aprecian de la siguiente manera. 

![Dashboard image](Imágenes/Dashboard_Valores_Reales.png)

La puerta se contralará a través de un Bot de Telegram con el nombre Espressifcim_bot. Actualmente, tiene dos comandos, el de “/abrir” y “/cerrar”. Para la representación de la puerta en el prototipo se utiliza un Led, ya que su estado simboliza la apertura de la puerta.

A continuación, la visualización del Bot: 

![Bot image](Imágenes/Telegram_bot.png)


## [Arduino](/Prototipo/Arduino)
Sitio donde se colgaran todos los codigos que se hagan para los dispositivos que hay en el proyecto. Actualmente no son los códigos finales ya que aún falta reducir el mensaje json para que sea lo más eficiente posible.

- ### [Camion](/Prototipo/Arduino/camion)

- ### [Puerta](/Prototipo/Arduino/puerta)

- ### [Tanque](/Prototipo/Arduino/tanque)

Además, el código de LoraWAN solo se ha utilizado como prueba de comunicación con un gateway hecho por nosotros. Se ha puesto a modo de prueba ya que se utilizarán dispositivos y gateways industriales los sensores de la vaca.

- ### [Vaca](/Prototipo/Arduino/vaca)

[[Go to top]](#Prototipo)


## [Azure](/Prototipo/Azure)
Documentación importante sobre Azure. Actualmente información sobre la creación de contenedores persistentes y/o máquinas virtuales.


## [Nodered](/Prototipo/Nodered)
Dashboards i base de datos entregados hasta la fecha de _ESPRESSIF_ a _RASP_.

- ### [Sprint 01](/Prototipo/Nodered/Sprint%2001%20-%2003042020)

- ### [Sprint 02](/Prototipo/Nodered/Sprint%2002%20-%2012042020)

- ### [Sprint 03](/Prototipo/Nodered/Sprint%2003%20-%2002062020)

- ### [Sprint 04](/Prototipo/Nodered/Sprint%2004%20-%2009062020)

- ### [Sprint 05](/Prototipo/Nodered/Sprint%2005%20-%2014072020)

## [Raspberry](/Prototipo/Raspberry)
Documentación i códigos para la configuración de la raspberry como gateway i access point.

[[Go to top]](#Prototipo)
