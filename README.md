# BIOS-KINECTandSMARTPHONE-CONTROLLER-FOR-DISPLAYCLUSTER
BIOS-Kinect+Smartphone Controller for DisplayCluster

La aplicación consiste en controlar DisplayCluster a partir del Kinect y la aplicación
TUIOdroid instalado en un smartphone.

- Abrir la carpeta \KinectTUIO-Full_Interaction\BUILD2010\bin\Release\
- Ejecutar IVW_Application.exe
- En DisplayCluster se debe realizar la conexión con el servicio mostrado en la barra de estado
de la aplicación, por defecto aparecerá un servicio llamado Analog0@[DireccionIP]
- Con la app TUIOdroid se debe realizar una conexión a la misma dirección IP que se conecta VRPN. Si
existen problemas para conectar se debe desactivar el firewall de Windows.

Con la aplicación del celular se maneja el cursor de DisplayCluster, con el Kinect se realiza el
movimiento de las ventanas cuando se extiende los brazos horizontalmente frente al Kinect, la distancia
de la persona al Kinect hace Zoom In o Zoom Out a la ventana activa.