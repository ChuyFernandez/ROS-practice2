// Importamos las declaraciones de las clases (interfaces) que necesitaremos
// ...
#include <hardware_interface/robot_hw.h>

/*
Para un robot es posible utilizar interfaces estandar (cuando es posible y compatible)
como tambien interfaces especificas del robot.

    - Interfaces estandar nos referimos a las inerfaces ya definidas y listas para usar.

    - Interfaces especificas del robot nos referimos a las que nosotros mismos creamos 
      especificas de nuestro robot.

A continuacion se muestra un ejemplo de un robot con interfaces estandar y especificas
del robot.
*/

// Heredamos de la clase "hardware_interface::RobotHW"
class MyRobot : public hardware_interface::RobotHW{
    public:
        /*
        Dentro del constructor inicializamos los recursos del robot.
        */
        MyRobot(){
            // Registramos interfaces de hardware estandar
            registerInterface(...);

            // Registramos algunas interfaces especificas del robot
            registerInterface(&my_interface);
        }
        ~MyRobot(){}
    private: 
        // Declaramos las interfaces que necesitaremos
        // ...
        MyCustomInterface my_interface;

        // Algunas variables
        // ...
};