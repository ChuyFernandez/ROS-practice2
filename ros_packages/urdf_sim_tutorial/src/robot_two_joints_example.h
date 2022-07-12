// Importamos las declaraciones de las clases (interfaces) que necesitaremos
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// Heredamos de la clase "hardware_interface::RobotHW"
class MyRobot : public hardware_interface::RobotHW{
    public:
        /*
        Dentro del constructor inicializamos los recursos del robot (articulaciones, sensores, actuadores)
        y las interfaces.
        */
        MyRobot(){
            /*
            Creamos un objeto de tipo "JointStateHandle" para cada articulacion y los registramos con 
            el objeto de tipo "JointStateInterface".
                "JointStateHandle" es un "handle" (manejador) que se usa para leer el estado de una sola
                articulacion. Actualmente los campos de posicion, velocidad y esfuerzo son obligatorios.
            */
            hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
            joint_state_interface.registerHandle(state_handle_a);
            hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
            joint_state_interface.registerHandle(state_handle_b);
            /*
            Registramos el objeto de tipo "JointStateInterface" que contiene las articulaciones de solo
            lectura.
            */
            registerInterface(&joint_state_interface);

            /*
            Creamos un objeto de tipo "JointHandle" (lectura y escritura) para cada articulacion 
            controlable usando los identificadores de articulacion de solo lectura dentro del objeto de 
            tipo "JointStateInterface" y los registramos en el objeto de tipo "PositionJointInterface".
                "JointHandle" es un "handle" (manejador) que se utiliza para leer y controlar una sola
                articulacion.
            */
            hardware_interface::JointHandle pos_handle_a(joint_state_interface.getHandle("A"), &cmd[0]);
            position_joint_interface.registerHandle(pos_handle_a);
            hardware_interface::JointHandle pos_handle_b(joint_state_interface.getHandle("B"), &cmd[1]);
            position_joint_interface.registerHandle(pos_handle_b);
            /*
            Registramos el objeto de tipo "PositionJointInterface" que contiene las articulaciones de solo
            lectura/escritura.
            */
            registerInterface(&position_joint_interface);
        }
        ~MyRobot(){}
    private: 
        // Declaramos las interfaces que necesitaremos
        hardware_interface::PositionJointInterface position_joint_interface;
        hardware_interface::JointStateInterface joint_state_interface;
        /*
        Declaramos una matriz para almacenar los comandos del controlador que se envian a los recursos del 
        robot (articulaciones, actuadores).
        */
        double cmd[2];
        /*
        Declaramos matrices para almacenar el estado de los recursos del robot (articulaciones, sensores).
        */
        double pos[2], vel[2], eff[2];
};