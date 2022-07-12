/*
El archivo gazebo/gazebo.hh incluye un conjunto basico de funciones basicas
de gazebo. 

Todos los complementos deben estar en el espacio de nombres "gazebo".

Cada complemento debe heredar de un tipo de complemento, que en este caso
es la clase "worldPlugin".
*/
#include <gazebo/gazebo.hh>

// Declaramos un espacio de nombres
namespace gazebo{
    // Heredamos de la clase "WorldPlugin"
    class WorldPluginTutorial : public WorldPlugin{
        // En el constructor inicializamos primero la clase base y despues la clase derivada
        public: WorldPluginTutorial() : WorldPlugin(){
            printf("Hello World!\n");
        }

        /*
        Esta funcion es obligatoria, la cual recibe un elemento SDF que contiene
        los elementos y atributos especificos en el archivo SDF cargado.
        */
        public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){

        }
    };

    /*
    Finalmente, el complemento debe registrarse en el simulador mediante el macro
    "GZ_REGISTER_WORLD_PLUGIN". El unico parametro de esta macro es el nombre de 
    la clase del complemento.

    Hay macros de registro para cada tipo de complemento:
        GZ_REGISTER_<tipo_de_complemeto>_PLUGIN
            - WORLD
            - MODEL
            - SENSOR
            - GUI
            - SYSTEM
            - VISUAL
    */
    GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}