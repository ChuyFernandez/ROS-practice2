#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
// Incluimos el archivo .h de la definicion de nuestro robot.
// ...

int main(int argc, char** argv){
    // Inicializamos un nodo ROS.
    ros::init(argc,argv, "my_robot");

    // Creamos una instancia de nuestro robot.
    MyRobot::MyRobot robot;

    /*
    Creamos una instancia del administrador del controlador y le pasamos por referencia
    la instancia del robot.
    */
    controller_manager::ControllerManager cm(&robot);

    /*
    Configuramos un hilo separado que se utilizara para dar servicio a las
    devoluciones de llamada de ROS (se configura de forma asincrona).
        El numero de hilos es la cantidad de subprocesos que se utilizaran.

    Recordar que tambien existe otro metodo llamado "spin()", el cual entra en un
    buble para procesar devoluciones de llamada.
    */
    ros::AsyncSpinner spinner(1); // Numero de hilos
    spinner.start();

    // Configuracion para el bucle de control
    ros::Time prev_time=ros::Time::now();
    ros::Rate rate(10.0); // Tasa de 10 Hz

    /*
    ros::ok() devuelve verdadedo mientras el nodo no este apagado.
    */
    while(ros::ok()){
        /*
        Contabilidad basica para obtener la hora del sistema con el fin de calcular 
        el periodo de control.
        */
        const ros::Time time=ros::Time::now();
        // Aqui obtenemos el tiempo transcurrido hasta el momento.
        const ros::Duration period=time-prev_time;

        // Leemos los datos del hardware del robot.
        robot.read();
        
        // Actualizamos todos los controladores activos (con la informacion reciente).
        cm.update(time, period);

        // Escribimos comandos en el hardware del robot (articulaciones, actuadores).
        robot.write();

        // Todos estos pasos se seguiran repitiendo con la tasa especificada (10 Hz).
        rate.sleep();
    }
    return 0;
}