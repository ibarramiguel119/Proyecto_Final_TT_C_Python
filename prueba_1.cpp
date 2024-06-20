#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include <pybind11/complex.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <tuple>
#include <windows.h>
#include <string>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <fcntl.h>
#include <chrono>
#include <thread>



namespace py = pybind11;


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


std::string floatToString(float value, int precision);
bool enviarDatosPorPuertoSerie(const std::string& datos);
std::string recibirDatosPorPuertoSerie();
double radianes_a_grados(double radianes);
double gradosARadianes(double grados);
std::string recibirDatosBloqueante(HANDLE serial_fd);
bool enviarDatosBloqueante(HANDLE serial_fd, const std::string& datos);
bool enviarDatosCaracterPorCaracter(HANDLE hSerial, const std::string& datosAEnviar);
HANDLE abrirPuertoSerie(const std::string& puerto);
void printVectorOfVectors(const std::vector<std::vector<int>>& vec);
void delaySeconds(int seconds);



int addNumbers(int a, int b) {
    auto sum= a + b;
    std::cout << "Tu suma es : " << sum << "\n";
    return sum;
}


void CalcularGrados(int Altitud, int Asimuth, int Roll ,int& GAltitude, int& GAsimut, int& GRoll ){
    double H[3];
    H[0]=Altitud;
    H[1]=Asimuth;
    H[2]=Roll;
    //Numero de grados se moveran por punto
    GAltitude=60/H[0];
    GAsimut=360/H[1];
    GRoll=270/H[2];
}

// En el siguiente arreglo almacena la distribuccion de los grados en altitude
std::vector<int>CalcularArreglo1(int GAltitude) {
    std::vector<int> array1;
    for (int i = 0; i <= 360; i += GAltitude) {
        array1.push_back(i);
    }
    return array1;
}

// En el siguiente arreglo almacena la distribuccion de los grados en Azimuth
std::vector<int>CalcularArreglo2(int GAsimut) {
    std::vector<int> array2;
    for (int i = 0; i <= 60; i += GAsimut) {
        array2.push_back(i);
    }
    return array2;
}

// En el siguiente arreglo almacena la distribuccion de los grados en Roll
std::vector<int>CalcularArreglo3(int GRoll) {
    std::vector<int> array3;
    for (int i = 0; i <= 270; i += GRoll) {
        array3.push_back(i);
    }
    return array3;
}

// En la siguiente funcion forma el los puntos donde se movera el robot
//inicializador de contador 
std::vector<std::vector<int>> CalcularPuntosMovimiento(const std::vector<int>& array1, const std::vector<int>& array2, int radio) {
    std::vector<std::vector<int>> S_data;

    for (int i = 0; i < array2.size(); ++i) {
        for (int j = 0; j < array1.size(); ++j) {
            std::vector<int> S = {array1[j], array2[i], radio};
            S_data.push_back(S);
        }
    }
    return S_data;
}



//Funcion que convierte grados a radianes
std::vector<double> grados_a_radianes(const std::vector<int>& grados) {
    std::vector<double> radianes;
    radianes.reserve(grados.size()); // Reserva espacio para evitar reasignaciones

    for (int grado : grados) {
        double radian = grado * M_PI / 180.0;
        radianes.push_back(radian);
    }
    return radianes;
}

//Funcion para cordenadas Esfericas a cartecianas
std::tuple<double, double, double> sph2cart(double azimuth, double elevation, double radius) {
    double x = radius * std::cos(elevation) * std::cos(azimuth);
    double y = radius * std::cos(elevation) * std::sin(azimuth);  
    double z = radius * std::sin(elevation);
    return std::make_tuple(x, y, z);
}



std::vector<std::vector<int>> CalcularPuntosCinematicaInversa(const std::vector<std::vector<int>>& result, int si, int slider0Value, py::function callback) {
    std::vector<std::vector<int>> D; 
    int n = 0;
    int l2 = 300; //Modificar l2 es para el tamaño del eslabon ingresarlo 
    int cont = 1;

    
    // Configuracion de puerto serie 
    std::string puertoEspecifico = "\\\\.\\COM14"; // Cambiar al puerto serie específico
    HANDLE serial_fd = abrirPuertoSerie(puertoEspecifico);

    if (serial_fd == INVALID_HANDLE_VALUE) {
        std::cout << "Error al abrir el puerto" << std::endl;
        return D; // Salir de la función si no se puede abrir el puerto
    }
    /////////////////////////////////////////////////////////////////////
    //int x=0;
    //Variable para envirar datos 
    //while(x<=360){

        //auto q1=gradosARadianes(x);
        
        //auto grados1 = radianes_a_grados(q1);
            //std::cout << "Grados de q1:" << grados1 << std::endl;
        


        //auto q2=0.0000;
        //auto q3=0.0000;
        //auto q4=0.0000;

        //std::string q1_str = floatToString(q1, 4);
        //std::string q2_str = floatToString(q2, 4);
        //std::string q3_str = floatToString(q3, 4);
        //std::string q4_str = floatToString(q4, 4);
        //std::string datosAEnviar = q1_str + "a" + q2_str + "b" + q3_str + "c" + q4_str + ">";



        //if (enviarDatosCaracterPorCaracter(serial_fd, datosAEnviar)) {
                //std::cout << "Datos enviados: " << datosAEnviar << std::endl;
                //std::string datosRecibidos = recibirDatosBloqueante(serial_fd);
                //if (!datosRecibidos.empty()) {
                    //std::cout << "Datos recibidos: " << datosRecibidos << std::endl;
                    //py::gil_scoped_acquire acquire;
                    //callback(grados1);
                    //py::gil_scoped_release release;
                //} else {
                    //std::cout << "No se recibieron datos." << std::endl;
                //}
            //} else {
                //std::cerr << "Error al enviar datos." << std::endl;
        //}
        //delaySeconds(3);
        //x=x+15;

    //}

    auto numerototal=si;


    while (n <= si) {
        std::vector<int> temp = result[n]; // Asigna el vector de enteros a un vector temporal
        auto x = slider0Value;
        std::cout << "Contenido del slider que se esta calculando" << slider0Value << ": "<< std::endl;
        
       


        std::cout << "Contenido del vector temporal temp: ";
        for (int elem : temp) {
            std::cout << elem << " ";
        }
        std::cout << std::endl;

        // Imprime los dos primeros elementos del vector temporal
        std::cout << "Contenido de temp en la iteracion Prueba " << n << ": ";
        std::cout << temp[0] << " " << temp[1] <<""<< temp[2]<< std::endl;

        // En esta seccion se calculan los datos de grados a radianes 
        std::vector<double> rads = grados_a_radianes(temp);
        std::cout << "Contenido de rads:" << std::endl;
        for (double radian : rads) {
            std::cout << radian << " "; // Separacion de los datos solo 
        }
        std::cout << std::endl;
        std::cout << "Contenido de rads por separado:" << std::endl;
        std::cout << rads[0] << " " << rads[1] << std::endl;
        std::cout << std::endl;

        // En esta seccion se calculara los datos de la funcion de cordenadas esfericas a cartecianas 
        auto resultado = sph2cart(rads[0], rads[1], temp[2]);
        // En esta seccion se imprime los datos de las cordenadas 
        double px = std::get<0>(resultado);
        double py = std::get<1>(resultado);
        double pz = std::get<2>(resultado);


        std::cout << "Coordenadas cartesianas (px, py, pz): ";
        std::cout << px << ", " << py << ", " << pz << std::endl;

        // En la siguiente seccion se va a calcular la cinematica inversa  
        if (cont <= (x + 1)/2) {
            auto q1 = atan(-px / py) + 3.1416;
            auto qr = radianes_a_grados(q1);
            auto q2 = px * sin(q1) - py * cos(q1) ; 
            auto q3 = pz;
            auto q4 = atan(-q2 / q3);
            std::cout << "Coordenadas cartesianas (q1, q2, q3, q4): ";
            std::cout << q1 << ", " << q2 << ", " << q3 << "," << q4 << std::endl;

            // Conversion a string
            std::string q1_str = floatToString(q1, 4);
            std::string q2_str = floatToString(q2, 4);
            std::string q3_str = floatToString(q3, 4);
            std::string q4_str = floatToString(q4, 4);
            std::string datosAEnviar = q1_str + "a" + q2_str + "b" + q3_str + "c" + q4_str + ">";

            auto grados1 = radianes_a_grados(q1);
            std::cout << "Grados de q1:" << grados1 << std::endl;

            if (enviarDatosCaracterPorCaracter(serial_fd, datosAEnviar)) {
                std::cout << "Datos enviados: " << datosAEnviar << std::endl;

                

                std::string datosRecibidos = recibirDatosBloqueante(serial_fd);
                if (!datosRecibidos.empty()) {
                    std::cout << "Datos recibidos: " << datosRecibidos << std::endl;
                    py::gil_scoped_acquire acquire;
                    callback(grados1,numerototal);
                    py::gil_scoped_release release;
                } else {
                    std::cout << "No se recibieron datos." << std::endl;
                }
            } else {
                std::cerr << "Error al enviar datos." << std::endl;
            }
            delaySeconds(10);
                
        }
        
        if (cont > (x + 1)/2 && cont <=x+2) {
            auto q1 = atan(-px / py);
            auto qr = radianes_a_grados(q1);
            auto q2 = px * sin(q1) - py * cos(q1) ; 
            auto q3 = pz;
            auto q4 = atan(-q2 / q3);
            std::cout << "Coordenadas cartesianas (q1, q2, q3, q4): ";
            std::cout << q1 << ", " << q2 << ", " << q3 << "," << q4 << std::endl;
            
            // Conversion a string
            std::string q1_str = floatToString(q1, 4);
            std::string q2_str = floatToString(q2, 4);
            std::string q3_str = floatToString(q3, 4);
            std::string q4_str = floatToString(q4, 4);
            std::string datosAEnviar = q1_str + "a" + q2_str + "b" + q3_str + "c" + q4_str + ">";

            auto grados1 = radianes_a_grados(q1);
            std::cout << "Grados de q1:" << grados1 << std::endl;

            if (enviarDatosCaracterPorCaracter(serial_fd, datosAEnviar)) {
                std::cout << "Datos enviados: " << datosAEnviar << std::endl;

                std::string datosRecibidos = recibirDatosBloqueante(serial_fd);
                if (!datosRecibidos.empty()) {
                    std::cout << "Datos recibidos: " << datosRecibidos << std::endl;
                    py::gil_scoped_acquire acquire;
                    callback(grados1,numerototal);
                    py::gil_scoped_release release;
                } else {
                    std::cout << "No se recibieron datos." << std::endl;
                }
            } else {
                std::cerr << "Error al enviar datos." << std::endl;
            }
            delaySeconds(10);
        }

        D.push_back(temp); // Agrega el vector temporal a D

        // Imprime el contenido de D en esta iteración
        std::cout << "Contenido de D en la iteracion " << n << ": ";
        for (const auto& elem : D.back()) {
            std::cout << elem << " ";
        }
        std::cout << std::endl;
        

        // Incrementa n para pasar a la siguiente fila
        n++;
        cont++;
        if (cont == x+2) {
            cont = 1;
        }
    }
        
    return D;
}




void procesarDatos(int slider1Value, int slider0Value, int slider2Value,int radioEsfera,py::function callback) {
    int GAltitude, GAsimut, GRoll;
    //int radioEsfera = 299;

    CalcularGrados(slider1Value, slider0Value, slider2Value, GAltitude, GAsimut, GRoll);
    std::cout << "Grados en Altitude: " << GAltitude << std::endl;
    std::cout << "Grados en Asimuth: " << GAsimut << std::endl;
    std::cout << "Grados en Roll:" << GRoll << std::endl;

    //Muestra el arreglo de distribuccion de angulos de Azimuth
    std::vector<int> resultado1 = CalcularArreglo1(GAsimut);
    // Imprimir el contenido del vector devuelto
    std::cout << "Los datos del arreglo 1:" << std::endl;
    for (int value1 : resultado1) {
        std::cout << value1 << " ";
    }
    std::cout << std::endl;

    //Muestra el arreglo de distribuccion de angulos de Altitude
    std::vector<int> resultado2 = CalcularArreglo2(GAltitude);
    // Imprimir el contenido del vector devuelto
    std::cout << "Los datos del arreglo 2:" << std::endl;
    for (int value2 : resultado2) {
        std::cout << value2 << " ";
    }
    std::cout << std::endl;

    //Muestra el arreglo de distribuccion de grados de Roll
    std::vector<int> resultado3 = CalcularArreglo3(GRoll);
    // Imprimir el contenido del vector devuelto
    std::cout << "Los datos del arreglo 3:" << std::endl;
    for (int value3 : resultado3) {
        std::cout << value3 << " ";
    }
    std::cout << std::endl;

    // Esta funcion calcula los puntos de posicionamiento de la circunferencia
    std::vector<std::vector<int>> result = CalcularPuntosMovimiento(resultado1, resultado2, slider2Value);
    printVectorOfVectors(result);

    // Imprimir el resultado
    // Calculo del tamaño del tamaño de la forma en se movera el robot 
    int si = (resultado1.size() - 1) * (resultado2.size() - 1);
    std::cout << si << std::endl;

    ///Imprimir los datos de vector D por separado
    //CalcularPuntosCinematicaInversa(result, si,callback); 
    CalcularPuntosCinematicaInversa(result, si,slider0Value,callback); 
}

void Select_Imagenes_modo_2(int Numero_imagenes,int& NAltitude, int& NAsimut, int& NRoll){
    switch (Numero_imagenes)
    {
    case 36: 
        NAltitude=6;
        NAsimut=6;
        NRoll=3;
        break;
    case 64:
        NAltitude=8;
        NAsimut=8;
        NRoll=3;
        break;    
    case 72:
    std::cout << "Se ha inicializado el el modo de selecion automatico 72" << std::endl;
        break;
    case 128:
        std::cout << "Se ha inicializado el el modo de selecion automatico 128" << std::endl;
        break;
    case 256:
        std::cout << "Se ha inicializado el el modo de selecion automatico 256" << std::endl;
        break;
    case 512:
        std::cout << "Se ha inicializado el el modo de selecion automatico 512" << std::endl;
        break;
    default:
        std::cout << "Error verificar el problema con el dato de entrada" << std::endl;
        break;
    }
}




std::string floatToString(float value, int precision) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

//Funcion de  conversion
double radianes_a_grados(double radianes) {
    return radianes * (180.0 / M_PI);
}


double gradosARadianes(double grados) {
    return grados * M_PI / 180.0;
}







// Función para abrir el puerto serie y configurar la comunicación
HANDLE abrirPuertoSerie(const std::string& puerto) {
    HANDLE serial_fd = CreateFile(puerto.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (serial_fd == INVALID_HANDLE_VALUE) {
        std::cerr << "Error al abrir el puerto serie " << puerto << std::endl;
        return INVALID_HANDLE_VALUE;
    }

    DCB serial_options = { 0 };
    serial_options.DCBlength = sizeof(serial_options);
    if (!GetCommState(serial_fd, &serial_options)) {
        std::cerr << "Error al obtener la configuración del puerto serie." << std::endl;
        CloseHandle(serial_fd);
        return INVALID_HANDLE_VALUE;
    }

    serial_options.BaudRate = CBR_115200;
    serial_options.ByteSize = 8;
    serial_options.StopBits = ONESTOPBIT;
    serial_options.Parity = NOPARITY;
    if (!SetCommState(serial_fd, &serial_options)) {
        std::cerr << "Error al configurar el puerto serie." << std::endl;
        CloseHandle(serial_fd);
        return INVALID_HANDLE_VALUE;
    }

    return serial_fd;
}

// Función para enviar datos por el puerto serie
bool enviarDatos(HANDLE serial_fd, const std::string& datos) {
    DWORD bytes_written;
    if (!WriteFile(serial_fd, datos.c_str(), datos.length(), &bytes_written, NULL)) {
        std::cerr << "Error al enviar datos por el puerto serie." << std::endl;
        return false;
    }
    return true;
}

// Función para recibir datos por el puerto serie
std::string recibirDatos(HANDLE serial_fd) {
    std::string datosRecibidos;
    char buffer[20]; // Buffer para recibir datos
    DWORD bytes_read;

    // Leer datos del puerto serie
    if (ReadFile(serial_fd, buffer, sizeof(buffer), &bytes_read, NULL)) {
        if (bytes_read > 0) {
            datosRecibidos = std::string(buffer, bytes_read);
        }
    } else {
        std::cerr << "Error al recibir datos por el puerto serie." << std::endl;
    }

    return datosRecibidos;
}


bool enviarDatosBloqueante(HANDLE serial_fd, const std::string& datos) {
    DWORD bytes_written;
    if (!WriteFile(serial_fd, datos.c_str(), datos.length(), &bytes_written, NULL)) {
        std::cerr << "Error al enviar datos por el puerto serie." << std::endl;
        return false;
    }
    // Esperar hasta que todos los datos hayan sido transmitidos
    if (!FlushFileBuffers(serial_fd)) {
        std::cerr << "Error al vaciar el buffer del puerto serie." << std::endl;
        return false;
    }
    return true;
}


bool enviarDatosCaracterPorCaracter(HANDLE hSerial, const std::string& datosAEnviar) {
    DWORD bytesWritten;
    for (char c : datosAEnviar) {
        if (!WriteFile(hSerial, &c, 1, &bytesWritten, NULL)) {
            std::cerr << "Error al enviar carácter: " << c << std::endl;
            return false;
        }else{
             std::cout << "Carácter enviado: " << c << " (" << static_cast<int>(c) << ")" << std::endl;
        }
    }
    // Esperar hasta que todos los datos hayan sido transmitidos
    if (!FlushFileBuffers(hSerial)) {
        std::cerr << "Error al vaciar el buffer del puerto serie." << std::endl;
        return false;
    }
    return true;
}

std::string recibirDatosBloqueante(HANDLE serial_fd) {
    std::string datosRecibidos;
    char buffer[100]; // Buffer para recibir datos
    DWORD bytes_read;
    while (true) {
        // Leer datos del puerto serie
        if (ReadFile(serial_fd, buffer, sizeof(buffer), &bytes_read, NULL)) {
            if (bytes_read > 0) {
                datosRecibidos.append(buffer, bytes_read);
                break; // Salir del bucle cuando se reciban datos
            }
        } else {
            std::cerr << "Error al recibir datos por el puerto serie." << std::endl;
            break;
        }
    }
    return datosRecibidos;
}


/// Imprimir los datos del vector Result para comparar en matlab  
void printVectorOfVectors(const std::vector<std::vector<int>>& vec) {
    for (const auto& innerVec : vec) {
        std::cout << '[';
        for (size_t i = 0; i < innerVec.size(); ++i) {
            std::cout << innerVec[i];
            if (i < innerVec.size() - 1) {
                std::cout << ", ";
            }
        }
        std::cout << ']' << std::endl;
    }
}

void delaySeconds(int seconds) {
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
}




PYBIND11_MODULE(prueba_1, m) {
    m.doc() = "sistema de cordenadas para el robot3DSystem"; // optional module docstring

    m.def("addNumbers", &addNumbers, "A function which adds twonumbers");
    m.def("calcular_grados", [](int Altitud, int Asimuth, int Roll) {
        int GAltitude, GAsimut, GRoll;
        CalcularGrados(Altitud, Asimuth, Roll, GAltitude, GAsimut, GRoll);
        // Retorna los resultados como una tupla de Python
        return py::make_tuple(GAltitude, GAsimut, GRoll);
    }, "Calculates degrees for Altitude, Asimuth, and Roll");

    m.def("CalcularArreglo1", &CalcularArreglo1, "Función para calcular arreglo 1");
    m.def("CalcularArreglo2", &CalcularArreglo2, "Función para calcular arreglo 2");
    m.def("CalcularArreglo3", &CalcularArreglo3, "Función para calcular arreglo 3");
    m.def("CalcularPuntosMovimiento", &CalcularPuntosMovimiento, "Función para calcular puntos de movimiento");
    m.def("grados_a_radianes", &grados_a_radianes, "Funcion de que convierte grados a radianes");
    m.def("sph2cart", &sph2cart, "Funcion que convierte cordendas esfericas a cartesianas");
    
    m.def("procesarDatos", &procesarDatos,"Esta funcion simplifica la llamadas en el programa principal");

    m.def("CalcularPuntosCinematicaInversa", &CalcularPuntosCinematicaInversa, "Funcion que calcula la cinematica inversa del robot3DSystem");


    m.def("Select_Imagenes_modo_2", [](int Numero_imagenes) {
        int NAltitude, NAsimut, NRoll;
        Select_Imagenes_modo_2(Numero_imagenes, NAltitude, NAsimut, NRoll);
        // Retorna los resultados como una tupla de Python
        return py::make_tuple(NAltitude, NAsimut, NRoll);
    }, "Calculates el numero de imagenes a regresar");

}


