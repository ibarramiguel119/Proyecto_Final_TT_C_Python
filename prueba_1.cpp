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


namespace py = pybind11;


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    GRoll=90/H[2];
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
    for (int i = 0; i <= 90; i += GRoll) {
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



std::vector<std::vector<int>> CalcularPuntosCinematicaInversa(const std::vector<std::vector<int>>& result, int si) {
    std::vector<std::vector<int>> D; 
    int n = 0;
    int l2=300; //Modificar l2 es para el tamaño del eslabon ingresarlo 
    int cont=1;
    
    while (n <= si) {
        std::vector<int> temp = result[n]; // Asigna el vector de enteros a un vector temporal

        std::cout << "Contenido del vector temporal temp: ";
        for (int elem : temp) {
            std::cout << elem << " ";
        }
        std::cout << std::endl;

       // Imprime los dos primeros elementos del vector temporal
        std::cout << "Contenido de temp en la iteracion Prueba " << n << ": ";
        std::cout << temp[0] << " " << temp[1] << std::endl;


        // En esta seccion se calculan los datos de grados a radianes 
        std::vector<double> rads=grados_a_radianes(temp);
        std::cout << "Contenido de rads:" << std::endl;
        for (double radian : rads) {
            std::cout << radian << " ";// Separacion de los datos solo 
           
        }
        //Solo es prueba para inprimir los datos del vector rads 
        std::cout << std::endl;
        std::cout << "Contenido de rads por separado:" << std::endl;
        std::cout << rads[0] << " " << rads[1] << std::endl;
        std::cout << std::endl;

        //En esta seccion se calculara los datos de la funcion de cordenadas esfericas a cartecianas 
        auto resultado = sph2cart(rads[0],rads[1],temp[2]);
        //En esta seccion se imprime los datos de las cordenadas 
        double px = std::get<0>(resultado);
        double py = std::get<1>(resultado);
        double pz = std::get<2>(resultado);
        std::cout << "Coordenadas cartesianas (px, py, pz): ";
        std::cout << px << ", " << py << ", " << pz << std::endl;

        //En la siguiente seccion se va a calcular la cinematica inversa  
        //Para q1 se tiene que:

        //Poner el algoritmo  del if para que la mitadad del movimiento funcione correctamente///////////////////////////////////////////

        if (cont<=4)
        {
            auto q1=atan(-px/py)+3.1416;
            auto q2=px*sin(q1)-py*cos(q1)+l2; 
            //Para q3 se tiene que:
            auto q3=pz;
            //para q4 se tiene que:
            auto q4=atan(-q2/q3);
            std::cout << "Coordenadas cartesianas (q1,q2,q3,q4): ";
            std::cout << q1 << ", " << q2 << ", " << q3 <<","<<q4<< std::endl;
        }
        
        if (cont > 4 && cont <= 8)
        {
            auto q1=atan(-px/py);
            auto q2=px*sin(q1)-py*cos(q1)+l2; 
            //Para q3 se tiene que:
            auto q3=pz;
            //para q4 se tiene que:
            auto q4=atan(-q2/q3);
            std::cout << "Coordenadas cartesianas (q1,q2,q3,q4): ";
            std::cout << q1 << ", " << q2 << ", " << q3 <<","<<q4<< std::endl;
        }
        
        //Para q2 se tiene que:
      
        //Impresion de las cordenadas de cada una de las cordenadas de la cinematica inversa 
       

        D.push_back(temp); // Agrega el vector temporal a D

        // Imprime el contenido de D en esta iteración
        std::cout << "Contenido de D en la iteracion " << n << ": ";
        for (const auto& elem : D.back()) {
            std::cout << elem << " ";
        }
        std::cout << std::endl;
        // Incrementa n para pasar a la siguiente fila
        n++;
        cont=cont+1;
        if (cont==8)
        {
            cont=1;
        }
        
    }
        
    return D;
}


void procesarDatos(int slider1Value, int slider0Value, int slider2Value) {
    int GAltitude, GAsimut, GRoll;
    int radioEsfera = 299;

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

    // Imprimir el resultado
    // Calculo del tamaño del tamaño de la forma en se movera el robot 
    int si = (resultado1.size() - 1) * (resultado2.size() - 1);
    std::cout << si << std::endl;

    ///Imprimir los datos de vector D por separado
    CalcularPuntosCinematicaInversa(result, si);
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


