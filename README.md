# Tarea de Admisión: Percepción - Driverless (ARUS)

Este repositorio contiene la solución a las tareas de admisión para el departamento de Percepción. El objetivo principal es el filtrado de una nube de puntos (LiDAR) para eliminar el suelo y aislar los conos del circuito.

## Tarea 1: Razonamiento del Algoritmo

Necesitamos eliminar el suelo de la nube de puntos sin perder los datos de los conos; para ello, debemos buscar una forma de diferenciar el suelo. En la nube de puntos, el suelo es el plano que contiene más puntos. Por lo tanto, solo debemos encontrar un algoritmo capaz de hallar este plano.

### Ajuste del plano
Para lograrlo, debemos localizar tres puntos (X, Y, Z) incluidos en ese plano. La forma más óptima es ir iterando de tres en tres puntos hasta encontrar el plano con mayor número de puntos contenidos.

### Tratamiento de rugosidad
Por otro lado, nos encontramos con el problema del ruido y los baches. Para solucionarlo, introduciremos un umbral (threshold) de 20 cm al plano.

### Optimización del algoritmo
Además, podemos optimizar este algoritmo gracias al dato del ring y la intensidad con la que se devuelve el punto:
* Podemos hacer uso de los rings, debido a que los de menor índice siempre impactan en el suelo. Por lo tanto, para no tener que iterar entre todos los puntos, restringiremos la búsqueda de los puntos entre estos rings.
* Podemos hacer uso de la intensidad, debido a que el asfalto tendrá una intensidad menor a la del plástico de los conos.

### RANSAC
Para no reinventar la rueda, utilizaremos la librería PCL (Point Cloud Library), que incluye RANSAC, un algoritmo iterativo utilizado para estimar los parámetros de un modelo matemático, permitiéndonos centrar nuestros esfuerzos en la lógica de filtrado y optimización de los parámetros


---

## Tarea 2: Implementación (C++)

La implementación se ha realizado en **C++** utilizando **Ubuntu 22.04**.

### Requisitos
* **PCL** (Point Cloud Library).
* **CMake** para la compilación.
* **CloudCompare** para la visualización de resultados.

## Compilar el proyecto

``` bash
mkdir build && cd build
cmake ..
make
./eliminar_suelo