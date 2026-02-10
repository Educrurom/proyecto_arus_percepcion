# Tarea de Admisión: Percepción - Driverless (ARUS)

Este repositorio contiene la solución a las tareas de admisión para el departamento de Percepción. El objetivo principal es el filtrado de una nube de puntos (LiDAR) para eliminar el suelo y aislar los conos del circuito.

## Tarea 1: Razonamiento del Algoritmo

Necesitamos eliminar el suelo de la nube de puntos sin perder los datos de los conos; para ello, debemos buscar una forma de diferenciar el suelo. En la nube de puntos, el suelo es el plano que contiene más puntos. Por lo tanto, solo debemos encontrar un algoritmo capaz de hallar este plano.

### Ajuste del plano
Para lograrlo, debemos localizar tres puntos (X, Y, Z) incluidos en ese plano. La forma más óptima es ir iterando de tres en tres puntos hasta encontrar el plano con mayor número de puntos contenidos.

### Tratamiento de rugosidad
Por otro lado, nos encontramos con el problema del ruido y los baches. Para solucionarlo, introduciremos un umbral (threshold).

### Optimización del algoritmo
Además, podemos optimizar este algoritmo gracias al dato del ring y la intensidad con la que se devuelve el punto:
* Podemos hacer uso de los rings, debido a que los de menor índice siempre impactan en el suelo. Por lo tanto, para no tener que iterar entre todos los puntos, restringiremos la búsqueda de los puntos entre estos rings.
* Podemos hacer uso de la intensidad con la que se devuelve el punto, debido a que el suelo devolverá menos intensidad que el plastico de los conos. Por lo tanto, restringiremos la búsqueda de los puntos entre los de menor intensidad.

### RANSAC
Para no reinventar la rueda, utilizaremos la librería PCL (Point Cloud Library), que incluye RANSAC, un algoritmo iterativo utilizado para estimar los parámetros de un modelo matemático, permitiéndonos centrar nuestros esfuerzos en la lógica de filtrado y optimización de los parámetros

### Filtrado final
Una vez obtenido el plano del suelo, se realizará el filtrado final utilizando tanto la distancia de los puntos al plano como la intensidad reflejada. Los puntos que se encuentren fuera del umbral del plano y su intensidad sea superior a la media de la intensidad del suelo, será tratado como un objeto, por lo que se mantendrán en la nube de puntos resultante.

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
./eliminar_suelo ../saved_pointcloud.pcd
```

---

## Resultados
| Antes del procesamiento | Después del procesamiento |
|-------------------------|---------------------------|
| ![Antes](images/antes.png) | ![Después](images/resultado.png) |

