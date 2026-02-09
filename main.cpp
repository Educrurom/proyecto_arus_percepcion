#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

int main(int argc, char** argv) {
    


    // ---- 1. Contenedores de nubes de puntos ----
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);



    // ---- 2. Carga del archivo .pcd ----
    if (pcl::io::loadPCDFile<pcl::PointXYZI>("../saved_pointcloud.pcd", *cloud) == -1) {
        std::cerr << "Error: No se ha podido cargar el archivo .pcd" << std::endl;
        return (-1);
    }



    // ---- 3. Configuración de RANSAC para detección de planos ----
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);      
    seg.setModelType(pcl::SACMODEL_PLANE);      // Buscamos una geometría plana
    seg.setMethodType(pcl::SAC_RANSAC);         // Algoritmo contra ruido (outliers)
    
    seg.setMaxIterations(2000);                 // Establecemos intentos para encontrar el plano
    seg.setDistanceThreshold(0.5);              // Establecemos umbral de 0,5



    // ---- 4. Segmentación del suelo ----
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        std::cerr << "Error: No se encontró un plano de suelo en la escena." << std::endl;
        return (-1);
    }



    // ---- 5. Extracción de los puntos (Borrar el suelo) ----
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  
    extract.filter(*cloud_filtered);



    // ---- 6. Guardado del resultado ----
    pcl::io::savePCDFileASCII("conos_aislados.pcd", *cloud_filtered);
    
    return (0);
}