#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

int main(int argc, char** argv) {
    
    // 1. Contenedores de nubes de puntos
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    // 2. Carga del archivo .pcd
    // El path "../" asume que el ejecutable está en la carpeta /build
    if (pcl::io::loadPCDFile<pcl::PointXYZI>("../saved_pointcloud.pcd", *cloud) == -1) {
        std::cerr << "Error: No se ha podido cargar el archivo .pcd" << std::endl;
        return (-1);
    }

    // 3. Configuración de RANSAC para detección de planos
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);      // Ajuste fino del plano mediante mínimos cuadrados
    seg.setModelType(pcl::SACMODEL_PLANE);  // Buscamos una geometría plana (suelo)
    seg.setMethodType(pcl::SAC_RANSAC);     // Algoritmo robusto contra ruido (outliers)
    
    // --- PARÁMETROS DE AJUSTE ---
    seg.setMaxIterations(2000);    // Aumentamos intentos para asegurar encontrar el plano dominante
    seg.setDistanceThreshold(0.5); // Umbral de 30cm: captura baches y evita dejar restos de asfalto

    // 4. Segmentación del suelo
    // Aplicamos sobre la nube completa para mantener la integridad de los índices
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        std::cerr << "Error: No se encontró un plano de suelo en la escena." << std::endl;
        return (-1);
    }

    // 5. Extracción de los puntos (Borrar el suelo)
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  // "true" elimina los inliers (suelo) y mantiene el resto (conos)
    extract.filter(*cloud_filtered);

    // 6. Guardado del resultado
    pcl::io::savePCDFileASCII("conos_aislados.pcd", *cloud_filtered);
    
    // Resumen por consola
    std::cout << ">>> Suelo eliminado correctamente." << std::endl;
    std::cout << ">>> Puntos originales: " << cloud->size() << std::endl;
    std::cout << ">>> Puntos restantes:   " << cloud_filtered->size() << std::endl;
    std::cout << ">>> Resultado guardado en: build/conos_aislados.pcd" << std::endl;

    return (0);
}