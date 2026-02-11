#define PCL_NO_PRECOMPILE

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <cmath>

// Estructura
struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Traductor
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Uso: ./eliminar_suelo input.pcd" << std::endl;
        return -1;
    }

    // Cargar nube
    pcl::PointCloud<PointXYZIRT>::Ptr cloud(new pcl::PointCloud<PointXYZIRT>); 
    
    if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
    {
        std::cerr << "Error al leer el archivo PCD" << std::endl;
        return -1;
    }

    // Subnube optimizada
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_rings(new pcl::PointCloud<PointXYZIRT>);
    float intensidad_max = 40.0f;
    uint16_t umbral_ring_inferiores = 10;

    for (const auto& p : cloud->points)
    {
        if (p.ring <= umbral_ring_inferiores and p.intensity< intensidad_max) 
            cloud_rings->push_back(p);
    }

    // RANSAC
    pcl::SACSegmentation<PointXYZIRT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); 
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.2);
    seg.setInputCloud(cloud_rings);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty())
    {
        std::cerr << "No se pudo encontrar un plano del suelo" << std::endl;
        return -1;
    }

    // Coeficientes: Ax + By + Cz + D = 0
    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];
    float norm = std::sqrt(a*a + b*b + c*c);
    
    // FILTRADO FINAL
    pcl::PointCloud<PointXYZIRT>::Ptr objetos(new pcl::PointCloud<PointXYZIRT>);

    float h = 0.4f; 
    float intensidad_suelo = 35.0f;

    for (const auto& p : cloud->points)
    {   // Calculo de distancia del punto respecto al plano
        float dist = (a*p.x + b*p.y + c*p.z + d) / norm;

        // El punto debe estar por encima de la intensidad promedio del suelo y por encima del umbral del plano.
        if (dist >= h && p.intensity > intensidad_suelo) 
        {
            objetos->push_back(p);
        }
    }

    // 5. Guardar resultados
    pcl::io::savePCDFileBinary("objetos.pcd", *objetos);

    std::cout << "Original: " << cloud->size() << " | Objetos: " << objetos->size() << std::endl;
    return 0;
}