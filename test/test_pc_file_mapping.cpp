#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <elevmap/elevation_map.h>

int main()
{
    const std::array<std::string, 7> map_names = {"elevation", "elevation_filtered", "uncertainty", "slope", "step_height", "roughness", "traversability"};
    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();

    // Create temp directory if it doesn't exist
    std::filesystem::create_directories("temp");

    // Load point cloud from file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string filename = "assets/room.ply";

    if (pcl::io::loadPLYFile(filename, *cloud) == -1)
    {
        std::cerr << "Failed to load point cloud from " << filename << std::endl;
        return -1;
    }

    std::cout << "Loaded point cloud with " << cloud->points.size() << " points" << std::endl;

    float resolution = 0.1;
    float length_x = 10;
    float length_y = 10;
    std::size_t max_top_points_in_grid = 5;
    std::string method = "direct";
    std::string elevation_map_filter_type = "gaussian";
    float max_height = 1.5;

    // Direct method parameters
    float slope_weight = 0.33;
    float step_height_weight = 0.33;
    float roughness_weight = 0.34;
    float slope_critical = 0.3;
    float step_height_critical = 0.1;
    float roughness_critical = 0.05;

    // Gaussian process parameters
    float l = 2.0;
    float sigma_f = 1.0;
    float sigma_n = 0.2;

    ElevationMap elevation_map(
        method,
        length_x,
        length_y,
        resolution,
        max_top_points_in_grid,
        elevation_map_filter_type,
        max_height,
        slope_weight,
        step_height_weight,
        roughness_weight,
        slope_critical,
        step_height_critical,
        roughness_critical,
        l,
        sigma_f,
        sigma_n);

    // Update the map with the point cloud
    uint8_t repeat_times = 10;
    auto processing_start = std::chrono::high_resolution_clock::now();
    for (uint8_t i = 0; i < repeat_times; i++)
    {
        elevation_map.update(cloud);
    }
    auto processing_end = std::chrono::high_resolution_clock::now();

    const std::vector<Eigen::MatrixXf> &maps = elevation_map.getMaps();

    // Export all maps as binary files
    for (std::size_t i = 0; i < maps.size(); i++)
    {
        const Eigen::MatrixXf &map = maps[i];
        const std::string &map_name = map_names[i];

        std::string filepath = "temp/binary/maps_direct_static/" + map_name + ".bin";

        // Create directory if it doesn't exist
        std::filesystem::create_directories("temp/binary/maps_direct_static/");

        std::ofstream file(filepath, std::ios::binary);

        if (file.is_open())
        {
            // Write matrix dimensions
            int rows = map.rows();
            int cols = map.cols();
            file.write(reinterpret_cast<const char *>(&rows), sizeof(int));
            file.write(reinterpret_cast<const char *>(&cols), sizeof(int));

            // Write matrix data
            file.write(reinterpret_cast<const char *>(map.data()), sizeof(float) * rows * cols);

            file.close();
            std::cout << "Exported " << map_name << " map to " << filepath << std::endl;
        }
        else
        {
            std::cerr << "Failed to open file for writing: " << filepath << std::endl;
        }
    }

    // Print timing information
    auto end_time = std::chrono::high_resolution_clock::now();
    auto processing_duration = std::chrono::duration_cast<std::chrono::milliseconds>(processing_end - processing_start);
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    std::cout << "Point cloud processing time: " << processing_duration.count() / repeat_times << " ms" << std::endl;
    std::cout << "Total execution time: " << total_duration.count() << " ms" << std::endl;

    return 0;
}