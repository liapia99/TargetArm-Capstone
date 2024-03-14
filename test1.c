#include <stdio.h>
#include <stdlib.h>
#include <rplidar.h>

int main() {
  // Create a new RPLidar object
  rplidar_t *lidar = rplidar_create("/dev/ttyUSB0");

  // Check if the lidar object was created successfully
  if (lidar == NULL) {
    printf("Failed to create RPLidar object\n");
    return 1;
  }

  // Start the lidar scan
  rplidar_start_scan(lidar);

  // Get the lidar scan data
  rplidar_scan_data_t *scan_data = rplidar_get_scan_data(lidar);

  // Check if the scan data was obtained successfully
  if (scan_data == NULL) {
    printf("Failed to get scan data\n");
    return 1;
  }

  // Print the scan data
  for (int i = 0; i < scan_data->count; i++) {
    printf("Distance: %f, Angle: %f\n", scan_data->distances[i], scan_data->angles[i]);
  }

  // Stop the lidar scan
  rplidar_stop_scan(lidar);

  // Destroy the lidar object
  rplidar_destroy(lidar);

  return 0;
}
