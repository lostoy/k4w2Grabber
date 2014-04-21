#include <IMUReader.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>

pcl::visualization::PCLVisualizer viewer("cloud");
IMUReader imureader;
IMUReader::SensorData last_data, data;

int main()
{
	
	imureader.init("COM5");
	imureader.calibration(1000);
	
	viewer.addCoordinateSystem();
		for (int i = 0;!viewer.wasStopped(); i++)
		{


			if (imureader.getDataOnce(data))
			{
				if (i != 0)
				{
					//update the pose with AHRS algorithm
					imureader.ahrs_.SamplePeriod = (data.time - last_data.time) / 1e6;
					imureader.ahrs_.Update(data.gyro[0], data.gyro[1], data.gyro[2], data.acce[0], data.acce[1], data.acce[2]);
					
					//visualize the pose with a cube
					pcl::ModelCoefficients cof;
					cof.values.resize(10);

					cof.values[0] = cof.values[1] = cof.values[2] = 0;
					cof.values[3] = imureader.ahrs_.Quaternion[2];
					cof.values[4] = -imureader.ahrs_.Quaternion[0];
					cof.values[5] = imureader.ahrs_.Quaternion[1];
					cof.values[6] = imureader.ahrs_.Quaternion[3];
					cof.values[7] = 1;
					cof.values[8] = 1;
					cof.values[9] = 1;
					
					viewer.removeAllShapes();
					viewer.addCube(cof, "cube");

					//visualize the pose with a line
					pcl::ModelCoefficients line_coeff;
					line_coeff.values.resize(6);    // We need 6 values
					line_coeff.values[0] = 0;
					line_coeff.values[1] = 0;
					line_coeff.values[2] = 0;
					line_coeff.values[3] = imureader.ahrs_.Quaternion[2] * imureader.ahrs_.Quaternion[3]*10;
					line_coeff.values[4] = imureader.ahrs_.Quaternion[0] * imureader.ahrs_.Quaternion[3]*10;
					line_coeff.values[5] = imureader.ahrs_.Quaternion[1] * imureader.ahrs_.Quaternion[3]*10;
					viewer.addLine(line_coeff);

							

				}
				last_data = data;
			}
			viewer.spinOnce(100);
		}
		
		
	imureader.stop();
	return 0;
}