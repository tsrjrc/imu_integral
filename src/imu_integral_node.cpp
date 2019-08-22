#include <ros/ros.h>
#include <fstream>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

using std::sin;
using std::cos;
using std::atan2;


//imu时间戳大于当前点云时间戳的位置
int imuPointerFront = 0;
//imu最新收到的点在数组中的位置
int imuPointerLast = -1;
//imu循环队列长度
const int imuQueLength = 200;



//IMU信息
double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};

float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};

float imuVeloX[imuQueLength] = {0};
float imuVeloY[imuQueLength] = {0};
float imuVeloZ[imuQueLength] = {0};

float imuShiftX[imuQueLength] = {0};
float imuShiftY[imuQueLength] = {0};
float imuShiftZ[imuQueLength] = {0};


ros::Publisher pubdatadisplay;

//积分速度与位移
std::ofstream anga("/home/qjny/data/anga.txt");
std::ofstream angv("/home/qjny/data/angv.txt");
std::ofstream acc("/home/qjny/data/acc.txt");
std::ofstream vel("/home/qjny/data/vel.txt");
void AccumulateIMUShift()
{
  float roll = imuRoll[imuPointerLast];
  float pitch = imuPitch[imuPointerLast];
  float yaw = imuYaw[imuPointerLast];
  float accX = imuAccX[imuPointerLast];
  float accY = imuAccY[imuPointerLast];
  float accZ = imuAccZ[imuPointerLast];

  //将当前时刻的加速度值绕交换过的ZXY固定轴（原XYZ）分别旋转(roll, pitch, yaw)角，转换得到世界坐标系下的加速度值(right hand rule)
  //绕z轴旋转(roll)
  float x1 = cos(roll) * accX - sin(roll) * accY;
  float y1 = sin(roll) * accX + cos(roll) * accY;
  float z1 = accZ;
  //绕x轴旋转(pitch)
  float x2 = x1;
  float y2 = cos(pitch) * y1 - sin(pitch) * z1;
  float z2 = sin(pitch) * y1 + cos(pitch) * z1;
  //绕y轴旋转(yaw)
  accX = cos(yaw) * x2 + sin(yaw) * z2;
  accY = y2;
  accZ = -sin(yaw) * x2 + cos(yaw) * z2;

  //上一个imu点
  int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
  //上一个点到当前点所经历的时间，即计算imu测量周期
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
  //要求imu的频率至少比lidar高，这样的imu信息才使用，后面校正也才有意义
  if (timeDiff < 0.1) {//（隐含从静止开始运动）
    //求每个imu时间点的位移与速度,两点之间视为匀加速直线运动
    imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff 
                              + accX * timeDiff * timeDiff / 2;
    imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff 
                              + accY * timeDiff * timeDiff / 2;
    imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff 
                              + accZ * timeDiff * timeDiff / 2;

    imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
    imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
    imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
    vel  << ros::Time::now() << "\t" << imuVeloX[imuPointerLast] << "\t" << imuVeloY[imuPointerLast] << "\t" << imuVeloZ[imuPointerLast] << std::endl;
  }

//  ROS_INFO("imuShiftX[%d]:%f,imuShiftY[%d]:%f,imuShiftZ[%d]:%f",
//           imuPointerLast,imuShiftX[imuPointerLast],
//           imuPointerLast,imuShiftY[imuPointerLast],
//           imuPointerLast,imuShiftZ[imuPointerLast]);
//  ROS_INFO("timeDiff:%f",timeDiff);
//  geometry_msgs::PoseStamped display_imu;
//  display_imu.header.stamp = ros::Time::now();
//  display_imu.pose.position.x = imuVeloX[imuPointerLast];
//  display_imu.pose.position.y = imuVeloY[imuPointerLast];
//  display_imu.pose.position.z = imuVeloZ[imuPointerLast];
//  pubdatadisplay.publish(display_imu);

}

//接收imu消息，imu坐标系为x轴向前，y轴向右，z轴向上的右手坐标系
void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  //convert Quaternion msg to Quaternion
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  //This will get the roll pitch and yaw from the matrix about fixed axes X, Y, Z respectively. That's R = Rz(yaw)*Ry(pitch)*Rx(roll).
  //Here roll pitch yaw is in the global frame
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  //减去重力的影响,求出xyz方向的加速度实际值，并进行坐标轴交换，统一到z轴向前,x轴向左的右手坐标系, 交换过后RPY对应fixed axes ZXY(RPY---ZXY)。Now R = Ry(yaw)*Rx(pitch)*Rz(roll).
  float accX = imuIn->linear_acceleration.x + sin(pitch) * 9.81;
  float accY = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
  float accZ = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;

  //循环移位效果，形成环形数组
  imuPointerLast = (imuPointerLast + 1) % imuQueLength;

  imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
  imuRoll[imuPointerLast] = roll;
  imuPitch[imuPointerLast] = pitch;
  imuYaw[imuPointerLast] = yaw;
  imuAccX[imuPointerLast] = accX;
  imuAccY[imuPointerLast] = accY;
  imuAccZ[imuPointerLast] = accZ;
//  ROS_INFO("%f",imuTime[imuPointerLast]);
  anga  << ros::Time::now() << "\t" << roll << "\t" << pitch << "\t" << yaw << std::endl;
  acc  << ros::Time::now() << "\t" << imuIn->linear_acceleration.x << "\t" << imuIn->linear_acceleration.y << "\t" << imuIn->linear_acceleration.z << std::endl;
  angv  << ros::Time::now() << "\t" << imuIn->angular_velocity.x <<"\t" << imuIn->angular_velocity.y << "\t" << imuIn->angular_velocity.z << std::endl;

  AccumulateIMUShift();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;


  anga << "time\t" << "roll" << "\t" << "pitch" << "\t" << "yaw" << std::endl;
  acc  << "time\t"  << "linear_acceleration_x" << "\t" << "linear_acceleration_y" << "\t" << "linear_acceleration_z" << std::endl;
  angv << "time\t" << "angular_velocity_x" <<"\t" << "angular_velocity_y" << "\t" << "angular_velocity_z" << std::endl;
  vel  << "time\t" << "VeloX" << "\t" << "VeloY" << "\t" << "VeloZ" << std::endl;
  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);

  pubdatadisplay = nh.advertise<geometry_msgs::PoseStamped>("/display", 10);

  ros::spin();

  return 0;
}


