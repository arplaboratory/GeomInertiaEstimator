#ifndef V_TOOLS_H
#define V_TOOLS_H

#include <ros/ros.h>
#include <strstream>
#include <string.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Eigen>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>

using namespace std;

#define doPrint true
typedef float flt;
typedef Eigen::Quaternion<flt> quat;
typedef Eigen::Matrix<flt,4,1> vec6;
typedef Eigen::Matrix<flt,4,1> vec4;
typedef Eigen::Matrix<flt,3,1> vec3;
typedef Eigen::Matrix<flt,2,1> vec2;
typedef Eigen::Matrix<flt,3,3> mat3;
typedef Eigen::Matrix<flt,6,6> mat6;
typedef Eigen::Matrix<flt,9,9> mat9;
typedef Eigen::Matrix<flt,3,1> angleA;

template <typename T>
void print(const string &s, const T &t) {
  if (doPrint) {
    stringstream ss;
    ss << s << "\n" << t;
    ROS_INFO(ss.str().c_str());
  }
}
void print(const string &s, const quat &q);
void print(const string &s);
void print(void);

template <typename T>
void warn(const string &s, const T &t) {
  if (1) {
    stringstream ss;
    ss << s << " " << t;
    ROS_WARN(ss.str().c_str());
  }
}
void warn(const string &s);
void warn(const char* ch);

template <typename T>
vec3 msg2vec(T const &t);
geometry_msgs::Vector3 vec2msg(vec3 const &t);
geometry_msgs::Vector3 ivec2msg(vec3 const &t);
quat msg2quat(geometry_msgs::Quaternion const &q);
geometry_msgs::Quaternion quat2msg(quat const &q);
geometry_msgs::Quaternion iquat2msg(quat const &q);

template <typename T>
void getCovInMsg(const T &covariance, Eigen::Matrix<flt,6,6> &matrix) {
  int dim = 6;
  for (int idx = 0; idx < dim*dim; idx++)
          matrix(idx%dim,idx/dim) = covariance[idx];
}

template <typename T>
void getCovInMsg(const T &covariance, Eigen::Matrix<flt,3,3> &matrix) {
  int dim = 3;
  for (int idx = 0; idx < dim*dim; idx++)
          matrix(idx%dim,idx/dim) = covariance[idx];
}

template <typename T>
void setCovInMsg(T &covariance, const Eigen::Matrix<flt,6,6> &matrix) {
  int dim = 6;
  for (int idx = 0; idx < dim*dim; idx++)
          covariance[idx] = matrix(idx%dim,idx/dim);
}


// quaternion functions
quat qExp(const angleA& aA);
angleA qLog(const quat& q);

quat qBoxPlus(const quat& q, const angleA& delta);
angleA qBoxMinus(const quat& q1, const quat& q2);

//quat qMean(const quat* qList);
//quat qMean(const quat* qList, const flt* weights);
//vec4 qMean(const Eigen::Matrix<flt,4,Eigen::Dynamic> &qMatrix);
vec4 qMean(const Eigen::Matrix<flt,4,Eigen::Dynamic> &qMatrix,
           const Eigen::Matrix<flt,Eigen::Dynamic,1> &weights);
quat qMean(const quat *qList,
           const Eigen::Matrix<flt,Eigen::Dynamic,1> &weights);
//vec4 qMean(const Eigen::Matrix<flt,4,Eigen::Dynamic> &qMatrix, const flt *weights);

vec3 qRotateVec(const quat& q, const vec3& vec);

#endif // V_TOOLS_H
