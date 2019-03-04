#include "v_tools.h"

void print(const string &s) {
  if (doPrint) {
    ROS_INFO(s.c_str());
  }
}

void print()
{
  static int i;
  i++;
  if (doPrint) {
    ROS_INFO("position: %d", i);
  }
}

void warn(const string &s) {
  if (1) {
    ROS_WARN(s.c_str());
  }
}

void warn(const char* ch) {
  if (1) {
    ROS_WARN(ch);
  }
}

template <typename T>
vec3 msg2vec(T const &t) {
  return vec3(t.x,t.y,t.z);
}

geometry_msgs::Vector3 vec2msg(vec3 const &t) {
  geometry_msgs::Vector3 msg;
  msg.x = t(0);
  msg.y = t(1);
  msg.z = t(2);
  return msg;
}

geometry_msgs::Vector3 ivec2msg(vec3 const &t) {
  geometry_msgs::Vector3 msg;
  msg.x = t(0);
  msg.y = -t(1);
  msg.z = -t(2);
  return msg;
}

quat msg2quat(const geometry_msgs::Quaternion &q) {
  return quat(q.w,q.x,q.y,q.z);
}

geometry_msgs::Quaternion quat2msg(quat const &q) {
  geometry_msgs::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

geometry_msgs::Quaternion iquat2msg(quat const &q) {
  geometry_msgs::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = -q.y();
  msg.z = -q.z();
  return msg;
}

quat qExp(const angleA &aA)
{
  quat q;
  flt phi(aA.norm());
  if (phi == 0)
    q = quat(1.0,0.0,0.0,0.0);
  else
  {
    vec3 u(aA.normalized());
    q.w() = cos(phi);
    q.x() = sin(phi)*u[0];
    q.y() = sin(phi)*u[1];
    q.z() = sin(phi)*u[2];
  }
  return q;
}

angleA qLog(const quat &q)
{
  vec3 qv(q.x(),q.y(),q.z());
  flt qvNorm = qv.norm();
  flt phi(atan2(qvNorm,q.w()));
  vec3 u;
  if (phi == 0)
    u = vec3(0.0,0.0,1.0);
  else if (phi < 1e-6)
    u = qv/q.w()*(1-qvNorm*qvNorm/(3*q.w()*q.w()));
  else
    u = qv.normalized();

  return angleA(phi*u);
}

void print(const string &s, const quat &q)
{
  if (doPrint) {
    stringstream ss;
    ss << s << "\n" << q.w() << "  " << q.x() << "  " << q.y() << "  " << q.z();
    ROS_INFO(ss.str().c_str());
  }
}

quat qBoxPlus(const quat &q, const angleA &delta)
{
  return quat(q*qExp(delta/2.0));
}

angleA qBoxMinus(const quat &q1, const quat &q2)
{
  return angleA(2.0*qLog(q2.inverse()*q1));
}

quat qMean(const quat *qList, const Eigen::Matrix<flt,Eigen::Dynamic,1> &weights)
{
  const int n = weights.rows();
  quat mu = qList[0];
  angleA error[n];
  int k = 1;
  angleA meanError(0.0,0.0,0.0);
  while (true) {
    meanError = angleA(0.0,0.0,0.0);
    for (int i = 0; i < n; i++)
    {
      error[i] = qBoxMinus(qList[i],mu);
    }
    for (int i = 0; i < n; i++)
    {
      meanError += error[i];
    }
    meanError = meanError/flt(n);
    mu = qBoxPlus(mu,meanError);
    if (k>2) {
      //print("mean quat calc too long, done steps",k);
      break;
    }
    if (meanError.norm() < numeric_limits<flt>::epsilon()*1.0e3)
      break;
    k++;
  }
  return mu;
}

vec4 qMean(const Eigen::Matrix<flt,4,Eigen::Dynamic> &qMatrix, const Eigen::Matrix<flt,Eigen::Dynamic,1> &weights)
{
  quat qList[qMatrix.cols()];
  for (int i=0; i<qMatrix.cols(); i++)
  {
    qList[i] = quat(qMatrix(0,i),qMatrix(1,i),qMatrix(2,i),qMatrix(3,i));
  }
  quat qMu = qMean(qList,weights);
  return vec4(qMu.w(),qMu.x(),qMu.y(),qMu.z());
}

vec3 qRotateVec(const quat &q, const vec3 &vec)
{
  quat qvec;
  qvec.x() = vec(0);
  qvec.y() = vec(1);
  qvec.z() = vec(2);
  qvec = q*qvec*q.inverse();
  return vec3(qvec.x(),qvec.y(),qvec.z());
}
