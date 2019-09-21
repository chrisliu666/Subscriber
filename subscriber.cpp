#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
using namespace sensor_msgs;
using namespace std;
pcl::PointCloud<pcl::PointXYZ>::Ptr msg1 (new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PCLPointCloud2::Ptr last_msg (new pcl::PCLPointCloud2);
pcl::PointCloud<pcl::PointXYZ>::Ptr last_msg1 (new pcl::PointCloud<pcl::PointXYZ>);

//
int flag=0;
int tcount=0;
int ncount=0;
Eigen::Quaterniond q;
Eigen::Vector3d t;
Eigen::Matrix3d R;
Eigen::Vector3d p_rotated;
//tf::TransformListener  listener;


void callback(sensor_msgs::PointCloud2::Ptr msg)
{		
	if(tcount%10==0)
	{
			/* 
		try
		{
			listener.lookupTransform("world","firefly1/base_link",ros::Time(0), stamped_transform);                   
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		} 
		t(0)=stamped_transform.getOrigin().x();
		t(1)=stamped_transform.getOrigin().y();
		t(2)=stamped_transform.getOrigin().z()+0.1177;
		q.x()=stamped_transform.getRotation().getX();
		q.y()=stamped_transform.getRotation().getY();
		q.z()=stamped_transform.getRotation().getZ();	
		q.w()=stamped_transform.getRotation().getW();
		*/
		pcl::fromROSMsg(*msg, *msg1);
		for(int i=0;i<msg1->points.size();i++)
		{
			//cout<<"Origin:"<<msg1-makemak>points[i].x<<"  "<<msg1->points[i].y<<"  "<<msg1->points[i].z<<endl;
			p_rotated(0)=msg1->points[i].x;
			p_rotated(1)=msg1->points[i].y;
			p_rotated(2)=msg1->points[i].z;
			R=q.matrix();
			p_rotated=R*p_rotated+t;
			msg1->points[i].x=p_rotated(0);
			msg1->points[i].y=p_rotated(1);
			msg1->points[i].z=p_rotated(2);
			//cout<<"Transformed:"<<msg1->points[i].x<<"  "<<msg1->points[i].y<<"  "<<msg1->points[i].z<<endl;
		}
		if(flag==0)
		{
			*last_msg1=*msg1;
			flag=1;
		}
		else
		{
			*last_msg1+=*msg1;
		}
		pcl::io::savePCDFileASCII("all.pcd",*last_msg1);
		cout<<"Saved!"<<"  ";	

	}
	cout<<"received"<<tcount<<endl;
	tcount++;
}
void callbacktf(const tf2_msgs::TFMessage::ConstPtr& tf)
{
	geometry_msgs::TransformStamped transformStamped;
	transformStamped=tf->transforms[0];
	if(transformStamped.header.frame_id=="world"&&transformStamped.child_frame_id=="firefly1/base_link")
	{
		t(0)=transformStamped.transform.translation.x;
		t(1)=transformStamped.transform.translation.y;
		t(2)=transformStamped.transform.translation.z+0.1177;//
		q.x()=transformStamped.transform.rotation.x;
		q.y()=transformStamped.transform.rotation.y;
		q.z()=transformStamped.transform.rotation.z;	
		q.w()=transformStamped.transform.rotation.w;
	//	cout<<q.coeffs()<<"  "<<ncount<<endl;
	//	ncount++;
	}
} 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/firefly1/velodyne/velodyne_points", 1, callback);
  ros::Subscriber sub1 = nh.subscribe("/tf", 1, callbacktf);
  ros::spin();
/*  	while(nh.ok())
	{
		tf::StampedTransform stamped_transform;   //定义存放变换关系的变量
		try
		{
			listener.lookupTransform("world","firefly1/base_link",ros::Time(0), stamped_transform);                   
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		} 
	}  */
 // ros::Rate r(1);//1Hz  
 /*
  while (ros::ok())
  {
  	ros::spinOnce();                   // Handle ROS events
  	r.sleep();
  }
*/
return 0;   	
}
/*
		 */
/*
	if(flag==0)
	{
		for(int i=0;i<msg1->points.size();i++)
		{
			last_msg1->points[i].x=msg1->points[i].x;
			last_msg1->points[i].y=msg1->points[i].y;
			last_msg1->points[i].z=msg1->points[i].z;
		}
		flag=1;
	}
	else
	{
		for(int i=last_msg1->points.size();i<last_msg1->points.size()+msg1->points.size();i++)
		{
			last_msg1->points[i].x=msg1->points[i].x;
			last_msg1->points[i].y=msg1->points[j].y;
			last_msg1->points[i].z=msg1->points[j].z;
		}
//		pcl::io::savePCDFileASCII("all.pcd",*last_msg1);
	}
*/


//	pcl::fromPCLPointCloud2(*msg,*msg1);	const pcl::PCLPointCloud2::ConstPtr& msg
//	pcl::fromPCLPointCloud2(*last_msg,*last_msg1);	
/*
	last_msg1->width    = msg1->points.size();
	last_msg1->height   = 1;
	last_msg1->is_dense = false;
	last_msg1->points.resize (last_msg1->width *last_msg1->height);
*/