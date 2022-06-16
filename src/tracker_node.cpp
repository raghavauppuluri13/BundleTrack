#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include "yaml-cpp/yaml.h"

#include "Frame.h"
#include "Bundler.h"

class ObjectTracker {
    private:
        ros::NodeHandle nh_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::CameraInfo>
    SyncPolicyT;
        message_filters::Subscriber<sensor_msgs::Image> *masked_depth_sub_;
        message_filters::Subscriber<sensor_msgs::Image> *masked_image_sub_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> *depth_info_sub_;
        ros::Subscriber roi_sub_;
        message_filters::Synchronizer<SyncPolicyT> *sync_;

        std::shared_ptr<YAML::Node> yml_;

        //PointCloudRGBNormal::Ptr mesh_pc_;

        tf::TransformListener tf_listener_;
        tf::TransformBroadcaster br_;

        std::shared_ptr<Frame> frame_ptr_;
        Eigen::Matrix3f K_;
        std::string cam_frame_id_;
        std::string base_frame_id_;

        Eigen::Vector4f roi_; 

        Bundler* bundler;

        int im_id_;

    public:
        ObjectTracker();

        void roi_cb(const sensor_msgs::RegionOfInterest& msg);
        void main_cb(const sensor_msgs::ImageConstPtr& depth_im, const sensor_msgs::ImageConstPtr& color_im, const sensor_msgs::CameraInfoConstPtr& info);

};

ObjectTracker::ObjectTracker() : yml_(new YAML::Node), im_id_(0) {
    K_.setIdentity();
    ros::param::get("/base_frame_id",base_frame_id_);
    ros::param::get("~cam_frame_id",cam_frame_id_);

    std::string config_path = "/root/catkin_ws/src/BundleTrack/config_ycbineoat.yml";
    //ros::param::get("~config_path",config_path);
    *yml_ = YAML::LoadFile(config_path);

    bundler = new Bundler(yml_);

    std::string masked_depth_topic = "/d455_0/camera/aligned_depth_to_color/masked_image";
    //ros::param::get("~masked_depth_topic", masked_depth_topic);
    masked_depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_,masked_depth_topic,10);

    std::string masked_image_topic = "/d455_0/camera/color/masked_image";
    //ros::param::get("~masked_image_topic", masked_image_topic);
    masked_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_,masked_image_topic,10);
    std::string roi_topic = "/d455_0/camera/color/roi";
    //ros::param::get("~masked_image_topic", masked_image_topic);
    roi_sub_ = nh_.subscribe(roi_topic,10,&ObjectTracker::roi_cb,this);

    std::string depth_info_topic = "/d455_0/camera/aligned_depth_to_color/camera_info";
    //ros::param::get("~depth_info_topic", depth_info_topic);

    depth_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_,depth_info_topic,1);

    sync_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),*masked_depth_sub_, *masked_image_sub_,*depth_info_sub_);
    sync_->registerCallback(boost::bind(&ObjectTracker::main_cb, this, _1, _2,_3));
}

void ObjectTracker::roi_cb(const sensor_msgs::RegionOfInterest& msg) {
    roi_ << msg.x_offset, msg.x_offset + msg.width, msg.y_offset, msg.y_offset + msg.height;
}


void ObjectTracker::main_cb(const sensor_msgs::ImageConstPtr& depth_im, const sensor_msgs::ImageConstPtr& color_im, const sensor_msgs::CameraInfoConstPtr& info) {
    cv_bridge::CvImagePtr cv_image;        
    cv::Mat color, depth, depth_raw, depth_sim;
    cv_image = cv_bridge::toCvCopy(*depth_im);
    depth = cv_image->image;
    depth_raw = depth.clone();
    depth_sim = depth.clone();

    cv_image = cv_bridge::toCvCopy(*color_im);
    color = cv_image->image;

    for(int i = 0; i < info->K.size(); i++) {
        K_ << info->K[i];
    }

    tf::StampedTransform transform;
    try {
        tf_listener_.lookupTransform(base_frame_id_, cam_frame_id_, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    Eigen::Affine3d init_pose_tf;
    tf::transformTFToEigen(transform,init_pose_tf);
    Eigen::Matrix4f init_pose;
    init_pose.setIdentity();
    init_pose.block<3,3>(0,0) = init_pose_tf.rotation().cast<float>();
    init_pose.block<3,1>(0,3) = init_pose_tf.translation().cast<float>();

    std::shared_ptr<Frame> frame(new Frame(color,depth,depth_raw,depth_sim, roi_, init_pose, im_id_, std::to_string(im_id_), K_, yml_));
    im_id_++;

    bundler->processNewFrame(frame); 
    Eigen::Matrix4f pose = frame->_pose_in_model.inverse();


    geometry_msgs::TransformStamped tf;
    Eigen::Quaternionf q(pose.topLeftCorner<3, 3>());
    transform.setOrigin(tf::Vector3(pose.col(3)(0), pose.col(3)(1), pose.col(3)(2)));
    transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    tf.child_frame_id = cam_frame_id_;
    tf.header.frame_id = base_frame_id_;
    tf.header.stamp = ros::Time::now();
    tf.transform.translation.x = pose.col(3)(0);
    tf.transform.translation.y = pose.col(3)(1);
    tf.transform.translation.z = pose.col(3)(2);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    br_.sendTransform(tf);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "localizer_node");
    ObjectTracker tracker;
    ros::spin();
}