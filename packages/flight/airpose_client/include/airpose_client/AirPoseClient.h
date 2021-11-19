#ifndef AIRPOSE_CLIENT_AIRPOSECLIENT_HH
#define AIRPOSE_CLIENT_AIRPOSECLIENT_HH

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <neural_network_detector/BoostTCPClient.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <neural_network_detector/NeuralNetworkDetection.h>
#include <neural_network_detector/NeuralNetworkDetectionArray.h>
#include <neural_network_detector/NeuralNetworkFeedback.h>
#include <neural_network_detector/NeuralNetworkNumberOfDetections.h>
#include <cv/extensions/projection.h>

namespace airpose_client {

		typedef struct __attribute__ ((__packed__)) {
				uint8_t label;
				float score;
				int16_t xmin;
				int16_t xmax;
				int16_t ymin;
				int16_t ymax;
		} detection_info;

		typedef struct __attribute__ ((__packed__)) {
				int state;
				std::unique_ptr<uint8_t[]> buffer_final_img_;
				float bx;
				float by;
				float scale;
		} first_message;

		typedef struct __attribute__ ((__packed__)) {
				int state;
				std::unique_ptr<float[]> data;
		} second_message;

		typedef struct __attribute__ ((__packed__)) {
				int state;
				std::unique_ptr<float[]> data;
		} third_message;

		typedef struct __attribute__ ((__packed__)) {
				uint16_t count;
				detection_info detection[INT_MAX]; // over_allocated since variable length arrays are not allowed in C++
		} detection_results;

		cv::Rect get_crop_area(const neural_network_detector::NeuralNetworkFeedback &latest_feedback,
		                       const cv::Size2i &original_resolution, const cv::Size2i &desired_resolution,
		                       float aspect_ratio, cv::projection2i &proj_crop, int &bx, int &by, bool timed_out);

		class AirPoseClient {
		private:
				std::unique_ptr<first_message> first_msg_;
				std::unique_ptr<second_message> second_msg_;
				std::unique_ptr<third_message> third_msg_;

				std::unique_ptr<BoostTCPClient> c_{
					new BoostTCPClient}; // will call destructor when out of scope, closing connection smoothly
				ros::NodeHandle pnh_{"~"}, nh_;
				image_transport::Subscriber img_sub_;
				cv::Mat mat_img_;
				std::string host_, port_;
				size_t length_final_img_;
				std::unique_ptr<uint8_t[]> buffer_results_; // will automatically delete when out of scope
				ros::Publisher detection_pub_, step1_pub_, step2_pub_;
				image_transport::Publisher debug_result_pub_;
				ros::Subscriber feedback_sub_, step1_sub_, step2_sub_;
				neural_network_detector::NeuralNetworkFeedback latest_feedback_;
				ros::Duration timeout_;
				ros::Publisher detection_amount_pub_;
				double border_dropoff_{.05};

				// Connect to TCP server (NN)
				bool connect();

				// Try multiple times to connect
				bool connectMultiple(ros::Rate sleeper, int tries);

		public:
				cv::Size2i desired_resolution;
				float score_threshold{0.5};
				int desired_class{15};
				float aspect_ratio{1.33333};
				float var_const_x_min{0}, var_const_x_max{0}, var_const_y_min{0}, var_const_y_max{0};

				float timing_camera{0.025};
				float timing_network_stage1{0.1};
				float timing_network_stage2{0.05};
				float timing_network_stage3{0.05};
				float timing_communication_stage1{0.5};
				float timing_communication_stage2{0.5};
				float timing_communication_stage3{0.5};
				double timing_whole_sequence{999.9};
				ros::Time timing_next_wakeup{0};
				volatile int timing_current_stage{-1};

				bool max_update_force{false};
				std::unique_ptr<ros::Rate> max_update_rate;

				// Constructor
				AirPoseClient(char *host, char *port);

				// Image callback in which communication is handled as well
				void imgCallback(const sensor_msgs::ImageConstPtr &msgp);

				// Disconnect callback to print disconnect message
				void connectCallback();

				// Feedback callback for updating latest feedback info
				void feedbackCallback(const neural_network_detector::NeuralNetworkFeedbackConstPtr &msg);

				void step1Callback(const neural_network_detector::NeuralNetworkFeedbackConstPtr &msg);

				void step2Callback(const neural_network_detector::NeuralNetworkFeedbackConstPtr &msg);

				uint64_t getFrameNumber(ros::Time frameTime);
				ros::Time getFrameTime(uint64_t frameNumber);
				void sleep_until(ros::Time then);
				void mainLoop(void);
		};

}
#endif //AIRPOSE_CLIENT_AIRPOSECLIENT_HH
