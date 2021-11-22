#include <airpose_client/AirPoseClient.h>
#include <cv_bridge/cv_bridge.h>

namespace airpose_client {

#undef DEBUG_ROTATE_IMAGE_90_CW

		static const int color_channels = 3;

		cv::Rect get_crop_area(const neural_network_detector::NeuralNetworkFeedback &latest_feedback,
		                       const cv::Size2i &original_resolution,
		                       const cv::Size2i &desired_resolution, float aspect_ratio, cv::projection2i &proj_crop,
		                       int &bx, int &by,
		                       const bool timed_out = false) {
			// Feedback - zoom level
			if (timed_out || latest_feedback.ymin > original_resolution.height || latest_feedback.ymax < 0) {
				// Special case - target is not in view of camera frame - flagged by ymin > max_y or ymax < 0
				// In this case, we skip the frame
				return cv::Rect(0, 0, 0, 0);
			}

			// Clamp the values to resolution
			int16_t ymin = std::max<int16_t>(latest_feedback.ymin, 0);
			int16_t ymax = std::min<int16_t>(latest_feedback.ymax, original_resolution.height);

			// we have ground truth, so we can crop the image to the ground truth
			if (latest_feedback.debug_included == true) {
				// Clamp the values to resolution
				int16_t xmin = std::max<int16_t>(latest_feedback.xcenter,
				                                 0); // xcenter is not a typo, it is xmin in the gt data
				int16_t xmax = std::min<int16_t>(latest_feedback.ycenter,
				                                 original_resolution.width); // ycenter is not a typo, it is xmax in the gt data

				proj_crop.offset.x = xmin;
				proj_crop.offset.y = ymin;

				bx = (xmax - xmin) / 2;
				by = (ymax - ymin) / 2;

				return cv::Rect(xmin, ymin, xmax - xmin, ymax - ymin);
			}
				// we have no ground truth, so we need to crop the image to the desired resolution
			else {

				// fixme: this is old code, might be wrong in some way in our case

				// Given the aspect ratio that the NN wants, obtain x values
				// Minimum of 10% of desired resolution is required
				int16_t delta_y = std::max<int16_t>((int16_t) (.1 * desired_resolution.height), ymax - ymin);
				int16_t delta_x = (int16_t) (aspect_ratio * delta_y);

				// X center is within image bounds and half the length
				int16_t half_delta_x = (int16_t) (.5 * delta_x);
				int16_t xcenter = std::max<int16_t>(half_delta_x, std::min<int16_t>(latest_feedback.xcenter,
				                                                                    original_resolution.width - half_delta_x));
				bx = xcenter;
				by = (ymax - ymin) / 2;

				// Compute xmin and xmax, even though xcenter is clamped let's not take risks
				int16_t xmin = std::max<int16_t>((int16_t) (xcenter - half_delta_x), 0);
				//int16_t xmax = std::min<int16_t>((int16_t)(xcenter + half_delta_x), original_resolution.width);

				// Set the properties of the projection and build the zoom (crop) rectangle
				proj_crop.offset.x = xmin;
				proj_crop.offset.y = ymin;

				// One final check, we might need to take away 1 pixel due to rounding
				delta_x = std::min<int16_t>(delta_x, original_resolution.width - xmin);
				delta_y = std::min<int16_t>(delta_y, original_resolution.height - ymin);

				return cv::Rect(proj_crop << cv::Point2i(0, 0), proj_crop << cv::Point2i(delta_x, delta_y));
			}
		}

		AirPoseClient::AirPoseClient(char *host, char *port) : host_{host}, port_{port} {

			// Parameters
			std::string img_topic{"video"};
			pnh_.getParam("img_topic", img_topic);

			std::string detections_topic{"object_detections"};
			pnh_.getParam("detections_topic", detections_topic);

			std::string detection_amount_topic{"object_detections/amount"};
			pnh_.getParam("detection_amount_topic", detection_amount_topic);

			std::string feedback_topic{"object_detections/feedback"};
			pnh_.getParam("feedback_topic", feedback_topic);

			std::string step1_topic_fb{"step1_feedback"};
			pnh_.getParam("step1_topic", step1_topic_fb);

			std::string step1_topic_pub{"step1_pub"};
			pnh_.getParam("step1_topic_pub", step1_topic_pub);

			std::string step2_topic_fb{"step2_feedback"};
			pnh_.getParam("step2_topic", step2_topic_fb);

			std::string step2_topic_pub{"step2_pub"};
			pnh_.getParam("step2_topic_pub", step2_topic_pub);
			
			std::string step3_topic_pub{"step3_pub"};
			pnh_.getParam("step3_topic_pub", step3_topic_pub);

			pnh_.param<int>("desired_resolution/x", desired_resolution.width, 224);
			pnh_.param<int>("desired_resolution/y", desired_resolution.height, 224);

			pnh_.getParam("score_threshold", score_threshold);
			pnh_.getParam("desired_class", desired_class);
			pnh_.getParam("aspect_ratio", aspect_ratio);

			double timeout_sec = 5.0;
			pnh_.getParam("timeout_seconds", timeout_sec);
			timeout_ = ros::Duration(timeout_sec);

			pnh_.getParam("timing/camera", timing_camera);
			pnh_.getParam("timing/network_stage1", timing_network_stage1);
			pnh_.getParam("timing/network_stage2", timing_network_stage2);
			pnh_.getParam("timing/network_stage3", timing_network_stage3);
			pnh_.getParam("timing/communication_stage1", timing_communication_stage1);
			pnh_.getParam("timing/communication_stage2", timing_communication_stage2);
			pnh_.getParam("timing/communication_stage3", timing_communication_stage3);
			timing_whole_sequence = timing_network_stage1 + timing_network_stage2 + timing_network_stage3 + timing_communication_stage1 + timing_communication_stage2 + timing_camera;

			pnh_.getParam("border_dropoff", border_dropoff_);

			pnh_.getParam("variance/x/min", var_const_x_min);
			pnh_.getParam("variance/x/max", var_const_x_max);
			pnh_.getParam("variance/y/min", var_const_y_min);
			pnh_.getParam("variance/x/max", var_const_y_max);

			if (pnh_.getParam("max_update/force", max_update_force)) {
				if (max_update_force) {
					double update_hz{1.0};
					pnh_.getParam("max_update/rate", update_hz);
					max_update_rate = std::unique_ptr<ros::Rate>(new ros::Rate(update_hz));
				}
			}


			// Connect to server
			if (!this->connectMultiple(ros::Rate(0.2), 10)) {
				ros::shutdown();
				return;
			}

			// Some pre-allocations
			length_final_img_ = size_t(desired_resolution.width * desired_resolution.height * color_channels);

			buffer_send_ = std::unique_ptr<uint8_t[]>(new uint8_t[length_final_img_+sizeof(first_message)]);
			buffer_send_msg = std::unique_ptr<second_and_third_message>(new second_and_third_message);

			// Publisher of step1 feedback
			step1_pub_ = nh_.advertise<airpose_client::AirposeNetworkData>(step1_topic_pub, 5);
			// Publisher of step2 feedback
			step2_pub_ = nh_.advertise<airpose_client::AirposeNetworkData>(step2_topic_pub, 5);
			// Publisher of step3 feedback (the actual result)
			step3_pub_ = nh_.advertise<airpose_client::AirposeNetworkData>(step3_topic_pub, 5);

			// Image transport interface
			image_transport::ImageTransport it(nh_);

			latest_feedback_.xcenter = latest_feedback_.ymin = latest_feedback_.ymax = -1;
			// Feedback subscriber
			feedback_sub_ = nh_.subscribe(feedback_topic, 1, &AirPoseClient::feedbackCallback,
			                              this);  // only need the most recent

			// Img subscriber
			img_sub_ = it.subscribe(img_topic, 1, &AirPoseClient::imgCallback,
			                        this); // queue of 1, we only want the latest image to be processed


			step1_sub_ = nh_.subscribe(step1_topic_fb, 1, &AirPoseClient::step1Callback,
			                           this); // queue of 1, we only want the latest image to be processed

			step2_sub_ = nh_.subscribe(step2_topic_fb, 1, &AirPoseClient::step2Callback,
			                           this); // queue of 1, we only want the latest image to be processed


		}

		bool AirPoseClient::connectMultiple(ros::Rate sleeper, int tries) {
			int try_n = 0;
			bool res = false;
			while (++try_n <= tries) {
				// Reconnect
				ROS_INFO("Trying to connect to TCP server #%d", try_n);
				res = this->connect();

				if (res)
					break;

				else {
					ROS_INFO("Sleeping before trying again");
					sleeper.sleep();
				}
			}

			if (res)
				ROS_INFO("Connected to %s:%s", host_.c_str(), port_.c_str());
			else {
				ROS_ERROR("Failed to connect to TCP server");
				return false;
			}

			return true;
		}

		bool AirPoseClient::connect() {
			// reset sequencer on reconnect
			timing_current_stage=-1;
			try {
				c_->connect(host_, port_, boost::posix_time::seconds(3));
			}
			catch (std::exception &e) {
				ROS_WARN("Exception: %s", e.what());
				return false;
			}

			if (max_update_force)
				max_update_rate->reset();

			return true;
		}

		void AirPoseClient::imgCallback(const sensor_msgs::ImageConstPtr &msgp) {

			if (!msgp) {
				ROS_WARN("Invalid ImageConstPtr received, not handled.");
				return;
			}

			// check current state. we are only interested in new images prior to stage 1:
			if (timing_current_stage!=0) {
				return;
			}

			//ROS_INFO("Callback called for image seq %d", msgp->header.seq);

			try {
				//ROS_INFO("Parsing image...Update cv::mat object");
				mat_img_ = cv_bridge::toCvShare(msgp, "bgr8")->image;

#ifdef DEBUG_ROTATE_IMAGE_90_CW
				cv::transpose(mat_img_, mat_img_);
				cv::flip(mat_img_, mat_img_, 1); //transpose+flip(1) = Rotate 90 CW
#endif

				//ROS_INFO("Parsing image...Calculate");
				const cv::Size2i original_resolution(mat_img_.cols, mat_img_.rows);

				bool timed_out{false};
				if (msgp->header.stamp - latest_feedback_.header.stamp > timeout_) {
					timed_out = true;
					ROS_INFO_STREAM_THROTTLE(0.5, "Skipping frame " << msgp->header.stamp - latest_feedback_.header.stamp);
				}

				int bx, by;
				// Create an auxiliary, custom projection object to aid in calculations
				cv::projection2i proj_crop(cv::Point2f(1, 1), cv::Point2i(0, 0));
				const auto crop_area = get_crop_area(latest_feedback_, original_resolution, desired_resolution, aspect_ratio,
				                                     proj_crop, bx, by, timed_out);
				if (crop_area.width == 0) {
					ROS_WARN("No crop area found, skipping frame");
					// todo check what to do in this case
					return;
				}

				// we accepted a frame, advancing stage
				timing_current_stage = 1;
				timing_current_frame_time = msgp->header.stamp;


				//ROS_INFO_STREAM("crop " << crop_area);
				cv::Mat cropped = mat_img_(crop_area);
				//ROS_INFO("Center %d,%d\tCrop %d\tOriginal res %d,%d", center.x, center.y, crop_length, original_resolution.width, original_resolution.height);

				//ROS_INFO("Parsing image...Resize");
				// Resize to desired resolution with interpolation, using our allocated buffer
				
				// convenient pointers to create the to-be-sent data - yes, these are ugly raw pointers but only with local scope
				first_message *send_data = (first_message *) (buffer_send_.get());

				const int sizes[2] = {desired_resolution.height, desired_resolution.width}; // this is not a typo, y comes first
				cv::Mat resized(2, sizes, CV_8UC3, buffer_send_.get()+sizeof(first_message));

				// compute values to make it square
				int fill_x = 0, fill_y = 0;
				float scale = 0;
				if (cropped.rows > cropped.cols) {
					scale = desired_resolution.height / (float) cropped.rows;
				} else {
					scale = desired_resolution.width / (float) cropped.cols;
				}

				cv::Mat squared_cropped;
				cv::resize(cropped, squared_cropped, cv::Size(int(scale * cropped.cols), int(scale * cropped.rows)));

				fill_x = int((desired_resolution.width - squared_cropped.cols) / 2);
				fill_y = int((desired_resolution.height - squared_cropped.rows) / 2);
				cv::copyMakeBorder(squared_cropped, resized, fill_x, desired_resolution.width - squared_cropped.cols - fill_x,
				                   fill_y, desired_resolution.height - squared_cropped.rows - fill_y,
				                   cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

				ROS_INFO_STREAM("Final size " << resized.size());

				// fixme not sure of what this does
				cv::projection2i proj_scale(
					cv::Point2f(desired_resolution.width / (float) (cropped.cols),
					            desired_resolution.height / (float) (cropped.rows)),
					cv::Point2i(0, 0));

				send_data->bx = (mat_img_.cols / 2.0 - bx) / (mat_img_.cols / 2.0);
				send_data->by = (mat_img_.rows / 2.0 - by) / (mat_img_.rows / 2.0);
				send_data->scale = scale;
				send_data->state = 1;

				//ROS_INFO("Sending to NN");
				c_->write_bytes(buffer_send_.get(), sizeof(first_message)+length_final_img_, boost::posix_time::seconds(10));

				// Array to be published
				airpose_client::AirposeNetworkData network_data_msg;
				// Header is the same as img msg
				network_data_msg.header.frame_id = msgp->header.frame_id;
				network_data_msg.header.stamp = msgp->header.stamp;

				// Block while waiting for reply
				//ROS_INFO("Receiving answer...");
			
				// Read the count from the buffer
				c_->read_bytes((uint8_t *) &network_data_msg.data[0], sizeof(network_data_msg.data), boost::posix_time::seconds(
					10)); // 2 seconds are not enough here, initialization on first conect might take longer

				// IMPORTANT - the network finished executing - advancing sequencer
				timing_current_stage=2;

				step1_pub_.publish(network_data_msg);

			}
			catch (std::exception &e) {
				ROS_WARN("Exception: %s", e.what());
				ROS_INFO("Creating new TCP client");

				// Delete old client and create new one
				c_.reset(new BoostTCPClient);

				// Try to reconnect to server
				if (!this->connectMultiple(ros::Rate(0.2), 100)) {
					ros::shutdown();
					return;
				}
			}

		}

		void AirPoseClient::connectCallback() {
			/*if (debug_result_pub_.getNumSubscribers() > 0)
				ROS_INFO("At least one client is connected to debug topic");
			else
				ROS_INFO("Clients disconnected from debug topic - stopping debug");*/
		}

		void AirPoseClient::feedbackCallback(const neural_network_detector::NeuralNetworkFeedbackConstPtr &msg) {
			latest_feedback_ = *msg;
		}

		void AirPoseClient::step1Callback(const airpose_client::AirposeNetworkDataConstPtr &msg) {
			first_msg_ = *msg;
		}

		void AirPoseClient::step2Callback(const airpose_client::AirposeNetworkDataConstPtr &msg) {
			second_msg_ = *msg;
		}

		uint64_t AirPoseClient::getFrameNumber(ros::Time frameTime) {
			return ((uint64_t)(frameTime.toSec()/timing_whole_sequence));
		}

		ros::Time AirPoseClient::getFrameTime(uint64_t frameNumber) {
			return ros::Time(((double)(frameNumber)) * ((double)(timing_whole_sequence)));
		}

		void AirPoseClient::sleep_until(ros::Time then) {
			ros::Duration d(then-ros::Time::now());
			if (d>ros::Duration(0.0)) {
				d.sleep();
			}
		}

		void AirPoseClient::resetLoop(uint64_t FrameNumber) {
			timing_current_stage=-1;
			timing_next_wakeup=getFrameTime(FrameNumber+1);
			sleep_until(timing_next_wakeup);
			timing_current_stage=0;
		}

		void AirPoseClient::mainLoop(void) {

			// initial bootstrap - step is now -1
			resetLoop(getFrameNumber(ros::Time::now())+1);

			// stage set to 0 - stage1 - next frame will be received and sent to network
			// stage is then set to 1 by img subscriber
			// result will be received by the network

			while (ros::ok()) {
				uint64_t currentFrame = getFrameNumber(ros::Time::now() + ros::Duration(timing_camera*0.5));
				timing_next_wakeup = getFrameTime(currentFrame) + ros::Duration(timing_camera + timing_network_stage1 + timing_communication_stage1);
				sleep_until(timing_next_wakeup);
				switch (timing_current_stage) {
					case 2:
						// we should be in stage 2 now
						break;
					case 1:
						// image was received but neural network never sent a result back
						while (timing_current_stage==1) {
							// this will eventually cause a network timeout and a reconnect. we just need to wait
							timing_next_wakeup=ros::Time::now() + ros::Duration(timing_camera);
							sleep_until(timing_next_wakeup);
						}
						// it's now either 2 or -1 but we need to skip the frame, same as case 0 (no break ;)
					default:
						// no image was received in time. Skip frame, try again
						resetLoop(getFrameNumber(ros::Time::now()));
						continue;
				}
				// we are now in stage 2

				// check if data for stage 1 has been received from other copter
				// if it was received the subscribers wrote it in the respective objects
				if (getFrameNumber(first_msg_.header.stamp)!=currentFrame) {
					resetLoop(currentFrame);
					continue;
				}
				try {
					buffer_send_msg->state=2;
					std::memcpy(&buffer_send_msg->data[0],&first_msg_.data[0],sizeof(buffer_send_msg->data));
					c_->write_bytes((uint8_t*)buffer_send_msg.get(), sizeof(second_and_third_message), boost::posix_time::seconds(1));

					airpose_client::AirposeNetworkData network_data_msg;
					network_data_msg.header.frame_id = first_msg_.header.frame_id;
					network_data_msg.header.stamp = timing_current_frame_time;

					c_->read_bytes((uint8_t *) &network_data_msg.data[0], sizeof(network_data_msg.data), boost::posix_time::seconds(
						1));
					//PUBLISH DATA TO OTHER COPTER
					step2_pub_.publish(network_data_msg);
					timing_current_stage=3;
				}
				catch (std::exception &e) {
					ROS_WARN("Exception: %s", e.what());
					ROS_INFO("Creating new TCP client");

					// Delete old client and create new one
					c_.reset(new BoostTCPClient);

					// Try to reconnect to server
					if (!this->connectMultiple(ros::Rate(0.2), 100)) {
						ros::shutdown();
						return;
					}
					resetLoop(getFrameNumber(ros::Time::now()));
					continue;
				}


				timing_next_wakeup = getFrameTime(currentFrame) + ros::Duration(timing_camera + timing_network_stage1 + timing_network_stage2 + timing_communication_stage1 + timing_communication_stage2);
				sleep_until(timing_next_wakeup);

				// we are now in stage 3
				// check if data for stage 2 has been received from other copter
				// if it was received the subscribers wrote it in the respective objects
				if (getFrameNumber(second_msg_.header.stamp)!=currentFrame) {
					resetLoop(currentFrame);
					continue;
				}
				try {
					buffer_send_msg->state=3;
					std::memcpy(&buffer_send_msg->data[0],&second_msg_.data[0],sizeof(buffer_send_msg->data));
					c_->write_bytes((uint8_t*)buffer_send_msg.get(), sizeof(second_and_third_message), boost::posix_time::seconds(1));

					airpose_client::AirposeNetworkData network_data_msg;
					network_data_msg.header.frame_id = second_msg_.header.frame_id;
					network_data_msg.header.stamp = timing_current_frame_time;

					c_->read_bytes((uint8_t *) &network_data_msg.data[0], sizeof(network_data_msg.data), boost::posix_time::seconds(
						1));
					//PUBLISH DATA TO THE WORLD
					step3_pub_.publish(network_data_msg);
				}
				catch (std::exception &e) {
					ROS_WARN("Exception: %s", e.what());
					ROS_INFO("Creating new TCP client");

					// Delete old client and create new one
					c_.reset(new BoostTCPClient);

					// Try to reconnect to server
					if (!this->connectMultiple(ros::Rate(0.2), 100)) {
						ros::shutdown();
						return;
					}
					continue;
				}

				resetLoop(currentFrame);
			}

		}

}
