#include <airpose_client/AirPoseClient.h>
#include <cv_bridge/cv_bridge.h>

namespace airpose_client {

#undef DEBUG_ROTATE_IMAGE_90_CW

		static const int color_channels = 3;

		void AirPoseClient::reproject_image(const cv::Mat &image) {
			if (map1.empty() || map2.empty()) {
				// first cv::Mat is the D coeffs, second is R (should be left empty)
				cv::initUndistortRectifyMap(camera_matrix_, cv::Mat(), cv::Mat(),
				                            airpose_camera_matrix_, image.size(), CV_16SC2, map1, map2);
				cv::initUndistortRectifyMap(airpose_camera_matrix_, cv::Mat(), cv::Mat(),
				                            camera_matrix_, image.size(), CV_16SC2, map1_inverse, map2_inverse);
			}

			cv::remap(image, image, map1, map2, cv::INTER_LINEAR);
		}

		void AirPoseClient::reproject_coord(int16_t &xmin, int16_t &ymin, int16_t &xmax, int16_t &ymax,
		                                    cv::Size2i original_resolution) {
			// todo debug this -- order y/x 0,1
			int xmin_new, ymin_new, xmax_new, ymax_new;
			ymin_new = map1_inverse.at<cv::Vec2s>(ymin, xmin)[1];
			ymax_new = map1_inverse.at<cv::Vec2s>(ymax, xmax)[1];
			xmin_new = map1_inverse.at<cv::Vec2s>(ymin, xmin)[0];
			xmax_new = map1_inverse.at<cv::Vec2s>(ymax, xmax)[0];

			ymin = std::max<int16_t>(ymin_new, 0);
			xmin = std::max<int16_t>(xmin_new, 0);
			ymax = std::min<int16_t>(ymax_new, original_resolution.height-1);
			xmax = std::min<int16_t>(xmax_new, original_resolution.width-1);
		}

		cv::Rect AirPoseClient::get_crop_area(const neural_network_detector::NeuralNetworkFeedback &latest_feedback,
		                                      const cv::Size2i &original_resolution,
		                                      float &bx, float &by,
		                                      const bool timed_out = false) {
			// Feedback - zoom level
			if (timed_out || latest_feedback.ymin > original_resolution.height || latest_feedback.ymax < 0) {
				// Special case - target is not in view of camera frame - flagged by ymin > max_y or ymax < 0
				// In this case, we skip the frame
				return cv::Rect(0, 0, 0, 0);
			}

			// Clamp the values to resolution
			int16_t ymin = std::max<int16_t>(latest_feedback.ymin, 0);
			int16_t ymax = std::min<int16_t>(latest_feedback.ymax, original_resolution.height-1);

			int16_t xmin = -1;
			int16_t xmax = -1;

			// we have ground truth, so we can crop the image to the ground truth
			if (has_groundtruth) {
				ROS_INFO_STREAM("Using ground truth");
				// Clamp the values to resolution
				xmin = std::max<int16_t>(latest_feedback.ycenter, 0); // xcenter is not a typo, it is xmin in the gt data
				xmax = std::min<int16_t>(latest_feedback.xcenter,
				                         original_resolution.width); // ycenter is not a typo, it is xmax in the gt data
			}
				// we have no ground truth, so we need to crop the image to the desired resolution
			else {

				// Given the aspect ratio that the NN wants, obtain x values
				// Minimum of 10% of desired resolution is required
				int16_t delta_y = std::max<int16_t>((int16_t) (.1 * desired_resolution.height), ymax - ymin);
				int16_t delta_x = (int16_t) (aspect_ratio * delta_y);

				// X center is within image bounds and half the length
				int16_t half_delta_x = (int16_t) (.5 * delta_x);
				int16_t xcenter = std::max<int16_t>(half_delta_x, std::min<int16_t>(latest_feedback.xcenter,
				                                                                    original_resolution.width - half_delta_x));

				// Compute xmin and xmax, even though xcenter is clamped let's not take risks
				xmin = std::max<int16_t>((int16_t) (xcenter - half_delta_x), 0);
				xmax = std::min<int16_t>((int16_t) (xcenter + half_delta_x), original_resolution.width-1);
			}
			if (need_reproj)
				reproject_coord(xmin, ymin, xmax, ymax, original_resolution);

			bx = (xmax + xmin) / 2.0;
			by = (ymax + ymin) / 2.0;
//			ROS_INFO_STREAM("xmin: " << xmin << " xmax: " << xmax << " ymin: " << ymin << " ymax: " << ymax);

			return cv::Rect(xmin, ymin, xmax - xmin, ymax - ymin);
		}

		AirPoseClient::AirPoseClient(char *host, char *port) : host_{host}, port_{port} {

			// Parameters
			std::string img_topic{"video"};
			pnh_.getParam("img_topic", img_topic);

			std::string feedback_topic{"object_detections/feedback"};
			pnh_.getParam("feedback_topic", feedback_topic);
			ROS_INFO_STREAM("feedback_topic: " << feedback_topic);

			int robotID;
			pnh_.getParam("robotID", robotID);
			int other_robotID = robotID == 1 ? 2 : 1;
			ROS_INFO_STREAM("robotID: " << robotID << " other_robotID: " << other_robotID);

			pnh_.getParam("groundtruth", has_groundtruth);
			pnh_.getParam("reproject", need_reproj);

			std::string camera_info_topic{"video/camera_info"};
			pnh_.getParam("camera_info_topic", camera_info_topic);
			camera_matrix_ = cv::Mat::eye(3, 3, CV_64F) * -1;

			std::string step1_topic_pub{"step1_pub"};
			pnh_.getParam("step1_topic_pub", step1_topic_pub);
			std::string step1_topic_fb{"/machine_" + std::to_string(other_robotID) + "/" + step1_topic_pub};

			std::string step2_topic_pub{"step2_pub"};
			pnh_.getParam("step2_topic_pub", step2_topic_pub);
			std::string step2_topic_fb{"/machine_" + std::to_string(other_robotID) + "/" + step2_topic_pub};

			std::string step3_topic_pub{"step3_pub"};
			pnh_.getParam("step3_topic_pub", step3_topic_pub);

			pnh_.param<int>("desired_resolution/x", desired_resolution.width, 224);
			pnh_.param<int>("desired_resolution/y", desired_resolution.height, 224);

			pnh_.param<int>("min_area", min_area_, 50);
			min_area_ *= min_area_;

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

			double fx, fy, cx, cy;
			if (robotID == 1) {
				pnh_.getParam("network_camera_matrix_1/fx", fx);
				pnh_.getParam("network_camera_matrix_1/fy", fy);
				pnh_.getParam("network_camera_matrix_1/cx", cx);
				pnh_.getParam("network_camera_matrix_1/cy", cy);
			} else if (robotID == 2) {
				pnh_.getParam("network_camera_matrix_2/fx", fx);
				pnh_.getParam("network_camera_matrix_2/fy", fy);
				pnh_.getParam("network_camera_matrix_2/cx", cx);
				pnh_.getParam("network_camera_matrix_2/cy", cy);
			} else {
				ROS_ERROR("NO CAM PARAM GIVEN for robotID %d", robotID);
			}
			airpose_camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
			airpose_camera_matrix_.at<double>(0, 0) = fx;
			airpose_camera_matrix_.at<double>(1, 1) = fy;
			airpose_camera_matrix_.at<double>(0, 2) = cx;
			airpose_camera_matrix_.at<double>(1, 2) = cy;
//			ROS_INFO_STREAM(airpose_camera_matrix_);

			timing_whole_sequence =
				timing_network_stage1 + timing_network_stage2 + timing_network_stage3 + timing_communication_stage1 +
				timing_communication_stage2 + timing_camera;

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

			buffer_send_ = std::unique_ptr<uint8_t[]>(new uint8_t[length_final_img_ + sizeof(first_message)]);
			buffer_send_msg = std::unique_ptr<second_and_third_message>(new second_and_third_message);

			// Publisher of step1 feedback
			step1_pub_ = nh_.advertise<airpose_client::AirposeNetworkData>(step1_topic_pub, 5);
			// Publisher of step2 feedback
			step2_pub_ = nh_.advertise<airpose_client::AirposeNetworkData>(step2_topic_pub, 5);
			// Publisher of step3 feedback (the actual result)
			step3_pub_ = nh_.advertise<airpose_client::AirposeNetworkResult>(step3_topic_pub, 5);

			// Image transport interface
			image_transport::ImageTransport it(nh_);

			latest_feedback_.xcenter = latest_feedback_.ymin = latest_feedback_.ymax = -1;
			// Feedback subscriber
			feedback_sub_ = nh_.subscribe(feedback_topic, 1, &AirPoseClient::feedbackCallback,
			                              this);  // only need the most recent

			// Img subscriber
			ROS_INFO_STREAM("Subscribing to " << img_topic);
			img_sub_ = it.subscribe(img_topic, 1, &AirPoseClient::imgCallback,
			                        this); // queue of 1, we only want the latest image to be processed


			step1_sub_ = nh_.subscribe(step1_topic_fb, 1, &AirPoseClient::step1Callback,
			                           this); // queue of 1, we only want the latest msg to be processed

			step2_sub_ = nh_.subscribe(step2_topic_fb, 1, &AirPoseClient::step2Callback,
			                           this); // queue of 1, we only want the latest msg to be processed
			ROS_INFO_STREAM("Camera info topic " << camera_info_topic);
			camera_info_sub_ = nh_.subscribe(camera_info_topic, 1, &AirPoseClient::cameraInfoCallback,
			                                 this); // queue of 1, we only want the msg image to be processed
			seq = -1;
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
			timing_current_stage = -1;
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

//			ROS_INFO_STREAM("Received image of size " << msgp->width << "x" << msgp->height);
			// check current state. we are only interested in new images prior to stage 1:
			if (timing_current_stage != 0) {
//				ROS_WARN_STREAM("Not handling image, current stage is " << timing_current_stage);
				return;
			}
			uint64_t frameID = (uint64_t) timing_current_frame_id;

			//ROS_INFO("Callback called for image seq %d", msgp->header.seq);

			if (ros::Time::now() - msgp->header.stamp >= ros::Duration(2.0 * timing_camera)) {
				ROS_WARN_STREAM("WARNING: Camera runs at " << 1.0 / timing_camera
				                                           << "fps, but we received an image 2 or more frames outdated. Can't synchronize this!\n");
				return;
			}

			try {
				//ROS_INFO("Parsing image...Update cv::mat object");
				cv::Mat local_mat_img_ = cv_bridge::toCvShare(msgp, "bgr8")->image;
				seq = msgp->header.seq;

#ifdef DEBUG_ROTATE_IMAGE_90_CW
				cv::transpose(local_mat_img_, local_mat_img_);
				cv::flip(local_mat_img_, local_mat_img_, 1); //transpose+flip(1) = Rotate 90 CW
#endif

				bool timed_out{false};
				if (msgp->header.stamp - latest_feedback_.header.stamp > timeout_) {
					timed_out = true;
//					ROS_INFO_STREAM_THROTTLE(0.5, "Skipping frame " << msgp->header.stamp - latest_feedback_.header.stamp);
//					ROS_INFO_STREAM("Timeout " << timeout_ << " msgp " << msgp->header.stamp << " latest_feedback "
//					                           << latest_feedback_.header.stamp);
				}
//				ROS_INFO_STREAM("Latest feedback id " << latest_feedback_.header.seq << " timed out " << timed_out);
				if (has_groundtruth && !timed_out) {
					// force skip if benchtest is active
					if (latest_feedback_.header.seq != msgp->header.seq) {
						timed_out = true;
					}
				}
				if (timed_out){
					ROS_WARN_STREAM("Timed out");
					return;
				}

//				cv::imwrite("orig.png", local_mat_img_);
				if (need_reproj) {
					reproject_image(local_mat_img_);
//					cv::imwrite("reproj.png", local_mat_img_);
//					ros::Duration(10).sleep();
					ROS_INFO_STREAM("Reprojected");
				}

				//ROS_INFO("Parsing image...Calculate");
				// original_resolution is always updated after reprojection
				const cv::Size2i original_resolution(local_mat_img_.cols, local_mat_img_.rows);

				float bx, by;
				// Create an auxiliary, custom projection object to aid in calculations
				 auto crop_area = get_crop_area(latest_feedback_, original_resolution, bx, by, timed_out);
				if (crop_area.width == 0 or crop_area.height * crop_area.width < min_area_) {
					ROS_WARN("Very small area, FULL frame");
					
					latest_feedback_.xcenter = camera_matrix_.at<double>(0,2);
					latest_feedback_.ycenter = camera_matrix_.at<double>(1,2);
					latest_feedback_.ymin = 0;
					latest_feedback_.ymax = local_mat_img_.rows-1;
					
					crop_area = get_crop_area(latest_feedback_, original_resolution, bx, by, timed_out);
				}

				// we accepted a frame, advancing stage
				timing_current_stage = 1;
				timing_current_frame_time = msgp->header.stamp;


				//ROS_INFO_STREAM("crop " << crop_area);
				cv::Mat cropped = local_mat_img_(crop_area);
//				cv::imwrite("cropped.png", cropped);

//				ROS_INFO("Center %f,%f\t \tOriginal res %d,%d", bx, by, original_resolution.width, original_resolution.height);

				//ROS_INFO("Parsing image...Resize");
				// Resize to desired resolution with interpolation, using our allocated buffer

				// convenient pointers to create the to-be-sent data - yes, these are ugly raw pointers but only with local scope
				first_message *send_data = (first_message *) (buffer_send_.get());

				const int sizes[2] = {desired_resolution.height, desired_resolution.width}; // this is not a typo, y comes first
				cv::Mat resized(2, sizes, CV_8UC3, buffer_send_.get() + sizeof(first_message));

				// compute values to make it square
				int fill_x = 0, fill_y = 0;
				float scale = 0;
				if (cropped.rows > cropped.cols) {
					scale = desired_resolution.height / (float) cropped.rows;
				} else {
					scale = desired_resolution.width / (float) cropped.cols;
				}

				cv::resize(cropped, cropped, cv::Size(int(scale * cropped.cols), int(scale * cropped.rows)));
//				ROS_INFO_STREAM("Resized to " << cropped.cols << "x" << cropped.rows);

				fill_x = int((desired_resolution.width - cropped.cols) / 2);
				fill_y = int((desired_resolution.height - cropped.rows) / 2);

//				ROS_INFO_STREAM("Filling " << fill_x << " " << fill_y);

				cv::copyMakeBorder(cropped, resized, fill_y, desired_resolution.height - cropped.rows - fill_y,
				                   fill_x, desired_resolution.width - cropped.cols - fill_x, cv::BORDER_CONSTANT,
				                   cv::Scalar(0, 0, 0));
//				cv::imwrite("resized.png", resized);

				send_data->bx = (float) (bx / airpose_camera_matrix_.at<double>(0, 2) - 1);
				send_data->by = (float) (by / airpose_camera_matrix_.at<double>(1, 2) - 1);

				send_data->scale = scale;
				send_data->state = 0;
//				ROS_INFO_STREAM("Image id " << msgp->header.seq << " feedback id: " << latest_feedback_.header.seq);
//				ROS_INFO_STREAM("IMAGE " << std::setprecision(15) << msgp->header.seq << " bx " << send_data->bx << " by " << send_data->by << " scale "
//				                         << send_data->scale << " cx " << camera_matrix_.at<double>(0,2) << " cy " << camera_matrix_.at<double>(1,2));
				c_->write_bytes(buffer_send_.get(), sizeof(first_message) + length_final_img_, boost::posix_time::seconds(10));

				// Array to be published
				airpose_client::AirposeNetworkData network_data_msg;
				// Header is the same as img msg
				network_data_msg.header.frame_id = msgp->header.frame_id;
				network_data_msg.frame_id = frameID;
				network_data_msg.header.stamp = msgp->header.stamp;

				// Block while waiting for reply
				//ROS_INFO("Receiving answer...");

				// Read the count from the buffer
				c_->read_bytes((uint8_t *) &network_data_msg.data[0], sizeof(network_data_msg.data), boost::posix_time::seconds(
					10)); // 2 seconds are not enough here, initialization on first conect might take longer
//				for (auto d: network_data_msg.data) {
//					ROS_INFO_STREAM("Network data: " << d);
//				}

				// IMPORTANT - the network finished executing - advancing sequencer
				timing_current_stage = 2;
				network_data_msg.p_vector[0] = send_data->bx;
				network_data_msg.p_vector[1] = send_data->by;
				network_data_msg.p_vector[2] = send_data->scale;

				network_data_msg.bb_vector[0] = crop_area.x;
				network_data_msg.bb_vector[1] = crop_area.y;
				network_data_msg.bb_vector[2] = crop_area.height;
				network_data_msg.bb_vector[3] = crop_area.width;

				network_data_msg.detector_feedback_stamp = latest_feedback_.header.stamp;

				step1_pub_.publish(network_data_msg);

				// do this after, since expensive (slow)
//				mat_img_ = cv_bridge::toCvCopy(msgp, "bgr8")->image;
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

		void AirPoseClient::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
			camera_matrix_ = cv::Mat(3, 3, CV_64F, (void *) msg->K.data());
//			ROS_INFO_STREAM("Camera info received");
//			dist_coeffs_ = cv::Mat(1, 5, CV_64F, (void *) msg->D.data());
		}

		uint64_t AirPoseClient::getFrameNumber(ros::Time frameTime) {
			return ((uint64_t) (frameTime.toSec() / timing_whole_sequence));
		}

		ros::Time AirPoseClient::getFrameTime(uint64_t frameNumber) {
			return ros::Time(((double) (frameNumber)) * ((double) (timing_whole_sequence)));
		}

		void AirPoseClient::sleep_until(ros::Time then) {
			ros::Duration d(then - ros::Time::now());
			if (d > ros::Duration(0.0)) {
				d.sleep();
			}
		}

		void AirPoseClient::resetLoop(uint64_t FrameNumber) {
			timing_current_stage = -1;
			timing_next_wakeup = getFrameTime(FrameNumber + 1);
			sleep_until(timing_next_wakeup);
			timing_current_stage = 0;
			timing_current_frame_id = getFrameNumber(ros::Time::now() + ros::Duration(epsilon));
		}

		void AirPoseClient::mainLoop(void) {

			// initial bootstrap - step is now -1
			resetLoop(getFrameNumber(ros::Time::now()) + 1);

			// stage set to 0 - stage1 - next frame will be received and sent to network
			// stage is then set to 1 by img subscriber
			// result will be received by the network

			while (ros::ok()) {
				timing_next_wakeup = getFrameTime(timing_current_frame_id) +
				                     ros::Duration(timing_camera + timing_network_stage1 + timing_communication_stage1);
				sleep_until(timing_next_wakeup);
				switch (timing_current_stage) {
					case 2:
						// we should be in stage 2 now
						break;
					case 1:
						// image was received but neural network never sent a result back
						while (timing_current_stage == 1) {
							// this will eventually cause a network timeout and a reconnect. we just need to wait
							timing_next_wakeup = ros::Time::now() + ros::Duration(timing_camera);
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
				if (first_msg_.frame_id != (uint64_t) timing_current_frame_id) {
					ROS_INFO_STREAM("Current frame 1 " << first_msg_.frame_id << " != " << timing_current_frame_id);
					resetLoop(timing_current_frame_id);
					ROS_INFO_STREAM("Step 2 skip..");
					continue;
				}
				try {
					ROS_INFO_STREAM("Step 2 init..");
					buffer_send_msg->state = 1;
					std::memcpy(&buffer_send_msg->data[0], &first_msg_.data[0], sizeof(buffer_send_msg->data));
//					for (auto d: first_msg_.data) {
//						ROS_INFO_STREAM("Network data from other copter: " << d);
//					}
					c_->write_bytes((uint8_t *) buffer_send_msg.get(), sizeof(second_and_third_message),
					                boost::posix_time::seconds(1));


					airpose_client::AirposeNetworkData network_data_msg;
					network_data_msg.header.frame_id = first_msg_.header.frame_id;
					network_data_msg.frame_id = first_msg_.frame_id;
					network_data_msg.header.stamp = timing_current_frame_time;

					c_->read_bytes((uint8_t *) &network_data_msg.data[0], sizeof(network_data_msg.data),
					               boost::posix_time::seconds(1));
					//PUBLISH DATA TO OTHER COPTER
					step2_pub_.publish(network_data_msg);
					timing_current_stage = 3;
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


				timing_next_wakeup = getFrameTime(timing_current_frame_id) + ros::Duration(
					timing_camera + timing_network_stage1 + timing_network_stage2 + timing_communication_stage1 +
					timing_communication_stage2);
				sleep_until(timing_next_wakeup);

				// we are now in stage 3
				// check if data for stage 2 has been received from other copter
				// if it was received the subscribers wrote it in the respective objects
				if (second_msg_.frame_id != (uint64_t) timing_current_frame_id) {
					resetLoop(timing_current_frame_id);
					ROS_INFO_STREAM("Step 3 skip..");
					continue;
				}
				try {
					ROS_INFO_STREAM("Step 3 init..");
					buffer_send_msg->state = 2;
					std::memcpy(&buffer_send_msg->data[0], &second_msg_.data[0], sizeof(buffer_send_msg->data));
//					for (auto d: second_msg_.data) {
//						ROS_INFO_STREAM("Network data from other copter: " << d);
//					}
					c_->write_bytes((uint8_t *) buffer_send_msg.get(), sizeof(second_and_third_message),
					                boost::posix_time::seconds(1));

					airpose_client::AirposeNetworkResult network_result_msg;
					network_result_msg.header.frame_id = second_msg_.header.frame_id;
					network_result_msg.frame_id = second_msg_.frame_id;
					network_result_msg.header.stamp = timing_current_frame_time;

					c_->read_bytes((uint8_t *) &network_result_msg.data[0], sizeof(network_result_msg.data),
					               boost::posix_time::seconds(1));

//					try {
//						cv_bridge::CvImage img_bridge;
//						std_msgs::Header header; // empty header
//						header.stamp = network_result_msg.header.stamp; // time
//						header.frame_id = std::to_string(seq); // id
//						img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, mat_img_);
//						img_bridge.toImageMsg(network_result_msg.img); // from cv_bridge to sensor_msgs::Image
//					}
//					catch (cv_bridge::Exception &e) {
//						ROS_ERROR_STREAM("cv_bridge exception: %s" << e.what());
//					}
					network_result_msg.header.frame_id = std::to_string(seq); // id

					//PUBLISH DATA TO THE WORLD
					step3_pub_.publish(network_result_msg);
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

				resetLoop(timing_current_frame_id);
			}

		}

}
