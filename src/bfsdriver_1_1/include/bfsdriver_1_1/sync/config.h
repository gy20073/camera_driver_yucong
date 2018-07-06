#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}
#include <chrono>
#include <boost/filesystem.hpp>
#include <bfsdriver_1_1/ImageStamp.h>
#include <sstream>
#include <iostream>
#include <deque>
#include <thread>
#include <mutex>

typedef std::chrono::microseconds Micro;
typedef std::chrono::high_resolution_clock Time;
typedef boost::shared_ptr<bfsdriver_1_1::ImageStamp> ImageStampPtr;


void little_sleep(Micro us) {
	auto start = std::chrono::high_resolution_clock::now();
	auto end = start + us;
	do {
		std::this_thread::yield();
	} while (std::chrono::high_resolution_clock::now() < end);
}

// Self defined.
#include <bfsdriver_1_1/ImageStamp.h>

namespace config {
	const bool IMAGE_IS_RESIZED = true;
	const int MAX_FRAMES = 800;
	float FPS = 10.0; // Shall be dynamic. Later.
	int WIDTH = 1024; // Shall be dynamic. Later.
	int HEIGHT = 768; // Same. Shall be dynamic.
	const int PRECISION = 10000; // To forth digit.
};

class Compressor {
public:
	Compressor(int num): cameraNum(num), threads_started(false) { }

	void receive(bfsdriver_1_1::ImageStamp imageStamp) {
		std::unique_lock<std::mutex> queue_lock(queue_mutex, std::defer_lock);
		queue_lock.lock();

		img_queue.push_back(imageStamp.img); // Make a copy. it's not too big to blow the mem.
		time_queue.push_back(imageStamp.timestamp); // Save the timestamp of the image. 
		printf("Image queue size is %lu\n", img_queue.size());
		received_count++;

		queue_lock.unlock();
		if (!threads_started) {
			thread_pool.push_back(std::thread(&Compressor::compressing, this));
			// for (auto& t: thread_pool) { t.join(); }
			threads_started = true;
		}
		
	}

	void addNewFrame(long timestamp) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			int ret;
			cv_ptr = cv_bridge::toCvCopy(img_queue.front(), sensor_msgs::image_encodings::RGB8);
			img_queue.pop_front();

			/* allocate frame buffer for encoding. */
			frame = av_frame_alloc();
			std::vector<uint8_t> framebuf(avpicture_get_size(c->pix_fmt, config::WIDTH, config::HEIGHT));
			avpicture_fill(reinterpret_cast<AVPicture*>(frame), framebuf.data(), c->pix_fmt, config::WIDTH, config::HEIGHT);
			frame->width = config::WIDTH;
			frame->height = config::HEIGHT;
			frame->format = static_cast<int>(c->pix_fmt);

			// Ads_scale(swding cv_ptr->image to the new frame. (To be added ...)
			AVPacket pkt;
			pkt.data = nullptr;
			pkt.size = 0;
			av_init_packet(&pkt);
			fflush(stdout);

			/* put image in frame in packet */
			
			const int stride[] = { static_cast<int>(cv_ptr->image.step[0]) };
			sws_scale(swsctx, &(cv_ptr->image).data, stride, 0, cv_ptr->image.rows, frame->data, frame->linesize);
			
			printf("Img_queue size is: %lu, frame_encoded is %d\n", img_queue.size(), frame_count);
			printf("timestamp is %lu\n", timestamp);
			for (int i = 0; i < cv_ptr->image.dims; ++i) {
				printf("Step is: %lu\n", cv_ptr->image.step[i]);
			}
			
			printf("Stride is: %d\n", stride[0]);
			printf("Col is: %d\n", cv_ptr->image.cols);
			printf("Row is: %d\n", cv_ptr->image.rows);
			for (int i = 0; i < 8; ++i) {
				printf("Linesize is: %d\n", frame->linesize[i]);
			}
			// frame->linesize[0] = 3072;
			// frame->linesize[1] = 1024;


			cv::Mat mat(config::HEIGHT, config::WIDTH, CV_8UC3, frame->data[0], frame->linesize[0]);
			imwrite("test11.jpg", cv_ptr->image);

			
			frame->pts = frame_pts++;


			/* Encode a frame of video. 
			 * Takes input raw video data from frame and writes 
			 * the next output packet, if available, to avpkt. */
			frame_count++;
			ret = avcodec_encode_video2(c, &pkt, frame_count == config::MAX_FRAMES ? nullptr : frame, &got_output);
			printf("Packet size is %d\n", pkt.size);

			if (ret < 0) {
	            fprintf(stderr, "Error encoding frame\n");
	            exit(1);
	        }

	        if (got_output) {
	        	pkt.duration = 1; // Set duration. Default is 1 time_base (1 / frame_rate).
	        	fwrite(pkt.data, 1, pkt.size, f);
	        	av_packet_unref(&pkt); // Free the packet after encoding.
	        	ROS_INFO("Num %d frame encoded.", frame_count);
	        } else {
	        	printf("Output packet empty!\n");
	        }
	         
	        av_frame_free(&frame);
	        

		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge Exception: %s", e.what());
			return;
		}
	}

	/* This function shall be executed by a worker thread. */
	void compressing() {
		printf("Start compressing...\n");
		while (is_running) {
			if (img_queue.empty()) {
				ROS_INFO("Waiting for some image...");
				little_sleep(Micro(500000));
				continue;
			}
			if (is_recording) {
				// Add a new frame.
				std::unique_lock<std::mutex> queue_lock(queue_mutex, std::defer_lock);
				queue_lock.lock();
				long stamp = time_queue.front();
				time_queue.pop_front();
				addNewFrame(stamp); // Would add new image frame with this timestamp.
				queue_lock.unlock();

				if (frame_count >= buffer_size) {
					
					ROS_INFO("%d frames encoded", frame_count);
					uint8_t endcode[] = { 0, 0, 1, 0xb7 };
					fwrite(endcode, 1, sizeof(endcode), f);
					fclose(f);

					is_recording = false; // Reset. 
					frame_count = received_count = frame_pts = 0; // Reset everything.
					
					/* Free everything alloced. Would be taken care of when
					the next image arrives and reopen everything. */
					// av_frame_free(&frame);
					avcodec_close(c);
					++video_count;
					ROS_INFO("Done writing video %d", video_count);
				}
			} else {
				/* If there's enough pictures in the queue, start recording */
				if (img_queue.size() > 10) {

					/* Set fps */
					is_recording = true;
					long start_us = time_queue[0];
					long end_us = time_queue[10];
					setFps(start_us, end_us);

					/* Create filename */
					long timestamp = time_queue.front();
					time_queue.pop_front();


 					/* Creating a dir for this camera. if exist, skip. */
					std::ostringstream dirpath;
					dirpath << "./Camera_" << cameraNum;
					boost::filesystem::path p(dirpath.str().c_str());
					createLocalDir(p);
					ROS_INFO("======= Dir: %s created =========", dirpath.str().c_str());

 					/* Creating filename to be stored */ 
					std::ostringstream filename;
					filename << dirpath.str() << "/Sync-" << timestamp << "-" << cameraNum << ".h264"; //Infer

					/* Find the mpeg1 video encoder */
					int codec_id = AV_CODEC_ID_H264; /* Using h264 as encode id. */
					codec = avcodec_find_encoder(AV_CODEC_ID_H264);
					if (!codec) {
						fprintf(stderr, "codec for h264 not found\n");
						exit(1);
					}

					/* Alloc context for the codec */
					c = avcodec_alloc_context3(codec);
					if (!c) {
						fprintf(stderr, "Couldn't allocate video codec context\n");
						exit(1);
					}

					
					c->width = config::WIDTH;
					c->height = config::HEIGHT;
					c->time_base = dst_fps;
					// Questionable settings.....
					// c->bit_rate = 400000;
					c->gop_size = 10;
					// c->max_b_frames = 1;
					c->pix_fmt = AV_PIX_FMT_YUV420P;

					if (codec_id == AV_CODEC_ID_H264) {
						av_opt_set(c->priv_data, "preset", "slow", 0);
					}

					// AV_PIX_FMT_RGB24 is equivalent to the OpenCV RGB8 (CV_8UC3)
					swsctx = sws_getCachedContext(nullptr, config::WIDTH, config::HEIGHT, AV_PIX_FMT_RGB24, 
						config::WIDTH, config::HEIGHT, c->pix_fmt, SWS_BICUBIC, nullptr, nullptr, nullptr);
					if (!swsctx) {
						std::cerr << "Fail to sws_getCachedContext.";
						return;
					}

					/* Open the video */
					if (avcodec_open2(c, codec, nullptr) < 0) {
						fprintf(stderr, "Could not open codec\n");
						exit(1);
					}

					f = fopen(filename.str().c_str(), "wb");
					if (!f) {
						fprintf(stderr, "Could not open %s\n", filename.str().c_str());
					}

					// Adding cv_ptr->image to the new frame. (To be added ...)
					addNewFrame(timestamp);

				} else {
					ROS_INFO("%lu image stored", img_queue.size());
					little_sleep(Micro(10000));
				}
			}
		}
	}
    
    void setFps(long start_us, long end_us) {
		double framerate = config::MAX_FRAMES / ((end_us - start_us) / 1000000.0); 
		dst_fps.num = framerate * config::PRECISION;
		dst_fps.den = config::PRECISION;
	}

private:
	int cameraNum; // Camera id.
	int video_count = 0; // Count video (1000 images) recorded.
	int frame_count = 0; // Count frame encoded. (shall be equal to received_count)
	int buffer_size = config::MAX_FRAMES; // Maximum images in a buffer.
	int received_count = 0; // Count image received.
	int frame_pts = 0; // Presentation timestamp in time_base units.
	

	bool is_running = true; // Default it true

	bool is_recording = false; // Indicate camera status.

	bool threads_started = false; // Indicate when to start threads.
	// Indicators for stream
	bool end_of_stream = false;
	
	
	std::deque<sensor_msgs::Image> img_queue;
	std::deque<long> time_queue;
	std::vector<std::thread> thread_pool; // Thread pool for holding compressors. (currently only 1)
	std::mutex queue_mutex; // Control access.

	AVRational dst_fps;
	AVCodecContext *c;
	FILE *f;
	AVFrame* frame;
	AVCodec* codec;
	SwsContext* swsctx; /* Used to scale image and insert into frame */
	int got_output = 0; // Indicate if we got the packet.

	/* Creating directory for different camera */
	void createLocalDir(boost::filesystem::path &p) {
	    if (boost::filesystem::exists(p)) {
	      if (boost::filesystem::is_directory(p)) {
	      	printf("dir: %s exists. \n", p.string().c_str());
	      } else if (boost::filesystem::is_regular_file(p)) {
	        printf("file with same name exists. quitting.\n");
	      }
	    } else {
	      boost::system::error_code ec;
	      if (!boost::filesystem::create_directory(p, ec)) {
	        printf("%s\n", ec.message().c_str());
	      } else {
	        printf("Directory %s created.\n", p.string().c_str());
	      }
	    }
	}

};

/**
 * Each listener would have a compressor and subscribed to one camera thread.
 * Usage: new 6 listeners with 6 camera 
 *        (don't put them on stack, since ros::spin() will make them out of scope (ughhh)
 * 		  Then sit back and relax. Callback will call compressor to handle everything.
 */
class Listener {
public:
	Listener(ros::NodeHandle& nh, int num): cameraNum(num), compressor(num) {
		
		image_sender_name = "image_sender_";
		image_sender_name += std::to_string(num);

		sub = nh.subscribe(image_sender_name.c_str(), 1000, &Listener::imageSubscriberCallback, this);
	}

	void imageSubscriberCallback(const bfsdriver_1_1::ImageStamp &imageStamp) {
		
		ROS_INFO("Camera: %d: I received an image with %d height and %d width at %lu.", 
			cameraNum, imageStamp.img.height, imageStamp.img.width, imageStamp.timestamp);

		compressor.receive(imageStamp);
	}


private:
	std::string image_sender_name;
	ros::Subscriber sub;
	int cameraNum; // mark which camera it is listening to.
	Compressor compressor; 
};