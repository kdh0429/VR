#include "ricohStream.h"
#include <iostream>



using namespace std;

ricohRos::ricohRos() :ricohContext(NULL) {
		av_init_packet(&ricohPacket_l);
		av_init_packet(&ricohPacket_r);
		avcodec_register_all();
		av_log_set_level(AV_LOG_FATAL);
		codec = avcodec_find_decoder(AV_CODEC_ID_H264);

		if (!codec) {
			cout << "cannot find according codec in ffmpeg" << endl;
			return;
		}
		c_l = avcodec_alloc_context3(codec);
		c_r = avcodec_alloc_context3(codec);
		if (avcodec_open2(c_l, codec, 0) < 0 || avcodec_open2(c_r, codec, 0) < 0) {
			cout << "cannot open codec" << endl;
			return;
		}
		
		c_l->thread_count = 8;
		c_r->thread_count = 8;

		ricohleftFrame = av_frame_alloc();
		ricohrightFrame = av_frame_alloc();
}

ricohRos::~ricohRos() {
			avcodec_close(c_l);
			avcodec_close(c_r);
			av_free(c_l);
			av_free(c_r);
			av_frame_free(&ricohleftFrame);
			av_frame_free(&ricohrightFrame);
}

