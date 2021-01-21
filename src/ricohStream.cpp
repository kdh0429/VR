#include "ricohStream.h"
#include <iostream>



using namespace std;

ricohRos::ricohRos() :ricohContext(NULL) {
		av_init_packet(&ricohPacket);
		avcodec_register_all();
		av_log_set_level(AV_LOG_FATAL);
		codec = avcodec_find_decoder(AV_CODEC_ID_H264);

		if (!codec) {
			cout << "cannot find according codec in ffmpeg" << endl;
			return;
		}
		c = avcodec_alloc_context3(codec);
		if (avcodec_open2(c, codec, 0) < 0) {
			cout << "cannot open codec" << endl;
			return;
		}
		ricohleftFrame = av_frame_alloc();
		ricohrightFrame = av_frame_alloc();
}

ricohRos::~ricohRos() {
			avcodec_close(c);
			av_free(c);
			av_frame_free(&ricohleftFrame);
			av_frame_free(&ricohrightFrame);
}

