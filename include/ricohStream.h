#ifndef ricohStream_h
#define ricohStream_h


extern "C" {
#include "libswscale/swscale.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavcodec/avcodec.h"
#include "libavutil/mathematics.h"
#include "libavutil/samplefmt.h"
}


class ricohRos {

	public:
		ricohRos();
		~ricohRos();

		SwsContext* ricohContext = NULL;

		AVCodec* codec;
		AVCodecContext* c_l;
		AVCodecContext* c_r;
		AVFrame* ricohleftFrame;
		AVFrame* ricohrightFrame;

		AVPacket ricohPacket_l;
		AVPacket ricohPacket_r;

};

#endif