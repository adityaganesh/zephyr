#include "icm20948_decoder.h"
#include "icm20948.h"

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = icm20948_decoder_get_frame_count,
	.get_size_info = icm20948_decoder_get_size_info,
	.decode = icm20948_decoder_decode,
	.has_trigger = icm20948_decoder_has_trigger,
};