// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "capture.hpp"

#include <spa/param/buffers.h>
#include <spa/param/video/format-utils.h>
#include <spa/pod/builder.h>

#include <cstdio>
#include <vector>

namespace rd {
namespace {

// BGRA first: an alpha-carrying format keeps the desktop opaque in the overlay.
constexpr spa_video_format kFormats[] = {SPA_VIDEO_FORMAT_BGRA, SPA_VIDEO_FORMAT_BGRx};

const spa_pod *
format_pod(spa_pod_builder *b, spa_video_format format, const std::vector<uint64_t> *modifiers, uint32_t max_fps)
{
	spa_rectangle def = SPA_RECTANGLE(1920, 1080), min = SPA_RECTANGLE(1, 1), max = SPA_RECTANGLE(16384, 16384);
	spa_fraction any = SPA_FRACTION(0, 1), low = SPA_FRACTION(1, 1), high = SPA_FRACTION(max_fps, 1);
	spa_pod_frame object, choice;
	spa_pod_builder_push_object(b, &object, SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat);
	spa_pod_builder_add(b, SPA_FORMAT_mediaType, SPA_POD_Id(SPA_MEDIA_TYPE_video), SPA_FORMAT_mediaSubtype,
	                    SPA_POD_Id(SPA_MEDIA_SUBTYPE_raw), SPA_FORMAT_VIDEO_format, SPA_POD_Id(format), 0);
	if (modifiers) {
		// The producer test-allocates and fixates one of these (PipeWire DMA-BUF negotiation).
		spa_pod_builder_prop(b, SPA_FORMAT_VIDEO_modifier,
		                     SPA_POD_PROP_FLAG_MANDATORY | SPA_POD_PROP_FLAG_DONT_FIXATE);
		spa_pod_builder_push_choice(b, &choice, SPA_CHOICE_Enum, 0);
		spa_pod_builder_long(b, int64_t(modifiers->front()));
		for (uint64_t m : *modifiers) {
			spa_pod_builder_long(b, int64_t(m));
		}
		spa_pod_builder_pop(b, &choice);
	}
	spa_pod_builder_add(b, SPA_FORMAT_VIDEO_size, SPA_POD_CHOICE_RANGE_Rectangle(&def, &min, &max),
	                    SPA_FORMAT_VIDEO_framerate, SPA_POD_Fraction(&any), SPA_FORMAT_VIDEO_maxFramerate,
	                    SPA_POD_CHOICE_RANGE_Fraction(&high, &low, &high), 0);
	return static_cast<const spa_pod *>(spa_pod_builder_pop(b, &object));
}

std::vector<const spa_pod *>
format_pods(spa_pod_builder *b, const std::vector<uint64_t> &modifiers, uint32_t max_fps)
{
	std::vector<const spa_pod *> pods;
	if (!modifiers.empty()) {
		for (spa_video_format f : kFormats) {
			pods.push_back(format_pod(b, f, &modifiers, max_fps));
		}
	}
	for (spa_video_format f : kFormats) {
		pods.push_back(format_pod(b, f, nullptr, max_fps));
	}
	return pods;
}

} // namespace

Capture::Capture(pw_core *core, Gpu &gpu, const std::string &target_object, uint32_t max_fps, FrameFn on_frame)
    : gpu_(gpu), on_frame_(std::move(on_frame)), max_fps_(max_fps)
{
	static const pw_stream_events events = {
	    .version = PW_VERSION_STREAM_EVENTS,
	    .state_changed = on_state,
	    .param_changed = on_param,
	    .add_buffer = on_add_buffer,
	    .remove_buffer = on_remove_buffer,
	    .process = on_process,
	};
	pw_properties *props = pw_properties_new(PW_KEY_MEDIA_TYPE, "Video", PW_KEY_MEDIA_CATEGORY, "Capture",
	                                         PW_KEY_MEDIA_ROLE, "Screen", PW_KEY_TARGET_OBJECT,
	                                         target_object.c_str(), nullptr);
	stream_ = pw_stream_new(core, "rifts-desktop", props);
	if (!stream_) {
		failed_ = true;
		return;
	}
	pw_stream_add_listener(stream_, &listener_, &events, this);

	uint8_t storage[8192];
	spa_pod_builder b = SPA_POD_BUILDER_INIT(storage, sizeof(storage));
	auto pods = format_pods(&b, gpu_.import_modifiers(), max_fps_);
	auto flags = pw_stream_flags(PW_STREAM_FLAG_AUTOCONNECT | PW_STREAM_FLAG_MAP_BUFFERS);
	if (pw_stream_connect(stream_, PW_DIRECTION_INPUT, PW_ID_ANY, flags, pods.data(), uint32_t(pods.size())) < 0) {
		failed_ = true;
	}
}

Capture::~Capture()
{
	if (stream_) {
		pw_stream_destroy(stream_); // runs remove_buffer for every buffer
	}
	gpu_.destroy(shm_image_);
}

void
Capture::offer_formats(bool with_dmabuf)
{
	uint8_t storage[8192];
	spa_pod_builder b = SPA_POD_BUILDER_INIT(storage, sizeof(storage));
	auto pods = format_pods(&b, with_dmabuf ? gpu_.import_modifiers() : std::vector<uint64_t>{}, max_fps_);
	pw_stream_update_params(stream_, pods.data(), uint32_t(pods.size()));
}

void
Capture::on_state(void *data, pw_stream_state, pw_stream_state state, const char *error)
{
	auto *self = static_cast<Capture *>(data);
	if (state == PW_STREAM_STATE_ERROR || state == PW_STREAM_STATE_UNCONNECTED) {
		fprintf(stderr, "rifts-desktop: stream %s%s%s\n", pw_stream_state_as_string(state), error ? ": " : "",
		        error ? error : "");
		self->failed_ = true;
	}
}

void
Capture::on_param(void *data, uint32_t id, const spa_pod *param)
{
	auto *self = static_cast<Capture *>(data);
	if (!param || id != SPA_PARAM_Format) {
		return;
	}
	spa_video_info_raw info = {};
	if (spa_format_video_raw_parse(param, &info) < 0 || info.size.width == 0 || info.size.height == 0) {
		self->failed_ = true;
		return;
	}
	self->width_ = info.size.width;
	self->height_ = info.size.height;
	self->dmabuf_ = !self->dmabuf_failed_ && spa_pod_find_prop(param, nullptr, SPA_FORMAT_VIDEO_modifier) != nullptr;
	self->modifier_ = info.modifier;
	fprintf(stderr, "rifts-desktop: stream %ux%u %s %s modifier 0x%llx\n", info.size.width, info.size.height,
	        info.format == SPA_VIDEO_FORMAT_BGRA ? "BGRA" : "BGRx", self->dmabuf_ ? "dmabuf" : "shm",
	        static_cast<unsigned long long>(info.modifier));

	uint8_t storage[512];
	spa_pod_builder b = SPA_POD_BUILDER_INIT(storage, sizeof(storage));
	int types = self->dmabuf_ ? 1 << SPA_DATA_DmaBuf : (1 << SPA_DATA_MemFd) | (1 << SPA_DATA_MemPtr);
	const spa_pod *buffers = static_cast<const spa_pod *>(spa_pod_builder_add_object(
	    &b, SPA_TYPE_OBJECT_ParamBuffers, SPA_PARAM_Buffers, SPA_PARAM_BUFFERS_buffers,
	    SPA_POD_CHOICE_RANGE_Int(4, 2, 8), SPA_PARAM_BUFFERS_dataType, SPA_POD_CHOICE_FLAGS_Int(types)));
	pw_stream_update_params(self->stream_, &buffers, 1);
}

void
Capture::on_add_buffer(void *data, pw_buffer *buffer)
{
	auto *self = static_cast<Capture *>(data);
	if (self->dmabuf_ && buffer->buffer->datas[0].type == SPA_DATA_DmaBuf) {
		buffer->user_data = new GpuImage{}; // imported on first use, when chunk layout is final
	}
}

void
Capture::on_remove_buffer(void *data, pw_buffer *buffer)
{
	auto *self = static_cast<Capture *>(data);
	if (auto *image = static_cast<GpuImage *>(buffer->user_data)) {
		self->gpu_.destroy(*image);
		delete image;
		buffer->user_data = nullptr;
	}
}

void
Capture::on_process(void *data)
{
	auto *self = static_cast<Capture *>(data);
	// Only the newest frame matters; older queued ones go straight back.
	pw_buffer *buffer = nullptr;
	while (pw_buffer *next = pw_stream_dequeue_buffer(self->stream_)) {
		if (buffer) {
			pw_stream_queue_buffer(self->stream_, buffer);
		}
		buffer = next;
	}
	if (!buffer) {
		return;
	}
	spa_buffer *sb = buffer->buffer;
	if (sb->n_datas == 0 || sb->n_datas > 4) {
		pw_stream_queue_buffer(self->stream_, buffer);
		return;
	}
	spa_data &d = sb->datas[0];
	if (!d.chunk || (d.chunk->flags & SPA_CHUNK_FLAG_CORRUPTED)) {
		pw_stream_queue_buffer(self->stream_, buffer);
		return;
	}

	if (auto *image = static_cast<GpuImage *>(buffer->user_data)) {
		if (!image->image) {
			DmabufPlane planes[4];
			uint32_t count = sb->n_datas;
			for (uint32_t i = 0; i < count; i++) {
				if (!sb->datas[i].chunk || sb->datas[i].chunk->stride <= 0) {
					pw_stream_queue_buffer(self->stream_, buffer);
					return;
				}
				planes[i] = {int(sb->datas[i].fd), sb->datas[i].mapoffset + sb->datas[i].chunk->offset,
				             uint32_t(sb->datas[i].chunk->stride)};
			}
			if (!self->gpu_.import_dmabuf(*image, self->width_, self->height_, self->modifier_, planes, count)) {
				fprintf(stderr, "rifts-desktop: DMA-BUF import failed, falling back to SHM\n");
				self->dmabuf_failed_ = true;
				pw_stream_queue_buffer(self->stream_, buffer);
				self->offer_formats(false);
				return;
			}
		}
		if (self->gpu_.begin(*image)) {
			self->on_frame_(*image);
			if (!self->gpu_.end(*image)) self->failed_ = true;
		} else self->failed_ = true;
	} else if (d.data && d.chunk->size > 0) {
		uint32_t stride = d.chunk->stride > 0 ? uint32_t(d.chunk->stride) : self->width_ * 4;
		uint64_t needed = uint64_t(self->height_ - 1) * stride + uint64_t(self->width_) * 4;
		if (stride < self->width_ * 4 || d.chunk->offset > d.maxsize || needed > d.maxsize - d.chunk->offset) {
			pw_stream_queue_buffer(self->stream_, buffer);
			return;
		}
		const auto *pixels = static_cast<const uint8_t *>(d.data) + d.chunk->offset;
		if (self->gpu_.upload(self->shm_image_, self->width_, self->height_, pixels, stride)) {
			self->on_frame_(self->shm_image_);
		}
	}
	pw_stream_queue_buffer(self->stream_, buffer);
}

} // namespace rd
