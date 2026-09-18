#ifndef UVCGPUPREVIEWRENDERER_H_
#define UVCGPUPREVIEWRENDERER_H_

#include <stdint.h>
#include <android/native_window.h>
#include "libuvc/libuvc.h"

class UVCGpuPreviewRenderer {
public:
	UVCGpuPreviewRenderer();
	~UVCGpuPreviewRenderer();

	bool render(uvc_frame_t *frame, ANativeWindow *window, uint64_t *frame_ready_ns);
	/** After rendering a frame whose planes are AHardwareBuffers: a native
	 * fence fd that signals when the GPU has finished reading them, or -1.
	 * Ownership passes to the caller (hand it to AHardwareBuffer_lock). */
	int takeRenderFenceFd();
	/** 3x3 column-major NDC transform applied to the quad (rotation, flip,
	 * zoom, pan).  Called under the same lock as render(). */
	void setTransform(const float m[9]);
	/** Re-read the surface size on the next draw (the window may have resized). */
	void invalidateSurfaceSize();
	void resetSurface();
	void shutdown();

private:
	struct Impl;
	Impl *impl;
};

#endif /* UVCGPUPREVIEWRENDERER_H_ */
