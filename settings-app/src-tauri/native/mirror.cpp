#define GL_GLEXT_PROTOTYPES
#include "openvr.h"
#include <GL/gl.h>
#include <GL/glx.h>
#include <X11/Xlib.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {
Display *display = nullptr;
GLXContext context = nullptr;
GLXPbuffer surface = 0;
GLuint shared_texture = 0, scaled_texture = 0, read_fbo = 0, draw_fbo = 0;
vr::glSharedTextureHandle_t shared_handle = nullptr;
std::vector<uint8_t> pixels;
int scaled_width = 0, scaled_height = 0;

bool make_current()
{
    if (!display) {
        XInitThreads();
        display = XOpenDisplay(nullptr);
        if (!display) return false;
        const int attributes[] = {GLX_DRAWABLE_TYPE, GLX_PBUFFER_BIT, GLX_RENDER_TYPE, GLX_RGBA_BIT,
                                  GLX_RED_SIZE, 8, GLX_GREEN_SIZE, 8, GLX_BLUE_SIZE, 8, None};
        int count = 0;
        auto *configs = glXChooseFBConfig(display, DefaultScreen(display), attributes, &count);
        if (configs && count) {
            context = glXCreateNewContext(display, configs[0], GLX_RGBA_TYPE, nullptr, True);
            const int size[] = {GLX_PBUFFER_WIDTH, 16, GLX_PBUFFER_HEIGHT, 16, None};
            surface = glXCreatePbuffer(display, configs[0], size);
        }
        if (configs) XFree(configs);
    }
    return context && surface && glXMakeContextCurrent(display, surface, surface, context);
}
void release_current()
{
    glXMakeContextCurrent(display, None, None, nullptr);
}
}

// Called under the bridge mutex. Context ownership is released between calls,
// so Tauri's command workers can use the same native OpenVR mirror safely.
extern "C" const uint8_t *rifts_mirror_frame(vr::IVRCompositor *compositor, uint32_t *size)
{
    *size = 0;
    if (!compositor || !make_current()) return nullptr;
    if (!shared_texture && compositor->GetMirrorTextureGL(vr::Eye_Left, &shared_texture, &shared_handle) !=
                               vr::VRCompositorError_None) {
        release_current();
        return nullptr;
    }
    compositor->LockGLSharedTextureForAccess(shared_handle);
    glBindTexture(GL_TEXTURE_2D, shared_texture);
    GLint width = 0, height = 0;
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);
    bool ok = width > 0 && height > 0 && width <= 8192 && height <= 8192;
    if (ok) {
        const int output_width = std::min(width, 512);
        const int output_height = std::clamp(height * output_width / width, 1, 720);
        if (!scaled_texture) glGenTextures(1, &scaled_texture);
        glBindTexture(GL_TEXTURE_2D, scaled_texture);
        if (scaled_width != output_width || scaled_height != output_height) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, output_width, output_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
            scaled_width = output_width; scaled_height = output_height;
        }
        if (!read_fbo) glGenFramebuffers(1, &read_fbo);
        if (!draw_fbo) glGenFramebuffers(1, &draw_fbo);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, read_fbo);
        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, shared_texture, 0);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, draw_fbo);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, scaled_texture, 0);
        ok = glCheckFramebufferStatus(GL_READ_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE &&
             glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
        if (ok) {
            glBlitFramebuffer(0, 0, width, height, 0, output_height, output_width, 0, GL_COLOR_BUFFER_BIT, GL_LINEAR);
            glBindFramebuffer(GL_READ_FRAMEBUFFER, draw_fbo);
            pixels.resize(8 + output_width * output_height * 4);
            const uint32_t dimensions[] = {static_cast<uint32_t>(output_width), static_cast<uint32_t>(output_height)};
            std::memcpy(pixels.data(), dimensions, sizeof(dimensions));
            glReadPixels(0, 0, output_width, output_height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data() + 8);
            ok = glGetError() == GL_NO_ERROR;
        }
    }
    compositor->UnlockGLSharedTextureForAccess(shared_handle);
    release_current();
    if (!ok) return nullptr;
    *size = static_cast<uint32_t>(pixels.size());
    return pixels.data();
}

extern "C" void rifts_mirror_shutdown(vr::IVRCompositor *compositor)
{
    if (!display) return;
    if (make_current()) {
        if (shared_texture && compositor) compositor->ReleaseSharedGLTexture(shared_texture, shared_handle);
        glDeleteFramebuffers(1, &read_fbo);
        glDeleteFramebuffers(1, &draw_fbo);
        glDeleteTextures(1, &scaled_texture);
        release_current();
    }
    shared_texture = scaled_texture = read_fbo = draw_fbo = 0;
    shared_handle = nullptr;
    if (surface) glXDestroyPbuffer(display, surface);
    if (context) glXDestroyContext(display, context);
    if (display) XCloseDisplay(display);
    surface = 0; context = nullptr; display = nullptr;
    scaled_width = scaled_height = 0;
    pixels.clear();
}
