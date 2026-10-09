#include "openvr.h"
#include <dlfcn.h>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

// Only Background clients. No scene submission, GL mirror or driver mutation.
namespace {
void *library = nullptr;
vr::IVRSystem *vr_system = nullptr;
vr::IVRCompositor *compositor = nullptr;
vr::IVRChaperone *chaperone = nullptr;
vr::IVRChaperoneSetup *setup = nullptr;
vr::IVROverlay *overlay = nullptr;
vr::IVRTrackedCamera *camera = nullptr;
vr::IVRExtendedDisplay *display = nullptr;
vr::TrackedCameraHandle_t camera_handle = 0;
using Clock = std::chrono::steady_clock;
Clock::time_point previous_time;
uint32_t previous_sequence = 0;
bool previous_frame = false;
std::string output;
using GetInterface = void *(*)(const char *, vr::EVRInitError *);
template<typename T> T *get(GetInterface fn, const char *version) {
    vr::EVRInitError error = vr::VRInitError_None;
    return static_cast<T *>(fn(version, &error));
}
bool connect(const char *path) {
    if (vr_system) return true;
    if (!library) library = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if (!library) return false;
    auto init = reinterpret_cast<uint32_t (*)(vr::EVRInitError *, vr::EVRApplicationType)>(dlsym(library, "VR_InitInternal"));
    auto generic = reinterpret_cast<GetInterface>(dlsym(library, "VR_GetGenericInterface"));
    if (!init || !generic) return false;
    vr::EVRInitError error = vr::VRInitError_None;
    init(&error, vr::VRApplication_Background);
    if (error != vr::VRInitError_None) return false;
    vr_system = get<vr::IVRSystem>(generic, vr::IVRSystem_Version);
    compositor = get<vr::IVRCompositor>(generic, vr::IVRCompositor_Version);
    chaperone = get<vr::IVRChaperone>(generic, vr::IVRChaperone_Version);
    setup = get<vr::IVRChaperoneSetup>(generic, vr::IVRChaperoneSetup_Version);
    overlay = get<vr::IVROverlay>(generic, vr::IVROverlay_Version);
    camera = get<vr::IVRTrackedCamera>(generic, vr::IVRTrackedCamera_Version);
    display = get<vr::IVRExtendedDisplay>(generic, vr::IVRExtendedDisplay_Version);
    return vr_system;
}
template<typename T> void number(std::ostream &o, T v, bool valid = true) {
    if (valid && std::isfinite(static_cast<double>(v))) o << v; else o << "null";
}
void points(std::ostream &o, const vr::HmdVector3_t *p, unsigned count) {
    o << '[';
    for (unsigned i=0; i<count; ++i) {
        if (i) o << ',';
        o << '['; number(o,p[i].v[0]); o << ','; number(o,p[i].v[1]); o << ','; number(o,p[i].v[2]); o << ']';
    }
    o << ']';
}
}

extern "C" const char *rifts_snapshot(const char *path) {
    if (!connect(path)) return "{}";
    std::ostringstream o;
    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
    vr_system->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, poses, vr::k_unMaxTrackedDeviceCount);
    o << "{\"devices\":[";
    vr::TrackedDeviceIndex_t ids[] = {0,
        vr_system->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_LeftHand),
        vr_system->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand)};
    // Sleeping controllers can lose their assigned role. The driver retains
    // its role hint, so their real disconnected/untracked state stays visible.
    if(ids[1]==vr::k_unTrackedDeviceIndexInvalid || ids[2]==vr::k_unTrackedDeviceIndexInvalid) {
        for(uint32_t index=1;index<vr::k_unMaxTrackedDeviceCount;++index) {
            if(vr_system->GetTrackedDeviceClass(index)!=vr::TrackedDeviceClass_Controller) continue;
            vr::ETrackedPropertyError error;
            int role=vr_system->GetInt32TrackedDeviceProperty(index,vr::Prop_ControllerRoleHint_Int32,&error);
            if(error==vr::TrackedProp_Success && (role==vr::TrackedControllerRole_LeftHand || role==vr::TrackedControllerRole_RightHand)) {
                unsigned slot=role==vr::TrackedControllerRole_LeftHand ? 1:2;
                if(ids[slot]==vr::k_unTrackedDeviceIndexInvalid) ids[slot]=index;
            }
        }
    }
    for (unsigned i=0;i<3;++i) {
        if(i) o << ',';
        auto id=ids[i];
        bool valid=id<vr::k_unMaxTrackedDeviceCount;
        const char *tracking=nullptr;
        if (valid && vr_system->GetTrackedDeviceClass(id) == (i ? vr::TrackedDeviceClass_Controller : vr::TrackedDeviceClass_HMD)) {
            const auto &pose=poses[id];
            if (!pose.bDeviceIsConnected) tracking="Disconnected";
            else if (!pose.bPoseIsValid) tracking="Untracked";
            else if (pose.eTrackingResult==vr::TrackingResult_Running_OK) tracking="6DoF";
            else if (pose.eTrackingResult==vr::TrackingResult_Fallback_RotationOnly) tracking="3DoF";
            else tracking="Limited";
        }
        o << "{\"tracking\":";
        if(tracking) o << '"' << tracking << '"'; else o << "null";
        vr::ETrackedPropertyError err=vr::TrackedProp_UnknownProperty;
        float battery=valid ? vr_system->GetFloatTrackedDeviceProperty(id,vr::Prop_DeviceBatteryPercentage_Float,&err) : 0;
        o << ",\"battery\":"; number(o,battery,err==vr::TrackedProp_Success && battery>=0 && battery<=1); o << '}';
    }
    o << "],\"refresh\":";
    vr::ETrackedPropertyError err;
    float refresh=vr_system->GetFloatTrackedDeviceProperty(0,vr::Prop_DisplayFrequency_Float,&err);
    number(o,refresh,err==vr::TrackedProp_Success);
    o << ",\"display\":";
    int width=vr_system->GetInt32TrackedDeviceProperty(0,vr::Prop_DisplayMCImageWidth_Int32,&err);
    bool has_width=err==vr::TrackedProp_Success && width>0;
    int height=vr_system->GetInt32TrackedDeviceProperty(0,vr::Prop_DisplayMCImageHeight_Int32,&err);
    if(has_width && err==vr::TrackedProp_Success && height>0) o << '[' << width << ',' << height << ']';
    else if(display) {
        int32_t x=0,y=0; uint32_t w=0,h=0; display->GetWindowBounds(&x,&y,&w,&h);
        if(w && h) o << '[' << w << ',' << h << ']'; else o << "null";
    } else o << "null";
    vr::Compositor_FrameTiming timing{}; timing.m_nSize=sizeof(timing);
    bool timed=compositor && compositor->GetFrameTiming(&timing);
    o << ",\"frameTime\":"; number(o,timing.m_flTotalRenderGpuMs,timed);
    o << ",\"dropped\":"; number(o,timing.m_nNumDroppedFrames,timed);
    float ipd=vr_system->GetFloatTrackedDeviceProperty(0,vr::Prop_UserIpdMeters_Float,&err);
    o << ",\"ipd\":"; number(o,ipd*1000,err==vr::TrackedProp_Success);
    o << ",\"proximity\":";
    vr::VRControllerState_t state{};
    if(vr_system->GetControllerState(0,&state,sizeof(state))) o << ((state.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_ProximitySensor)) ? "true":"false"); else o << "null";
    o << ",\"idle\":" << (poses[0].bPoseIsValid && std::abs(poses[0].vVelocity.v[0])+std::abs(poses[0].vVelocity.v[1])+std::abs(poses[0].vVelocity.v[2])<0.02 && std::abs(poses[0].vAngularVelocity.v[0])+std::abs(poses[0].vAngularVelocity.v[1])+std::abs(poses[0].vAngularVelocity.v[2])<0.05 ? "true":"false");
    vr::HmdQuad_t rect{};
    o << ",\"rect\":";
    if(chaperone && chaperone->GetPlayAreaRect(&rect)) points(o,rect.vCorners,4); else o << "null";
    o << ",\"walls\":[";
    uint32_t count=0;
    if(setup) setup->GetLiveCollisionBoundsInfo(nullptr,&count);
    if(setup && count<4096) {
        std::vector<vr::HmdQuad_t> walls(count);
        if(count && setup->GetLiveCollisionBoundsInfo(walls.data(),&count))
            for(uint32_t i=0;i<count;++i) { if(i) o << ','; points(o,walls[i].vCorners,4); }
    }
    o << "],\"floor\":";
    auto m=vr_system->GetRawZeroPoseToStandingAbsoluteTrackingPose();
    number(o,-(m.m[0][1]*m.m[0][3]+m.m[1][1]*m.m[1][3]+m.m[2][1]*m.m[2][3]),chaperone && chaperone->GetCalibrationState()==vr::ChaperoneCalibrationState_OK);
    o << ",\"head\":";
    if(poses[0].bPoseIsValid) o << '[' << poses[0].mDeviceToAbsoluteTracking.m[0][3] << ',' << poses[0].mDeviceToAbsoluteTracking.m[2][3] << ']'; else o << "null";
    bool has_camera=false;
    if(camera) camera->HasCamera(0,&has_camera);
    vr::CameraVideoStreamFrameHeader_t frame{};
    bool fresh=false; double rate=0;
    if(has_camera && !camera_handle) camera->AcquireVideoStreamingService(0,&camera_handle);
    if(camera_handle && camera->GetVideoStreamFrameBuffer(camera_handle,vr::VRTrackedCameraFrameType_Undistorted,nullptr,0,&frame,sizeof(frame))==vr::VRTrackedCameraError_None) {
        auto now=Clock::now();
        double seconds=std::chrono::duration<double>(now-previous_time).count();
        fresh=previous_frame && frame.nFrameSequence!=previous_sequence;
        if(fresh && seconds>0 && frame.nFrameSequence>previous_sequence) rate=(frame.nFrameSequence-previous_sequence)/seconds;
        previous_sequence=frame.nFrameSequence; previous_time=now; previous_frame=true;
    } else previous_frame=false;
    o << ",\"cameraExport\":" << (has_camera ? "true":"false") << ",\"cameraActive\":" << (fresh ? "true":"false") << ",\"cameraRate\":"; number(o,rate,fresh && rate>0 && rate<100);
    o << '}'; output=o.str(); return output.c_str();
}

extern "C" int rifts_action(int action) {
    if(!vr_system) return 0;
    if(action==1 && compositor) { compositor->ShowMirrorWindow(); return 1; }
    if(action==2 && overlay) { overlay->ShowDashboard(""); return 1; }
    if(action==3 && chaperone) { chaperone->ResetZeroPose(vr::TrackingUniverseStanding); return 1; }
    return 0;
}
extern "C" void rifts_shutdown() {
    if(camera_handle && camera) camera->ReleaseVideoStreamingService(camera_handle);
    camera_handle=0; camera=nullptr; previous_frame=false;
    if(library) { auto shutdown=reinterpret_cast<void (*)()>(dlsym(library,"VR_ShutdownInternal")); if(shutdown) shutdown(); dlclose(library); }
    library=nullptr; vr_system=nullptr; compositor=nullptr; chaperone=nullptr; setup=nullptr; overlay=nullptr; display=nullptr;
}
