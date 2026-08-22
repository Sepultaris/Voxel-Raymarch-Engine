#include "audio/audio_engine.hpp"

#include <AL/alc.h>

#include <iostream>

namespace voxel {

AudioEngine::AudioEngine() {
    device_ = alcOpenDevice(nullptr);
    if (device_ == nullptr) {
        std::cerr << "OpenAL: no playback device is available; audio is disabled.\n";
        return;
    }

    context_ = alcCreateContext(device_, nullptr);
    if (context_ == nullptr || alcMakeContextCurrent(context_) == ALC_FALSE) {
        std::cerr << "OpenAL: context creation failed; audio is disabled.\n";
        if (context_ != nullptr) {
            alcDestroyContext(context_);
            context_ = nullptr;
        }
        alcCloseDevice(device_);
        device_ = nullptr;
    }
}

AudioEngine::~AudioEngine() {
    if (context_ != nullptr) {
        alcMakeContextCurrent(nullptr);
        alcDestroyContext(context_);
    }
    if (device_ != nullptr) {
        alcCloseDevice(device_);
    }
}

} // namespace voxel

