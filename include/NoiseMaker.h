#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdlib.h>
#include <string>
#include <thread>

#include <Windows.h>

// TODO: need to find a better runtime solution for this
using tString = std::basic_string<TCHAR>;

constexpr double PI = 3.14159265358979;

namespace Utils
{
template <class T>
T atomic_fetch_add(std::atomic<T>& obj, T arg,
                   const std::memory_order success = std::memory_order_seq_cst,
                   const std::memory_order failure = std::memory_order_seq_cst)
{
    T expected = obj.load(std::memory_order_relaxed);
    while (
        !obj.compare_exchange_weak(expected, expected + arg, success, failure))
    {
    }
    return expected;
}

} // namespace Utils

template <class T>
class NoiseMaker
{
  public:
    NoiseMaker(const tString& outputDevice, unsigned sampleRate = 44100,
               unsigned numChannels = 1, unsigned numBlocks = 8,
               unsigned numBlockSamples = 256)
    {
        Create(outputDevice, sampleRate, numChannels, numBlocks,
               numBlockSamples);
    }

    ~NoiseMaker() { Destroy(); }

    void Stop()
    {
        mReady.store(false);
        mThread.join();
    }

    double GetTime() const { return mGlobalTime.load(); }

    static std::vector<tString> Enumerate()
    {
        const unsigned deviceCount = waveOutGetNumDevs();
        std::vector<tString> devices;
        WAVEOUTCAPS woc;
        for (unsigned n = 0; n < deviceCount; ++n)
        {
            if (waveOutGetDevCaps(n, &woc, sizeof(WAVEOUTCAPS)) ==
                MMSYSERR_NOERROR)
            {
                devices.emplace_back(woc.szPname);
            }
        }
        return devices;
    }

    void SetUserFunction(std::function<double(unsigned, double)> func)
    {
        mUserFunction = func;
    }

  private:
    bool Create(const tString& outputDevice, const unsigned sampleRate = 44100,
                const unsigned numChannels = 1, const unsigned numBlocks = 8,
                const unsigned numSamplesPerBlock = 256)
    {
        mReady.store(false);
        mSampleRate = sampleRate;
        mNumChannels = numChannels;
        mNumBlocks = numBlocks;
        mSamplesPerBlock = numSamplesPerBlock;
        mBlockFree.store(mNumBlocks);
        mCurrentBlock = 0;

        // Allocate Wave|Block Memory
        mBlockMemoryVec.resize(mNumBlocks * mSamplesPerBlock);
        mWaveHeadersVec.resize(mNumBlocks);

        // Validate Device
        const auto devices = Enumerate();
        const auto device =
            std::find(std::cbegin(devices), std::cend(devices), outputDevice);
        if (device != std::cend(devices))
        {
            // Device is available
            const auto deviceID =
                static_cast<UINT>(std::distance(std::cbegin(devices), device));
            WAVEFORMATEX waveFormat;
            waveFormat.wFormatTag = WAVE_FORMAT_PCM;
            waveFormat.nSamplesPerSec = mSampleRate;
            waveFormat.wBitsPerSample = sizeof(T) * 8;
            waveFormat.nChannels = mNumChannels;
            waveFormat.nBlockAlign =
                (waveFormat.wBitsPerSample / 8) * waveFormat.nChannels;
            waveFormat.nAvgBytesPerSec =
                waveFormat.nSamplesPerSec * waveFormat.nBlockAlign;
            waveFormat.cbSize = 0;

            // Open Device if valid
            if (waveOutOpen(&mHWDevice, deviceID, &waveFormat,
                            reinterpret_cast<DWORD_PTR>(WaveOutProcWrap),
                            reinterpret_cast<DWORD_PTR>(this),
                            CALLBACK_FUNCTION) != S_OK)
            {
                return Destroy();
            }
        }
        else
        {
            return Destroy();
        }

        // Link headers to block memory
        for (unsigned n = 0; n < mNumBlocks; ++n)
        {
            mWaveHeadersVec[n].dwBufferLength = mSamplesPerBlock * sizeof(T);
            mWaveHeadersVec[n].lpData = reinterpret_cast<LPSTR>(
                mBlockMemoryVec.data() + (n * mSamplesPerBlock));
        }

        mReady.store(true);
        mThread = std::thread(&NoiseMaker::MainThread, this);

        // Start the ball rolling
        std::unique_lock<std::mutex> lm(mMuxBlockNotZero);
        mCVBlockNotZero.notify_one();

        return true;
    }

    bool Destroy() { return false; }

    double Clip(const double sample, const double max)
    {
        if (sample >= 0.0)
            return std::fmin(sample, max);
        else
            return std::fmax(sample, -max);
    }

    // Override to process current sample
    virtual double UserProcess(const unsigned channel, const double time)
    {
        (void)channel;
        (void)time;

        // return no sound
        return 0.0;
    }

    // this call back can tell if sound card is done with current block
    // Handler for soundcard request for more data
    void WaveOutProc(HWAVEOUT waveOut, UINT msg, DWORD_PTR param1,
                     DWORD_PTR param2)
    {
        if (msg != WOM_DONE)
            return;

        mBlockFree.fetch_add(1);
        // std::unique_lock<std::mutex> lm(mMuxBlockNotZero);
        // mCVBlockNotZero.notify_one();
    }

    // static wrapper for sound card handler
    static void CALLBACK WaveOutProcWrap(HWAVEOUT waveOut, UINT msg,
                                         DWORD_PTR instance, DWORD_PTR param1,
                                         DWORD_PTR param2)
    {
        reinterpret_cast<NoiseMaker*>(instance)->WaveOutProc(waveOut, msg,
                                                             param1, param2);
    }

    // Main thread. This loop responds to requests from the soundcard to fill
    // 'blocks' with audio data. If no requests are available it foes dormant
    // untill the sound card is ready for more data. The block is filled by the
    // the 'user' in some manner and then issued to the soundcard.
    void MainThread()
    {
        // Note: Do not (de)allocate memory in this thread

        mGlobalTime.store(0.0);
        const double timeStep = 1.0 / (double)mSampleRate;

        constexpr double maxSample =
            static_cast<double>((std::numeric_limits<T>::max)());

        while (mReady.load())
        {
            // Wait for a block to become available
            // if (mBlockFree.load() == 0)
            // {
            //     // std::unique_lock<std::mutex> lm(mMuxBlockNotZero);
            //     // while (mBlockFree.load() == 0) // sometimes, Windows
            //     signals incorrectly
            //     // mCVBlockNotZero.wait(lm);
            // }
            while (mBlockFree.load() == 0)
            {
            }

            // Block is here, so use it
            mBlockFree.fetch_sub(1);

            // Prepare block for processing
            if (mWaveHeadersVec[mCurrentBlock].dwFlags & WHDR_PREPARED)
            {
                waveOutUnprepareHeader(mHWDevice,
                                       &mWaveHeadersVec[mCurrentBlock],
                                       sizeof(WAVEHDR));
            }

            const int curBlock = mCurrentBlock * mSamplesPerBlock;

            for (unsigned n = 0; n < mSamplesPerBlock; n += mNumChannels)
            {
                // User Process
                for (unsigned c = 0; c < mNumChannels; ++c)
                {
                    const T newSample = [&]() -> T {
                        if (!mUserFunction)
                        {
                            return static_cast<T>(
                                Clip(UserProcess(c, mGlobalTime), 1.0) *
                                maxSample);
                        }
                        else
                        {
                            return static_cast<T>(
                                Clip(mUserFunction(c, mGlobalTime), 1.0) *
                                maxSample);
                        }
                    }();

                    mBlockMemoryVec[curBlock + n] = newSample;
                }

                Utils::atomic_fetch_add<double>(mGlobalTime, timeStep);
            }

            // Send block to sound device
            waveOutPrepareHeader(mHWDevice, &mWaveHeadersVec[mCurrentBlock],
                                 sizeof(WAVEHDR));
            waveOutWrite(mHWDevice, &mWaveHeadersVec[mCurrentBlock],
                         sizeof(WAVEHDR));

            mCurrentBlock++;
            mCurrentBlock %= mNumBlocks;
        }
    }

    // double (*mUserFunction)(unsigned, double);
    std::function<double(unsigned, double)> mUserFunction;

    unsigned mSampleRate;
    unsigned mNumChannels;
    unsigned mNumBlocks;
    unsigned mSamplesPerBlock;
    unsigned mCurrentBlock;

    std::vector<T> mBlockMemoryVec;
    std::vector<WAVEHDR> mWaveHeadersVec;
    HWAVEOUT mHWDevice;

    std::thread mThread;
    std::atomic<bool> mReady;
    std::atomic<unsigned> mBlockFree;
    std::condition_variable mCVBlockNotZero;
    std::mutex mMuxBlockNotZero;

    std::atomic<double> mGlobalTime;
};
