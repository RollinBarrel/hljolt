// this is Jolt's (https://github.com/jrouwe/JoltPhysics) JobSystemThreadPool
// rewritten to use pthread to allow changing the stack size

#pragma once

#include <Jolt/Core/JobSystemWithBarrier.h>
#include <Jolt/Core/FixedSizeFreeList.h>
#include <Jolt/Core/Semaphore.h>

#include <pthread.h>

using namespace JPH;

class JobSystemPThread;

class PTThread {
public:
    pthread_t handle;
    int index;
    JobSystemPThread* jobSystem;

    PTThread(int index, JobSystemPThread* system);
    ~PTThread();

    bool joinable();
    void join();
};

class JobSystemPThread final : public JobSystemWithBarrier
{
public:
	JPH_OVERRIDE_NEW_DELETE

    JobSystemPThread(uint inMaxJobs, uint inMaxBarriers, int inNumThreads = -1);
    JobSystemPThread() = default;
	virtual	~JobSystemPThread() override;

	using InitExitFunction = function<void(int)>;
	void SetThreadInitFunction(const InitExitFunction &inInitFunction)	{ mThreadInitFunction = inInitFunction; }
	void SetThreadExitFunction(const InitExitFunction &inExitFunction)	{ mThreadExitFunction = inExitFunction; }

	void Init(uint inMaxJobs, uint inMaxBarriers, int inNumThreads = -1);

	virtual int GetMaxConcurrency() const override { return int(mThreads.size()) + 1; }
	virtual JobHandle CreateJob(const char *inName, ColorArg inColor, const JobFunction &inJobFunction, uint32 inNumDependencies = 0) override;

	void SetNumThreads(int inNumThreads) { StopThreads(); StartThreads(inNumThreads); }

	void ThreadMain(int inThreadIndex);

protected:
	virtual void QueueJob(Job *inJob) override;
	virtual void QueueJobs(Job **inJobs, uint inNumJobs) override;
	virtual void FreeJob(Job *inJob) override;

private:
	void StartThreads(int inNumThreads);
	void StopThreads();

	inline uint GetHead() const;

	inline void QueueJobInternal(Job *inJob);

	InitExitFunction mThreadInitFunction;
	InitExitFunction mThreadExitFunction;

	using AvailableJobs = FixedSizeFreeList<Job>;
	AvailableJobs mJobs;

	Array<PTThread> mThreads;

	static constexpr uint32 cQueueLength = 1024;
	static_assert(IsPowerOf2(cQueueLength));								// We do bit operations and require queue length to be a power of 2
	atomic<Job *> mQueue[cQueueLength];

	atomic<uint> * mHeads = nullptr;								///< Per executing thread the head of the current queue
	alignas(JPH_CACHE_LINE_SIZE) atomic<uint> mTail = 0;					///< Tail (write end) of the queue

	Semaphore mSemaphore;

	atomic<bool> mQuit = false;
};
