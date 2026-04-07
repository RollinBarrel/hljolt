// this is Jolt's (https://github.com/jrouwe/JoltPhysics) JobSystemThreadPool
// rewritten to use pthread to allow changing the stack size

#include <Jolt/Jolt.h>

#include "JobSystemPThread.hpp"
#include <Jolt/Core/FPException.h>

#include <pthread.h>
#include <thread>

#ifdef JPH_PLATFORM_LINUX
	#include <sys/prctl.h>
#endif

using namespace JPH;
using std::thread;

static void* ptthread_main(void* ptthread) {
    PTThread* pt = (PTThread*)ptthread;
    pt->jobSystem->ThreadMain(pt->index);
    return NULL;
}

PTThread::PTThread(int id, JobSystemPThread* system) {
    index = id;
    jobSystem = system;


    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 1024 * 1024); // 1MB stack

    pthread_create(&handle, &attr, ptthread_main, (void*)this);

    pthread_attr_destroy(&attr);
}

PTThread::~PTThread() {
    if (handle != 0) {
        pthread_cancel(handle);
    }
}

bool PTThread::joinable() {
    return handle != 0;
}

void PTThread::join() {
    void* ret;
    pthread_join(handle, &ret);
    handle = 0;
}

void JobSystemPThread::Init(uint inMaxJobs, uint inMaxBarriers, int inNumThreads)
{
	JobSystemWithBarrier::Init(inMaxBarriers);
	mJobs.Init(inMaxJobs, inMaxJobs);
	for (atomic<Job *> &j : mQueue)
		j = nullptr;
	StartThreads(inNumThreads);
}

JobSystemPThread::JobSystemPThread(uint inMaxJobs, uint inMaxBarriers, int inNumThreads)
{
	Init(inMaxJobs, inMaxBarriers, inNumThreads);
}

void JobSystemPThread::StartThreads([[maybe_unused]] int inNumThreads)
{
	if (inNumThreads < 0)
		inNumThreads = thread::hardware_concurrency() - 1;

	if (inNumThreads == 0)
		return;

	mQuit = false;

	mHeads = reinterpret_cast<atomic<uint> *>(Allocate(sizeof(atomic<uint>) * inNumThreads));
	for (int i = 0; i < inNumThreads; ++i)
		mHeads[i] = 0;

	JPH_ASSERT(mThreads.empty());
	mThreads.reserve(inNumThreads);
	for (int i = 0; i < inNumThreads; ++i) {
		mThreads.emplace_back(i, this);
    }
}

JobSystemPThread::~JobSystemPThread()
{
	StopThreads();
}

void JobSystemPThread::StopThreads()
{
	if (mThreads.empty())
		return;

	mQuit = true;
	mSemaphore.Release((uint)mThreads.size());

	for (PTThread &t : mThreads)
		if (t.joinable())
			t.join();

	mThreads.clear();

	for (uint head = 0; head != mTail; ++head)
	{
		Job *job_ptr = mQueue[head & (cQueueLength - 1)].exchange(nullptr);
		if (job_ptr != nullptr)
		{
			job_ptr->Execute();
			job_ptr->Release();
		}
	}

	Free(mHeads);
	mHeads = nullptr;
	mTail = 0;
}

JobHandle JobSystemPThread::CreateJob(const char *inJobName, ColorArg inColor, const JobFunction &inJobFunction, uint32 inNumDependencies)
{
	uint32 index;
	for (;;)
	{
		index = mJobs.ConstructObject(inJobName, inColor, this, inJobFunction, inNumDependencies);
		if (index != AvailableJobs::cInvalidObjectIndex)
			break;
		JPH_ASSERT(false, "No jobs available!");
		std::this_thread::sleep_for(std::chrono::microseconds(100));
	}
	Job *job = &mJobs.Get(index);

	JobHandle handle(job);

	if (inNumDependencies == 0)
		QueueJob(job);

	return handle;
}

void JobSystemPThread::FreeJob(Job *inJob)
{
	mJobs.DestructObject(inJob);
}

uint JobSystemPThread::GetHead() const
{
	uint head = mTail;
	for (size_t i = 0; i < mThreads.size(); ++i)
		head = min(head, mHeads[i].load());
	return head;
}

void JobSystemPThread::QueueJobInternal(Job *inJob)
{
	inJob->AddRef();

	uint head = GetHead();

	for (;;)
	{
		uint old_value = mTail;
		if (old_value - head >= cQueueLength)
		{
			head = GetHead();
			old_value = mTail;

			if (old_value - head >= cQueueLength)
			{
				mSemaphore.Release((uint)mThreads.size());

				std::this_thread::sleep_for(std::chrono::microseconds(100));
				continue;
			}
		}

		Job *expected_job = nullptr;
		bool success = mQueue[old_value & (cQueueLength - 1)].compare_exchange_strong(expected_job, inJob);

		mTail.compare_exchange_strong(old_value, old_value + 1);

		if (success)
			break;
	}
}

void JobSystemPThread::QueueJob(Job *inJob)
{
	if (mThreads.empty())
		return;

	QueueJobInternal(inJob);

	mSemaphore.Release();
}

void JobSystemPThread::QueueJobs(Job **inJobs, uint inNumJobs)
{
	JPH_ASSERT(inNumJobs > 0);

	if (mThreads.empty())
		return;

	for (Job **job = inJobs, **job_end = inJobs + inNumJobs; job < job_end; ++job)
		QueueJobInternal(*job);

	mSemaphore.Release(min(inNumJobs, (uint)mThreads.size()));
}

#if defined(JPH_PLATFORM_LINUX)
static void SetThreadName(const char *inName)
{
    JPH_ASSERT(strlen(inName) < 16);
    prctl(PR_SET_NAME, inName, 0, 0, 0);
}
#endif // JPH_PLATFORM_LINUX

void JobSystemPThread::ThreadMain(int inThreadIndex)
{
	char name[64];
	snprintf(name, sizeof(name), "Worker %d", int(inThreadIndex + 1));

#if defined(JPH_PLATFORM_LINUX)
	SetThreadName(name);
#endif // JPH_PLATFORM_LINUX

	FPExceptionsEnable enable_exceptions;
	JPH_UNUSED(enable_exceptions);

	mThreadInitFunction(inThreadIndex);

	atomic<uint> &head = mHeads[inThreadIndex];

	while (!mQuit)
	{
		mSemaphore.Acquire();

		{
			while (head != mTail)
			{
				atomic<Job *> &job = mQueue[head & (cQueueLength - 1)];
				if (job.load() != nullptr)
				{
					Job *job_ptr = job.exchange(nullptr);
					if (job_ptr != nullptr)
					{
						job_ptr->Execute();
						job_ptr->Release();
					}
				}
				head++;
			}
		}
	}

	mThreadExitFunction(inThreadIndex);
}