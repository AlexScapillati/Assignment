
#pragma once


#include "thread_queue.h"
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>


class thread_pool
{
private:
	class IThreadTask
	{
	public:
		IThreadTask(void) = default;
		virtual ~IThreadTask(void) = default;
		IThreadTask(const IThreadTask& rhs) = delete;
		IThreadTask& operator=(const IThreadTask& rhs) = delete;
		IThreadTask(IThreadTask&& other) = default;
		IThreadTask& operator=(IThreadTask&& other) = default;

		/**
		 * Run the task.
		 */
		virtual void execute() = 0;
	};

	template <typename Func>
	class ThreadTask : public IThreadTask
	{
	public:
		ThreadTask(Func&& func)
			:m_func{ std::move(func) }
		{
		}

		~ThreadTask(void) override = default;
		ThreadTask(const ThreadTask& rhs) = delete;
		ThreadTask& operator=(const ThreadTask& rhs) = delete;
		ThreadTask(ThreadTask&& other) = default;
		ThreadTask& operator=(ThreadTask&& other) = default;

		/**
		 * Run the task.
		 */
		void execute() override
		{
			m_func();
		}

	private:
		Func m_func;
	};

public:
	/**
	 * A wrapper around a std::future that adds the behavior of futures returned from std::async.
	 * Specifically, this object will block and wait for execution to finish before going out of scope.
	 */
	template <typename T>
	class TaskFuture
	{
	public:
		TaskFuture(std::future<T>&& future)
			:m_future{ std::move(future) }
		{
		}

		TaskFuture(const TaskFuture& rhs) = delete;
		TaskFuture& operator=(const TaskFuture& rhs) = delete;
		TaskFuture(TaskFuture&& other) = default;
		TaskFuture& operator=(TaskFuture&& other) = default;
		~TaskFuture(void)
		{
			if (m_future.valid())
			{
				m_future.get();
			}
		}

		auto get(void)
		{
			return m_future.get();
		}


	private:
		std::future<T> m_future;
	};

public:
	/**
	 * Constructor.
	 */
	thread_pool(void)
		:thread_pool{ (std::max)(std::thread::hardware_concurrency(), 2u) - 1u }
	{
		/*
		 * Always create at least one thread.  If hardware_concurrency() returns 0,
		 * subtracting one would turn it to UINT_MAX, so get the maximum of
		 * hardware_concurrency() and 2 before subtracting 1.
		 */
	}

	/**
	 * Constructor.
	 */
	explicit thread_pool(const std::uint32_t numThreads)
		:m_done{ false },
		m_workQueue{},
		m_threads{}
	{
		try
		{
			for (std::uint32_t i = 0u; i < numThreads; ++i)
			{
				m_threads.emplace_back(&thread_pool::worker, this);
			}
		}
		catch (...)
		{
			destroy();
			throw;
		}
	}

	/**
	 * Non-copyable.
	 */
	thread_pool(const thread_pool& rhs) = delete;

	/**
	 * Non-assignable.
	 */
	thread_pool& operator=(const thread_pool& rhs) = delete;

	/**
	 * Destructor.
	 */
	~thread_pool(void)
	{
		destroy();
	}

	/**
	 * Submit a job to be run by the thread pool.
	 */
	template <typename Func, typename... Args>
	auto submit(Func&& func, Args&&... args)
	{
		auto boundTask = std::bind(std::forward<Func>(func), std::forward<Args>(args)...);
		using ResultType = std::result_of_t<decltype(boundTask)()>;
		using PackagedTask = std::packaged_task<ResultType()>;
		using TaskType = ThreadTask<PackagedTask>;

		PackagedTask task{ std::move(boundTask) };
		TaskFuture<ResultType> result{ task.get_future() };
		m_workQueue.push(std::make_unique<TaskType>(std::move(task)));
		return result;
	}
	


private:
	/**
	 * Constantly running function each thread uses to acquire work items from the queue.
	 */
	void worker(void)
	{
		while (!m_done)
		{
			std::unique_ptr<IThreadTask> pTask{ nullptr };
			if (m_workQueue.waitPop(pTask))
			{
				pTask->execute();
			}
		}
	}

	/**
	 * Invalidates the queue and joins all running threads.
	 */
	void destroy(void)
	{
		m_done = true;
		m_workQueue.invalidate();
		for (auto& thread : m_threads)
		{
			if (thread.joinable())
			{
				thread.join();
			}
		}
	}

public:

	size_t n_threads() const { return m_threads.size(); }

	size_t n_threads_available() const { return n_threads() - m_workQueue.count(); }

	bool done() const { return m_workQueue.count() == 0; }

private:
	std::atomic_bool m_done;
	thread_queue<std::unique_ptr<IThreadTask>> m_workQueue;
	std::vector<std::thread> m_threads;
};
