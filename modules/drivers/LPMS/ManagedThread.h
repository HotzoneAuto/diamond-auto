//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZenRos driver, under the MIT License.
// See the LICENSE file in the top-most folder for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#ifndef MANAGED_THREAD_H
#define MANAGED_THREAD_H

#include <functional>
#include <thread>
#include <atomic>

template <class TThreadParameter>
class ManagedThread {
public:

	typedef std::function<bool(TThreadParameter)> WorkerFunction;

    ManagedThread() {}

	ManagedThread(WorkerFunction func) : m_func(func) {

	}

    ~ManagedThread() {
        stop();
    }

	bool start(TThreadParameter threadParams) {
		if (m_workerThread.joinable()) {
			// cannot start while thread is running
			return false;
		}

		m_terminate = false;

		auto & terminateLocal = m_terminate;
		WorkerFunction workerLocalCopy = m_func;
		// capture by copy so we are independent of the lifetime of the 
		// thread parameters
		const auto lmdWrapperManager = [threadParams, 
										&terminateLocal, 
										workerLocalCopy](){
			while (! terminateLocal.load()) {
                // exit if the thread worker code requested it
                if (!workerLocalCopy(threadParams))
                    return;
			}
		};

		m_workerThread = std::thread(lmdWrapperManager);
		return true;
	}

	void stopAsync() {
		m_terminate = true;
		// don't join explicitly
	}

    /**
    \param waitForThreadTerminate if true, the thread loop will not
           terminate after the next execution round but the class
           will wait until the worker lambda returned false.
    */
    void stop(bool waitForThreadTerminate = false) {
		m_terminate = !waitForThreadTerminate;
		// join the worker thread, so we can be sure that 
		// it terminated
		if (m_workerThread.joinable()) {
			m_workerThread.join();
		}
	}

    bool isRunning() const {
        return m_workerThread.joinable();
    }

private:
	WorkerFunction m_func;
	std::thread m_workerThread;
	std::atomic<bool> m_terminate;
};

#endif