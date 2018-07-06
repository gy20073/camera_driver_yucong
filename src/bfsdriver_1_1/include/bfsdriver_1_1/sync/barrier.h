#include <cstdlib>
#include <mutex>
#include <condition_variable>
#include <iostream>

using namespace std;

class Barrier {
public:
	explicit Barrier(std::size_t count):
		nThreads(count),
		curThreads(count),
		mGeneration(0)
	{}

	void wait(int id) {

		if (nThreads == 0) {
			cout << "[Error]: You must set thread counts before using this barrier." << endl;
			return;
		}
		printf("id: %d, Before lock generation: %lu, thread: %lu\n", id, mGeneration, curThreads);
		std::unique_lock<std::mutex> notify_lock(notify_mutex, std::defer_lock);
		std::unique_lock<std::mutex> count_lock(count_mutex, std::defer_lock);

		notify_lock.lock();
		count_lock.lock();
		curThreads--; // Arrived.
		printf("id: %d, After lock generation: %lu, thread: %lu\n", id, mGeneration, curThreads);
		

		auto gen = mGeneration; /* Fetch generation. */
		count_lock.unlock();

		/* If this is the last arriver */
		if (curThreads == 0) {
			mGeneration++; // Next generation
			curThreads = nThreads; // Reset cur thread count.
			cond.notify_all(); // Let everybody start.
		} else {
			/* if not the last arriver, loop on generation number. */
			while (gen == mGeneration) {
				cond.wait(notify_lock);
			}
		}
		notify_lock.unlock();
	}

	void setThreadCount(std::size_t count) {
		nThreads = count;
		curThreads = count;
	}

	/* Quick function for barrier to check whether it is ready. */
	int isReady() {
		std::unique_lock<std::mutex> count_lock(count_mutex);
		if (curThreads == 1) {
			return 1;
		}
		return 0;
	}

private:
	std::size_t nThreads;
	std::size_t curThreads;
	std::size_t mGeneration;
	std::mutex count_mutex; // 
	std::mutex notify_mutex;
	std::condition_variable cond;
};

