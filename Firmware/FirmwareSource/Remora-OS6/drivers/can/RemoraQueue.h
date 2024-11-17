#ifndef REMORA_QUEUE_H
#define REMORA_QUEUE_H

#include "mbed.h"


template<typename T, size_t pSize>
class RMPool {
private:
    struct Block {
        T data;
        size_t next;
        bool in_use;

        Block() : next(0), in_use(false) {}
    };

    Block* pool;
    size_t free_head;
    size_t capacity;
    size_t used_count;

public:
    explicit RMPool()
        : capacity(pSize)
        , free_head(0)
        , used_count(0) {

        pool = new Block[capacity];

        // Initialize the free list
        for (size_t i = 0; i < capacity - 1; ++i) {
            pool[i].next = i + 1;
        }
        pool[capacity - 1].next = static_cast<size_t>(-1); // End of list marker
    }

    // Destructor to clean up memory
    ~RMPool() {
        delete[] pool;
    }

    // Prevent copying
    RMPool(const RMPool&) = delete;
    RMPool& operator=(const RMPool&) = delete;

    // Allow moving
    RMPool(RMPool&& other) noexcept
        : pool(other.pool)
        , free_head(other.free_head)
        , capacity(other.capacity)
        , used_count(other.used_count) {
        other.pool = nullptr;
        other.capacity = 0;
        other.free_head = static_cast<size_t>(-1);
        other.used_count = 0;
    }

    RMPool& operator=(RMPool&& other) noexcept {
        if (this != &other) {
            delete[] pool;

            pool = other.pool;
            free_head = other.free_head;
            capacity = other.capacity;
            used_count = other.used_count;

            other.pool = nullptr;
            other.capacity = 0;
            other.free_head = static_cast<size_t>(-1);
            other.used_count = 0;
        }
        return *this;
    }

	bool isFull() const {
        if (free_head == static_cast<size_t>(-1)) {
            return true;
        }
		return false;
	}
    // Get an index to a free block
    size_t allocate() {
        if (free_head == static_cast<size_t>(-1)) {
            grow();
        }

        size_t allocated = free_head;
        free_head = pool[free_head].next;
        pool[allocated].in_use = true;
        ++used_count;
        return allocated;
    }

    // Return a block to the free list
    bool deallocate(size_t index) {
        if (index >= capacity || !pool[index].in_use) {
			return false;
        }

        pool[index].in_use = false;
        pool[index].next = free_head;
        free_head = index;
        --used_count;
		return true;
    }

    // Access the data at a given index
    T& operator[](size_t index) {
		static T default_value{};
        if (index >= capacity || !pool[index].in_use) {
			return default_value;
        }
        return pool[index].data;
    }

    const T& operator[](size_t index) const {
        if (index >= capacity || !pool[index].in_use) {
			return nullptr;
        }
        return pool[index].data;
    }

    // Get current usage statistics
    size_t size() const { return used_count; }
    size_t get_capacity() const { return capacity; }
    bool empty() const { return used_count == 0; }

private:
    void grow() {
        size_t new_size = capacity * 2;
        Block* new_pool = new Block[new_size];

        // Copy existing blocks
        for (size_t i = 0; i < capacity; ++i) {
            new_pool[i] = std::move(pool[i]);
        }

        // Initialize new blocks
        for (size_t i = capacity; i < new_size - 1; ++i) {
            new_pool[i].next = i + 1;
        }
        new_pool[new_size - 1].next = static_cast<size_t>(-1);

        // Clean up old pool and update members
        delete[] pool;
        pool = new_pool;
        free_head = capacity;
        capacity = new_size;
    }
};


template<typename T, size_t qSize>
class RemoraQueue {
private:
    // Memory pool for message allocation
    RMPool<T, qSize> msgPool;

    // Queue structure
    struct Node {
        T data;
        Node* next;
    };

    volatile Node* head;
    volatile Node* tail;
    volatile size_t count;

    // Critical section for thread safety
    CriticalSectionLock lock;

public:

	RemoraQueue() : head(nullptr), tail(nullptr), count(0) {}
	bool try_put(const T& item) {
		CriticalSectionLock cs(lock);

		// Check if pool is full
		if (msgPool.isFull()) {
			return false;
		}

		size_t index = msgPool.allocate();
	    Node* node = reinterpret_cast<Node*>(&msgPool[index]); // Correctly map the pool block to a Node
	    if (node == nullptr) {
    	    return false; // Allocation failed
    	}

		// Initialize node
		node->data = item;
		node->next = nullptr;

		// Insert into queue
		if (tail == nullptr) {
			head = tail = node;
		} else {
			tail->next = node;
			tail = node;
		}

		count++;
		return true;
	}

	bool try_get(T* item) {
		CriticalSectionLock cs(lock);

		if (head == nullptr) {
			return false;
		}

		// Get data from head
		*item = const_cast<Node*>(head)->data;
		// Remove the head node
		Node* old_head = const_cast<Node*>(head); // Access current head
		head = head->next;

		if (head == nullptr) {
			tail = nullptr; // If queue is now empty, reset tail
		}

		// Return the node to the pool
		size_t index = reinterpret_cast<size_t>(old_head) - reinterpret_cast<size_t>(&msgPool[0]);
		msgPool.deallocate(index);

		count--;
		return true;
	}
	bool isEmpty() const {
		CriticalSectionLock cs(lock);
		return head == nullptr;
	}
	bool isFull() const {
		CriticalSectionLock cs(lock);
		return msgPool.isFull();
	}
	size_t available() const {
		CriticalSectionLock cs(lock);
		return qSize - count;
	}
	size_t size() const {
		CriticalSectionLock cs(lock);
		return count;
	}
	// Method to peek at head without removing
	bool peek(T* item) const {
		CriticalSectionLock cs(lock);

		if (head == nullptr) {
			return false;
		}

		*item = const_cast<Node*>(head)->data;
		return true;
	}

	template<typename Func>
	void process_all(Func callback) {
		CriticalSectionLock cs(lock);

		T item;
		while (try_get(&item)) {
			callback(item);
		}
	}


};


#endif // REMORA_QUEUE_H
