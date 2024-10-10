#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>

#define NUM_ITERATIONS 1000000

int counter = 0;
pthread_mutex_t lock;

void* increment_counter(void* arg) {
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        pthread_mutex_lock(&lock);
        counter++;
        pthread_mutex_unlock(&lock);
    }
    return NULL;
}

int main() {
    pthread_t thread1, thread2;

    // Initialize the mutex
    pthread_mutex_init(&lock, NULL);

    // Create two threads that will run the increment_counter function
    pthread_create(&thread1, NULL, increment_counter, NULL);
    pthread_create(&thread2, NULL, increment_counter, NULL);

    // Wait for both threads to finish
    pthread_join(thread1, NULL);
    pthread_join(thread2, NULL);

    // Destroy the mutex
    pthread_mutex_destroy(&lock);

    // Print the final value of the counter
    printf("Final counter value: %d\n", counter);

    return 0;
}



// #include <atomic>
// #include <thread>
// #include <vector>
//
// std::atomic_int acnt;
//
// void f()
// {
//     for (int n = 0; n < 10; ++n)
//     {
//         ++acnt;
//         // Note: for this example, relaxed memory order
//         // is sufficient, e.g. acnt.fetch_add(1, std::memory_order_relaxed);
//     }
// }
//
// int main()
// {
//     {
//         std::vector<std::jthread> pool;
//         for (int n = 0; n < 5; ++n)
//             pool.emplace_back(f);
//     }
// }

