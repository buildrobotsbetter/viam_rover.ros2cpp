#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>


class Watchdog
{
public:
    Watchdog();
    ~Watchdog() = default;

    /// @brief Register a callback to be executed when the watchdog expires
    /// @param callback Function to be called
    /// The watchdog does not automatically stop during execution of the callback
    /// The watchdog that triggered the callback is provided as an argument
    /// The callback must be registered before the watchdog is started
    void registerCallback(std::function<void(Watchdog&)> callback);

    /// @brief Set the timeout interval of the watchdog
    /// @param value Interval in milliseconds
    /// Has no effect if watchdog is running
    void setTimeoutInterval_ms(uint32_t value);

    /// @brief Start the watchdog as timed out.
    /// Start the watchdog as timed out. By default timedOut() will return false until
    /// the watchdog times out. Setting this to true will have timedOut() be true when
    /// start() is called until reset() is called. It will also cause the callback to be
    /// executed when start() is called.
    void setTimedOutOnStart(bool value);

    /// @brief Starts the watchdog using the timeout interval
    /// Using this method on a running watchdog has no effect
    /// An interval value of 0 will cause the watchdog to never timeout
    /// or to always report timed out depending on setTimedOutOnStart
    void start();

    /// @brief Cancel the watchdog
    /// Using this method on a stopped watchdog has no effect
    void cancel();

    /// @brief Reset the watchdog
    /// Resets the watchdog such that it does not timeout. If the watchdog is not reset
    /// within the timeout interval, the watched dog will move to the timeout state
    /// and the timeout function will be executed.
    void reset();

    /// @brief Specifies if the watchdog is running.
    /// Specifies if the watchdog is running. This is independent of the timedOut status and
    /// will return false before the start method is called and after the cancel method is called.
    bool isRunning() const;

    /// @brief Specifies if the watchdog has timed out.
    /// Specifies if the watchdog has timed out. This is set to true when the reset interval
    /// has not been called within the timeout interval. If timedOut is true, reset must
    /// be called at (or faster than) the drive timeout interval for the length of time specified
    /// by the recovery time interval before the timeout will be cleared.
    bool timedOut() const;

private:
    enum class WatchdogStateEnum { NOT_STARTED, NORMAL, TIMEDOUT };

    void watchdogThread();

    mutable std::mutex m_mutex;
    WatchdogStateEnum m_state;
    bool m_timedOutOnStart;

    std::function<void(Watchdog&)> m_callback;
    std::condition_variable m_condition;
    std::thread m_thread;
    uint32_t m_interval_ms;
};
