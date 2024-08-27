#include <utilities/Watchdog.hpp>

Watchdog::Watchdog() : 
    m_state(WatchdogStateEnum::NOT_STARTED),
    m_interval_ms(0)
{
    // Intentionally left blank
}

void Watchdog::registerCallback(std::function<void(Watchdog&)> callback)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_state != WatchdogStateEnum::NOT_STARTED)
    {
        return;
    }

    m_callback = callback;
}

void Watchdog::setTimeoutInterval_ms(uint32_t value)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_state != WatchdogStateEnum::NOT_STARTED)
    {
        return;
    }

    m_interval_ms = value;
}

void Watchdog::setTimedOutOnStart(bool value)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_state != WatchdogStateEnum::NOT_STARTED)
    {
        return;
    }

    m_timedOutOnStart = value;
}

void Watchdog::start()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_state == WatchdogStateEnum::NOT_STARTED)
    {
        if (m_timedOutOnStart)
        {
            m_state = WatchdogStateEnum::TIMEDOUT;
            if (m_callback)
            {
                m_callback(*this);
            }
        }
        else
        {
            m_state = WatchdogStateEnum::NORMAL;
        }

        if (m_interval_ms == 0)
        {
            return;
        }

        m_thread = std::thread(&Watchdog::watchdogThread, this);
    }
}

void Watchdog::cancel()
{
    m_mutex.lock();
    if (m_state != WatchdogStateEnum::NOT_STARTED)
    {
        m_state = WatchdogStateEnum::NOT_STARTED;
        m_condition.notify_all();
    }
    m_mutex.unlock();

    if (m_thread.joinable())
    {
        m_thread.join();
    }
}

void Watchdog::reset()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_state != WatchdogStateEnum::NOT_STARTED)
    {
        m_state = WatchdogStateEnum::NORMAL;
        m_condition.notify_all();
    }
}

bool Watchdog::isRunning() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state != WatchdogStateEnum::NOT_STARTED;
}

bool Watchdog::timedOut() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state == WatchdogStateEnum::TIMEDOUT;
}

void Watchdog::watchdogThread()
{
    while (isRunning())
    {
        std::unique_lock<std::mutex> lock(m_mutex);

        auto result = m_condition.wait_for(lock, std::chrono::milliseconds(m_interval_ms));
        if (result == std::cv_status::timeout)
        {
            if (m_state == WatchdogStateEnum::NORMAL)
            {
                m_state = WatchdogStateEnum::TIMEDOUT;
                if (m_callback)
                {
                    m_callback(*this);
                }
            }
        }
    }
}