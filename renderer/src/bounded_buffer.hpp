#pragma once
#include <condition_variable>
#include <mutex>
#include <thread>
#include <boost/circular_buffer.hpp>

template <class T>
class bounded_buffer
{
  public:
    typedef boost::circular_buffer<T> container_type;
    typedef typename container_type::size_type size_type;
    typedef typename container_type::value_type value_type;

    explicit bounded_buffer(size_type capacity)
        : m_unread(0)
        , m_container(capacity)
    {}
    bounded_buffer(const bounded_buffer &) = delete;
    bounded_buffer &operator=(const bounded_buffer) = delete;

    void push_front(T &&item)
    {
        {
            std::unique_lock<std::mutex> lk{mtx_};
            not_full_.wait(lk, std::bind(&bounded_buffer<T>::is_not_full, this));
            m_container.push_front(std::forward<T>(item));
            ++m_unread;
        }
        not_empty_.notify_one();
    }

    void pop_back(value_type *pItem)
    {
        {
            std::unique_lock<std::mutex> lk{mtx_};
            not_empty_.wait(lk, std::bind(&bounded_buffer<T>::is_not_empty, this));
            *pItem = m_container[--m_unread];
        }
        not_full_.notify_one();
    }

    bool is_not_empty() const
    {
        return m_unread > 0;
    }
    bool is_not_full() const
    {
        return m_unread < m_container.capacity();
    }

    size_type m_unread;
    container_type m_container;
    std::mutex mtx_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
};
