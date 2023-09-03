#include <iostream>
#include <thread> 

class TimeMeter
{
    using TimeType = std::chrono::time_point<std::chrono::system_clock>;
public:
    enum class Unit : int64_t
    {
        us = 1,
        ms = 1000,
        s = ms * 1000,
    };

    // 参数 auto_split 用来指定是否要在对象被析构时输出经过的时间
    TimeMeter(bool auto_split = false)
        : auto_split_(auto_split)
    {
        start_ = std::chrono::system_clock::now();
    }

    ~TimeMeter()
    {
        if (auto_split_) { Split("- "); }
    }

    // 重置开始时间
    void Reset()
    {
        start_ = std::chrono::system_clock::now();
    }

    // 获取经过的时间，参数 unit 用来指定获取的时间的单位
    int64_t GetElapse(Unit unit = Unit::us)
    {
        using namespace std::chrono;
        auto now = system_clock::now();
        auto us = duration_cast<std::chrono::microseconds>(now - start_).count();
        return us / static_cast<int64_t>(unit);
    }

    // 打印距离对象构造或调用 Reset() 后经过的时间
    void Split(const char* msg = "")
    {
        auto us = GetElapse();
        
    }

private:
    bool auto_split_{};
    TimeType start_;
}; 


template <int64_t hz, bool auto_sleep = false>
class RateLoop
{
public:
    RateLoop() : timer() {}

    ~RateLoop()
    {
        if (auto_sleep) { Sleep(); }
    }

    bool Sleep()
    {
        // 获取经过的时间
        int64_t elapsed = timer.GetElapse();
        constexpr int64_t once_us = 1000000 / hz;
        // 计算本轮循环剩余的时间
        int64_t remain_us = once_us - elapsed;
        if (remain_us >= 0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(remain_us));
        }
        else {
            std::cerr << "执行超时\n";
        }
        // 重置计算器
        timer.Reset();
        return remain_us >= 0;
    }

private:
    TimeMeter timer;
};


