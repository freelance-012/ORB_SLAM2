#pragma once
#include <memory>
#include <map>
#include <chrono>
#include <mutex>
#include <iostream>
#include <vector>
#include <sstream>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <thread>
#include <algorithm>

namespace ORB_SLAM2
{

class Profiler
{
private:
    using Clock = std::chrono::high_resolution_clock;
    using Duration = std::chrono::duration<float, std::milli>;
    
public:

    static void Print(const std::string& msg, const bool display)
    {
        if(!display) {
            return ;
        }

        std::lock_guard<std::mutex> lock(Profiler::GetInstance().mutex_print_);
        std::cout << "[" << std::this_thread::get_id() << "]: " << msg;
    }

    static void SetTimestamp(const double ts)
    {
        std::lock_guard<std::mutex> lock(Profiler::GetInstance().mutex_save_);
        Profiler::GetInstance().timestamp = ts;
    }

    static void Start(const std::string& name)
    {
        Profiler::GetInstance().start(std::this_thread::get_id(), name);
    }

    static Duration Stop(const std::string& name)
    {
        return Profiler::GetInstance().stop(name);
    }

    static void Display(const std::string& name = "")
    {
        if(!name.empty())
        {
            std::vector<std::string> vname = {name};
            Print(Profiler::GetInstance().displayTimeLogs(vname), true);
        }
        else
        {
            Print(Profiler::GetInstance().displayTimeLogs(), true);
        }
    }

    static float StopAndDisplay(const std::string& name, const bool display)
    {
        Duration t = Stop(name);
        Profiler::GetInstance().save_time(name, t);

        std::stringstream ss;
        ss << "ts: " << Profiler::GetInstance().timestamp << "  " << name << ": " << t.count() << " ms." << std::endl;
        Profiler::Print(ss.str(), display);

        return t.count();
    }

    static void PrintThreadTiming(const std::string &thread_name="", const bool &clear = true)
    {
        auto thread_id = std::this_thread::get_id();
        Profiler::GetInstance().stopThread(thread_id);
        Profiler::GetInstance().printTime(thread_id, thread_name);

        if (clear)
        {
            Profiler::GetInstance().clearThread(thread_id);
        }
    }

    static void ClearThreadTiming()
    {
        auto thread_id = std::this_thread::get_id();
        Profiler::GetInstance().clearThread(thread_id);
    }

private:
    Profiler() = default;
    ~Profiler() = default;
    Profiler(const Profiler& ) = delete;
    Profiler& operator=(const Profiler&) = delete;

    static Profiler& GetInstance()
    {
        static Profiler profiler;
        return profiler;
    }

    void start(const std::thread::id &thread_id, const std::string& name)
    {
        std::lock_guard<std::mutex> lock(mutex_msg_);
        start_map_[name] = Clock::now();

        if (thread_id_time_map.count(thread_id) == 0)
        {
            thread_id_time_map[thread_id] = {name};
        }
        else
        {
            auto &time_names = thread_id_time_map[thread_id];
            auto it = std::find(time_names.begin(), time_names.end(), name);
            if (it != time_names.end())
            {
                time_names.erase(it);
            }
            time_names.push_back(name);
        }
    }

    Duration stop(const std::string& name)
    {
        Clock::time_point chrono_end = Clock::now();

        std::lock_guard<std::mutex> lock(mutex_msg_);
        
        auto stime_iter = start_map_.find(name);
        if(stime_iter == start_map_.end())
        {
            return Duration();
        }
        
        Clock::time_point chrono_begin = stime_iter->second;
        start_map_.erase(stime_iter);

        Duration duration = std::chrono::duration_cast<Duration>(chrono_end - chrono_begin);

        time_map_[name] = duration;
        history_timing_map_[name].update(duration.count());

        return duration;
    }


    std::string displayTimeLogs(const std::vector<std::string> & name_list = std::vector<std::string>()) const
    {
        std::stringstream ss;
        ss << "\n****************************************************************";
        ss << "\n   Time Logs Summary (average time \u00B1 std [min , max]  [ms])\n";

        if(name_list.empty()) {
            for(const auto& el : history_timing_map_) {
                ss << std::endl << ">>> " << el.first << " : " << el.second.show();
            }
        } else {
            for(const std::string &name : name_list) {
                auto it = history_timing_map_.find(name);
                if( it != history_timing_map_.end()) {
                    auto & state = it->second;
                    ss << std::endl << ">>> " << name << " : " << state.show() ;
                } else {
                    ss << std::endl << ">>>" << "timer " << name
                       << "  not found";
                }
            }
        }

        ss << "\n***********************************\n";
        return ss.str();
    }

    void save_time(const std::string& name, const Duration& t)
    {
        if(name[0] > '0') {
            return;
        }
        std::lock_guard<std::mutex> lock(Profiler::GetInstance().mutex_save_);
        static std::ofstream f("sf_vloc_cost.txt");
        
        static bool first_line = false;
        if(!first_line)
        {
           f << "timestamp\tletter\tvalue\n";
           first_line = true;
        }
            

        f << std::fixed;
        f << std::to_string(timestamp) << "\t" << name << "\t" << std::setprecision(4) << t.count() << "\n";
    }

    void stopThread(const std::thread::id &thread_id)
    {
        std::vector<std::string> time_names;

        {
            std::lock_guard<std::mutex> lock(mutex_msg_);
            if (thread_id_time_map.count(thread_id) == 0)
            {
                return;
            }
            time_names = thread_id_time_map[thread_id];
        }

        for (auto name : time_names)
        {
            stop(name);
        }
    }

    void clearThread(const std::thread::id &thread_id)
    {
        std::lock_guard<std::mutex> lock(mutex_msg_);

        if (thread_id_time_map.count(thread_id) == 0)
        {
            return;
        }

        const auto &time_names = thread_id_time_map[thread_id];
        for (auto name : time_names)
        {
            time_map_.erase(name);
        }
        thread_id_time_map.erase(thread_id);
    }

    void printTime(const std::thread::id &thread_id, const std::string &thread_name)
    {
        std::lock_guard<std::mutex> lock(mutex_msg_);

        if (thread_id_time_map.count(thread_id) == 0)
        {
            return;
        }

        std::cout << "[" << thread_id << ":" << thread_name << "] "
                  << "Process time(ms)"
                  << std::endl;

        const auto &time_names = thread_id_time_map[thread_id];
        for (auto name : time_names)
        {
            if (time_map_.count(name))
            {
                printf("\t%9.2f: %s\n", time_map_[name].count(), name.c_str());
            }
        }
    }

private:
    class State {
        size_t n_ = 0;
        float mu_ = 0.0f, s_ = 0.0f, min_= std::numeric_limits<float>::max(), max_ = -std::numeric_limits<float>::max();
        float acc_ = 0.0f;
      public:
        
        void update(float dt) {
            n_++;
            dt += acc_;
            float err =dt - mu_;
            mu_ += err / n_;
            
            s_ += (dt - mu_)*err;
            
            if (dt < min_)
              min_ = dt;
              
            if (dt > max_)
              max_ = dt;
            
            acc_ = 0.0f;
        }
        
        void store(float dt)
        {
          acc_ += dt;
        }
        
        bool have_store()
        {
          return acc_ != 0.0;
        }
        
        float store_value()
        {
          return acc_;
        }
        
        float var() const {
          return (n_<2) ? 0.0: s_/(n_-1);
        }
        
        float std() const {
          return (n_<2) ? 0.0: std::sqrt(s_/(n_-1));
        }
        
        float mean() const {
          return mu_;
        }
        
        float min() const {
          return min_;
        }
        
        float max() const {
          return max_;
        }

        std::string show() const {
            return std::to_string(mu_) + " \u00B1 "
                    + std::to_string( std() ) 
                    + " [" + std::to_string(min_) 
                    + "," + std::to_string(max_) 
                    + "] ms";
        }
    };


private:
    std::mutex mutex_msg_;
    std::mutex mutex_save_;
    std::mutex mutex_print_;

    std::map<std::string, Clock::time_point> start_map_;
    std::map<std::string, Duration> time_map_;
    std::map<std::thread::id, std::vector<std::string>> thread_id_time_map;
    std::map<std::string, State> history_timing_map_;
    std::ofstream f;
    double timestamp;
};

} // namespace ORB_SLAM2
